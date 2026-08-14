#!/usr/bin/env python3
"""GİRDAP İDA — `ATC_SPEED_I` / `ATC_SPEED_P` SÜPÜRMESİ (ROS'suz, toplu).

14.08.2026 (§0.99t/§0.99u). Kaptan: *"ATC_SPEED_I için sanal gölde süpürme
yap."*

🔴 **NEDEN AYRI BİR ARAÇ — `sanal_gol.py` BU İŞİ YAPAMAZ.**
Sanal gölün tekne modeli tek satırdır:

    self.u += (cmd_u - self.u) * dt / 0.8

Yani **komut edilen hızı olduğu gibi takip eder.** İçinde ne gaz, ne
`CRUISE_THROTTLE`, ne `ATC_SPEED_*` vardır — uçuş kontrolcüsünün hız çevrimi
hiç modellenmemiştir. Bu yüzden 14.08'de ölçülen **"düşük hız komutunda iki
katı gitme"** hatasını sanal göl **yapısal olarak yakalayamaz** ve orada
`ATC_SPEED_I` süpürmek **anlamsızdır** (parametre döngüde yok).

Bu araç eksik katmanı modeller:

    komut hızı ──► [hedef eğim sınırı ATC_ACCEL_MAX]
                ──► [ileri besleme: CRUISE_THROTTLE × v/CRUISE_SPEED]
                ──► [PI: ATC_SPEED_P, ATC_SPEED_I, IMAX]
                ──► [MOT_THR_MIN/MAX, MOT_SLEWRATE]
                ──► [TEKNE: ivme = a_max·(gaz − (v/v_max)²)]  ← karekök yasası
                ──► gerçekleşen hız

🔑 **ÖNCE DOĞRULA, SONRA SÜPÜR.** Model, 14.08 15:32 su koşumunun **gerçek**
komut/gerçekleşen serisine oturtulur (`--dogrula`); oturmazsa süpürme sonucu
da güvenilmez. Tekne katsayıları (`v_max`, `a_max`) o veriden **fit edilir**,
uydurulmaz.

Kullanım:
    python3 hiz_supurme.py --dogrula seri_guided2.csv
    python3 hiz_supurme.py --supur   seri_guided2.csv
    python3 hiz_supurme.py --supur   seri_guided2.csv --p-de-supur
"""
from __future__ import annotations

import argparse
import csv
import math

# --- teknedeki canlı değerler (14.08.2026 19:15 dökümü) ---------------------
CRUISE_THROTTLE = 0.95      # %95 → kesir
CRUISE_SPEED = 1.05         # m/s
ATC_SPEED_P = 0.2
ATC_SPEED_I = 0.2
ATC_SPEED_IMAX = 1.0
ATC_ACCEL_MAX = 1.0         # m/s²
MOT_THR_MIN = 0.10
MOT_THR_MAX = 1.00
MOT_SLEWRATE = 1.00         # %100/s → kesir/s
DT = 0.02                   # 50 Hz


def cevrim(komutlar, dt_dizi, v_max, a_max, kp, ki, thr_min_yeniden_esle):
    """Uçuş kontrolcüsü hız çevrimi + tekne. Gerçekleşen hız dizisi döner."""
    v = 0.0
    hedef = 0.0
    integ = 0.0
    thr_onceki = 0.0
    cikti = []
    for k, dt in zip(komutlar, dt_dizi):
        # hedef eğim sınırı (ATC_ACCEL_MAX)
        adim = ATC_ACCEL_MAX * dt
        hedef += max(-adim, min(adim, k - hedef))
        # ileri besleme
        thr = (hedef / CRUISE_SPEED) * CRUISE_THROTTLE
        # PI
        hata = hedef - v
        integ += ki * hata * dt
        integ = max(-ATC_SPEED_IMAX, min(ATC_SPEED_IMAX, integ))
        thr += kp * hata + integ
        # çıkış sınırları
        thr = max(-MOT_THR_MAX, min(MOT_THR_MAX, thr))
        egim = MOT_SLEWRATE * dt
        thr = thr_onceki + max(-egim, min(egim, thr - thr_onceki))
        thr_onceki = thr
        # MOT_THR_MIN: ArduPilot ölü bölgeyi telafi eder (isteğe bağlı model)
        if thr_min_yeniden_esle and abs(thr) > 1e-6:
            thr = math.copysign(MOT_THR_MIN + abs(thr) * (1 - MOT_THR_MIN), thr)
        # tekne: itki ∝ gaz (MOT_THST_EXPO=0), sürükleme ∝ v²
        v += a_max * (thr - math.copysign((v / v_max) ** 2, v)) * dt
        v = max(0.0, v)
        cikti.append(v)
    return cikti


def seri_oku(yol):
    t, k, g = [], [], []
    with open(yol) as f:
        for s in csv.DictReader(f):
            t.append(float(s["t"])); k.append(float(s["komut"])); g.append(float(s["gerceklesen"]))
    dt = [max(0.005, min(0.5, t[i] - t[i-1])) for i in range(1, len(t))]
    return t[1:], k[1:], g[1:], dt


def rms(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)) / len(a))


def olcutler(komut, gercek):
    dus = [(k, g) for k, g in zip(komut, gercek) if k < 0.4]
    yuk = [(k, g) for k, g in zip(komut, gercek) if k > 0.9]
    d = sum(g - k for k, g in dus) / len(dus) if dus else float("nan")
    y = sum(g - k for k, g in yuk) / len(yuk) if yuk else float("nan")
    mut = sum(abs(g - k) for k, g in zip(komut, gercek)) / len(komut)
    return d, y, mut


def dogrula(yol):
    t, k, g, dt = seri_oku(yol)
    print(f"gerçek veri: {len(k)} örnek · {t[-1]-t[0]:.0f} s")
    gd, gy, gm = olcutler(k, g)
    print(f"GERÇEK  → düşük(<0,4) açık={gd:+.3f} · yüksek(>0,9) açık={gy:+.3f} "
          f"· ort|hata|={gm:.3f} m/s")
    en_iyi = None
    for yeniden in (False, True):
        for vmax in [x / 100 for x in range(100, 141, 2)]:
            for amax in [x / 100 for x in range(20, 201, 5)]:
                s = cevrim(k, dt, vmax, amax, ATC_SPEED_P, ATC_SPEED_I, yeniden)
                e = rms(s, g)
                if en_iyi is None or e < en_iyi[0]:
                    en_iyi = (e, vmax, amax, yeniden, s)
    e, vmax, amax, yeniden, s = en_iyi
    md, my, mm = olcutler(k, s)
    print(f"\nEN İYİ MODEL: v_max={vmax:.2f} m/s · a_max={amax:.2f} m/s² · "
          f"MOT_THR_MIN yeniden eşleme={'VAR' if yeniden else 'YOK'} · RMS={e:.3f} m/s")
    print(f"MODEL   → düşük açık={md:+.3f} · yüksek açık={my:+.3f} · ort|hata|={mm:.3f}")
    print(f"\nDOĞRULAMA: düşük uç {gd:+.3f} (gerçek) ↔ {md:+.3f} (model) · "
          f"fark {abs(gd-md):.3f} m/s")
    print("→ MODEL " + ("KABUL EDİLEBİLİR" if abs(gd - md) < 0.12
                        else "ZAYIF — süpürme sonucu ihtiyatla okunmalı"))
    return vmax, amax, yeniden


def supur(yol, p_de, vmax, amax, yeniden):
    t, k, g, dt = seri_oku(yol)
    print(f"\n{'='*74}\nSÜPÜRME — v_max={vmax:.2f} a_max={amax:.2f} "
          f"(gerçek veriden fit edildi)\n{'='*74}")
    p_listesi = [0.1, 0.2, 0.4, 0.6, 0.8, 1.0] if p_de else [ATC_SPEED_P]
    print(f"{'P':>5} {'I':>5} | {'düşük açık':>11} {'yüksek açık':>12} "
          f"{'ort|hata|':>10} {'aşım':>7} {'RMS':>7}")
    print("-" * 74)
    sonuc = []
    for kp in p_listesi:
        for ki in [0.0, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 3.0]:
            s = cevrim(k, dt, vmax, amax, kp, ki, yeniden)
            d, y, m = olcutler(k, s)
            asim = max(0.0, max(a - b for a, b in zip(s, k)))
            r = rms(s, k)
            sonuc.append((abs(d) + abs(y) + m, kp, ki, d, y, m, asim, r))
            im = " ←şimdiki" if (kp == ATC_SPEED_P and ki == ATC_SPEED_I) else ""
            print(f"{kp:5.1f} {ki:5.1f} | {d:+11.3f} {y:+12.3f} {m:10.3f} "
                  f"{asim:7.3f} {r:7.3f}{im}")
    sonuc.sort()
    print("\n🏆 EN İYİ ÜÇ (|düşük açık| + |yüksek açık| + ort|hata| toplamına göre):")
    for s in sonuc[:3]:
        print(f"   ATC_SPEED_P={s[1]:.1f} · ATC_SPEED_I={s[2]:.1f} → "
              f"düşük {s[3]:+.3f} · yüksek {s[4]:+.3f} · ort|hata| {s[5]:.3f} m/s")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dogrula", metavar="CSV")
    ap.add_argument("--supur", metavar="CSV")
    ap.add_argument("--p-de-supur", action="store_true")
    a = ap.parse_args()
    yol = a.dogrula or a.supur
    v, am, y = dogrula(yol)
    if a.supur:
        supur(a.supur, a.p_de_supur, v, am, y)
