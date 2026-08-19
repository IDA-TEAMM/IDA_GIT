#!/usr/bin/env python3
"""GEÇİŞ ÖLÇÜTÜ — ÇOK KOŞULLU SÜPÜRME (§19.4: "A/B tek koşumla YAPILAMAZ").

🔑 YÖNTEM — iki kural birlikte:
  · **tohum İÇİNDE eşleştir**: iki kol AYNI parkurda koşar (adil kıyas)
  · **tohumlar ARASINDA süpür**: hüküm tek koşuma değil DAĞILIMA dayanır
Aranan şey "aynı koşulda tekrarlanabilirlik" değil, **farklı koşullarda
başarı**. Tek tohumda kazanmak kanıt değildir.

Sahne: şartnamenin 8-12 m kapıları (Şekil 3), tohuma göre genişlik + yanal
kaçıklık + yaklaşma açısı değişir. Görev noktası kapı ortasında OLMAYABİLİR
(md 5.5.2.2) — kaçıklık kasıtlı.

Kapalı döngü: MissionManager (nişan) → saf takip kontrolcüsü →
CatamaranDynamics. Ölçüt geçişin sayıldığı yerle AYNI: varış ilan edildiği
andaki bacak yönünde işaretli mesafe; eşik PASS_EK_YOL = 1,53 m.
"""
from __future__ import annotations
import math, random, sys
sys.path.insert(0, "/home/girdap/IDA_GIT/son_kodv2/karar")
import numpy as np
from prototype.dynamics.catamaran import CatamaranDynamics
from prototype.mission.mission_manager import (
    MissionManager, MissionManagerConfig, Waypoint, latlon_to_enu)

PASS_EK_YOL = 1.53
LAT0, LON0 = 40.0, 31.0
M_LAT = 1.0 / 111320.0
M_LON = 1.0 / (111320.0 * math.cos(math.radians(LAT0)))


def _wp_enu(x_dogu: float, y_kuzey: float, parkur: int = 1) -> Waypoint:
    return Waypoint(LAT0 + y_kuzey * M_LAT, LON0 + x_dogu * M_LON, parkur=parkur)


def parkur_uret(tohum: int, n_kapi: int = 6):
    """Tohuma göre DEĞİŞEN parkur: genişlik, kaçıklık, yaklaşma açısı."""
    rng = random.Random(tohum)
    noktalar, y = [], 0.0
    yon = math.radians(rng.uniform(-25.0, 25.0))     # parkurun genel eğimi
    x = 0.0
    for i in range(n_kapi):
        aralik = rng.uniform(9.0, 14.0)
        y += aralik * math.cos(yon)
        x += aralik * math.sin(yon)
        genislik = rng.uniform(8.0, 12.0)            # şartname 8-12 m
        # hakem noktası kapı ortasında OLMAYABİLİR (md 5.5.2.2)
        kacik = rng.uniform(-0.25, 0.25) * genislik / 2.0
        noktalar.append(_wp_enu(x + kacik, y))
    return noktalar


def kosum(tohum: int, gecis_zorunlu: bool, n_kapi: int = 6, adim: float = 0.1,
          maks_s: float = 400.0):
    wps = parkur_uret(tohum, n_kapi)
    mm = MissionManager(wps, MissionManagerConfig(gecis_zorunlu=gecis_zorunlu))
    mm.start()
    dyn = CatamaranDynamics(); par = dyn.p
    durum = np.array([0.0, 0.0, math.radians(90.0), 0.0, 0.0, 0.0])
    t, onceki_idx, sonuc = 0.0, 0, []
    while t < maks_s and mm.current_index < len(wps):
        la = LAT0 + durum[1] * M_LAT
        lo = LON0 + durum[0] * M_LON
        nisan = mm.update(la, lo, t)
        if nisan is None:
            break
        if mm.current_index != onceki_idx:            # VARIŞ ilan edildi
            i = onceki_idx; wp = wps[i]
            e, n = latlon_to_enu(la, lo, wp.lat, wp.lon)
            if i > 0:
                tx, ty = latlon_to_enu(wps[i-1].lat, wps[i-1].lon, wp.lat, wp.lon)
            else:
                tx, ty = 0.0, 1.0
            nn = math.hypot(tx, ty)
            sonuc.append(((-e) * tx + (-n) * ty) / nn if nn > 1e-6 else float("nan"))
            onceki_idx = mm.current_index
        # saf takip: nişana dön + ileri it.
        # `latlon_to_enu` → (doğu, kuzey) = (x, y); durum[2] = psi = atan2(y, x).
        e, n = nisan
        hata = math.atan2(n, e) - durum[2]
        hata = (hata + math.pi) % (2*math.pi) - math.pi
        ileri = par.max_thrust * 0.7 * max(0.0, math.cos(hata))
        donus = par.max_thrust * 0.6 * max(-1.0, min(1.0, hata / (math.pi/3)))
        # 🔴 İŞARET: `catamaran.py` Mz = (T_r − T_l)·s/2 ⇒ SOL dönüş (psi artışı)
        # için T_r > T_l olmalı. Ters yazmak aracı hedeften UZAKLAŞTIRIP
        # cos(hata)≤0'da itkisiz bırakıyordu (ölçüldü: psi 229°'de takılı).
        u = np.array([ileri - donus, ileri + donus])
        durum = dyn.step_rk4(durum, u, adim)
        t += adim
    return sonuc


def main() -> None:
    n_tohum = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    print(f"═══ GEÇİŞ ÖLÇÜTÜ SÜPÜRMESİ · {n_tohum} FARKLI PARKUR · eşik {PASS_EK_YOL} m ═══")
    print(f"{'tohum':>5} {'A kapalı':>22} {'B geçiş zorunlu':>24}")
    print(f"{'':>5} {'geçen/kapı  ortanca':>22} {'geçen/kapı  ortanca':>24}")
    ta = tb = ka = kb = 0
    ma, mb = [], []
    for s in range(n_tohum):
        a = [v for v in kosum(s, False) if not math.isnan(v)]
        b = [v for v in kosum(s, True) if not math.isnan(v)]
        ga = sum(1 for v in a if v > PASS_EK_YOL); gb = sum(1 for v in b if v > PASS_EK_YOL)
        ta += ga; tb += gb; ka += len(a); kb += len(b); ma += a; mb += b
        oa = f"{ga}/{len(a)}  {np.median(a):+6.2f}" if a else "   —"
        ob = f"{gb}/{len(b)}  {np.median(b):+6.2f}" if b else "   —"
        isaret = "✅" if gb > ga else ("=" if gb == ga else "🔴")
        print(f"{s:>5} {oa:>22} {ob:>24}  {isaret}")
    print("─" * 60)
    pa = 100*ta/max(ka,1); pb = 100*tb/max(kb,1)
    print(f"TOPLAM   A: {ta}/{ka} geçti (%{pa:.1f})   B: {tb}/{kb} geçti (%{pb:.1f})")
    if ma and mb:
        print(f"ortanca bacak-yönü mesafesi   A: {np.median(ma):+.2f} m   B: {np.median(mb):+.2f} m")


if __name__ == "__main__":
    main()
