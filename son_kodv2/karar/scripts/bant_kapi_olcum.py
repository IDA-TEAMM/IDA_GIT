#!/usr/bin/env python3
"""GİRDAP İDA — göl bandından KAPI DAVRANIŞI ölçümü (15.08.2026, §1.13/§1.14 FAZ 0).

🔴 NEDEN VAR: 15.08 göl koşumlarında "kapı arasını geçemedi, dubanın üstüne
sürdü" şikâyeti bant analiziyle köke indirildi (GIRDAP_DURUM §1.13): kenar
hafızası 3 573 kayda şişti, ikiz kayıtlar `_huni_payi`'yi direklerin
%84,8'inde SIFIRLADI (çarpışma koruması söküldü), kilitlerin %13'ü gövdenin
sığmadığı "kapı"lardı. O analiz beş ayrı geçici betikle yapılmıştı — §0.31e
dersi: onay bekleyen iş geçici klasörde bırakılmaz. Bu araç o ölçümü
KALICI ve TEKRARLANABİLİR yapar; her düzeltme fazı (FAZ 1-5) aynı bantla
bu araçtan geçirilip "önce/sonra" tablosu üretilir.

NE ÖLÇER (§1.13'ün altı sayısı):
  ① mod/ARM pencereleri + kat edilen yol + hız (komut ve ölçülen)
  ② kenar hafızası büyümesi (yayımlanan kayıt sayısı: ilk/tepe/son)
  ③ huni payı çökmesi: en yakın komşu (W) dağılımı + pay=0 oranı
  ④ kilitlenen kapıların GERÇEK genişliği (nişanı üreten çift geri bulunur)
     + dar-kapı epizotları (tekne nişana girdi mi)
  ⑤ kapı kilidi kesintileri (>1 sn "kapı YOK") + kapı sayaçları
  ⑥ çarpma sınaması: IMU darbesi + temas-bandı epizotları + ani hız düşüşü
     ⚠ IMU sessizliği "hiç çarpmadı" DEMEZ (0,5 m/s'de yumuşak sürtme
     1,3 g üretmez); yalnız SERT çarpma imzasının yokluğunu söyler.

KULLANIM:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 scripts/bant_kapi_olcum.py <bant_dizini>

⚠ GÜÇ KESİNTİSİNE DAYANIKLI OKUMA: `metadata.yaml` yoksa (fiş çekilmiş bant,
§1.12c) .mcap dosyaları TEK TEK, doğal sırayla okunur; bozuk kuyruk atlanır
ve raporlanır. `bant_onar.py` gerekmez — bu araç salt okurdur, banda yazmaz.

⚠ SALT OKUR — üretim koduna ve banda dokunmaz. Eşikler ölçülmüş tekne
boyutu (gövde 0,785 m) ya da şartname sabiti (duba çapı 0,30 m); serbest
eşik yok (§0.0d).
"""
from __future__ import annotations

import math
import os
import re
import sys
from collections import Counter, defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# --- ölçülmüş/şartname sabitleri (gate_follower ile aynı; §0.0d) ------------
BUOY_R = 0.15                 # şartname: duba çapı 30 cm
HULL_W = 0.785                # ölçülmüş gövde genişliği
MIN_W = HULL_W + 2 * BUOY_R   # 1,085 m — kodun kapı kabul eşiği
TEMAS = BUOY_R + HULL_W / 2   # 0,5425 m — merkez bu kadar yaklaşırsa temas

KONULAR = [
    "/girdap/fusion/pose",
    "/girdap/planning/gate",
    "/girdap/planning/gate_count",
    "/girdap/planning/edge_buoys",
    "/girdap/control/inhibit_reason",
    "/mavros/imu/data",
    "/mavros/state",
    "/mavros/local_position/velocity_body",
    "/mavros/setpoint_velocity/cmd_vel_unstamped",
    "/perception/gate_count",
    "/perception/classified_obstacles",
]


# --- güç kesintisine dayanıklı okuyucu --------------------------------------
def _dogal_sira(p: str) -> int:
    m = re.search(r"_(\d+)\.mcap$", p)
    return int(m.group(1)) if m else 0


def _depolama_kimligi(dizin: str) -> str:
    """Bandın depolama biçimi: saha kayıtları `mcap`, `ros2 bag record`
    varsayılanı `sqlite3`. Yeniden koşum deneylerinin çıktısı ikinci türden
    olduğu için biçim metadata'dan OKUNUR, varsayılmaz."""
    yol = os.path.join(dizin, "metadata.yaml")
    try:
        with open(yol, encoding="utf-8") as f:
            for satir in f:
                if "storage_identifier:" in satir:
                    return satir.split(":", 1)[1].strip()
    except OSError:
        pass
    return "mcap"


def bant_oku(dizin: str) -> dict:
    """Bandı okur; metadata yoksa .mcap'leri doğal sırayla tek tek dener."""
    depolama = _depolama_kimligi(dizin)
    if os.path.exists(os.path.join(dizin, "metadata.yaml")):
        parcalar = [dizin]
    else:
        parcalar = sorted(
            (os.path.join(dizin, f) for f in os.listdir(dizin) if f.endswith(".mcap")),
            key=_dogal_sira,
        )
        print(f"⚠ metadata.yaml yok (güç kesintisi bandı) — {len(parcalar)} parça tek tek okunuyor")
    v: dict = defaultdict(list)
    for p in parcalar:
        r = rosbag2_py.SequentialReader()
        try:
            r.open(rosbag2_py.StorageOptions(uri=p, storage_id=depolama),
                   rosbag2_py.ConverterOptions("", ""))
            tipler = {t.name: t.type for t in r.get_all_topics_and_types()}
            var = [k for k in KONULAR if k in tipler]
            if not var:
                continue
            r.set_filter(rosbag2_py.StorageFilter(topics=var))
            while r.has_next():
                konu, ham, t = r.read_next()
                v[konu].append((t / 1e9, deserialize_message(ham, get_message(tipler[konu]))))
        except Exception as exc:  # noqa: BLE001 — bozuk kuyruk beklenen hâl (§1.12c)
            print(f"⚠ {os.path.basename(p)}: {type(exc).__name__} — parçanın kalanı atlandı")
        finally:
            del r
    for k in v:
        v[k].sort(key=lambda x: x[0])
    return v


def _epizotlar(zamanlar: np.ndarray, bosluk_sn: float) -> list[list[int]]:
    """Ardışık indeksleri, aradaki boşluk `bosluk_sn`'yi aşınca böl."""
    gruplar: list[list[int]] = []
    son = -math.inf
    for i, t in enumerate(zamanlar):
        if t - son > bosluk_sn:
            gruplar.append([])
        gruplar[-1].append(i)
        son = t
    return gruplar


def main() -> None:  # noqa: PLR0915 — tek rapor akışı, bölmek okumayı zorlaştırır
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    dizin = sys.argv[1]
    v = bant_oku(dizin)
    if not any(v.values()):
        sys.exit("bant okunamadı / ilgili konu yok")
    t0 = min(x[0][0] for x in v.values() if x)
    print("=" * 78)
    print("BANT:", dizin.rstrip("/").split("/")[-1])
    for k in KONULAR:
        if v[k]:
            print(f"  {k:46s} {len(v[k]):6d}  {v[k][0][0]-t0:7.1f}s → {v[k][-1][0]-t0:7.1f}s")

    # ① mod + iz + hız --------------------------------------------------------
    print("\n① MOD / İZ / HIZ")
    onceki = None
    for t, m in v["/mavros/state"]:
        a = (m.mode, m.armed)
        if a != onceki:
            print(f"  {t-t0:7.1f}s  {m.mode:12s} armed={m.armed}")
            onceki = a
    poz = np.array([[t - t0, m.pose.position.x, m.pose.position.y]
                    for t, m in v["/girdap/fusion/pose"]]).reshape(-1, 3)
    if len(poz) > 1:
        yol = float(np.hypot(*np.diff(poz[:, 1:3], axis=0).T).sum())
        print(f"  iz: {len(poz)} poz · kat edilen yol {yol:.1f} m")
    hz = np.array([math.hypot(m.twist.linear.x, m.twist.linear.y)
                   for _, m in v["/mavros/local_position/velocity_body"]])
    if len(hz):
        print(f"  ölçülen hız: ortanca {np.median(hz):.3f} · %95 {np.percentile(hz,95):.3f}"
              f" · tepe {hz.max():.3f} m/s")
    cmd = np.array([m.linear.x for _, m in v["/mavros/setpoint_velocity/cmd_vel_unstamped"]])
    if len(cmd):
        sifir = float((np.abs(cmd) < 1e-6).mean())
        print(f"  komut: tepe {np.abs(cmd).max():.3f} m/s · SIFIR komut %{100*sifir:.1f}"
              f" ({len(cmd)} setpoint) — ArduPilot 3 sn komutsuz kalırsa DURUR (resmî belge)")

    # ② hafıza büyümesi -------------------------------------------------------
    print("\n② KENAR HAFIZASI (yayımlanan /girdap/planning/edge_buoys)")
    snaps = [(t - t0, np.array([[p.position.x, p.position.y] for p in m.poses]).reshape(-1, 2))
             for t, m in v["/girdap/planning/edge_buoys"]]
    if snaps:
        boy = [len(s[1]) for s in snaps]
        print(f"  {len(snaps)} anlık görüntü · kayıt: ilk {boy[0]} · TEPE {max(boy)}"
              f" · son {boy[-1]}   (gerçek P1'de 8–16 duba beklenir)")

    # ③ huni payı çökmesi -----------------------------------------------------
    print("\n③ HUNİ PAYI (en yakın komşu W; pay = clamp((W−1,085)/2, 0, tavan))")
    nn_hepsi = []
    for _, S in snaps[::5]:
        if len(S) < 2:
            continue
        D = np.hypot(S[:, 0][:, None] - S[:, 0][None, :],
                     S[:, 1][:, None] - S[:, 1][None, :])
        np.fill_diagonal(D, np.inf)
        nn_hepsi.append(D.min(axis=1))
    if nn_hepsi:
        NN = np.concatenate(nn_hepsi)
        pay0 = float((NN < MIN_W).mean())
        print(f"  {len(NN)} direk örneği · W ortanca {np.median(NN):.3f} m")
        for lo, hi in ((0, 0.30), (0.30, 0.60), (0.60, MIN_W), (MIN_W, 2.0), (2.0, 99.0)):
            k = int(((NN >= lo) & (NN < hi)).sum())
            print(f"     {lo:5.2f}–{hi:5.2f} m: %{100*k/len(NN):4.1f} {'█'*int(50*k/len(NN))}")
        print(f"  🔑 PAY=0 ORANI: %{100*pay0:.1f}  (hedef < %10, §1.14 FAZ 2 ölçütü)")

    # ④ kilitlenen kapı genişliği + dar epizotlar -----------------------------
    print("\n④ KİLİTLENEN KAPI GENİŞLİĞİ (nişanı üreten çift geri bulunur)")
    gate = [(t - t0, m.pose.position.x, m.pose.position.y)
            for t, m in v["/girdap/planning/gate"]]
    si, kayit = 0, []
    for t, gx, gy in gate:
        while si + 1 < len(snaps) and snaps[si + 1][0] <= t:
            si += 1
        S = snaps[si][1] if snaps else np.zeros((0, 2))
        if len(S) < 2:
            continue
        d = np.hypot(S[:, 0] - gx, S[:, 1] - gy)
        Y = S[d < 8.0]
        if len(Y) < 2:
            continue
        i, j = np.triu_indices(len(Y), 1)
        mid = (Y[i] + Y[j]) / 2
        sep = np.hypot(*(Y[i] - Y[j]).T)
        res = np.hypot(mid[:, 0] - gx, mid[:, 1] - gy)
        ok = sep >= MIN_W - 1e-6
        if not ok.any():
            continue
        k = np.where(ok)[0][np.argmin(res[ok])]
        if res[k] > 0.25:      # nişan bu çiftin ortası değil (engel kayması)
            continue
        kayit.append((t, gx, gy, float(sep[k])))
    if kayit:
        W = np.array([r[3] for r in kayit])
        print(f"  eşleşen kilit {len(W)}/{len(gate)} nişan mesajı · genişlik ortanca {np.median(W):.2f} m")
        for lo, hi, ad in ((MIN_W, 1.5, "1,085–1,50 m  GÖVDE SIĞMAZ"),
                           (1.5, 2.5, "1,50–2,50 m   şüpheli"),
                           (2.5, 4.0, "2,50–4,00 m   dar"),
                           (4.0, 99.0, "4,00 m üstü    gerçek P1 kapısı")):
            n = int(((W >= lo) & (W < hi)).sum())
            print(f"     {ad:30s} %{100*n/len(W):4.1f} ({n})")
        # dar epizotlar + tekne nişana girdi mi
        dar = [(t, gx, gy) for t, gx, gy, w in kayit if w < 2.5]
        if dar and len(poz):
            gruplar = _epizotlar(np.array([r[0] for r in dar]), 3.0)
            girilen = 0
            for g in gruplar:
                t_a, t_b = dar[g[0]][0], dar[g[-1]][0]
                gx, gy = dar[g[len(g) // 2]][1], dar[g[len(g) // 2]][2]
                mask = (poz[:, 0] >= t_a) & (poz[:, 0] <= t_b + 25)
                if mask.any():
                    yak = float(np.hypot(poz[mask, 1] - gx, poz[mask, 2] - gy).min())
                    if yak < 1.0:
                        girilen += 1
            print(f"  🔑 DAR (<2,5 m) EPİZOT: {len(gruplar)} · tekne nişana girdi: {girilen}"
                  f"  (hedef 0, §1.14 FAZ 2 ölçütü)")
    else:
        print("  kapı kilidi yok / eşleşme kurulamadı")

    # ⑤ kilit kesintileri + sayaçlar ------------------------------------------
    print("\n⑤ KAPI KİLİDİ SÜREKLİLİĞİ")
    if gate:
        kes = sum(1 for i in range(1, len(gate)) if gate[i][0] - gate[i - 1][0] > 1.0)
        dk = (gate[-1][0] - gate[0][0]) / 60.0
        print(f"  >1 sn kesinti: {kes} / {dk:.0f} dk  (hedef <10/25 dk, §1.14 FAZ 5 ölçütü)")
    for k in ("/girdap/planning/gate_count", "/perception/gate_count"):
        if v[k]:
            print(f"  {k}: " + ", ".join(f"{t-t0:.0f}s→{m.data}" for t, m in v[k]))
    say = Counter(m.data for _, m in v["/girdap/control/inhibit_reason"])
    for s, n in say.most_common(6):
        print(f"  kilit {n:5d}× {s}")

    # ⑥ çarpma sınaması -------------------------------------------------------
    print("\n⑥ ÇARPMA SINAMASI")
    A = np.array([math.sqrt(m.linear_acceleration.x ** 2 + m.linear_acceleration.y ** 2
                            + m.linear_acceleration.z ** 2) for _, m in v["/mavros/imu/data"]])
    if len(A):
        print(f"  IMU {len(A)} örnek · tepe {A.max():.1f} m/s² ({A.max()/9.81:.2f} g)"
              f" · >1,5 g örnek: {int((A > 1.5*9.81).sum())}")
        print("  ⚠ IMU sessizliği yumuşak sürtmeyi DIŞLAMAZ — yalnız sert çarpma imzası ölçülür")
    if len(hz) > 1:
        T3 = np.array([t - t0 for t, _ in v["/mavros/local_position/velocity_body"]])
        d, dt = np.diff(hz), np.diff(T3)
        sert = int(((d < -0.35) & (dt < 0.5) & (hz[:-1] > 0.6)).sum())
        print(f"  ani hız düşüşü (>0,35 m/s): {sert}")
    if snaps and len(poz):
        si, yak = 0, []
        for t, x, y in poz:
            while si + 1 < len(snaps) and snaps[si + 1][0] <= t:
                si += 1
            S = snaps[si][1]
            if len(S):
                yak.append((t, float(np.hypot(S[:, 0] - x, S[:, 1] - y).min())))
        if yak:
            Y = np.array([r[1] for r in yak])
            T2 = np.array([r[0] for r in yak])
            alt = np.where(Y < TEMAS)[0]
            gr = _epizotlar(T2[alt], 2.0) if len(alt) else []
            print(f"  temas bandı (<{TEMAS:.2f} m) epizodu: {len(gr)}"
                  f" · en yakın {Y.min():.2f} m  (hedef 0, §1.14 FAZ 6 ölçütü)")

    _gercek_kapi_genisligi(v)
    _gercek_duba_yerlesimi(v)
    _yon_ile_rota_karsilastir(v)


def _yon_ile_rota_karsilastir(v: dict) -> None:
    """⑩ YÖN (ψ) ↔ YER ROTASI (COG) — pusula ne kadar yalan söylüyor?

    ⑨'un ① hipotezinin BAĞIMSIZ sınaması. Tekne ileri giderken burnunun
    baktığı yön ile fiilen gittiği yön birbirine yakın olmalıdır (deniz
    aracında yengeç kayması birkaç dereceyi geçmez, akıntı yoksa). Aradaki
    fark ONLARCA DERECE ise pusula sapmıştır — ve tespitleri dünyaya taşıyan
    dönüşüm (`planning_node._body_to_world`) tam olarak bu açıyı kullanır.

    🌐 Dış dayanak: manyetometreler en iyi hâlde ±2° verir; elektrikli
    araçlarda motor/ESC paraziti bunu bile bozar. Bizde ölçüldü: `PreArm:
    Check mag field` xy farkı 185, sınır 100. Deniz araçları için literatürün
    önerisi, yol takibinde baş açısı yerine **yer rotası (COG)** kullanmak;
    ArduPilot tarafındaki karşılığı GPS'ten yaw (hareketli taban) ya da
    EKF3'ün GPS hızından yaw kestiren GSF kestiricisidir.

    ⚠ Geri giderken COG ile ψ 180° ayrılır; bu yüzden yalnız İLERİ hareket
    örnekleri alınır (gövde çerçevesinde ileri hız pozitif).
    """
    poz = v.get("/girdap/fusion/pose") or []
    hiz = v.get("/mavros/local_position/velocity_body") or []
    print("\n⑩ YÖN (ψ) ↔ YER ROTASI (COG) — pusula sınaması")
    if len(poz) < 20:
        print("  poz yok — ölçülemedi")
        return
    ht = np.array([t for t, _ in hiz]) if hiz else np.zeros(0)
    hx = np.array([m.twist.linear.x for _, m in hiz]) if hiz else np.zeros(0)

    fark = []
    ADIM = 10                       # ~0,2 s (poz 50 Hz) — gürültüyü yumuşat
    for i in range(0, len(poz) - ADIM, ADIM):
        t0_, m0 = poz[i]
        t1_, m1 = poz[i + ADIM]
        dx = m1.pose.position.x - m0.pose.position.x
        dy = m1.pose.position.y - m0.pose.position.y
        dt = t1_ - t0_
        if dt <= 0:
            continue
        v_yer = math.hypot(dx, dy) / dt
        if v_yer < 0.30:            # duruyor → COG anlamsız
            continue
        if len(ht):
            j = int(np.searchsorted(ht, t1_, side="right") - 1)
            if j >= 0 and hx[j] < 0.1:      # ileri gitmiyor → 180° tuzağı
                continue
        q = m1.pose.orientation
        psi = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cog = math.atan2(dy, dx)
        fark.append(math.degrees((psi - cog + math.pi) % (2 * math.pi) - math.pi))

    if len(fark) < 10:
        print("  yeterli ileri hareket örneği yok")
        return
    F = np.array(fark)
    # Dairesel ortalama (±180 sarmasında aritmetik ortalama yanıltır)
    ort = math.degrees(math.atan2(np.sin(np.radians(F)).mean(),
                                  np.cos(np.radians(F)).mean()))
    print(f"  {len(F)} ileri hareket örneği (yer hızı > 0,30 m/s)")
    print(f"  ψ − COG: dairesel ortalama {ort:+.1f}° · ortanca {np.median(F):+.1f}°"
          f" · %10 {np.percentile(F, 10):+.1f}° · %90 {np.percentile(F, 90):+.1f}°")
    print(f"  |fark| > 30°: %{100*(np.abs(F) > 30).mean():.1f}   "
          f"|fark| > 60°: %{100*(np.abs(F) > 60).mean():.1f}")
    print("  ⚠ sağlıklı pusulada dairesel ortalama birkaç derece olmalı;"
          " onlarca derece = sistematik sapma")


def _gercek_duba_yerlesimi(v: dict, beklenen: int = 4) -> None:
    """⑧ GERÇEK DUBA YERLEŞİMİ — sahada FİİLEN kaç duba vardı, nerede.

    🔑 **Neden gerekli (kaptan, 16.08):** *"bizde 4 duba vardı."* Oysa
    `/girdap/planning/edge_buoys` tepe **421 kayıt** yayınlıyordu ve videoda
    aynı anda 19 kenar dubası görünüyor. Aradaki fark hayalet/ikiz kayıt.
    Bu bölüm ham tespitleri DÜNYA çerçevesinde kümeleyip **en çok gözlenen
    N kümeyi** çıkarır: gerçek duba defalarca aynı yerde görülür, hayalet
    dağılır. Böylece parkurun gerçek geometrisi (kapı açıklığı, kapılar arası
    mesafe) tahminle değil ölçümle bilinir — sanal göl de o geometriye
    kurulabilir.

    ⚠ Poz sürüklenmesi kümeleri yayar (§1.11: 136 m yolda 13 cm); bant kısa
    olduğu için etkisi kümeleme yarıçapının altında kalır. Yine de bu bir
    KÜMELEME sonucudur, yer gerçeği değil — kaptanın ölçtüğü mesafe varsa o
    kazanır.
    """
    kareler = v.get("/perception/classified_obstacles") or []
    pozlar = v.get("/girdap/fusion/pose") or []
    print(f"\n⑧ GERÇEK DUBA YERLEŞİMİ (dünya çerçevesinde kümeleme, en yoğun {beklenen})")
    if not kareler or not pozlar:
        print("  sınıflı algı ya da poz yok — ölçülemedi")
        return

    pt = np.array([t for t, _ in pozlar])
    px = np.array([m.pose.position.x for _, m in pozlar])
    py = np.array([m.pose.position.y for _, m in pozlar])
    ppsi = np.array([
        math.atan2(2.0 * (m.pose.orientation.w * m.pose.orientation.z
                          + m.pose.orientation.x * m.pose.orientation.y),
                   1.0 - 2.0 * (m.pose.orientation.y ** 2 + m.pose.orientation.z ** 2))
        for _, m in pozlar
    ])

    dunya: list[tuple[float, float]] = []
    for t, msg in kareler:
        i = int(np.searchsorted(pt, t, side="right") - 1)
        if i < 0:
            continue
        c, s = math.cos(ppsi[i]), math.sin(ppsi[i])
        for det in msg.detections:
            cls = None
            if det.results:
                try:
                    cls = int(det.results[0].hypothesis.class_id)
                except (TypeError, ValueError):
                    cls = None
            if cls != 0:
                continue
            b = det.bbox.center.position
            dunya.append((px[i] + b.x * c - b.y * s, py[i] + b.x * s + b.y * c))

    if not dunya:
        print("  kenar dubası tespiti yok")
        return

    # Kümeleme yarıçapı: duba çapı + poz/tespit gürültüsü payı = 1 m.
    # Amaç duba AYIRMAK değil, aynı dubanın tekrar tekrar görülmesini
    # TOPLAMAK; gerçek dubalar metrelerce ayrı olduğu için 1 m güvenli.
    YARICAP = 1.0
    D = np.array(dunya)
    ızgara: dict[tuple[int, int], list[int]] = defaultdict(list)
    for idx, (x, y) in enumerate(D):
        ızgara[(int(x // YARICAP), int(y // YARICAP))].append(idx)
    kumeler = []
    for (gx, gy), idxs in ızgara.items():
        komsu = [j for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                 for j in ızgara.get((gx + dx, gy + dy), [])]
        merkez = D[komsu].mean(axis=0)
        kumeler.append((len(komsu), merkez, len(idxs)))
    kumeler.sort(key=lambda k: -k[0])

    secilen: list[tuple[int, np.ndarray]] = []
    for sayi, merkez, _ in kumeler:
        if all(math.dist(merkez, m) > 2.0 * YARICAP for _, m in secilen):
            secilen.append((sayi, merkez))
        if len(secilen) >= beklenen:
            break

    print(f"  {len(D)} kenar tespiti · {len(kumeler)} ızgara hücresi")
    for i, (sayi, m) in enumerate(secilen, 1):
        print(f"     duba {i}: ({m[0]:7.2f}, {m[1]:7.2f})  ·  {sayi:5d} gözlem"
              f"  (%{100*sayi/len(D):.1f})")
    kapsam = sum(s for s, _ in secilen)
    print(f"  🔑 en yoğun {len(secilen)} küme tespitlerin %{100*kapsam/len(D):.1f}'ini"
          f" açıklıyor — kalan %{100*(1-kapsam/len(D)):.1f} dağınık (hayalet/kıyı)")
    if len(secilen) >= 2:
        print("  ikili mesafeler (m):")
        for i in range(len(secilen)):
            for j in range(i + 1, len(secilen)):
                d = math.dist(secilen[i][1], secilen[j][1])
                print(f"     duba {i+1}–{j+1}: {d:6.2f}")
    _hayaletin_kaynagi(kareler, pt, px, py, ppsi, [m for _, m in secilen])
    _gorulme_olasiligi(kareler, pt, px, py, ppsi, [m for _, m in secilen])


def _gorulme_olasiligi(kareler, pt, px, py, ppsi, gercek) -> None:
    """⑪ GÖRÜLME OLASILIĞI ↔ MENZİL — "görülmedi" ne zaman KANIT sayılır?

    🔑 Hafızadan kayıt silmenin tek dürüst ölçütü şudur: cisim, görülmesinin
    KESİN olduğu bir yerdeyken görülmediyse orada değildir. 09.08'de "unutma
    yok" kararı tam da bu koşul konulmadığı için verilmişti (LiDAR 8 m'nin
    ötesinde dubayı zaten göremez; oradaki silme, cismi yok olduğu için değil
    menzil yetmediği için silerdi).

    Bu bölüm o menzili ÖLÇER: gerçek dubaların her biri için, o karede
    1 m yakınında herhangi bir tespit var mıydı? Olasılık menzille birlikte
    nasıl düşüyor? Ölüm kapısının yarıçapı, olasılığın hâlâ yüksek olduğu
    bölgeden seçilir — orada "görülmedi" gerçekten kanıttır.

    ⚠ Sınıf AYRIMI YOK: sınıfsız LiDAR kümesi de tazeleme sayılır (hafıza da
    öyle çalışır — `siniflandir` her iki geçişte `taze` işaretler).
    """
    if not gercek:
        return
    G = np.array(gercek)
    kova = [(0, 3), (3, 5), (5, 8), (8, 12), (12, 18), (18, 25)]
    gorulen = defaultdict(int)
    toplam = defaultdict(int)
    for t, msg in kareler:
        i = int(np.searchsorted(pt, t, side="right") - 1)
        if i < 0:
            continue
        c, s = math.cos(ppsi[i]), math.sin(ppsi[i])
        nokta = []
        for det in msg.detections:
            b = det.bbox.center.position
            nokta.append((px[i] + b.x * c - b.y * s, py[i] + b.x * s + b.y * c))
        P = np.array(nokta).reshape(-1, 2)
        for gx, gy in G:
            menzil = math.hypot(gx - px[i], gy - py[i])
            for k, (a, bb) in enumerate(kova):
                if a <= menzil < bb:
                    toplam[k] += 1
                    if len(P) and float(np.hypot(P[:, 0] - gx, P[:, 1] - gy).min()) <= 1.0:
                        gorulen[k] += 1
                    break

    print("\n⑪ GERÇEK DUBA GÖRÜLME OLASILIĞI ↔ MENZİL (1 m içinde herhangi bir tespit)")
    for k, (a, b) in enumerate(kova):
        if not toplam[k]:
            continue
        p = gorulen[k] / toplam[k]
        print(f"   {a:2d}–{b:2d} m: %{100*p:5.1f}  ({gorulen[k]:6d}/{toplam[k]:6d}) "
              + "█" * int(40 * p))
    print("     👉 ölüm kapısının yarıçapı, olasılığın yüksek kaldığı son kovadan seçilir")


def _hayaletin_kaynagi(kareler, pt, px, py, ppsi, gercek) -> None:
    """⑨ HAYALET NEREDEN GELİYOR — yön hatası mı, sahte tespit mi?

    🔑 **Neden bu ayrım her şeyi belirliyor.** ⑧ ölçtü: kenar tespitlerinin
    yalnız yarısı gerçek dubaların üstünde. Kalanın kaynağı iki farklı arıza
    olabilir ve **düzeltmeleri ortak değil**:

      ① **Yön (pusula/ψ) hatası** — tespit doğru, dünyaya yanlış açıyla
         taşınıyor. İmzası: konum hatası MENZİLLE ORANTILI büyür
         (hata ≈ menzil × Δψ) ve **kerteriz hatasının ortalaması sıfır
         değildir** (sistematik kayma). Düzeltme poz/pusula tarafında.
      ② **Sahte tespit** — orada duba yok. İmzası: hata menzilden bağımsız,
         kerteriz hatası geniş ve ortalaması sıfıra yakın (rastgele).
         Düzeltme algı süzmesinde.

    Ölçüt kerteriz (bearing) hatasıdır, konum hatası değil: konum hatası iki
    arızada da büyür, ama SİSTEMATİK bir açı kayması yalnız ①'de olur.
    """
    if not gercek:
        return
    G = np.array(gercek)
    menzil_kova = [(0, 3), (3, 5), (5, 8), (8, 12), (12, 25)]
    kayit: dict[tuple[int, int], list[tuple[float, float]]] = defaultdict(list)
    for t, msg in kareler:
        i = int(np.searchsorted(pt, t, side="right") - 1)
        if i < 0:
            continue
        c, s = math.cos(ppsi[i]), math.sin(ppsi[i])
        for det in msg.detections:
            cls = None
            if det.results:
                try:
                    cls = int(det.results[0].hypothesis.class_id)
                except (TypeError, ValueError):
                    cls = None
            if cls != 0:
                continue
            b = det.bbox.center.position
            menzil = math.hypot(b.x, b.y)
            if menzil < 0.5:
                continue
            wx, wy = px[i] + b.x * c - b.y * s, py[i] + b.x * s + b.y * c
            # En yakın GERÇEK dubaya olan konum ve kerteriz hatası
            d = np.hypot(G[:, 0] - wx, G[:, 1] - wy)
            j = int(d.argmin())
            konum_hatasi = float(d[j])
            ker_tespit = math.atan2(wy - py[i], wx - px[i])
            ker_gercek = math.atan2(G[j, 1] - py[i], G[j, 0] - px[i])
            ker_hata = math.degrees((ker_tespit - ker_gercek + math.pi)
                                    % (2 * math.pi) - math.pi)
            for k, (a, bb) in enumerate(menzil_kova):
                if a <= menzil < bb:
                    kayit[(k, 0)].append((konum_hatasi, ker_hata))
                    break

    print("\n⑨ HAYALETİN KAYNAĞI (her tespit → en yakın GERÇEK dubaya sapma)")
    print("     menzil     örnek   konum hatası (m)      kerteriz hatası (°)"
          "        <1 m")
    print("                        ortanca    %90      ortalama   ortanca  ±sapma")
    toplam_ker = []
    for k, (a, b) in enumerate(menzil_kova):
        ornek = kayit.get((k, 0)) or []
        if not ornek:
            continue
        H = np.array([o[0] for o in ornek])
        K = np.array([o[1] for o in ornek])
        toplam_ker.extend(K.tolist())
        print(f"   {a:2d}–{b:2d} m  {len(H):7d}   {np.median(H):7.2f}  "
              f"{np.percentile(H, 90):7.2f}   {K.mean():+8.1f}  {np.median(K):+7.1f}"
              f"  {K.std():6.1f}   %{100*(H < 1.0).mean():4.1f}")
    if toplam_ker:
        T = np.array(toplam_ker)
        print(f"  🔑 TÜM KERTERİZ HATASI: ortalama {T.mean():+.1f}° · "
              f"ortanca {np.median(T):+.1f}° · standart sapma {T.std():.1f}°")
        print("     ① yön hatası imzası: ortalama sıfırdan UZAK + konum hatası menzille büyür")
        print("     ② sahte tespit imzası: ortalama sıfıra yakın + sapma geniş + menzilden bağımsız")


def _gercek_kapi_genisligi(v: dict) -> None:
    """⑦ GERÇEK KAPI GENİŞLİĞİ — tek karede birlikte görülen kenar duba çiftleri.

    🔑 **Neden ④ yetmiyor.** ④ kilitlenen kapının genişliğini ölçer; o çift
    KENAR HAFIZASINDAN gelir ve hafıza ikiz/hayalet kayıtlarla şişebilir
    (§1.13: 3 573 kayıt, gerçekte 8-16 duba). Yani ④'ün ortancası kodun ne
    SANDIĞInı ölçer, sahada ne OLDUĞUNU değil.

    Bu bölüm ham `/perception/classified_obstacles` karelerini kullanır.
    Aynı karede görülen iki tespit **iki ayrı fiziksel cisimdir** — hafıza
    duplikasyonu tanım gereği kareler ARASINDA oluşur, kare İÇİNDE değil.
    Dolayısıyla kare içi çift mesafeleri, elimizdeki en temiz gerçek-genişlik
    ölçüsüdür.

    Çiftleme ölçütü kodun kendi geometrisi: `|Δileri| < |Δyanal|`
    (`select_gate`'in dikeylik testi; tespitler base_link'te, x=ileri).
    Yeni bir eşik getirmez — kapı, kursa DİK duran çifttir.
    """
    kareler = v.get("/perception/classified_obstacles") or []
    print("\n⑦ GERÇEK KAPI GENİŞLİĞİ (ham algı karesi; hafıza duplikasyonundan bağımsız)")
    if not kareler:
        print("  /perception/classified_obstacles bantta YOK — ölçülemedi")
        return

    kapi_araligi: list[float] = []      # dikeylik testini geçen çiftler
    tum_ciftler: list[float] = []       # bütün kenar-kenar çiftleri (bağlam)
    kenar_sayisi: list[int] = []
    # ⑦b — GERÇEK KAPI NE KADAR HIZLI DEĞİŞİYOR (kaptan: "aralıklar sudan
    # dolayı sürekli değişiyordu, açısı da"). Ardışık karelerde EN YAKIN kapı
    # çifti eşleştirilir; genişlik/açı/orta nokta değişimi saniyeye bölünür.
    # Bu, kapı izleyicisinin hız sınırını TAHMİNLE değil ÖLÇÜMLE verir:
    # fiziksel sürüklenme yavaştır, hızlı olan her şey yanlış eşleşmedir.
    onceki = None                       # (t, orta, genislik, aci)
    d_genislik: list[float] = []
    d_aci: list[float] = []
    d_orta: list[float] = []
    for t_kare, msg in kareler:
        noktalar = []
        for det in msg.detections:
            cls = None
            if det.results:
                try:
                    cls = int(det.results[0].hypothesis.class_id)
                except (TypeError, ValueError):
                    cls = None
            if cls == 0:                                   # turuncu KENAR dubası
                c = det.bbox.center.position
                noktalar.append((c.x, c.y))
        kenar_sayisi.append(len(noktalar))
        en_yakin = None                                    # (menzil, orta, gen, açı)
        for i in range(len(noktalar)):
            for j in range(i + 1, len(noktalar)):
                dx = noktalar[i][0] - noktalar[j][0]
                dy = noktalar[i][1] - noktalar[j][1]
                d = math.hypot(dx, dy)
                tum_ciftler.append(d)
                if abs(dx) < abs(dy) and d >= MIN_W:       # kursa dik + gövde sığar
                    kapi_araligi.append(d)
                    orta = (0.5 * (noktalar[i][0] + noktalar[j][0]),
                            0.5 * (noktalar[i][1] + noktalar[j][1]))
                    menzil = math.hypot(orta[0], orta[1])  # base_link: araç orijinde
                    aci = math.atan2(dy, dx) % math.pi     # kiriş doğrultusu (±yön yok)
                    if en_yakin is None or menzil < en_yakin[0]:
                        en_yakin = (menzil, orta, d, aci)

        # ⑦b: aynı kapıyı ardışık karelerde izle (yalnız 1 sn'den kısa
        # aralıklar ve orta noktası 1 m'den az kaymış eşleşmeler — daha
        # uzağı zaten "aynı kapı" sayılamaz, oraya bakmak arızayı ölçmek olur)
        if en_yakin is not None:
            if onceki is not None:
                dt = t_kare - onceki[0]
                kayma = math.hypot(en_yakin[1][0] - onceki[1][0],
                                   en_yakin[1][1] - onceki[1][1])
                if 0.0 < dt <= 1.0 and kayma <= 1.0:
                    da = abs(en_yakin[3] - onceki[3])
                    da = min(da, math.pi - da)             # ±π sarması
                    d_genislik.append(abs(en_yakin[2] - onceki[2]) / dt)
                    d_aci.append(math.degrees(da) / dt)
                    d_orta.append(kayma / dt)
            onceki = (t_kare, en_yakin[1], en_yakin[2], en_yakin[3])

    N = np.array(kenar_sayisi)
    print(f"  {len(kareler)} kare · kare başına kenar dubası: ortanca {np.median(N):.1f}"
          f" · tepe {N.max() if len(N) else 0} · ≥2 duba görülen kare {int((N >= 2).sum())}")
    if not kapi_araligi:
        print("  kursa dik çift HİÇ oluşmadı — kapı genişliği ölçülemedi")
        return
    G = np.array(kapi_araligi)
    T = np.array(tum_ciftler)
    print(f"  bütün kenar-kenar çiftleri: {len(T)} · ortanca {np.median(T):.2f} m")
    print(f"  🔑 KAPI ÇİFTİ (|Δileri|<|Δyanal|, ≥{MIN_W:.2f} m): {len(G)} örnek")
    print(f"     ortanca {np.median(G):.2f} m · %5 {np.percentile(G, 5):.2f}"
          f" · %95 {np.percentile(G, 95):.2f} · en dar {G.min():.2f} · en geniş {G.max():.2f} m")
    kenarlar = [MIN_W, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 999.0]
    for a, b in zip(kenarlar[:-1], kenarlar[1:]):
        p = 100.0 * float(((G >= a) & (G < b)).sum()) / len(G)
        print(f"     {a:5.2f}–{b:5.1f} m: %{p:4.1f} " + "█" * int(p / 2))

    # ⑦b — kapı GERÇEKTEN mi kayıyor, algı mı zıplatıyor?
    if d_genislik:
        W = np.array(d_genislik)
        A = np.array(d_aci)
        O = np.array(d_orta)
        print(f"  🔑 ARDIŞIK KARE DEĞİŞİMİ ({len(W)} eşleşme, <1 sn ara, orta nokta <1 m kaymış)")
        print(f"     genişlik: ortanca {np.median(W):.2f} · %95 {np.percentile(W, 95):.2f} m/s")
        print(f"     kiriş açısı: ortanca {np.median(A):.1f} · %95 {np.percentile(A, 95):.1f} °/s")
        print(f"     orta nokta: ortanca {np.median(O):.2f} · %95 {np.percentile(O, 95):.2f} m/s")
        print("     ⚠ suyun sürüklemesi cm/s mertebesindedir; bunun üstü ALGI oynamasıdır")


if __name__ == "__main__":
    main()
