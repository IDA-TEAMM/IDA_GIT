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


def bant_oku(dizin: str) -> dict:
    """Bandı okur; metadata yoksa .mcap'leri doğal sırayla tek tek dener."""
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
            r.open(rosbag2_py.StorageOptions(uri=p, storage_id="mcap"),
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
