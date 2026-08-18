#!/usr/bin/env python3
"""GİRDAP İDA — BANT VİDEOSU: koşumda gerçekte NASIL bir yol izlendi (16.08.2026).

🔴 NEDEN VAR (kaptan isteği): *"rosbaglerle test eder misin, gerçekten yay gibi
bir şey mi oluşuyor; canlı bir video yapar mısın nasıl bir yol oluşturuyor."*
Sayı tablosu *"itkinin %58'i pivot"* diyor ama bir yörüngenin **yay mı yoksa
kırık çizgi mi** olduğu ancak bakılınca anlaşılır. Bu araç bandı olduğu gibi
oynatır — simülasyon değil, o gün suda ne olduysa o.

NE ÇİZER (hepsi bandın kendi kaydından, hiçbir şey yeniden hesaplanmaz):
  · teknenin GERÇEK izi — **itkiye göre renkli**: ileri giderken yeşil,
    pivot (zıt itki, yerinde dönüş) kırmızı, duruşta gri. "Dön–düz git–dön"
    davranışı bu renklerde doğrudan görünür.
  · o an kilitli kapı nişanı (yıldız) ve tekneden nişana giden çizgi:
    nişan zıpladığında çizgi bir karede öbür tarafa atlar.
  · kenar dubaları (turuncu) ve sınıfsız engeller (gri) — düğümün o an
    gördükleri, yani kararın girdisi.
  · sağ üstte saat · hız · itki · pivot bayrağı.

KULLANIM:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 scripts/bant_video.py <bant_dizini> [--ofset 2085] [--sure 300]
                                  [--hiz 8] [--cikti yol.mp4]

`--hiz` = gerçek zamana göre hızlandırma (varsayılan 8×). Çıktı mp4
(ffmpeg yoksa gif). Varsayılan çıktı: ~/girdap_logs/viz/bant_yol_<etiket>.mp4

⚠ SALT OKUR — banda ve üretim koduna dokunmaz.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

# Başsız Jetson: pencere açmaya çalışmasın.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                   # noqa: E402
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from scripts.bant_kapi_olcum import bant_oku                      # noqa: E402

KONULAR_VIDEO = [
    "/girdap/fusion/pose",
    "/girdap/control/thrust",
    "/girdap/planning/gate",
    "/girdap/planning/edge_buoys",
    "/perception/classified_obstacles",
    "/girdap/planning/global_path",     # yalnız yeniden koşum kaydında bulunur
]
PIVOT_ESIGI = 0.05          # N — altındaki itki "duruyor" sayılır (gürültü bandı)


def _yaw(q) -> float:
    """Quaternion → ψ (ENU, radyan)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _en_yakin(dizi, t):
    """`dizi` (zaman, değer) sıralı; t'ye kadar olan SON değeri döndür."""
    i = np.searchsorted([d[0] for d in dizi], t, side="right") - 1
    return dizi[i][1] if i >= 0 else None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bant")
    ap.add_argument("--ofset", type=float, default=0.0, help="bant başından saniye")
    ap.add_argument("--sure", type=float, default=300.0)
    ap.add_argument("--hiz", type=float, default=8.0, help="hızlandırma katsayısı")
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--cikti", default=None)
    a = ap.parse_args()

    import scripts.bant_kapi_olcum as bko
    bko.KONULAR = KONULAR_VIDEO                    # yalnız gerekli konular okunur
    v = bant_oku(a.bant)
    poz = v.get("/girdap/fusion/pose") or []
    if not poz:
        sys.exit("bantta /girdap/fusion/pose yok — video üretilemez")

    t0 = poz[0][0]
    bas, bit = t0 + a.ofset, t0 + a.ofset + a.sure

    iz = [(t - t0, m.pose.position.x, m.pose.position.y, _yaw(m.pose.orientation))
          for t, m in poz if bas <= t <= bit]
    if len(iz) < 2:
        sys.exit("seçilen pencerede poz yok — --ofset/--sure değerlerini kontrol et")

    itki = [(t - t0, tuple(m.data[:2])) for t, m in (v.get("/girdap/control/thrust") or [])
            if len(m.data) >= 2]
    nisan = [(t - t0, (m.pose.position.x, m.pose.position.y))
             for t, m in (v.get("/girdap/planning/gate") or [])]
    kenar = [(t - t0, [(p.position.x, p.position.y) for p in m.poses])
             for t, m in (v.get("/girdap/planning/edge_buoys") or [])]
    # Küresel yol: canlı koşumda BANDA KAYDEDİLMİYORDU; yalnız
    # `bant_kosum_deneyi` ile yeniden koşup çıktısını kaydettiğimizde bulunur.
    yol = [(t - t0, [(p.pose.position.x, p.pose.position.y) for p in m.poses])
           for t, m in (v.get("/girdap/planning/global_path") or [])]

    # Sınıflı tespitler gövde çerçevesinde → dünyaya teknenin o anki pozuyla taşınır.
    ham = []
    for t, m in (v.get("/perception/classified_obstacles") or []):
        ts = t - t0
        if not (a.ofset <= ts <= a.ofset + a.sure):
            continue
        noktalar = []
        for det in m.detections:
            cls = None
            if det.results:
                try:
                    cls = int(det.results[0].hypothesis.class_id)
                except (TypeError, ValueError):
                    cls = None
            c = det.bbox.center.position
            noktalar.append((c.x, c.y, cls))
        ham.append((ts, noktalar))

    X = np.array([p[1] for p in iz])
    Y = np.array([p[2] for p in iz])
    T = np.array([p[0] for p in iz])
    pay = max(6.0, 0.12 * max(X.ptp(), Y.ptp()))

    fig, ax = plt.subplots(figsize=(9, 9), dpi=110)
    ax.set_aspect("equal")
    ax.set_xlim(X.min() - pay, X.max() + pay)
    ax.set_ylim(Y.min() - pay, Y.max() + pay)
    ax.set_xlabel("doğu (m)")
    ax.set_ylabel("kuzey (m)")
    ax.grid(alpha=0.25, linewidth=0.5)
    ax.set_title(f"{os.path.basename(a.bant.rstrip('/'))} — gerçek iz "
                 f"(t={a.ofset:.0f}…{a.ofset + a.sure:.0f} s, {a.hiz:.0f}× hız)")

    ax.plot(X, Y, color="0.85", linewidth=1.0, zorder=1)          # tüm iz, soluk
    iz_cizgileri = ax.scatter([], [], s=6, zorder=3)              # renkli geçmiş
    tekne = ax.plot([], [], marker=(3, 0, 0), markersize=13,
                    color="tab:blue", zorder=6)[0]
    nisan_nk = ax.plot([], [], marker="*", markersize=17,
                       color="tab:red", linestyle="", zorder=6)[0]
    nisan_cizgi = ax.plot([], [], color="tab:red", linewidth=1.0,
                          alpha=0.6, zorder=5)[0]
    kenar_nk = ax.plot([], [], marker="o", markersize=7, color="darkorange",
                       linestyle="", zorder=4)[0]
    engel_nk = ax.plot([], [], marker=".", markersize=4, color="0.5",
                       linestyle="", zorder=2)[0]
    yol_cizgi = ax.plot([], [], color="tab:blue", linewidth=1.8, alpha=0.85,
                        linestyle="--", zorder=5)[0]
    yazi = ax.text(0.015, 0.985, "", transform=ax.transAxes, va="top", ha="left",
                   fontsize=9, family="monospace",
                   bbox=dict(fc="white", ec="0.7", alpha=0.85))
    ax.plot([], [], color="tab:green", linewidth=3, label="ileri sürüş")
    ax.plot([], [], color="tab:red", linewidth=3, label="PİVOT (yerinde dönüş)")
    ax.plot([], [], color="0.6", linewidth=3, label="duruyor")
    ax.plot([], [], marker="o", color="darkorange", linestyle="",
            label="kenar dubası (kapı direği)")
    ax.plot([], [], marker="*", color="tab:red", linestyle="", label="kapı nişanı")
    ax.plot([], [], color="tab:blue", linestyle="--", label="RRT* küresel yolu")
    ax.legend(loc="lower right", fontsize=8, framealpha=0.9)

    adim = a.hiz / a.fps                                # video karesi başına bant saniyesi
    kare_t = np.arange(a.ofset, a.ofset + a.sure, adim)

    def renk(u):
        if u is None:
            return "0.6"
        ul, ur = u
        if abs(ul) < PIVOT_ESIGI and abs(ur) < PIVOT_ESIGI:
            return "0.6"
        if ul * ur < 0.0:                               # zıt işaret = pivot
            return "tab:red"
        return "tab:green" if (ul + ur) > 0.0 else "tab:orange"

    def kare(k):
        t = kare_t[k]
        i = int(np.searchsorted(T, t, side="right") - 1)
        if i < 0:
            i = 0
        gecmis = slice(0, i + 1)
        renkler = [renk(_en_yakin(itki, tt)) for tt in T[gecmis]]
        iz_cizgileri.set_offsets(np.column_stack([X[gecmis], Y[gecmis]]))
        iz_cizgileri.set_color(renkler)

        x, y, psi = X[i], Y[i], iz[i][3]
        tekne.set_data([x], [y])
        tekne.set_marker((3, 0, math.degrees(psi) - 90.0))

        n = _en_yakin(nisan, t)
        if n is not None:
            nisan_nk.set_data([n[0]], [n[1]])
            nisan_cizgi.set_data([x, n[0]], [y, n[1]])
        else:
            nisan_nk.set_data([], [])
            nisan_cizgi.set_data([], [])

        kb = _en_yakin(kenar, t) or []
        kenar_nk.set_data([p[0] for p in kb], [p[1] for p in kb])

        yy = _en_yakin(yol, t) if yol else None
        if yy:
            yol_cizgi.set_data([p[0] for p in yy], [p[1] for p in yy])
        else:
            yol_cizgi.set_data([], [])

        h = _en_yakin(ham, t) or []
        hx = [x + p[0] * math.cos(psi) - p[1] * math.sin(psi) for p in h]
        hy = [y + p[0] * math.sin(psi) + p[1] * math.cos(psi) for p in h]
        engel_nk.set_data(hx, hy)

        u = _en_yakin(itki, t)
        hiz = 0.0
        if i > 0:
            dt = T[i] - T[i - 1]
            if dt > 0:
                hiz = math.hypot(X[i] - X[i - 1], Y[i] - Y[i - 1]) / dt
        u_metin = "yok" if u is None else f"{u[0]:+.2f} / {u[1]:+.2f} N"
        pivot = "EVET" if (u is not None and u[0] * u[1] < 0.0) else "hayır"
        yazi.set_text(f"t = {t:7.1f} s\nhız  = {hiz:4.2f} m/s\n"
                      f"itki = {u_metin}\npivot= {pivot}\n"
                      f"kenar dubası = {len(kb)}")
        return (iz_cizgileri, tekne, nisan_nk, nisan_cizgi, kenar_nk,
                engel_nk, yol_cizgi, yazi)

    anim = FuncAnimation(fig, kare, frames=len(kare_t), interval=1000 // a.fps,
                         blit=False)
    etiket = os.path.basename(a.bant.rstrip("/"))
    cikti = a.cikti or os.path.expanduser(
        f"~/girdap_logs/viz/bant_yol_{etiket}_{int(a.ofset)}.mp4")
    os.makedirs(os.path.dirname(cikti), exist_ok=True)
    try:
        anim.save(cikti, writer=FFMpegWriter(fps=a.fps, bitrate=2400))
    except Exception as exc:                       # noqa: BLE001 — ffmpeg yoksa gif
        print(f"⚠ ffmpeg yazıcı kullanılamadı ({type(exc).__name__}) → gif'e düşülüyor")
        cikti = os.path.splitext(cikti)[0] + ".gif"
        anim.save(cikti, writer=PillowWriter(fps=a.fps))
    print(f"✅ video: {cikti}  ({len(kare_t)} kare · {len(kare_t)/a.fps:.0f} s · "
          f"{a.hiz:.0f}× hız)")


if __name__ == "__main__":
    main()
