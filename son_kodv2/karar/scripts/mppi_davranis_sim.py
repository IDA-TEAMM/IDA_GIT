#!/usr/bin/env python3
"""Girdap İDA — MPPI DAVRANIŞ SİMÜLASYONU (görsel, 17.08.2026).

🔴 NEDEN VAR: `kapi_orani.py` **sayı** üretiyor (kaç kapı, kaç PDÇ), ama
"MPPI ne YAPIYOR" sorusu sayıdan okunmuyor. 17.08 ölçümünde zor başlangıçta
3 koşum 8 kapının **0'ını** geçti ve 370 sn parkur dışında kaldı; sayı bunu
gösteriyor ama **nerede** ve **neden** saptığını göstermiyor. Bu araç aynı
kapalı döngüyü koşturup **izi çiziyor** — arıza kipi gözle ayırt edilebilir
hâle geliyor.

⚠ SAHTE MODEL YOK: gerçek çekirdekler koşar — `PlanningPipeline` (MPPI +
RRT* bypass) · `GateFollower` · `EdgeBuoyMemory` · `CatamaranDynamics` ·
`ParkurSiniri`/`ParkurDisiSayaci`. Parkur `prototype/configs/parkur_nihai.world`
dosyasından okunur (kapı açıklığı 12 m, kaptan teyitli — §0.17b).
Koşum mantığı `kapi_orani.py`'den İTHAL EDİLİR, kopyalanmaz (tek kaynak).

📐 ŞARTNAME BAĞI: karşılıklı kenar dubası arası mesafe **8-12 m**
(Şekil 3 lejantı; metinde YOK — `pc-memory/sartname-gorsel-bulgulari.md`).
Yani 12 m kapıda ~2 m'lik ortalama sapması TEK BAŞINA risk değil; puanı
yiyen şey **PDÇ** (şartname s.24-25) ve kapı kaybı.

KULLANIM:
    python3 scripts/mppi_davranis_sim.py                    # normal + zor
    python3 scripts/mppi_davranis_sim.py --kip zor --kosum 6
    python3 scripts/mppi_davranis_sim.py --cikti ~/girdap_logs/mppi.png
    python3 scripts/mppi_davranis_sim.py --gpu               # ⚠ §1.47'yi oku
    python3 scripts/mppi_davranis_sim.py --bellek 3G         # kafes tavanını değiştir

🔴🔴 ÜÇ KAPI — 17.08.2026 DONMASININ ÜRÜNÜ (GIRDAP_DURUM §1.47):
Bu betiğin **üç eşzamanlı koşumu** 17.08 15:19'da Jetson'ı dondurdu; her
süreç 2,4 GB yerleşik bellek tutuyordu, 3,7 GB'lık sıkıştırılmış takas
alanı **tamamen** doldu (`Free swap = 0kB`), çekirdek takas cambazlığına
girdi ve makine elle kapatılmak zorunda kaldı. Sebep ayar değil **koşum
biçimiydi**; o yüzden ayar yerine üç kapı kondu:

  ① **Tek örnek kilidi** — ikinci koşum başlamaz, açık uyarı verip çıkar.
  ② **Bellek kafesi** — koşum `systemd-run --user --scope` içinde, tavanı
     aşarsa **betik** ölür (`MemorySwapMax=0` → takasa hiç dokunamaz),
     yığın ayakta kalır. `--bellek yok` ile kapatılır.
  ③ **İşlemci (numpy) varsayılan** — bu araç gerçek zamanlı değil, grafik
     üretiyor; ekran kartı hesap yolunun süreç başına ~8,5 GB sanal alan +
     havuz maliyetine gerek yok. `--gpu` açık istektir.

Ayrıca süreç kendi `oom_score_adj`'ını **800**'e çeker: bellek gerçekten
biterse çekirdek ilk bunu öldürür, `girdap-karar`/`girdap-algi`'yı değil.
"""
from __future__ import annotations

import argparse
import fcntl
import importlib.util
import math
import os
import shutil
import statistics
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

_KOK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_KOK))

from prototype.mission.parkur_dunyasi import oku as parkuru_oku  # noqa: E402

# `kapi_orani.py` bir betik (paket değil) → importlib ile modül olarak yükle.
# Koşum mantığını kopyalamak iki kaynak demek olurdu; §0.17b'nin dersi bu.
_spec = importlib.util.spec_from_file_location(
    "kapi_orani", Path(__file__).resolve().parent / "kapi_orani.py"
)
ko = importlib.util.module_from_spec(_spec)
sys.modules["kapi_orani"] = ko
_spec.loader.exec_module(ko)

DISARIDA = "DISARIDA"

# --- §1.47 kapıları -----------------------------------------------------------
_KAFES_ISARETI = "GIRDAP_MPPI_SIM_KAFESTE"   # yeniden başlatma döngüsü kapısı
VARSAYILAN_BELLEK = "2G"                     # ölçüm: tek koşum tepe 2,4 GB (gpu)
KILIT_YOLU = Path.home() / ".cache" / "girdap" / "mppi_sim.kilit"
_kilit_dosyasi = None                        # ömrü sürecin ömrü kadar (kapanmaz)


def _kafes_kurulabilir() -> bool:
    """`systemd-run --user --scope` bu makinede bellek tavanı uygulayabiliyor mu?

    Kafes kurulamıyorsa koşumu ENGELLEMEYİZ (araç teşhis aracı, kırılgan
    olmamalı) — ama sessizce de geçmeyiz, uyarı basılır.
    """
    if shutil.which("systemd-run") is None:
        return False
    try:
        deneme = subprocess.run(
            ["systemd-run", "--user", "--scope", "--quiet",
             "-p", "MemoryMax=64M", "-p", "MemorySwapMax=0", "/bin/true"],
            capture_output=True, timeout=20,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return deneme.returncode == 0


def _kafese_gir(tavan: str) -> None:
    """Kapı ②: kendini bellek tavanlı bir `scope` içinde yeniden başlat.

    Neden `RLIMIT_AS` değil: ekran kartı hesap yolu süreç başına ~8,5 GB
    **sanal** alan ayırıyor (CUDA bağlamı) — sanal adres sınırı ya koşumu
    hemen öldürür ya da tavanı anlamsız yükseğe çeker. cgroup tavanı
    YERLEŞİK belleği sayar, doğru büyüklük odur.
    Neden `MemorySwapMax=0`: 17.08'de makineyi donduran şey tavan aşımı
    değil, **takas cambazlığıydı**; takas hiç kullanılmazsa aşım anında
    tek kurban bu süreç olur.
    """
    if tavan == "yok" or os.environ.get(_KAFES_ISARETI) == "1":
        return
    if not _kafes_kurulabilir():
        print(f"⚠ bellek kafesi KURULAMADI (systemd-run yok/yetkisiz) → "
              f"koşum tavansız gidiyor. Tepe belleği izle: free -m")
        return
    ortam = dict(os.environ)
    ortam[_KAFES_ISARETI] = "1"
    komut = ["systemd-run", "--user", "--scope", "--quiet",
             "-p", f"MemoryMax={tavan}", "-p", "MemorySwapMax=0",
             sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]]
    # ⚠ flush ZORUNLU: tavan aşılırsa süreç SIGKILL alır (çıkış 137) ve
    # tamponda bekleyen bu satır kaybolur — kullanıcı kafesi hiç görmez.
    print(f"🔒 bellek kafesi: MemoryMax={tavan} · MemorySwapMax=0 "
          f"(aşarsa yalnız bu betik ölür — §1.47)", flush=True)
    os.execvpe(komut[0], komut, ortam)


def _tek_ornek_kilidi(yol: Optional[Path] = None) -> None:
    """Kapı ①: aynı anda ikinci koşum başlamasın.

    `flock` seçildi çünkü süreç nasıl ölürse ölsün (öldürüldü, elektrik
    kesildi) kilit çekirdek tarafından bırakılır — bayat kilit dosyası
    sorunu doğmaz.
    """
    global _kilit_dosyasi
    yol = yol or KILIT_YOLU
    yol.parent.mkdir(parents=True, exist_ok=True)
    dosya = open(yol, "w")
    try:
        fcntl.flock(dosya.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        dosya.close()
        raise SystemExit(
            f"⛔ BU BETİK ZATEN KOŞUYOR (kilit: {yol}).\n"
            f"   17.08'de üç eşzamanlı koşum Jetson'ı dondurdu (§1.47) — "
            f"ikinci koşum başlatılmıyor.\n"
            f"   Koşan süreç: pgrep -af mppi_davranis_sim"
        )
    _kilit_dosyasi = dosya                     # kapatılmaz: kilit süreçle ölür


def _oom_onceligini_yukselt(deger: int = 800) -> None:
    """Bellek gerçekten biterse çekirdek İLK bu süreci öldürsün.

    Yığın (`girdap-karar`, `girdap-algi`) 0 önceliğinde kalır; teşhis aracı
    onların önüne geçmez. Yazma başarısız olursa sessizce geçilir — bu bir
    iyileştirme, koşum ön koşulu değil.
    """
    try:
        Path("/proc/self/oom_score_adj").write_text(f"{deger}\n")
    except OSError:
        pass


def _hesap_yolunu_sec(gpu: bool) -> str:
    """Kapı ③: varsayılan işlemci (numpy).

    `MPPIConfig.backend` boru hattı yapılandırmasında **açığa çıkmıyor**
    (`PlanningPipelineConfig`'te alan yok), ama `_resolve_backend("auto")`
    ekran kartı sayısı 0 görürse sessizce numpy'a düşüyor — doğru kol bu:
    `CUDA_VISIBLE_DEVICES` boşaltılır, kütüphane dokunulmaz.
    """
    if gpu:
        return "ekran kartı (cupy/float32) — açık istek"
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    return "işlemci (numpy/float64) — varsayılan"


def kosumlari_topla(parkur, zor: bool, adet: int, sure: float,
                    mppi_k: int, mppi_t: int) -> List[dict]:
    """Her başlangıç için kapalı döngüyü koştur, iz + metrik topla."""
    sonuc = []
    for i, (poz, aci) in enumerate(ko.baslangiclar(parkur, zor, adet), 1):
        iz: List[dict] = []
        r = ko.kosum(parkur, baslangic=poz, yon_hatasi_rad=aci,
                     sure=sure, mppi_k=mppi_k, mppi_t=mppi_t, iz=iz)
        r["iz"] = iz
        r["baslangic"] = poz
        r["aci_derece"] = math.degrees(aci)
        r["no"] = i
        sonuc.append(r)
        print(f"  koşum {i:2d} ({poz[0]:6.1f},{poz[1]:5.1f}) "
              f"{math.degrees(aci):3.0f}°  kapı {r['gecilen']}/{r['toplam_kapi']}"
              f"  GN {r['varilan_gn']}/{r['toplam_gn']}"
              f"  sapma {r['sapma_ort']:.2f} m"
              f"  PDÇ {r['pdc']}  dışarıda {r['pdc_sure']:.0f} s")
    return sonuc


def _parkuru_ciz(ax, parkur) -> None:
    """Kapı direkleri, kapı kirişleri, engeller, görev noktaları."""
    for k, (sol, sag) in enumerate(parkur.kapilar):
        ax.plot([sol[0], sag[0]], [sol[1], sag[1]],
                color="0.75", lw=1.0, zorder=1)
        orta = ((sol[0] + sag[0]) / 2.0, (sol[1] + sag[1]) / 2.0)
        ax.plot(*orta, marker="+", color="0.55", ms=6, mew=1.2, zorder=2)
    dubalar = parkur.dubalar()
    ax.scatter([b[0] for b in dubalar], [b[1] for b in dubalar],
               s=22, c="#d95f02", marker="o", zorder=3,
               label=f"kenar dubası ({len(dubalar)})")
    if parkur.engeller:
        ax.scatter([b[0] for b in parkur.engeller],
                   [b[1] for b in parkur.engeller],
                   s=26, c="#e6ab02", marker="D", zorder=3,
                   label=f"engel dubası ({len(parkur.engeller)})")
    gn = ko.gorev_rotasi(parkur)
    ax.plot([p[0] for p in gn], [p[1] for p in gn],
            ls=":", color="#377eb8", lw=1.0, zorder=2)
    ax.scatter([p[0] for p in gn], [p[1] for p in gn],
               s=60, c="#377eb8", marker="s", zorder=4,
               label=f"görev noktası ({len(gn)})")
    for i, p in enumerate(gn, 1):
        ax.annotate(f"GN{i}", p, textcoords="offset points", xytext=(5, 5),
                    fontsize=7, color="#377eb8")


def _izi_ciz(ax, r: dict, renk: str) -> None:
    """Tek koşumun izi; parkur DIŞINDA geçen parçalar kalın kırmızı."""
    iz = r["iz"]
    xs = [k["x"] for k in iz]
    ys = [k["y"] for k in iz]
    ax.plot(xs, ys, color=renk, lw=1.1, alpha=0.85, zorder=5)
    # Dışarıda geçen adımlar — ardışık parçalar hâlinde
    parca_x: List[float] = []
    parca_y: List[float] = []
    for k in iz:
        if k.get("parkur_durumu") == DISARIDA:
            parca_x.append(k["x"])
            parca_y.append(k["y"])
        elif parca_x:
            ax.plot(parca_x, parca_y, color="#e41a1c", lw=2.6, alpha=0.75,
                    zorder=6, solid_capstyle="round")
            parca_x, parca_y = [], []
    if parca_x:
        ax.plot(parca_x, parca_y, color="#e41a1c", lw=2.6, alpha=0.75,
                zorder=6, solid_capstyle="round")
    ax.plot(xs[0], ys[0], marker="o", color=renk, ms=6, mec="k", mew=0.6,
            zorder=7)
    ax.plot(xs[-1], ys[-1], marker="X", color=renk, ms=8, mec="k", mew=0.6,
            zorder=7)


def cizim(parkur, kosumlar: List[dict], baslik: str, cikti: Path) -> None:
    n = len(kosumlar)
    fig, axlar = plt.subplots(2, 1, figsize=(13.5, 11.0),
                              gridspec_kw={"height_ratios": [3, 2]})
    ust, alt = axlar

    _parkuru_ciz(ust, parkur)
    renkler = plt.cm.viridis([i / max(1, n - 1) for i in range(n)])
    for r, c in zip(kosumlar, renkler):
        _izi_ciz(ust, r, c)
    ust.plot([], [], color="#e41a1c", lw=2.6, label="PARKUR DIŞINDA")
    ust.set_aspect("equal", adjustable="datalim")
    ust.set_title(baslik, fontsize=11)
    ust.set_xlabel("x [m] (ENU)")
    ust.set_ylabel("y [m]")
    ust.grid(alpha=0.25, lw=0.5)
    ust.legend(fontsize=7, loc="upper left", ncol=2)

    # Alt panel: koşum başına kapı oranı + dışarıda geçen süre
    nolar = [r["no"] for r in kosumlar]
    oran = [100.0 * r["gecilen"] / r["toplam_kapi"] for r in kosumlar]
    dis = [r["pdc_sure"] for r in kosumlar]
    alt.bar([x - 0.2 for x in nolar], oran, width=0.4,
            color="#4daf4a", label="kapı geçme %")
    alt.set_ylabel("kapı geçme [%]", color="#4daf4a")
    alt.set_ylim(0, 105)
    alt.set_xlabel("koşum no")
    alt.grid(alpha=0.25, lw=0.5, axis="y")
    ikiz = alt.twinx()
    ikiz.bar([x + 0.2 for x in nolar], dis, width=0.4,
             color="#e41a1c", label="parkur dışında [s]")
    ikiz.set_ylabel("parkur dışında [s]", color="#e41a1c")
    for x, o, d, r in zip(nolar, oran, dis, kosumlar):
        alt.annotate(f"{r['baslangic'][1]:+.1f}m\n{r['aci_derece']:.0f}°",
                     (x, 2), ha="center", fontsize=6, color="0.3")
    alt.set_title("koşum başına: kapı geçme oranı (yeşil) ↔ parkur dışında "
                  "geçen süre (kırmızı) · etiket = başlangıç yanal kaçıklığı/açı",
                  fontsize=9)

    fig.tight_layout()
    cikti.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(cikti, dpi=130)
    plt.close(fig)
    print(f"\n  → çizim: {cikti}")


def ozet(kosumlar: List[dict]) -> Dict[str, float]:
    oranlar = [r["gecilen"] / r["toplam_kapi"] for r in kosumlar]
    sapmalar = [r["sapma_ort"] for r in kosumlar
                if not math.isnan(r["sapma_ort"])]
    return {
        "kapi_ort": 100.0 * statistics.mean(oranlar),
        "kapi_en_kotu": 100.0 * min(oranlar),
        "sapma": statistics.mean(sapmalar) if sapmalar else float("nan"),
        "pdc_ort": statistics.mean(r["pdc"] for r in kosumlar),
        "dis_sure_ort": statistics.mean(r["pdc_sure"] for r in kosumlar),
        "sifir_kapi": sum(1 for r in kosumlar if r["gecilen"] == 0),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="MPPI davranış simülasyonu")
    ap.add_argument("--kip", choices=("normal", "zor", "ikisi"), default="ikisi")
    ap.add_argument("--kosum", type=int, default=6)
    ap.add_argument("--sure", type=float, default=400.0)
    ap.add_argument("--K", type=int, default=ko.URETIM_K)
    ap.add_argument("--T", type=int, default=ko.URETIM_T)
    ap.add_argument("--cikti", type=Path,
                    default=Path.home() / "girdap_logs" / "mppi_sim")
    ap.add_argument("--gpu", action="store_true",
                    help="ekran kartı hesap yolu (cupy). ⚠ süreç başına ~2,4 GB "
                         "— 17.08 donmasının kaynağı (§1.47)")
    ap.add_argument("--bellek", default=VARSAYILAN_BELLEK,
                    help=f"bellek kafesi tavanı (varsayılan {VARSAYILAN_BELLEK}); "
                         f"'yok' → kafes kurulmaz")
    n = ap.parse_args()

    # §1.47 kapıları — SIRA ÖNEMLİ: kafes (süreci yeniden başlatır) → kilit
    # (kafesin içinde tutulsun) → öncelik → hesap yolu (ağır import'lardan önce).
    _kafese_gir(n.bellek)
    _tek_ornek_kilidi()
    _oom_onceligini_yukselt()
    hesap_yolu = _hesap_yolunu_sec(n.gpu)

    parkur = parkuru_oku()
    gen = parkur.kapi_genislikleri
    print(f"parkur: {len(parkur.kapilar)} kapı · açıklık "
          f"{min(gen):.1f}-{max(gen):.1f} m · {len(parkur.engeller)} engel · "
          f"MPPI K={n.K} T={n.T} · hesap yolu: {hesap_yolu}")
    print("şartname Şekil 3: karşılıklı kenar dubası arası 8-12 m\n")

    kipler = (("normal", False), ("zor", True)) if n.kip == "ikisi" \
        else ((n.kip, n.kip == "zor"),)

    for ad, zor in kipler:
        print(f"── {ad.upper()} kip ({n.kosum} koşum)")
        ks = kosumlari_topla(parkur, zor, n.kosum, n.sure, n.K, n.T)
        o = ozet(ks)
        print(f"  ÖZET  kapı ort %{o['kapi_ort']:.1f} · en kötü "
              f"%{o['kapi_en_kotu']:.1f} · sapma {o['sapma']:.2f} m · "
              f"PDÇ ort {o['pdc_ort']:.1f} · dışarıda ort "
              f"{o['dis_sure_ort']:.0f} s · SIFIR kapı {o['sifir_kapi']}"
              f"/{len(ks)}")
        cizim(parkur, ks,
              f"MPPI davranışı — {ad} başlangıçlar (K={n.K} T={n.T}) · "
              f"kapı açıklığı {min(gen):.0f}-{max(gen):.0f} m",
              Path(str(n.cikti) + f"_{ad}.png"))
        print()


if __name__ == "__main__":
    main()
