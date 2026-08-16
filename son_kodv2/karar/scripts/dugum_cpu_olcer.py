#!/usr/bin/env python3
"""GİRDAP İDA — DÜĞÜM DÜĞÜM CPU ÖLÇER (saha aracı, bağımlılıksız).

🔴 NEDEN VAR (17.08.2026):
    16.08 göl bandının tegrastats'ında **6 çekirdek ortalama %80,1 doluluk**,
    örneklerin %41,3'ü ≥%85, frekans sabit 1728 MHz (**kısılma YOK** — sistem
    tam hızda ve yetişemiyor). Yani CPU tükeniyor. Ama **hangi düğümün ne
    yediğini kimse ölçmedi** ve geriye dönük ölçülemiyor:
      · `/diagnostics` bantlarda YALNIZ MAVROS'u taşıyor (tarandı: 9 durum
        adının hepsi `mavros*`; GIRDAP düğümlerinden tek kayıt yok),
      · repoda süreç/CPU okuyan tek satır kod yoktu
        (`grep -rln "psutil|/proc/stat|cpu_percent"` → boş).
    ⇒ Bu araç o boşluğu kapatır. Bir sonraki su koşumunda `canli_nobetci` ile
    birlikte koşturulur ve soru **ölçümle** kapanır.

🪤 ÖNCE ELENEN ADAYLAR (17.08, ölçüldü — bu araç onları tekrar aramasın diye):
    · MPPI      → cupy yolunda **9,9 ms/adım** (§1.24) = 10 Hz'te ~%10 tek çekirdek
    · LiDAR kümeleme → 20.000 noktada **22,9 ms/tarama** = 10 Hz bütçesinin %23'ü
      (Livox Mid-360 = 200.000 nokta/s, resmî spec) ≈ 6 çekirdeğin %3,8'i
    · rosbag2   → `compression: "None"` + `noChunking: true` (sıkıştırma yok)
    · iSAM2     → 20 dk görevde toplam 5,5 s CPU (§ eski ölçüm)
    Üçü/dördü toplansa bile %80,1 çıkmıyor ⇒ **kalan yük başka yerde**.

⚙️ YÖNTEM ve GEÇERLİLİK ŞARTI:
    `ros2 launch` her düğümü **AYRI SÜREÇ** olarak açar (17.08'de doğrulandı:
    planning_node, fusion_node, duba_gecis_navigator … hepsi kendi PID'i).
    Bu yüzden `/proc/<pid>/stat`'tan süreç bazlı atıf GEÇERLİDİR.
    ⚠ Düğümler bir gün `ComponentContainer` içinde toplanırsa atıf ANLAMINI
    YİTİRİR (tek süreçte çok düğüm). Araç bunu tespit edip UYARIR.

🔬 ARACIN KENDİ DOĞRULAMASI (memory kuralı: *ölçüm aracı doğrulanmadan
    sonucu raporlanmaz*): her turda süreç toplamı, `/proc/stat`'tan bağımsız
    okunan sistem geneli meşguliyetle kıyaslanır. Süreç toplamı sistemi
    AŞIYORSA sayım hatalıdır ve satır `⚠ TUTARSIZ` damgalanır.

KULLANIM:
    python3 scripts/dugum_cpu_olcer.py                  # 30 sn, ekrana
    python3 scripts/dugum_cpu_olcer.py --sure 600 --csv ~/girdap_logs/cpu.csv
    python3 scripts/dugum_cpu_olcer.py --aralik 2 --sure 0   # süresiz (Ctrl-C)

ÇIKTI: her tur bir satır CSV (zaman, pid, ad, cpu_yuzde, rss_mb) + kapanışta
özet tablo. Kendi maliyeti ihmal edilebilir (~1 ms/tur, yalnız /proc okur).
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from collections import defaultdict

_SAAT_TIK = os.sysconf("SC_CLK_TCK")          # genelde 100
_SAYFA_KB = os.sysconf("SC_PAGE_SIZE") // 1024

#: Bizim saydığımız süreçler — args içinde bunlardan biri geçmeli.
_DESENLER = (
    "girdap_decision/lib/",       # karar düğümleri
    "girdap_ida_algi/lib/",       # algı düğümü
    "mavros_node",                # MAVROS (C++ — sık en pahalı olan)
    "ros2 bag record",            # kayıt
    "livox_ros_driver2",          # LiDAR sürücüsü
)


def _oku_stat(pid: str):
    """(utime+stime tik, rss_kb, comm) — süreç kaybolduysa None."""
    try:
        with open(f"/proc/{pid}/stat", "rb") as f:
            ham = f.read().decode("utf-8", "replace")
        # comm parantez içinde ve boşluk içerebilir → sondan ayrıştır
        sag = ham[ham.rindex(")") + 2:].split()
        comm = ham[ham.index("(") + 1: ham.rindex(")")]
        utime, stime = int(sag[11]), int(sag[12])       # alan 14,15
        rss = int(sag[21]) * _SAYFA_KB                  # alan 24
        return utime + stime, rss, comm
    except (OSError, ValueError, IndexError):
        return None


def _oku_sistem():
    """/proc/stat: (mesgul_tik, toplam_tik) — süreçlerden BAĞIMSIZ ikinci kaynak."""
    try:
        with open("/proc/stat") as f:
            a = f.readline().split()
        v = [int(x) for x in a[1:11]]
        toplam = sum(v)
        bos = v[3] + v[4]                                # idle + iowait
        return toplam - bos, toplam
    except (OSError, ValueError, IndexError):
        return None, None


def _ad_coz(pid: str, comm: str) -> str:
    """Süreç adı: cmdline'dan düğüm adını çıkar, olmazsa comm."""
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            argv = f.read().decode("utf-8", "replace").split("\0")
    except OSError:
        return comm
    for a in argv:
        for kok in ("girdap_decision/lib/girdap_decision/",
                    "girdap_ida_algi/lib/girdap_ida_algi/"):
            if kok in a:
                return a.rsplit("/", 1)[-1]
        if a.endswith("mavros_node"):
            return "mavros_node"
    if "ros2" in comm and any("bag" in a for a in argv):
        return "rosbag2_record"
    return comm


def _bizimkiler():
    """İzlenecek PID'ler + (varsa) tek süreçte çok düğüm uyarısı."""
    bulunan, kapsayici = {}, []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cl = f.read().decode("utf-8", "replace")
        except OSError:
            continue
        if not any(d in cl for d in _DESENLER):
            continue
        if "component_container" in cl:
            kapsayici.append(pid)
        st = _oku_stat(pid)
        if st:
            bulunan[pid] = _ad_coz(pid, st[2])
    return bulunan, kapsayici


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sure", type=float, default=30.0,
                    help="toplam ölçüm süresi sn (0 = süresiz, Ctrl-C ile bitir)")
    ap.add_argument("--aralik", type=float, default=2.0, help="örnekleme aralığı sn")
    ap.add_argument("--csv", help="satır satır CSV yolu")
    a = ap.parse_args()

    cek = os.cpu_count() or 1
    izlenen, kapsayici = _bizimkiler()
    if not izlenen:
        print("🔴 GIRDAP süreci bulunamadı — yığın koşuyor mu? "
              "(systemctl is-active girdap-karar girdap-algi)", file=sys.stderr)
        return 1
    if kapsayici:
        print(f"⚠️  component_container tespit edildi (pid {kapsayici}) — o süreçteki "
              "düğümler TEK satırda görünür, düğüm bazlı atıf GEÇERSİZDİR.",
              file=sys.stderr)

    print(f"çekirdek: {cek} · izlenen süreç: {len(izlenen)} · "
          f"aralık: {a.aralik} sn · süre: {a.sure or '∞'} sn")
    print("(yüzdeler TEK çekirdek cinsinden; 'sistem' satırı 6 çekirdeğin "
          "toplam meşguliyeti)\n")

    csv = open(a.csv, "w", buffering=1) if a.csv else None
    if csv:
        csv.write("zaman,pid,ad,cpu_yuzde_tek_cekirdek,rss_mb\n")

    onceki = {p: _oku_stat(p) for p in izlenen}
    sis_onceki = _oku_sistem()
    toplam_ort = defaultdict(list)
    sistem_ort, tutarsiz = [], 0
    dur = False

    def _bitir(*_):
        nonlocal dur
        dur = True
    signal.signal(signal.SIGINT, _bitir)

    t0 = time.time()
    while not dur and (a.sure <= 0 or time.time() - t0 < a.sure):
        time.sleep(a.aralik)
        simdi = time.time()
        sis_simdi = _oku_sistem()
        sistem_yuzde = None
        if None not in sis_onceki and None not in sis_simdi:
            dt = sis_simdi[1] - sis_onceki[1]
            if dt > 0:
                sistem_yuzde = 100.0 * (sis_simdi[0] - sis_onceki[0]) / dt
        sis_onceki = sis_simdi

        satirlar, toplam = [], 0.0
        for pid, ad in list(izlenen.items()):
            yeni = _oku_stat(pid)
            eski = onceki.get(pid)
            if not yeni or not eski:
                onceki[pid] = yeni
                continue
            tik = yeni[0] - eski[0]
            onceki[pid] = yeni
            yuzde = 100.0 * (tik / _SAAT_TIK) / a.aralik      # tek çekirdek cinsi
            rss = yeni[1] / 1024.0
            satirlar.append((yuzde, ad, pid, rss))
            toplam += yuzde
            toplam_ort[ad].append(yuzde)
            if csv:
                csv.write(f"{simdi:.1f},{pid},{ad},{yuzde:.2f},{rss:.1f}\n")

        # 🔬 ARACIN KENDİ DOĞRULAMASI — ⚠ BİRİM TUZAĞI (17.08'de bu araç
        # kendi kendini yakaladı): süreç yüzdeleri **TEK ÇEKİRDEK** cinsinden
        # (%100 = bir çekirdek dolu), `/proc/stat` toplamı ise **TÜM
        # ÇEKİRDEKLER** cinsinden (%100 = altısı birden dolu). İlk sürümde
        # ikisi doğrudan kıyaslanıyordu ve her turda sahte "TUTARSIZ" yanıyordu
        # (25,9 ↔ 7,8). Kıyas TEK BİRİMDE yapılır: süreç toplamı çekirdek
        # sayısına bölünür.
        damga = ""
        toplam_tum_cek = toplam / cek
        if sistem_yuzde is not None:
            sistem_ort.append(sistem_yuzde)
            if toplam_tum_cek > sistem_yuzde + 5.0:
                damga = "  ⚠ TUTARSIZ (süreç toplamı > sistem)"
                tutarsiz += 1
        satirlar.sort(reverse=True)
        ust = " · ".join(f"{ad} {y:.0f}%" for y, ad, _, _ in satirlar[:4])
        print(f"[{simdi - t0:6.1f}s] GIRDAP toplam {toplam:5.1f}% "
              f"({toplam / cek:4.1f}% × {cek} çk) | sistem "
              f"{sistem_yuzde if sistem_yuzde is not None else float('nan'):5.1f}% "
              f"| {ust}{damga}")

    if csv:
        csv.close()

    print("\n" + "=" * 70)
    print(f"{'düğüm':<28}{'ort %':>9}{'maks %':>9}{'örnek':>8}")
    print("=" * 70)
    genel = 0.0
    for ad, v in sorted(toplam_ort.items(), key=lambda kv: -sum(kv[1]) / len(kv[1])):
        ort = sum(v) / len(v)
        genel += ort
        print(f"{ad:<28}{ort:>9.1f}{max(v):>9.1f}{len(v):>8}")
    print("-" * 70)
    print(f"{'GIRDAP TOPLAM (tek çk)':<28}{genel:>9.1f}"
          f"   = {genel / cek:.1f}% × {cek} çekirdek")
    if sistem_ort:
        sis = sum(sistem_ort) / len(sistem_ort)
        # ⚠ ikisi de TÜM ÇEKİRDEK cinsine getirilerek kıyaslanır (birim tuzağı).
        bizim = genel / cek
        print(f"{'  → tüm çekirdek cinsinden':<28}{bizim:>9.1f}")
        print(f"{'SİSTEM GENELİ (/proc/stat)':<28}{sis:>9.1f}")
        print(f"{'BİZİM DIŞIMIZDA':<28}{max(0.0, sis - bizim):>9.1f}"
              "   ← büyükse yük GIRDAP dışında (çekirdek/sürücü/IO)")
        if sis > 0:
            print(f"{'GIRDAP payı':<28}{100 * bizim / sis:>8.0f}%"
                  "   ← sistem meşguliyetinin ne kadarı bizim")
    if tutarsiz:
        print(f"\n⚠️  {tutarsiz} turda süreç toplamı sistemi aştı — sayım şüpheli, "
              "sonucu tek başına raporlama.")
    print("\nℹ️  Bu ölçüm YALNIZ koştuğu andaki yükü gösterir. Sensörler "
          "(OAK/LiDAR/Pixhawk) takılı DEĞİLSE sonuç saha yükünü TEMSİL ETMEZ.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
