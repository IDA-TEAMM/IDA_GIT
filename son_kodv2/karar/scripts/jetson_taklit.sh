#!/bin/bash
# JETSON TAKLİDİ — gölü Orin Nano'nun KAYNAK ZARFINDA koşturur.
#
# 🔴 NEDEN: yapay göl bu masaüstünde (i7-13620H, 16 iş parçacığı, 4,9 GHz)
# koşuyor. Sahadaki hedef **Jetson Orin Nano 8GB Super**: 6× Cortex-A78AE
# @1,5 GHz · 8 GB LPDDR5 **CPU+GPU PAYLAŞIMLI** · 64 GB/s.
# Göl bu farkı üretmezse "10 Hz kontrol" varsayımı sahada çöker ve göl bunu
# HABER VERMEZ — KAR-11'in tam olarak yaşandığı sınıf.
#
# ── ÇAPA: UYDURMA DEĞİL, ÖLÇÜLEN İKİ DEĞERİN ORANI ────────────────────────
#   gerçek Jetson, oturum başlangıcı (temiz durum, `hatalar/karar.md` KAR-11):
#       117 · 111 · 145 · 141 ms   → ortalama 128 ms
#   bu makine, AYNI iş yükü (K=1000, T=50, PARKUR2 sahnesi):  51,2 ms
#   ⇒ ORAN = 2,51×
# Spec tabanlı kaba tahmin (~3-5×) ile tutarlı; ama tahmin yerine ÖLÇÜM
# kullanıldı çünkü araştırma net: "ISA tek başına belirleyici değil,
# performans iş yüküne bağlı".
#
# ⚠ SINIR (abartma): bu taklit CPU zarfını üretir, GPU'yu ETMEZ.
#   · MPPI'nin CUDA yolu burada sınanmaz (cupy zaten yok, numpy koşuyor)
#   · VPU (kamera NN) hiç yok — o zaten OAK-D'nin içinde
#   · Bellek BANT GENİŞLİĞİ (64 GB/s) taklit edilmez, yalnız KAPASİTE
#   Gerçek yük sadakati ancak Jetson'ın kendisinde ölçülür; bu, ona en yakın
#   ucuz yaklaşımdır ve "hiç sınamamak"tan iyidir.
#
# Kullanım:
#     bash jetson_taklit.sh gol_kos.sh 4 12.0 4.0 2
#     GIRDAP_JETSON_KAT=3.0 bash jetson_taklit.sh ...   (elle oran)
# 🪤 `set -e` KULLANILMIYOR (18.08): bu sarmalayıcı `pgrep`/`kill`/`taskset`
# gibi "bulamadım" durumunda SIFIRDAN FARKLI dönen komutlar çağırıyor.
# `set -e` ile betik hiç çıktı vermeden ölüyordu ve sebebi görünmüyordu —
# tam da gölün ayıklamak istediği "sessiz ölüm" sınıfı.
S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

KAT="${GIRDAP_JETSON_KAT:-2.51}"     # ölçülen oran (yukarıdaki çapa)
CEKIRDEK="${GIRDAP_JETSON_CEKIRDEK:-6}"   # Orin Nano: 6 çekirdek
BELLEK_MB="${GIRDAP_JETSON_BELLEK_MB:-8192}"  # 8 GB PAYLAŞIMLI

# ── 1) ÇEKİRDEK KISITI — `taskset` ile 6 çekirdeğe hapset ────────────────
# Orin Nano'da 6 çekirdeği TÜM düğümler paylaşır. Burada 16 iş parçacığı
# varken ölçüm iyimser çıkar: 16.08 ölçümü, arka planda yük varken aynı
# adımın 85,8 → 119 ms (maks 321) çıktığını göstermişti.
TOPLAM=$(nproc)
if [ "$CEKIRDEK" -lt "$TOPLAM" ]; then
    CPU_SET="0-$((CEKIRDEK - 1))"
    ONEK="taskset -c $CPU_SET"
else
    ONEK=""
fi

# ── 2) CPU YAVAŞLATMA — `cpulimit` yoksa yük üreticisiyle ────────────────
# Hedef: her çekirdeğin etkin hızını ~1/KAT'a indirmek. `cpulimit` varsa
# doğrudan sürece uygulanır; yoksa rakip yük (spin) ile aynı etki üretilir.
# ⚠ Rakip yük yöntemi GÜRÜLTÜLÜDÜR (zamanlayıcıya bağlı) — ölçüm alırken
# tercih `cpulimit`tir.
# 🪤 TRAP TASARIM KUSURU (18.08, ölçümle bulundu):
# `gol_kos.sh` düğümleri ARKA PLANDA başlatıp HEMEN dönüyor. `trap ... EXIT`
# ile rakip yükü sarmalayıcı çıkarken öldürüyorduk ⇒ göl ayaktayken ZARF
# KAYBOLUYORDU. Ölçüldü: 14 düğüm koşuyor ama yük süreci 0 — yani "Jetson
# zarfında koştu" sanılan koşum aslında MASAÜSTÜ hızındaydı. Sessiz ve
# tamamen yanıltıcı bir ölçüm; gölün ayıklamak istediği sınıfın ta kendisi.
#
# Doğrusu: yük gölün ömrü boyunca yaşar ve `jetson_yuk_dur.sh` ile ölür.
# PID'ler göl log dizinine yazılır. INT/TERM'de (elle Ctrl-C) yine temizlenir
# — yalnız EXIT'te DEĞİL.
YUK_PIDS=()
YUK_DOSYA="${GIRDAP_GOL_LOG:-$HOME/girdap_logs/gol}/jetson_yuk.pids"
temizle() {
    for p in "${YUK_PIDS[@]}"; do kill "$p" 2>/dev/null; done
    rm -f "$YUK_DOSYA"
}
trap temizle INT TERM

if [ "${GIRDAP_JETSON_YUK:-1}" = "1" ]; then
    # 🔴 RAKİP YÜK SAYISI KALİBRE EDİLDİ (ölçümle, formülle DEĞİL).
    # İlk sürüm `N = çekirdek × (1 − 1/KAT)` diyordu ⇒ N=4 ve ölçülen adım
    # yalnız **65,4 ms** çıktı (hedef 128). Formül yanlıştı: rakip süreçler
    # çekirdekleri PAYLAŞIYOR, tam işgal etmiyor.
    # Kalibrasyon (aynı iş yükü, 6 çekirdeğe hapsedilmiş):
    #     N=6  →  74,1 ms
    #     N=10 → 133,8 ms   ← gerçek Jetson 128 ms'e EN YAKIN  ✅
    #     N=14 → 173,8 ms
    # ⇒ N ≈ çekirdek × (KAT − 1) / 0,9   (N=10 için 2,51 → 10,1)
    ORAN=$(python3 -c "print(max(0.0, min(0.95, 1 - 1/$KAT)))")
    N=$(python3 -c "print(max(1, round($CEKIRDEK * ($KAT - 1) / 0.9)))")
    for _ in $(seq "$N"); do
        $ONEK python3 -c "
while True: pass" >/dev/null 2>&1 &
        YUK_PIDS+=($!)
    done
    mkdir -p "$(dirname "$YUK_DOSYA")"
    printf "%s\n" "${YUK_PIDS[@]}" > "$YUK_DOSYA"
    echo "  🔧 rakip yük: $N çekirdek meşgul (oran %$(python3 -c "print(int($ORAN*100))"))"
fi

# ── 3) BELLEK — 🔴 TAVAN UYGULANMIYOR, SEBEBİ ÖLÇÜLDÜ ────────────────────
# İlk sürüm `ulimit -v $((BELLEK_MB*1024))` koyuyordu. ÇALIŞMADI: kabuk ve
# alt süreçler **exit 144** ile ölüyordu.
# Kök neden (ölçüldü): `ulimit -v` **SANAL adres alanını** (VSZ) sınırlar,
# fiziksel kullanımı değil. numpy/DDS/CUDA büyük sanal alan ayırır ama
# çoğunu KULLANMAZ ⇒ 8 GB VSZ tavanı gerçek 8 GB RAM'i taklit ETMEZ,
# yalnız süreci erken öldürür. Yanlış araç.
#
# Doğru araçlar (bu makinede kurulu değil, saha için not):
#   · cgroup v2 `memory.max` — GERÇEK fiziksel tavan, OOM killer ile
#   · `systemd-run --scope -p MemoryMax=8G`
#   · en dürüstü: Jetson'ın kendisinde koşturmak
#
# ⚠ Bu yüzden bellek zarfı ŞU AN TAKLİT EDİLMİYOR. MPPI'nin engel tensörü
# N=2000'de 1,6 GB (F-M.2) ⇒ burada sığan, Orin'de OOM olabilir ve bu göl
# onu YAKALAMAZ. Kural motorundaki R1 (kaynak tükenmesi) hâlâ yazılmadı.
if [ "${GIRDAP_JETSON_BELLEK:-0}" = "1" ]; then
    echo "  ⚠ bellek tavanı istendi ama ulimit -v YANLIŞ ARAÇ (bkz. yorum)"
fi

echo "🔵 JETSON TAKLİDİ: ${CEKIRDEK} çekirdek · yavaşlatma ${KAT}× · bellek ${BELLEK_MB} MB"
echo "   (çapa: gerçek Jetson 128 ms ↔ bu makine 51,2 ms, KAR-11 ölçümü)"

KOMUT="$1"; shift
$ONEK bash "$S/$KOMUT" "$@"
DURUM=$?
echo "  ⏹  zarfı kaldırmak için: bash $S/jetson_yuk_dur.sh"
exit $DURUM
