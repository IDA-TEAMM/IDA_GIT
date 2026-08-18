"""SANAL GÖL artıklarını YAŞA göre temizle — canlı yığına dokunmadan.

🔴 Neden gerekli: `ros2 run girdap_decision X` bir sarmalayıcı süreç açar,
asıl düğüm ise `.../lib/girdap_decision/X` olarak koşar — yani komut satırı
CANLI YIĞINLA AYNI görünür. Sarmalayıcıyı öldürmek çocuğu öldürmez; kalan
çocuk domain 77'de hayalet düğüm olarak yaşar (ölçüldü: 4 kopya `fsm_node`,
FSM'in KAR-01 bekçisi "3 yayıncı" diye bağırdı ve görev durumu KILL'e titredi).

Ayrım ölçütü UYDURMA DEĞİL: canlı yığın saatlerdir koşuyor, sim düğümleri
dakikalar önce açıldı. `esik_s`'ten GENÇ olanlar sim artığıdır.

🔴🔴 ATA KORUMASI (13.08, ÜÇÜNCÜ kez yaşandıktan sonra eklendi): kalıbı arayan
komutun KENDİ komut satırı da kalıbı içerebilir — ör. `grep gol_log/sanal_gol.log`
yazan bir kabuk "sanal_gol" kalıbına uyar ve bu betik onu öldürür. Bu oturumda
kabuk üç kez böyle düştü. Artık kendi PID'i VE bütün ata zinciri (bash, ros2,
sarmalayıcılar) atlanır.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys

ESIK_S = 1800          # 30 dk: canlı yığın bundan çok daha yaşlı


#: Göle ait süreçleri tanıyan desenler. Yeni bir sahte düğüm eklenirse
#: BURAYA da eklenmeli — yoksa `gol_dur.sh` onu öldüremez ve hayalet kalır.
#: `test_gol_temizleyici_TUM_gol_dugumlerini_taniyor` bu listeyi göl
#: betiğiyle bağlar; ayrışırsa CI kırmızı.
# 🔴 GÖLE YENİ DÜĞÜM EKLEYEN BUNU DA GÜNCELLESİN. Aksi hâlde düğüm
# `gol_dur.sh`/`gol_temizle.py` sonrası HAYATTA KALIR ve bir sonraki koşumda
# İKİNCİ bir yayıncı olarak görünür. Ölçüldü (18.08): `duba_gecis_navigator`
# eklendi ama buraya yazılmadı ⇒ `/perception/buoys` yayıncı **2** oldu,
# ölçülen hız 4,94 yerine **9,88 Hz** çıktı ve ms ölçümü anlamsızlaştı.
# (Aynı sınıf 13.08'de `sahte_ham_sensor` ile de yaşandı.)
_GOL_DESENLERI = ("girdap_decision", "sanal_gol", "sahte_ham_sensor",
                  "girdap_ida_algi", "duba_gecis_navigator")


def _ata_zinciri() -> set[int]:
    """Kendi PID'i + bütün ataları (init'e kadar) — bunlara ASLA dokunulmaz."""
    korunan, pid = set(), os.getpid()
    while pid and pid not in korunan:
        korunan.add(pid)
        try:
            with open(f"/proc/{pid}/stat", encoding="utf-8") as f:
                # ppid, komut adı parantez içerdiği için sondan ayrıştırılır
                pid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    return korunan


def main() -> None:
    esik = int(sys.argv[1]) if len(sys.argv) > 1 else ESIK_S
    dokunma = _ata_zinciri()
    cikti = subprocess.run(
        ["ps", "-eo", "pid,etimes,args"], capture_output=True, text=True
    ).stdout.splitlines()
    oldurulen, korunan = [], []
    for satir in cikti[1:]:
        parca = satir.split(None, 2)
        if len(parca) < 3:
            continue
        pid, yas, cmd = int(parca[0]), int(parca[1]), parca[2]
        # 🔴 18.08: eskiden yalnız `girdap_decision` ve `sanal_gol` aranıyordu.
        # `sahte_ham_sensor.py` (ve gelecekte eklenecek her `scripts/*.py`
        # sahte düğümü) bu listeye GİRMİYORDU ⇒ hiç öldürülmüyordu.
        # ÖLÇÜLDÜ: ardışık göl koşumlarından **6 kopya** birikti, en eskisi
        # 26 dakikalık. Aynı topic'e altı üretici basıyordu — hangi verinin
        # kimden geldiği belirsizleşir ve ölçüm sessizce anlamsızlaşır.
        # (Betiğin kendi docstring'i tam bu sınıfı uyarıyordu: "hayalet düğüm".)
        if not any(d in cmd for d in _GOL_DESENLERI):
            continue
        if pid in dokunma:                      # kendi süreç zincirim
            continue
        if yas < esik:
            oldurulen.append((pid, yas, cmd[:60]))
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        else:
            korunan.append((pid, yas))
    print(f"öldürülen (sim artığı, <{esik} s): {len(oldurulen)}")
    print(f"korunan (canlı yığın, >{esik} s): {len(korunan)} süreç")


if __name__ == "__main__":
    main()
