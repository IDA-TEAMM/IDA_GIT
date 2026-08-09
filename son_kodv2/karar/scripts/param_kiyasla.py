#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""İki ArduPilot `.param` dökümünü kıyaslar ve KRİTİK sapmaları öne çıkarır.

    python3 param_kiyasla.py <temel.param> <yeni.param>

NEDEN VAR: 2026-08-09'da FC parametreleri **başkası tarafından tamamen
değiştirildi**. 928 satırı gözle kıyaslamak imkânsız; üstelik önemli olan
"kaç satır değişti" değil, **hangilerinin** değiştiği. Bazı parametrelerin
yanlış değeri SESSİZDİR ve yalnızca suda ya da yarışma günü anlaşılır.

Bu script kritik listeyi ayrı raporluyor. Liste, daha önce ölçümle/testle
kazanılmış bilgiden geliyor — her satırın yanında **bozulursa ne olur**
yazıyor, çünkü "eski değere dön" demek yetmez, neden döndüğü bilinmeli.
"""
from __future__ import annotations

import sys
from pathlib import Path

#: parametre -> (beklenen_deger_ya_da_None, bozulursa_ne_olur)
#: None = "sabit bir doğrusu yok ama DEĞİŞTİYSE bilmek isteriz".
KRITIK: dict[str, tuple[str | None, str]] = {
    # --- Emniyet: bunların yanlışı can/puan kaybı ---
    "ARMING_REQUIRE": ("1", "0 ise DISARM HIC CALISMAZ -> yazilimin KILL yolu "
                            "(mavros_bridge disarm+sifir thrust) olu; aylarca "
                            "fark edilmedi (md 4.2 acil durdurma)"),
    "BRD_SAFETY_DEFLT": ("0", "1 ise safety switch beklenir; 0 secildi ama "
                              "tekne sessizce motor calistirabilir -> buzzer sart"),
    "MOT_SAFE_DISARM": ("0", "disarm'da PWM davranisi degisir"),
    "MOT_THR_MIN": ("10", "olu bant; buyurse motorlar bosta doner"),
    "ARMING_CHECK": ("0", "bench icin 0'landi; sahada acilmasi TARTISILMALI"),
    "ARMING_RUDDER": ("2", "kumandadan arm/disarm yolu"),
    # --- Tahrik: yanlisi ters/asimetrik itki ---
    "FRAME_CLASS": ("2", "2=Boat. Baska deger skid-steering mixing'i bozar"),
    "SERVO1_FUNCTION": ("74", "74=ThrottleRight. Karisirsa tekne ters doner"),
    "SERVO3_FUNCTION": ("73", "73=ThrottleLeft"),
    "SERVO2_FUNCTION": ("0", "bu teknede 2 motor var; 0 olmali"),
    "SERVO4_FUNCTION": ("0", "ayni"),
    "SERVO1_MIN": ("1000", "cift yonlu ESC: min = TAM GERI, 'motor durur' DEGIL"),
    "SERVO1_MAX": ("2000", ""),
    "SERVO1_TRIM": ("1487", "sifir itki. Kayarsa tekne kendiliginden surer"),
    "SERVO1_REVERSED": ("0", ""),
    "SERVO3_MIN": ("1000", ""),
    "SERVO3_MAX": ("2000", ""),
    "SERVO3_TRIM": ("1487", "sol/sag simetrisi 125 orneklemde 0 fark olculdu"),
    "SERVO3_REVERSED": ("0", ""),
    # --- Saat: yeni saat servisi bunlara BAGIMLI (09.08) ---
    "BRD_RTC_TYPES": ("1", "1=YALNIZ GPS. 2/3 olursa Jetson (26 saat geride!) "
                           "FC'nin saatini ve .BIN log damgalarini BOZAR"),
    "SR2_EXTRA3": ("10", "TELEM2'de SYSTEM_TIME bu grupta akiyor. 0 olursa "
                         "girdap-saat GPS saatini HIC alamaz -> teslim "
                         "damgalari guvenilmez kalir (md 4.2, 5 ceza puani)"),
    # --- Buzzer: sessiz tekne tehlikeli ---
    "NTF_BUZZ_TYPES": ("5", "0 ise tekne sessizce armed olur"),
    "NTF_BUZZ_VOLUME": ("100", ""),
    # --- Konum ofsetleri: A-3'te girilecek, su an 0 ---
    "INS_POS1_X": (None, "A-3: -0.055 girilecek"),
    "INS_POS1_Y": (None, "A-3: -0.1375"),
    "INS_POS1_Z": (None, "A-3: -0.155"),
    "GPS1_POS_X": (None, "A-3: -0.035"),
    "GPS1_POS_Y": (None, "A-3: -0.16"),
    "GPS1_POS_Z": (None, "A-3: -0.365"),
    # --- Mod / RC ---
    "MODE_CH": ("8", "mod anahtari kanali"),
    "RC10_OPTION": (None, "31=Motor E-Stop ama 9 kanalli R9DS'te ERISILEMEZ"),
    "RC7_OPTION": (None, "gecersiz 2 degerinden 0'a cekilmisti"),
    # --- Tuning: Yahya tezgahta dusurdu, suda dogrulanacak ---
    "ATC_STR_RAT_P": (None, "Yahya tezgahta dusurdu; suda dogrulanmadi"),
    "ATC_STR_RAT_D": (None, "ayni"),
    "CRUISE_SPEED": (None, "gercek seyir 1,05 m/s olculdu"),
    "WP_SPEED": (None, "telemetry.fc_cruise_setpoint_mps ile SENKRON olmali"),
    # --- Batarya ---
    "BATT_CAPACITY": (None, "3300 yaziyordu, gercek ~35000 mAh (acik konu)"),
    "BATT_LOW_VOLT": (None, ""),
    "BATT_CRT_VOLT": (None, ""),
    "BATT_MONITOR": (None, "0 olursa voltaj/akim okunmaz, failsafe olu"),
    "FS_THR_ENABLE": (None, "RC kayip failsafe"),
}


def oku(yol: Path) -> dict[str, str]:
    d: dict[str, str] = {}
    for satir in yol.read_text(errors="replace").splitlines():
        satir = satir.strip()
        if not satir or satir.startswith("#"):
            continue
        for ayirici in (",", "\t", " "):
            if ayirici in satir:
                ad, _, deger = satir.partition(ayirici)
                d[ad.strip()] = deger.strip()
                break
    return d


def _esit(a: str, b: str) -> bool:
    try:
        return abs(float(a) - float(b)) < 1e-9
    except ValueError:
        return a == b


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    t_yol, y_yol = Path(sys.argv[1]), Path(sys.argv[2])
    temel, yeni = oku(t_yol), oku(y_yol)

    print(f"TEMEL : {t_yol.name}  ({len(temel)} parametre)")
    print(f"YENI  : {y_yol.name}  ({len(yeni)} parametre)")

    degisen = {k: (temel[k], yeni[k]) for k in temel.keys() & yeni.keys()
               if not _esit(temel[k], yeni[k])}
    silinen = sorted(temel.keys() - yeni.keys())
    eklenen = sorted(yeni.keys() - temel.keys())

    print(f"\nDEGISEN: {len(degisen)} · yeni dosyada YOK: {len(silinen)} · "
          f"YENI: {len(eklenen)}")

    # 1) Kritik sapmalar — ONCE bunlar.
    kritik_sapma = {k: v for k, v in degisen.items() if k in KRITIK}
    print("\n" + "=" * 78)
    if kritik_sapma:
        print(f"🔴 KRITIK SAPMA: {len(kritik_sapma)}")
        print("=" * 78)
        for k in sorted(kritik_sapma):
            eski, yni = kritik_sapma[k]
            beklenen, neden = KRITIK[k]
            isaret = ""
            if beklenen is not None:
                isaret = ("  ✅ beklenen deger KORUNDU"
                          if _esit(yni, beklenen)
                          else f"  ❌ BEKLENEN {beklenen} — SU AN {yni}")
            print(f"\n  {k}: {eski} -> {yni}{isaret}")
            if neden:
                print(f"      bozulursa: {neden}")
    else:
        print("✅ KRITIK LISTEDE HIC SAPMA YOK")
        print("=" * 78)

    # 2) Kritik listede olup DEGISMEYENLER de teyit edilmeli: dosyada hic
    #    yoksa parametre kaybolmus olabilir (firmware/frame degisikligi).
    eksik_kritik = [k for k in KRITIK if k in temel and k not in yeni]
    if eksik_kritik:
        print(f"\n⚠️  KRITIK ama yeni dokumde YOK ({len(eksik_kritik)}): "
              f"{', '.join(eksik_kritik)}")
        print("    (firmware/FRAME_CLASS degisikligi parametreyi yok edebilir)")

    # 3) Kalan degisiklikler — gozden gecirilecek.
    diger = {k: v for k, v in degisen.items() if k not in KRITIK}
    if diger:
        print(f"\n{'-' * 78}\nDIGER DEGISIKLIKLER ({len(diger)}) — gozden gecir:")
        for k in sorted(diger):
            print(f"  {k}: {diger[k][0]} -> {diger[k][1]}")
    if silinen:
        print(f"\nYeni dokumde OLMAYANLAR ({len(silinen)}):")
        print("  " + ", ".join(silinen))
    if eklenen:
        print(f"\nYalniz yeni dokumde OLANLAR ({len(eklenen)}):")
        print("  " + ", ".join(eklenen))
    return 0


if __name__ == "__main__":
    sys.exit(main())
