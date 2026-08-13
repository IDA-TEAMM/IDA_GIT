"""FC parametre öz-denetimi — ÖLÜMCÜL sapmaları bul, gerisini sus.

🔴 **Neden var.** Parametreleri belirlemek takımda BAŞKASININ görevi ve her
testten sonra güncelleniyor — bizim kodumuzdan bağımsız olarak. 13.08'de
Pixhawk'a bağlanıldığında **39 parametre** değişmiş bulundu; ölçülmüş IMU
konumlarımız sıfırlanmış, batarya izleme kapatılmış, failsafe eylemi
kaldırılmıştı. Farkı elle ayıklamak yarım saat sürdü.

**Kodumuz zaten parametreden bağımsız** — hiçbir node FC parametresi okumaz,
hepsi topic üzerinden çalışır. Sorun bağımlılık değil, **sessizlik**: bir
parametre bozulduğunda bunu ancak sahada, arıza olarak öğreniyorduk.

⇒ Bu modül bağımsızlığı değil **görünürlüğü** sağlar.

## Neden yalnız "ölümcül"

Uyarı listesi uzarsa okunmaz. 900+ parametrenin çoğu bizi hiç ilgilendirmiyor
(`WP_SPEED`, `CRUISE_SPEED`, `ATC_*` — biz AUTO kullanmıyoruz, GUIDED'da
`setpoint_velocity` ile sürüyoruz). Burada YALNIZCA şu ölçütü geçen
parametreler var:

    "Bu değer yanlışken görev başarısız olur ya da tekne güvensiz hale gelir,
     ve bunu SAHADA fark etmeyiz."

`LOG_*` bilerek DIŞARIDA: yanlışsa yalnız teşhis kaybederiz, görev yürür.

## Bağlantı parametreleri neden listede yok

`SERIAL*_BAUD` gibi hat ayarları en ölümcül olanlar — ama onlar yanlışsa
MAVROS zaten hiç bağlanmaz ve bu denetim de koşamaz. Yani hattın kendisi
kendi kanıtıdır; ayrıca kontrol etmek gereksiz.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass(frozen=True)
class Beklenti:
    """Bir parametre için beklenen değer ve YANLIŞSA NE OLUR."""

    deger: float
    sebep: str
    #: `True` → "EN AZ bu kadar olmalı"; fazlası sorun DEĞİL.
    #: Akış hızları için: fazla veri zarar vermez, az veri bekçileri aç bırakır.
    #: ⚠ İlk yazımda bunu `tolerans` payıyla ifade etmeye çalıştım ve anlamı
    #: bulanıklaştı — 5 Hz beklerken tolerans 4,9 vermek "0,1 Hz de kabul"
    #: demek oluyordu, yani PAR-04'ün ta kendisini kaçırırdı. Testim yakaladı.
    asgari: bool = False


#: 🔒 ÖLÜMCÜL LİSTE. Buraya ekleme yaparken tek ölçüt: "yanlışken görev
#: başarısız olur ya da tekne güvensizleşir VE sahada fark edilmez."
#: Değerlerin çoğu BİZİM ÖLÇÜMÜMÜZ; parametre sorumlusu değiştirirse
#: bilerek mi değiştirdiğini sormak için buradalar.
OLUMCUL: Dict[str, Beklenti] = {
    # --- Kestirim kalitesi: sıfırlanırsa odometri sessizce bozulur -------
    "INS_POS1_X": Beklenti(-0.055, "IMU konumu ÖLÇÜLDÜ; sıfırsa EKF yanlış kol mesafesi kullanır"),
    "INS_POS1_Y": Beklenti(-0.1375, "IMU konumu ÖLÇÜLDÜ (Pixhawk 13,75 cm iskeleye kayık)"),
    "INS_POS1_Z": Beklenti(-0.155, "IMU konumu ÖLÇÜLDÜ"),
    # --- Güvenlik: yanlışken tekne bozuk kestirimle sürmeye devam eder ---
    "FS_ACTION": Beklenti(2.0, "0 = failsafe'te HİÇBİR ŞEY yapma; EKF bozulunca motorlar komut almaya devam eder"),
    "ARMING_CHECK": Beklenti(1.0, "0 = tüm ön kontroller kapalı; tekne bozuk kestirimle ARM olur ve her şey yeşil görünür (§0.41)"),
    "BATT_MONITOR": Beklenti(3.0, "0 = batarya izleme YOK, düşük voltaj failsafe'i çalışmaz (akım kanalı PM06'da ölü, o yüzden 3)"),
    # --- Akış hızı: düşükse bekçilerimiz aç kalır ve SAHTE KILL üretir ---
    "SR2_EXT_STAT": Beklenti(5.0, "PAR-04: /mavros/state 0,17 Hz'e düşünce heartbeat eşiği her aralıkta aşıldı, oturumun %86'sı KILL'de geçti", asgari=True),
    "SR2_EXTRA1": Beklenti(10.0, "IMU/attitude akışı; düşükse füzyon aç kalır", asgari=True),
    "SR2_POSITION": Beklenti(5.0, "konum akışı; düşükse planlama bayat pozla çalışır", asgari=True),
}


@dataclass(frozen=True)
class Bulgu:
    ad: str
    beklenen: float
    okunan: Optional[float]
    sebep: str

    def __str__(self) -> str:
        o = "OKUNAMADI" if self.okunan is None else f"{self.okunan:g}"
        return f"{self.ad}: {o} (beklenen {self.beklenen:g}) — {self.sebep}"


def denetle(okunan: Dict[str, Optional[float]]) -> List[Bulgu]:
    """Okunan parametreleri beklentiyle karşılaştır, YALNIZ sapanları döndür.

    `asgari=True` olan parametrelerde yalnız ALTINA düşmek sapmadır (akış
    hızları); diğerlerinde her sapma bulgudur.
    """
    bulgular: List[Bulgu] = []
    for ad, b in OLUMCUL.items():
        v = okunan.get(ad)
        if v is None:
            bulgular.append(Bulgu(ad, b.deger, None, b.sebep + " [parametre okunamadı]"))
            continue
        if b.asgari:
            if v < b.deger:
                bulgular.append(Bulgu(ad, b.deger, v, b.sebep))
        elif abs(v - b.deger) > 1e-6:
            bulgular.append(Bulgu(ad, b.deger, v, b.sebep))
    return bulgular


def ozet(bulgular: List[Bulgu]) -> str:
    """Mission Planner STATUSTEXT'e sığacak tek satır (50 karakter sınırı)."""
    if not bulgular:
        return "GIRDAP FC PARAM OK"
    return f"GIRDAP FC PARAM SAPMA x{len(bulgular)}: {bulgular[0].ad}"[:50]
