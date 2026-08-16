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
    "GPS1_POS_X": Beklenti(-0.035, "GPS anteni konumu ÖLÇÜLDÜ; sıfır/ters ise EKF kol mesafesini yanlış kullanır"),
    "GPS1_POS_Y": Beklenti(-0.16, "GPS anteni ÖLÇÜLDÜ −0.16 (iskele). 13.08'de FC'de +0.16 bulundu — İŞARET TERS, 32 cm'lik yanlış kol"),
    "GPS1_POS_Z": Beklenti(-0.365, "GPS anteni konumu ÖLÇÜLDÜ"),
    # 16.08 göl testi (§1.20g): 0 = "gecikmeyi sürücüye sor" demek; u-blox
    # sürücüsü modülün hardware generation'ını çözemeyince get_lag() false
    # döner ve EKF3 HİÇ BAŞLAMAZ — FC 10 sn'de bir `EKF3 waiting for GPS
    # config data` basar, LOCAL_POSITION_NED akışı komple ölür, GUIDED
    # istekleri `Flight mode change failed` ile reddedilir. Sahada 2,5 saat
    # kaybettirdi ve HİÇBİR bekçi göstermedi: GPS'in kendisi sağlıklı
    # görünüyordu (31 uydu, 3B fix, 10 Hz). 200 yazılınca EKF anında açıldı.
    "GPS1_DELAY_MS": Beklenti(200.0, "0 = otomatik; u-blox surucusu lag veremezse EKF3 HIC baslamaz, GUIDED reddedilir (§1.20g). Sabit deger surucu sorgusunu baypas eder"),
    # --- Eyleyici: 0 iken ölü bant aşılamaz, düşük komutlar motoru döndürmez
    # ⚠ TEKRARLAYAN REGRESYON: 10.08'de de 0'a düşmüş, geri yazılmıştı
    # (`fc_param_uzlasma_2026-08-10.md`); 13.08'de YİNE 0 bulundu. İkinci kez
    # olması "yanlışlıkla" açıklamasını zayıflatıyor — parametre sorumlusuna
    # NEDEN sıfırlandığı sorulmalı, yoksa üçüncü kez olur ve o sefer suda olur.
    "MOT_THR_MIN": Beklenti(10.0, "0 = olu bant telafisi yok; MPPI'nin kucuk itki komutlari motoru hic dondurmez. Deger SU TESTINDEN GECTI (10.08). 2. kez sifirlandi — sorumluya NEDEN diye sor"),
    # --- Donanım kimliği: yanlışsa araç FİZİKSEL olarak yanlış davranır ---
    # 14.08'de eklendi. Bunlar hiç değişmemesi gereken değerler; değiştiyse
    # gerçekten sorun var demektir, o yüzden yanlış alarm riski düşük.
    # Listede OLMAMALARININ bedeli iki yönlü: yakalanmıyorlardı VE referansa
    # bozuk hâlleriyle donabiliyorlardı (13.08'de GPS1_POS_Y ve MOT_THR_MIN'de
    # tam bu oldu).
    "FRAME_CLASS": Beklenti(2.0, "2 = tekne. Yanlissa ArduPilot bambaska bir arac gibi surer"),
    "SERVO1_FUNCTION": Beklenti(74.0, "ThrottleRight — sag motor. Yanlissa motorlar ters/olu"),
    "SERVO3_FUNCTION": Beklenti(73.0, "ThrottleLeft — sol motor"),
    "BATT_VOLT_MULT": Beklenti(5.091626, "PM06 KALIBRASYON degeri; varsayilana donerse voltaj yanlis okunur ve batarya failsafe'i yanlis esikte tetiklenir"),
    "SERIAL2_BAUD": Beklenti(921.0, "TELEM2 = Jetson hatti; MAVROS fcu_url ile ayni olmali, uyusmazsa MAVROS HIC baglanmaz"),
    "ARMING_REQUIRE": Beklenti(1.0, "0 = arm sarti yok"),

    # --- Güvenlik: yanlışken tekne bozuk kestirimle sürmeye devam eder ---
    "FS_ACTION": Beklenti(2.0, "0 = failsafe'te HİÇBİR ŞEY yapma; EKF bozulunca motorlar komut almaya devam eder"),
    "ARMING_CHECK": Beklenti(1.0, "0 = tüm ön kontroller kapalı; tekne bozuk kestirimle ARM olur ve her şey yeşil görünür (§0.41)"),
    # 16.08 kaptan kararı: batarya failsafe'i KAPALI kalacak. PM06 hiçbir şey
    # okumuyor (canlı ölçüm: 0,007 V / 0,01 A / %−1) → BATT_MONITOR=3 iken
    # `PreArm: Battery 1 unhealthy` arm'ı ENGELLİYORDU. Beklentiyi 0'a çekmezsek
    # bekçi her koşumda sahte ölümcül sapma bildirir (§0.96b'de öngörülmüştü).
    "BATT_MONITOR": Beklenti(0.0, "kaptan kararı: batarya izleme KAPALI (PM06 ölü, açıkken arm'ı engelliyor). 3 olursa sahte batarya-critical geri gelir"),
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
