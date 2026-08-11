"""Test izolasyonu — testler CANLI ROS domain'ine sızmasın (PAR-01 / KAR-01).

🔴 **Neden bu dosya var.** Kaptanın 14 rosbag oturumluk arıza analizi
(`son_kodv2/hatalar/parametre.md` PAR-01) testlerin canlı domaine sızdığını
sayısal olarak kanıtladı:

- `/mavros/global_position/global`'a **24.430 sahte GPS mesajı** enjekte edilmiş
  (41.0/29.0 · 40.8002/29.3 · 0/0) → füzyon zehirlenmiş, odometri ışınlanmış
  (KAR-06: 25 ms'de 6,54 m = 257 m/s).
- `/girdap/mission/state`'e test yayınları karışmış; o oturumda `PARKUR1`
  **16.982** örnekle görünüyor — testlerin yaydığı değerlerden biri (KAR-01).
  FSM'i dinleyen her düğüm saniyede 10 kez çelişen durum görmüş.

Bu, ölçümü bozmakla kalmıyor: **teşhisi de zehirliyor.** O bag'lerden çıkarılan
"otonomi çalışmıyor" sonuçlarının bir kısmı gerçek arıza değil, test artefaktı.

⚠️ **`ROS_LOCALHOST_ONLY` bu sorunu ÇÖZMEZ.** Testler çoğunlukla Jetson'ın
kendisinde koşuyor ve canlı yığın da orada — aynı makinede localhost zaten
paylaşılıyor. Tek gerçek ayrım **farklı `ROS_DOMAIN_ID`**.

⚠️ **Sıra önemli:** bu dosya modül seviyesinde ortam değişkenini yazıyor, çünkü
pytest `conftest.py`'yi test modüllerini **import etmeden önce** yükler. Fixture
içinde yazmak GEÇ olurdu: `rclpy` import edildiği anda ya da ilk `rclpy.init()`
çağrısında domain okunur ve bir daha değişmez.
"""

from __future__ import annotations

import os

#: Testlerin koşacağı domain. Canlı sistem 42 kullanıyor (bkz. CLAUDE.md ve
#: `girdap-karar.service`); burada BİLİNÇLİ olarak farklı bir değer seçildi.
#: 0-101 aralığı her platformda güvenli (ROS 2 önerisi).
VARSAYILAN_TEST_DOMAIN = "91"

#: Kasıtlı olarak canlı sisteme karşı test koşturmak isteyen biri için kaçış
#: kapısı. Ortam değişkeni verilmezse izolasyon ZORUNLUDUR — "unuttum" diye bir
#: durum olmasın.
_ORTAM_ANAHTARI = "GIRDAP_TEST_DOMAIN"

_istenen = os.environ.get(_ORTAM_ANAHTARI, VARSAYILAN_TEST_DOMAIN)
_onceki = os.environ.get("ROS_DOMAIN_ID")
os.environ["ROS_DOMAIN_ID"] = _istenen

# Ağa taşmayı da kapat: aynı domaine sahip BAŞKA bir makine (ör. laptop↔Jetson
# aynı alt ağda) varsa oraya da sızmasın. Domain ayrımı asıl koruma, bu ikinci
# katman.
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")


def pytest_report_header(config) -> str:      # noqa: ANN001
    """Koşum başlığına izolasyonu yaz — sessiz kalmasın, gözle görülsün."""
    ek = f" (canlı domain {_onceki} idi)" if _onceki and _onceki != _istenen else ""
    return (
        f"girdap: test izolasyonu AÇIK — ROS_DOMAIN_ID={_istenen}{ek} · "
        f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY')}"
    )
