"""Parkur-3 hedef rengi — ROS tarafı kapı (12'lik madde #4, md 5.5.3.1).

Çekirdek mantık `prototype/mission/kamikaze_hedef.py`'de (ROS'suz, test
edilebilir). Bu dosya onun ROS kablosu: parametre + `ros2 param set` yolu +
şartname zamanlama kapısı + yeniden etiketleme çağrısı.

**Neden ayrı bir yardımcı:** mekanizma İKİ node'da lazım.

- `perception_fusion_node` — **ASIL yer.** `/perception/buoys`'u kim üretirse
  üretsin (bizim HSV yedeği ya da algı ekibinin `girdap-ida-algi` paketi)
  füzyon onun altında ve **her zaman** koşuyor. Aşağı akışta hedefe bakan her
  şey (`planning_node`, MPPI `w_kamikaze`) `classified_obstacles`'ı okuyor.
- `perception_camera_node` — belgenin işaret ettiği yer; yalnız
  `use_onboard_camera:=true` iken koşuyor (varsayılan **false**, 2026-08-04
  algı ekibi kararı). Burada da olması bizim yedek yolun kendi başına doğru
  olmasını sağlıyor.

İki yerde koşması güvenli: yeniden etiketleme **fikir olarak idempotent** —
kamera node'u kırmızıyı 2'ye taşımışsa füzyonda taşınacak kırmızı kalmaz.
"Hedef hiç görülmedi" uyarısı bu yüzden zaten-2 olanları da sayar, yoksa
yanlış alarm verirdi.
"""

from __future__ import annotations

from typing import Iterable, Optional, Protocol

from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from prototype.mission.kamikaze_hedef import (
    CLASS_HEDEF,
    DEDEKTORU_OLAN_SINIFLAR,
    HedefRengiHatasi,
    degistirilebilir_mi,
    hedef_isaretle,
    renk_to_class,
)

_PARAM = "kamikaze_target_color"


class _Tespit(Protocol):
    class_id: int


class KamikazeHedefKapisi:
    """Hedef rengi parametresini yönetir ve tespitlere uygular.

    Kullanım (node `__init__` içinde, publisher/subscriber'lardan önce)::

        self._hedef = KamikazeHedefKapisi(self)
        ...
        self._hedef.uygula(tespitler)     # her karede
    """

    def __init__(self, node) -> None:                    # noqa: ANN001
        self._node = node
        self._log = node.get_logger()
        self._gorev_durumu: Optional[str] = None
        self._sinif: Optional[int] = None
        self._renk_adi = ""
        self._gorulmedi_uyarildi = False

        node.declare_parameter(_PARAM, "")
        ham = str(node.get_parameter(_PARAM).value)
        try:
            self._sinif = renk_to_class(ham)
            self._renk_adi = ham.strip()
        except HedefRengiHatasi as exc:
            # Açılışta geçersiz renk → node ÖLMEZ. Parkur-1/2 hedef rengine
            # bağlı değil; onları da düşürmek zararı büyütmek olurdu. Ama
            # sessiz kalmıyor: hedefsiz koşmak, YANLIŞ hedefe koşmaktan iyidir.
            self._log.error(f"{_PARAM} GECERSIZ, hedef ATANMADI: {exc}")

        # Rengin SAHİBİ burasıdır ⇒ ilanı da buradan yapılır. `fsm_node`
        # Parkur-3 kapısını (`p3_bekleniyor`) bu topic'ten öğrenir; parametreyi
        # ikinci bir node'dan okumak iki kaynak = sessiz sürüklenme demekti.
        # 🔴 13.08 av turu: varsayılan QoS **VOLATILE**. İlan `__init__`'te bir
        # kez yapılıyor; `fsm_node` o an henüz abone değilse (node başlatma
        # sırası garanti DEĞİL) mesaj KAYBOLUR ⇒ `p3_bekleniyor` sonsuza kadar
        # False ⇒ **Parkur-3 hiç açılmaz**, hiçbir belirti vermeden.
        # TRANSIENT_LOCAL = geç gelen abone SON değeri alır (latch).
        self._pub_renk = node.create_publisher(
            String, "/girdap/mission/hedef_rengi",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._renk_ilan_et()

        node.add_on_set_parameters_callback(self._on_param_set)
        # md 5.5.3.1 kapısı görev durumunu gerektiriyor (fsm_node yayınlıyor).
        self._sub = node.create_subscription(
            String, "/girdap/mission/state", self._on_mission_state, 10
        )
        if self._sinif is not None:
            self._log.info(
                f"PARKUR-3 hedef rengi (config): {self._renk_adi!r} → "
                f"class_id={CLASS_HEDEF}"
            )

    def _renk_ilan_et(self) -> None:
        """Seçili rengi yayınla (boş dize = hedef atanmamış)."""
        m = String()
        m.data = self._renk_adi if self._sinif is not None else ""
        self._pub_renk.publish(m)

    # ---------------------------------------------------------------- durum

    @property
    def sinif(self) -> Optional[int]:
        """Seçili hedef sınıfı; `None` = hedef atanmamış."""
        return self._sinif

    def _on_mission_state(self, msg: String) -> None:
        self._gorev_durumu = msg.data

    # ------------------------------------------------------- ros2 param set

    def _on_param_set(self, params) -> SetParametersResult:   # noqa: ANN001
        """Şartname zamanlama kapısı — md 5.5.3.1.

        *"hedef bilgisi … harekete başladıktan sonra aktarılamaz."* Bunu
        operatörün hatırlamasına bırakmıyoruz: görev PARKUR* durumundaysa
        değişiklik **reddedilir** ve ROS parametre değeri de değişmez.

        ⚠ Bu callback olmadan `ros2 param set` bu node'larda hiçbir etki
        yapmazdı (değerler yalnız `__init__`'te okunuyor) — belgede yazan
        "ros2 param set" yolu fiilen sessiz kalırdı.
        """
        for pr in params:
            if pr.name != _PARAM:
                continue
            izin, neden = degistirilebilir_mi(self._gorev_durumu)
            if not izin:
                self._log.error(f"hedef rengi DEGISTIRILMEDI — {neden}")
                return SetParametersResult(successful=False, reason=neden)
            try:
                yeni = renk_to_class(str(pr.value))
            except HedefRengiHatasi as exc:
                self._log.error(f"hedef rengi REDDEDILDI: {exc}")
                return SetParametersResult(successful=False, reason=str(exc))
            self._sinif = yeni
            self._renk_adi = str(pr.value).strip()
            self._gorulmedi_uyarildi = False
            self._renk_ilan_et()
            # WARN seviyesi bilinçli: operatör koşu öncesi bunu GÖRMELİ.
            self._log.warn(
                "PARKUR-3 HEDEF RENGI = "
                + (f"{self._renk_adi!r} → class_id={CLASS_HEDEF}"
                   if yeni is not None else "ATANMADI (bos)")
                + f" · {neden}"
            )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------- uygulama

    def uygula(self, tespitler: Iterable[_Tespit]) -> int:
        """Seçili rengi `CLASS_HEDEF`e taşı; taşınan sayıyı döndür.

        Hedef atanmamışsa hiçbir şey yapmaz (0 döner).
        """
        if self._sinif is None:
            return 0
        liste = list(tespitler)
        n = hedef_isaretle(liste, self._sinif)
        # İdempotenslik: yukarı akıştaki node zaten etiketlemiş olabilir.
        # "Hiç görülmedi" uyarısı için zaten-hedef olanları da say, yoksa iki
        # node birlikte koşarken füzyon yanlış alarm verirdi.
        zaten = sum(1 for t in liste if t.class_id == CLASS_HEDEF)
        if zaten == 0:
            if not self._gorulmedi_uyarildi:
                self._gorulmedi_uyarildi = True
                if self._sinif not in DEDEKTORU_OLAN_SINIFLAR:
                    # Ör. SİYAH: bu node'da dedektörü YOK. "HSV esigini
                    # kontrol et" demek operatoru olmayan bir soruna surer.
                    self._log.warn(
                        f"hedef rengi {self._renk_adi!r} atandi — bu node o "
                        f"rengi TESPIT ETMIYOR (dedektoru yok). Hedefi algi "
                        f"ekibinin P3 node'u gorecek; burada uyari normaldir."
                    )
                else:
                    self._log.warn(
                        f"hedef rengi {self._renk_adi!r} atandi ama karede HIC "
                        f"gorulmedi — HSV esigi / isik / renk adi kontrol edilmeli"
                    )
        else:
            self._gorulmedi_uyarildi = False
        return n
