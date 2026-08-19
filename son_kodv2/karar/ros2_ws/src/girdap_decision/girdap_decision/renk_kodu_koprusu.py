"""Uçuş kontrolcüsü parametresi → Parkur-3 hedef rengi köprüsü.

    Mission Planner (YKİ ARAYÜZÜ)
      │  CONFIG → Full Parameter List → SCR_USER1 = 3 → "Write Params"
      ▼
    Pixhawk / ArduPilot        değer EEPROM'da KALICI (güç gitse de durur)
      │  MAVLink PARAM_REQUEST_READ / PARAM_VALUE
      ▼
    MAVROS (Jetson'da zaten koşuyor)      /mavros/param/get
      ▼
    BU NODE            kod (float) → kod_to_renk() → "siyah"
      │  /kamikaze_param_node/set_parameters  (rcl_interfaces)
      ▼
    KamikazeHedefKapisi._on_param_set  ← md 5.5.3.1 ZAMANLAMA KAPISI BURADA
      │  latched /girdap/mission/hedef_rengi
      ▼
    fsm_node (p3_bekleniyor) + planning_node (nişan)
      ▼
    Parkur-3 mantığı

**Neden bu yol** (Eyüp, 13.08): rengi terminalden `ros2 param set` ile girmek
şartname s.21 ile sürtüşüyor — *"Görev yükleme aşamasında … YKİ'de **sadece YKİ
arayüzü** açık olacak"*. Mission Planner'ın parametre ekranı bir YKİ arayüzüdür;
terminal değildir. (Şartname bu konuda kesin konuşmuyor ⇒ S&C'ye soruldu;
cevap "terminal serbest" gelirse bu node'a gerek kalmaz — bkz. SÖKÜLEBİLİRLİK.)

**Neden kapıyı burada TEKRARLAMIYORUZ:** değeri kendimiz uygulamıyoruz, hedef
node'un parametresini **set ediyoruz**; md 5.5.3.1 kapısı (`degistirilebilir_mi`)
orada zaten var ve hareket başladıysa **reddediyor**. Kapı tek yerde kalsın —
iki kopya sessizce ayrışır.

**SÖKÜLEBİLİRLİK:** bu node hiçbir yerden otomatik başlatılmıyor; launch'a
`use_renk_kodu_koprusu:=true` ile eklenir. Kaldırmak = bu dosyayı silmek.
Aşağı akışta hiçbir şey buna bağlı değil (renk elle de girilebilir).

🔴 **JETSON'DA DOĞRULANACAK** — PC'de `mavros_msgs` yok, bu dosyanın ROS yolu
burada koşturulamadı. Saf mantık (`prototype/mission/renk_kodu.py`) 22 testle
doğrulandı; buradaki iş yalnız kablolama.
"""
from __future__ import annotations

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

from std_msgs.msg import String

from prototype.mission.kamikaze_hedef import HedefRengiHatasi, degistirilebilir_mi
from prototype.mission.renk_kodu import RenkUygulamaDurumu

#: Hedef node — rengin SAHİBİ. 🔴 19.08 düzeltmesi: 13.08'deki değer
#: `perception_fusion_node` idi; o gün parametre orada yaşıyordu. 16.08'de HSV
#: yedek kamera hattı kaldırılınca parametre **sahipsiz kaldı** ve kendi
#: node'una taşındı (`kamikaze_param_node`). Eski adla geri getirilseydi köprü
#: var olmayan bir servise yazmaya çalışır, "servis hazır değil" deyip sonsuza
#: kadar bekler ve renk HİÇ uygulanmazdı — belirti yalnız bir WARN satırı.
_HEDEF_NODE = "kamikaze_param_node"
_HEDEF_PARAM = "kamikaze_target_color"

#: FC parametresi. `SCR_USER1..6` ArduPilot'ta **script kullanıcı alanı**dır ve
#: `SCR_ENABLE=1` + yeniden başlatma ister (bir kereliktir, ATÖLYEDE yapılır;
#: yarışma sabahı yalnız değer yazılır). Firmware bu değeri kendisi kullanmaz.
_VARSAYILAN_FC_PARAM = "SCR_USER1"


class RenkKoduKoprusu(Node):
    """FC parametresini yoklar, renge çevirir, hedef node'a önerir.

    Yoklama (poll) bilinçli: MAVROS parametre önbelleği ile Mission Planner'ın
    yazması arasındaki tazelenmeyi garanti eden tek basit yol bu. Periyot
    büyük (varsayılan 2 sn) — koşu öncesi bir defa doğru değeri yakalamak için
    fazlasıyla yeterli, kimseyi meşgul etmiyor.
    """

    def __init__(self) -> None:
        super().__init__("renk_kodu_koprusu")
        self.declare_parameter("fc_param_adi", _VARSAYILAN_FC_PARAM)
        self.declare_parameter("periyot_s", 2.0)
        self.declare_parameter("hedef_node", _HEDEF_NODE)

        self._fc_param = str(self.get_parameter("fc_param_adi").value)
        hedef = str(self.get_parameter("hedef_node").value)

        self._param_get = self.create_client(self._param_get_tipi(),
                                             "/mavros/param/get")
        self._set_cli = self.create_client(SetParameters,
                                           f"/{hedef}/set_parameters")
        # 🔴 Kod ancak BAŞARIYLA UYGULANDIKTAN sonra işlenmiş sayılır —
        # geçici hata (servis hazır değil / çağrı düştü) yeniden denenir.
        # İlk sürüm okunanı uygulamadan önce önbelleğe yazıyordu ⇒ açılışta
        # hedef node hazır değilse renk BİR DAHA HİÇ uygulanmıyordu (sessiz).
        self._durum = RenkUygulamaDurumu()
        self._uyarildi = False
        self._servis_uyarildi = False
        # 🔴 13.08 2. av turu: her turda koşulsuz `call_async` yapılıyordu.
        # Servis VAR ama cevap vermiyorsa (FC susmuş, MAVLink linki koptu)
        # istekler birikir — hem kaynak sızıntısı hem de sonradan hepsi birden
        # dönüp aynı değeri defalarca uygulamaya kalkar. Uçuşta tanısı zor.
        self._okuma_ucusta = False
        self._yazma_ucusta = False
        self._gorev_durumu: str | None = None
        self._durdu = False
        # md 5.5.3.1 — hareket BAŞLADIKTAN SONRA hedef bilgisi aktarılamaz.
        # Hedef node zaten reddediyor; burada AYRICA yoklamayı kesiyoruz ki
        # sistem koşu ortasında gelen bir rengi almaya **kalkışmasın** bile.
        # ("reddediliyor" savunması "hiç denemiyor"dan zayıftır; ayrıca
        # journal'da koşu boyunca param okuma trafiği görünmesin.)
        self.create_subscription(String, "/girdap/mission/state",
                                 self._on_gorev_durumu, 10)

        self.create_timer(float(self.get_parameter("periyot_s").value),
                          self._tik)
        self.get_logger().info(
            f"renk kodu köprüsü: FC '{self._fc_param}' → {hedef}.{_HEDEF_PARAM} "
            "(0=karar yok 1=kirmizi 2=yesil 3=siyah)"
        )

    # ------------------------------------------------------------------ ROS
    @staticmethod
    def _param_get_tipi():
        """`mavros_msgs` tembel import — paket yoksa node açılışta dürüstçe ölür,
        sessizce çalışıyormuş gibi yapmaz."""
        from mavros_msgs.srv import ParamGet      # noqa: PLC0415
        return ParamGet

    def _on_gorev_durumu(self, msg) -> None:            # noqa: ANN001
        self._gorev_durumu = msg.data

    def _tik(self) -> None:
        # md 5.5.3.1 kapısı — kuralın KAYNAĞI kamikaze_hedef'te, burada yalnız
        # "şimdi okumaya gerek var mı" sorusu için kullanılıyor (kural kopyası
        # DEĞİL; kopya sessizce ayrışırdı).
        izin, neden = degistirilebilir_mi(self._gorev_durumu)
        if not izin:
            if not self._durdu:
                self._durdu = True
                self.get_logger().info(
                    f"renk kodu yoklamasi DURDURULDU — {neden}"
                )
            return
        self._durdu = False

        # Bekleyen bir uygulama varsa YOKLAMAYI BEKLEMEDEN yeniden dene.
        bekleyen = self._durum.bekleyen
        if bekleyen is not None:
            self._uygula(bekleyen)

        if self._okuma_ucusta:
            return                      # önceki istek hâlâ cevap bekliyor
        if not self._param_get.service_is_ready():
            if not self._uyarildi:
                self.get_logger().warn(
                    "/mavros/param/get HAZIR DEĞİL — MAVROS koşuyor mu? "
                    "Hedef rengi FC'den okunamıyor."
                )
                self._uyarildi = True
            return
        self._uyarildi = False
        istek = self._param_get.srv_type.Request()
        istek.param_id = self._fc_param
        self._okuma_ucusta = True
        gelecek = self._param_get.call_async(istek)
        gelecek.add_done_callback(self._cevap_geldi)

    def _cevap_geldi(self, gelecek) -> None:            # noqa: ANN001
        self._okuma_ucusta = False
        try:
            cevap = gelecek.result()
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"param/get çağrısı düştü: {exc}")
            return
        if cevap is None or not cevap.success:
            self.get_logger().warn(
                f"FC parametresi '{self._fc_param}' OKUNAMADI — "
                "SCR_ENABLE=1 mi, autopilot yeniden başlatıldı mı?"
            )
            return

        # ParamValue: FLOAT parametre `real`de, INT parametre `integer`da gelir;
        # ikisinden dolu olanı alınır (ikisi de 0 ise kod zaten 0 = karar yok).
        ham = float(getattr(cevap.value, "real", 0.0)) or float(
            getattr(cevap.value, "integer", 0)
        )
        try:
            renk, yeni = self._durum.kod_geldi(ham)
        except HedefRengiHatasi as exc:
            # Sessizce "hedef yok"a düşmek operatörün hatasını GİZLERDİ.
            self.get_logger().error(f"FC'den GEÇERSİZ renk kodu: {exc}")
            return

        if renk is None:
            if yeni and self._durum.uygulanan is None:
                self.get_logger().info(
                    "FC renk kodu 0 — hedef atanmamış (bekleniyor)")
            return
        self._uygula(renk)

    def _uygula(self, renk: str) -> None:
        """Hedef node'un parametresini set et — md 5.5.3.1 kapısı ORADA.

        Başarısızlık durumunda `_durum.bekleyen` dolu kalır ⇒ bir sonraki
        turda yeniden denenir (sessiz kayıp yok).
        """
        if renk == self._durum.uygulanan:
            return
        if self._yazma_ucusta:
            return                      # aynı anda iki set çağrısı gönderme
        if not self._set_cli.service_is_ready():
            if not self._servis_uyarildi:
                self._servis_uyarildi = True
                self.get_logger().warn(
                    "hedef node'un parametre servisi hazır değil — "
                    "renk BEKLETİLİYOR, denemeye devam ediliyor")
            return
        self._servis_uyarildi = False
        p = ParameterMsg()
        p.name = _HEDEF_PARAM
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                 string_value=renk)
        istek = SetParameters.Request(parameters=[p])
        self._yazma_ucusta = True
        gelecek = self._set_cli.call_async(istek)

        def _sonuc(f) -> None:                           # noqa: ANN001
            self._yazma_ucusta = False
            try:
                sonuclar = f.result().results
            except Exception as exc:                     # noqa: BLE001
                self.get_logger().warn(f"parametre set çağrısı düştü: {exc}")
                return
            ok = bool(sonuclar) and sonuclar[0].successful
            if ok:
                self._durum.uygulandi(renk)
                # WARN bilinçli: operatör koşu öncesi bunu GÖRMELİ.
                self.get_logger().warn(
                    f"PARKUR-3 HEDEF RENGI FC'den alindi: {renk.upper()}"
                )
            else:
                neden = sonuclar[0].reason if sonuclar else "?"
                # Beklenen tek ret sebebi: hareket başlamış (md 5.5.3.1).
                self.get_logger().error(
                    f"hedef rengi UYGULANAMADI ({renk}): {neden}"
                )

        gelecek.add_done_callback(_sonuc)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RenkKoduKoprusu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
