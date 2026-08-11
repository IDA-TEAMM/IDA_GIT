"""Soft-restart yayını — 12'lik madde #11, şartname md 5.5.3.1.

**Şartnamenin dediği:** yeniden başlama hakkı **1 kez** kullanılabilir,
toplanan puanlar **sıfırlanır**, **süre DURMAZ**.

Süre durmadığı için elle node yeniden başlatmak (ROS keşfi + MAVROS'un FC'ye
yeniden bağlanması + GPS/EKF'in oturması) lüks — dakikalar gider. Bu yüzden
yığın **ayakta kalırken** durum sıfırlanır.

**Neden tek servis + fan-out topic:** sıfırlanacak şey beş node'a dağılmış
(FSM durumu · kapı hafızası + MPPI sıcak durumu · görev index'i · CSV oturumu ·
PNG/mp4 oturumu). Operatörün beş ayrı çağrı yapmasını beklemek md 5.5.3.1'in
"süre durmaz" gerçeğiyle çelişir. Bu yüzden:

    /girdap/mission/reset  (std_srvs/Trigger)   → YALNIZ fsm_node sunar
              ↓ yayın
    /girdap/mission/reset_seq  (std_msgs/Int32, TRANSIENT_LOCAL)
              ↓
    planning_node · mission_manager_node · telemetry_node ·
    local_map_node · lidar_kayit_node  → her biri kendini toplar

**Neden SAYAÇ, bayrak değil:** "bu sıfırlamayı işledim mi?" sorusu bayrakla
belirsiz kalır (aynı `True` iki kez gelirse ikinci gerçek bir yeniden başlama
mı, yoksa tekrar yayın mı?). Artan sayaçta her yeni değer bir kez işlenir.

**Neden TRANSIENT_LOCAL:** sıfırlama TEK ATIŞ bir olay. Bir node o anda meşgul
olsa ya da (Restart=on-failure ile) yeniden doğsa, son sıfırlama değerini
yayında bulur ve kaçırmaz. Volatile QoS'ta olay sessizce kaybolurdu — ve
kaybolduğu ancak ikinci turun ortasında (yanlış davranışla) fark edilirdi.
"""

from __future__ import annotations

from typing import Callable, Optional

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32

#: Servis (yalnız fsm_node sunar) ve fan-out topic adı.
RESET_SERVICE = "/girdap/mission/reset"
RESET_TOPIC = "/girdap/mission/reset_seq"


def reset_qos() -> QoSProfile:
    """Tek atışlık olay için: RELIABLE + TRANSIENT_LOCAL, derinlik 1.

    ⚠ Yayıncı ve abonelerin QoS'u UYUŞMALI. TRANSIENT_LOCAL yayıncıya karşı
    VOLATILE abone bağlanır (uyumlu) ama geçmişi almaz — yani geç doğan node
    sıfırlamayı KAÇIRIR. Bu yüzden iki taraf da bu tek fonksiyonu kullanıyor.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class ResetYayinci:
    """Sıfırlama sayacını yayınlar (fsm_node kullanır)."""

    def __init__(self, node) -> None:                    # noqa: ANN001
        self._pub = node.create_publisher(Int32, RESET_TOPIC, reset_qos())
        self._sayac = 0

    @property
    def sayac(self) -> int:
        return self._sayac

    def yayinla(self) -> int:
        """Sayacı artır ve yayınla; yeni sayacı döndür."""
        self._sayac += 1
        m = Int32()
        m.data = self._sayac
        self._pub.publish(m)
        return self._sayac


class ResetAbonesi:
    """Sıfırlama olayını dinler ve verilen işi BİR KEZ koşturur.

    Kullanım (node `__init__` içinde)::

        self._reset = ResetAbonesi(self, self._yeniden_basla)

    `isi` çağrılırken bir istisna fırlarsa YAKALANIR ve loglanır: bir node'un
    sıfırlanamaması diğerlerini engellememeli (yarım sıfırlanmış yığın kötü,
    ama çöken node daha kötü).
    """

    def __init__(self, node, isi: Callable[[], None]) -> None:  # noqa: ANN001
        self._node = node
        self._log = node.get_logger()
        self._isi = isi
        self._son: Optional[int] = None
        self._sub = node.create_subscription(
            Int32, RESET_TOPIC, self._on_reset, reset_qos()
        )

    def _on_reset(self, msg: Int32) -> None:
        if self._son is not None and msg.data <= self._son:
            return                       # tekrar yayın / geç teslim — yut
        self._son = msg.data
        try:
            self._isi()
        except Exception as exc:         # noqa: BLE001 — bkz. sınıf docstring'i
            self._log.error(
                f"YENIDEN BASLAMA #{msg.data} bu node'da BASARISIZ: {exc!r} — "
                f"digerleri etkilenmedi, bu node ELLE kontrol edilmeli"
            )
            return
        self._log.warn(                  # WARN: operatör görmeli
            f"YENIDEN BASLAMA #{msg.data} uygulandi (md 5.5.3.1)"
        )
