#!/usr/bin/env python3
"""ACİL DURDURMA + KLAVYEYLE MANUEL SÜRÜŞ — laptoptan, Jetson'sız.

    python3 scripts/acil_kumanda.py                       # MP'nin UDP çıkışına
    python3 scripts/acil_kumanda.py --baglanti /dev/ttyUSB0:57600
    python3 scripts/acil_kumanda.py --kuru                 # araçsız deneme

🔴 **NEDEN VAR.** 14.08 göl testinde tekne hiç çalışmadı, sürüklenerek karşı
kıyıya gitti ve oradan elle alındı. RC ile de müdahale edilemedi — çünkü
Pixhawk'ta **RC alıcısı fiziksel olarak yok** (§0.41c: `SYS_STATUS` alıcı biti
kapalı, `/mavros/rc/in` boş, `MODE1-6` altısı da MANUAL). Yarışmada RC
kullanılmayacak, ama **testte tekne kaçtığında elde hiçbir şey kalmıyor.**

⇒ Kaptan isteği (karar-eksikler.odt madde 1): laptop tuşlarıyla acil durdurma
ve manuel kontrol.

## Neden Jetson üzerinden DEĞİL

Bu araç doğrudan uçuş kontrolcüsüne konuşur. 14.08'de Jetson, ROS yığını ve
otonomi zincirinin tamamı işe yaramazken telemetri radyosu ayaktaydı. Acil
durdurmanın, kurtarmaya çalıştığı şeyin sağlığına bağlı olması saçmadır.

## Güvenlik tasarımı — okumadan değiştirme

1. **ÖLÜ ADAM (dead-man).** Gaz komutu yalnız tuş BASILI tutulurken sürer;
   `--olu-adam` süresi (varsayılan 0,4 s) boyunca tuş gelmezse gaz **nötre**
   döner. Takılı kalan tuş, donmuş terminal ya da kopmuş SSH kaçak tekne
   üretmesin diye.
2. **DEVREYE ALMA ayrı adım.** Açılışta hiçbir şey gönderilmez; sürüş için
   önce `M` (MANUAL + devreye al) gerekir. Yanlışlıkla basılan bir yön tuşu
   tek başına tekneyi hareket ettiremez.
3. **ÇIKIŞTA NÖTR + BIRAK.** `Q`, Ctrl+C ve beklenmedik çöküş dahil her
   çıkışta override serbest bırakılır (`UINT16_MAX`) ve gaz nötre çekilir.
4. **ACİL DURDURMA HER ZAMAN AÇIK.** `BOŞLUK` devreye alınmış olsun olmasın
   çalışır: `MAV_CMD_COMPONENT_ARM_DISARM` + **21196** zorlama koduyla —
   ArduPilot'un "uçuşta bile disarm et" kodu. Sürüklenen bir teknede motoru
   kesmek, konumu tutmaya çalışmaktan daha güvenilirdir (HOLD, GPS/EKF ister;
   14.08'de ikisi de bozuktu).
"""

from __future__ import annotations

import argparse
import atexit
import os
import select
import sys
import termios
import time
import tty

VARSAYILAN_BAGLANTI = "udp:127.0.0.1:14551"
NOTR = 1500
BIRAK = 65535                 # RC_CHANNELS_OVERRIDE: kanalı serbest bırak
ZORLA_DISARM = 21196          # ArduPilot "force" sihirli sayısı
YOLLAMA_HZ = 20               # ArduPilot override akışı ~3 s'de zaman aşar

YARDIM = """
  M          MANUAL moda geç + sürüşü DEVREYE AL
  W / S      ileri / geri            A / D    sola / sağa
  BOŞLUK     🔴 ACİL DURDURMA (zorla disarm)      H  HOLD moduna al
  X          override'ı bırak (otonomiye geri ver)
  Q          çık (nötr + bırak)
"""


class Kumanda:
    def __init__(self, baglanti: str, kuru: bool, olu_adam: float,
                 gaz_adim: int, don_adim: int) -> None:
        self.kuru, self.olu_adam = kuru, olu_adam
        self.gaz_adim, self.don_adim = gaz_adim, don_adim
        self.devrede = False
        self.gaz = self.don = NOTR
        self.son_tus = 0.0
        self.mav = None
        if not kuru:
            from pymavlink import mavutil
            print(f"bağlanılıyor: {baglanti} …")
            self.mav = mavutil.mavlink_connection(baglanti)
            self.mav.wait_heartbeat(timeout=15)
            print(f"✅ FC bağlı (sys {self.mav.target_system})")

    # ---- gönderim ----
    def _override(self, don: int, gaz: int) -> None:
        """RC1 = direksiyon, RC3 = gaz (Rover varsayılan RCMAP)."""
        if self.kuru or self.mav is None:
            return
        k = [BIRAK] * 18
        k[0], k[2] = don, gaz
        self.mav.mav.rc_channels_override_send(
            self.mav.target_system, self.mav.target_component, *k[:18])

    def birak(self) -> None:
        if self.kuru or self.mav is None:
            return
        self.mav.mav.rc_channels_override_send(
            self.mav.target_system, self.mav.target_component, *([BIRAK] * 18))

    def mod(self, ad: str) -> None:
        if self.kuru or self.mav is None:
            print(f"[kuru] mod → {ad}"); return
        self.mav.set_mode(self.mav.mode_mapping()[ad])

    def acil_durdur(self) -> None:
        """Zorla disarm — devrede olmasa da çalışır."""
        self.devrede = False
        self.gaz = self.don = NOTR
        self.birak()
        if self.kuru or self.mav is None:
            print("\n[kuru] 🔴 ACİL DURDURMA (zorla disarm)"); return
        from pymavlink import mavutil
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, ZORLA_DISARM, 0, 0, 0, 0, 0)
        print("\n🔴 ACİL DURDURMA GÖNDERİLDİ (zorla disarm)")

    # ---- döngü ----
    def calistir(self) -> None:
        atexit.register(self._temizle)
        if not sys.stdin.isatty():
            print("🔴 Bu araç GERÇEK TERMINAL ister (tuş okuyor).\n"
                  "   Boru/yönlendirme ile çalışmaz — terminalden doğrudan çalıştır.\n"
                  "   Mantığı sınamak için: pytest prototype/tests/test_acil_kumanda.py",
                  file=sys.stderr)
            return
        print(YARDIM)
        eski = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            period = 1.0 / YOLLAMA_HZ
            while True:
                t0 = time.monotonic()
                if select.select([sys.stdin], [], [], period)[0]:
                    if not self._tus(os.read(sys.stdin.fileno(), 1).decode(
                            "utf-8", "ignore").lower()):
                        break
                # ÖLÜ ADAM: tuş gelmiyorsa gazı nötre çek
                if self.devrede and time.monotonic() - self.son_tus > self.olu_adam:
                    self.gaz = NOTR
                if self.devrede:
                    self._override(self.don, self.gaz)
                self._durum()
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, eski)
            self._temizle()

    def _tus(self, c: str) -> bool:
        self.son_tus = time.monotonic()
        if c == " ":
            self.acil_durdur()
        elif c == "m":
            self.mod("MANUAL"); self.devrede = True
            self.gaz = self.don = NOTR
        elif c == "h":
            self.devrede = False; self.birak(); self.mod("HOLD")
        elif c == "x":
            self.devrede = False; self.birak()
        elif c == "q":
            return False
        elif self.devrede:
            if c == "w":   self.gaz = min(2000, self.gaz + self.gaz_adim)
            elif c == "s": self.gaz = max(1000, self.gaz - self.gaz_adim)
            elif c == "a": self.don = max(1000, self.don - self.don_adim)
            elif c == "d": self.don = min(2000, self.don + self.don_adim)
        return True

    def _durum(self) -> None:
        d = "DEVREDE " if self.devrede else "beklemede"
        sys.stdout.write(f"\r  {d} · dönüş {self.don:4d} · gaz {self.gaz:4d}   ")
        sys.stdout.flush()

    def _temizle(self) -> None:
        try:
            self.gaz = self.don = NOTR
            self.birak()
        except Exception:                                    # noqa: BLE001
            pass


def main() -> int:
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--baglanti", default=VARSAYILAN_BAGLANTI,
                   help="pymavlink bağlantı dizgesi (MP açıkken UDP kullan)")
    a.add_argument("--kuru", action="store_true", help="araçsız deneme")
    a.add_argument("--olu-adam", type=float, default=0.4,
                   help="bu süre tuş gelmezse gaz NÖTRE döner (s)")
    a.add_argument("--gaz-adim", type=int, default=40)
    a.add_argument("--don-adim", type=int, default=60)
    n = a.parse_args()
    try:
        Kumanda(n.baglanti, n.kuru, n.olu_adam, n.gaz_adim, n.don_adim).calistir()
    except KeyboardInterrupt:
        print("\nçıkıldı (nötr + bırak)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
