"""Parkur-3'ten ÇIKIŞ ölçütleri — ROS'suz, saat/odom girdisiyle test edilebilir.

🔴 **Neden var:** P3'ün tek çıkışı IMU şokuydu (`shock_threshold_g = 3.0`) ve
o eşik **asla aşılmaz**. Karar tarafının kendi ölçümü (`pipeline.py:100-104`)
kamikaze temas hızını **0,134-0,154 m/s** veriyor; bu hızdan duruş 0,1-0,5 sn'de
**0,03-0,14 g** üretir, IMU durağanken zaten **1,0 g** okur ⇒ tepe 1,03-1,14 g.
Eşik 3,0 g. Yani şok kanalı P3'ü **hiç bitiremez**.

FAZ 1'de giriş kuralı değiştiği için (son waypoint → PARKUR3) bu delik
**kritik hâle geldi**: `mission_complete → TAMAMLANDI` artık P3'te devre dışı,
dolayısıyla şok gelmezse tekne **sonsuza kadar** kamikaze çekicisiyle hedefe
yüklenir. İki bağımsız çıkış eklendi:

1. **İlerleme yok** — P3'te kamikaze profili tam gaz sürerken hız ~0 kalıyorsa
   sabit bir cisme yaslanmışız demektir (temas). Dalga bunu taklit edemez:
   dalgada hız **salınır**, sıfırda takılı kalmaz.
2. **Süre aşımı** — hedef hiç bulunamazsa tekne sonsuza kadar sürüklenmesin.
   Şartname s.22: yarışma süresi sınırlı (20 dk yazıyor, tanımlar bölümünde
   "DSB" — [[sartname]] çelişkisi kayıtlı), P3 sonrası dönüş süreye dahil değil.

⚠️ **EŞİKLER SUDA ÖLÇÜLECEK.** Buradaki değerler makul başlangıç; gerçek temas
davranışı (tekne hedefi itiyor mu, kayıyor mu) ancak sahada görülür.
"""
from __future__ import annotations

from typing import Optional

#: Bu hızın altı "ilerlemiyor" sayılır (m/s). Ölçülen temas hızı 0,134-0,154
#: olduğu için eşik onun ALTINDA olmalı — yoksa normal yaklaşma da "temas"
#: sanılır. 0,08: temas hızının ~yarısı.
DURMA_HIZI_MPS = 0.08
#: Bu kadar sürekli durgunluktan sonra temas kabul edilir. Kısa tutulursa
#: dalga çukuru/anlık yavaşlama yanlış tetikler; uzun tutulursa tekne hedefi
#: gereksiz uzun iter (şartname hedefe sürekli temas konusunda SESSİZ).
DURMA_SURESI_S = 3.0
#: P3'e girişten sonra en fazla bu kadar beklenir. Hedef bulunamadıysa
#: dürüstçe bitir — sonsuza kadar kamikaze modunda sürüklenmek hem tehlikeli
#: hem de dönüş süresini yer.
AZAMI_P3_SURESI_S = 120.0


class P3CikisIzleyici:
    """P3'teyken: temas ettik mi, süre doldu mu?

    Saat dışarıdan verilir (`monotonic`) — node saat kaynağını seçer, burası
    test edilebilir kalır.
    """

    def __init__(self, durma_hizi: float = DURMA_HIZI_MPS,
                 durma_suresi_s: float = DURMA_SURESI_S,
                 azami_sure_s: float = AZAMI_P3_SURESI_S) -> None:
        self._durma_hizi = float(durma_hizi)
        self._durma_suresi = float(durma_suresi_s)
        self._azami_sure = float(azami_sure_s)
        self._giris_t: Optional[float] = None
        self._durgun_baslangic: Optional[float] = None

    def p3ye_girildi(self, t: float) -> None:
        """PARKUR3'e geçildi — sayaçlar burada başlar."""
        self._giris_t = float(t)
        self._durgun_baslangic = None

    def sifirla(self) -> None:
        """P3'ten çıkıldı / yeniden başlama — her şey sıfırlanır."""
        self._giris_t = None
        self._durgun_baslangic = None

    def guncelle(self, t: float, hiz_mps: float) -> tuple[bool, bool]:
        """Döner: (ilerleme_yok, sure_doldu). P3'e girilmediyse ikisi de False."""
        if self._giris_t is None:
            return False, False

        if abs(float(hiz_mps)) < self._durma_hizi:
            if self._durgun_baslangic is None:
                self._durgun_baslangic = float(t)
            ilerleme_yok = (t - self._durgun_baslangic) >= self._durma_suresi
        else:
            # 🔑 Tek bir hızlı örnek sayacı SIFIRLAR: dalgada hız salınır,
            # gerçek temasta salınmaz. Biriktirme değil, KESİNTİSİZ durgunluk.
            self._durgun_baslangic = None
            ilerleme_yok = False

        sure_doldu = (t - self._giris_t) >= self._azami_sure
        return ilerleme_yok, sure_doldu

    @property
    def p3te_mi(self) -> bool:
        return self._giris_t is not None
