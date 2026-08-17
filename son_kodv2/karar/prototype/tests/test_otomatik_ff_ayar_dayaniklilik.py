"""`otomatik_ff_ayar.py` — SERVİS KİPİNİN HAYATTA KALMASI.

🔴 NEDEN (17.08.2026 sabahı, reboot'ta cihazda ölçüldü):

    İPTAL 🔴 RuntimeError: ATC_STR_RAT_FF beklenmedik tip (0) — FC'de var mı?

Araç boot'tan **29 saniye** sonra öldü. İki ayrı kusur üst üste bindi:

  ① MAVROS ayağa kalkar kalkmaz `get_parameters` servisi VARDIR ama FC'den
    parametreleri henüz indirmemiştir → okuma `PARAMETER_NOT_SET` döner.
    Araç bunu *"FC'de yok"* sanıp iptal etti. Oysa bu bir hata değil,
    bir BEKLEME sebebi.
  ② Süreç 0 ile çıktığı için `Restart=on-failure` olan servis onu
    diriltmedi. `systemctl is-enabled` hâlâ "enabled" diyordu; kimse
    fark etmedi.

Sonuç: tekne suya girdiğinde FF ayarı **ortada yoktu** ve hiçbir hata
basılmamıştı. Bu dosya o iki kusurun geri gelmesini engeller.

⚠️ Bu testler FC'ye yazmaz, düğüm açmaz — `_servis_dongusu` saf akış
mantığıdır ve sahte bir düğümle koşturulur.
"""
import importlib.util
import os

import pytest

_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_BETIK = os.path.join(_KOK, "scripts", "otomatik_ff_ayar.py")

pytest.importorskip("rclpy", reason="ROS ortamı yok")
pytest.importorskip("mavros_msgs", reason="mavros_msgs yok")


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("otomatik_ff_ayar", _BETIK)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(autouse=True)
def _hizli(mod, monkeypatch):
    """Testler 30 sn beklemesin; gecikmenin DEĞERİ ayrıca sınanıyor."""
    monkeypatch.setattr(mod, "YENIDEN_DENE_SN", 0.0)


class _SahteDugum:
    """`_servis_dongusu`nun dokunduğu yüzeyin tamamı — fazlası değil."""

    def __init__(self, davranislar):
        self._davranislar = list(davranislar)
        self._ayar_yazildi = False
        self._geri_yuklendi = False
        self._ozgun = {"x": 1.0}
        self.geri_yukleme_sayisi = 0
        self.cagri = 0
        self.satirlar = []

    def _bas(self, tur, metin):
        self.satirlar.append((tur, metin))

    def geri_yukle(self):
        self.geri_yukleme_sayisi += 1
        self._geri_yuklendi = True

    def calistir(self):
        self.cagri += 1
        d = self._davranislar.pop(0) if self._davranislar else "yaz"
        if isinstance(d, BaseException):
            raise d
        if d == "yaz":
            self._ayar_yazildi = True
        # "olcemedi": sessizce döner, hiçbir şey yazmaz


def test_parametre_HENUZ_GELMEDI_hatasi_ARACI_OLDURMEZ(mod):
    """17.08'in tam senaryosu: ilk iki deneme RuntimeError, üçüncüde yazar."""
    d = _SahteDugum([RuntimeError("ATC_STR_RAT_FF beklenmedik tip (0)"),
                     RuntimeError("ATC_STR_RAT_FF beklenmedik tip (0)"),
                     "yaz"])
    mod._servis_dongusu(d)
    assert d.cagri == 3, "araç ilk hatada ölmüş — 17.08 kusuru geri geldi"
    assert d._ayar_yazildi is True
    assert d.geri_yukleme_sayisi == 2, "her hatada özgün değerler geri yüklenmeli"


def test_OLCEMEDI_hali_de_YENIDEN_DENENIR(mod):
    """Gürültü kapısı sonuç ilan edemediyse süreç ölmez, koşulları bekler.

    Sahada tekne bir kez disarm olunca ayarın bir daha hiç koşmaması
    istenmiyor.
    """
    d = _SahteDugum(["olcemedi", "olcemedi", "yaz"])
    mod._servis_dongusu(d)
    assert d.cagri == 3
    assert d._ayar_yazildi is True


def test_SIGTERM_ANINDA_cikar_ve_GERI_YUKLER(mod):
    """systemd durduruyorsa itiraz etme — ama önce özgüne dön."""
    d = _SahteDugum([KeyboardInterrupt("SIGTERM"), "yaz"])
    mod._servis_dongusu(d)
    assert d.cagri == 1, "SIGTERM'den sonra yeni deneme BAŞLATILMAMALI"
    assert d.geri_yukleme_sayisi == 1
    assert d._ayar_yazildi is False


def test_yazdiktan_SONRA_cikar_sonsuz_dongu_YOK(mod):
    d = _SahteDugum(["yaz"])
    mod._servis_dongusu(d)
    assert d.cagri == 1


# ── özgün değerleri sabırla okuma ──────────────────────────────────────────

class _SahteOkuyucu:
    """`_ozgunleri_oku` için asgari yüzey."""

    def __init__(self, bekle, gelis_denemesi):
        self.bekle = bekle
        self._gelis = gelis_denemesi
        self._deneme = 0
        self.satirlar = []

    def _bas(self, tur, metin):
        self.satirlar.append((tur, metin))

    def _oku(self, ad, hosgor=False):
        self._deneme += 1
        if self._deneme < self._gelis:
            return None            # MAVROS henüz FC'den indirmedi
        return 0.52


def test_ozgun_okuma_FC_GECIKMESINI_BEKLER(mod, monkeypatch):
    monkeypatch.setattr(mod.rclpy, "spin_once", lambda *a, **k: None)
    monkeypatch.setattr(mod.time, "sleep", lambda *_a: None)
    s = _SahteOkuyucu(bekle=True, gelis_denemesi=5)
    d = mod.FFAyar._ozgunleri_oku(s, (mod.PARAM_FF,))
    assert d == {mod.PARAM_FF: 0.52}
    assert s._deneme >= 5, "beklemeden pes etmiş"


def test_elle_kosumda_SEBEBI_SOYLEYEREK_duser(mod, monkeypatch):
    """Servis değilse sonsuza kadar bekleme — ama mesaj teşhis edilebilir olsun."""
    monkeypatch.setattr(mod.rclpy, "spin_once", lambda *a, **k: None)
    monkeypatch.setattr(mod.time, "sleep", lambda *_a: None)
    monkeypatch.setattr(mod, "PARAM_BEKLEME_AZAMI", 0.0)
    s = _SahteOkuyucu(bekle=False, gelis_denemesi=10**9)
    with pytest.raises(RuntimeError) as e:
        mod.FFAyar._ozgunleri_oku(s, (mod.PARAM_FF,))
    assert mod.PARAM_FF in str(e.value)
    assert "--bekle" in str(e.value), "elle koşana servis kipi önerilmeli"


def test_servis_kipinde_bekleme_SINIRSIZ(mod):
    """Tekne saatler sonra suya girebilir — elle koşumun sabrı ölçüt olamaz."""
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert "if not self.bekle and gecen > PARAM_BEKLEME_AZAMI" in kaynak, (
        "zaman aşımı servis kipinde de uygulanıyor — araç suya girmeden ölür")


def test_ayar_yazildi_bayragi_GERI_YUKLEMEDEN_AYRI(mod):
    """`_geri_yuklendi` 'iş bitti' sorusunu cevaplayamaz: hem yazınca hem
    geri yükleyince True olur. Servis kipi ayrı bayrağa bakmalı."""
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert kaynak.count("self._ayar_yazildi = True") == 2, (
        "yazma noktalarından biri bayrağı kaldırmıyor — servis kipi "
        "yazdıktan sonra sonsuza kadar yeniden dener")
