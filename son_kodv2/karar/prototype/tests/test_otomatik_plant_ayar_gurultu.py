"""`otomatik_plant_ayar.py` — GÜRÜLTÜYÜ İVME SANMA + SERVİS HAYATTA KALMA.

🔴 NEDEN (17.08.2026 reboot koşumu, cihazda ölçüldü):

    ÖLÇÜM  ⏳ 540/600 sn · eğri örneği 0 (MANUAL) · ivme örneği 4301
    ÖLÇÜM  ivme %95 3.42 (maks 12.22) · fren %95 3.73 (maks 12.49) m/s²
    ÖLÇÜM  🔴 ATC_ACCEL_MAX=3.42 yazılamadı (sınır dışı ya da FC'de yok)
    SONUÇ  🔴 HİÇBİR PARAMETRE YAZILMADI
    systemd: girdap-plant-ayar.service: Deactivated successfully.

Tekne o sırada KARADAYDI: `eğri örneği 0` (hiç arm olmadı, hiç kımıldamadı),
EKF3 `is using GPS` ancak **10:55:49**'da — ölçüm bittikten 13 dakika sonra.
4.301 örneğin tamamı, yerinde duran tekneyi metrelerce gezdiren poz
gürültüsünün türeviydi. Havuzun 4 oturumluk gerçek değeri **0,28 m/s²**.

İki ayrı kusur:

  ① `_on_hiz` ivme örneğini KOŞULSUZ topluyordu. Eğri örneği
    `armed`+`MANUAL`+hareket şartına bağlıyken ivmenin hiçbir şartı yoktu —
    aracın *"tekne kımıldıyor mu?"* diye soran kapısı yoktu.
    ⚠️ Yazmayı durduran tek şey `IVME_UST=3,00` sert sınırıydı ve onu
    TESADÜFEN yakaladı: gürültü %95'i 2,5 çıksaydı araç gerçek sınırın
    9 katını FC'ye KALICI yazıp "geri okundu ✅" basacaktı.
  ② Araç ölçemese de 0 ile çıkıyordu; `Restart=on-failure` servisi
    diriltmedi. Tekne suya girdiğinde araç ortada olmayacaktı.

Bu dosya iki kusurun da geri gelmesini engeller.

⚠️ Testler FC'ye YAZMAZ, düğüm açmaz — sahte düğümle saf akış sınanır.
"""
import importlib.util
import os

import pytest

_KOK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_BETIK = os.path.join(_KOK, "scripts", "otomatik_plant_ayar.py")

pytest.importorskip("rclpy", reason="ROS ortamı yok")
pytest.importorskip("mavros_msgs", reason="mavros_msgs yok")


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("otomatik_plant_ayar", _BETIK)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ── ① kontrol grubu: gürültü ivme sanılmasın ────────────────────────────────

class _SahteOlcum:
    """`_gurultu_tabani` / `_gurultu_kapisi`nın dokunduğu yüzeyin tamamı."""

    def __init__(self, gurultu):
        self._gurultu = list(gurultu)
        self.satirlar = []

    def _bas(self, tur, metin):
        self.satirlar.append((tur, metin))


def test_17_08_SENARYOSU_gurultu_ivme_SANILMAZ(mod):
    """Yerinde duran teknenin gürültüsü 'fiziksel sınır' diye yazılmasın.

    Gerçek koşumun sayıları: gürültü tabanı ~3,4 ↔ 'ölçülen' 3,42 ⇒ oran 1,0.
    """
    s = _SahteOlcum([3.4] * mod.ASGARI_GURULTU)
    taban = mod.PlantAyar._gurultu_tabani(s)
    gecti, sebep = mod.PlantAyar._gurultu_kapisi(s, 3.42, taban)
    assert gecti is False, "gürültü, ölçüm diye kabul edildi — 17.08 kusuru"
    assert "gürültü" in sebep


def test_GERCEK_hareket_kapiyi_GECER(mod):
    """Tekne gerçekten hızlanıyorsa kapı engel olmamalı (yanlış pozitif yok)."""
    s = _SahteOlcum([0.05] * mod.ASGARI_GURULTU)
    taban = mod.PlantAyar._gurultu_tabani(s)
    gecti, _ = mod.PlantAyar._gurultu_kapisi(s, 0.28, taban)   # havuzun değeri
    assert gecti is True, "gerçek ölçüm gürültü sanıldı"


def test_taban_ILAN_EDILEMEZSE_sessizce_gecmis_SAYILMAZ(mod):
    """Kontrol grubu yoksa kapı uygulanamaz — ama bu RAPOR EDİLMELİ.

    'Kapı yok' ile 'kapıdan geçti' aynı şey değildir; log ikisini ayırmalı.
    """
    s = _SahteOlcum([0.05] * (mod.ASGARI_GURULTU - 1))
    taban = mod.PlantAyar._gurultu_tabani(s)
    assert taban is None
    gecti, _ = mod.PlantAyar._gurultu_kapisi(s, 0.28, taban)
    assert gecti is True
    assert any("kontrol grubu YOK" in m for _t, m in s.satirlar), (
        "kapı uygulanamadığı sessizce geçti")


def test_ARMED_ve_DISARM_ornekleri_AYRI_kovada(mod):
    """`_on_hiz` ivmeyi arm durumuna göre ayırmalı — kusurun tam yeri."""
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert "(self._ivme if self._armed else self._gurultu).append(a)" in kaynak, (
        "ivme örneği yine KOŞULSUZ toplanıyor — disarm gürültüsü fiziksel "
        "sınır diye FC'ye yazılabilir")


def test_sert_sinir_TEK_BASINA_kapi_SAYILMAZ(mod):
    """IVME_UST son çaredir; ölçümü ondan önce eleyen bir kapı olmalı."""
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert "_gurultu_kapisi" in kaynak
    i_kapi = kaynak.index("gecti, sebep = self._gurultu_kapisi")
    i_sinir = kaynak.index("SERT SINIR DIŞI")
    assert i_kapi < i_sinir, "gürültü kapısı sert sınırdan SONRA uygulanıyor"


# ── ② servis kipi: ölçene kadar yaşa ───────────────────────────────────────

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
        self._egri = []
        self._ivme = []
        self._gurultu = []
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
        d = self._davranislar.pop(0) if self._davranislar else "olctu"
        if isinstance(d, BaseException):
            raise d
        if d == "olctu":
            self._ayar_yazildi = True
        # "olcemedi" / "yedek": hiçbir şey yazmaz — bayrak kalkmaz


def test_OLCEMEDI_hali_ARACI_OLDURMEZ(mod):
    """17.08'in tam senaryosu: ölçemedi ama süreç yaşamaya devam etmeli.

    Eğri YALNIZ kaptan MANUAL'de sürerken oluşur; boot anında o koşul
    tanımı gereği yoktur. Bir kez ölçememek 'iş bitti' değildir.
    """
    d = _SahteDugum(["olcemedi", "olcemedi", "olctu"])
    mod._servis_dongusu(d)
    assert d.cagri == 3, "araç ilk ölçememede çıkmış — 17.08 kusuru geri geldi"
    assert d._ayar_yazildi is True


def test_YEDEK_deger_ISI_BITIRMEZ(mod):
    """Kaptanın sorusu: 'yedek değer yazmak işin bittiği anlamına gelir mi?'

    Hayır. Yedek 'bilinen-kötüye düşmedim' demek; gerçek ölçüm hâlâ borçtur.
    """
    d = _SahteDugum(["yedek", "yedek", "olctu"])
    mod._servis_dongusu(d)
    assert d.cagri == 3, "yedek değer yazınca servis işi bitmiş saymış"


class _SahteYazici:
    """`_yazma_asamasi`nın dokunduğu yüzeyin tamamı — GERÇEK karar yolu."""

    def __init__(self, ozgun):
        self._ozgun = dict(ozgun)
        self._ayar_yazildi = False
        self._geri_yuklendi = False
        self._yedek = "/dev/null"
        self.yazilanlar = {}
        self.geri_yukleme_sayisi = 0
        self.satirlar = []

    def _bas(self, tur, metin):
        self.satirlar.append((tur, metin))

    def _yaz(self, ad, deger):
        self.yazilanlar[ad] = deger

    def geri_yukle(self):
        self.geri_yukleme_sayisi += 1


def test_GERCEK_yolda_yedek_deger_bayragi_KALDIRMAZ(mod):
    """Kaptanın sorusu, gerçek yazma yolunda: yedek 'iş bitti' değildir.

    Yüklü ATC_ACCEL_MAX bilinen-kötü (1,0); ölçüm yok. Araç yedeği yazmalı
    AMA `_ayar_yazildi` kalkmamalı — yoksa servis çıkar ve gerçek ölçüm
    hiç yapılmaz.
    """
    s = _SahteYazici({mod.P_ACCEL: 1.0, mod.P_EXPO: 0.0})
    mod.PlantAyar._yazma_asamasi(s, {})
    assert s.yazilanlar == {mod.P_ACCEL: mod.BASLANGIC[mod.P_ACCEL]}
    assert s._ayar_yazildi is False, (
        "yedek değer 'iş bitti' sayıldı — servis çıkar, gerçek ölçüm "
        "hiç yapılmaz (kaptanın sorusunun tam cevabı)")


def test_GERCEK_olcum_bayragi_KALDIRIR(mod):
    s = _SahteYazici({mod.P_ACCEL: 1.0, mod.P_EXPO: 0.0})
    mod.PlantAyar._yazma_asamasi(s, {mod.P_ACCEL: 0.31})
    assert s.yazilanlar == {mod.P_ACCEL: 0.31}
    assert s._ayar_yazildi is True


def test_yuklu_deger_yedege_ESITSE_TEKRAR_YAZILMAZ(mod):
    """Servis her turda aynı değeri FC'ye (EEPROM) yeniden yazmamalı."""
    s = _SahteYazici({mod.P_ACCEL: mod.BASLANGIC[mod.P_ACCEL], mod.P_EXPO: 0.0})
    mod.PlantAyar._yazma_asamasi(s, {})
    assert s.yazilanlar == {}, "aynı değer her turda EEPROM'a yeniden yazılıyor"
    assert s.geri_yukleme_sayisi == 1
    assert s._ayar_yazildi is False


def test_SIGTERM_ANINDA_cikar_ve_GERI_YUKLER(mod):
    d = _SahteDugum([KeyboardInterrupt("SIGTERM"), "olctu"])
    mod._servis_dongusu(d)
    assert d.cagri == 1, "SIGTERM'den sonra yeni deneme BAŞLATILMAMALI"
    assert d.geri_yukleme_sayisi == 1
    assert d._ayar_yazildi is False


def test_hata_GERI_YUKLEYIP_yeniden_dener(mod):
    d = _SahteDugum([RuntimeError("ATC_DECEL_MAX beklenmedik tip (0)"), "olctu"])
    mod._servis_dongusu(d)
    assert d.cagri == 2
    assert d.geri_yukleme_sayisi == 1, "hatada özgün değerlere dönülmedi"


def test_olctukten_SONRA_cikar_sonsuz_dongu_YOK(mod):
    d = _SahteDugum(["olctu"])
    mod._servis_dongusu(d)
    assert d.cagri == 1


def test_yeni_denemede_ORNEK_KOVALARI_temizlenir(mod):
    """Eski turun gürültüsü yeni turun ölçümüne karışmamalı."""
    d = _SahteDugum(["olcemedi", "olctu"])
    d._ivme = [9.0] * 10
    d._gurultu = [9.0] * 10
    mod._servis_dongusu(d)
    assert d._ivme == [] and d._gurultu == [], "kovalar turlar arası taşıyor"


# ── ③ 'henüz gelmedi' ≠ 'FC'de yok' ────────────────────────────────────────

class _SahteOkuyucu:
    def __init__(self, bekle, gelis_denemesi, yok=()):
        self.bekle = bekle
        self._gelis = gelis_denemesi
        self._yok = set(yok)
        self._deneme = 0
        self.satirlar = []

    def _bas(self, tur, metin):
        self.satirlar.append((tur, metin))

    def _oku(self, ad, hosgor=False):
        if ad in self._yok:
            return None
        self._deneme += 1
        if self._deneme < self._gelis:
            return None            # MAVROS henüz FC'den indirmedi
        return 0.30


def test_DECEL_gecikmesi_KALICI_ATLAMAYA_donusmez(mod, monkeypatch):
    """17.08'de ATC_DECEL_MAX 1 sn erken sorulduğu için koşum boyu atlandı."""
    monkeypatch.setattr(mod.rclpy, "spin_once", lambda *a, **k: None)
    monkeypatch.setattr(mod.time, "sleep", lambda *_a: None)
    s = _SahteOkuyucu(bekle=True, gelis_denemesi=6)
    d = mod.PlantAyar._ozgunleri_oku(s, (mod.P_EXPO, mod.P_ACCEL),
                                     istege_bagli=(mod.P_DECEL,))
    assert d[mod.P_DECEL] == 0.30, "gecikme 'FC'de yok' sanıldı"


def test_GERCEKTEN_yok_olan_parametre_ATLANIR(mod, monkeypatch):
    """Zorunlular geldiği hâlde boş dönen parametre gerçekten yoktur."""
    monkeypatch.setattr(mod.rclpy, "spin_once", lambda *a, **k: None)
    monkeypatch.setattr(mod.time, "sleep", lambda *_a: None)
    s = _SahteOkuyucu(bekle=True, gelis_denemesi=1, yok={mod.P_DECEL})
    d = mod.PlantAyar._ozgunleri_oku(s, (mod.P_EXPO, mod.P_ACCEL),
                                     istege_bagli=(mod.P_DECEL,))
    assert mod.P_DECEL not in d
    assert any("FC'de YOK" in m for _t, m in s.satirlar)


def test_servis_kipinde_bekleme_SINIRSIZ(mod):
    """Tekne saatler sonra suya girebilir — elle koşumun sabrı ölçüt olamaz."""
    kaynak = open(_BETIK, encoding="utf-8").read()
    assert "if not self.bekle and gecen > PARAM_BEKLEME_AZAMI" in kaynak, (
        "zaman aşımı servis kipinde de uygulanıyor — araç suya girmeden ölür")


def test_BEKLE_bayragi_SERVIS_DONGUSUNE_baglanmis(mod):
    """`--bekle` (servis kipi) tek koşuma değil, döngüye bağlanmalı.

    Birim `--bekle` ile koşuyor; `main()` burada `calistir()` çağırırsa
    döngü hiç devreye girmez ve 17.08 kusuru aynen geri gelir.
    (Kaynak denetimi: `main()` ROS'suz koşturulamaz.)
    """
    import ast
    agac = ast.parse(open(_BETIK, encoding="utf-8").read())
    ana = next(d for d in agac.body
               if isinstance(d, ast.FunctionDef) and d.name == "main")
    cagrilar = [n.func.id for n in ast.walk(ana)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)]
    assert "_servis_dongusu" in cagrilar, (
        "main() servis kipinde _servis_dongusu çağırmıyor — araç tek "
        "koşumdan sonra ölür, systemd de Restart=on-failure ile diriltmez")
