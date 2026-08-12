# FC parametre dökümleri — geçmiş

**GÜNCEL DOSYA MASAÜSTÜNDE:** `canli_2026-08-10_GUNCEL.param`
Bu klasör yalnız **geçmiş** tutar. Karşılaştırma aracı:
`~/ros2_ws/son_kodv2/karar/scripts/param_kiyasla.py`

| Dosya | Ne | Not |
|---|---|---|
| `canli_2026-08-10_baskasi_degistirmis.param` | 09.08'de **başkası** değiştirdikten sonraki hâl | 07.08'e göre 32 fark |
| `HEDEF_yuklenecek_2026-08-10.param` | Yazılmak üzere hazırlanan hedef | Artık GÜNCEL ile birebir aynı |
| `canli_2026-08-07d.param` | 🔴 **ADI YANILTICI** — tarihi **10.08**'dir | Yazma sonrası döküm; GÜNCEL bunun kopyası |
| `canli_2026-08-07c.param` | Bizim eski 07.08 temeli | ⚠️ **GERİ YAZILMAZ** — içinde `BATT_VOLT_MULT=18.18` hatası var (batarya 57.7 V okuyordu) ve bayat jiroskop kalibrasyonu |
| `canli_2026-08-07b/.param`, `canli_2026-08-06.param`, `DFJKJSLF.param` | Daha eski | — |
| `parametrelerDefMP.param` | 🔴 **BU TEKNEYİ HİÇ TARİF ETMİYOR** | 146 parametrede farklıydı; buna dayanan tüm "doğrulandı" iddiaları geri alındı |

## Kural

Bir `.param` dosyasını "mevcut değerler" diye kullanmadan önce **o araçtan** ve
**güncel** olduğunu doğrula. Tekneye bağlanıp taze döküm al.

Gerekçelerin tamamı: `son_kodv2/karar/docs/fc_param_uzlasma_2026-08-10.md`

## 2026-08-12 09:52 — `fc_parametreler_2026-08-12_0952_SON_HAL.param` ⇐ GÜNCEL

**Bu dosya artık geçerli SON HÂL.** Önceki (`..._2026-08-12_SON_HAL.param`,
01:18) arşiv sayılır.

### Kasıtlı değişiklik — SD kart loglama yükü (2 parametre)

| Parametre | Eski | Yeni | Neden |
|---|---|---|---|
| `LOG_DARM_RATEMAX` | 0 | **5** | Arm DEĞİLKEN log hızı 5 Hz'e kısıldı |
| `LOG_FILE_RATEMAX` | 0 | **25** | Arm İKEN her mesaj tipi 25 Hz'e kısıldı |

Sebep: kartın yazma hızı **10,2 MB/s** ölçüldü (Class 10 tabanı) ve tek bir
oturumda **553 MB** log yazılmıştı — hepsi araç arm DEĞİLKEN. `PreArm:
Logging failed` bundan geliyordu. Kalıcı çözüm hızlı kart (U3/A1, sipariş
edildi); bu ikisi o gelene kadarki geçici kısıt.

**`LOG_FILE_BUFSIZE` 400 yapılmadı, 200'de BIRAKILDI.** Mission Planner
"out of range" uyarısı verdi. Mevcut 200 zaten MP'nin bildiği aralığın (4-64)
üstünde — yani metadata eski ve 400'ün kartta gerçekten ayrılabileceğini
doğrulayamıyoruz. Ayrılamazsa ArduPilot loglamayı hiç başlatamayabilir, yani
çözmeye çalıştığımız arızayı geri çağırırdık. Kazanç da marjinaldi: tampon
yalnız kartın takıldığı anı yutar, hız kısıtları ise veri ÜRETİMİNİ azaltır.

**`LOG_DISARMED` 1 KALDI, `LOG_BITMASK` 65535 KALDI** — bkz. `hata_defteri.md`:
`LOG_DISARMED=0` önerim geri çekildi (araç arm olmuyorken tezgâh logu tek veri
kaynağımızdı); bitmask'e dokunmak bit anlamları sürümler arası değiştiği için
veriyi sessizce kaybettirme riski taşıyor.

### Kendiliğinden değişenler (12 satır — MÜDAHALE DEĞİL)

`BARO1_GND_PRESS` (her açılışta yer basıncı) · `COMPASS_DEC` (GPS konumundan
manyetik sapma, fark 2e-7) · `INS_GYR*OFFS` + `INS_GYR*_CALTEMP` (açılışta
jiroskop kalibrasyonu; 40,5 → 42,6 °C, cihaz ısınmış) · `STAT_BOOTCNT`
193→197 · `STAT_RUNTIME`.

⇒ **Başka hiçbir parametre değişmemiş.** Bu kontrol boşuna değil: 11.08'de
`SERIAL1_BAUD`'un sessizce 57→19 düştüğü tam böyle yakalanmıştı.

### 🔑 Bu döküm ayrıca PUSULA KALİBRASYONU ÖNCESİ son hâldir

Kayıt için: `COMPASS_OFS_X = -37.99903`, `COMPASS_OFS_Y = 36.8756`.
Açık alanda Large Vehicle MagCal yapıldıktan sonra alınacak dökümle
karşılaştırılıp "kapalı alan kalibrasyonu bozuktu" tezi sayısal
doğrulanacak.
