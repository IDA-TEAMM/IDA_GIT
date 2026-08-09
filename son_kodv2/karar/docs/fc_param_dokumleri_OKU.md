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
