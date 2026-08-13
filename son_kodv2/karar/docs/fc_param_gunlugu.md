# FC parametre günlüğü

> `scripts/fc_param_denetle.py` referansa işlediği ÖLÜMCÜL OLMAYAN
> değişiklikleri buraya ekler. Ölümcül sapmalar buraya GİRMEZ; onlar
> referansa hiç işlenmez, her koşumda yeniden bildirilir.

## pixdesuanolan13-agustos-2026-saat16.param → referans (3 değişiklik)

| parametre | eski | yeni |
|---|---|---|
| `LOG_DARM_RATEMAX` | 5.0 | 0.0 |
| `LOG_FILE_RATEMAX` | 25.0 | 0.0 |
| `MOT_THR_MIN` | 10.0 | 0.0 |

---

## `MOT_THR_MIN` — tekrarlayan regresyon (A3.2)

| Tarih | Bulundu | Yazıldı |
|---|---|---|
| 10.08 | 0 | 10 (`fc_param_uzlasma_2026-08-10.md` §geri alınan 5) |
| 13.08 | **0 yine** | denetleyiciye eklendi, yükleme dosyasında düzeltiliyor |

**Değerin dayanağı sağlam:** *"düşük hız kalkış eşiği; **su testinden geçti**.
0'da ince manevra (kapı ortalama) authority kaybı."* Tezgâh ölçümü de var
(`ardurover_bench.parm.md`): iki motor da %1 güçte dönmeye başlıyor, ölü
bantlar simetrik — yani asıl sorun asimetri değil, 0'da küçük komutların
motoru hiç döndürmemesi.

🔑 **İkinci kez sıfırlanması "yanlışlıkla" açıklamasını zayıflatıyor.**
Parametre sorumlusuna *neden* sıfırlandığı sorulmalı: bilerek mi (bir
gerekçesi mi var), yoksa bir varsayılan yükleme mi üzerine yazıyor? Sorulmazsa
üçüncü kez olur ve o sefer suda olur — belirtisi "tekne küçük düzeltmelere
cevap vermiyor, kapı ortasını tutturamıyor" şeklinde görünür ve PID sanılır.
