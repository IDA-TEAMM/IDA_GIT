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

## pix_dokum_15agustos.param → referans (67 değişiklik)

| parametre | eski | yeni |
|---|---|---|
| `AHRS_COMP_BETA` | 0.1 | 0.10000000149011612 |
| `AHRS_RP_P` | 0.2 | 0.20000000298023224 |
| `AHRS_YAW_P` | 0.2 | 0.20000000298023224 |
| `ATC_BAL_D` | 0.03 | 0.029999999329447746 |
| `ATC_BAL_LIM_THR` | 0.6 | 0.6000000238418579 |
| `ATC_BAL_P` | 1.8 | 1.7999999523162842 |
| `ATC_BAL_PIT_FF` | 0.4 | 0.4000000059604645 |
| `ATC_SAIL_I` | 0.1 | 0.10000000149011612 |
| `ATC_SPEED_I` | 0.2 | 0.20000000298023224 |
| `ATC_SPEED_P` | 0.2 | 0.20000000298023224 |
| `ATC_STOP_SPEED` | 0.1 | 0.10000000149011612 |
| `ATC_STR_RAT_FF` | 0.2 | 0.20000000298023224 |
| `ATC_STR_RAT_I` | 0.2 | 0.20000000298023224 |
| `ATC_STR_RAT_P` | 0.2 | 0.20000000298023224 |
| `ATC_TURN_MAX_G` | 0.6 | 0.6000000238418579 |
| `AVOID_BACKUP_DZ` | 0.1 | 0.10000000149011612 |
| `BRD_HEAT_I` | 0.07 | 0.07000000029802322 |
| `BRD_VBUS_MIN` | 4.3 | 4.300000190734863 |
| `COMPASS_DIA2_X` | 1.030617 | 1.0306169986724854 |
| `COMPASS_DIA2_Y` | 1.087597 | 1.0875970125198364 |
| `COMPASS_DIA2_Z` | 1.045406 | 1.0454059839248657 |
| `COMPASS_DIA_X` | 0.9786924 | 0.9786924123764038 |
| `COMPASS_DIA_Y` | 0.9421124 | 0.942112386226654 |
| `COMPASS_DIA_Z` | 1.013652 | 1.013651967048645 |
| `COMPASS_ODI2_X` | -0.001209271 | -0.0012092710239812732 |
| `COMPASS_ODI2_Y` | -0.1081911 | -0.1081911027431488 |
| `COMPASS_ODI2_Z` | -0.09101866 | -0.0910186618566513 |
| `COMPASS_ODI_X` | 0.006007828 | 0.006007827818393707 |
| `COMPASS_ODI_Y` | 0.08542461 | 0.08542460948228836 |
| `COMPASS_ODI_Z` | 0.0630231 | 0.06302309781312943 |
| `COMPASS_OFS2_X` | 70.80528 | 70.80528259277344 |
| `COMPASS_OFS2_Y` | 25.89602 | 25.896020889282227 |
| `COMPASS_OFS2_Z` | 41.45361 | 41.453609466552734 |
| `COMPASS_OFS_X` | 22.8682 | 22.868200302124023 |
| `COMPASS_OFS_Y` | 25.0574 | 25.05739974975586 |
| `COMPASS_OFS_Z` | 2.645692 | 2.6456921100616455 |
| `CRUISE_SPEED` | 1.0 | 1.0499999523162842 |
| `CRUISE_THROTTLE` | 25.0 | 95.0 |
| `DOCK_STOP_DIST` | 0.3 | 0.30000001192092896 |
| `EK3_ABIAS_P_NSE` | 0.02 | 0.019999999552965164 |
| `EK3_ACC_P_NSE` | 0.35 | 0.3499999940395355 |
| `EK3_EAS_M_NSE` | 1.4 | 1.399999976158142 |
| `EK3_ERR_THRESH` | 0.2 | 0.20000000298023224 |
| `EK3_GBIAS_P_NSE` | 0.001 | 0.0010000000474974513 |
| `EK3_GYRO_P_NSE` | 0.015 | 0.014999999664723873 |
| `EK3_MAGB_P_NSE` | 0.0001 | 9.999999747378752e-05 |
| `EK3_MAGE_P_NSE` | 0.001 | 0.0010000000474974513 |
| `EK3_MAG_M_NSE` | 0.05 | 0.05000000074505806 |
| `EK3_TERR_GRAD` | 0.1 | 0.10000000149011612 |
| `EK3_VELD_M_NSE` | 0.7 | 0.699999988079071 |
| `EK3_VIS_VERR_MAX` | 0.9 | 0.8999999761581421 |
| `EK3_VIS_VERR_MIN` | 0.1 | 0.10000000149011612 |
| `EK3_WENC_VERR` | 0.1 | 0.10000000149011612 |
| `EK3_WIND_P_NSE` | 0.1 | 0.10000000149011612 |
| `FS_EKF_THRESH` | 0.8 | 0.800000011920929 |
| `INS_STILL_THRESH` | 0.1 | 0.10000000149011612 |
| `PSC_POS_P` | 0.2 | 0.20000000298023224 |
| `SR0_ADSB` | 0.0 | 10.0 |
| `SR0_EXTRA1` | 4.0 | 10.0 |
| `SR0_EXTRA2` | 4.0 | 10.0 |
| `SR0_EXTRA3` | 2.0 | 10.0 |
| `SR0_EXT_STAT` | 2.0 | 10.0 |
| `SR0_POSITION` | 2.0 | 10.0 |
| `SR0_RAW_CTRL` | 0.0 | 10.0 |
| `SR0_RAW_SENS` | 2.0 | 10.0 |
| `SR0_RC_CHAN` | 2.0 | 10.0 |
| `WP_SPEED` | 1.2 | 0.949999988079071 |
