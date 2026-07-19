# tum_loglar — 11 Pixhawk log'unun tamamı

`LOGS/00000001.BIN` – `00000011.BIN` için ayrı ayrı üretildi: `logNN_animasyon.html` (interaktif oynat/duraklat/scrub), `logNN_animasyon.png` (statik önizleme), `logNN_animasyon.mp4`, `logNN_animasyon.gif`.

Her biri THR/NTUN mesajlarından (hız, heading, ThrOut — gerçek vs. istek) yeniden üretildi. Gri bant = AUTO modda geçen süre.

| Log | Dosya | Süre | Modlar | AUTO var mı | AUTO aralıkları |
|---|---|---|---|---|---|
| 01 | 00000001.BIN | 226s | HOLD | ❌ | — |
| 02 | 00000002.BIN | 311s | MANUAL, HOLD | ❌ | — |
| 03 | 00000003.BIN | 275s | MANUAL, HOLD | ❌ | — |
| 04 | 00000004.BIN | 593s | HOLD | ❌ | — |
| 05 | 00000005.BIN | 507s | MANUAL | ❌ | — |
| 06 | 00000006.BIN | 1103s | MANUAL, HOLD, AUTO | ✅ | 745-759s |
| 07 | 00000007.BIN | 279s | MANUAL, AUTO | ✅ | 141-176s, 192-236s |
| 08 | 00000008.BIN | 2537s | MANUAL, AUTO, GUIDED | ✅ | 1532-1578s, 1777-1825s, 1921-1930s, 2448-2461s |
| 09 | 00000009.BIN | 2003s | MANUAL, AUTO | ✅ | 138-163s, 210-220s, 563-611s, 1903-1943s |
| 10 | 00000010.BIN | 518s | MANUAL | ❌ | — |
| 11 | 00000011.BIN | 590s | MANUAL, AUTO | ✅ | 210-228s, 311-366s, 522-590s |

**AUTO modda olunan loglar: 06, 07, 08, 09, 11.**
