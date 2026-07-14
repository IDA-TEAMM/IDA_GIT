---
name: hata-defteri-kurali
description: "Yeni bulunan HER hata/bug debug verisiyle birlikte girdap-decision/docs/hata_defteri.md'ye yazılır — tek canlı hata dosyası; kod_denetimi.md yalnız arşiv"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 9ee3d9d9-28cd-4ab8-b918-cfa44962dee9
---

Eyüp (2026-07-14): hataların/bug'ların debug verileriyle birlikte doğrudan yazıldığı, YALNIZ hataları içeren TEK dosya istedi; yeni hata çıktığında oraya işlenecek.

**Dosya: `~/ros2_ws/src/girdap-decision/docs/hata_defteri.md`** (Jetson; PC klonunda aynı yol `docs/hata_defteri.md`). İçinde kayıt şablonu var (tarih/kod/belirti/debug verisi/kök neden/etki/durum + 🔴🟠🟡).

**Why:** Hatalar memory'ye, günlüklere ve kod_denetimi.md'ye dağılmıştı; test günlerinde "hangi hatalar açık" tek bakışta görünmüyordu. Ham log `~/girdap_logs/`'ta kalır, defterden yalnız yol verilir.

**How to apply:**
1. Oturumda yeni bir bug/hata bulunduğunda (test FAIL, node ölümü, saha anomalisi) düzeltmeden ÖNCE hata_defteri.md "AÇIK HATALAR"a şablonla kaydet — eldeki debug verisini (log yolu, komut çıktısı, ölçüm) o anda yaz.
2. Hata kapanınca girişi silme; "KAPANANLAR" tablosuna taşı (kod + tek satır + düzeltme commit'i).
3. `docs/kod_denetimi.md` derin denetim ARŞİVİ — yeni hata oraya değil deftere yazılır; defterden gerekirse ona link verilir.
4. Defter repoda → her commit fork+yedek'e push'lanır ([[girdap-decision-entegrasyon]] çift-push). İlgili: [[donanim-test-plani]], [[sik-memory-checkpoint]].
