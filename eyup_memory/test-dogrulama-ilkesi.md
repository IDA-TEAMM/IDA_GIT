---
name: test-dogrulama-ilkesi
description: "GİRDAP: her kod değişikliği testle doğrulanır (TDD kırmızı→yeşil); ortam izin vermiyorsa py_compile'a düş ve BUNU AÇIKÇA İŞARETLE — asla test edilmemiş şeyi test edilmiş gibi sunma"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 2f2e1ca8-ee68-4e07-ac8c-e13cc93bdf0d
---

Eyüp (2026-07-11): "her şeyi test ederek yaz" — bu bir tercih değil İLKE.

**Why:** Yarışma kodu; yanlış-pozitif "düzelttim" beyanı sahada tekneyi
gömebilir. Faz 16'da kanıtlandı: test altyapısı bile "yeşil görünen sıfır
koşu"ya düşebiliyordu (launch_testing tuzağı, F16.1) — koşturduğunu SANMAK
ile koşturmak farklı. Ayrıca F12.1 vakası: düzeltme "uygulandı" sanılıp
uygulanmamıştı; test/doğrulama kaydı olmayan iş yok hükmündedir.

**How to apply:**
1. Kod düzeltmesi = TDD: önce hatayı yakalayan test (kırmızı), sonra düzeltme
   (yeşil), sonra TÜM suite. girdap-decision'da komut: `source
   /opt/ros/humble/setup.bash && export PYTHONPATH=/home/eyup/girdap-decision:
   /home/eyup/girdap-decision/ros2_ws/src/girdap_decision:$PYTHONPATH &&
   python3 -m pytest prototype/tests/ -q` → beklenen taban: **156 passed /
   8 gerekçeli skip** (2026-07-11; sayı büyüyebilir, DÜŞMEMELİ).
2. Ortam engeli varsa (bu makinede mavros_msgs/vision_msgs YOK, scipy ABI
   kırık) geri düşüş sırası: ilgili çekirdek testleri → `py_compile` →
   salt okuma. Hangi seviyede kaldıysan commit'e ve denetim dokümanına
   AÇIKÇA yaz ("py_compile — mavros yok, Jetson'da doğrulanacak" gibi).
3. Doküman/config değişikliğinde koşacak kod yoksa her iddiayı birinci
   kaynağa karşı doğrula (koddaki gerçek değer, dosyanın varlığı, şartname
   maddesi — [[sartname-once-kural]]).
4. Rapor dili dürüst: "test edildi" yalnız gerçekten koşan test için;
   davranış değiştiren düzeltmede testi doğru davranışa güncelle (buggy
   davranışı dondurmak için değil — bearing/F6.1 dersi).
5. Test edilemeyen işler [[bekleyen-girdiler-isaret]] gibi saha/Jetson
   listelerine düşer, sessizce "bitti" sayılmaz.

İlgili: [[girdap-decision-entegrasyon]], [[jetson-yuk-kod-sadeligi]],
[[sik-memory-checkpoint]].
