---
name: ida-project
description: GİRDAP İDA projesinin genel tanımı — TEKNOFEST 2026 İnsansız Deniz Aracı yarışması
metadata: 
  node_type: memory
  type: project
  originSessionId: b947f7ad-b0f6-4a34-b949-cbbe03fd8065
---

**GİRDAP İDA** — TEKNOFEST 2026 İnsansız Deniz Aracı (İDA) Yarışması projesi (Takım ID 989124). Otonom su üstü aracı + destekleyici bir İHA (quadcopter).

**Platform:** Katamaran (çift tekneli) gövde, dümensiz **diferansiyel sürüş** (sağ/sol itici devir farkıyla manevra). İtki: Mucif Mitras sualtı iticileri. Gövde ASA filament (FDM, %30 Gyroid), alüminyum iskelet, PU köpük. Boyut 0,78×1,04×0,52 m, kütle ~11,8 kg. Enerji: Aspilsan INR18650A28 4S7P (~290 Wh, ~63 dk görev).

**3 parkur / görev senaryosu:**
- **Parkur-1:** Nokta takip (waypoint slalom, 16 turuncu duba)
- **Parkur-2:** Engelli ortamda nokta takip (dinamik engel kaçınım, ön harita YOK — yalnız gerçek zamanlı sensör)
- **Parkur-3:** Kamikaze angajman (İHA hedef renk plakasını tespit edip MAVLink ile İDA'ya iletir, İDA doğru renkli hedefe fiziksel temas)
- Parkurlar arası geçiş otomatik (kullanıcı girişi olmadan); görev başladıktan sonra dışarıdan müdahale kapalı.

**İHA:** Raspberry Pi 5 + Pixhawk 6C Mini (ArduCopter) + RPi HQ Kamera (IMX477) + 4× RS2205 motor. ~1,1 kg. Görevi: Parkur-3'te hedef renk tespiti.

İki ana doküman Desktop'ta: Kritik Tasarım Raporu PDF'i ("Girdap İnsansız Deniz Aracı Takım ID 989124.pdf", 31 sayfa) ve yazılım analiz raporu (~/Downloads/GIRDAP_Yazilim_Analiz_Raporu.docx).

İlgili: [[ida-software-status]] · [[ida-sim-workspace]] · [[ida-hardware]] · [[user-role]]
