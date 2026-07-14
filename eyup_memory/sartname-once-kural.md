---
name: sartname-once-kural
description: "GİRDAP projesinde her kod/karar şartnameye bağlı gerekçelendirilecek — öncelik puan/ceza/eleme tablosundan çıkar, mühendislik sezgisinden değil"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7f762d05-ebe8-4ea3-9b25-133521740824
---

Eyüp (2026-07-10): kod yazarken ve öncelik belirlerken **TEKNOFEST 2026 İDA şartnamesini sürekli akılda tut.**

**Why:** Bu projede "iyi mühendislik" ile "puan getiren iş" farklı şeyler. Öncelik sezgiden değil şartnameden çıkmalı. Somut örnekler bu projede:
- Otonomi videosu bir **eleme kapısı** (md 3, 3.3) — geçilmezse P1+P2+P3=0. Bu yüzden algıdaki en ağır bug'lar (F5.1 LiDAR, ters sınıf sırası) bile videodan SONRA gelir; video kodu (MAVROS, görev, MPPI kararlılığı) önce.
- md 3.3.1.1 "istemsiz hareket → video BAŞARISIZ" → MPPI kararlılığı ve FSM temiz-duruşu doğrudan eleme kriteri (F12.2, F13.1).
- Her eksik teslim dosyası **5 ceza puanı** (md 4.2) → Dosya-1/2/3 üretimi çökmemeli (F4.1, F2.1).
- MIN_GECIT=2 (md 5.5.2.4), geçit=kenar×kenar (G/KD tanımı), RAL kodları, MIN boyutlar — hepsi şartname sayısı.

**How to apply:**
1. Bir kod/öncelik kararı verirken önce sor: *bu şartnamenin hangi maddesine, hangi puana/cezaya/eleme koşuluna dokunuyor?* Cevabı bulguya/commit'e yaz (md numarasıyla).
2. Şartnameyi ezberden değil **birinci kaynaktan** oku: `/home/eyup/ida_sartname_2026.pdf` (Türkçe karakter araması tutmaz; "Dosya"/"mp4"/madde no ile ara). Ayrıntılar [[sartname-ida-2026]].
3. Öncelik katmanları: **T0 video (eleme, 21.07.2026)** > **T1 yarışma parkurları (30 Eylül)** > **T2 Parkur-3**. Bkz. [[girdap-decision-entegrasyon]] görev sırası.
4. Şartname bir şeyi zorunlu kılıyorsa kodda karşılığı var mı diye EŞLE (video 6 şartını koda eşleyince 3 boşluk çıktı — YKİ, görev yükleme, thrust kaydı).
5. Emin olmadığın şartname yorumunu "açık soru" işaretle, uydurma.
