---
name: bekleyen-girdiler-isaret
description: Dışarıdan bilgi/dosya/ölçüm bekleyen tüm açık işlerin listesi repoda tutuluyor — girdap-ida-algi/docs/bekleyen_girdiler.md
metadata: 
  node_type: memory
  type: reference
  originSessionId: 7f762d05-ebe8-4ea3-9b25-133521740824
---

Eyüp'ün isteğiyle (2026-07-10) **henüz belirlenmemiş donanımsal konumlar, model dosyaları ve saha verileri** tek bir notta toplandı:

**`/home/eyup/girdap-ida-algi/docs/bekleyen_girdiler.md`** (commit `9fe8ae1` + `bd62d7c`, push'lu)

**GÖNDERİLEBİLİR FORM (2026-07-11, commit `e6e55ef`): `docs/olcum_formu.md`** — Eyüp "su nerde duruyor / lidar height sormuştun, kaydet" deyince §A doldurmalık forma çevrildi: Livox `h` + x/y (nasıl ölçüleceğiyle: tekne YÜKLÜ suda, şerit metre, ±5 cm), base_link kararı (öneri: su hattı + tekne merkezi), OAK montaj (yükseklik/ofset/pitch/yaw), beam + thruster geometrisi, IMU konumu, FC failsafe paramları (FS_ACTION/FS_TIMEOUT/FS_GCS_ENABLE/FS_THR_ENABLE/FS_CRASH_CHECK) + RC bandı. Mekanik/FC ekibine olduğu gibi gönderilir, dolunca değerler hardware.launch TF'lerine + F5.1 filtresine + MPPI marjına işlenir.

Her madde şu şablonda: *ne bekleniyor, kimden, neyi bloke ediyor, geldiğinde tam olarak ne yapılacak.* Bölümler: A) mekanik ölçüler, B) model & eğitim verisi, C) takım arkadaşı, D) ölçüm/kalibrasyon, E) yarışma günü gelecek veriler.

## Sahaya çıkmadan cevaplanması ZORUNLU üç soru

1. **Livox Mid-360 montaj yüksekliği `h`** (su hattından) — bilinmezse `/perception/obstacle_map` boş kalabilir, MPPI dubaların içinden geçer. Bkz. [[girdap-decision-entegrasyon]] F5.1.
2. **Jetson'daki NN Archive'ın sınıf sırası** — bkz. [[yolo-model-durumu]].
3. **`base_link` orijini nerede?** (su hattı / güverte / IMU) — her geometrik hesap buna dayanıyor; şu an tüm static TF'ler 0,0,0 ve hiçbir node tf2 okumuyor.

**Eyüp'ün elindeki iş (2026-07-11 itibarıyla):** `olcum_formu.md`'yi mekanik+FC ekibine GÖNDERMEK. Dolu form gelince değerler koda işlenecek: hardware.launch static TF'leri + F5.1 lidar_height_m (üreteç+testlerle AYNI commit) + MPPI obstacle_margin. Yeni oturum formun dolu gelip gelmediğini SORARAK başlayabilir.

Kredi bittiğinde ya da yeni oturumda: önce bu notu oku, sonra devam et.

## 🔴 EYÜP TEYİDİ (2026-07-14): mekanik/donanımsal konum değerleri RASTGELE/PLACEHOLDER — topluca düzeltilecek

Eyüp açıkça söyledi: koddaki konum verileri vb. mekaniksel/donanımsal değerler gerçek ölçüm OLMADAN rastgele atanmıştı; hepsi tekrar düzeltilmeli. Kapsam (bilinenler):
- **Static TF'ler 0,0,0** (hardware.launch) — LiDAR/kamera/IMU montaj ofsetleri gerçek değil; hiçbir node tf2 okumuyor ama F5.1 düzeltmesi + füzyon geometrisi bu değerlere bağlanacak.
- **Livox `h`** (su hattından yükseklik) + x/y ofseti — F5.1 blokeri.
- **base_link orijini** kararlaştırılmadı (öneri: su hattı + tekne merkezi).
- **OAK montaj** (yükseklik/ofset/pitch/yaw) — bearing/mesafe hesabına girer.
- MPPI `obstacle_margin`, min_range, IMU konumu — ölçümle birlikte değerlendirilecek.
- `dynamics.yaml` TÜMÜYLE UYDURMA sayılacak (Eyüp 2026-07-14: arkadaşın verileri de uydurma, hiçbirine bakılmadı): `thruster_spacing`, `max_thrust`, RPM→N tablosu, `mass`, `inertia_z`, `Xu/Yv/Nr` — istisna YOK, hepsi ölçüm/kalibrasyon listesinde ([[tekne-cift-motor]]).

Düzeltme günü tek kaynak: dolu `olcum_formu.md` → değerler hardware.launch TF'leri + F5.1 `lidar_height_m` + MPPI marjına AYNI oturumda işlenir (F6.2 reçetesi: üreteç+testlerle aynı commit). Bu yapılmadan gerçek-veri saha testlerinin geometrik sonuçlarına güvenme.
