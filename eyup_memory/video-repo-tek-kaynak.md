---
name: video-repo-tek-kaynak
description: "2026-07-14 Eyüp kararı: video işi YALNIZ github.com/EyupEker1/girdap-video reposunda; girdap-decision fork+yedek artık kullanılmayacak, çift-push maddesi İPTAL"
metadata: 
  node_type: memory
  type: project
  originSessionId: 1a4a984d-3a21-47c6-878a-21b6e630b350
---

**Eyüp kararı (2026-07-14): eski repolar (EyupEker1/girdap-decision fork + girdap-decision-yedek) artık KULLANILMAYACAK. Video için tek repo: `github.com/EyupEker1/girdap-video`.** Çalışma EyupEker1 hesabındaki repolarda sürecek.

- Bu kararla [[jetson-sifirlama-2026-07-13]]'teki "çift-push origin geri kurulacak" açık maddesi **İPTAL** (zaten kurulmuştu ama artık gereksiz; ~/ros2_ws/src/girdap-decision origin'inde mükerrer push URL'leri duruyor — zararsız, repo emekli).
- **girdap-video yapısı** (`~/girdap-video`, origin senkron, son commit `98b5386`):
  `karar/` = girdap-decision @ `15dc238` **+ F-M.3 yaması (2026-07-14)** ·
  `kontrol-listesi.md` · `testler/video_testleri.sh` (Jetson 12/12 ✅) ·
  `test-plani.md`. Güncelleme yalnız bilinçli kararla (README'de komut/not).
- ✅ **DÜZELTME AKIŞI İLK KEZ UYGULANDI (2026-07-14, F-M.3):** runtime klonda
  TDD düzeltme + lokal commit `8050ceb` (push YOK — repo emekli) → 3 dosya
  girdap-video/karar'a kopya → commit+push `98b5386` + README dondurma notu
  güncellendi. Suite yeni taban **267/2**. Akış çalışıyor; symlink kurulum
  sayesinde rebuild gerekmedi.
- ✅ **KOD AYNILIĞI DOĞRULANDI (2026-07-14, ince ayrıntı):** `~/girdap-video/karar`
  ↔ `~/ros2_ws/src/girdap-decision` diff'i SIFIR kod farkı (her ikisi `15dc238`).
  Runtime zinciri de teyitli: `girdap-karar.service` → install → build →
  **HEPSİ SYMLINK** (colcon --symlink-install; egg-link + config/launch link'leri
  kaynağa gider; import çözümü canlı doğrulandı) → teknede koşan .py = kaynak klon
  = girdap-video/karar. Yani .py/config düzeltmesi kaynak klona işlenirse rebuild
  GEREKMEZ, servis restart yeter.
- ✅ `docs/hata_defteri.md` + `scripts/girdap-karar.service` girdap-video/karar'a
  kopyalanıp push'landı (`75196a9`, 2026-07-14, Eyüp onayıyla) — artık GitHub'da
  güvende. Hata defterinin CANLI kopyası hâlâ ros2_ws klonunda; güncellenince
  girdap-video'ya da kopyalanmalı.
- Video dönemi düzeltme akışı (önerilen): düzeltme `~/ros2_ws/src/girdap-video`…
  değil — düzeltme runtime klonda yapılır+test edilir → aynı dosya
  `~/girdap-video/karar/`'a kopyalanır → girdap-video commit+push (README'deki
  dondurma commit notu güncellenir).
- AÇIK SORU: T1 (yarışma, 30 Eylül) geliştirmesi hangi repoda sürecek —
  video sonrası Eyüp'e sorulacak. [[hata-defteri-kurali]]'ndaki defter yolu da
  (girdap-decision/docs/hata_defteri.md) o karara göre taşınabilir.
