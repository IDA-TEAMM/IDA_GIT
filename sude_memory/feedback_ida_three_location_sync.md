---
name: feedback-ida-three-location-sync
description: "IDA USV node dosyalari 4 farkli yerde tutuluyor, her degisiklikte hepsi guncellenmeli"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 69473ebd-427a-4cc3-b61f-3fe1132cd962
---

`ida_topics_yeni` altindaki node dosyalarinin (`sensor_node.py`, `perception_node.py`, `decision_node.py`, `control_node.py`, `telemetri_node.py`, `kamera_kayit_node.py`, `local_map_node.py`, `gps_imu_driver_node.py`, `oakd_driver_node.py`, `livox_driver_node.py`, `sistem_baslat.sh`) **4 ayri kopyasi** var ve kullanici bunlarin hepsinin senkron kalmasini istiyor:

1. **Git repo (asil):** `~/ros2_ws/src/ida_topics_yeni/ida_topics/<node>.py`
2. **`~/Masaüstü/IDA_YAZILIM/nodes/<node>.py`** — ayni dosya adiyla.
3. **`~/Masaüstü/<node>.py`** (DUZ isim, alt klasor yok) — 2026-07-13'te fark edildi, önceden bu memory'de yoktu. `control_node.py` icin istisna: bu konumdaki dosyanin adi `control_node_mavros.py` (farkli isim ama icerik `control_node.py` ile ayni sey, o dosyaya kopyalanmali).
4. **`~/Masaüstü/<node>_GUNCEL_<tarih>.py`** (ör. `decision_node_GUNCEL_20260625.py`) — sadece 3 dosya icin var: `decision_node`, `perception_node`, `sensor_node` (digerlerinin GUNCEL suffixli kopyasi yok, olusturmaya gerek yok). Dosya adindaki tarih eski/sabit kalabilir, degistirmeye gerek yok, mevcut dosya adina uzerine yaz.

**Not:** `~/Masaüstü/topics/<node>.py` klasoru bu senkronun DISINDA — Mayis 2026'dan kalma eski/farkli bir kopya, güncel node revizyonlariyla eslesmiyor, dokunulmuyor.

**Git icindeki gizli 5. kopya — COZULDU (2026-07-12):** Repo kokunde (`~/ros2_ws/ida_topics_yeni/`, src/ ONCESI) hayalet bir tracked dizin vardi, bazi commit'ler yanlislikla oraya gidiyordu. 02d61a0 commit'iyle tamamen silindi. Bundan sonra `git add`/`git commit` her zaman `src/ida_topics_yeni/...` path'iyle yapilmali, repo kokunde boyle bir klasor bir daha OLUSTURULMAMALI.

**Why:** Kullanici 2026-07-12'de acikca "bu yerlerde dosyalar bulunuyor ve her degisiklikte hepsini guncelleyecegiz" dedi. Git repo tek "source of truth" degil — Masaustu kopyalari da elle senkron tutuluyor (muhtemelen yedekleme/offline erisim icin). 2026-07-13'te 3. lokasyonun (duz Masaustu kopyalari) varligi tesadufen fark edildi — bunu kacirmak, dosyanin "senkron" sanilip aslinda eski kalmasi riski tasir, dikkatli olunmali.

**How to apply:** `src/ida_topics_yeni/ida_topics/` altinda herhangi bir node dosyasi veya `sistem_baslat.sh` degistiginde (Edit/Write ile), degisiklik onaylandiktan sonra ayni dosyayi otomatik olarak diger UC konuma da kopyala (`cp`) — kullanicinin ayrica hatirlatmasini bekleme. Her ihtimale karsi, bir node uzerinde calismaya baslamadan once o node'un tum Masaustu kopyalarinin (duz + IDA_YAZILIM/nodes + GUNCEL varsa) guncel olup olmadigini `diff -q` ile kontrol et, sadece IDA_YAZILIM/nodes'u kontrol etmek yetersiz kalabilir (2026-07-13'te tam olarak bu sekilde bir kopya atlandi).
