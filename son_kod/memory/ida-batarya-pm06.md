---
name: ida-batarya-pm06
description: "İDA Pixhawk 6C batarya izleme — Daly BMS CAN çıkmazı, PM06 planı, kalan iş (14 Tem 2026)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 553533e9-dfeb-4586-bb0f-900a74403556
---

İDA'da Pixhawk 6C batarya izleme kurulumu (durum: 14 Temmuz 2026, yarım kaldı).

**Çıkmaz yol (tekrar denenmesin):** Daly BMS (mavi kasa, J tipi) CAN2 portuna bağlandı ama Daly DroneCAN konuşmuyor (proprietary CAN, ~250kbps vs DroneCAN 1Mbps) — bu yoldan asla veri gelmez. BMS bağımsız koruma olarak kalacak, CAN kablosu iptal.

**Karar:** Analog power module **Holybro PM06** kullanılacak (elde var). Batarya → PM06 → dağıtım/iticiler; 6-pin JST-GH → POWER1. Dikkat: PM06 sürekli ~60A — iticilerin toplam çekişiyle karşılaştırılmadı henüz.

**Parametreler (Pixhawk 6C):**
- `BATT_MONITOR=4` (Analog Voltage and Current; reboot şart)
- `BATT_VOLT_PIN=8`, `BATT_CURR_PIN=4` (6C POWER1 defaultları; QGC "Unknown: 8" ve "CubeOrange_PM2/Navigator" etiketleri yanıltıcı ama değerler doğru)
- `BATT_VOLT_MULT=18.182`, `BATT_AMP_PERVLT=36.364` (PM06 nominal)
- Temizlik: `BATT2_MONITOR=0`, `CAN_P2_DRIVER=0`

**Kaldığımız yer / sıradaki adım:** QGC Calculate penceresinde "Vehicle voltage" boş görünüyor. Teşhis: MAVLink Inspector → `SYS_STATUS.voltage_battery` — ~13200 ise QGC sorunu; 0/65535 ise fiziksel bağlantı eksik; mesaj hiç yoksa `SR1_EXT_STAT≥2`. En olası neden: PM06'nın POWER1'e fiziksel bağlantısının henüz yapılmamış olması (doğrulanamadı). Not: daha önce görülen 57.8V okuması, sensör takılı değilken ADC gürültüsüydü.

**Batarya:** 4S, multimetreyle 13.2V (~3.3V/hücre, neredeyse boş) — şarj edilecek. Min arming voltage 14.0V, bu seviyede arm etmez. Kalibrasyon: dolu bataryayla (~16.8V) ikinci doğrulama yapılacak.

İlgili: [[ida-hardware]], [[ida-e32-lora]] (telemetri SERIAL1 MicoAir)
