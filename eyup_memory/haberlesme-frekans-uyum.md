---
name: haberlesme-frekans-uyum
description: "GİRDAP RF/frekans uyumu — telemetri MicoAir LR868 @ 868 MHz (YASAL, 'RFD868x' bayat); RC kumanda bandı AÇIK (yasal banda çekilecek); şartname md 4.1 yasak bantlar + 55 ceza"
metadata:
  node_type: memory
  type: reference
  originSessionId: 186fe6aa-acca-43d9-85c1-8354c18eb5eb
---

Şartname md 4.1 (birinci kaynak, V1.2, 2026-07-14 Jetson'da yeniden okundu): **İDA/İDA-YKİ/İHA/RC kumandalar VE telemetri modülleri dahilinde 2.4-2.8 GHz ve 5.15-5.85 GHz aralıkları YASAK.** Ceza: yasak frekans = **55 ceza puanı** (md 5.5.4.3.2) + teknik kontrolde (md 5.2) elenme. Ayrıca "haberleşme modüllerinin frekans kanalı SEÇİLEBİLİR olacaktır" (hakem yarışma kanalını atar). İlgili: [[sartname-ida-2026]], [[donanim-test-plani]].

## ✅ Telemetri modülü = MicoAir LR868 @ 868 MHz — YASAL (2026-07-14 teyit)
- Eyüp teyit etti: telemetri **868 MHz**. Web'den doğrulandı: MicoAir'in **LR868** serisi = 868 MHz LoRa telemetri radyosu (air+ground çift; LR900 varyantı 890-915 MHz — o da yasal). 868 hem 2.4-2.8 hem 5.15-5.85 dışında → **md 4.1 ihlali YOK, 55 ceza riski KALKTI.**
- **"RFD868x" tüm eski notlarda BAYAT** — Yahya markayı düzeltti (KOD_DEGISIKLIKLERI.txt 2026-07-14, 8 dosyada RFD868→MicoAir). Yahya frekanstan emin değildi ("bekliyor" yazmıştı, "2.4 GHz çıkarsa yasak" uyarısıyla); Eyüp 868'i teyit etti → marka(MicoAir)+frekans(868) çelişmiyor, aynı modül.
- Kanal-seçilebilir şartını da karşılıyor (MicoAir manuel kanal + AUTO en-az-girişim modu).
- **Not:** telemetri ≠ RC. QGC↔Pixhawk MicoAir 868 üzerinden; MAVROS Jetson↔Pixhawk USB (ttyACM0). İki ayrı MAVLink kanalı, `gcs_url=""` doğru.

## 🔴 RC kumanda bandı — AÇIK (yasal banda çekilecek, VİDEO-kritik)
- Çoğu hobi RC 2.4 GHz varsayılan → **YASAK**. RC md 4.1 kapsamında ("RC kumandalar VE telemetri").
- **Video da RC'yi zorunlu kılıyor:** md 3.3.1(4) "RC'den komut verilse de motorların dönmediği gösterilecek" → RC verici çekimde AÇIK+yayında olacak; md 3.3.1(3) manuel dönüş. Yani 2.4 GHz ise videoda ihlal görünür.
- Eyüp kararı (2026-07-14): **RC bandı değiştirilecek.** Seçenek elindeki donanıma bağlı (Eyüp'ün RC verici/alıcı modeli SORULACAK, henüz bilinmiyor):
  - Verici modül yuvalıysa (RadioMaster/Jumper/FrSky Taranis, JR/nano bay) → **900 MHz modül ekle**: ExpressLRS 900 (868/915, ~$30-50, LoRa) / TBS Crossfire (868/915) / FrSky R9 (868/915) + eşleşen alıcı. En ucuz+hızlı.
  - Sabit 2.4 GHz sistemse → komple 900 MHz set (verici+alıcı) — **tedarik zaman-kritik, 7 gün** (kargo+eşleştirme+prova).
  - Zaten 900 MHz ise → sadece teyit+kanal ayarı.
- Telemetri 868'de olduğu için RC'yi de 868/915'e alırsak aynı banda düşer — hakem ayrı frekans atar ama girişim için **kanalları ayrı seç** (ELRS/MicoAir destekler).

## 🟠 Jetson'da BLUETOOTH AÇIK — md 4.1 ihlali (2026-07-14 bulundu, VİDEO SONRASINA ertelendi)
- Ölçüm (2026-07-14, Jetson): `rfkill list` → WiFi **soft blocked: yes** ✓ ama **Bluetooth soft blocked: NO** → BT 2.4 GHz'de yayında. Üstelik **bağlı bir BT klavye** var (Eyüp kullanıyor).
- Şartname: md 4.1 2.4-2.8 GHz YASAK → 55 ceza + teknik kontrol riski. WiFi kapatılmış ama BT gözden kaçmış.
- **Eyüp kararı (2026-07-14): videodan SONRA halledilecek** — videoda BT görünmüyor, asıl yeri yarışma günü teknik kontrolü.
- Yapılacak (yarışma günü kontrol listesine): **USB klavye/fare temin** → `rfkill block bluetooth` (sudo GEREKMİYOR, denendi) → `rfkill list` ile teyit. systemd-rfkill durumu reboot'ta korur (klavye geri istenirse `rfkill unblock bluetooth`).
- Aynı denetimde kalıcı çıkanlar (her açılışta kurulum GEREKMİYOR): girdap-karar.service `enabled` ✓ · Livox statik IP artık NetworkManager `livox` profilinde (enP8p1s0, 192.168.117.50/24, autoconnect) → elle `ip addr add` BAYAT ✓ · OAK udev `80-movidius.rules` ✓ · kullanıcı `dialout`+`plugdev` ✓. Kalıcı OLMAYAN: `jetson_clocks` (her boot sıfırlanır).

## Güç kesme (md 4.2) — ilgili emniyet notu
"Motorlara giden sinyal akışını kesmek YETERLİ DEĞİL, gücün kesilmesi ŞART." Bizim yazılım KILL/disarm videoyu karşılar (motorlar dönmez), ama YARIŞMA emniyeti + teknik kontrol için **fiziksel güç kesen röle/mekanizma** gerekir (mekanik/FC işi). Kırmızı fiziksel anahtar + uzaktan güç kesme (YKİ ya da RC'den) zaten şart.
