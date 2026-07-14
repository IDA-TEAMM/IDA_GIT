---
name: ida-e32-lora
description: "İDA telemetri için E32 (433MHz LoRa) modül çifti kurulumu, çalışan ayarlar ve iki kritik tuzak"
metadata: 
  node_type: memory
  type: project
  originSessionId: cca7794d-fc15-46cd-972f-bcb0c77d90cb
---

**GÜNCELLEME (2026-07-13):** Araç telemetrisinde artık E32 KULLANILMIYOR — MicoAir telemetri modülü takılı (SERIAL1: 57600 + MAVLink2, standart, air-rate derdi yok). **Frekans: 868 MHz (Yahya teyit etti, 2026-07-14) → şartname md 4.1 yasak aralıklarının DIŞINDA, uygun ✓.** Kalan: kanal seçilebilirlik teyidi (hakem frekans tahsisi için) + RC kumanda frekansı hâlâ açık. Aşağıdaki E32 kayıtları eski/yedek çalışma olarak duruyor.

GİRDAP İDA telemetri/komut hattı için EBYTE **E32 433MHz** modül çifti kullanılıyor (bkz [[ida-project]], [[ida-hardware.md]]). 2026-07-05'te iki modül arası çift yönlü haberleşme çalışır hale getirildi.

**Çalışan E32 ayarları (fabrika varsayılanı, iki modül de aynı):** `C0 00 00 1A 17 44` → adres 0000, kanal 23 (~433 MHz), hava hızı 2.4k, UART 9600 8N1, şeffaf mod. Şeffaf mod için **M0=GND, M1=GND**. Config okuma/yazma için M0=M1=3.3V (HIGH) + `C1 C1 C1` komutu.

**İki kritik tuzak (ikisi de bu oturumda saatler kaybettirdi):**
1. **Verici beslemesi 5V olmalı.** E32 gönderim anında ~100-120mA çeker; USB-TTL adaptörün (FTDI/Prolific) 3.3V pini ~50mA verir → brownout → hiç/bozuk yayın. VCC'yi adaptörün 5V pinine bağla (E32 VCC 3.0-5.2V dayanır; mantık pinleri 3.3V kalsın). Config okuma az akım çektiği için 3.3V'ta çalışır ama gerçek gönderim çalışmaz — yanıltıcı.
2. **Portları by-id ile tanı, sabit ttyUSB numarası yazma.** Adaptör her çıkar-takta ttyUSB0/1 yer değiştiriyor. `/dev/serial/by-id/usb-FTDI...` (E32#1), `usb-Prolific...` (E32#2), `usb-1a86_USB_Serial...` (Uno CH340) sabittir.

**Ortam:** Ubuntu 22.04. CH340/FTDI sürücüsü ilk takışta bağlanmıyordu; `sudo apt remove brltty` ile kalıcı çözüldü. arduino-cli `~/.local/bin`'de, çekirdek arduino:avr, Uno FQBN `arduino:avr:uno`. Alıcı test sketch'i `~/Arduino/e32_alici`, config okuyucu `~/Arduino/e32_config_oku`.
