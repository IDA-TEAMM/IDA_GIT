# Video Kontrol Listesi — şartname md 3.3.1 / 3.3.1.1 birebir

Çekimden ÖNCE prova et; yayına almadan ÖNCE videoyu baştan izleyerek her kutuyu işaretle.

## 0. Kod kanıtı (çekim gününden önce, Jetson'da)

- [ ] `GIRDAP_KOD=~/ros2_ws/src/girdap-decision bash testler/video_testleri.sh --tam`
      → **12/12 bileşen ✅, failed/error = 0** (testler/README.md — kırmızı
      bileşen varken çekime çıkılmaz)

## md 3.3.1 — videoda GÖSTERİLMESİ ZORUNLU 6 madde

- [ ] **(1) İDA↔YKİ kablosuz bağlantı** kurulu ve İDA bilgileri YKİ'de görünüyor
      (QGC ↔ RFD868x; Ekran-1'de telemetri akışı seçilebilir olmalı)
- [ ] **(2) YKİ'den 4 noktalı görev tanımlandı ve İDA'ya GÖNDERİLDİ** — noktalar
      **DİKDÖRTGEN oluşturacak** (md 3.3.1(2) açık şart; QGC Plan → 4 köşe →
      Upload; videoda upload ânı görünsün)
      ⚠ **Görev TAM 4 nokta: başlangıca-dönüş noktası EKLEME** — md 3.3.1(3)
      dönüşü MANUEL istiyor; 5. nokta hem "4 noktalı görev" tanımını bozar hem
      dönüşü otonomlaştırır (F-V.4)
- [ ] **(3) Tek komutla görev başladı** (mod → GUIDED) **ve 4. (SON) noktada otonom tamamlandı**
      (sıfır thrust; sonrası manuel dönüş serbest)
- [ ] **(4) Güvenlik anahtarıyla güç kesildi + RC komutuna rağmen motorlar DÖNMÜYOR**
      (fiziksel gösterim — kameraya net)
- [ ] **(5) Kapaklar açıldı, su almadığı gösterildi**
- [ ] **(6) Hepsi TEK, KESİNTİSİZ gösterimde** (kurgu ile atlanmış adım yok)

## md 3.3.1.1 — biçim şartları

- [ ] 3 bölmeli ekran **yerleşimi Şekil 1 ile BİREBİR** (serbest düzen değil):

      ```
      ┌───────────┬────────────────┐
      │ 1 Ekran-1 │                │
      │ (üst-sol) │  3  Ekran-3    │
      ├───────────┤ (sağ, büyük    │
      │ 2 Ekran-2 │  dikey bölme)  │
      │ (alt-sol) │                │
      └───────────┴────────────────┘
      ```

      **Ekran-1** YKİ (QGC) kaydı · **Ekran-2** senkron grafikler ·
      **Ekran-3** dış kamera
- [ ] Ekran-2'de ÜÇ sinyal de var: hız + hız_setpoint · heading + yaw_setpoint ·
      **thrusterlardan kuvvet isteği** (setpoint'ler AÇI/hız İSTEĞİ — yaw
      setpoint F-V.1 düzeltmesiyle artık açı, heading'le aynı eksende)
- [ ] **Ekran-3 içeriği aşamaya uygun** (şartname örnekleri): görev koşusu =
      İDA'nın suda hareketi · başlatma/güç kesme = RC kumanda + motor yakın
      çekim · su almazlık = İDA'nın iç görüntüsü
- [ ] **Görev aşamasında YKİ ekranı (Ekran-1) ile İDA hareketleri (Ekran-3)
      SENKRON** — kaba test: köşe dönüşünde QGC harita ikonu + Ekran-2 heading
      eğrisi + dış kameradaki dönüş AYNI anda
- [ ] Grafikler İDA hareketiyle **senkron** (run_ekran2 --t0/--t1 kırpma)
- [ ] **İstemsiz dönüş/sürüş YOK** (zikzak/titreme görünüyorsa çekim tekrarı!)
- [ ] Görüntü ve hareketler net (bulanık/uzak ise BAŞARISIZ sayılabilir)
- [ ] ≥720p · süre **en az 2 dk, en fazla 5 dk**
- [ ] **YALNIZ YouTube** (başka platform kabul EDİLMEZ), **liste dışı** yüklendi,
      link farklı cihazdan/oturumsuz AÇILIYOR ("linkte sorun = eleme")
- [ ] Link KYS'ye girildi (21.07 17:00'dan ÖNCE — son güne bırakma; hedef 20.07)

## Çekim listesi (Ekran-3 dış kamera)

- [ ] Görev yükleme + ARM + GUIDED komutu ânı (operatör eli/ekran)
- [ ] Teknenin 4 noktayı gezişi (geniş açı, tekne hep kadrajda)
- [ ] Duruş (sıfır thrust) + manuel dönüş
- [ ] Güç kesme anahtarı yakın çekim + RC deneme (motor sessiz)
- [ ] Kapak açma / içi kuru gösterimi

## Bilinen tuzaklar (masa testlerinden)

- GPS fix olmadan GUIDED reddedilir → açık alanda fix bekle (planning de fix ister — F-M.1)
- QGC'den disarm edilirse köprü sahte FAILSAFE logu basar → disarm için SERVİSİ kullan
  (`/girdap/bridge/disarm`) — runbook §5/2
- Boot'ta mod zaten GUIDED ise başlamaz (kenar tetik): HOLD'a al → ARM → GUIDED
- RFD kanal/frekans ayarı önceden yapılmış olmalı (2.4 GHz RC YASAK bandı)
