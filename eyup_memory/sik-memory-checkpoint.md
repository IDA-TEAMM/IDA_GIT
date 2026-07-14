---
name: sik-memory-checkpoint
description: "Eyüp'ün kredisi oturum ortasında bitebilir — uzun işlerde her ~500 satır/büyük adımda memory'ye ara kayıt at"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 2afb4def-d306-47e8-8bf0-f118690f51af
---

Eyüp'ün Claude kredisi oturum ortasında tükenebiliyor; uzun işleri fazlara bölmemi ve **faz sonunu beklemeden, her ~500 satırlık iş / her büyük adımda** memory'ye ara kayıt atmamı istedi (2026-07-10).

**Why:** Kredi bitince oturum kesilir; kaydedilmemiş ilerleme (hangi dosya ne kadar düzenlendi, ne doğrulandı, ne kaldı) kaybolur ve yeni oturumda baştan keşif yapmak ekstra kredi yakar.

**How to apply:** Çok adımlı işlerde ilgili proje memory dosyasında (örn. [[girdap-decision-entegrasyon]]) faz durum bölümünü her önemli adımdan sonra güncelle: ne bitti (dosya + ne yapıldı + doğrulama durumu), ne yarım, sıradaki somut adım. Kayıt kısa olsun — hedef, yeni oturumun tek okumayla kaldığı yerden devam edebilmesi.
