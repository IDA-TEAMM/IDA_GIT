---
name: feedback-buyuk-isleri-kaydet
description: "Kullanıcı isteği: her büyük değişiklik/iş sonunda sormadan hafızaya kaydet"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 97fbf033-cf13-4546-afbf-c6b146529521
---

Yahya, her büyük değişikliğin ve tamamlanan işin hafızaya kaydedilmesini istiyor (2026-07-14).

**Why:** Oturumlar arası süreklilik kritik — TEKNOFEST İDA işinde kararlar, fix'ler ve dosya konumları sonraki oturumda hatırlanmazsa aynı inceleme/karar tekrar yapılıyor.

**How to apply:** Büyük bir iş bitince (kod fix'i, mimari karar, inceleme, dosya taşıma/kurulum, parametre kararı) sormadan ilgili memory dosyasını güncelle veya yeni dosya aç + MEMORY.md index'ine ekle. Küçük/geçici işler (tek komut, deneme) için kayıt gerekmez; mevcut kaydı güncellemek yeni dosya açmaya tercih edilir.

**İki-dosya kuralı (2026-07-14):** Önemli KOD değişimleri → [[kod-duzeltme]] (+ ~/Desktop/son_kod/memory/ kopyası tazelenir). Normal işler/genel durum özeti → `~/Desktop/ida_son_durum.md` (kullanıcının masaüstü durum dosyası) güncel tutulur.

**Git kuralı (2026-07-14):** `~/Desktop/son_kod` = git deposu → GitHub `girdap-kaptan-video` (PRIVATE, sadece davetliler). son_kod'da her kod düzenlemesinde: (1) memory/ klasörü tazelenir, (2) değişiklik + memory birlikte commit edilir. Commit kimliği: Yahya Seha Danış / yahyasehadanis1@gmail.com (repo-local config ayarlı).

İlgili: [[user-role]] · [[ida-project]]
