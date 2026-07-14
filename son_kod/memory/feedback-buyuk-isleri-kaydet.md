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

**IDA_GIT sınır kuralı (2026-07-14, Yahya'nın talimatı):** IDA-TEAMM/IDA_GIT'te **YALNIZ `son_kod/` içinde düzenleme yap** — dışındaki klasörlere (src/, docs/ vb. takım alanları) ASLA yazma. Akış: IDA_GIT'in son_kod-dışı klasörlerinde değişiklik olduğunda (git log/diff ile TARA) → ilgili değişimleri değerlendirip son_kod'u ona göre güncelle (örn. src/ida_topics_yeni'ye gelen fix'ler kanonik `son_kod/karar/ros2_ws/src/ida_topics_yeni`'ye taşınır). son_kod = Yahya'nın IDA_GIT içindeki kişisel çalışma alanı. Push sırası: önce girdap-kaptan-video, sonra IDA_GIT son_kod/ (rebase ile).

**Why:** Takım deposunda başkalarının alanına dokunmak çakışma/silme riski yaratıyor (bugünkü sude ida_topics olayı); tek yönlü akış (takım → son_kod) drift'i kontrollü tutar.
**How to apply:** IDA_GIT'e her push öncesi `git pull --rebase` + `git log --oneline -10 -- . ':(exclude)son_kod'` ile dış değişiklikleri kontrol et; anlamlı değişiklik varsa kullanıcıya raporla ve son_kod'a uyarla.

**Otomatik tarama (2026-07-14):** SessionStart hook'u kurulu (`~/.claude/settings.json` → `~/.claude/ida-git-tarama.sh`): her oturum başında IDA_GIT'in son_kod-dışı yeni commit'leri bağlama otomatik düşer ("[IDA_GIT tarama] ..."). O notu gördüğünde sınır kuralına göre değerlendir. Klon önbelleği: ~/.cache/idagit-scan, işaretçi: ~/.cache/idagit-scan-last.

İlgili: [[user-role]] · [[ida-project]]
