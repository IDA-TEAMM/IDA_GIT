# GİRDAP Memory (Claude Code kalıcı hafızası)

Bu repo, Eyüp'ün PC'sindeki Claude Code kalıcı hafıza dizininin
(`~/.claude/projects/-home-eyup/memory/`) yedeğidir. **Kaynak-of-truth lokal dizindir**;
her önemli memory güncellemesinden sonra buraya commit+push atılır.

- `MEMORY.md` — indeks (her oturumda yüklenen tek dosya; her memory'ye bir satır).
- `*.md` — tek dosya = tek konu; frontmatter'da `type: user|feedback|project|reference`.
- `repolar/` — proje repolarındaki CLAUDE.md'lerin **anlık kopyaları** (bayatlayabilir;
  gerçeği reponun kendisinden oku):
  - `girdap-decision-CLAUDE.md` (fork: EyupEker1/girdap-decision)
  - `IDA_GIT-upstream-CLAUDE.md` (gömülü ekip yığını; revizyon kopyası: EyupEker1/IDA_GIT)
  - girdap-ida-algi'de CLAUDE.md yok (bilgi README + docs/ altında).

⚠️ PRIVATE kalmalı — takım/proje/yarışma ayrıntıları içerir.
