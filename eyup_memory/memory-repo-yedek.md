---
name: memory-repo-yedek
description: Memory dizini github.com/EyupEker1/memory (private) reposunda yedekleniyor — önemli güncellemelerden sonra commit+push; repolardaki CLAUDE.md anlık kopyaları repolar/ altında
metadata: 
  node_type: memory
  type: reference
  originSessionId: 614f4432-b279-4bb2-abdc-48995516a1c1
---

Eyüp'ün isteği (2026-07-13): genel memory'ler bir GitHub reposunda dursun.

- **Repo: github.com/EyupEker1/memory (PRIVATE)** — içeriği bu dizinin kendisi:
  `/home/eyup/.claude/projects/-home-eyup/memory/` git reposu yapıldı, `origin` = o repo.
- **Kaynak-of-truth LOKAL dizindir**; repo yedek/paylaşım kopyası. Akış: memory dosyası
  yaz/güncelle → `git -C /home/eyup/.claude/projects/-home-eyup/memory add -A && git commit
  && git push` (uzun oturumlarda [[sik-memory-checkpoint]] ile birlikte uygula).
- `repolar/` klasörü: proje repolarındaki CLAUDE.md'lerin **anlık kopyaları**
  (girdap-decision, IDA_GIT-upstream; girdap-ida-algi'de CLAUDE.md yok). Bunlar bayatlar —
  gerçeği her zaman reponun kendi CLAUDE.md'sinden oku; buradakiler sadece toplu bakış için.
- Private kalmalı: içerde takım/proje ayrıntıları var. Arkadaşların erişimi YOK.
