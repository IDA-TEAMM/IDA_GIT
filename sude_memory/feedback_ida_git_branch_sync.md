---
name: feedback-ida-git-branch-sync
description: "IDA USV repo: git push'lar hem main hem sude-feature-v2'ye yapilmali, ikisi senkron tutulmali"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 69473ebd-427a-4cc3-b61f-3fe1132cd962
---

`~/ros2_ws` reposunda (origin `IDA-TEAMM/IDA_GIT`) kullanici artik `main` ve `sude-feature-v2` branch'lerinin **surekli birebir ayni commit'lere isaret etmesini** istiyor.

2026-07-12'de `sude-feature-v2` main'den fast-forward'du (main tam ata konumdaydi, sapma yoktu) ve kullanici acikca "push isini hem main'e hem sude-feature'a yapicaz" dedi.

**How to apply:** Bundan sonra bu repoda `git push` gerektiren her degisiklikte (commit sonrasi), SORMADAN dogrudan uygula:
1. Once calisilan branch'e (genelde `sude-feature-v2`) push et.
2. Ardindan `main`'i de ayni commit'e getir: `git checkout main && git merge --ff-only sude-feature-v2 && git push origin main && git checkout sude-feature-v2`.
3. Fast-forward calismazsa (main sapmis, kendine ozgu commit'leri varsa): normal `git merge sude-feature-v2` dene ve push et — gercek bir conflict (merge cakismasi, dosya icinde `<<<<<<<` isaretleri) cikarsa o zaman kullaniciya sor, aksi halde otomatik ilerle. 2026-07-13'te kullanici acikca "sorma direk yap" dedi.
4. Vendored submodule'lerdeki (`src/ros2/*`, `src/gazebo_ros_pkgs` vb.) "değiştirildi (m)" isaretlerine dokunma, CLAUDE.md'nin dedigi gibi bunlar commit'e dahil edilmez.

**Why:** Kullanici iki branch'i paralel/senkron tutmak istiyor, main'i ayri bir PR/review adimi olmadan guncel tutmayi tercih ediyor (repo kucuk/tek gelistiricili, TEKNOFEST yarisma projesi, hizli iterasyon onemli — bkz. [[project-ida-teknofest-overview]]).
