---
name: girdap-ida-birlesik-repo
description: "github.com/EyupEker1/girdap-ida (private) — 4 kaynak reponun tek çatısı (algi+karar subtree tam geçmiş, gomulu anlık görüntü, logger subtree); temelli revizyon burada yürüyecek; ⚠️ Jetson/video kurulumu HÂLÂ ayrı klonlarla, çift-bakım kuralı geçerli"
metadata: 
  node_type: memory
  type: project
  originSessionId: 29fc60b5-19c2-4682-bbc4-8892fb0f0740
---

Eyüp'ün isteğiyle (2026-07-13 akşam) tüm İDA kodu tek repoda toplandı:
**github.com/EyupEker1/girdap-ida (PRIVATE, main)**, lokal **/home/eyup/girdap-ida**.
Git kimliği lokal Team GIRDAP. İlk push `ed06fab`; **CI 3 job YEŞİL** (karar-çekirdek
1m34s + gömülü-köprü 11s + sözdizimi 12s).

## Yapı (kök README'de tablo)

| Dizin | Kaynak | Commit | Yöntem |
|---|---|---|---|
| `algi/` | girdap-ida-algi | `732abea` | **subtree** (tam geçmiş) |
| `karar/` | girdap-decision | `d7f9be6` | **subtree** (tam geçmiş, çift-bakım 2026-07-13 gece) |
| `gomulu/` | IDA_GIT | `11f43f4` | **anlık görüntü** — yalnız yaşayan katman (ida_topics_yeni + girdap_yenimodel URDF + docs + CLAUDE.md); vendored ~55 ROS dizini BİLEREK dışarıda; ayrıntı `gomulu/KAYNAK.md` |
| `logger/` | girdap-logger | `5450a2d` | **subtree** (tam geçmiş) |

Doğrulama (kuruluş günü, repo içinden koşuldu): **karar 257 passed / 4 skip** (PC taban
birebir; deps ws source'lu) + **gömülü 8 passed** + algi/logger py_compile OK.

## ⚠️ ÇİFT-BAKIM KURALI (en kritik)

- **Kaynak repolar SİLİNMEDİ ve operasyonel gerçek ONLAR:** Jetson'daki video kurulumu
  (21.07 eleme kapısı) ayrı klonlarla test edildi, ÖYLE KALIYOR. Bu repoya geçiş kararı
  **video SONRASI**.
- O güne kadar bir düzeltme HANGİ kopyada yapılırsa yapılsın karşıya da taşınmalı,
  yoksa kopyalar sessizce ayrışır. Güncelleme komutları:
  - `git subtree pull --prefix=algi /home/eyup/girdap-ida-algi main` (karar/logger benzer)
  - gomulu: IDA_GIT'ten `git archive <commit> src/ida_topics_yeni src/girdap_yenimodel docs CLAUDE.md | tar -x -C gomulu` + KAYNAK.md commit güncelle
- girdap-decision fork'unun upstream (vistastris) PR akışı ve çift-push yedeği eskisi
  gibi KAYNAK repoda yaşıyor — birleşik repo o akışın yerine geçmiyor (şimdilik).

## Video reposu (aynı gün kuruldu, Eyüp isteği)

**github.com/EyupEker1/girdap-video (PRIVATE, main)**, lokal **/home/eyup/girdap-video**:
`karar/` = girdap-decision @ `d7f9be6` **DONDURULMUŞ** subtree (c77dca3→d7f9be6 bilinçli güncelleme 2026-07-13 gece, video denetim düzeltmeleri) (videoda koşacak kod —
geliştirme sürse bile burada sabit; güncelleme yalnız bilinçli `git subtree pull` +
README commit notu) + kök README (video günü akışı + FC OLAY güvenlik bloğu + Jetson
sıfırlama yönlendirmesi) + `kontrol-listesi.md` (md 3.3.1 6 şart + md 3.3.1.1 biçim +
çekim listesi + masa testi tuzakları). Dondurulmuş kod içinden suite koşuldu: 257/4 ✓.
**Jetson'a bu repo KURULMAZ** — video, test edilmiş ayrı-klon düzeniyle koşar
(BURADAN_BASLA.md); bu repo plan + kod arşividir. Eyüp Jetson'ı SIFIRLAYACAK —
kurulum rehberi masaüstü JETSON_REHBERI.md; FC'deki sahte görev Jetson sıfırlamayla
SİLİNMEZ (OLAY aksiyonları geçerli).

## Sıradaki: TEMELLİ REVİZYON bu repoda
Kapsam Eyüp'le netleşecek ([[yeni-arkadas-repo-plani]] son bölüm): repolar arası sözleşme
tutarlılığı / rol-çakışma matrisinin 4 parçaya genişletilmesi / tek sistem dokümanı.
Kurallar: [[sartname-once-kural]] · [[test-dogrulama-ilkesi]] · [[jetson-yuk-kod-sadeligi]].
⚠️ T0 video (21.07) önceliği her şeyin önünde.
