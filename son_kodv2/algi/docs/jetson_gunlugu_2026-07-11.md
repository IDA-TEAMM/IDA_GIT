# Jetson Günlüğü — 2026-07-11 (akşam oturumu)

> Jetson'da (girdap@ubuntu) fiilen yapılanların kısa kaydı. Ayrıntılar için
> işaret edilen dokümanlara bak. Önceki oturumun kaldığı yer: suite teyidi
> bekliyordu.

## ✅ Yapılanlar

1. **Suite Jetson'da teyit edildi:** önce `246 passed / 1 skipped`
   (numpy 1.26.4 + opencv 4.11.0.86 pinleriyle) — eski `girdap_ws`
   gölgelemesi temiz, dialout aktif, jetson_kontrol yalnız bilinen 3 madde.
2. **MPPI CUDA Faz A yapıldı** (girdap-decision `c612fb0`):
   - cupy-cuda12x 13.6.0 kuruldu (numpy pini aynı komutta — altın kural).
   - 🐛 Gerçek bug bulundu+düzeltildi: cupy Generator'da `.normal()` yok →
     `_sample_noise` GPU'da ölüyordu (parite testinin monkeypatch'i
     maskelemişti). TDD ile kapatıldı.
   - **Yeni suite tabanı: `246 passed / 2 skipped`** (masa runbook M0 buna
     güncellendi, `5985edc`). CI yeşil.
   - **Bench (MAXN_SUPER):** numpy 209 ms (~4.8 Hz) · cupy 302 ms.
     Maliyet GPU'da 18× hızlı, rollout launch-overhead'e takılı →
     **Faz B (rollout kernel füzyonu) tetiklendi.** ⚠ `control_rate_hz: 10`
     Jetson CPU'da tutmuyor — karar Faz B sonrası.
     Ayrıntı: girdap-decision `docs/mppi_cuda_plani.md` + `docs/kod_denetimi.md`.
3. **§2.6 OAK kamera testi PASS:** OAK-D-LITE, 12.0 FPS sabit, kareler net.
   - ⚠→✅ İlk takışta USB2 (UsbSpeed.HIGH) kaldı; SS kablo + USB3.2 port
     sağlamdı. **Çözüm: USB-C ucunu 180° ters çevirmek → SUPER.**
     Rehbere işlendi: `BURADAN_BASLA.md` §5 hata 10 (`0db6404`).
     Muhafazaya almadan önce `getUsbSpeed()` → SUPER teyidi ŞART.
4. **bashrc onarıldı** (sed temizliği PYTHONPATH satırını da silmişti) +
   `~/ros2_ws` temiz ortamda yeniden derlendi (bayat girdap_ws zincir
   uyarıları gitti). Yeni terminalde import'lar doğru.

## 📌 Düzeltilen kayıt hataları

- "push'lu" sanılan `c742284` (BURADAN_BASLA §2.9-2.10) GitHub'da yoktu —
  PC'de kalmış. **PC'den push edilecek**, sonra Jetson'da `git pull`.
- girdap-decision yedek reposu Jetson push'larını almıyor (çift-push URL
  yalnız PC klonunda) — **PC'den `git pull && git push` ile senkronlanacak.**

## 🚀 EK (aynı gece): CUDA Faz B BİTTİ — MPPI 302→9.0 ms (~33×)

Rollout tek CUDA RawKernel'e füzyonlandı (girdap-decision `1558ead`):
tavan ~112 Hz, **20 VE 50 Hz kriterleri GEÇTİ**, 600 adımda sürüklenme
−1.0%. Suite yeni taban **249/2** (süre 313→67 sn). control_rate: 10 Hz
rahat, 20 Hz mümkün (sahada doğrulanacak). CUDA planında kalan iş yok.

## ⏭️ Kalanlar (öncelik sırasıyla)

- [ ] ~~CUDA Faz B~~ ✅ bitti (yukarı bak)
- [ ] `sudo jetson_clocks` (parola gerekli) → bench tekrarı (artık opsiyonel)
- [ ] Pixhawk takılınca masa runbook M1'den devam (pervaneler sökülü!)
- [ ] Ölçüm formu (`docs/olcum_formu.md`) mekanik/FC ekibine gönderilecek
- [ ] PC'den: `c742284` push + yedek repo senkronu
- [ ] En son: `sudo rfkill block wifi bluetooth` (şartname 4.1)
