---
name: ida-jetson-ethernet
description: "Kaptan laptop (eno1) <-> Jetson ethernet baglantisi, IP/DDS kalici ayari ve NM split-brain tuzagi"
metadata: 
  node_type: memory
  type: project
  originSessionId: dc2dae05-6442-486b-b3fc-1175c69c5fcb
---

Kaptan laptop (yahya-seha) <-> Jetson (girdap) arasi ethernet ROS2/DDS koprusu.

**Topoloji:** eno1 = 192.168.117.1/24 (laptop), Jetson = 192.168.117.50/24. Baglanti dogrulandi (2026-07-11): ARP cozuluyor, SSH `ssh girdap@192.168.117.50` (parolayla) calisiyor. Jetson ICMP ping'e KAPALI — ping cevap vermezse panige gerek yok, SSH/ARP ile teyit et.

**Kalici IP:** systemd-networkd yonetiyor: `/etc/systemd/network/jetson-eth.network` (`Name=enp5s0` — bu `eno1`'in altname'i, o yuzden uyuyor). NM'de `eno1` UNMANAGED yapildi: `/etc/NetworkManager/conf.d/eno1-unmanaged.conf` -> `unmanaged-devices=interface-name:eno1`.

**TUZAK (cozuldu):** NM'in "Wired connection 1" profili `eno1`'e yanlislikla 192.168.117.50 (Jetson'in IP'si!) vermeye ayarliydi -> split-brain, reboot'ta eno1 .50 olup Jetson ile cakisabilirdi. Cozum: eno1'i NM'den cikarip sadece systemd-networkd'ye birakmak. NM release edince link gecici DOWN gorunur; networkd yeniden ayaga kaldirir (carrier varsa). NM `livox` profili de placeholder (`ENP_ADI`, .50) — pasif, eno1'i etkilemiyor. Bu laptopta TEK ethernet portu var (eno1).

**DDS:** `ROS_DOMAIN_ID=42` her iki tarafta bashrc'de. Docker konteyner `~/ros2_baslat_v5.sh` ile `--network host --env ROS_DOMAIN_ID=42` (host DDS gorunurlugu icin sart). Onceki oturumda Jetson'dan `ros2 topic list` ile laptop topic'leri gorulmustu.

Yardimci scriptler laptop home'da: `jetson_ag_duzelt.sh` (kurulum), `jetson_ag_tani.sh`/`jetson_baglanti_test.sh` (teshis). Ilgili: [[ida-e32-lora]] [[ida-livox-mid360]] [[ida-sim-workspace]]
