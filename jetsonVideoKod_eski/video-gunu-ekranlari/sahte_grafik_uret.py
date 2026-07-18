#!/usr/bin/env python3
"""SAHTE grafik CSV üretici — Ekran-2 önizlemesi için (VIDEO SENARYOSU TAKLİDİ).

Gerçek telemetry_node'un fc modundaki (setpoint_source=fc, AUTO görev)
davranışını taklit eder:
  - Görev öncesi (BEKLEMEDE): setpoint hücreleri BOŞ; disarm'da thrust BOŞ
    (PWM=0 → boş hücre kuralı), arm sonrası rölanti %0.
  - AUTO başlayınca: hiz_setpoint = WP_SPEED = 2.0 sabit; yon_setpoint =
    aktif waypoint'e açı (köşede sıçrar); thrust = FC servo çıkışı ±%100.
  - 4. noktada görev TAMAMLANIR: setpoint'ler kesilir, kısa manuel dönüş.

⚠️ SAHTE VERİ — ~/girdap_logs'a YAZILMAZ, yalnız önizleme.
"""
import csv
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

OUT = Path(__file__).parent / "grafik_SAHTE_onizleme.csv"
HZ = 10.0
DT = 1.0 / HZ
RNG = np.random.default_rng(42)

# Video görevi: 4 köşe dikdörtgen (60 m x 40 m), tekne P1'in az gerisinde.
WPS = [(0.0, 0.0), (60.0, 0.0), (60.0, 40.0), (0.0, 40.0)]
WP_RADIUS = 2.0       # FC WP_RADIUS = arrival_radius_m
CRUISE = 2.0          # WP_SPEED = fc_cruise_setpoint_mps (param turu 15.07)
T_PREARM = 8.0        # disarm bekleme (PWM=0 -> thrust hücresi BOŞ)
T_ARMED = 7.0         # armed rölanti (AUTO'ya alınmadan önce, %0)
T_MANUEL = 25.0       # görev sonu manuel dönüş kesiti
MAX_YAWRATE = 0.35    # rad/s (dönüş hızı sınırı)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def main() -> None:
    t0 = datetime(2026, 7, 20, 9, 0, 0, tzinfo=timezone.utc)
    rows = []
    t = 0.0

    # Durum: tekne P1'in 7 m güneybatısında, burnu kuzeydoğuya bakıyor.
    x, y, psi, v = -5.0, -5.0, math.radians(80.0), 0.0
    wp_i = 0
    faz = "prearm"
    t_mission0 = None
    t_manuel0 = None

    def dalga(tt: float, f: float, amp: float, seed_ofs: float) -> float:
        return amp * math.sin(2 * math.pi * f * tt + seed_ofs)

    while True:
        stamp = t0 + timedelta(seconds=t)
        hiz = hiz_sp = heading = yon_sp = th_sol = th_sag = ""

        # --- faz makinesi ---
        if faz == "prearm" and t >= T_PREARM:
            faz = "armed"
        if faz == "armed" and t >= T_PREARM + T_ARMED:
            faz = "mission"
            t_mission0 = t
        if faz == "manuel" and t - t_manuel0 >= T_MANUEL:
            break

        # --- gerçek hız/heading her fazda yazılır (EKF canlı) ---
        gurultu_psi = math.radians(dalga(t, 0.33, 1.8, 1.0)) + RNG.normal(0, math.radians(0.5))
        gurultu_v = dalga(t, 0.45, 0.07, 2.3) + RNG.normal(0, 0.035)

        if faz in ("prearm", "armed"):
            hiz = max(0.0, 0.02 + gurultu_v * 0.5)
            heading = wrap(psi + gurultu_psi)
            if faz == "armed":
                th_sol = RNG.normal(0, 0.4)
                th_sag = RNG.normal(0, 0.4)
        elif faz == "mission":
            tx, ty = WPS[wp_i]
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx)
            if dist < WP_RADIUS:
                if wp_i + 1 < len(WPS):
                    wp_i += 1
                    tx, ty = WPS[wp_i]
                    dx, dy = tx - x, ty - y
                    bearing = math.atan2(dy, dx)
                else:
                    faz = "manuel"
                    t_manuel0 = t
            if faz == "mission":
                err = wrap(bearing - psi)
                yawrate = max(-MAX_YAWRATE, min(MAX_YAWRATE, 1.2 * err))
                # Rover köşede yavaşlar: büyük yön hatasında hedef hız düşer.
                v_hedef = CRUISE * (1.0 - 0.55 * min(1.0, abs(err) / 1.0))
                v += max(-0.6, min(0.35, (v_hedef - v))) * DT / 0.9
                psi = wrap(psi + yawrate * DT)
                x += v * math.cos(psi) * DT
                y += v * math.sin(psi) * DT

                hiz = max(0.0, v + gurultu_v)
                heading = wrap(psi + gurultu_psi)
                hiz_sp = CRUISE
                yon_sp = bearing
                base = 18.0 + 30.0 * v_hedef / CRUISE + dalga(t, 0.5, 2.0, 4.1)
                steer = 38.0 * yawrate / MAX_YAWRATE
                th_sol = base - steer + RNG.normal(0, 1.2)
                th_sag = base + steer + RNG.normal(0, 1.2)
        if faz == "manuel":
            # Görev bitti: setpoint YOK (F-V.2/F-V.8), FC Hold -> operatör manuel.
            tm = t - t_manuel0
            if tm < 4.0:            # duruş: thrust sıfırlanır, tekne yavaşlar
                v = max(0.0, v - 0.5 * DT)
                th_sol = RNG.normal(0, 0.5)
                th_sag = RNG.normal(0, 0.5)
            else:                   # operatör geri sürüyor (yarım güç, geniş dönüş)
                v += (1.1 - v) * DT / 1.5
                psi = wrap(psi + 0.12 * DT)
                th_sol = 34.0 + dalga(t, 0.4, 3.0, 0.7) + RNG.normal(0, 1.5)
                th_sag = 40.0 + dalga(t, 0.37, 3.0, 2.9) + RNG.normal(0, 1.5)
            x += v * math.cos(psi) * DT
            y += v * math.sin(psi) * DT
            hiz = max(0.0, v + gurultu_v)
            heading = wrap(psi + gurultu_psi)

        def fmt(val, nd=3):
            return "" if val == "" else f"{val:.{nd}f}"

        rows.append([
            stamp.isoformat(timespec="milliseconds"),
            fmt(hiz), fmt(hiz_sp), fmt(heading, 4), fmt(yon_sp, 4),
            fmt(th_sol, 1), fmt(th_sag, 1),
        ])
        t += DT

    with open(OUT, "w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(["zaman", "hiz", "hiz_setpoint", "heading",
                    "yon_setpoint", "thrust_sol", "thrust_sag"])
        w.writerows(rows)
    print(f"{OUT}: {len(rows)} satır, {t:.0f} s "
          f"(görev {t_mission0:.0f}-{t_manuel0:.0f} s)")


if __name__ == "__main__":
    main()
