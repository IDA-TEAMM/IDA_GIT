"""
Girdap İDA — 3-DOF Katamaran Dinamik Modeli
Durum vektörü: [x, y, psi, u, v, r]
  x, y   : dünya koordinatlarında konum (m)
  psi    : yaw açısı (rad), ENU çerçevesinde
  u      : ileri sürat, body-frame (m/s)
  v      : yanal sürat, body-frame (m/s)
  r      : yaw hızı (rad/s)
Kontrol vektörü: [T_left, T_right] — thruster kuvvetleri (N)
Referans: Fossen (2011) Marine Craft Hydrodynamics, Bölüm 7

Parametreler prototype/configs/dynamics.yaml dosyasından yüklenir.
Mitras itki analizi raporu (SolidWorks CFD, Nisan 2026) kaynak — değerler
mekanik ekipten onay aldıkça aynı YAML'dan güncellenir, koda dokunulmaz.

Dalga bozucusu (WaveDisturbance): Deniz Durumu-2 yaklaşık simülasyonu için
opsiyonel sinüsoidal Fx ve Mz bozucu kuvvetleri. derivatives/step_rk4/simulate
artık t (zaman) parametresi alır; varsayılan t=0.0 ve wave.enabled=False ile
geriye uyumludur.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple, Union

import numpy as np
import yaml

# Repo kökü: bu dosya …/prototype/dynamics/catamaran.py
_DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "configs" / "dynamics.yaml"
)


@dataclass
class WaveDisturbance:
    """
    Sinüsoidal dalga bozucu modeli — Deniz Durumu-2 yaklaşık temsili.

    Gerçek deniz dalgası geniş spektrumlu; bu sadeleştirilmiş tek-frekans
    sinüsoidal model Layer 0 dayanıklılık testleri için yeterli. Saha
    karakterizasyonunda JONSWAP/Pierson-Moskowitz spektrumu eklenecek.
    """

    enabled: bool = False
    Fx_amp: float = 0.0      # N, ileri eksen bozucu genliği
    Fx_freq: float = 0.0     # Hz
    Mz_amp: float = 0.0      # N·m, yaw bozucu genliği
    Mz_freq: float = 0.0     # Hz
    phase: float = 0.0       # rad, sinüs başlangıç fazı

    def force_at(self, t: float) -> Tuple[float, float]:
        """t anındaki (Fx_wave, Mz_wave) bozucu kuvvet/moment çiftini üret."""
        if not self.enabled:
            return (0.0, 0.0)
        Fx = self.Fx_amp * math.sin(
            2.0 * math.pi * self.Fx_freq * t + self.phase
        )
        Mz = self.Mz_amp * math.sin(
            2.0 * math.pi * self.Mz_freq * t + self.phase
        )
        return (Fx, Mz)


@dataclass
class CatamaranParams:
    """3-DOF katamaran sabitleri — kaynak: configs/dynamics.yaml."""

    mass: float                  # kg
    inertia_z: float             # kg·m² (yaw ekseni)
    Xu: float                    # N·s/m, ileri sönümleme
    Yv: float                    # N·s/m, yanal sönümleme
    Nr: float                    # N·m·s/rad, yaw sönümleme
    thruster_spacing: float      # m, iki thruster arası mesafe (B)
    max_thrust: float            # N, tek thruster doygunluk sınırı
    wave: WaveDisturbance = field(default_factory=WaveDisturbance)

    @classmethod
    def from_yaml(
        cls, path: Union[Path, str] = _DEFAULT_CONFIG_PATH
    ) -> "CatamaranParams":
        """
        YAML'dan parametreleri yükle. `catamaran:` zorunlu, `wave:` opsiyonel
        (yoksa enabled=False varsayılan WaveDisturbance kullanılır).
        """
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        wave = WaveDisturbance(**data.get("wave", {}))
        return cls(**data["catamaran"], wave=wave)


class CatamaranDynamics:
    """
    RK4 entegratörlü 3-DOF yüzey aracı dinamik modeli.
    Diferansiyel tahrik: sağ-sol thruster farkıyla dönüş.
    """

    def __init__(self, params: Optional[CatamaranParams] = None) -> None:
        # Argüman verilmezse YAML'dan yükle (mutable default antipattern'inden
        # kaçınmak için Optional + None pattern'i)
        self.p = params if params is not None else CatamaranParams.from_yaml()

    def _clip_control(self, control: np.ndarray) -> np.ndarray:
        """Thruster kuvvetlerini fiziksel sınırlar içinde tut."""
        return np.clip(control, -self.p.max_thrust, self.p.max_thrust)

    def derivatives(
        self,
        state: np.ndarray,
        control: np.ndarray,
        t: float = 0.0,
    ) -> np.ndarray:
        """
        Durum türevlerini hesapla (continuous-time model).
        state  : [x, y, psi, u, v, r]
        control: [T_left, T_right] (N)
        t      : simülasyon zamanı (s) — wave.enabled=True iken kullanılır
        return : [x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot]
        """
        x, y, psi, u, v, r = state
        T_l, T_r = self._clip_control(control)

        # Toplam kuvvet ve moment (body-frame)
        Fx = T_l + T_r
        Mz = (T_r - T_l) * self.p.thruster_spacing / 2.0

        # Dalga bozucusu (Deniz Durumu-2). Disabled iken bedava (early return).
        if self.p.wave.enabled:
            Fx_w, Mz_w = self.p.wave.force_at(t)
            Fx += Fx_w
            Mz += Mz_w

        # Kinematik: body → dünya dönüşümü
        x_dot   = u * np.cos(psi) - v * np.sin(psi)
        y_dot   = u * np.sin(psi) + v * np.cos(psi)
        psi_dot = r

        # Dinamik: Newton-Euler (M * v_dot = tau + D*v)
        u_dot = (Fx + self.p.Xu * u) / self.p.mass
        v_dot = (self.p.Yv * v) / self.p.mass
        r_dot = (Mz + self.p.Nr * r) / self.p.inertia_z

        return np.array([x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot])

    def step_rk4(
        self,
        state: np.ndarray,
        control: np.ndarray,
        dt: float,
        t: float = 0.0,
    ) -> np.ndarray:
        """
        Runge-Kutta 4. derece entegrasyon adımı.
        Wave bozucusu zaman-bağımlı olduğu için RK4 evrelerinde t kayar.
        """
        k1 = self.derivatives(state,                control, t)
        k2 = self.derivatives(state + dt / 2 * k1,  control, t + dt / 2)
        k3 = self.derivatives(state + dt / 2 * k2,  control, t + dt / 2)
        k4 = self.derivatives(state + dt * k3,      control, t + dt)
        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def simulate(
        self,
        state0: np.ndarray,
        controls: np.ndarray,
        dt: float,
        t0: float = 0.0,
    ) -> np.ndarray:
        """
        Çoklu adım simülasyon.
        controls: (N, 2) — her adım için [T_left, T_right]
        t0      : başlangıç zamanı (wave fazlandırması için)
        return  : (N+1, 6) — başlangıç dahil tüm durum geçmişi
        """
        n_steps = controls.shape[0]
        history = np.zeros((n_steps + 1, 6))
        history[0] = state0
        state = state0.copy()
        t = t0
        for i, ctrl in enumerate(controls):
            state = self.step_rk4(state, ctrl, dt, t)
            history[i + 1] = state
            t += dt
        return history
