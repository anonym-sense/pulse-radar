"""Radar frames. Every frame is played cyclically by both transmitters.

MIMO frames (pulse, LFM pulse, FMCW, long FMCW) have two TDM slots: TX1 alone, then TX2
alone. With RX1/RX2 that gives four TX->RX paths = a 4-element virtual array.

The TX-beam frame physically steers: after a sync slot and the two single-TX slots it has
12 slots where both transmitters radiate together with TX2 advanced by 0, 30 ... 330 deg.

All frames use the same DAC peak (amp), so the comparison is at equal peak power, which is
the real limit of this hardware. Energy per frame is then decided by duty cycle.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

C = 299_792_458.0
FS = 61.44e6
BW = 50e6
D_M = 0.0508                       # element pitch; row is TX1 TX2 RX1 RX2
X_TX = np.array([0.0, 1.0]) * D_M
X_RX = np.array([2.0, 3.0]) * D_M
X_VIRT = np.array([X_TX[t] + X_RX[r] for t in range(2) for r in range(2)])   # [2,3,3,4]*d; order t0r0,t0r1,t1r0,t1r1


@dataclass
class Frame:
    key: str
    label: str
    P: int                              # frame length (samples)
    frame: np.ndarray                   # (2, P) to transmit
    slots: list                         # [(start, shape ndarray)] one per slot
    slot_tx: np.ndarray                 # (n_slots, 2) complex TX weights of each slot
    on_samples: int                     # samples each TX radiates per frame
    kind: str = "mimo"                  # mimo | txbeam
    phases: np.ndarray | None = None    # txbeam: TX2 phase per beam slot
    notes: dict = field(default_factory=dict)

    @property
    def n_slots(self):
        return len(self.slots)

    def energy_db(self):
        return float(10 * np.log10(np.sum(np.abs(self.frame[0]) ** 2)))

    def describe(self, lo):
        lam = C / lo
        return {"key": self.key, "label": self.label, "P": self.P, "kind": self.kind,
                "duty": round(self.on_samples / self.P, 4), "energy_db": round(self.energy_db(), 2),
                "on_us": round(self.on_samples / FS * 1e6, 2), "range_res_m": round(C / (2 * BW), 2),
                "bin_m": round(C / (2 * FS), 3), "lam_mm": round(lam * 1e3, 2), **self.notes}


def taper(n, frac):
    w = np.ones(n)
    m = int(frac * n / 2)
    if m > 0:
        ramp = 0.5 - 0.5 * np.cos(np.pi * (np.arange(m) + 0.5) / m)
        w[:m], w[-m:] = ramp, ramp[::-1]
    return w


def chirp(n, bw=BW, down=False, tap=0.0):
    t = (np.arange(n) - (n - 1) / 2) / FS
    k = bw / (n / FS) * (-1 if down else 1)
    return np.exp(1j * np.pi * k * t ** 2) * taper(n, tap)


def _mimo(key, label, P, s1, o1, s2, o2, amp, on, notes):
    fr = np.zeros((2, P), complex)
    fr[0, o1:o1 + len(s1)] = s1
    fr[1, o2:o2 + len(s2)] = s2
    return Frame(key, label, P, (fr * amp).astype(np.complex64), [(o1, s1), (o2, s2)],
                 np.array([[1, 0], [0, 1]], complex), on, notes=notes)


def pulse(amp=0.7, width=2, P=1024):
    """Unmodulated burst: its 2-sample width gives the range resolution, so energy is tiny.
    Slot 2 starts at 0.4 P, not P/2, so the frame start stays unambiguous."""
    s = np.ones(width, complex)
    return _mimo("pulse", f"Pulse ({width} samples, {width / FS * 1e9:.0f} ns)", P, s, 0, s, int(0.4 * P), amp, width,
                 {"family": "pulse", "tx_while_rx": False})


def lfm(amp=0.7, width=256, P=1024):
    """Pulse compression: a gated chirp, then the transmitter is silent."""
    return _mimo("lfm", f"LFM pulse ({width / FS * 1e6:.1f} us chirp, {width / P:.0%} duty)", P,
                 chirp(width, tap=0.15), 0, chirp(width, down=True, tap=0.15), int(0.4 * P), amp, width,
                 {"family": "pulse", "tx_while_rx": False})


def fmcw(amp=0.7, T=512, key="fmcw"):
    """Continuous sweep: TX1 sweeps up, then TX2 sweeps down, end to end. Something always
    radiates while receiving. Compressed digitally (matched filter), not by a mixer dechirp."""
    return _mimo(key, f"FMCW ({T / FS * 1e6:.1f} us sweep per TX)", 2 * T, chirp(T), 0, chirp(T, down=True), T,
                 amp, T, {"family": "fmcw", "tx_while_rx": True})


TB_SLOT, TB_PULSE, TB_NPH = 1024, 256, 12
TB_PHASES = np.arange(TB_NPH) * 2 * np.pi / TB_NPH


def txbeam(amp=0.5):
    """Physical TX beamforming frame (both TX at amp each, so the beam slot can add to 2x field).
    slot 0: down-chirp TX1 (frame sync); slot 1: TX1 alone; slot 2: TX2 alone;
    slot 3+i: TX1 + TX2 * exp(j phase_i)."""
    up, dn = chirp(TB_PULSE, tap=0.15), chirp(TB_PULSE, down=True, tap=0.15)
    n_slot = 3 + TB_NPH
    P = TB_SLOT * n_slot
    fr = np.zeros((2, P), complex)
    fr[0, :TB_PULSE] = dn
    fr[0, TB_SLOT:TB_SLOT + TB_PULSE] = up
    fr[1, 2 * TB_SLOT:2 * TB_SLOT + TB_PULSE] = up
    w = [[1, 0], [1, 0], [0, 1]]
    for i, ph in enumerate(TB_PHASES):
        s = (3 + i) * TB_SLOT
        fr[0, s:s + TB_PULSE] = up
        fr[1, s:s + TB_PULSE] = up * np.exp(1j * ph)
        w.append([1, np.exp(1j * ph)])
    slots = [(0, dn)] + [(k * TB_SLOT, up) for k in range(1, n_slot)]
    return Frame("txbeam", "TX beam scan (12 steered beams + 2 single-TX slots)", P, (fr * amp).astype(np.complex64),
                 slots, np.array(w, complex), TB_PULSE, kind="txbeam", phases=TB_PHASES,
                 notes={"family": "pulse", "tx_while_rx": False})


def registry():
    return {"pulse": pulse(), "lfm": lfm(), "fmcw": fmcw(), "fmcw_long": fmcw(T=1024, key="fmcw_long"), "txbeam": txbeam()}


MIMO_KEYS = ["pulse", "lfm", "fmcw", "fmcw_long"]
