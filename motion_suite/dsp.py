"""Signal processing shared by every node.

Fast time (per capture): matched filter -> channel profile h[slot, rx, delay bin], anchored so
bin A holds the direct TX->RX leakage.

Slow time (per display tick): uniform resampling, phase-noise lock, static removal, Doppler,
background whitening, detection, MIMO beamforming.
"""
from __future__ import annotations

import base64

import numpy as np

import waveforms as W

A = 2               # anchor: index of the leakage peak in a cut profile
NBK = 18            # delay bins kept per profile (~44 m)
RB = W.C / (2 * W.FS)   # metres per delay bin (2.44 m)


def spectral_window(P):
    """Hamming over the 50 MHz occupied band: identical for every waveform, so their range
    responses are the same and only energy / leakage behaviour differs."""
    f = np.fft.fftfreq(P, 1 / W.FS)
    w = 0.54 + 0.46 * np.cos(2 * np.pi * f / (1.04 * W.BW))
    w[np.abs(f) > 0.52 * W.BW] = 0
    return w


class Compressor:
    """MIMO frames: one matched filter per TDM slot over the whole frame."""

    def __init__(self, fr: W.Frame):
        self.fr = fr
        P = fr.P
        Wn = spectral_window(P)
        G = []
        for o, s in fr.slots:
            r = np.zeros(P, complex)
            r[o:o + len(s)] = s
            F = np.fft.fft(r)
            G.append(np.conj(F) * Wn / np.mean(np.abs(F) ** 2 * Wn))
        self.G = np.array(G)
        self.idx0 = np.arange(NBK) - A

    def process(self, x):
        P = self.fr.P
        k = x.shape[-1] // P
        m = x[:, :k * P].reshape(2, k, P).mean(1)
        R = np.fft.fft(m, axis=-1)
        h = np.fft.ifft(R[None] * self.G[:, None], axis=-1)       # (S, 2, P)
        d0 = int(np.argmax(np.sum(np.abs(h) ** 2, axis=(0, 1))))   # all slots agree only at the true start
        return h[..., (self.idx0 + d0) % P].astype(np.complex64)


class BeamCompressor:
    """TX-beam frame: sync on the unique down-chirp, then a short matched filter per slot."""

    def __init__(self, fr: W.Frame):
        self.fr = fr
        S = W.TB_SLOT
        up = fr.slots[1][1]
        dn = fr.slots[0][1]
        Wn = spectral_window(S)
        U = np.fft.fft(up, S)
        self.G = np.conj(U) * Wn / np.mean(np.abs(U) ** 2 * Wn)
        z = np.zeros(fr.P, complex)
        z[:len(dn)] = dn
        self.Sync = np.conj(np.fft.fft(z))

    def process(self, x):
        P, S, n_s = self.fr.P, W.TB_SLOT, self.fr.n_slots
        x = x[:, :P]
        c = np.fft.ifft(np.fft.fft(x, axis=-1) * self.Sync, axis=-1)
        d0 = int(np.argmax(np.sum(np.abs(c) ** 2, axis=0)))
        y = np.roll(x, -d0 + A, axis=-1).reshape(2, n_s, S).transpose(1, 0, 2)
        h = np.fft.ifft(np.fft.fft(y, axis=-1) * self.G, axis=-1)
        return h[..., :NBK].astype(np.complex64)


def compressor(fr):
    return BeamCompressor(fr) if fr.kind == "txbeam" else Compressor(fr)


# ------------------------------------------------------------------ slow time
def uniform(T, H):
    """Host timestamps are irregular; put slow time on a uniform grid (linear interpolation)."""
    n = len(T)
    tu = np.linspace(T[0], T[-1], n)
    j = np.clip(np.searchsorted(T, tu, side="right") - 1, 0, n - 2)
    w = ((tu - T[j]) / np.maximum(T[j + 1] - T[j], 1e-9)).astype(np.float32)
    w = w.reshape((-1,) + (1,) * (H.ndim - 1))
    return tu, H[j] * (1 - w) + H[j + 1] * w


def common_mode(z):
    """Per-capture phasor shared by all leakage paths (both chains share each LO), least
    squares against the median path values. Dividing it out removes phase noise only."""
    L = np.median(z.real, 0) + 1j * np.median(z.imag, 0)
    c = np.sum(np.conj(L) * z, axis=1) / np.sum(np.abs(L) ** 2)
    return c / np.maximum(np.abs(c), 1e-12)


def doppler(x, nfft=None):
    """(K, ...) complex slow time -> fftshifted spectrum, unit gain for a tone."""
    K = x.shape[0]
    w = np.hanning(K).astype(np.float32).reshape((-1,) + (1,) * (x.ndim - 1))
    nfft = nfft or K
    return np.fft.fftshift(np.fft.fft(x * w, n=nfft, axis=0), axes=0) / w.sum()


def peak_pos(mag, half=4, up=32):
    """Band-limited peak position near A, in bins relative to A."""
    seg = mag[A - min(half, A):A + half + 1]
    lo = A - min(half, A)
    n = len(seg)
    F = np.fft.fft(seg)
    Z = np.zeros(up * n, complex)
    Z[:n // 2] = F[:n // 2]
    Z[-(n - n // 2):] = F[n // 2:]
    return float(np.argmax(np.abs(np.fft.ifft(Z))) / up + lo - A)


def upsample_last(X, U):
    """Fourier interpolation along the last axis (range): smoother display, no new resolution."""
    N = X.shape[-1]
    F = np.fft.fft(X, axis=-1)
    Z = np.zeros(X.shape[:-1] + (N * U,), complex)
    h = N // 2
    Z[..., :h] = F[..., :h]
    Z[..., -(N - h):] = F[..., h:]
    return np.fft.ifft(Z, axis=-1) * U


def regrid(axis_src, Y, axis_dst):
    """Linear interpolation along axis 0 from a monotonic source axis; zeros outside."""
    order = np.argsort(axis_src)
    s = axis_src[order]
    Y = Y[order]
    pos = np.interp(axis_dst, s, np.arange(len(s)), left=-1, right=-1)
    ok = pos >= 0
    p = np.clip(pos, 0, len(s) - 1.0001)
    i0 = p.astype(int)
    w = (p - i0).reshape((-1,) + (1,) * (Y.ndim - 1))
    out = Y[i0] * (1 - w) + Y[i0 + 1] * w
    out[~ok] = 0
    return out


# ------------------------------------------------------------------ beamforming
DEFAULT_ANTENNAS = {"tx": [[0.0, 0.0], [W.D_M, 0.0]], "rx": [[2 * W.D_M, 0.0], [3 * W.D_M, 0.0]], "measured": False}


class Geometry:
    """Where the four antennas really are. Positions [x, y] in metres: x along the antenna row
    (toward the RX side), y toward the scene; any origin, everything is re-centred on the
    centroid of the four, which is where range and angle are measured from. All antennas are
    assumed at about the same height.

    Steering uses the exact path TX -> point -> RX, so it stays right inside the near field
    (closer than 2 D^2 / lambda, D the TX-RX span), where plane-wave steering is wrong."""

    def __init__(self, ant, lam):
        ant = ant or DEFAULT_ANTENNAS
        tx = np.array(ant["tx"], float).reshape(2, -1)
        rx = np.array(ant["rx"], float).reshape(2, -1)
        tx = np.pad(tx, ((0, 0), (0, 2 - tx.shape[1])))[:, :2]
        rx = np.pad(rx, ((0, 0), (0, 2 - rx.shape[1])))[:, :2]
        c = np.vstack([tx, rx]).mean(0)
        self.tx, self.rx = tx - c, rx - c
        self.measured = bool(ant.get("measured", False))
        self.lam, self.k = lam, 2 * np.pi / lam
        self.pairs = [(t, r) for t in range(2) for r in range(2)]          # order t0r0, t0r1, t1r0, t1r1
        self.xv = np.array([self.tx[t, 0] + self.rx[r, 0] for t, r in self.pairs])
        self.d_tx = float(np.linalg.norm(self.tx[1] - self.tx[0]))
        span = max(np.linalg.norm(a - b) for a in np.vstack([self.tx, self.rx]) for b in np.vstack([self.tx, self.rx]))
        self.far_m = float(2 * span ** 2 / lam)
        self.unamb_deg = self._unambiguous(self.xv)
        self.tx_unamb_deg = float(np.degrees(np.arcsin(min(1.0, lam / (2 * max(self.d_tx, 1e-6))))))

    def _unambiguous(self, xv):
        """Half the distance in sin(theta) to the first grating lobe of the virtual array."""
        ds = np.linspace(1e-4, 2.0, 4000)
        g = np.abs(np.exp(1j * self.k * np.outer(ds, xv)).sum(1)) / len(xv)
        below = np.flatnonzero(g < 0.5)
        if not len(below):
            return 90.0
        after = np.flatnonzero((g > 0.9) & (np.arange(len(ds)) > below[0]))
        return 90.0 if not len(after) else float(np.degrees(np.arcsin(min(1.0, ds[after[0]] / 2))))

    def steer(self, theta_rad, r=None):
        """Steering vectors. r None: plane wave, (n_theta, 4). r scalar: exact paths at that
        range, (n_theta, 4). r array (NR,): (n_theta, 4, NR). Phase reference: the path 2 r."""
        th = np.atleast_1d(theta_rad)
        if r is None:
            return np.exp(1j * self.k * np.outer(np.sin(th), self.xv))
        rr = np.atleast_1d(np.asarray(r, float))
        px = np.sin(th)[:, None] * rr[None]
        py = np.cos(th)[:, None] * rr[None]
        dist = lambda e: np.hypot(px - e[0], py - e[1])
        L = np.stack([dist(self.tx[t]) + dist(self.rx[r_]) for t, r_ in self.pairs], axis=1)   # (n_th, 4, NR)
        a = np.exp(-1j * self.k * (L - 2 * rr[None, None, :]))
        return a[..., 0] if np.ndim(r) == 0 else a

    def info(self):
        return {"tx": np.round(self.tx, 4).tolist(), "rx": np.round(self.rx, 4).tolist(), "xv_mm": np.round(self.xv * 1e3, 1).tolist(),
                "d_tx_mm": round(self.d_tx * 1e3, 1), "far_m": round(self.far_m, 2), "unamb_deg": round(self.unamb_deg, 1),
                "tx_unamb_deg": round(self.tx_unamb_deg, 1), "measured": self.measured}


def calibrate(Rsum, a):
    """Element phases from the summed normalised covariance of a target whose steering vector
    a (4,) is known (known angle and range). Returns (cal (4,), quality = lambda1 / trace,
    1.0 means a perfect point target)."""
    ev, V = np.linalg.eigh(Rsum)
    u = V[:, -1]
    q = float(ev[-1] / max(np.trace(Rsum).real, 1e-20))
    cal = u * np.conj(a)
    cal = cal * np.exp(-1j * np.angle(cal[0]))
    return cal / np.abs(cal), q


# ------------------------------------------------------------------ transport
def u8(db, lo, hi):
    q = np.clip((np.nan_to_num(db, nan=lo) - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
    return base64.b64encode(q.tobytes()).decode()


def db10(x):
    return 10 * np.log10(np.maximum(x, 1e-30))
