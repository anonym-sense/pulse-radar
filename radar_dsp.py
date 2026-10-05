"""Pulse-radar DSP: waveform generation, pulse compression, quality metrics,
slow-time (MTI / range-Doppler) processing.

Conventions
-----------
* IQ arrays are complex64, scaled so that |x| = 1.0 is converter full scale.
* The transmitter plays one "frame" of `period` samples cyclically. TX1 carries
  an up-chirp at sample 0, TX2 a down-chirp at period/2, so the two transmitters
  can be told apart regardless of where the receive capture happens to start.
* A receive capture is `k * period` contiguous samples, so it folds exactly onto
  one period. Range bin 0 is re-anchored on the direct TX->RX leakage.
"""
from __future__ import annotations

import numpy as np

C = 299_792_458.0


# ----------------------------------------------------------------- waveforms
def make_pulse(fs: float, bw: float, length: int, down: bool = False,
               taper: float = 0.15) -> np.ndarray:
    """Linear-FM pulse of `length` samples sweeping `bw` Hz, Tukey-tapered."""
    n = np.arange(length)
    t = (n - (length - 1) / 2) / fs
    k = bw / (length / fs)
    if down:
        k = -k
    p = np.exp(1j * np.pi * k * t ** 2)
    if taper > 0:
        m = max(1, int(taper * length / 2))
        w = np.ones(length)
        ramp = 0.5 - 0.5 * np.cos(np.pi * (np.arange(m) + 0.5) / m)
        w[:m] = ramp
        w[-m:] = ramp[::-1]
        p = p * w
    return p.astype(np.complex64)


def make_frame(fs: float, bw: float, pulse_len: int, period: int,
               tx_mode: str = "tx1", amplitude: float = 0.5):
    """Build one cyclic TX frame.

    Returns (frame, refs): frame is (2, period) complex64 to send to TX1/TX2,
    refs is (2, period) unit-amplitude matched-filter references (always both,
    so a silent transmitter can still be probed for leakage/crosstalk).
    """
    if pulse_len > period // 2:
        raise ValueError("pulse_len must be <= period/2")
    refs = np.zeros((2, period), np.complex64)
    refs[0, :pulse_len] = make_pulse(fs, bw, pulse_len, down=False)
    refs[1, period // 2:period // 2 + pulse_len] = make_pulse(fs, bw, pulse_len, down=True)
    on = {"tx1": (1, 0), "tx2": (0, 1), "tdm": (1, 1)}[tx_mode]
    frame = refs * (amplitude * np.array(on, np.float32)[:, None])
    return frame.astype(np.complex64), refs


# --------------------------------------------------------- pulse compression
def fold(rx: np.ndarray, period: int) -> np.ndarray:
    """(n_rx, k*period) -> (n_rx, k, period)."""
    k = rx.shape[-1] // period
    return rx[..., :k * period].reshape(rx.shape[0], k, period)


def make_window(fs: float, bw: float, period: int, kind: str = "hamming") -> np.ndarray:
    """Spectral weighting for the matched filter. "hamming" trades ~1.5x
    mainlobe width for ~-40 dB range sidelobes (plain LFM gives -13 dB, which
    lets the direct leakage bury nearby echoes)."""
    f = np.fft.fftfreq(period, 1 / fs)
    if kind == "none":
        return np.ones(period)
    w = 0.54 + 0.46 * np.cos(2 * np.pi * f / (1.04 * bw))
    w[np.abs(f) > 0.52 * bw] = 0
    return w


def compress(rx_mean: np.ndarray, refs: np.ndarray, win=None) -> np.ndarray:
    """Circular (optionally sidelobe-weighted) matched filter.

    rx_mean (n_rx, period), refs (n_tx, period) -> h (n_tx, n_rx, period): the
    channel impulse response, in units of full scale per unit TX amplitude.
    """
    R = np.fft.fft(rx_mean, axis=-1)
    F = np.fft.fft(refs, axis=-1)
    if win is None:
        win = np.ones(refs.shape[-1])
    e = np.mean(np.abs(F) ** 2 * win, axis=-1)
    return np.fft.ifft(R[None, :, :] * np.conj(F[:, None, :]) * win, axis=-1) / e[:, None, None]


def find_delay(h: np.ndarray, active_tx) -> int:
    """Sample index of the direct path, common to all active paths."""
    p = np.sum(np.abs(h[list(active_tx)]) ** 2, axis=(0, 1))
    return int(np.argmax(p))


def upsample_profile(h: np.ndarray, factor: int = 4) -> np.ndarray:
    """Band-limited interpolation of a circular profile along the last axis."""
    n = h.shape[-1]
    H = np.fft.fft(h, axis=-1)
    out = np.zeros(h.shape[:-1] + (n * factor,), complex)
    half = n // 2
    out[..., :half] = H[..., :half]
    out[..., -half:] = H[..., -half:]
    return np.fft.ifft(out, axis=-1) * factor


def db(x, floor=1e-12):
    return 10 * np.log10(np.maximum(np.abs(x) ** 2, floor))


def path_quality(h_path: np.ndarray, folded_rx: np.ndarray, ref: np.ndarray,
                 d0: int, fs: float, pulse_len: int) -> dict:
    """Quality metrics for one TX->RX path.

    h_path (period,), folded_rx (k, period) raw periods of that RX channel,
    ref (period,) that TX's reference.
    """
    period = h_path.size
    up = 8
    hu = upsample_profile(np.roll(h_path, -d0), up)
    mag = np.abs(hu)
    win = 6 * up
    seg = np.concatenate([mag[-win:], mag[:win + 1]])
    pk_i = int(np.argmax(seg)) - win
    pk = mag[pk_i]
    pk_db = float(db(pk))

    # -3 dB mainlobe width, and first nulls for sidelobe measurement
    def walk(step):
        i = pk_i
        while mag[(i + step) % mag.size] < mag[i % mag.size] and abs(i - pk_i) < 4 * up:
            i += step
        return i
    lo, hi = walk(-1), walk(+1)
    a = pk_i
    while mag[a % mag.size] > pk / np.sqrt(2) and pk_i - a < 4 * up:
        a -= 1
    b = pk_i
    while mag[b % mag.size] > pk / np.sqrt(2) and b - pk_i < 4 * up:
        b += 1
    width = (b - a) / up
    idx = np.arange(mag.size)
    rel = (idx - pk_i + mag.size // 2) % mag.size - mag.size // 2
    near = (np.abs(rel) <= 40 * up) & ((rel < lo - pk_i) | (rel > hi - pk_i))
    far = np.abs(rel) > 60 * up
    pslr = float(db(mag[near].max()) - pk_db) if near.any() else float("nan")
    floor_db = float(db(np.median(mag[far]))) if far.any() else float("nan")

    # raw (pre-compression) signal and noise, from period-to-period repeatability
    k = folded_rx.shape[0]
    mean = folded_rx.mean(0)
    shifted_ref = np.roll(ref, d0)
    on = np.abs(shifted_ref) > 0.5
    sig_p = float(np.mean(np.abs(mean[on]) ** 2))
    if k > 1:
        noise_p = float(np.mean(np.abs(folded_rx - mean) ** 2) * k / (k - 1))
    else:
        noise_p = float("nan")
    z = folded_rx @ np.conj(shifted_ref)          # per-period complex peak
    amp_ripple = float(np.std(20 * np.log10(np.abs(z) + 1e-12))) if k > 1 else float("nan")
    ph = np.angle(z * np.conj(z.mean()))
    return {
        "peak_db": pk_db,                                  # path gain, dB re full scale
        "delay_frac": pk_i / up,                           # samples, relative to d0
        "phase_deg": float(np.degrees(np.angle(hu[pk_i]))),
        "width_bins": float(width),
        "width_m": float(width * C / (2 * fs)),
        "pslr_db": pslr,
        "floor_db": floor_db,
        "dyn_range_db": pk_db - floor_db,
        "raw_snr_db": float(10 * np.log10(sig_p / noise_p)) if noise_p and noise_p > 0 else float("nan"),
        "comp_snr_db": float(10 * np.log10(sig_p / noise_p * pulse_len * k)) if noise_p and noise_p > 0 else float("nan"),
        "amp_ripple_db": amp_ripple,
        "phase_jitter_deg": float(np.degrees(np.std(ph))) if k > 1 else float("nan"),
    }


def channel_stats(rx: np.ndarray) -> list:
    """Converter-level health of each RX channel."""
    out = []
    for x in rx:
        i, q = x.real, x.imag
        pk = float(max(np.abs(i).max(), np.abs(q).max()))
        out.append({
            "peak_dbfs": float(20 * np.log10(pk + 1e-9)),
            "rms_dbfs": float(10 * np.log10(np.mean(np.abs(x) ** 2) + 1e-18)),
            "clip_pct": float(100 * np.mean((np.abs(i) > 0.97) | (np.abs(q) > 0.97))),
            "dc_dbfs": float(20 * np.log10(np.abs(x.mean()) + 1e-9)),
        })
    return out


def spectrum_db(x: np.ndarray, nfft: int = 512) -> np.ndarray:
    """Averaged periodogram (fftshifted), dB re full-scale sine per bin."""
    n = (x.size // nfft) * nfft
    if n == 0:
        x = np.pad(x, (0, nfft - x.size))
        n = nfft
    seg = x[:n].reshape(-1, nfft) * np.hanning(nfft)
    P = np.mean(np.abs(np.fft.fft(seg, axis=-1)) ** 2, axis=0) / (np.sum(np.hanning(nfft)) ** 2)
    return 10 * np.log10(np.fft.fftshift(P) + 1e-18)


# ------------------------------------------------------------------ slow time
class SlowTime:
    """Ring buffer of direct-path-anchored range profiles for one TX->RX path,
    with clutter removal and range-Doppler processing."""

    def __init__(self, n_range: int, n_frames: int = 64, bg_alpha: float = 0.02):
        self.n_range = n_range
        self.n_frames = n_frames
        self.bg_alpha = bg_alpha
        self.reset()

    def reset(self):
        self.buf = np.zeros((self.n_frames, self.n_range), complex)
        self.t = np.zeros(self.n_frames)
        self.count = 0
        self.bg = None
        self.unwrapped = 0.0
        self.prev = None

    def push(self, profile: np.ndarray, t: float):
        self.buf = np.roll(self.buf, -1, axis=0)
        self.t = np.roll(self.t, -1)
        self.buf[-1] = profile
        self.t[-1] = t
        self.count += 1
        if self.bg is None:
            self.bg = profile.copy()
        else:
            self.bg += self.bg_alpha * (profile - self.bg)

    @property
    def filled(self):
        return min(self.count, self.n_frames)

    def frame_rate(self):
        n = self.filled
        if n < 3:
            return 0.0
        dt = np.diff(self.t[-n:])
        return float(1.0 / np.median(dt)) if np.median(dt) > 0 else 0.0

    def mti(self):
        """Latest profile with the slowly-adapting static background removed."""
        return self.buf[-1] - self.bg

    def uniform(self):
        """Frames resampled onto a uniform slow-time grid (host timing is
        irregular). Returns (frames, prf)."""
        n = self.filled
        x, t = self.buf[-n:], self.t[-n:]
        if n < 4 or t[-1] <= t[0]:
            return x, 0.0
        tu = np.linspace(t[0], t[-1], n)
        y = np.empty_like(x)
        for r in range(x.shape[1]):
            y[:, r] = np.interp(tu, t, x[:, r].real) + 1j * np.interp(tu, t, x[:, r].imag)
        return y, (n - 1) / (t[-1] - t[0])

    def doppler(self, nfft: int = 64, dc: bool = True, hp_hz: float = 0.0,
                sg: str = "off", sg_win: int = 15, sg_order: int = 3,
                band_lo: float = 0.0, band_hi: float = 0.0, window: str = "hann"):
        """Slow-time filtering and FFT of every range bin.

        Returns (X[n_doppler, n_range] complex, fftshifted, prf). Optional stages,
        applied in this order to the uniformly re-sampled slow-time buffer:

          dc       subtract each range bin's slow-time mean (static clutter)
          sg       Savitzky-Golay polynomial fit over `sg_win` frames:
                   "smooth" keeps the fit (low-pass), "detrend" subtracts it
                   (removes slow drift without the ringing of a sharp filter)
          hp_hz    zero-phase 4th-order Butterworth high-pass in slow time
          window   taper before the FFT: hann | hamming | blackman | rect
          band_lo, band_hi   Doppler-domain band limits in Hz applied to the
                   spectrum (smooth 4th-order edges); 0 disables that edge
        """
        x, prf = self.uniform()
        n = x.shape[0]
        if n < 8 or prf <= 0:
            return None, 0.0
        if dc:
            x = x - x.mean(axis=0, keepdims=True)
        if sg != "off":
            from scipy.signal import savgol_filter
            win = min(int(sg_win) | 1, n if n % 2 else n - 1)
            order = min(int(sg_order), win - 1)
            if win > order:
                fit = (savgol_filter(x.real, win, order, axis=0)
                       + 1j * savgol_filter(x.imag, win, order, axis=0))
                x = fit if sg == "smooth" else x - fit
        if 0 < hp_hz < 0.45 * prf and n > 30:
            from scipy.signal import butter, sosfiltfilt
            sos = butter(4, hp_hz / (prf / 2), "highpass", output="sos")
            x = sosfiltfilt(sos, x, axis=0)
        w = {"hann": np.hanning, "hamming": np.hamming, "blackman": np.blackman,
             "rect": np.ones}.get(window, np.hanning)(n)[:, None]
        nf = max(nfft, n)
        X = np.fft.fftshift(np.fft.fft(x * w, n=nf, axis=0), axes=0) / w.sum()
        if band_lo > 0 or band_hi > 0:
            f = np.abs(np.fft.fftshift(np.fft.fftfreq(nf, 1 / prf)))
            g = np.ones(nf)
            if band_lo > 0:
                g = g / np.sqrt(1 + (band_lo / np.maximum(f, 1e-9)) ** 8)
            if band_hi > 0:
                g = g / np.sqrt(1 + (f / band_hi) ** 8)
            X = X * g[:, None]
        return X, prf

    def track_phase(self, bin_idx: int, lam: float, min_abs: float = 0.0):
        """Radial displacement (m) of the scatterer in `bin_idx`, from the
        unwrapped phase of the clutter-removed return. Phase is only integrated
        while the return is above `min_abs`; below that it is noise and would
        random-walk."""
        z = self.buf[-1, bin_idx] - self.bg[bin_idx]
        if abs(z) <= min_abs:
            self.prev = None
            return -self.unwrapped * lam / (4 * np.pi)
        if self.prev is not None:
            self.unwrapped += float(np.angle(z * np.conj(self.prev)))
        self.prev = z
        return -self.unwrapped * lam / (4 * np.pi)
