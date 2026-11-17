"""Radio back-ends with one interface: the real AD9361 board (libiio) and a physics simulator.

The board layer follows the radio behaviour measured on these boards earlier:
  * RX tracking loops converge on a TONE for 2 s, then all are frozen (ready()).
  * IQ is complex64 with |x| = 1.0 at converter full scale.

The simulator plays the transmitted frame through a geometric channel: direct leakage,
static walls and an articulated walking/waving person. It uses the same antenna layout,
so beamforming, ranging and the waveform comparison can be checked without hardware.
"""
from __future__ import annotations

import time

import numpy as np

RX_FS = 2048.0      # 12-bit ADC, right-justified in 16 bits
TX_FS = 32767.0     # 12-bit DAC, left-justified in 16 bits
C = 299_792_458.0

CFG = dict(fs=61.44e6, lo=5.845e9, rf_bw=56e6, tx_gain=(0, 0), rx_gain=(45, 45))


def scan_usb():
    try:
        import iio
        return sorted(u for u in iio.scan_contexts() if u.startswith("usb:"))
    except Exception:
        return []


class Radio:
    kind = "board"

    def __init__(self, uri="usb:"):
        import adi
        self.uri = uri
        self.s = adi.ad9361(uri)
        self.phy = self.s.ctx.find_device("ad9361-phy")
        self.adc = self.s.ctx.find_device("cf-ad9361-lpc")
        self.dds = self.s.ctx.find_device("cf-ad9361-dds-core-lpc")
        self.s.rx_enabled_channels = [0, 1]
        self.s.tx_enabled_channels = [0, 1]
        self.s.tx_cyclic_buffer = True
        self.n, self.buf, self.fs, self.lo = 0, None, 0.0, 0.0
        self._dds_off()

    def _dds_off(self):
        for c in self.dds.channels:
            if c.id.startswith("altvoltage"):
                c.attrs["scale"].value = "0"

    def configure(self, fs, lo, rf_bw, tx_gain, rx_gain):
        s = self.s
        self.tx_stop()
        self._drop_rx()
        s.sample_rate = int(fs)
        s.rx_rf_bandwidth = int(rf_bw)
        s.tx_rf_bandwidth = int(rf_bw)
        s.rx_lo = int(lo)
        s.tx_lo = int(lo)
        s.gain_control_mode_chan0 = "manual"
        s.gain_control_mode_chan1 = "manual"
        s.rx_hardwaregain_chan0, s.rx_hardwaregain_chan1 = rx_gain
        s.tx_hardwaregain_chan0, s.tx_hardwaregain_chan1 = tx_gain
        self.fs, self.lo = float(fs), float(lo)

    def tracking(self, bb_dc, rf_dc, quad):
        for c in self.phy.channels:
            if c.id == "voltage0" and not c.output:
                c.attrs["bb_dc_offset_tracking_en"].value = str(int(bb_dc))
                c.attrs["rf_dc_offset_tracking_en"].value = str(int(rf_dc))
                c.attrs["quadrature_tracking_en"].value = str(int(quad))

    def tx_cyclic(self, frame):
        self.tx_stop()
        self.s.tx([np.round(frame[0] * TX_FS), np.round(frame[1] * TX_FS)])

    def tx_stop(self):
        try:
            self.s.tx_destroy_buffer()
        except Exception:
            pass
        self._dds_off()

    def _drop_rx(self):
        self.buf, self.n = None, 0
        try:
            self.s.rx_destroy_buffer()
        except Exception:
            pass

    def capture(self, n):
        """(2, n) gapless samples, newest data only (one kernel buffer)."""
        if n != self.n or self.buf is None:
            import iio
            self._drop_rx()
            for ch in self.adc.channels:
                ch.enabled = ch.id in ("voltage0", "voltage1", "voltage2", "voltage3")
            try:
                self.adc.set_kernel_buffers_count(1)
            except Exception:
                pass
            self.buf = iio.Buffer(self.adc, n, False)
            self.n = n
            self.buf.refill()
        self.buf.refill()
        r = np.frombuffer(self.buf.read(), np.int16).reshape(-1, 4).astype(np.float32)
        return (r[:, 0::2] + 1j * r[:, 1::2]).T.astype(np.complex64) / RX_FS

    def temp_c(self):
        for c in self.phy.channels:
            if c.id == "temp0":
                return float(c.attrs["input"].value) / 1000
        return float("nan")

    def close(self):
        try:
            self.tx_stop()
            self.tracking(1, 1, 1)
            self._drop_rx()
        except Exception:
            pass


def ready(radio, frame, seconds=2.0):
    """Converge the RX loops on a tone, freeze them, then start the radar frame.
    Quadrature tracking left on adds ~20 dB of leakage modulation in the walking Doppler band."""
    n = 16384
    tone = 0.5 * np.exp(2j * np.pi * round(10e6 / radio.fs * n) * np.arange(n) / n)
    radio.tracking(1, 1, 1)
    radio.tx_cyclic(np.stack([tone, tone]).astype(np.complex64))
    t0 = time.time()
    while time.time() - t0 < seconds:
        radio.capture(n)
    radio.tracking(0, 0, 0)
    radio.tx_cyclic(frame)


# ====================================================================== simulator
class Person:
    """Articulated body as point scatterers. Positions in metres, radar at the origin,
    y = boresight, x = along the antenna row (toward the RX end). Scripted path loops."""

    # (time s, x, y, mode) waypoints; mode: walk | wave | still | out
    # scene coordinates: origin at the centroid of the four antennas
    PATH = [(0, 0.0, 12.0, "out"), (6, 2.0, 7.5, "walk"), (10, 1.0, 6.0, "walk"), (13, 0.0, 5.0, "walk"),
            (19, 0.0, 1.8, "walk"), (21, 0.0, 1.6, "wave"), (28, 0.4, 1.5, "still"), (32, 0.4, 1.5, "walk"),
            (37, -2.2, 6.0, "walk"), (42, 2.5, 6.8, "walk"), (47, 2.5, 6.8, "wave"), (53, 0.0, 12.0, "walk"),
            (59, 0.0, 12.0, "out")]

    def __init__(self):
        self.T = self.PATH[-1][0]

    def where(self, t):
        t = t % self.T
        P = self.PATH
        for i in range(len(P) - 1):
            if P[i][0] <= t < P[i + 1][0]:
                a, b = P[i], P[i + 1]
                u = (t - a[0]) / (b[0] - a[0])
                mode = a[3]
                if mode in ("wave", "still", "out"):
                    return np.array([a[1], a[2]]), np.zeros(2), mode
                p = np.array([a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])])
                v = np.array([b[1] - a[1], b[2] - a[2]]) / (b[0] - a[0])
                return p, v, mode
        return np.array([0, 12.0]), np.zeros(2), "out"

    def scatterers(self, t):
        """[(x, y, amplitude)] at time t; amplitude is field (sqrt RCS) relative units."""
        p, v, mode = self.where(t)
        if mode == "out":
            return []
        sp = np.linalg.norm(v)
        u = v / sp if sp > 1e-6 else np.array([0.0, -1.0])
        out = [(p[0], p[1], 1.0), (p[0], p[1] + 0.02 * np.sin(2 * np.pi * 0.25 * t), 0.35)]  # torso + chest
        if mode == "walk":
            fg = 0.9 * max(sp, 0.6)                     # gait cycles / s
            ph = 2 * np.pi * fg * t
            for s in (1, -1):                           # legs: forward swing up to ~2.4x torso speed
                d = 0.35 * s * np.sin(ph)
                out.append((p[0] + d * u[0], p[1] + d * u[1], 0.35))
                d = 0.28 * -s * np.sin(ph)              # arms swing opposite
                out.append((p[0] + d * u[0], p[1] + d * u[1], 0.22))
        elif mode == "wave":
            ph = 2 * np.pi * 1.3 * t                    # both forearms waving toward/away from radar
            for s, amp in ((1, 0.30), (-1, 0.22)):
                out.append((p[0] + 0.25 * s, p[1] - amp * (1 + np.sin(ph + (s > 0) * 1.2)), 0.3))
        else:                                           # still: breathing only (on the chest)
            pass
        return out


class SimRadio:
    """Same calls as Radio. Captures are paced like the USB link (~2.4 MS/s per channel)."""
    kind = "sim"

    def __init__(self, uri="sim", seed=1, geom=None, chain_phase_deg=(0, 37, 0, -64), noise_dbfs=-58.0):
        self.uri = uri
        self.rng = np.random.default_rng(seed)
        # antenna positions [x, y] relative to their centroid (from the node's Geometry)
        if geom is None:
            import dsp
            geom = dsp.Geometry(None, C / 5.845e9)
        self.p_tx, self.p_rx = geom.tx.copy(), geom.rx.copy()
        # fixed chain phases: TX1, TX2, RX1, RX2 (cables, PCB) -> what calibration must find
        cp = np.radians(chain_phase_deg)
        self.c_tx, self.c_rx = np.exp(1j * cp[:2]), np.exp(1j * cp[2:])
        self.noise = 10 ** (noise_dbfs / 20) / np.sqrt(2)
        self.person = Person()
        self.frame = None
        self.fs, self.lo = 61.44e6, 5.845e9
        self.t0 = time.time()
        self.next_t = time.time()
        self.leak = np.array([[0.45, 0.50], [0.30, 0.42]]) * np.exp(1j * np.array([[0.3, 1.9], [-2.2, 0.8]]))
        self.walls = [(-1.8, 3.4, 0.25), (0.2, 7.9, 0.4), (3.0, 5.0, 0.2), (-0.5, 0.9, 0.12), (0.3, -2.5, 0.3)]   # last: behind   # static clutter
        self.ph = 0.0

    # board-only controls are no-ops
    def configure(self, fs, lo, rf_bw, tx_gain, rx_gain):
        self.fs, self.lo = float(fs), float(lo)

    def tracking(self, *a):
        pass

    def tx_cyclic(self, frame):
        self.frame = np.asarray(frame, np.complex64)
        self.F = np.fft.fft(self.frame, axis=-1)
        P = self.frame.shape[-1]
        self.f = np.fft.fftfreq(P)                          # cycles / sample

    def tx_stop(self):
        self.frame = None

    def temp_c(self):
        return 40.0

    def close(self):
        pass

    def truth(self, t=None):
        t = (time.time() - self.t0) if t is None else t
        p, v, mode = self.person.where(t)
        q = p                                               # scene coordinates are centred on the antennas
        r = float(np.hypot(*q))
        return {"t": t, "x": float(q[0]), "y": float(q[1]), "r": r, "theta_deg": float(np.degrees(np.arctan2(q[0], q[1]))),
                "v_radial": float(np.dot(v, q) / max(r, 1e-6)), "mode": mode}

    def _channel(self, t):
        """H (2 tx, 2 rx, P) in the frequency domain of the frame (exact near-field geometry)."""
        k = 2 * np.pi * self.lo / C
        sc = [(x, y, a * 0.8) for x, y, a in self.walls] + [(x, y, 0.55 * a) for x, y, a in self.person.scatterers(t)]
        sc = np.array(sc)                                                   # (ns, 3)
        d_t = np.hypot(sc[None, :, 0] - self.p_tx[:, None, 0], sc[None, :, 1] - self.p_tx[:, None, 1])  # (2, ns)
        d_r = np.hypot(sc[None, :, 0] - self.p_rx[:, None, 0], sc[None, :, 1] - self.p_rx[:, None, 1])
        L = d_t[:, None, :] + d_r[None, :, :]                               # (2, 2, ns)
        g = sc[:, 2] * 0.04 / (d_t[:, None, :] * d_r[None, :, :]) * np.exp(-1j * k * L)
        tau = L / C * self.fs + 0.41                                        # 0.41: where range zero sits (samples)
        # fractional delays as a 64-tap windowed-sinc FIR, then one FFT per path (cheap for long frames)
        n = np.arange(-16, 48)
        dn = n[None, None, None, :] - tau[..., None]                        # (2, 2, ns, 64)
        ker = np.sinc(dn) * np.where(np.abs(dn) < 16, 0.5 + 0.5 * np.cos(np.pi * dn / 16), 0)
        h = np.einsum("trs,trsn->trn", g, ker)
        lk = np.sinc(n - 0.41) * (0.5 + 0.5 * np.cos(np.pi * (n - 0.41) / 16))
        h = h + self.leak[..., None] * lk
        P = len(self.f)
        hp = np.zeros((2, 2, P), complex)
        hp[..., :48] += h[..., 16:]
        hp[..., -16:] += h[..., :16]
        H = np.fft.fft(hp, axis=-1)
        return H * (self.c_tx[:, None, None] * self.c_rx[None, :, None])

    def capture(self, n):
        # pace to the USB rate measured on the board
        dt = max(n / 2.4e6, 1 / 600)
        now = time.time()
        if self.next_t > now:
            time.sleep(self.next_t - now)
        self.next_t = max(self.next_t, now) + dt
        if self.frame is None:
            return (self.rng.standard_normal((2, n)) + 1j * self.rng.standard_normal((2, n))).astype(np.complex64) * self.noise
        t = time.time() - self.t0
        H = self._channel(t)
        Y = np.einsum("tf,trf->rf", self.F, H)
        y = np.fft.ifft(Y, axis=-1)
        P = y.shape[-1]
        off = int(self.rng.integers(P))
        reps = int(np.ceil((n + off) / P))
        y = np.tile(y, reps)[:, off:off + n]
        # common phase noise (shared LOs) + a little leakage-amplitude flutter
        self.ph = 0.97 * self.ph + np.radians(1.2) * self.rng.standard_normal()
        y = y * np.exp(1j * self.ph) * (1 + 0.002 * self.rng.standard_normal())
        y = y + self.noise * (self.rng.standard_normal((2, n)) + 1j * self.rng.standard_normal((2, n)))
        return y.astype(np.complex64)


def open_radio(uri, geom=None):
    if uri == "sim" or uri.startswith("sim"):
        return SimRadio(uri, seed=abs(hash(uri)) % 1000, geom=geom)
    return Radio(uri)
