"""Pulse-radar bring-up GUI server, for one or more boards.

    py -3.11 server.py [--port 8765] [--transport auto|iio|serial|sim] [--boards 2]

Open http://localhost:8765 . Each board has its own Engine and its own worker
thread (all access to that board's radio happens there); a Fusion thread
combines the boards' live angle estimates. Results go to the browser over a
websocket, tagged with the board index.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import queue
import threading
import time
import traceback

import numpy as np
import tornado.ioloop
import tornado.web
import tornado.websocket

import radar_dsp as dsp
from radios import RX_FS, TX_FS, open_radio, scan_usb

HERE = os.path.dirname(os.path.abspath(__file__))
N_NEG, N_RANGE = 4, 24          # range bins kept: -4 .. +19 around the direct path
UP = 4                          # display interpolation of range profiles

DEFAULTS = {
    "fs": 61.44e6, "lo": 5.8e9, "bw_frac": 0.8, "pulse_len": 256, "period": 1024,
    "tx_gain": [-10.0, -10.0], "rx_gain": [30.0, 30.0],
    "tx": 0, "rx": 1, "window": "hamming", "periods": 0, "cpi": 128,
    "rx_spacing_mm": 50.8, "angle_cal_deg": 0.0,
    "rx_x_mm": [-25.4, 25.4],   # RX1, RX2 position along the array line (multi-board geometry)
    # slow-time / Doppler filters (all optional; defaults give the plain display)
    "f_dc": True, "f_hp_hz": 0.0, "f_sg": "off", "f_sg_win": 15, "f_sg_order": 3,
    "f_band_lo": 0.0, "f_band_hi": 0.0, "f_window": "hann",
}

# Per-board defaults. Both boards transmit at once in the combined view, so their
# LOs are 70 MHz apart: wider than either chirp (0.8 x 61.44 = 49 MHz), so neither
# board's matched filter responds to the other's pulses, and both stay inside the
# 5.725-5.875 GHz band. Geometry: two 2-inch receive pairs on one line with a
# 50 mm gap between them, array centre at x = 0.
BOARD_DEFAULTS = [
    {"lo": 5.765e9, "port": "COM5", "rx_x_mm": [-75.8, -25.0]},
    {"lo": 5.835e9, "port": "COM6", "rx_x_mm": [25.0, 75.8]},
]
BOARD_NAMES = "ABCDEFGH"

# settings that change processing only, not the radio or the waveform
FILTER_KEYS = {"f_dc", "f_hp_hz", "f_sg", "f_sg_win", "f_sg_order", "f_band_lo", "f_band_hi", "f_window"}
ANGLE_KEYS = {"rx_spacing_mm", "angle_cal_deg", "rx_x_mm"}
SOFT_KEYS = {"tx", "rx", "window", "periods", "cpi", "port"} | ANGLE_KEYS | FILTER_KEYS


def clean(o):
    """Make numpy / NaN values JSON-safe."""
    if isinstance(o, dict):
        return {k: clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return clean(o.tolist())
    if isinstance(o, (np.floating, float)):
        f = float(o)
        return f if math.isfinite(f) else None
    if isinstance(o, np.integer):
        return int(o)
    return o


def r2(a, nd=2):
    return np.round(np.asarray(a, float), nd)


class Engine:
    def __init__(self, emit, idx=0, hub=None):
        self.idx, self.name, self.hub = idx, BOARD_NAMES[idx], hub
        self.emit = lambda m: emit({**m, "board": idx})
        self.cfg = dict(DEFAULTS, **(BOARD_DEFAULTS[idx] if idx < len(BOARD_DEFAULTS) else {}))
        self.cfg["rx_x_mm"] = list(self.cfg["rx_x_mm"])
        self.radio = None
        self.uri = None
        self.q = queue.Queue()
        self.live = False
        self.tx_mode = None
        self.slow = None
        self.summary = None             # latest live angle/motion result, read by Fusion
        threading.Thread(target=self._worker, daemon=True).start()

    # ------------------------------------------------------------ plumbing
    def log(self, msg, level="info"):
        self.emit({"type": "log", "level": level, "msg": f"[{self.name}] {msg}", "t": time.strftime("%H:%M:%S")})

    def submit(self, cmd):
        if cmd.get("cmd") == "live_stop":
            self.live = False
        self.q.put(cmd)

    def _worker(self):
        while True:
            try:
                cmd = self.q.get(timeout=0.01) if not self.live else self.q.get_nowait()
            except queue.Empty:
                if self.live:
                    try:
                        self._live_frame()
                    except Exception as e:
                        self.live = False
                        self.log(f"live stopped: {e}", "error")
                        self.emit({"type": "live", "on": False})
                continue
            name = cmd.get("cmd")
            try:
                self.emit({"type": "busy", "on": True, "what": name})
                getattr(self, "do_" + name)(cmd)
            except Exception as e:
                traceback.print_exc()
                self.log(f"{name} failed: {e}", "error")
            finally:
                self.emit({"type": "busy", "on": False})

    def _need(self):
        if self.radio is None:
            raise RuntimeError("not connected")

    def _wave(self, tx_mode):
        c = self.cfg
        frame, refs = dsp.make_frame(c["fs"], c["bw_frac"] * c["fs"], int(c["pulse_len"]),
                                     int(c["period"]), tx_mode)
        win = dsp.make_window(c["fs"], c["bw_frac"] * c["fs"], int(c["period"]), c["window"])
        return frame, refs, win

    def _tx(self, tx_mode):
        frame, refs, win = self._wave(tx_mode)
        if self.tx_mode != tx_mode:
            self.radio.start_tx(frame)
            self.tx_mode = tx_mode
        return frame, refs, win

    def _periods(self):
        if self.cfg["periods"]:
            return int(self.cfg["periods"])
        return 4 if self.radio.kind == "serial" else 8

    def _status(self):
        info = self.radio.info() if self.radio else None
        self.emit({"type": "status", "connected": self.radio is not None,
                   "kind": getattr(self.radio, "kind", None), "info": info, "cfg": self.cfg,
                   "name": self.name, "uri": self.uri,
                   "range_bin_m": dsp.C / (2 * self.cfg["fs"]),
                   "range_res_m": dsp.C / (2 * self.cfg["bw_frac"] * self.cfg["fs"])})

    # ------------------------------------------------------------ commands
    def do_connect(self, cmd):
        self.do_disconnect({})
        # one board at a time picks its USB device, so two never take the same one
        with self.hub.connect_lock if self.hub else threading.Lock():
            self._connect(cmd)
        self.log(f"connected: {self.radio.info()['transport']}", "ok")
        self.do_apply({"cfg": {}})

    def _connect(self, cmd):
        transport, uri = cmd.get("transport", "auto"), cmd.get("uri", "")
        port = cmd.get("port") or self.cfg["port"]
        self.cfg["port"] = port
        if transport in ("auto", "iio") and not uri:
            # several boards share one host: take a USB board nobody else holds.
            # A bare "usb:" or the shared 192.168.2.1 would grab an arbitrary one.
            taken = self.hub.taken_uris(self) if self.hub else set()
            free = [u for u in sorted(scan_usb()) if u not in taken]
            if free:
                uri = free[0]
            elif taken:
                if transport == "iio":
                    raise IOError(f"no free USB board (in use: {', '.join(sorted(taken))})")
                transport = "serial"
        elif uri and self.hub and uri in self.hub.taken_uris(self):
            raise IOError(f"{uri} is already open as another board")
        self.radio = open_radio(transport, uri, port, log=self.log, board=self.idx)
        self.uri = getattr(self.radio, "uri", None)

    def do_disconnect(self, cmd):
        self.live = False
        if self.radio:
            self.radio.close()
            self.radio = None
            self.uri = None
            self.tx_mode = None
        self.summary = None
        self._status()

    def do_apply(self, cmd):
        self._need()
        new = dict(cmd.get("cfg", {}))
        # receive geometry: spacing and positions describe the same thing
        if "rx_x_mm" in new:
            new["rx_x_mm"] = [float(v) for v in new["rx_x_mm"]]
            new["rx_spacing_mm"] = abs(new["rx_x_mm"][1] - new["rx_x_mm"][0])
        elif "rx_spacing_mm" in new:
            x1, x2 = self.cfg["rx_x_mm"]
            new["rx_x_mm"] = [x1, x1 + math.copysign(float(new["rx_spacing_mm"]), x2 - x1)]
        gains_only = bool(new) and set(new) <= {"tx_gain", "rx_gain"} and self.tx_mode is not None
        self.cfg.update(new)
        c = self.cfg
        if gains_only:
            self.radio.set_gains(c["tx_gain"], c["rx_gain"])
        elif not new or set(new) - SOFT_KEYS:
            self.tx_mode = None
            self.radio.set_loopback(0)
            self.radio.configure(c["fs"], c["lo"], min(56e6, 0.9 * c["fs"]), c["tx_gain"], c["rx_gain"])
            self.log(f"configured fs={c['fs'] / 1e6:.2f} Msps  LO={c['lo'] / 1e9:.3f} GHz  "
                     f"chirp={c['bw_frac'] * c['fs'] / 1e6:.1f} MHz x {c['pulse_len']} samples", "ok")
        if hasattr(self.radio, "set_geometry"):
            self.radio.set_geometry(c["rx_x_mm"])
        if not new or set(new) - (ANGLE_KEYS | FILTER_KEYS | {"port"}):
            self.slow = None
        elif set(new) & FILTER_KEYS and hasattr(self, "mhist"):
            self.mhist.clear()
        self._status()

    def do_step_digital(self, cmd):
        """Step 1: AD9361 internal digital loopback. Proves the host -> DAC-port
        sample path and the ADC-port -> host path are bit exact."""
        self._need()
        c, P = self.cfg, int(self.cfg["period"])
        self.radio.set_loopback(1)
        res = []
        try:
            for i in (0, 1):
                self.tx_mode = None
                frame, refs, win = self._tx(f"tx{i + 1}")
                rx = self.radio.capture(P * 2)
                h = dsp.compress(dsp.fold(rx, P).mean(1), refs, win)
                d0 = dsp.find_delay(h, (i,))
                got = np.roll(rx[i, :P], -d0)
                # what the 12-bit DAC port should see: the sent int16 with its low 4 bits dropped
                sent = np.round(frame[i] * TX_FS)
                exp = (np.floor(sent.real / 16) + 1j * np.floor(sent.imag / 16)) / RX_FS
                err = np.abs((got - exp) * RX_FS)
                other = float(np.abs(rx[1 - i]).max() * RX_FS)
                L = int(c["pulse_len"])
                view = slice(P // 2 * i, P // 2 * i + L)
                res.append({
                    "tx": i + 1, "max_err_lsb": float(err.max()), "rms_err_lsb": float(np.sqrt(np.mean(err ** 2))),
                    "other_rx_peak_lsb": other, "peak_db": float(dsp.db(np.abs(h[i, i]).max())),
                    "pass": bool(err.max() <= 1.0),
                    "n": np.arange(L), "tx_i": r2(exp.real[view], 4), "rx_i": r2(got.real[view], 4),
                    "tx_q": r2(exp.imag[view], 4), "rx_q": r2(got.imag[view], 4),
                })
                self.log(f"digital loopback TX{i + 1}: max error {err.max():.1f} LSB "
                         f"({'PASS' if err.max() <= 1 else 'FAIL'})", "ok" if err.max() <= 1 else "error")
        finally:
            self.radio.set_loopback(0)
            self.tx_mode = None
        self.emit({"type": "digital", "results": res})

    def do_step_ota(self, cmd):
        """Steps 2+3: each transmitter alone, over the air, seen by both receivers."""
        self._need()
        c, P, L = self.cfg, int(self.cfg["period"]), int(self.cfg["pulse_len"])
        K = max(4, self._periods())
        out = []
        for i in (0, 1):
            frame, refs, win = self._tx(f"tx{i + 1}")
            rx = self.radio.capture(P * K)
            fo = dsp.fold(rx, P)
            mean = fo.mean(1)
            h = dsp.compress(mean, refs, win)
            d0 = dsp.find_delay(h, (i,))
            start = d0 + P // 2 * i
            al = np.roll(mean, -start, axis=-1)                    # pulse now starts at sample 0
            pad = 48
            seg = np.roll(al, pad, axis=-1)[:, :L + 2 * pad]
            f = np.fft.fftshift(np.fft.fftfreq(256, 1 / c["fs"])) / 1e6
            hu = dsp.upsample_profile(np.roll(h[i], -d0, axis=-1), UP)
            rr = np.arange(-N_NEG * UP, N_RANGE * UP)
            paths = [dsp.path_quality(h[i, j], fo[j], refs[i], d0, c["fs"], L) for j in (0, 1)]
            stats = dsp.channel_stats(rx)
            for j in (0, 1):
                paths[j]["suggest_rx_gain"] = float(np.clip(round(c["rx_gain"][j] + (-12 - stats[j]["peak_dbfs"])), 0, 62))
            out.append({
                "tx": i + 1, "stats": stats, "paths": paths,
                "t_us": r2((np.arange(L + 2 * pad) - pad) / c["fs"] * 1e6, 3),
                "env_db": [r2(dsp.db(seg[j])) for j in (0, 1)],
                "tx_env_db": r2(dsp.db(np.roll(np.roll(frame[i], -P // 2 * i), pad)[:L + 2 * pad], 1e-9)),
                "f_mhz": r2(f, 3),
                "spec_db": [r2(dsp.spectrum_db(al[j, :L], 256)) for j in (0, 1)],
                "tx_spec_db": r2(dsp.spectrum_db(np.roll(frame[i], -P // 2 * i)[:L], 256)),
                "bins": r2(rr / UP, 3),
                "profile_db": [r2(dsp.db(hu[j][rr])) for j in (0, 1)],
            })
            for j in (0, 1):
                q = paths[j]
                self.log(f"OTA TX{i + 1}->RX{j + 1}: gain {q['peak_db']:.1f} dB, SNR {q['comp_snr_db']:.0f} dB, "
                         f"PSLR {q['pslr_db']:.0f} dB, phase jitter {q['phase_jitter_deg']:.2f} deg")
        self.emit({"type": "ota", "results": out, "rx_gain": c["rx_gain"], "tx_gain": c["tx_gain"],
                   "bin_m": dsp.C / (2 * c["fs"])})

    def do_live_start(self, cmd):
        self._need()
        self.slow = None
        self.live = True
        self.emit({"type": "live", "on": True})

    def do_live_stop(self, cmd):
        self.live = False
        self.summary = None
        if self.radio:                      # do not leave the transmitter radiating
            self.radio.stop_tx()
            self.tx_mode = None
        self.emit({"type": "live", "on": False})

    def do_reset_bg(self, cmd):
        self.slow = None

    # ---------------------------------------------------------------- live
    def _live_frame(self):
        """One capture. The slow-time state is updated on every capture; the
        (much more expensive) GUI message is built at most ~15 times a second."""
        c, P, L = self.cfg, int(self.cfg["period"]), int(self.cfg["pulse_len"])
        i, jsel = int(c["tx"]), int(c["rx"])
        frame, refs, win = self._tx(f"tx{i + 1}")
        K = self._periods()
        rx = self.radio.capture(P * K)
        t = time.time()
        fo = dsp.fold(rx, P)
        h = dsp.compress(fo.mean(1), refs, win)[i]                 # (n_rx, P)
        d0 = dsp.find_delay(h[None], (0,))
        idx = (d0 + np.arange(-N_NEG, N_RANGE)) % P
        prof = h[:, idx]
        n_cpi = int(c["cpi"])
        if self.slow is None or self.slow[0].n_frames != n_cpi:
            self.slow = [dsp.SlowTime(N_NEG + N_RANGE, n_cpi) for _ in (0, 1)]
            self.t0 = t
            self.track_bin = N_NEG
            self.dhist = collections.deque(maxlen=3000)
            self.last_emit = 0.0
            self.quiet = float('inf')
            self.mhist = collections.deque(maxlen=300)
            self.moving = False
            self.xs_avg = None
            self.fhist = collections.deque(maxlen=600)
        st = self.slow[jsel]
        fps = st.frame_rate()
        for s in self.slow:                 # background adapts over ~3 s whatever the frame rate
            s.bg_alpha = min(0.2, 1.0 / (3.0 * fps)) if fps > 0 else 0.05
            s.push(prof[self.slow.index(s)], t)
        lam = dsp.C / c["lo"]
        near = slice(N_NEG - 1, N_NEG + 4)
        # Motion energy: frame-to-frame change in the near bins (slow drift cancels),
        # short-term mean against a rolling quiet-level estimate. The quiet level is
        # set by leakage jitter, not thermal noise, so it has to be learned in place.
        if st.count >= 2:
            self.dhist.append(float(np.max(np.abs(st.buf[-1, near] - st.buf[-2, near]) ** 2)) + 1e-18)
        # integrate phase only while motion is detected; otherwise residual noise random-walks
        disp = 1e3 * st.track_phase(self.track_bin, lam, 2.0 * self.quiet if self.moving else float('inf'))
        if self.radio.kind != "serial" and t - self.last_emit < 1 / 15:
            return
        self.last_emit = t

        hu = dsp.upsample_profile(np.roll(h, -d0, axis=-1), UP)
        rr = np.arange(-N_NEG * UP, N_RANGE * UP)
        msg = {
            "type": "frame", "t": t - self.t0, "fps": fps, "n": st.count,
            "tx": i + 1, "rx": jsel + 1, "bin_m": dsp.C / (2 * c["fs"]),
            "stats": dsp.channel_stats(rx),
            "quality": [dsp.path_quality(h[j], fo[j], refs[i], d0, c["fs"], L) for j in (0, 1)],
            "bins": r2(rr / UP, 3),
            "profile_db": [r2(dsp.db(hu[j][rr])) for j in (0, 1)],
            "mti_db": r2(dsp.db(st.mti())), "mti_bins": np.arange(-N_NEG, N_RANGE),
            "disp_mm": disp, "track_bin": self.track_bin - N_NEG,
        }
        d = np.array(self.dhist)
        if d.size >= 30:
            self.quiet = float(np.sqrt(np.percentile(d, 20)))
        msg["motion_db"] = 0.0
        n = st.filled
        if n >= 8:                          # follow the bin that is actually moving, with hysteresis
            var = np.var(st.buf[-n:], axis=0)
            tb = int(np.argmax(var[near])) + N_NEG - 1
            if tb != self.track_bin and var[tb] > 4 * var[self.track_bin]:
                self.track_bin = tb
                st.unwrapped, st.prev = 0.0, None

        nd = n_cpi
        fopt = dict(dc=bool(c["f_dc"]), hp_hz=float(c["f_hp_hz"]), sg=c["f_sg"], sg_win=int(c["f_sg_win"]),
                    sg_order=int(c["f_sg_order"]), band_lo=float(c["f_band_lo"]), band_hi=float(c["f_band_hi"]),
                    window=c["f_window"])
        Xs = [s.doppler(nd, **fopt) for s in self.slow]
        X, prf = Xs[jsel]
        if X is not None:
            f = np.fft.fftshift(np.fft.fftfreq(nd, 1 / prf))
            v = f * lam / 2
            # The plots show the spectrum exactly as the selected filters leave it.
            # Detection, motion energy and angle additionally ignore zero Doppler and
            # two bins either side (Hann mainlobe of whatever static residue is left).
            passband = np.abs(f) >= 2.5 * prf / nd
            rd = np.abs(X) ** 2
            floor = float(np.median(rd[:, N_NEG + 10:]) + 1e-18)
            msg["rd_db"] = r2(10 * np.log10(rd[:, N_NEG - 2:N_NEG + 10] + 1e-18), 1)
            msg["rd_bins"] = np.arange(-2, 10)
            msg["vel"] = r2(v, 4)
            msg["prf"] = prf
            msg["dopp_bin_hz"] = prf / nd
            msg["dopp_db"] = r2(10 * np.log10(rd[:, near].sum(axis=1) + 1e-18), 1)
            m = rd[:, near] * passband[:, None]
            k = np.unravel_index(int(np.argmax(m)), m.shape)
            # floor along Doppler in the same delay bin: leakage jitter is white in
            # slow time, so thermal-only far bins would understate it
            col = rd[passband][:, near][:, k[1]]
            msg["det"] = {"vel": float(v[k[0]]), "bin": int(k[1]) - 1,
                          "snr_db": float(10 * np.log10(m[k] / (np.median(col) + 1e-18) + 1e-12))}
            msg["rd_floor_db"] = float(10 * np.log10(floor))
            # motion energy: Doppler-filtered power in the near bins over its learned quiet level
            pm = float(m.sum()) + 1e-18
            self.mhist.append(pm)
            if len(self.mhist) >= 20:
                msg["motion_db"] = float(10 * np.log10(pm / np.percentile(self.mhist, 20)))
                self.moving = msg["motion_db"] > 6

            # ---- angle: phase difference between the two receivers, moving part only
            X1, X2 = Xs[0][0][:, near], Xs[1][0][:, near]
            cur = np.stack([np.sum(X1 * np.conj(X2), axis=1),              # cross-spectrum per Doppler bin
                            np.sum(np.abs(X1) ** 2, axis=1), np.sum(np.abs(X2) ** 2, axis=1)])
            # average a few looks (~0.3 s) so coherence is a real multi-look estimate
            if getattr(self, "xs_avg", None) is None or self.xs_avg.shape != cur.shape:
                self.xs_avg = cur
            else:
                self.xs_avg = 0.7 * self.xs_avg + 0.3 * cur
            s12, p1, p2 = self.xs_avg[0], self.xs_avg[1].real, self.xs_avg[2].real
            g = np.sqrt(p1 * p2)
            # only Doppler bins that actually contain a mover; empty bins are noise
            # with random phase and would drag coherence down
            strong = passband & (g > 4 * np.median(g[passband]))
            use = strong if strong.sum() >= 2 else passband
            S = np.sum(s12[use])
            self.raw_dphi = float(np.angle(S))
            cal = np.radians(float(c["angle_cal_deg"]))
            d_m = float(c["rx_spacing_mm"]) * 1e-3
            wrap = lambda a: np.angle(np.exp(1j * a))
            to_deg = lambda dphi: np.degrees(np.arcsin(np.clip(dphi * lam / (2 * np.pi * d_m), -1, 1)))
            msg["angle"] = {
                "deg": float(to_deg(wrap(self.raw_dphi - cal))),
                "coh": float(np.abs(S) / (np.sqrt(p1[use].sum() * p2[use].sum()) + 1e-30)),
                "dphi_deg": float(np.degrees(wrap(self.raw_dphi - cal))),
                "raw_dphi_deg": float(np.degrees(self.raw_dphi)),
                "amb_deg": float(np.degrees(np.arcsin(min(1.0, lam / (2 * d_m))))),
                "n_bins": int(strong.sum()),
            }
            msg["ang_dopp"] = r2(np.where(strong, to_deg(wrap(np.angle(s12) - cal)), np.nan), 1)
            msg["ph_dopp"] = r2(np.where(strong, np.degrees(wrap(np.angle(s12) - cal)), np.nan), 1)
        self._summarise(msg, t, lam, near)
        self.emit(msg)

    def _summarise(self, msg, t, lam, near):
        """Angle and motion for the Fusion stage.

        Besides the Doppler-domain angle above (which averages over the whole
        Doppler window, seconds long) a "fast" angle is computed for tracking a
        hand wave: the frame difference d = p(t) - p(t - ~30 ms) of each
        receiver's near bins. Static scene and leakage cancel in the difference;
        a moving scatterer leaves d_j ~ a.exp(j.psi_j).(1 - exp(-j.dpsi)) with
        the same radial phase step dpsi on both receivers, so arg(d1.conj(d2))
        is the inter-receiver phase psi1 - psi2. Summed over ~0.15 s of frames."""
        c = self.cfg
        s1, s2 = self.slow
        fps = s1.frame_rate()
        k = max(1, int(round(0.03 * fps)))
        m = max(3, int(round(0.15 * fps)))
        fast = None
        if s1.filled >= k + m + 1:
            d1 = s1.buf[-m:, near] - s1.buf[-m - k:-k, near]
            d2 = s2.buf[-m:, near] - s2.buf[-m - k:-k, near]
            S = np.sum(d1 * np.conj(d2))
            p1, p2 = float(np.sum(np.abs(d1) ** 2)), float(np.sum(np.abs(d2) ** 2))
            self.fhist.append(p1 + p2 + 1e-30)
            # leakage jitter also survives the difference, coherently on both
            # receivers (shared LO), so gate on power over its learned quiet level
            lvl = (float(10 * np.log10((p1 + p2 + 1e-30) / np.percentile(self.fhist, 5)))
                   if len(self.fhist) >= 30 else 0.0)
            self.raw_dphi_fast = float(np.angle(S))
            coh = float(np.abs(S) / (np.sqrt(p1 * p2) + 1e-30))
            fast = {"raw": self.raw_dphi_fast, "coh": coh, "lvl_db": lvl, "valid": bool(coh > 0.5 and lvl > 6)}
        A = msg.get("angle")
        dop = None
        if A:
            dop = {"raw": self.raw_dphi, "coh": A["coh"],
                   "valid": bool(A["coh"] > 0.5 and A["n_bins"] >= 2 and msg["motion_db"] > 6)}
        self.summary = {"t": t, "fps": fps, "motion_db": msg["motion_db"], "lam": lam, "lo": c["lo"],
                        "rx_x_mm": list(c["rx_x_mm"]), "cal": math.radians(float(c["angle_cal_deg"])),
                        "fast": fast, "doppler": dop, "vel": msg["det"]["vel"] if msg.get("det") else None}

    def do_angle_cal(self, cmd):
        """Take the direction of whatever is moving right now as 0 degrees, or,
        with `expected_deg`, as the inter-receiver phase that a mover at a known
        position should give (two-board calibration)."""
        if cmd.get("clear"):
            self.cfg["angle_cal_deg"] = 0.0
        else:
            raw = getattr(self, "raw_dphi_fast" if cmd.get("estimator") == "fast" else "raw_dphi", 0.0)
            exp = math.radians(float(cmd.get("expected_deg", 0.0)))
            self.cfg["angle_cal_deg"] = float(np.degrees(np.angle(np.exp(1j * (raw - exp)))))
        self.log(f"angle reference: inter-receiver phase offset {self.cfg['angle_cal_deg']:.1f} deg", "ok")
        self._status()


# =================================================================== fusion
class Strokes:
    """Splits an angle trace into swipes: a stroke ends where the angle turns
    back by more than `hyst` degrees, or where the trace drops out."""

    def __init__(self, hyst=8.0):
        self.hyst = hyst
        self.done = collections.deque(maxlen=50)
        self.reset()

    def reset(self):
        self.dir, self.lo, self.hi, self.anchor, self.ext, self.last_t = 0, None, None, None, None, 0.0

    def _close(self, t):
        if self.dir and abs(self.ext - self.anchor) > self.hyst:
            self.done.append({"t": t, "dir": self.dir, "from": self.anchor, "to": self.ext})

    def push(self, t, a):
        if a is None:
            if self.last_t and t - self.last_t > 0.4:
                self._close(self.last_t)
                self.reset()
            return
        self.last_t = t
        if self.dir == 0:
            self.lo = a if self.lo is None else min(self.lo, a)
            self.hi = a if self.hi is None else max(self.hi, a)
            if self.hi - self.lo > self.hyst:
                self.dir = 1 if a == self.hi else -1
                self.anchor, self.ext = (self.lo, a) if self.dir > 0 else (self.hi, a)
        elif (a - self.ext) * self.dir > 0:
            self.ext = a
        elif (self.ext - a) * self.dir > self.hyst:
            self._close(t)
            self.anchor, self.ext, self.dir = self.ext, a, -self.dir

    def report(self, now):
        recent = [s for s in self.done if now - s["t"] < 3.0]
        last = self.done[-1] if self.done else None
        return {"n_recent": len(recent), "waving": len(recent) >= 2, "rate_hz": len(recent) / 6.0,
                "last": None if last is None else {**last, "ago": now - last["t"], "span": abs(last["to"] - last["from"])}}


def model_dphi(xg, zg, rx_x_mm, lam):
    """Inter-receiver phase psi1 - psi2 that a point scatterer at (x, z) gives:
    the common TX leg cancels, each receive leg adds -2.pi.r/lambda."""
    r1 = np.hypot(xg - rx_x_mm[0] * 1e-3, zg)
    r2 = np.hypot(xg - rx_x_mm[1] * 1e-3, zg)
    return -2 * np.pi * (r1 - r2) / lam


class Fusion:
    """Combines the boards' live angle estimates.

    The boards have independent clocks and LOs, so their signals cannot be
    compared in phase with each other; each board's own two receivers can.
    Every board therefore gives its own inter-receiver phase, which puts the
    hand on a hyperbola-like curve around that board's receive pair. Pairs at
    different places along the array see a nearby hand under different
    angles, so their curves cross where the hand is: a coarse position, a few
    cm laterally and much worse in range."""

    XS = np.linspace(-0.40, 0.40, 81)
    ZS = np.linspace(0.05, 1.00, 60)

    def __init__(self, hub, emit):
        self.hub, self.emit = hub, emit
        self.on = False
        self.estimator = "fast"
        self.t0 = time.time()
        self.strokes = Strokes()
        self.L = None
        self.xg, self.zg = np.meshgrid(self.XS, self.ZS)
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            time.sleep(1 / 15)
            if self.on:
                try:
                    self._step()
                except Exception:
                    traceback.print_exc()

    def _step(self):
        now = time.time()
        wrap = lambda a: float(np.angle(np.exp(1j * a)))
        boards, use = [], []
        for e in self.hub.engines:
            s = e.summary
            if not (e.live and s and now - s["t"] < 0.6):
                boards.append(None)
                continue
            x1, x2 = s["rx_x_mm"]
            d = abs(x2 - x1) * 1e-3
            est = s[self.estimator] or {}
            b = {"motion_db": s["motion_db"], "fps": s["fps"], "lo": s["lo"], "vel": s["vel"],
                 "move_db": s["fast"]["lvl_db"] if s["fast"] else None,
                 "valid": bool(est.get("valid")), "coh": est.get("coh"), "deg": None, "phi": None}
            if "raw" in est:
                dphi = wrap(est["raw"] - s["cal"])
                th = math.degrees(math.asin(max(-1.0, min(1.0, dphi * s["lam"] / (2 * math.pi * d)))))
                # a board's angle is + towards its RX1; the array angle is + towards +x
                b.update(dphi_deg=math.degrees(dphi), deg=th, phi=th if x1 > x2 else -th,
                         amb_deg=math.degrees(math.asin(min(1.0, s["lam"] / (2 * d)))))
                if b["valid"]:
                    use.append((dphi, est["coh"], s))
            boards.append(b)
        live = [b for b in boards if b]
        val = [b for b in live if b["valid"]]
        msg = {"type": "fused", "t": now - self.t0, "boards": boards, "estimator": self.estimator,
               "motion_db": max((b["motion_db"] for b in live), default=None),
               "move_db": max((b["move_db"] for b in live if b["move_db"] is not None), default=None),
               "angle": sum(b["phi"] * b["coh"] for b in val) / sum(b["coh"] for b in val) if val else None,
               "x": r2(self.XS * 1e3, 0), "z": r2(self.ZS * 1e3, 0), "pos": None}

        # position: every valid board scores each (x, z) by how well the phase it
        # would produce there matches the measured one, weighted by coherence
        if use:
            cost = np.zeros_like(self.xg)
            for dphi, coh, s in use:
                cost += coh * (1 - np.cos(dphi - model_dphi(self.xg, self.zg, s["rx_x_mm"], s["lam"])))
            L = np.exp(-cost / 0.1)                 # 0.1 ~ 1 - cos(25 deg) of phase noise
            self.L = L if self.L is None else 0.6 * self.L + 0.4 * L
        elif self.L is not None:
            self.L = None if self.L.max() < 1e-3 else self.L * 0.8
        if self.L is not None:
            Ln = self.L / (self.L.max() + 1e-30)
            msg["map_db"] = r2(10 * np.log10(Ln + 1e-3), 1)
            if len(use) >= 2:
                iz, ix = np.unravel_index(int(np.argmax(Ln)), Ln.shape)
                inside = Ln > 0.5                   # -3 dB region: a rough uncertainty
                msg["pos"] = {"x_mm": float(self.XS[ix] * 1e3), "z_mm": float(self.ZS[iz] * 1e3),
                              "dx_mm": float(np.ptp(self.xg[inside]) * 500),
                              "dz_mm": float(np.ptp(self.zg[inside]) * 500),
                              "deg": float(np.degrees(np.arctan2(self.XS[ix], self.ZS[iz])))}
        self.strokes.push(now, msg["angle"])
        msg["strokes"] = self.strokes.report(now)

        warn = []
        if len(live) < len(self.hub.engines):
            warn.append("not every board is running live")
        lo = [(e.cfg["lo"], e.cfg["bw_frac"] * e.cfg["fs"]) for e in self.hub.engines if e.radio]
        if len(lo) >= 2 and abs(lo[0][0] - lo[1][0]) < max(lo[0][1], lo[1][1]):
            warn.append("the two boards' chirps overlap in frequency, so each hears the other's pulses: "
                        "set their LOs at least one chirp bandwidth apart")
        msg["warn"] = warn
        self.emit(msg)


# ====================================================================== hub
class Hub:
    """Owns the per-board engines and the fusion stage; routes browser commands.
    A command with a "board" index goes to that board's engine."""

    def __init__(self, emit, n_boards=2):
        self.emit = emit
        self.connect_lock = threading.Lock()
        self.engines = [Engine(emit, k, self) for k in range(n_boards)]
        self.fusion = Fusion(self, emit)

    def taken_uris(self, me):
        return {e.uri for e in self.engines if e is not me and e.uri}

    def _fusion_state(self):
        self.emit({"type": "fusion", "on": self.fusion.on, "estimator": self.fusion.estimator})

    def submit(self, cmd):
        if "board" in cmd:
            k = int(cmd["board"])
            if 0 <= k < len(self.engines):
                self.engines[k].submit(cmd)
            return
        name = cmd.get("cmd")
        if name == "hello":
            for e in self.engines:
                e.submit({"cmd": "hello"})
            self._fusion_state()
        elif name == "scan":
            threading.Thread(target=lambda: self.emit({"type": "scan", "uris": scan_usb(),
                                                       "taken": {e.name: e.uri for e in self.engines}}),
                             daemon=True).start()
        elif name == "fused_start":
            self.fusion.strokes = Strokes()
            self.fusion.L = None
            self.fusion.on = True
            for e in self.engines:
                if e.radio:
                    e.submit({"cmd": "live_start"})
            self._fusion_state()
        elif name in ("fused_stop", "stop_all"):
            self.fusion.on = False
            for e in self.engines:
                e.submit({"cmd": "live_stop"})
            self._fusion_state()
        elif name == "fused_cfg":
            if cmd.get("estimator") in ("fast", "doppler"):
                self.fusion.estimator = cmd["estimator"]
            self._fusion_state()
        elif name == "fused_cal":
            # something moving at a known (x, z): set each board's reference so
            # that it reports the phase that position should give
            x, z = float(cmd.get("x_mm", 0)) * 1e-3, float(cmd.get("z_mm", 300)) * 1e-3
            for e in self.engines:
                if cmd.get("clear"):
                    e.submit({"cmd": "angle_cal", "clear": True})
                    continue
                exp = math.degrees(float(model_dphi(x, z, e.cfg["rx_x_mm"], dsp.C / e.cfg["lo"])))
                e.submit({"cmd": "angle_cal", "expected_deg": exp, "estimator": self.fusion.estimator})


# ====================================================================== web
class WS(tornado.websocket.WebSocketHandler):
    clients = set()

    def check_origin(self, origin):
        return True

    def open(self):
        WS.clients.add(self)
        HUB.submit({"cmd": "hello"})

    def on_close(self):
        WS.clients.discard(self)
        if not WS.clients:
            HUB.submit({"cmd": "stop_all"})

    def on_message(self, message):
        HUB.submit(json.loads(message))


def broadcast(msg):
    s = json.dumps(clean(msg))
    for c in list(WS.clients):
        try:
            c.write_message(s)
        except Exception:
            WS.clients.discard(c)


class Index(tornado.web.RequestHandler):
    def get(self):
        self.set_header("Cache-Control", "no-store")
        with open(os.path.join(HERE, "index.html"), encoding="utf-8") as f:
            self.write(f.read())


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--transport", default="")
    ap.add_argument("--boards", type=int, default=2)
    a = ap.parse_args()
    loop = tornado.ioloop.IOLoop.current()
    HUB = Hub(lambda m: loop.add_callback(broadcast, m), a.boards)
    for e in HUB.engines:
        e.do_hello = lambda cmd, e=e: e._status()
    app = tornado.web.Application([(r"/", Index), (r"/ws", WS)], template_path=HERE, autoreload=False)
    app.listen(a.port, address="127.0.0.1")
    print(f"pulse radar GUI: http://localhost:{a.port}")
    if a.transport:
        for e in HUB.engines:
            e.submit({"cmd": "connect", "transport": a.transport})
    try:
        loop.start()
    finally:
        for e in HUB.engines:
            if e.radio:
                e.radio.close()
