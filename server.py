"""Pulse-radar bring-up GUI server.

    py -3.11 server.py [--port 8765] [--transport auto|iio|serial|sim]

Open http://localhost:8765 . All radio access happens on one worker thread;
results are pushed to the browser over a websocket.
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
from radios import RX_FS, TX_FS, open_radio

HERE = os.path.dirname(os.path.abspath(__file__))
N_NEG, N_RANGE = 4, 24          # range bins kept: -4 .. +19 around the direct path
UP = 4                          # display interpolation of range profiles

DEFAULTS = {
    "fs": 61.44e6, "lo": 5.8e9, "bw_frac": 0.8, "pulse_len": 256, "period": 1024,
    "tx_gain": [-10.0, -10.0], "rx_gain": [30.0, 30.0],
    "tx": 0, "rx": 1, "window": "hamming", "periods": 0, "cpi": 128,
    "rx_spacing_mm": 50.8, "angle_cal_deg": 0.0,
    # slow-time / Doppler filters (all optional; defaults give the plain display)
    "f_dc": True, "f_hp_hz": 0.0, "f_sg": "off", "f_sg_win": 15, "f_sg_order": 3,
    "f_band_lo": 0.0, "f_band_hi": 0.0, "f_window": "hann",
}

# settings that change processing only, not the radio or the waveform
FILTER_KEYS = {"f_dc", "f_hp_hz", "f_sg", "f_sg_win", "f_sg_order", "f_band_lo", "f_band_hi", "f_window"}
SOFT_KEYS = {"tx", "rx", "window", "periods", "cpi", "rx_spacing_mm", "angle_cal_deg"} | FILTER_KEYS


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
    def __init__(self, emit):
        self.emit = emit
        self.cfg = dict(DEFAULTS)
        self.radio = None
        self.q = queue.Queue()
        self.live = False
        self.tx_mode = None
        self.slow = None
        threading.Thread(target=self._worker, daemon=True).start()

    # ------------------------------------------------------------ plumbing
    def log(self, msg, level="info"):
        self.emit({"type": "log", "level": level, "msg": str(msg), "t": time.strftime("%H:%M:%S")})

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
                   "range_bin_m": dsp.C / (2 * self.cfg["fs"]),
                   "range_res_m": dsp.C / (2 * self.cfg["bw_frac"] * self.cfg["fs"])})

    # ------------------------------------------------------------ commands
    def do_connect(self, cmd):
        self.do_disconnect({})
        self.radio = open_radio(cmd.get("transport", "auto"), cmd.get("uri", ""),
                                cmd.get("port", "COM5"), log=self.log)
        self.log(f"connected: {self.radio.info()['transport']}", "ok")
        self.do_apply({"cfg": {}})

    def do_disconnect(self, cmd):
        self.live = False
        if self.radio:
            self.radio.close()
            self.radio = None
            self.tx_mode = None
        self._status()

    def do_apply(self, cmd):
        self._need()
        new = cmd.get("cfg", {})
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
        if not new or set(new) - ({"rx_spacing_mm", "angle_cal_deg"} | FILTER_KEYS):
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
        self.emit(msg)

    def do_angle_cal(self, cmd):
        """Take the direction of whatever is moving right now as 0 degrees."""
        if cmd.get("clear"):
            self.cfg["angle_cal_deg"] = 0.0
        else:
            self.cfg["angle_cal_deg"] = float(np.degrees(getattr(self, "raw_dphi", 0.0)))
        self.log(f"angle reference: inter-receiver phase offset {self.cfg['angle_cal_deg']:.1f} deg", "ok")
        self._status()


# ====================================================================== web
class WS(tornado.websocket.WebSocketHandler):
    clients = set()

    def check_origin(self, origin):
        return True

    def open(self):
        WS.clients.add(self)
        ENGINE.q.put({"cmd": "hello"})

    def on_close(self):
        WS.clients.discard(self)
        if not WS.clients:
            ENGINE.submit({"cmd": "live_stop"})

    def on_message(self, message):
        ENGINE.submit(json.loads(message))


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
    a = ap.parse_args()
    loop = tornado.ioloop.IOLoop.current()
    ENGINE = Engine(lambda m: loop.add_callback(broadcast, m))
    ENGINE.do_hello = lambda cmd: ENGINE._status()
    app = tornado.web.Application([(r"/", Index), (r"/ws", WS)], template_path=HERE, autoreload=False)
    app.listen(a.port, address="127.0.0.1")
    print(f"pulse radar GUI: http://localhost:{a.port}")
    if a.transport:
        ENGINE.submit({"cmd": "connect", "transport": a.transport})
    try:
        loop.start()
    finally:
        if ENGINE.radio:
            ENGINE.radio.close()
