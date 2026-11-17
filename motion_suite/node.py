"""One radar node = one 2T2R board (or the simulator). Two threads:

  radio thread     owns the board: transmit frame, capture, matched filter, ring buffer,
                   waveform switching and the A/B comparison schedule
  analysis thread  10 Hz: clean -> range-Doppler -> detect -> beamform -> micro-Doppler,
                   emits one message per tick for the page

Several nodes can run side by side (one per board); the server fuses their targets.
"""
from __future__ import annotations

import json
import os
import queue
import subprocess
import threading
import time
import traceback
from collections import deque

import numpy as np

import dsp as D
import radio as R
import waveforms as W

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")

TICK = 0.1
WIN_RD = 0.5            # s of slow time per range-Doppler map
WIN_SP = 0.25           # s per spectrogram column
WIN_TB = 1.0            # s per TX-beam measurement
NV, V_DISP = 128, 5.0   # velocity grid of every map (m/s, + = moving away)
U = 8                   # range upsampling for display (2.44 m bins -> 0.31 m)
THR_DB = 13.0           # detection threshold above the learned background (cell false-alarm ~e^-20)
LEARN_TICKS = 20
N_TH = 91               # angles in the range-angle map
TH = np.radians(np.linspace(-90, 90, N_TH))
TH_FINE = np.radians(np.linspace(-90, 90, 361))
VG = np.linspace(-V_DISP, V_DISP, NV)


QUIET = False      # set by the server in --sim mode: no speech from simulated runs


def say(text):
    """Spoken cue (Windows speech) so the walker needs no screen."""
    if QUIET:
        return
    try:
        subprocess.Popen(["powershell", "-NoProfile", "-Command",
                          "Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('%s')" % text.replace("'", "")],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def rnd(x, n=2):
    if x is None:
        return None
    x = float(x)
    return None if not np.isfinite(x) else round(x, n)


class Ring:
    def __init__(self, fr, seconds=10.0, rate=650):
        self.fr = fr
        self.cap = int(seconds * rate)
        self.H = np.zeros((self.cap, fr.n_slots, 2, D.NBK), np.complex64)
        self.T = np.zeros(self.cap)
        self.i = 0
        self.born = time.time()
        self.tag = "live"

    def push(self, t, h):
        j = self.i % self.cap
        self.H[j] = h
        self.T[j] = t
        self.i += 1

    def last(self, sec, rate):
        i = self.i
        m = min(i, self.cap - 50, int(sec * rate * 1.4) + 16)
        if m < 16:
            return None, None
        idx = (np.arange(i - m, i)) % self.cap
        T = self.T[idx]
        keep = T >= T[-1] - sec
        return T[keep].copy(), self.H[idx[keep]].copy()


class Background:
    """Per-cell power the empty scene leaves after cleaning (leakage flutter near range zero,
    noise elsewhere). Learned as a median over the first ticks, then updated slowly only where
    nothing is detected; frozen during an A/B run so every waveform is judged the same way."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.stack = {}
        self.val = {}
        self.frozen = False

    def feed(self, name, P, snr_db=None, alpha=0.03):
        """Learn: mean of the lowest 80 % of LEARN_TICKS maps per cell (a median or log average
        sits 1.6-2.5 dB under the mean of exponentially distributed power, which over ~5000
        cells turns into false alarms every tick). Then a linear running mean, skipping detections."""
        if name not in self.val:
            st = self.stack.setdefault(name, [])
            st.append(P)
            if len(st) >= LEARN_TICKS:
                a = np.sort(np.array(st), axis=0)[: int(0.8 * len(st))]
                b = a.mean(0)
                pos = b > 1e-26
                b[~pos] = np.median(b[pos]) if pos.any() else 1e-24   # e.g. beyond the ambiguity limit while learning
                self.val[name] = b
                self.stack.pop(name)
            return
        if self.frozen:
            return
        upd = np.ones(P.shape, bool) if snr_db is None else snr_db < THR_DB
        upd &= P > 1e-26
        b = self.val[name]
        b[upd] = (1 - alpha) * b[upd] + alpha * P[upd]


class Node:
    def __init__(self, cfg, emit, log, sim=False):
        self.cfg = cfg
        self.id = cfg["id"]
        self.uri = "sim-" + self.id if sim else cfg.get("uri", "auto")
        self.emit, self.log = emit, log
        self.lo = float(cfg.get("lo", R.CFG["lo"]))
        self.lam = W.C / self.lo
        self.geom = D.Geometry(cfg.get("antennas"), self.lam)
        self.r_max = float(cfg.get("r_max", 12.0))
        self.reg = W.registry()
        self.wf_key = cfg.get("waveform", "fmcw")
        self.mode = "live"                      # live | txbeam | ab
        self.gates = {"local": list(cfg.get("local", [0.3, 2.5])), "back": list(cfg.get("back", [4.0, 9.0]))}
        self.flip = bool(cfg.get("flip", False))
        self.cmds = queue.Queue()
        self.stop_ev = threading.Event()
        self.ring = None
        self.radio = None
        self.state = "stopped"
        self.kind = "?"
        self.err = None
        self.rate = 0.0
        self.adc_peak = -99.0
        self.bgs = {}
        self.delta = {}
        self.hist = {"local": deque(maxlen=80), "back": deque(maxlen=80)}
        self.track = None
        self.cal, self.cal_q = np.ones(4, complex), None
        self.cal_job = None
        self.ab = None
        self.ab_acc = {}
        self.tb_bg = None
        self.threads = []

    # ------------------------------------------------------------ control
    def start(self):
        self.stop_ev.clear()
        self.threads = [threading.Thread(target=self._radio_main, daemon=True),
                        threading.Thread(target=self._analysis_main, daemon=True)]
        for t in self.threads:
            t.start()

    def stop(self):
        self.stop_ev.set()

    def running(self):
        return any(t.is_alive() for t in self.threads)

    def command(self, cmd, args):
        """Called from the web server thread."""
        if cmd in ("wf", "mode", "ab_start", "ab_stop"):
            self.cmds.put((cmd, args))
        elif cmd == "relearn":
            for b in self.bgs.values():
                b.reset()
            self.tb_bg = None
            self.log(f"[{self.id}] background re-learning: keep the area still for 2 s")
        elif cmd == "gates":
            self.gates = {"local": [float(args["local"][0]), float(args["local"][1])],
                          "back": [float(args["back"][0]), float(args["back"][1])]}
            for b in self.bgs.values():
                for k in ("local", "back"):
                    b.val.pop(k, None)
                    b.stack.pop(k, None)
        elif cmd == "flip":
            self.flip = bool(args.get("on"))
        elif cmd == "calibrate":
            self.cal_job = {"until": time.time() + float(args.get("seconds", 10)), "R": np.zeros((4, 4), complex), "rsum": 0.0,
                            "n": 0, "theta": float(args.get("theta", 0.0))}
            say("Calibration. Walk slowly toward and away from the radar along its centre line, between two and five meters. Keep your arms still.")
            self.log(f"[{self.id}] calibrating for {args.get('seconds', 10)} s at {args.get('theta', 0)} deg")
        elif cmd == "cal_reset":
            self.cal, self.cal_q = np.ones(4, complex), None
            p = self._cal_path()
            if os.path.exists(p):
                os.remove(p)

    def info(self):
        return {"id": self.id, "name": self.cfg.get("name", self.id), "uri": self.uri, "kind": self.kind, "state": self.state,
                "err": self.err, "rate": rnd(self.rate, 1), "adc_peak": rnd(self.adc_peak, 1), "wf": self.wf_key, "mode": self.mode,
                "lo": self.lo, "gates": self.gates, "flip": self.flip, "pose": self.cfg.get("pose", {}), "color": self.cfg.get("color"),
                "cal": None if self.cal_q is None else {"deg": np.round(np.degrees(np.angle(self.cal)), 1).tolist(), "q": rnd(self.cal_q, 3)},
                "waveforms": {k: f.describe(self.lo) for k, f in self.reg.items()}, "r_max": self.r_max,
                "unamb_deg": rnd(self.geom.unamb_deg, 1), "geom": self.geom.info(), "ab": self._ab_public()}

    # ------------------------------------------------------------ radio thread
    def _cal_path(self):
        return os.path.join(DATA, f"calib_{self.id}_{self.kind}.json")

    def _load_cal(self):
        p = self._cal_path()
        if os.path.exists(p):
            with open(p) as f:
                c = json.load(f)
            self.cal = np.array(c["re"]) + 1j * np.array(c["im"])
            self.cal_q = c["quality"]
            self.log(f"[{self.id}] calibration loaded ({c['when']}, quality {c['quality']:.2f})")

    def _use(self, fr, tag="live"):
        r = self.radio
        r.tx_cyclic(fr.frame)
        self.comp = D.compressor(fr)
        self.n_cap = fr.P * max(1, 4096 // fr.P)
        for _ in range(4):
            r.capture(self.n_cap)
        ring = Ring(fr, rate=200 if fr.kind == "txbeam" else 650)
        ring.tag = tag
        self.ring = ring
        if fr.kind != "txbeam":
            self.bgs.setdefault(fr.key, Background())

    def _radio_main(self):
        self.state, self.err = "starting", None
        try:
            if self.uri == "auto":
                found = R.scan_usb()
                if not found:
                    raise RuntimeError("no USB board found (iio scan empty)")
                self.uri = found[0]
            self.radio = R.open_radio(self.uri, self.geom)
            self.kind = self.radio.kind
            self._load_cal()
            cfg = dict(R.CFG, lo=self.lo)
            if "rx_gain" in self.cfg:
                cfg["rx_gain"] = tuple(self.cfg["rx_gain"])
            if "tx_gain" in self.cfg:
                cfg["tx_gain"] = tuple(self.cfg["tx_gain"])
            self.radio.configure(**cfg)
            fr = self.reg[self.wf_key]
            self.log(f"[{self.id}] {self.kind} {self.uri}: converging RX loops on a tone, then freezing them")
            R.ready(self.radio, fr.frame)
            self._use(fr)
            self.state = "running"
            self.t0 = time.time()
            n_win, t_win, pk = 0, time.time(), 0.0
            while not self.stop_ev.is_set():
                self._handle_cmds()
                self._ab_step()
                x = self.radio.capture(self.n_cap)
                h = self.comp.process(x)
                self.ring.push(time.time() - self.t0, h)
                n_win += 1
                pk = max(pk, float(np.abs(x).max()))
                if time.time() - t_win >= 1.0:
                    self.rate = n_win / (time.time() - t_win)
                    self.adc_peak = 20 * np.log10(max(pk, 1e-9))
                    n_win, t_win, pk = 0, time.time(), 0.0
                    self.emit({"type": "status", "node": self.info()})
        except Exception as e:
            self.err = f"{type(e).__name__}: {e}"
            self.state = "error"
            self.log(f"[{self.id}] ERROR {self.err}\n{traceback.format_exc(limit=3)}")
        finally:
            if self.radio is not None:
                self.radio.close()
                self.radio = None          # drop the libiio context: on Windows USB access is exclusive
                import gc
                gc.collect()
            if self.state != "error":
                self.state = "stopped"
            self.stop_ev.set()
            self.emit({"type": "status", "node": self.info()})

    def _handle_cmds(self):
        while True:
            try:
                cmd, args = self.cmds.get_nowait()
            except queue.Empty:
                return
            if cmd == "wf" and args.get("key") in W.MIMO_KEYS and self.mode == "live":
                self.wf_key = args["key"]
                self._use(self.reg[self.wf_key])
                self.log(f"[{self.id}] waveform -> {self.reg[self.wf_key].label}")
            elif cmd == "mode":
                m = args.get("mode")
                if m == "txbeam" and self.mode != "txbeam":
                    self.mode = "txbeam"
                    self.tb_bg = None
                    self._use(self.reg["txbeam"])
                    self.log(f"[{self.id}] TX beam scan: both transmitters radiate together, steered over 12 phases")
                elif m == "live" and self.mode != "live":
                    self.mode = "live"
                    self.ab = None
                    self._use(self.reg[self.wf_key])
            elif cmd == "ab_start":
                self._ab_begin(args)
            elif cmd == "ab_stop" and self.ab:
                self.ab = None
                self.mode = "live"
                self._use(self.reg[self.wf_key])
                say("Comparison stopped.")

    # ------------------------------------------------------------ A/B schedule
    def _ab_begin(self, args):
        keys = [k for k in args.get("keys", W.MIMO_KEYS) if k in W.MIMO_KEYS]
        learn, dwell, rounds = float(args.get("learn", 4)), float(args.get("dwell", 6)), int(args.get("rounds", 3))
        steps = []
        for i, k in enumerate(keys):
            steps.append({"key": k, "phase": "learn", "dur": learn,
                          "say": "Waveform comparison. Please leave the radar's view and stay still." if i == 0 else None})
        steps.append({"key": None, "phase": "prep", "dur": 4.0, "say": "Now walk around in front of the radar, near and far, and keep moving."})
        for _ in range(rounds):
            for k in keys:
                steps.append({"key": k, "phase": "walk", "dur": dwell, "say": None})
        self.ab = {"steps": steps, "i": -1, "t": 0.0, "keys": keys, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "settings": {"learn": learn, "dwell": dwell, "rounds": rounds}}
        self.ab_acc = {k: {"learn": [], "walk": []} for k in keys}
        self.mode = "ab"
        self.log(f"[{self.id}] A/B comparison of {', '.join(keys)}: {sum(s['dur'] for s in steps):.0f} s")

    def _ab_step(self):
        ab = self.ab
        if not ab:
            return
        if ab["i"] >= 0 and time.time() - ab["t"] < ab["steps"][ab["i"]]["dur"]:
            return
        ab["i"] += 1
        if ab["i"] >= len(ab["steps"]):
            self._ab_finish()
            return
        s = ab["steps"][ab["i"]]
        ab["t"] = time.time()
        if s["say"]:
            say(s["say"])
        if s["key"] is not None:
            self.wf_key = s["key"]
            if s["phase"] == "learn":
                self.bgs[s["key"]] = Background()
            self._use(self.reg[s["key"]], tag=s["phase"])
            if s["phase"] == "walk":
                self.bgs[s["key"]].frozen = True
        else:
            self.ring.tag = s["phase"]
        self.emit({"type": "ab", "node": self.id, "ab": self._ab_public()})

    def _ab_public(self):
        ab = self.ab
        if not ab or ab["i"] < 0:
            return None
        s = ab["steps"][min(ab["i"], len(ab["steps"]) - 1)]
        left = sum(x["dur"] for x in ab["steps"][ab["i"] + 1:]) + max(0, s["dur"] - (time.time() - ab["t"]))
        return {"step": ab["i"] + 1, "n": len(ab["steps"]), "phase": s["phase"], "key": s["key"], "left": rnd(left, 0)}

    def _ab_finish(self):
        ab = self.ab
        rows = []
        for k in ab["keys"]:
            L, Wk = self.ab_acc[k]["learn"], self.ab_acc[k]["walk"]
            fr = self.reg[k]
            med = lambda arr, f: rnd(np.median([a[f] for a in arr if a.get(f) is not None]), 1) if any(a.get(f) is not None for a in arr) else None
            det = [a for a in Wk if a["det"]]
            rows.append({"key": k, "label": fr.label, **fr.describe(self.lo), "n_learn": len(L), "n_walk": len(Wk),
                         "noise_dbfs": med(L, "noise_dbfs"), "near_resid_db": med(L, "near_resid_db"), "prf": med(Wk, "prf"),
                         "det_rate": rnd(len(det) / max(len(Wk), 1), 3), "snr_db": med(det, "snr"),
                         "snr_local_db": med(Wk, "snr_local"), "snr_back_db": med(Wk, "snr_back"),
                         "spread_local": med(Wk, "spread_local"), "spread_back": med(Wk, "spread_back"),
                         "coh": med(det, "coh"), "v_amb": med(Wk, "v_amb")})
        def score(r):
            return (r["det_rate"] or 0) * 20 + (r["snr_db"] or 0)
        best = max(rows, key=score) if rows else None
        res = {"node": self.id, "kind": self.kind, "when": ab["started"], "settings": ab["settings"], "rows": rows,
               "best": best["key"] if best else None,
               "rule": "score = 20 x detection rate + median detected SCNR (dB); SCNR is target power over the empty-scene background of that cell"}
        os.makedirs(os.path.join(DATA, "ab"), exist_ok=True)
        fn = os.path.join(DATA, "ab", f"ab_{self.id}_{self.kind}_{time.strftime('%Y%m%d_%H%M%S')}.json")
        with open(fn, "w") as f:
            json.dump(res, f, indent=1)
        self.log(f"[{self.id}] A/B finished, best: {res['best']}  -> {os.path.relpath(fn, HERE)}")
        say("Comparison finished. Thank you.")
        self.ab = None
        self.mode = "live"
        self.wf_key = res["best"] or self.wf_key
        self._use(self.reg[self.wf_key])
        for b in self.bgs.values():
            b.frozen = False
        self.emit({"type": "ab_result", "node": self.id, "result": res})

    # ------------------------------------------------------------ analysis thread
    def _analysis_main(self):
        while not self.stop_ev.is_set():
            t_next = time.time() + TICK
            try:
                ring = self.ring
                if ring is not None and self.state == "running" and time.time() - ring.born > WIN_RD + 0.15:
                    if ring.fr.kind == "txbeam":
                        self._tick_txbeam(ring)
                    else:
                        self._tick_mimo(ring)
            except Exception as e:
                self.log(f"[{self.id}] analysis error {e}\n{traceback.format_exc(limit=4)}")
                time.sleep(1.0)
            time.sleep(max(0.0, t_next - time.time()))

    def _gate_weights(self, delta, g):
        rb = D.RB
        rc = (np.arange(D.NBK) - D.A - delta) * rb
        lo, hi = g
        ov = np.clip(np.minimum(rc + rb / 2, hi) - np.maximum(rc - rb / 2, lo), 0, None) / rb
        return ov

    def _tick_mimo(self, ring):
        fr = ring.fr
        key = fr.key
        rate = max(self.rate, 50)
        T, H = ring.last(WIN_RD, rate)
        if T is None or len(T) < 64:
            return
        tu, Hu = D.uniform(T, H[:, :2])
        K = len(tu)
        prf = (K - 1) / max(tu[-1] - tu[0], 1e-3)
        lam = self.lam
        # 1. phase-noise lock on the leakage skirt one bin before the peak
        z = Hu[:, :, :, D.A - 1].reshape(K, 4)
        c = D.common_mode(z)
        Hl = Hu * np.conj(c)[:, None, None, None]
        mean_l = Hl.mean(0)
        dl = float(np.median([D.peak_pos(np.abs(mean_l[t, r])) for t in (0, 1) for r in (0, 1)]))
        delta = self.delta.get(key, dl)
        delta = 0.9 * delta + 0.1 * dl
        self.delta[key] = delta
        # 2. static removal (everything constant over the window)
        Hc = Hl - mean_l
        Hr = Hu - Hu.mean(0)
        # cleaning budget near the antennas (0..2 m) and far floor
        rc = (np.arange(D.NBK) - D.A - delta) * D.RB
        near = np.flatnonzero((rc > -1.3) & (rc < 2.0))
        far = np.flatnonzero((rc > 20) & (rc < 40))
        budget = {"static": rnd(D.db10(np.mean(np.abs(Hu[..., near]) ** 2)), 1),
                  "no_lock": rnd(D.db10(np.mean(np.abs(Hr[..., near]) ** 2)), 1),
                  "locked": rnd(D.db10(np.mean(np.abs(Hc[..., near]) ** 2)), 1),
                  "far": rnd(D.db10(np.mean(np.abs(Hc[..., far]) ** 2)), 1)}
        # 3. Doppler, range interpolation, onto the fixed velocity grid
        nd = int(2 ** np.ceil(np.log2(K)))
        f = np.fft.fftshift(np.fft.fftfreq(nd, 1 / prf))
        v = -f * lam / 2
        X = D.doppler(Hc.reshape(K, 4, D.NBK), nd)
        Xr = D.doppler(Hr.reshape(K, 4, D.NBK), nd)
        ru = (np.arange(D.NBK * U) / U - D.A - delta) * D.RB
        jsel = np.flatnonzero((ru >= 0) & (ru <= self.r_max))
        rax = ru[jsel]
        Vc = D.regrid(v, D.upsample_last(X, U)[..., jsel], VG)            # (NV, 4, NR)
        Vr = D.regrid(v, D.upsample_last(Xr, U)[..., jsel], VG)
        v_amb = prf * lam / 4
        valid = np.abs(VG) < 0.97 * v_amb
        Vc[~valid] = 0
        Vr[~valid] = 0
        Pc = np.mean(np.abs(Vc) ** 2, axis=1)                              # (NV, NR)
        Pr = np.mean(np.abs(Vr) ** 2, axis=1)
        # 4. whitening by the learned empty-scene background
        bg = self.bgs.setdefault(key, Background())
        if bg.val.get("rd") is not None and bg.val["rd"].shape != Pc.shape:
            bg.reset()
        B = bg.val.get("rd")
        learning = B is None
        notch = (np.abs(VG) < 0.06) | ~valid
        if learning:
            S_db = np.zeros_like(Pc)
            bg.feed("rd", Pc)
        else:
            S_db = D.db10(Pc / B)
            S_db[notch] = 0
            bg.feed("rd", Pc, S_db)
        floor = float(np.median(Pc[(np.abs(VG) > 1.0)][:, rax > self.r_max / 2])) if (rax > self.r_max / 2).any() else float(np.median(Pc))
        raw_db = D.db10(Pr / max(floor, 1e-30))
        # 5. detection: strongest cluster above threshold
        target = None
        cells = None
        if not learning:
            mask = (S_db > THR_DB) & ~notch[:, None]
            if mask.any():
                # detect on the whitened ratio, but locate on excess power P - B: the ratio is
                # biased away from the antennas, where the leakage residual raises B
                ex = np.where(mask, np.maximum(Pc - B, 0), 0)
                iv, ir = np.unravel_index(np.argmax(ex), ex.shape)
                er = ex.sum(0)
                near_c = np.abs(rax - rax[ir]) <= 1.0
                r_hat = float(np.sum(er * rax * near_c) / max(np.sum(er * near_c), 1e-12))
                cells = mask & near_c[None, :]
                w = ex[cells]
                vv = Vc.transpose(0, 2, 1)[cells] * np.conj(self.cal)[None]       # (n, 4) calibrated
                # MIMO beamforming on the target cells
                Aw = self.geom.steer(TH_FINE, r=max(r_hat, 0.3))       # exact paths at the target range (near field)
                Pn = np.abs(np.conj(Aw) @ vv.T) ** 2 / (4 * np.maximum(np.sum(np.abs(vv) ** 2, 1), 1e-30))   # (nth, n) 0..1
                spec = Pn @ w / max(w.sum(), 1e-12)
                ua = np.radians(self.geom.unamb_deg)
                okth = np.abs(TH_FINE) <= ua
                ith = np.flatnonzero(okth)[np.argmax(spec[okth])]
                th = float(TH_FINE[ith])
                a = Aw[ith]
                gain = np.sum(np.abs(vv @ np.conj(a)) ** 2) / max(4 * np.sum(np.mean(np.abs(vv) ** 2, 1)), 1e-30)
                af = np.abs(np.conj(Aw) @ a) ** 2 / 16
                sgn = -1 if self.flip else 1
                target = {"r": rnd(r_hat), "v": rnd(VG[iv]), "theta": rnd(sgn * np.degrees(th), 1), "snr": rnd(S_db[iv, ir], 1),
                          "gain_db": rnd(D.db10(gain), 2), "coh": rnd(gain / 4, 3), "n_cells": int(cells.sum()),
                          "spec": np.round(spec[::sgn], 3).tolist(), "af": np.round(af[::sgn], 3).tolist()}
                if self.cal_job is not None and S_db[iv, ir] > 15 and r_hat >= 1.8:
                    top = np.argsort(w)[-20:]
                    for q in top:
                        u = vv[q] * self.cal          # raw (uncalibrated) vector
                        self.cal_job["R"] += np.outer(u, np.conj(u)) / max(np.sum(np.abs(u) ** 2), 1e-30)
                        self.cal_job["n"] += 1
                        self.cal_job["rsum"] += r_hat
        self._cal_check()
        # 6. range-angle map from cells within 3 dB of the threshold (excess-weighted spatial spectrum)
        ra = np.zeros((N_TH, len(rax)))
        if not learning:
            ex3 = np.where(S_db > THR_DB - 3, np.maximum(Pc - B, 0), 0)     # 10 dB: noise cells pass ~5e-5 of the time
            Vcal = Vc * np.conj(self.cal)[None, :, None]
            At = self.geom.steer(TH, r=np.maximum(rax, 0.3))           # (N_TH, 4, NR): exact paths per range
            nrm = np.maximum(np.sum(np.abs(Vcal) ** 2, axis=1), 1e-30)          # (NV, NR)
            Y = np.abs(np.einsum("tcr,vcr->tvr", np.conj(At), Vcal)) ** 2 / (4 * nrm[None])
            ra = np.einsum("tvr,vr->tr", Y, ex3)
            if self.flip:
                ra = ra[::-1]
        # scale against the background, not the map's own maximum, so an empty scene stays dark
        ref = max(ra.max(), 30 * float(np.median(B)) if B is not None else 0.0, 1e-30)
        ra_db = D.db10(ra / ref) if ra.max() > 0 else np.full(ra.shape, -40.0)
        # 7. micro-Doppler columns for the two range gates
        spec_cols, feats = {}, {}
        ns = max(32, min(K, int(WIN_SP * prf)))
        Xs = D.doppler(Hc[-ns:].reshape(ns, 4, D.NBK), 256)
        fs_ = np.fft.fftshift(np.fft.fftfreq(256, 1 / prf))
        Ps = np.mean(np.abs(Xs) ** 2, axis=1)                                   # (256, NBK)
        now = tu[-1]
        for g in ("local", "back"):
            wgt = self._gate_weights(delta, self.gates[g])
            col = D.regrid(-fs_ * lam / 2, Ps @ wgt, VG)
            col[~valid] = 0
            Bg = bg.val.get(g)
            if Bg is None:
                bg.feed(g, col)
                sd = np.zeros(NV)
            else:
                sd = D.db10(col / Bg)
                sd[notch] = 0
                bg.feed(g, col, sd)
            spec_cols[g] = D.u8(sd, -5, 35)
            feats[g] = self._features(g, sd, now)
        # 8. track (alpha-beta on range, Doppler supplies the rate)
        if target:
            if self.track is None or now - self.track["t"] > 2.0:
                self.track = {"r": target["r"], "th": target["theta"], "t": now}
            else:
                a_ = 0.35
                self.track["r"] += a_ * (target["r"] - self.track["r"])
                self.track["th"] += a_ * (target["theta"] - self.track["th"])
                self.track["t"] = now
            target["r_trk"], target["th_trk"] = rnd(self.track["r"]), rnd(self.track["th"], 1)
            target["zone"] = "local" if self.gates["local"][0] <= target["r"] < self.gates["local"][1] else (
                "back" if self.gates["back"][0] <= target["r"] < self.gates["back"][1] else "other")
        # A/B bookkeeping
        if self.mode == "ab" and ring.tag in ("learn", "walk") and time.time() - ring.born > WIN_RD + 0.4:
            acc = self.ab_acc.setdefault(key, {"learn": [], "walk": []})
            if ring.tag == "learn" and not learning:
                farc = rax > self.r_max / 2
                nearc = rax < 2.0
                mv = (np.abs(VG) > 0.1) & (np.abs(VG) < 2.0)
                nf = float(np.median(Pc[np.abs(VG) > 1.0][:, farc])) if farc.any() else floor
                acc["learn"].append({"noise_dbfs": float(D.db10(nf)), "near_resid_db": float(D.db10(np.median(Pc[mv][:, nearc]) / nf))})
            elif ring.tag == "walk" and not learning:
                gl = (rax >= self.gates["local"][0]) & (rax < self.gates["local"][1])
                gb = (rax >= self.gates["back"][0]) & (rax < self.gates["back"][1])
                acc["walk"].append({"det": target is not None, "snr": target["snr"] if target else None,
                                    "snr_local": float(S_db[:, gl].max()) if gl.any() else None,
                                    "snr_back": float(S_db[:, gb].max()) if gb.any() else None,
                                    "spread_local": feats["local"]["spread"], "spread_back": feats["back"]["spread"],
                                    "coh": target["coh"] if target else None, "prf": prf, "v_amb": v_amb})
        msg = {"type": "tick", "node": self.id, "mode": self.mode, "wf": key, "t": rnd(now, 2), "prf": rnd(prf, 1), "v_amb": rnd(v_amb, 2),
               "learning": learning, "bg_frozen": bg.frozen, "r_axis": [rnd(rax[0], 3), rnd(rax[-1], 3), len(rax)], "v_axis": [-V_DISP, V_DISP, NV],
               "rd": D.u8(S_db.T, -5, 35), "rd_raw": D.u8(raw_db.T, -5, 55), "ra": D.u8(ra_db.T, -25, 0), "n_th": N_TH,
               "target": target, "spec": spec_cols, "feats": feats, "budget": budget, "delta": rnd(delta, 3),
               "unamb_deg": rnd(self.geom.unamb_deg, 1), "far_m": rnd(self.geom.far_m, 2), "tag": ring.tag, "cal_left": None if self.cal_job is None else rnd(self.cal_job["until"] - time.time(), 0)}
        if self.kind == "sim":
            msg["truth"] = self.radio.truth()
        self.emit(msg)

    def _features(self, g, sd, now):
        """Physics read-outs from one micro-Doppler column: torso (strongest) velocity, the
        envelope of limb velocities above 8 dB, and the gait cadence from the envelope's period."""
        m = sd > 8.0
        h = self.hist[g]
        if not m.any():
            h.append((now, 0.0, 0.0))
            out = {"active": False, "torso": None, "hi": None, "lo": None, "spread": 0.0}
        else:
            i = int(np.argmax(sd))
            vs = VG[m]
            spread = float(vs.max() - vs.min())
            e = float(np.sum(10 ** (sd[m] / 10)))
            h.append((now, spread, e))
            out = {"active": True, "torso": rnd(VG[i]), "hi": rnd(vs.max()), "lo": rnd(vs.min()), "spread": rnd(spread)}
        # cadence: autocorrelation of the envelope width over the last ~6 s
        cad = None
        if len(h) >= 40:
            s = np.array([x[1] for x in h])
            tt = np.array([x[0] for x in h])
            if s.std() > 0.15:
                s = s - s.mean()
                dt = np.median(np.diff(tt))
                ac = np.correlate(s, s, "full")[len(s) - 1:] / (s.var() * len(s))
                lo, hi = int(0.33 / dt), min(int(2.5 / dt), len(ac) - 1)
                if hi > lo + 2:
                    j = lo + int(np.argmax(ac[lo:hi]))
                    if ac[j] > 0.3:
                        cad = rnd(1 / (j * dt), 2)
        out["cadence_hz"] = cad
        t_, sp_ = out["torso"], out["spread"]
        if not out["active"]:
            out["activity"] = "quiet"
        elif t_ is not None and abs(t_) >= 0.4 and sp_ >= 1.0:
            out["activity"] = "walking"
        elif sp_ >= 0.6:
            out["activity"] = "limbs moving in place"
        else:
            out["activity"] = "small motion"
        return out

    def _cal_check(self):
        j = self.cal_job
        if j is None or time.time() < j["until"]:
            return
        self.cal_job = None
        if j["n"] < 30:
            self.log(f"[{self.id}] calibration failed: only {j['n']} target cells seen. Stand in front and move more.")
            self.emit({"type": "cal", "node": self.id, "ok": False, "n": j["n"]})
            say("Calibration failed. No clear target.")
            return
        r_cal = j["rsum"] / max(j["n"], 1)
        cal, q = D.calibrate(j["R"], self.geom.steer(np.radians(j["theta"]), r=r_cal))
        self.cal, self.cal_q = cal, q
        os.makedirs(DATA, exist_ok=True)
        with open(self._cal_path(), "w") as f:
            json.dump({"re": cal.real.tolist(), "im": cal.imag.tolist(), "quality": q, "n": j["n"], "theta_cal": j["theta"],
                       "deg": np.degrees(np.angle(cal)).tolist(), "r_cal": r_cal, "geom": self.geom.info(), "when": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=1)
        self.log(f"[{self.id}] calibrated: phases {np.round(np.degrees(np.angle(cal)), 1).tolist()} deg, point-target quality {q:.2f}")
        self.emit({"type": "cal", "node": self.id, "ok": True, "q": q, "deg": np.degrees(np.angle(cal)).tolist()})
        say("Calibration done.")

    def _tick_txbeam(self, ring):
        """Physical TX beams: motion energy in each steered slot vs the beam predicted from
        the two single-TX slots. Agreement shows the transmitters add coherently in the air."""
        fr = ring.fr
        T, H = ring.last(WIN_TB, max(self.rate, 30))
        if T is None or len(T) < 48:
            return
        tu, Hu = D.uniform(T, H)
        K = len(tu)
        prf = (K - 1) / max(tu[-1] - tu[0], 1e-3)
        z = Hu[:, 1:3, :, D.A - 1].reshape(K, 4)
        c = D.common_mode(z)
        Hl = Hu * np.conj(c)[:, None, None, None]
        mean_l = Hl.mean(0)
        delta = float(np.median([D.peak_pos(np.abs(mean_l[s, r])) for s in (1, 2) for r in (0, 1)]))
        Hc = Hl - mean_l
        nd = int(2 ** np.ceil(np.log2(K)))
        X = D.doppler(Hc, nd)                                          # (nd, S, 2, NBK)
        f = np.fft.fftshift(np.fft.fftfreq(nd, 1 / prf))
        band = np.abs(f * self.lam / 2) > 0.15
        E = np.sum(np.abs(X[band]) ** 2, axis=(0, 2))                  # (S, NBK)
        Em = E[1] + E[2]
        if self.tb_bg is None or self.tb_bg["n"] < LEARN_TICKS:
            if self.tb_bg is None:
                self.tb_bg = {"stack": [], "n": 0}
            self.tb_bg["stack"].append(Em)
            self.tb_bg["n"] += 1
            self.tb_bg["val"] = np.median(np.array(self.tb_bg["stack"]), axis=0)
            self.emit({"type": "txbeam", "node": self.id, "learning": True, "t": rnd(tu[-1]), "prf": rnd(prf, 1)})
            return
        rc = (np.arange(D.NBK) - D.A - delta) * D.RB
        cand = np.flatnonzero((rc > -1.3) & (rc < self.r_max))
        bgv = self.tb_bg["val"]
        snr = Em[cand] / np.maximum(bgv[cand], 1e-30)
        exc = np.maximum(Em - bgv, 0)
        b = int(cand[np.argmax(exc[cand])])                            # locate on excess, not on the ratio
        snr_db = float(D.db10(Em[b] / max(bgv[b], 1e-30)))
        r_hat = rc[b]
        if 0 < b < D.NBK - 1 and exc[b - 1] > 0 and exc[b + 1] > 0:
            l_, m_, h_ = np.log(exc[b - 1]), np.log(exc[b]), np.log(exc[b + 1])
            den = l_ - 2 * m_ + h_
            if den < 0:
                r_hat = rc[b] + 0.5 * (l_ - h_) / den * D.RB
        if snr.max() < 4:
            self.tb_bg["val"] = 0.95 * bgv + 0.05 * Em
        X1, X2 = X[band][:, 1, :, b], X[band][:, 2, :, b]
        meas = E[3:, b]
        pred = np.array([np.sum(np.abs(X1 + np.exp(1j * ph) * X2) ** 2) for ph in fr.phases])
        single = 0.5 * (E[1, b] + E[2, b])
        corr = float(np.corrcoef(meas, pred)[0, 1]) if meas.std() > 0 and pred.std() > 0 else None
        # TX2-vs-TX1 chain phase from the MIMO calibration (mean over both RX)
        psi = float(np.angle(self.cal[2] * np.conj(self.cal[0]) + self.cal[3] * np.conj(self.cal[1])))
        kd = self.geom.k * self.geom.d_tx                              # the two TX elements' real spacing
        sgn = -1 if self.flip else 1

        def to_angle(phase):                         # TX2/TX1 path phase -> angle
            w = (phase - psi + np.pi) % (2 * np.pi) - np.pi
            return sgn * float(np.degrees(np.arcsin(np.clip(w / kd, -1, 1))))
        # beam i points where phi_i + (TX2/TX1 path phase) = 0
        ang = np.array([to_angle(-ph) for ph in fr.phases])
        # the 12 beam powers trace A + B cos(phi + z): their first harmonic gives z exactly
        z_beam = np.angle(np.sum(meas * np.exp(-1j * fr.phases)))
        z_mimo = np.angle(np.sum(np.conj(X1) * X2))
        ib = int(np.argmax(meas))
        msg = {"type": "txbeam", "node": self.id, "learning": False, "t": rnd(tu[-1]), "prf": rnd(prf, 1), "bin": b, "r": rnd(r_hat),
               "snr_db": rnd(snr_db, 1), "meas": np.round(D.db10(meas / max(meas.max(), 1e-30)), 2).tolist(),
               "pred": np.round(D.db10(pred / max(pred.max(), 1e-30)), 2).tolist(),
               "gain_db": rnd(D.db10(meas.max() / max(single, 1e-30)), 2), "null_db": rnd(D.db10(meas.min() / max(meas.max(), 1e-30)), 1),
               "corr": rnd(corr, 3), "phases_deg": np.round(np.degrees(fr.phases), 1).tolist(), "angles": np.round(ang, 1).tolist(),
               "psi_deg": rnd(np.degrees(psi), 1), "kd": rnd(kd, 4), "flip": self.flip, "best": ib, "best_angle": rnd(ang[ib], 1),
               "angle_fit": rnd(to_angle(z_beam), 1), "angle_mimo": rnd(to_angle(z_mimo), 1), "present": snr_db >= 8}
        if self.kind == "sim":
            msg["truth"] = self.radio.truth()
        self.emit(msg)
