"""Headless check on the simulator: does the chain find the simulated person at the right
range, velocity and angle, and does calibration recover the simulator's chain phases?

  py -3.11 selftest.py [--secs 40] [--wf fmcw]
"""
import argparse
import time

import numpy as np

import node as N

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=40)
ap.add_argument("--wf", default="fmcw")
ap.add_argument("--txbeam", action="store_true")
a = ap.parse_args()

ticks, logs = [], []
nd = N.Node({"id": "T", "waveform": a.wf}, lambda m: ticks.append(m), lambda s: logs.append(s), sim=True)
nd.start()
time.sleep(1.0)
if a.txbeam:
    nd.command("mode", {"mode": "txbeam"})
t0 = time.time()
while time.time() - t0 < a.secs and nd.running():
    time.sleep(0.5)
nd.stop()
time.sleep(0.5)
for s in logs:
    print("LOG", s[:300])
st = [m for m in ticks if m["type"] == "status"]
print("capture rate", st[-1]["node"]["rate"] if st else None)
if a.txbeam:
    tb = [m for m in ticks if m["type"] == "txbeam" and not m["learning"] and m["present"]]
    print("txbeam ticks with target", len(tb))
    for m in tb[::10]:
        tr = m["truth"]
        print(f"t={m['t']:5.1f} r={m['r']:5.2f} (true {tr['r']:.2f}) best beam {m['best_angle']:6.1f} fit {m['angle_fit']:6.1f} mimo {m['angle_mimo']:6.1f} (true {tr['theta_deg']:6.1f}) gain {m['gain_db']} dB null {m['null_db']} corr {m['corr']}")
    raise SystemExit
tk = [m for m in ticks if m["type"] == "tick"]
print("ticks", len(tk), "learning", sum(m["learning"] for m in tk))
err_r, err_th, n_det, n_in = [], [], 0, 0
for m in tk:
    tr = m["truth"]
    if m["learning"]:
        continue
    inside = tr["mode"] != "out" and tr["r"] < 11
    n_in += inside
    if m["target"] and inside:
        n_det += 1
        err_r.append(m["target"]["r"] - tr["r"])
        if abs(tr["theta_deg"]) < 28:
            err_th.append(m["target"]["theta"] - tr["theta_deg"])
for m in tk[::15]:
    tr, tg = m["truth"], m["target"]
    s = f"t={m['t']:5.1f} true r={tr['r']:5.2f} th={tr['theta_deg']:6.1f} vr={tr['v_radial']:5.2f} {tr['mode']:5s} | "
    s += "none" if not tg else f"r={tg['r']:5.2f} th={tg['theta']:6.1f} v={tg['v']:5.2f} snr={tg['snr']:5.1f} gain={tg['gain_db']:4.1f} {m['feats']['local']['activity'][:12]:12s}/{m['feats']['back']['activity'][:12]}"
    print(s)
print(f"detected {n_det}/{n_in} ticks with the person inside; range err median {np.median(err_r):.2f} m, p90 |err| {np.percentile(np.abs(err_r), 90):.2f} m;"
      f" angle err median {np.median(err_th) if err_th else float('nan'):.1f} deg (uncalibrated)")
fa = [m for m in tk if not m["learning"] and m["target"] and (m["truth"]["mode"] == "out" or m["truth"]["r"] > 12.5)]
n_out = sum(1 for m in tk if not m["learning"] and (m["truth"]["mode"] == "out" or m["truth"]["r"] > 12.5))
print(f"false alarms {len(fa)}/{n_out} ticks with nobody inside")
for mode in ("walk", "wave", "still"):
    sel = [m for m in tk if not m["learning"] and m["truth"]["mode"] == mode and m["truth"]["r"] < 11]
    print(f"  {mode:5s}: detected {sum(1 for m in sel if m['target'])}/{len(sel)}")
print("budget", tk[-1]["budget"], "prf", tk[-1]["prf"])
