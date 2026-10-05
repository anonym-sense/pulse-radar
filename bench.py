"""Command-line bring-up check: digital loopback, then over-the-air TX/RX matrix.

    py -3.11 bench.py [serial|iio|sim|auto] [fs_hz] [rx_gain_db] [tx_gain_db]
"""
import sys
import time

import numpy as np

import radar_dsp as dsp
from radios import open_radio

transport = sys.argv[1] if len(sys.argv) > 1 else "auto"
fs = float(sys.argv[2]) if len(sys.argv) > 2 else 30.72e6
rxg = float(sys.argv[3]) if len(sys.argv) > 3 else 20
txg = float(sys.argv[4]) if len(sys.argv) > 4 else -10
LO, P, L, K = 5.8e9, 1024, 256, 4

r = open_radio(transport)
try:
    t = time.time()
    for k, v in r.info().items():
        print(f"  {k:10s} {v}")
    r.configure(fs, LO, 0.9 * fs if fs <= 56e6 else 56e6, [txg, txg], [rxg, rxg])
    print(f"configured in {time.time() - t:.1f}s ->", {k: v for k, v in r.info().items() if k in ("fs", "rx_lo", "rx_bw", "rx_gain", "tx_gain")})
    win = dsp.make_window(fs, 0.8 * fs, P)

    for name, loop in (("DIGITAL LOOPBACK", 1), ("OVER THE AIR", 0)):
        r.set_loopback(loop)
        print()
        print("==", name)
        for i in range(2):
            frame, refs = dsp.make_frame(fs, 0.8 * fs, L, P, f"tx{i + 1}")
            r.start_tx(frame)
            t = time.time()
            rx = r.capture(P * K)
            dt = time.time() - t
            fo = dsp.fold(rx, P)
            h = dsp.compress(fo.mean(1), refs, win)
            d0 = dsp.find_delay(h, (i,))
            print(f" TX{i + 1} on (capture {dt:.2f}s, start offset {d0})")
            for c, st in enumerate(dsp.channel_stats(rx)):
                print(f"  RX{c + 1}: " + "  ".join(f"{k}={v:7.2f}" for k, v in st.items()))
            for j in range(2):
                q = dsp.path_quality(h[i, j], fo[j], refs[i], d0, fs, L)
                print(f"  TX{i + 1}->RX{j + 1}: " + " ".join(f"{k}={v:.2f}" for k, v in q.items()))
            np.savez(f"bench_{'loop' if loop else 'ota'}_tx{i + 1}.npz", rx=rx, frame=frame, refs=refs, fs=fs)
    r.set_loopback(0)
finally:
    r.close()
