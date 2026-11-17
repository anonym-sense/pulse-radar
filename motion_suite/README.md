# Motion Radar Suite

Live human-motion radar for the AD9361 2T2R boards (Pluto firmware) at 5.845 GHz. Physics only, no training:
a cleaned range–Doppler map, MIMO and physical TX beamforming you can see, micro-Doppler in a **local** range
gate and body motion in a **back** gate, and a round-robin test of which waveform works best for a moving person.
Built for one SDR now; the second one is a config switch (`nodes.json`).

Lives beside the pulse-Doppler bring-up GUI at the repo root and uses the same boards.

## Run

```
py -3.11 server.py                          # boards from nodes.json  -> http://localhost:8790/
py -3.11 server.py --sim                    # no hardware: physics simulator, same pipeline, no speech
py -3.11 server.py --uri A=ip:192.168.2.1   # reach board A over its network link instead of USB
py -3.11 selftest.py [--txbeam]             # headless check against simulated truth
```

Find USB addresses with `iio_info -s`. Both boards report the **same serial**, so they are told apart only by
USB bus address (`usb:2.5.5`, `usb:2.6.5`, …), which changes when re-plugged. On Windows a USB board can be
opened by only one process: stop any other server or lab page using it first.

## What the page does

| Panel | What it shows |
|---|---|
| Range–Doppler, cleaned | phase-noise lock on the leakage skirt → static removal → whitening by the learned empty scene. *Raw* toggle shows it without the lock and whitening. |
| Range–angle | Bartlett scan of the 4-element virtual array (TX at 0, d; RX at 2d, 3d → virtual 2d, 3d, 3d, 4d) over every cell ≥ 10 dB, with the steered beam's lobe drawn on top. |
| Target | range, angle, radial speed, SNR over background, beamforming gain (6.02 dB ceiling for 4 paths), coherence. |
| Cleaning bars | dBFS near the antennas at each stage, so "cleaned" is a number. |
| Micro-Doppler local / Body motion back | scrolling spectrograms per range gate with torso velocity, limb envelope, spread and envelope period. |
| Receive beam | measured spatial spectrum of the target vs the steered array factor. |
| Physical TX beams | *TX beam scan* mode: TX1+TX2 radiate together, TX2 advanced 0…330°. Each lobe is a real beam shaded by what it returned; bars compare against the prediction from the single-TX slots. |
| Room | top view of every node and its track; fuses two nodes as tracks. |
| Waveform comparison | spoken-cue run: each waveform learns the empty scene, then they take turns (round-robin) while you move. Saved to `data/ab/`. |

Waveforms (all at the same DAC peak, so duty decides energy): pure pulse (2 samples), gated LFM pulse
(4.2 µs, 25 %), FMCW (8.3 µs sweep per TX, continuous), long FMCW (16.7 µs). FMCW here is compressed
digitally by a matched filter, not by a mixer dechirp.

## Antenna layout (enter it before trusting angles)

`nodes.json` → `antennas`: the `[x, y]` position of each TX (pole) and RX (patch) feed in metres. Angles depend
on the **TX-to-TX and RX-to-RX spacings**. The gap between the pole group and the patch group only shifts the
virtual array, and calibration absorbs that. Steering uses exact TX → point → RX paths, so the near field
(< 2D²/λ, ~3.5 m for a 30 cm span) is handled. Simulated, with poles 100 mm apart: the wrong (default) layout
gave 7.8° p90 angle error and 5.5 dB gain; the right one gave 2.4° and 6.0 dB. Poles 60 mm apart: about 1°
either way. Range, Doppler and micro-Doppler do not depend on the layout.

Mixed antennas: a vertical pole and a patch need the **same polarisation** (patch feed direction matching the
pole), or the echo loses 10–20 dB. The poles are omnidirectional, so echoes from behind and beside the radar are
suppressed only once, by the patches' front-to-back ratio.

## Procedure

1. Start, leave the area for the first 2 s (background learning; *Relearn background* repeats it).
2. **Calibrate angle**: walk slowly toward and away from the radar along its centre line, 2–5 m, arms still
   (12 s). A waving person or someone closer than ~2 m is not a point target in the far field, and gives wrong phases.
   Saved per node and radio kind in `data/calib_*.json`. If angles come out mirrored, use *Flip L/R*.
3. *Compare waveforms*: follow the spoken cues (~90 s at the defaults).

## Physics limits (read before trusting a number)

- **Range resolution is 3 m** (50 MHz). A person 1.5 m away spills into a gate starting at 2.5 m; keep the local
  and back gates ≥ 1.5 m apart (defaults 0.3–2.5 m and 4–9 m). The range read-out is finer than that (centroid of
  excess power) but two people closer than ~3 m in range merge.
- **Angles are unambiguous only within ±30°**: element pitch 50.8 mm ≈ 1 λ, so beyond that the scan aliases
  (grating lobes, drawn on the page). Two TX elements make physical TX beams ~30° wide.
- Of the ~6 dB "best TX beam vs one TX", 3 dB is the second transmitter's power, 3 dB is coherent addition.
- A person standing still (breathing only) is mostly **not** detected: the detector works on Doppler.
- Thresholds: detection at 13 dB over a per-cell mean background (a median/log-mean background sat 1.6–2.5 dB low
  and produced false alarms every tick across ~5000 cells).
- Two boards are not phase-locked: they are fused as tracks, never coherently. Give them different LOs
  (node B defaults to 5.765 GHz; check that channel for Wi-Fi bursts first — 5.785–5.8 GHz had them).
- Slow time uses host timestamps (resampled uniform); there is no hardware timestamp.

## Verified (2026-11-16)

Simulator (articulated walking/waving person, real frames through a geometric channel, hidden chain phases):
walking detected 99 %, waving 100 %, 0 false alarms with nobody inside; range error median 0.16 m (p90 0.5 m);
calibration recovered the hidden chain phases to within ~6° and lifted beam gain from 4.0 to 5.94 dB;
TX-beam angles within ~2–4° after calibration.

Board A (empty scene, FMCW, RX 38 dB): 411 captures/s over `ip:`, ADC peak −5.8 dBFS, 42 dB of leakage removed,
no false target. **Not yet done on hardware: a person walking, calibration, TX-beam scan, the waveform comparison.**
At RX 45 dB (the earlier setting) this setup clipped at +3 dBFS, hence 38 dB.

## Files

| File | Purpose |
|---|---|
| `radio.py` | board (libiio) + physics simulator, same interface; `ready()` tone-converge-then-freeze |
| `waveforms.py` | pulse, LFM, FMCW, long FMCW (TDM MIMO) and the 15-slot TX-beam frame |
| `dsp.py` | matched filters, uniform resampling, common-mode lock, Doppler, steering, calibration |
| `node.py` | per-board engine: radio thread + 10 Hz analysis, A/B schedule, calibration |
| `server.py`, `nodes.json` | web server, node config (URI, LO, gains, pose, gates) |
| `static/` | the page (plain canvas) |
| `selftest.py` | headless simulator check |

Transmitting must be legal where you are: check the rules for the band and power you use.
