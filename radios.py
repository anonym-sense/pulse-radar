"""Radio back-ends for the AD9361 2T2R board (Pluto firmware).

All back-ends expose the same small interface:

    configure(fs, lo, rf_bw, tx_gain_db, rx_gain_db)
    set_loopback(mode)          0 = normal, 1 = digital TX->RX inside the AD9361
    start_tx(frame)             (2, n) complex64, |x|<=1, played cyclically
    stop_tx()
    capture(n) -> (2, n) complex64, 1.0 = ADC full scale
    info() -> dict
    close()

IIORadio     live streaming over the board's data link (USB gadget / Ethernet).
SerialRadio  drives the on-board iio tools through the debug UART console.
             Slow (a frame every second or so) but needs only the debug cable.
SimRadio     synthetic scene, for testing the GUI and DSP without hardware.
"""
from __future__ import annotations

import gzip
import hashlib
import re
import time

import numpy as np

RX_FS = 2048.0      # 12-bit ADC, right-justified
TX_FS = 32767.0     # 12-bit DAC, left-justified in 16 bits

PHY = "/sys/bus/iio/devices/iio:device0"
DDS = "/sys/bus/iio/devices/iio:device2"
DBG = "/sys/kernel/debug/iio/iio:device0"


def _to_tx_bytes(frame: np.ndarray) -> bytes:
    out = np.empty((frame.shape[1], 4), np.int16)
    for c in range(2):
        out[:, 2 * c] = np.clip(np.round(frame[c].real * TX_FS), -32768, 32767)
        out[:, 2 * c + 1] = np.clip(np.round(frame[c].imag * TX_FS), -32768, 32767)
    return out.tobytes()


def _from_rx_bytes(raw: bytes) -> np.ndarray:
    a = np.frombuffer(raw, np.int16).reshape(-1, 4).astype(np.float32) / RX_FS
    return np.stack([a[:, 0] + 1j * a[:, 1], a[:, 2] + 1j * a[:, 3]]).astype(np.complex64)


# ============================================================== serial console
class Console:
    """Minimal driver for the board's root shell on the debug UART."""

    def __init__(self, port="COM5", baud=115200, user="root", password="analog"):
        import serial
        self.port = port
        self.s = serial.Serial(port, baud, timeout=0.05)
        self.user, self.password = user, password
        self._login()

    def _read_for(self, t):
        end, buf = time.time() + t, b""
        while time.time() < end:
            buf += self.s.read(65536)
        return buf

    def _read_until(self, pat: bytes, timeout):
        end, buf = time.time() + timeout, b""
        rx = re.compile(pat)
        while time.time() < end:
            chunk = self.s.read(65536)
            if chunk:
                buf += chunk
                m = rx.search(buf)
                if m:
                    return buf, m
        raise TimeoutError(f"console: no match for {pat!r}; tail={buf[-200:]!r}")

    def _login(self):
        # a previous session that died may have left the board at the fast rate
        for baud in (self.s.baudrate, 921600, 115200):
            self.s.baudrate = baud
            self.s.reset_input_buffer()
            self.s.write(b"\x03\n")
            out = self._read_for(0.6)
            if b"login:" in out:
                self.s.write(self.user.encode() + b"\n")
                out = self._read_for(0.6)
                if b"assword" in out:
                    self.s.write(self.password.encode() + b"\n")
                    self._read_for(1.0)
            try:
                self.run("true", timeout=1.5)
                return
            except TimeoutError:
                continue
        raise IOError(f"no shell on {self.port}")

    def run(self, cmd: str, timeout=10.0) -> str:
        """Run a shell command, return its stdout+stderr."""
        self.s.reset_input_buffer()
        self.s.write(f'echo __S""TART__; {cmd}; echo __E""ND__$?\n'.encode())
        buf, m = self._read_until(rb"__END__(\d+)", timeout)
        body = buf[buf.rfind(b"__START__") + 9:m.start()]
        return body.decode(errors="replace").replace("\r", "").strip("\n")

    def set_baud(self, baud: int) -> bool:
        """Switch both ends to `baud`; the board falls back by itself if the
        host cannot talk to it at the new rate."""
        old = self.s.baudrate
        if baud == old:
            return True
        self.run("rm -f /tmp/baud_ok")
        self.s.write(f"(sleep 5; [ -f /tmp/baud_ok ] || stty -F /dev/ttyPS0 {old}) & "
                     f"stty -F /dev/ttyPS0 {baud}\n".encode())
        time.sleep(0.4)
        self.s.baudrate = baud
        self.s.reset_input_buffer()
        try:
            self.s.write(b"\n")
            self._read_for(0.2)
            if self.run("touch /tmp/baud_ok; echo fine", timeout=2.0).endswith("fine"):
                return True
        except TimeoutError:
            pass
        time.sleep(5.5)
        self.s.baudrate = old
        self.s.reset_input_buffer()
        self.run("true", timeout=3.0)
        return False

    def upload(self, data: bytes, path: str):
        hx = data.hex()
        md5 = hashlib.md5(data).hexdigest()
        for _ in range(3):
            self.s.reset_input_buffer()
            self.s.write(f"stty -echo; cat > {path}.hex <<'XEOF'\n".encode())
            for i in range(0, len(hx), 60):
                self.s.write(hx[i:i + 60].encode() + b"\n")
                if (i // 60) % 16 == 15:
                    self.s.flush()
                    time.sleep(0.004)
                    self.s.read(65536)
            self.s.write(b"XEOF\nstty echo\n")
            self.s.flush()
            self._read_for(0.3)
            got = self.run(f"xxd -r -p {path}.hex > {path}; rm {path}.hex; md5sum {path}")
            if md5 in got:
                return
        raise IOError(f"console upload of {path} failed checksum")

    def download(self, path: str, timeout=60.0) -> bytes:
        for _ in range(3):
            out = self.run(f"md5sum < {path}; gzip -1 -c {path} | xxd -p -c 60", timeout)
            lines = out.split("\n")
            md5 = lines[0].split()[0]
            try:
                data = gzip.decompress(bytes.fromhex("".join(lines[1:])))
            except Exception:
                continue
            if hashlib.md5(data).hexdigest() == md5:
                return data
        raise IOError(f"console download of {path} failed checksum")

    def close(self):
        try:
            if self.s.baudrate != 115200:
                self.s.write(b"stty -F /dev/ttyPS0 115200\n")
                time.sleep(0.3)
            self.s.close()
        except Exception:
            pass


class SerialRadio:
    kind = "serial"

    def __init__(self, port="COM5", fast_baud=921600):
        self.con = Console(port)
        self.baud = 115200
        if fast_baud and self.con.set_baud(fast_baud):
            self.baud = fast_baud
        self.run = self.con.run
        self.run("killall iio_writedev iio_readdev 2>/dev/null")
        self.run(f"for f in {DDS}/out_altvoltage*_scale; do echo 0 > $f; done")
        self.fs = 0.0

    def info(self):
        g = lambda p: self.run(f"cat {p}").strip()
        return {
            "transport": f"serial console {self.con.port} @ {self.baud} baud (slow path)",
            "model": g("/sys/firmware/devicetree/base/model").strip("\x00"),
            "chip": self.run("iio_attr -C 2>/dev/null | grep 'ad9361-phy,model' | sed 's/.*value: //'"),
            "fw": self.run("iio_attr -C 2>/dev/null | grep fw_version | sed 's/.*value: //'"),
            "fs": float(g(f"{PHY}/in_voltage_sampling_frequency")),
            "rx_lo": float(g(f"{PHY}/out_altvoltage0_RX_LO_frequency")),
            "tx_lo": float(g(f"{PHY}/out_altvoltage1_TX_LO_frequency")),
            "rx_bw": float(g(f"{PHY}/in_voltage_rf_bandwidth")),
            "tx_bw": float(g(f"{PHY}/out_voltage_rf_bandwidth")),
            "rx_gain": [float(g(f"{PHY}/in_voltage{c}_hardwaregain").split()[0]) for c in (0, 1)],
            "tx_gain": [float(g(f"{PHY}/out_voltage{c}_hardwaregain").split()[0]) for c in (0, 1)],
            "rates": g(f"{PHY}/rx_path_rates"),
            "temp_c": float(g(f"{PHY}/in_temp0_input")) / 1000.0,
        }

    def configure(self, fs, lo, rf_bw, tx_gain_db, rx_gain_db):
        self.stop_tx()
        cmds = [
            f"echo {int(fs)} > {PHY}/in_voltage_sampling_frequency",
            f"echo {int(rf_bw)} > {PHY}/in_voltage_rf_bandwidth",
            f"echo {int(rf_bw)} > {PHY}/out_voltage_rf_bandwidth",
            f"echo {int(lo)} > {PHY}/out_altvoltage0_RX_LO_frequency",
            f"echo {int(lo)} > {PHY}/out_altvoltage1_TX_LO_frequency",
        ]
        for c in (0, 1):
            cmds += [f"echo manual > {PHY}/in_voltage{c}_gain_control_mode",
                     f"echo {rx_gain_db[c]} > {PHY}/in_voltage{c}_hardwaregain",
                     f"echo {tx_gain_db[c]} > {PHY}/out_voltage{c}_hardwaregain"]
        err = self.run(" && ".join(cmds) + " && echo cfg_ok", timeout=20)
        if "cfg_ok" not in err:
            raise IOError("configure failed: " + err)
        self.fs = fs

    def set_gains(self, tx_gain_db, rx_gain_db):
        cmds = []
        for c in (0, 1):
            cmds += [f"echo {rx_gain_db[c]} > {PHY}/in_voltage{c}_hardwaregain",
                     f"echo {tx_gain_db[c]} > {PHY}/out_voltage{c}_hardwaregain"]
        self.run("; ".join(cmds))

    def set_loopback(self, mode: int):
        self.run(f"echo {int(mode)} > {DBG}/loopback")

    def start_tx(self, frame):
        self.stop_tx()
        n = frame.shape[1]
        self.con.upload(_to_tx_bytes(frame), "/tmp/tx.bin")
        self.run(f"(iio_writedev -c -b {n} cf-ad9361-dds-core-lpc voltage0 voltage1 voltage2 voltage3 "
                 f"< /tmp/tx.bin > /tmp/tx.log 2>&1 &)")
        time.sleep(0.4)
        if "iio_writedev" not in self.run("ps | grep iio_writedev | grep -v grep"):
            raise IOError("iio_writedev exited: " + self.run("cat /tmp/tx.log"))

    def stop_tx(self):
        self.run("killall iio_writedev 2>/dev/null; sleep 0.2")

    def capture(self, n):
        # two buffers, keep the second: the first can hold enable transients
        out = self.run(f"iio_readdev -b {n} -s {2 * n} cf-ad9361-lpc voltage0 voltage1 voltage2 voltage3 "
                       f"2>/tmp/rx.log | tail -c {n * 8} > /tmp/rx.bin; wc -c < /tmp/rx.bin", timeout=20)
        if out.strip().split()[-1] != str(n * 8):
            raise IOError("capture failed: " + out + self.run("cat /tmp/rx.log"))
        return _from_rx_bytes(self.con.download("/tmp/rx.bin"))

    def close(self):
        try:
            self.stop_tx()
            self.set_loopback(0)
        finally:
            self.con.close()


# ==================================================================== libiio
class IIORadio:
    kind = "iio"

    def __init__(self, uri="ip:192.168.2.1"):
        import adi
        self.uri = uri
        self.sdr = adi.ad9361(uri)
        s = self.sdr
        s.rx_enabled_channels = [0, 1]
        s.tx_enabled_channels = [0, 1]
        s.tx_cyclic_buffer = True
        self.n = 0

    def info(self):
        s = self.sdr
        a = s.ctx.attrs
        return {
            "transport": f"libiio {self.uri} (live)",
            "model": a.get("hw_model", "?"), "chip": a.get("ad9361-phy,model", "?"),
            "fw": a.get("fw_version", "?"),
            "fs": float(s.sample_rate), "rx_lo": float(s.rx_lo), "tx_lo": float(s.tx_lo),
            "rx_bw": float(s.rx_rf_bandwidth), "tx_bw": float(s.tx_rf_bandwidth),
            "rx_gain": [float(s.rx_hardwaregain_chan0), float(s.rx_hardwaregain_chan1)],
            "tx_gain": [float(s.tx_hardwaregain_chan0), float(s.tx_hardwaregain_chan1)],
            "rates": "", "temp_c": float("nan"),
        }

    def configure(self, fs, lo, rf_bw, tx_gain_db, rx_gain_db):
        s = self.sdr
        self.stop_tx()
        s.rx_destroy_buffer()
        s.sample_rate = int(fs)
        s.rx_rf_bandwidth = int(rf_bw)
        s.tx_rf_bandwidth = int(rf_bw)
        s.rx_lo = int(lo)
        s.tx_lo = int(lo)
        s.gain_control_mode_chan0 = "manual"
        s.gain_control_mode_chan1 = "manual"
        self.set_gains(tx_gain_db, rx_gain_db)

    def set_gains(self, tx_gain_db, rx_gain_db):
        s = self.sdr
        s.rx_hardwaregain_chan0, s.rx_hardwaregain_chan1 = rx_gain_db
        s.tx_hardwaregain_chan0, s.tx_hardwaregain_chan1 = tx_gain_db

    def set_loopback(self, mode):
        self.sdr.loopback = int(mode)

    def start_tx(self, frame):
        self.stop_tx()
        self.sdr.tx([frame[0] * TX_FS, frame[1] * TX_FS])

    def stop_tx(self):
        try:
            self.sdr.tx_destroy_buffer()
        except Exception:
            pass

    def capture(self, n):
        s = self.sdr
        if n != self.n:
            s.rx_destroy_buffer()
            s.rx_buffer_size = n
            self.n = n
            try:    # keep only the freshest buffer so host timestamps mean something
                s._rxadc.set_kernel_buffers_count(1)
            except Exception:
                pass
            s.rx()
        d = s.rx()
        return (np.stack(d) / RX_FS).astype(np.complex64)

    def close(self):
        try:
            self.stop_tx()
            self.sdr.loopback = 0
            self.sdr.rx_destroy_buffer()
        except Exception:
            pass


# ================================================================= simulation
class SimRadio:
    kind = "sim"

    def __init__(self):
        self.fs, self.lo = 30.72e6, 5.8e9
        self.frame = np.zeros((2, 1024), np.complex64)
        self.loop = 0
        self.txg, self.rxg = [-10, -10], [20, 20]
        self.rng = np.random.default_rng(1)
        self.t0 = time.time()

    def info(self):
        return {"transport": "SIMULATION (no hardware)", "model": "synthetic scene", "chip": "sim",
                "fw": "-", "fs": self.fs, "rx_lo": self.lo, "tx_lo": self.lo, "rx_bw": self.fs,
                "tx_bw": self.fs, "rx_gain": self.rxg, "tx_gain": self.txg, "rates": "", "temp_c": 25.0}

    def configure(self, fs, lo, rf_bw, tx_gain_db, rx_gain_db):
        self.fs, self.lo = fs, lo
        self.set_gains(tx_gain_db, rx_gain_db)

    def set_gains(self, tx_gain_db, rx_gain_db):
        self.txg, self.rxg = list(tx_gain_db), list(rx_gain_db)

    def set_loopback(self, mode):
        self.loop = mode

    def start_tx(self, frame):
        self.frame = frame.copy()

    def stop_tx(self):
        self.frame = np.zeros_like(self.frame)

    def _delay(self, X, d):
        f = np.fft.fftfreq(X.shape[-1])
        return X * np.exp(-2j * np.pi * f * d)

    def capture(self, n):
        P = self.frame.shape[1]
        t = time.time() - self.t0
        lam = 299792458.0 / self.lo
        F = np.fft.fft(self.frame, axis=-1)
        rx = np.zeros((2, P), complex)
        if self.loop == 1:
            t16 = np.round(self.frame * TX_FS)
            rx = (np.floor(t16.real / 16) + 1j * np.floor(t16.imag / 16)) / RX_FS
        else:
            R = 2.5 + 0.15 * np.sin(2 * np.pi * 0.25 * t)
            for j in range(2):
                g_rx = 10 ** ((self.rxg[j] - 20) / 20)
                for i in range(2):
                    g = g_rx * 10 ** ((self.txg[i] + 10) / 20)
                    leak = 0.6 * np.exp(1j * (0.4 + i + 2 * j))
                    y = self._delay(F[i], 2.3) * leak
                    for rr, a in ((1.2, 0.02), (3.7, 0.01), (6.0, 0.006)):
                        y += self._delay(F[i], 2.3 + 2 * rr * self.fs / 299792458.0) * a * np.exp(-4j * np.pi * rr / lam)
                    y += self._delay(F[i], 2.3 + 2 * R * self.fs / 299792458.0) * 0.004 * np.exp(-4j * np.pi * R / lam)
                    rx[j] += g * np.fft.ifft(y)
        k = n // P + 1
        off = int(self.rng.integers(P))
        out = np.tile(rx, (1, k + 1))[:, off:off + n]
        if self.loop != 1:
            out = out + (self.rng.normal(size=out.shape) + 1j * self.rng.normal(size=out.shape)) * 0.002
        q = lambda v: np.clip(np.round(v * RX_FS), -2048, 2047) / RX_FS
        time.sleep(0.02)
        return (q(out.real) + 1j * q(out.imag)).astype(np.complex64)

    def close(self):
        pass


def open_radio(transport="auto", uri="", port="COM5", log=print):
    """transport: auto | iio | serial | sim"""
    if transport == "sim":
        return SimRadio()
    if transport in ("auto", "iio"):
        for u in ([uri] if uri else ["usb:", "ip:192.168.2.1", "ip:pluto.local"]):
            try:
                log(f"trying libiio {u} ...")
                return IIORadio(u)
            except Exception as e:
                log(f"  no: {e}")
        if transport == "iio":
            raise IOError("no libiio context reachable (plug in the board's data USB/Ethernet port)")
    log(f"falling back to debug serial console on {port}")
    return SerialRadio(port)
