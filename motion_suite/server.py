"""Human-motion radar suite: web server for one or more 2T2R radar nodes.

  py -3.11 server.py                 # boards from nodes.json (enabled ones)
  py -3.11 server.py --sim           # every enabled node simulated (no hardware)
  py -3.11 server.py --nodes A,B     # start these nodes regardless of "enabled"
  py -3.11 server.py --uri A=ip:192.168.2.1   # reach a board over its network link instead

Then open http://localhost:8790/
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import time

import tornado.ioloop
import tornado.web
import tornado.websocket

import node as N
import radio as R

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "static")
CONF = json.load(open(os.path.join(HERE, "nodes.json"), encoding="utf8"))
nodes: dict[str, N.Node] = {}
clients = set()
loop = None
LOG = []
SIM = False


def broadcast(msg):
    data = json.dumps(msg, separators=(",", ":"))

    def send():
        for c in list(clients):
            try:
                c.write_message(data)
            except Exception:
                clients.discard(c)
    if loop:
        loop.add_callback(send)


def log(s):
    line = time.strftime("%H:%M:%S ") + str(s)
    print(line, flush=True)
    LOG.append(line)
    del LOG[:-200]
    broadcast({"type": "log", "line": line})


def latest_ab():
    """Newest comparison per node, from the same kind of radio (simulated results never
    stand in for the board's)."""
    out = {}
    kind = "sim" if SIM else "board"
    for fn in sorted(glob.glob(os.path.join(N.DATA, "ab", "ab_*.json"))):
        with open(fn) as f:
            d = json.load(f)
        if d.get("kind") == kind:
            out[d["node"]] = d
    return out


def hello():
    return {"type": "hello", "room": CONF["room"], "sim": SIM,
            "nodes": [nodes[c["id"]].info() if c["id"] in nodes else {**c, "state": "off"} for c in CONF["nodes"]],
            "log": LOG[-60:], "ab": latest_ab()}


def resolve_uris(selected):
    """'auto' takes the first USB board no other node names. Both boards report the same
    serial number, so the USB bus address (usb:B.D.I) is the only way to tell them apart."""
    found = R.scan_usb()
    taken = {c["uri"] for c in selected if c.get("uri", "auto") != "auto"}
    for c in selected:
        if c.get("uri", "auto") == "auto":
            free = [u for u in found if u not in taken]
            c["uri"] = free[0] if free else "auto"
            taken.add(c["uri"])
        elif c["uri"].startswith("usb:") and c["uri"] not in found and found:
            log(f"[{c['id']}] {c['uri']} not in the USB scan {found}; check the address after re-plugging")


def start_node(cfg):
    nd = N.Node(cfg, broadcast, log, sim=SIM)
    nodes[cfg["id"]] = nd
    nd.start()


class Page(tornado.web.RequestHandler):
    def get(self):
        self.set_header("Cache-Control", "no-store")
        with open(os.path.join(STATIC, "index.html"), encoding="utf8") as f:
            self.write(f.read())


class NoCache(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        self.set_header("Cache-Control", "no-store")


class Cmd(tornado.web.RequestHandler):
    def post(self):
        d = json.loads(self.request.body or b"{}")
        nid, cmd, args = d.get("node"), d.get("cmd"), d.get("args", {})
        cfg = next((c for c in CONF["nodes"] if c["id"] == nid), None)
        if cfg is None:
            self.set_status(404)
            self.write({"ok": False, "err": "no such node"})
            return
        nd = nodes.get(nid)
        if cmd == "start":
            if nd and nd.running():
                self.write({"ok": False, "err": "already running"})
                return
            fresh = json.load(open(os.path.join(HERE, "nodes.json"), encoding="utf8"))
            keep = cfg.get("uri")
            cfg.update(next(c for c in fresh["nodes"] if c["id"] == nid))
            if keep and keep != "auto" and not keep.startswith("usb:"):
                cfg["uri"] = keep
            resolve_uris([cfg])
            start_node(dict(cfg))
        elif cmd == "stop":
            if nd:
                nd.stop()
        elif nd is None or not nd.running():
            self.write({"ok": False, "err": "node not running"})
            return
        else:
            nd.command(cmd, args)
        self.write({"ok": True})


class State(tornado.web.RequestHandler):
    def get(self):
        self.write(hello())


class WS(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        clients.add(self)
        self.write_message(json.dumps(hello(), separators=(",", ":")))

    def on_close(self):
        clients.discard(self)


def main():
    global loop, SIM
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--sim", action="store_true", help="simulate every node (no hardware)")
    ap.add_argument("--nodes", default=None, help="comma list of node ids to start, e.g. A,B")
    ap.add_argument("--uri", action="append", default=[], help="override a node's radio, e.g. --uri A=ip:192.168.2.1")
    a = ap.parse_args()
    SIM = a.sim
    N.QUIET = SIM
    want = a.nodes.split(",") if a.nodes else [c["id"] for c in CONF["nodes"] if c.get("enabled")]
    for o in a.uri:
        k, u = o.split("=", 1)
        for c in CONF["nodes"]:
            if c["id"] == k:
                c["uri"] = u
    sel = [dict(c) for c in CONF["nodes"] if c["id"] in want]
    if not SIM:
        resolve_uris(sel)
    app = tornado.web.Application([(r"/", Page), (r"/api/cmd", Cmd), (r"/api/state", State), (r"/ws", WS),
                                   (r"/static/(.*)", NoCache, {"path": STATIC})])
    app.listen(a.port)
    loop = tornado.ioloop.IOLoop.current()
    for c in sel:
        log(f"starting node {c['id']} ({'simulated' if SIM else c['uri']}) at {c['lo'] / 1e9:.3f} GHz")
        start_node(c)
    print(f"motion radar suite: http://localhost:{a.port}/", flush=True)
    try:
        loop.start()
    finally:
        for nd in nodes.values():
            nd.stop()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
