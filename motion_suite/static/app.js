"use strict";
// Motion Radar Suite page. Plain canvas, no build step. One websocket carries every node's ticks.

window.addEventListener("error", (e) => { const d = document.createElement("div"); d.className = "toast"; d.textContent = `Page error: ${e.message} (${e.lineno}:${e.colno})`; document.body.appendChild(d); setTimeout(() => d.remove(), 8000); });
const $ = (id) => document.getElementById(id);
const S = {
  nodes: {}, order: [], sel: null, room: null, sim: false, rdView: "clean",
  last: {}, tb: {}, ab: {}, abProg: {}, spec: {}, feat: {}, trails: {}, calTimer: null,
};
const WF_SHORT = { pulse: "Pulse", lfm: "LFM pulse", fmcw: "FMCW", fmcw_long: "FMCW long" };
const MIMO = ["pulse", "lfm", "fmcw", "fmcw_long"];
const WF_COL = { pulse: "#e87ba4", lfm: "#eda100", fmcw: "#3987e5", fmcw_long: "#1baf7a" };

// ---------------------------------------------------------------- colour map (inferno-like)
const LUT = (() => {
  const a = ["#000004", "#160b39", "#420a68", "#6a176e", "#932667", "#bc3754", "#dd513a", "#f37819", "#fca50a", "#f6d746", "#fcffa4"]
    .map((h) => [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16)));
  const L = new Uint8ClampedArray(256 * 3);
  for (let i = 0; i < 256; i++) {
    const x = (i / 255) * (a.length - 1), k = Math.min(a.length - 2, Math.floor(x)), f = x - k;
    for (let c = 0; c < 3; c++) L[i * 3 + c] = a[k][c] + (a[k + 1][c] - a[k][c]) * f;
  }
  return L;
})();
const b64 = (s) => { const b = atob(s), a = new Uint8Array(b.length); for (let i = 0; i < b.length; i++) a[i] = b.charCodeAt(i); return a; };
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const fmt = (x, d = 1, u = "") => (x === null || x === undefined || Number.isNaN(x) ? "–" : Number(x).toFixed(d) + u);
const deg = Math.PI / 180;

function fit(c) {
  const r = c.getBoundingClientRect(), d = window.devicePixelRatio || 1;
  const w = Math.max(10, Math.round(r.width * d)), h = Math.max(10, Math.round(r.height * d));
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  const g = c.getContext("2d");
  g.setTransform(d, 0, 0, d, 0, 0);
  return { g, w: r.width, h: r.height };
}
const INK = "rgba(255,255,255,0.78)", LINE = "rgba(255,255,255,0.16)";
function label(g, t, x, y, col = INK, align = "left", base = "alphabetic") {
  g.fillStyle = col; g.font = "11px system-ui, sans-serif"; g.textAlign = align; g.textBaseline = base; g.fillText(t, x, y);
}
function toast(t, ms = 5000) {
  const d = document.createElement("div"); d.className = "toast"; d.textContent = t; document.body.appendChild(d);
  setTimeout(() => d.remove(), ms);
}

// ---------------------------------------------------------------- server link
function cmd(c, args = {}, node = S.sel) {
  return fetch("/api/cmd", { method: "POST", body: JSON.stringify({ node, cmd: c, args }) })
    .then((r) => r.json()).then((j) => { if (!j.ok) toast(`${c}: ${j.err || "failed"}`); return j; });
}
function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { $("conn").className = "dot good"; };
  ws.onclose = () => { $("conn").className = "dot bad"; setTimeout(connect, 1500); };
  ws.onmessage = (e) => { try { onMsg(JSON.parse(e.data)); } catch (err) { console.error(err); toast(`Page error: ${err.message} @ ${(err.stack || "").split(String.fromCharCode(10))[1] || ""}`, 8000); } };
}
const CNT = {};
function onMsg(m) {
  CNT[m.type] = (CNT[m.type] || 0) + 1; document.body.dataset.msgs = JSON.stringify(CNT);
  switch (m.type) {
    case "hello":
      S.room = m.room; S.sim = m.sim; $("simBadge").hidden = !m.sim;
      S.order = m.nodes.map((n) => n.id);
      for (const n of m.nodes) S.nodes[n.id] = n;
      S.ab = m.ab || {};
      if (!S.sel || !S.nodes[S.sel]) S.sel = (m.nodes.find((n) => n.state && n.state !== "off") || m.nodes[0]).id;
      $("log").textContent = (m.log || []).join("\n");
      renderTabs(); renderControls(); renderAB(); drawRoom();
      break;
    case "status":
      S.nodes[m.node.id] = m.node; renderTabs(); if (m.node.id === S.sel) renderControls();
      break;
    case "tick":
      S.last[m.node] = m; pushSpec(m);
      if (m.target) { (S.trails[m.node] ||= []).push(m.target); if (S.trails[m.node].length > 40) S.trails[m.node].shift(); }
      else if (S.trails[m.node]?.length) S.trails[m.node].shift();
      if (m.node === S.sel) renderTick(m);
      drawRoom();
      break;
    case "txbeam":
      S.tb[m.node] = m; if (m.node === S.sel) drawTB(m);
      break;
    case "ab":
      S.abProg[m.node] = m.ab; renderABProg();
      break;
    case "ab_result":
      S.ab[m.node] = m.result; S.abProg[m.node] = null; renderAB(); renderABProg();
      toast(`Comparison finished on ${m.node}: best = ${WF_SHORT[m.result.best] || m.result.best}`, 8000);
      break;
    case "cal":
      toast(m.ok ? `Calibrated ${m.node}: phases ${m.deg.map((d) => d.toFixed(0)).join(", ")}°, point-target quality ${m.q.toFixed(2)}`
        : `Calibration failed on ${m.node}: only ${m.n} target cells. Walk along the centre line, 2–5 m, arms still.`, 9000);
      break;
    case "log": {
      const el = $("log"); el.textContent += (el.textContent ? "\n" : "") + m.line; el.scrollTop = el.scrollHeight;
      if (/ERROR/.test(m.line)) toast(m.line, 8000);
      break;
    }
  }
}

// ---------------------------------------------------------------- tabs and controls
function renderTabs() {
  const box = $("nodeTabs"); box.innerHTML = "";
  for (const id of S.order) {
    const n = S.nodes[id], b = document.createElement("button");
    const st = n.state || "off";
    const dot = st === "running" ? "good" : st === "error" ? "bad" : st === "starting" ? "warn" : "";
    b.className = "tab" + (id === S.sel ? " on" : "");
    b.innerHTML = `<span class="dot ${dot}"></span>${n.name || id}<span class="meta">${st === "running" ? `${n.kind} · ${fmt(n.rate, 0)}/s · ADC ${fmt(n.adc_peak, 1)} dBFS` : st}</span>`;
    b.title = n.err || n.uri || "";
    b.onclick = () => { S.sel = id; renderTabs(); renderControls(); renderAB(); renderABProg(); const m = S.last[id]; if (m) renderTick(m); redrawSpec(); };
    box.appendChild(b);
  }
}
function renderControls() {
  const n = S.nodes[S.sel]; if (!n) return;
  const running = n.state === "running" || n.state === "starting";
  const seg = $("wfSeg"); seg.innerHTML = "";
  for (const k of MIMO) {
    const d = n.waveforms?.[k], b = document.createElement("button");
    b.textContent = WF_SHORT[k]; b.className = n.wf === k && n.mode === "live" ? "on" : "";
    b.disabled = !running || n.mode !== "live";
    if (d) b.title = `${d.label}\nTX on ${d.on_us} us per frame (duty ${(d.duty * 100).toFixed(1)} %), energy ${d.energy_db} dB\n${d.tx_while_rx ? "transmits while receiving" : "silent while echoes return"}`;
    b.onclick = () => cmd("wf", { key: k });
    seg.appendChild(b);
  }
  for (const b of $("modeSeg").children) { b.className = n.mode === b.dataset.mode ? "on" : ""; b.disabled = !running; }
  if (n.gates && document.activeElement?.tagName !== "INPUT") {
    $("gL0").value = n.gates.local[0]; $("gL1").value = n.gates.local[1]; $("gB0").value = n.gates.back[0]; $("gB1").value = n.gates.back[1];
  }
  $("gLtxt").textContent = n.gates ? `${n.gates.local[0]}–${n.gates.local[1]} m` : "";
  $("gBtxt").textContent = n.gates ? `${n.gates.back[0]}–${n.gates.back[1]} m` : "";
  $("bFlip").className = n.flip ? "on" : "";
  $("bStart").textContent = running ? "Stop node" : "Start node";
  $("bCal").title = n.cal ? `Calibrated: ${n.cal.deg.join(", ")}° (quality ${n.cal.q})` : "Not calibrated: walk slowly toward/away along the centre line, 2–5 m";
  $("bCal").textContent = S.calTimer ? $("bCal").textContent : n.cal ? "Recalibrate angle" : "Calibrate angle";
  const gm = n.geom;
  $("raHint").textContent = (n.cal ? `calibrated (q ${n.cal.q})` : "uncalibrated") + (gm && !gm.measured ? " · antenna layout NOT measured" : "");
  $("raHint").style.color = gm && !gm.measured ? css("--warn") : "";
}
$("modeSeg").onclick = (e) => {
  const b = e.target.closest("button"); if (!b) return;
  if (b.dataset.mode === "ab") { $("bAB").scrollIntoView({ behavior: "smooth", block: "center" }); startAB(); return; }
  cmd("mode", { mode: b.dataset.mode });
};
$("rdView").onclick = (e) => {
  const b = e.target.closest("button"); if (!b) return;
  S.rdView = b.dataset.v; for (const x of $("rdView").children) x.className = x === b ? "on" : "";
  const m = S.last[S.sel]; if (m) drawRD(m);
};
$("gApply").onclick = () => cmd("gates", { local: [+$("gL0").value, +$("gL1").value], back: [+$("gB0").value, +$("gB1").value] });
$("bRelearn").onclick = () => { cmd("relearn"); toast("Re-learning the background: keep the area still for 2 s."); };
$("bFlip").onclick = () => cmd("flip", { on: !S.nodes[S.sel]?.flip });
$("bStart").onclick = () => { const n = S.nodes[S.sel]; cmd(n && (n.state === "running" || n.state === "starting") ? "stop" : "start"); };
$("bCal").onclick = () => {
  const secs = 12;
  cmd("calibrate", { seconds: secs, theta: 0 }).then((j) => {
    if (!j.ok) return;
    toast("Calibration: walk slowly toward and away from the radar along its centre line, between 2 and 5 m, arms still.", 9000);
    let left = secs; clearInterval(S.calTimer);
    S.calTimer = setInterval(() => { $("bCal").textContent = `Calibrating… ${left--} s`; if (left < 0) { clearInterval(S.calTimer); S.calTimer = null; renderControls(); } }, 1000);
  });
};
function startAB() {
  cmd("ab_start", { keys: MIMO, dwell: +$("abDwell").value || 6, rounds: +$("abRounds").value || 3, learn: 4 });
}
$("bAB").onclick = startAB;
$("bABstop").onclick = () => cmd("ab_stop");

// ---------------------------------------------------------------- per-tick rendering
function renderTick(m) {
  drawRD(m); drawRA(m); drawTarget(m); drawBudget(m); drawBP(m); redrawSpec();
  const n = S.nodes[S.sel];
  if (n && n.mode !== "txbeam") { const tb = S.tb[S.sel]; if (!tb) drawTBIdle(); }
}

function plotBox(w, h, l = 40, b = 26, t = 10, r = 10) { return { x0: l, y0: t, x1: w - r, y1: h - b, W: w - l - r, H: h - t - b }; }

const off = document.createElement("canvas");
function heat(u8, cols, rows, flipY = true) {
  off.width = cols; off.height = rows;
  const g = off.getContext("2d"), im = g.createImageData(cols, rows);
  for (let y = 0; y < rows; y++) {
    const sr = flipY ? rows - 1 - y : y;
    for (let x = 0; x < cols; x++) {
      const v = u8[sr * cols + x], o = (y * cols + x) * 4;
      im.data[o] = LUT[v * 3]; im.data[o + 1] = LUT[v * 3 + 1]; im.data[o + 2] = LUT[v * 3 + 2]; im.data[o + 3] = 255;
    }
  }
  g.putImageData(im, 0, 0);
  return off;
}

function drawRD(m) {
  const { g, w, h } = fit($("cRD")), P = plotBox(w, h);
  g.fillStyle = "#0e0e10"; g.fillRect(0, 0, w, h);
  const [r0, r1, NR] = m.r_axis, [v0, v1, NV] = m.v_axis;
  const raw = S.rdView === "raw";
  const u8 = b64(raw ? m.rd_raw : m.rd);
  g.imageSmoothingEnabled = true;
  g.drawImage(heat(u8, NV, NR), P.x0, P.y0, P.W, P.H);
  const X = (v) => P.x0 + ((v - v0) / (v1 - v0)) * P.W, Y = (r) => P.y1 - ((r - r0) / (r1 - r0)) * P.H;
  // grid + axes
  g.strokeStyle = LINE; g.lineWidth = 1;
  for (let v = -4; v <= 4; v += 2) { g.beginPath(); g.moveTo(X(v), P.y0); g.lineTo(X(v), P.y1); g.stroke(); label(g, `${v}`, X(v), h - 10, INK, "center"); }
  for (let r = 0; r <= r1; r += 2) { g.beginPath(); g.moveTo(P.x0, Y(r)); g.lineTo(P.x1, Y(r)); g.stroke(); label(g, `${r}`, P.x0 - 6, Y(r), INK, "right", "middle"); }
  label(g, "m/s (+ away)", P.x1, h - 10, INK, "right"); label(g, "m", 6, P.y0 + 8);
  // ambiguity limit
  if (m.v_amb < v1) {
    g.setLineDash([4, 4]); g.strokeStyle = "rgba(255,255,255,.5)";
    for (const s of [-1, 1]) { g.beginPath(); g.moveTo(X(s * m.v_amb), P.y0); g.lineTo(X(s * m.v_amb), P.y1); g.stroke(); }
    g.setLineDash([]);
  }
  // gates on the range axis
  const n = S.nodes[m.node];
  if (n?.gates) for (const [k, col] of [["local", css("--local")], ["back", css("--back")]]) {
    const [a, b] = n.gates[k], ya = Y(Math.max(a, r0)), yb = Y(Math.min(b, r1));
    g.fillStyle = col; g.fillRect(P.x0 - 4, yb, 3, ya - yb);
    g.strokeStyle = col; g.globalAlpha = 0.55; g.setLineDash([2, 4]);
    g.beginPath(); g.moveTo(P.x0, yb); g.lineTo(P.x1, yb); g.stroke(); g.setLineDash([]); g.globalAlpha = 1;
  }
  if (S.sim && m.truth && m.truth.mode !== "out" && m.truth.r <= r1) diamond(g, X(m.truth.v_radial), Y(m.truth.r));
  if (m.target) ring(g, X(m.target.v), Y(m.target.r), "#fff");
  $("rdBanner").hidden = !m.learning;
  $("rdBanner").textContent = "Learning the empty-scene background: keep the area still";
  $("rdFoot").innerHTML = raw
    ? `Raw: static removed only (no phase-noise lock, no whitening), dB over the far noise floor, 0–60 dB. PRF ${fmt(m.prf, 0)} Hz.`
    : `Cleaned: phase-noise lock → static removal → whitened by the learned empty scene. dB over background, 0–40 dB. PRF ${fmt(m.prf, 0)} Hz, unambiguous ±${fmt(m.v_amb, 1)} m/s${m.bg_frozen ? " · background frozen" : ""}.${S.sim ? " ◇ = simulated truth." : ""}`;
}
function ring(g, x, y, col) { g.strokeStyle = col; g.lineWidth = 2; g.beginPath(); g.arc(x, y, 7, 0, 7); g.stroke(); g.lineWidth = 1; }
function diamond(g, x, y) {
  g.strokeStyle = "#7ef0c4"; g.lineWidth = 1.5; g.beginPath(); g.moveTo(x, y - 6); g.lineTo(x + 6, y); g.lineTo(x, y + 6); g.lineTo(x - 6, y); g.closePath(); g.stroke(); g.lineWidth = 1;
}

// range-angle fan: per-pixel lookup into (range, angle), computed at half resolution
const fan = { key: "", idx: null, w: 0, h: 0 };
function drawRA(m) {
  const c = $("cRA"), { g, w, h } = fit(c);
  g.fillStyle = "#0e0e10"; g.fillRect(0, 0, w, h);
  const [r0, r1, NR] = m.r_axis, NT = m.n_th;
  const ox = w / 2, oy = h - 18, R = Math.min(w / 2 - 14, h - 30), sc = R / r1;
  const iw = Math.round(w / 2), ih = Math.round(h / 2), key = `${iw}x${ih}:${r0}:${r1}:${NR}:${NT}`;
  if (fan.key !== key) {
    fan.key = key; fan.w = iw; fan.h = ih; fan.idx = new Int32Array(iw * ih);
    for (let y = 0; y < ih; y++) for (let x = 0; x < iw; x++) {
      const dx = (x * 2 - ox) / sc, dy = (oy - y * 2) / sc, r = Math.hypot(dx, dy), th = Math.atan2(dx, dy) / deg;
      let k = -1;
      if (dy >= 0 && r >= r0 && r <= r1 && Math.abs(th) <= 90) {
        const ri = Math.round(((r - r0) / (r1 - r0)) * (NR - 1)), ti = Math.round(((th + 90) / 180) * (NT - 1));
        k = ri * NT + ti;
      }
      fan.idx[y * iw + x] = k;
    }
  }
  const u8 = b64(m.ra);
  off.width = iw; off.height = ih;
  const og = off.getContext("2d"), im = og.createImageData(iw, ih);
  for (let i = 0; i < iw * ih; i++) {
    const k = fan.idx[i], o = i * 4;
    if (k < 0) { im.data[o + 3] = 0; continue; }
    const v = u8[k];
    im.data[o] = LUT[v * 3]; im.data[o + 1] = LUT[v * 3 + 1]; im.data[o + 2] = LUT[v * 3 + 2]; im.data[o + 3] = 255;
  }
  og.putImageData(im, 0, 0);
  g.imageSmoothingEnabled = true; g.drawImage(off, 0, 0, w, h);
  const P = (r, th) => [ox + r * sc * Math.sin(th * deg), oy - r * sc * Math.cos(th * deg)];
  g.strokeStyle = LINE;
  for (let r = 2; r <= r1; r += 2) { g.beginPath(); g.arc(ox, oy, r * sc, Math.PI, 2 * Math.PI); g.stroke(); const [x, y] = P(r, 0); label(g, `${r} m`, x + 3, y - 3); }
  for (const th of [-60, -30, 0, 30, 60]) { const [x, y] = P(r1, th); g.beginPath(); g.moveTo(ox, oy); g.lineTo(x, y); g.stroke(); const [lx, ly] = P(r1 * 1.0, th); label(g, `${th}°`, lx, ly - 4, INK, "center"); }
  const ua = m.unamb_deg;
  g.setLineDash([5, 4]); g.strokeStyle = "rgba(255,255,255,.55)";
  for (const s of [-1, 1]) { const [x, y] = P(r1, s * ua); g.beginPath(); g.moveTo(ox, oy); g.lineTo(x, y); g.stroke(); }
  g.setLineDash([]);
  // steered beam lobe (array factor), drawn on the fan so the beam is visible
  if (m.target?.af) {
    g.strokeStyle = "rgba(126,240,196,.9)"; g.lineWidth = 1.5; g.beginPath();
    m.target.af.forEach((a, i) => { const th = -90 + i * 0.5, [x, y] = P(a * r1, th); i ? g.lineTo(x, y) : g.moveTo(x, y); });
    g.stroke(); g.lineWidth = 1;
  }
  if (m.far_m && m.far_m < r1) {   // inside this radius the array is in its near field (exact-path steering handles it)
    g.setLineDash([2, 5]); g.strokeStyle = "rgba(126,240,196,.45)"; g.beginPath(); g.arc(ox, oy, m.far_m * sc, Math.PI, 2 * Math.PI); g.stroke(); g.setLineDash([]);
    const [x, y] = P(m.far_m, -75); label(g, "near field", x, y - 3, "rgba(126,240,196,.7)");
  }
  drawArray(g, ox, oy);
  if (S.sim && m.truth && m.truth.mode !== "out" && m.truth.r <= r1) { const [x, y] = P(m.truth.r, m.truth.theta_deg * (S.nodes[m.node]?.flip ? -1 : 1)); diamond(g, x, y); }
  if (m.target) { const [x, y] = P(m.target.r, m.target.theta); ring(g, x, y, "#fff"); }
}
function drawArray(g, ox, oy) {
  const cols = ["#3987e5", "#3987e5", "#e8a530", "#e8a530"];
  for (let i = 0; i < 4; i++) { g.fillStyle = cols[i]; g.fillRect(ox - 15 + i * 8, oy + 4, 6, 5); }
  label(g, "TX TX RX RX", ox, oy + 16, "rgba(255,255,255,.55)", "center");
}

function drawTarget(m) {
  const t = m.target, n = S.nodes[m.node];
  const rows = [];
  if (t) {
    rows.push(["Range", `${fmt(t.r, 2)} m`, true], ["Angle", `${fmt(t.theta, 1)}°`, true], ["Radial velocity", `${fmt(t.v, 2)} m/s`],
      ["SNR over background", `${fmt(t.snr, 1)} dB`], ["Beamforming gain", `${fmt(t.gain_db, 2)} dB of 6.02`], ["Point-like (coherence)", fmt(t.coh * 100, 0, " %")],
      ["Cells in target", t.n_cells]);
  } else rows.push(["Range", "no motion", true], ["Angle", "–", true]);
  if (S.sim && m.truth) rows.push(["Sim truth", m.truth.mode === "out" ? "outside" : `${fmt(m.truth.r, 2)} m, ${fmt(m.truth.theta_deg, 1)}°, ${m.truth.mode}`]);
  if (n?.cal === null) rows.push(["Angle calibration", "none yet"]);
  $("tgt").innerHTML = rows.map(([k, v, big]) => `<div class="k">${k}</div><div class="v${big ? " big" : ""}">${v}</div>`).join("");
  const z = $("zoneTag");
  z.textContent = t ? (t.zone === "other" ? "outside gates" : `${t.zone} zone`) : "";
  z.className = "tag " + (t?.zone || "");
}
function drawBudget(m) {
  const b = m.budget; if (!b) return;
  const rows = [["Raw (leakage)", b.static], ["Static removed", b.no_lock], ["+ phase-noise lock", b.locked], ["Noise floor (far)", b.far]];
  const lo = -100, hi = 0;
  $("budget").innerHTML = rows.map(([k, v]) => `<div class="row"><span>${k}</span><div class="bar"><i style="width:${Math.max(0, Math.min(100, ((v - lo) / (hi - lo)) * 100))}%"></i></div><span class="n">${fmt(v, 1)}</span></div>`).join("")
    + `<div class="note">dBFS per cell. Total clean-up ${fmt(b.static - b.locked, 0)} dB; ${fmt(b.locked - b.far, 0)} dB of residual stays above noise near the antennas, which the background whitening absorbs.</div>`;
}

// ---------------------------------------------------------------- spectrograms
const NCOL = 120;
function specBuf(node, g) {
  const k = node + g;
  if (!S.spec[k]) { const c = document.createElement("canvas"); c.width = NCOL; c.height = 128; const x = c.getContext("2d"); x.fillStyle = "#000004"; x.fillRect(0, 0, NCOL, 128); S.spec[k] = c; S.feat[k] = []; }
  return S.spec[k];
}
function pushSpec(m) {
  for (const gname of ["local", "back"]) {
    const col = m.spec?.[gname]; if (!col) continue;
    const c = specBuf(m.node, gname), x = c.getContext("2d");
    x.drawImage(c, -1, 0);
    const u8 = b64(col), NV = u8.length, im = x.createImageData(1, NV);
    for (let i = 0; i < NV; i++) { const v = u8[NV - 1 - i], o = i * 4; im.data[o] = LUT[v * 3]; im.data[o + 1] = LUT[v * 3 + 1]; im.data[o + 2] = LUT[v * 3 + 2]; im.data[o + 3] = 255; }
    x.putImageData(im, NCOL - 1, 0);
    const F = S.feat[m.node + gname]; F.push(m.feats?.[gname] || {}); if (F.length > NCOL) F.shift();
  }
}
function redrawSpec() {
  const m = S.last[S.sel]; if (!m) return;
  for (const [gname, cid, fid, aid] of [["local", "cSL", "fL", "actL"], ["back", "cSB", "fB", "actB"]]) {
    const { g, w, h } = fit($(cid)), P = plotBox(w, h, 34, 20, 6, 8);
    g.fillStyle = "#0e0e10"; g.fillRect(0, 0, w, h);
    g.imageSmoothingEnabled = true; g.drawImage(specBuf(S.sel, gname), P.x0, P.y0, P.W, P.H);
    const V = 5, Y = (v) => P.y0 + ((V - v) / (2 * V)) * P.H, X = (i) => P.x0 + ((i + 0.5) / NCOL) * P.W;
    g.strokeStyle = LINE;
    for (let v = -4; v <= 4; v += 2) { g.beginPath(); g.moveTo(P.x0, Y(v)); g.lineTo(P.x1, Y(v)); g.stroke(); label(g, `${v}`, P.x0 - 5, Y(v), INK, "right", "middle"); }
    for (let s = 0; s <= 12; s += 2) label(g, s === 0 ? "now" : `-${s}s`, P.x1 - (s / 12) * P.W, h - 5, INK, "center");
    const F = S.feat[S.sel + gname] || [], off0 = NCOL - F.length;
    const line = (key, col, wdt) => {
      g.strokeStyle = col; g.lineWidth = wdt; g.beginPath(); let on = false;
      F.forEach((f, i) => { const v = f[key]; if (v === null || v === undefined) { on = false; return; } const x = X(i + off0), y = Y(v); on ? g.lineTo(x, y) : g.moveTo(x, y); on = true; });
      g.stroke(); g.lineWidth = 1;
    };
    line("hi", "rgba(255,255,255,.35)", 1); line("lo", "rgba(255,255,255,.35)", 1);
    g.fillStyle = "rgba(126,240,196,.95)";
    F.forEach((f, i) => { if (f.torso !== null && f.torso !== undefined) { g.beginPath(); g.arc(X(i + off0), Y(f.torso), 1.8, 0, 7); g.fill(); } });
    const f = m.feats?.[gname] || {};
    $(aid).textContent = f.activity || ""; $(aid).style.color = f.active ? css(gname === "local" ? "--local" : "--back") : css("--ink3");
    $(fid).innerHTML = `<span>torso</span><b>${fmt(f.torso, 2, " m/s")}</b><span>limb envelope</span><b>${fmt(f.lo, 1)} … ${fmt(f.hi, 1)} m/s</b>`
      + `<span>micro-Doppler spread</span><b>${fmt(f.spread, 1, " m/s")}</b><span>envelope period</span><b>${f.cadence_hz ? `${fmt(f.cadence_hz, 2)} Hz` : "–"}</b>`
      + `<span style="flex-basis:100%">green dots: strongest (torso) velocity · grey: limb envelope above 8 dB · y: m/s, + away · range resolution is 3 m, so keep the gates ≥ 1.5 m apart or they share the same person</span>`;
  }
}

// ---------------------------------------------------------------- beam on target (polar half-plane)
function polarFrame(g, w, h, ua) {
  const ox = w / 2, oy = h - 16, R = Math.min(w / 2 - 14, h - 28);
  g.strokeStyle = LINE;
  for (const f of [0.25, 0.5, 0.75, 1]) { g.beginPath(); g.arc(ox, oy, R * f, Math.PI, 2 * Math.PI); g.stroke(); }
  for (const th of [-90, -60, -30, 0, 30, 60, 90]) {
    const x = ox + R * Math.sin(th * deg), y = oy - R * Math.cos(th * deg);
    g.beginPath(); g.moveTo(ox, oy); g.lineTo(x, y); g.stroke(); label(g, `${th}°`, ox + (R + 2) * Math.sin(th * deg), oy - (R + 4) * Math.cos(th * deg), INK, "center");
  }
  if (ua) { g.setLineDash([5, 4]); g.strokeStyle = "rgba(255,255,255,.5)"; for (const s of [-1, 1]) { g.beginPath(); g.moveTo(ox, oy); g.lineTo(ox + R * Math.sin(s * ua * deg), oy - R * Math.cos(s * ua * deg)); g.stroke(); } g.setLineDash([]); }
  return { ox, oy, R };
}
function polarLine(g, F, vals, col, dash = [], fill = null) {
  g.strokeStyle = col; g.lineWidth = 1.8; g.setLineDash(dash); g.beginPath();
  const n = vals.length;
  vals.forEach((a, i) => { const th = (-90 + (180 * i) / (n - 1)) * deg, x = F.ox + a * F.R * Math.sin(th), y = F.oy - a * F.R * Math.cos(th); i ? g.lineTo(x, y) : g.moveTo(x, y); });
  if (fill) { g.fillStyle = fill; g.fill(); }
  g.stroke(); g.setLineDash([]); g.lineWidth = 1;
}
function drawBP(m) {
  const { g, w, h } = fit($("cBP"));
  g.fillStyle = "#0e0e10"; g.fillRect(0, 0, w, h);
  const F = polarFrame(g, w, h, m.unamb_deg);
  const t = m.target;
  if (!t) { label(g, "no moving target", F.ox, F.oy - F.R / 2, "rgba(255,255,255,.5)", "center"); $("bpFoot").textContent = "Waiting for motion."; return; }
  const mx = Math.max(...t.spec, 1e-9);
  polarLine(g, F, t.spec.map((v) => v / mx), "#fca50a", [], "rgba(252,165,10,.18)");
  polarLine(g, F, t.af, "rgba(126,240,196,.95)", [5, 4]);
  const x = F.ox + F.R * Math.sin(t.theta * deg), y = F.oy - F.R * Math.cos(t.theta * deg);
  g.strokeStyle = "#fff"; g.beginPath(); g.moveTo(F.ox, F.oy); g.lineTo(x, y); g.stroke();
  drawArray(g, F.ox, F.oy);
  $("bpFoot").innerHTML = `Steered to <b>${fmt(t.theta, 1)}°</b> · gain <b>${fmt(t.gain_db, 2)} dB</b> of 6.02 · coherence ${fmt(t.coh * 100, 0)} %. `
    + `Orange: measured spatial spectrum; green dashed: the steered beam (array factor). Lobes outside the dashed wedge are grating lobes: element pitch 50.8 mm ≈ 1 λ.`;
}

// ---------------------------------------------------------------- physical TX beams
function drawTBIdle() {
  const { g, w, h } = fit($("cTB")); g.fillStyle = "#0e0e10"; g.fillRect(0, 0, w, h);
  const F = polarFrame(g, w, h, null); drawArray(g, F.ox, F.oy);
  label(g, "TX beam scan is off", F.ox, F.oy - F.R / 2, "rgba(255,255,255,.55)", "center");
  const c2 = fit($("cTB2")); c2.g.fillStyle = "#0e0e10"; c2.g.fillRect(0, 0, c2.w, c2.h);
}
function drawTB(m) {
  const n = S.nodes[m.node];
  if (n && n.mode !== "txbeam") { drawTBIdle(); return; }
  $("rdBanner").hidden = false; $("rdBanner").textContent = "TX beam scan running: MIMO views pause (see Physical TX beams)";
  const { g, w, h } = fit($("cTB")); g.fillStyle = "#0e0e10"; g.fillRect(0, 0, w, h);
  const F = polarFrame(g, w, h, null);
  if (m.learning) { label(g, "learning background…", F.ox, F.oy - F.R / 2, "rgba(255,255,255,.6)", "center"); return; }
  const sgn = m.flip ? -1 : 1, psi = m.psi_deg * deg;
  const lin = m.meas.map((d) => Math.pow(10, d / 10));
  const order = lin.map((v, i) => i).sort((a, b) => lin[a] - lin[b]);
  for (const i of order) {
    const ph = m.phases_deg[i] * deg, vals = [];
    for (let k = 0; k <= 180; k++) { const th = (-90 + k) * deg * sgn, a = ph + psi + m.kd * Math.sin(th); vals.push(((1 + Math.cos(a)) / 2) * lin[i]); }
    const best = i === m.best;
    const hue = Math.round(260 - 220 * lin[i]);
    polarLine(g, F, vals, best ? "#ffffff" : `hsla(${hue},85%,60%,${0.25 + 0.7 * lin[i]})`, [], `hsla(${hue},85%,55%,${0.05 + 0.25 * lin[i]})`);
  }
  if (m.present && m.angle_fit !== null) {
    const x = F.ox + F.R * Math.sin(m.angle_fit * deg), y = F.oy - F.R * Math.cos(m.angle_fit * deg);
    g.strokeStyle = "#7ef0c4"; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(F.ox, F.oy); g.lineTo(x, y); g.stroke(); g.setLineDash([]);
  }
  drawArray(g, F.ox, F.oy);
  // bars: measured (per beam) vs predicted from the single-TX slots
  const c2 = fit($("cTB2")), g2 = c2.g, P = plotBox(c2.w, c2.h, 34, 20, 8, 8);
  g2.fillStyle = "#0e0e10"; g2.fillRect(0, 0, c2.w, c2.h);
  const lo = -25, Y = (d) => P.y0 + (Math.max(lo, d) / lo) * P.H, bw = P.W / 12;
  g2.strokeStyle = LINE; for (const d of [0, -10, -20]) { g2.beginPath(); g2.moveTo(P.x0, Y(d)); g2.lineTo(P.x1, Y(d)); g2.stroke(); label(g2, `${d}`, P.x0 - 5, Y(d), INK, "right", "middle"); }
  m.meas.forEach((d, i) => { g2.fillStyle = i === m.best ? "#fff" : "#fca50a"; g2.fillRect(P.x0 + i * bw + 3, Y(d), bw - 6, P.y1 - Y(d)); label(g2, `${m.angles[i].toFixed(0)}°`, P.x0 + (i + 0.5) * bw, c2.h - 5, INK, "center"); });
  g2.strokeStyle = "#7ef0c4"; g2.lineWidth = 1.6; g2.beginPath();
  m.pred.forEach((d, i) => { const x = P.x0 + (i + 0.5) * bw, y = Y(d); i ? g2.lineTo(x, y) : g2.moveTo(x, y); }); g2.stroke(); g2.lineWidth = 1;
  $("tbHint").textContent = m.present ? `target at ${fmt(m.r, 1)} m · ${fmt(m.snr_db, 0)} dB` : "no clear target";
  $("tbFoot").innerHTML = `Target direction from the 12 beams: <b>${fmt(m.angle_fit, 1)}°</b> (single-TX prediction ${fmt(m.angle_mimo, 1)}°). `
    + `Best beam vs one TX: <b>${fmt(m.gain_db, 1)} dB</b> (ideal 6: 3 dB from the second transmitter's power, 3 dB from coherent addition). `
    + `Worst beam ${fmt(m.null_db, 1)} dB below best. Measured pattern vs prediction from the single-TX slots: r = ${fmt(m.corr, 3)}. Bars: measured per beam (dB); green: predicted. PRF ${fmt(m.prf, 0)} Hz.`;
}

// ---------------------------------------------------------------- room / fusion
function drawRoom() {
  const c = $("cRoom"); if (!S.room) return;
  const { g, w, h } = fit(c); g.fillStyle = "#0e0e10"; g.fillRect(0, 0, w, h);
  const R = S.room, pad = 18, sx = (w - 2 * pad) / (R.x1 - R.x0), sy = (h - 2 * pad) / (R.y1 - R.y0), s = Math.min(sx, sy);
  const cx = (w - (R.x1 - R.x0) * s) / 2, cy = (h - (R.y1 - R.y0) * s) / 2;
  const X = (x) => cx + (x - R.x0) * s, Y = (y) => h - cy - (y - R.y0) * s;
  g.strokeStyle = LINE;
  for (let x = Math.ceil(R.x0); x <= R.x1; x++) { g.beginPath(); g.moveTo(X(x), Y(R.y0)); g.lineTo(X(x), Y(R.y1)); g.stroke(); }
  for (let y = Math.ceil(R.y0); y <= R.y1; y++) { g.beginPath(); g.moveTo(X(R.x0), Y(y)); g.lineTo(X(R.x1), Y(y)); g.stroke(); label(g, `${y}`, X(R.x0) - 4, Y(y), "rgba(255,255,255,.45)", "right", "middle"); }
  g.strokeStyle = "rgba(255,255,255,.6)"; g.strokeRect(X(R.x0), Y(R.y1), (R.x1 - R.x0) * s, (R.y1 - R.y0) * s);
  const fused = [];
  for (const id of S.order) {
    const n = S.nodes[id], p = n.pose || { x: 0, y: 0, heading: 0 }, col = n.color || "#3987e5";
    const live = n.state === "running";
    const hd = p.heading * deg, ua = (n.unamb_deg || 30) * deg, rr = Math.min(n.r_max || 12, 30);
    const W2 = (r, a) => [X(p.x + r * Math.sin(hd + a)), Y(p.y + r * Math.cos(hd + a))];
    g.globalAlpha = live ? 1 : 0.35;
    g.fillStyle = col + "22"; g.beginPath(); g.moveTo(X(p.x), Y(p.y));
    for (let a = -ua; a <= ua + 1e-6; a += ua / 12) { const [x, y] = W2(rr, a); g.lineTo(x, y); }
    g.closePath(); g.fill();
    g.save(); g.translate(X(p.x), Y(p.y)); g.rotate(hd); g.fillStyle = col; g.fillRect(-12, -3, 24, 6); g.restore();
    label(g, `${id}${live ? "" : " (off)"}`, X(p.x) + 10, Y(p.y) - 8, col);
    g.globalAlpha = 1;
    const tr = S.trails[id] || [];
    const toW = (t) => [p.x + t.r * Math.sin(hd + t.theta * deg), p.y + t.r * Math.cos(hd + t.theta * deg)];
    g.strokeStyle = col; g.globalAlpha = 0.6; g.beginPath();
    tr.forEach((t, i) => { const [x, y] = toW({ r: t.r_trk ?? t.r, theta: t.th_trk ?? t.theta }); i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)); });
    g.stroke(); g.globalAlpha = 1;
    const m = S.last[id];
    if (live && m?.target) {
      const [x, y] = toW({ r: m.target.r_trk ?? m.target.r, theta: m.target.th_trk ?? m.target.theta });
      g.fillStyle = col; g.beginPath(); g.arc(X(x), Y(y), 6, 0, 7); g.fill(); fused.push([x, y]);
    }
    if (live && S.sim && m?.truth && m.truth.mode !== "out") {
      const th = m.truth.theta_deg * (n.flip ? -1 : 1);
      const [x, y] = toW({ r: m.truth.r, theta: th }); diamond(g, X(x), Y(y));
    }
  }
  if (fused.length >= 2) {
    const [a, b] = fused;
    if (Math.hypot(a[0] - b[0], a[1] - b[1]) < 1.5) { const x = (a[0] + b[0]) / 2, y = (a[1] + b[1]) / 2; g.strokeStyle = "#fff"; g.lineWidth = 2; g.beginPath(); g.arc(X(x), Y(y), 10, 0, 7); g.stroke(); g.lineWidth = 1; }
  }
}

// ---------------------------------------------------------------- waveform comparison
function renderABProg() {
  const p = S.abProg[S.sel], el = $("abProg");
  el.hidden = !p;
  if (!p) return;
  el.querySelector("i").style.width = `${(100 * p.step) / p.n}%`;
  const what = p.phase === "learn" ? `learning ${WF_SHORT[p.key]} (keep the area empty)` : p.phase === "prep" ? "get ready to move" : `scoring ${WF_SHORT[p.key]} (keep moving)`;
  el.querySelector("span").textContent = `step ${p.step}/${p.n}: ${what} · ${p.left} s left`;
}
function renderAB() {
  const r = S.ab[S.sel], box = $("abResult");
  if (!r) { box.innerHTML = `<p class="dim">No comparison on this node yet.</p>`; return; }
  const rows = [...r.rows].sort((a, b) => (b.det_rate * 20 + (b.snr_db || 0)) - (a.det_rate * 20 + (a.snr_db || 0)));
  const best = rows[0], next = rows[1];
  const pct = (x) => (x === null ? "–" : `${Math.round(x * 100)} %`);
  let html = `<div class="verdict">Best here: <b>${best.label}</b>, detected in ${pct(best.det_rate)} of moving ticks at a median ${fmt(best.snr_db, 1)} dB over background`
    + (next ? `; next ${WF_SHORT[next.key]} at ${pct(next.det_rate)} / ${fmt(next.snr_db, 1)} dB.` : ".")
    + ` <span class="dim">${r.kind === "sim" ? "SIMULATED run · " : ""}${r.when} · dwell ${r.settings.dwell} s × ${r.settings.rounds} rounds</span></div>`;
  html += `<div class="scroll"><table><tr><th>Waveform</th><th>TX on / frame</th><th>duty</th><th>energy</th><th>PRF</th><th>±v max</th><th>noise floor</th><th>near residual</th><th>detected</th><th>SCNR</th><th>local peak</th><th>back peak</th><th>local µD spread</th><th>coherence</th></tr>`;
  for (const x of rows) html += `<tr class="${x.key === best.key ? "best" : ""}"><td>${x.label}</td><td>${fmt(x.on_us, 2)} µs</td><td>${fmt(x.duty * 100, 1)} %</td><td>${fmt(x.energy_db, 1)} dB</td><td>${fmt(x.prf, 0)} Hz</td><td>${fmt(x.v_amb, 1)} m/s</td>`
    + `<td>${fmt(x.noise_dbfs, 1)} dBFS</td><td>${fmt(x.near_resid_db, 1)} dB</td><td>${pct(x.det_rate)}</td><td>${fmt(x.snr_db, 1)} dB</td><td>${fmt(x.snr_local_db, 1)} dB</td><td>${fmt(x.snr_back_db, 1)} dB</td><td>${fmt(x.spread_local, 1)} m/s</td><td>${x.coh === null ? "–" : fmt(x.coh * 100, 0) + " %"}</td></tr>`;
  html += `</table></div>`;
  const bar = (title, key, unit, mx) => `<div><div class="bt">${title}</div>` + r.rows.map((x) => {
    const v = x[key]; const wd = v === null ? 0 : Math.max(0, Math.min(100, (v / mx) * 100));
    return `<div class="r"><span>${WF_SHORT[x.key]}</span><div class="b"><i style="width:${wd}%;background:${WF_COL[x.key]}"></i></div><span>${fmt(v, 1)}${unit}</span></div>`;
  }).join("") + `</div>`;
  const mx = Math.max(10, ...r.rows.flatMap((x) => [x.snr_local_db || 0, x.snr_back_db || 0]));
  html += `<div class="bars">${bar("Local zone: median peak over background (micro-Doppler)", "snr_local_db", " dB", mx)}${bar("Back zone: median peak over background (body motion)", "snr_back_db", " dB", mx)}${bar("Near-antenna residual when empty (lower is cleaner)", "near_resid_db", " dB", Math.max(10, ...r.rows.map((x) => x.near_resid_db || 0)))}</div>`;
  html += `<p class="dim">${r.rule}. Noise floor: far-range high-Doppler cells after compression. Near residual: leakage flutter left at 0–2 m in an empty scene, over that floor. Saved in data/ab/.</p>`;
  box.innerHTML = html;
}

window.addEventListener("resize", () => { fan.key = ""; const m = S.last[S.sel]; if (m) renderTick(m); drawRoom(); });
drawTBIdle();
connect();
