#!/usr/bin/env python3
"""Summarise / plot a corner_probe run: python3 corner_report.py /tmp/corner_run1"""
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from corner_probe import DEFAULT_WORLD, FOOT_HALF_L, FOOT_HALF_W, load_obstacles, rect  # noqa: E402

run = sys.argv[1] if len(sys.argv) > 1 else "/tmp/corner_probe"
df = pd.read_csv(f"{run}/samples.csv")
df["t"] -= df["t"].iloc[0]
spd = np.hypot(df.fd_vx, df.fd_vy)
moving = spd > 0.08

# Is p3d twist world-frame or body-frame? compare with finite difference.
c, s = np.cos(df.yaw), np.sin(df.yaw)
bw_x, bw_y = c * df.tw_x - s * df.tw_y, s * df.tw_x + c * df.tw_y  # if twist were body frame
m = moving & df.fd_vx.notna()
err_world = np.nanmedian(np.hypot(df.tw_x[m] - df.fd_vx[m], df.tw_y[m] - df.fd_vy[m]))
err_body = np.nanmedian(np.hypot(bw_x[m] - df.fd_vx[m], bw_y[m] - df.fd_vy[m]))

sat_vx = (df.cmd_vx.abs() > 0.30 + 1e-3)
sat_vy = (df.cmd_vy.abs() > 0.25 + 1e-3)
sat_wz = (df.cmd_wz.abs() > 0.50 + 1e-3)

# direction distortion from per-axis clipping
ang_req = np.arctan2(df.cmd_vy, df.cmd_vx)
ang_act = np.arctan2(df.champ_vy, df.champ_vx)
dang = np.degrees(np.abs((ang_req - ang_act + np.pi) % (2 * np.pi) - np.pi))
nz = np.hypot(df.cmd_vx, df.cmd_vy) > 0.05

print(f"samples={len(df)}  duration={df.t.iloc[-1]:.1f}s  moving={moving.mean()*100:.0f}%")
print(f"actual speed: median={np.median(spd[moving]):.2f}  max={spd.max():.2f} m/s")
print(f"twist frame check: |tw - fd| world={err_world:.3f}  body={err_body:.3f}  → "
      f"{'WORLD' if err_world < err_body else 'BODY'} frame")
print(f"cmd saturation vs CHAMP caps: vx>{0.30}: {sat_vx.mean()*100:.0f}%  "
      f"vy>{0.25}: {sat_vy.mean()*100:.0f}%  wz>{0.5}: {sat_wz.mean()*100:.0f}%")
print(f"cmd |vx| max={df.cmd_vx.abs().max():.2f}  |vy| max={df.cmd_vy.abs().max():.2f}  "
      f"|wz| max={df.cmd_wz.abs().max():.2f}")
print(f"direction distortion from per-axis clip: median={np.median(dang[nz]):.1f}°  "
      f"p90={np.percentile(dang[nz], 90):.1f}°")
print(f"path deviation: median={df.path_dev.median():.2f}  max={df.path_dev.max():.2f} m")
print(f"wall clearance: min={df.wall_clear.min():.3f} m at t={df.t[df.wall_clear.idxmin()]:.1f}s "
      f"({df.nearest[df.wall_clear.idxmin()]})")
hits = df[df.wall_clear <= 0.0]
print(f"contact samples (footprint overlaps wall): {len(hits)}")
if len(hits):
    for name, g in hits.groupby("nearest"):
        print(f"   {name}: {len(g)} samples, t={g.t.min():.1f}-{g.t.max():.1f}s, "
              f"deepest={-g.wall_clear.min()*100:.1f} cm")

# ---------- plot ----------
fig, axes = plt.subplots(1, 2, figsize=(15, 7), gridspec_kw={"width_ratios": [1.3, 1]})
ax = axes[0]
for name, poly in load_obstacles(DEFAULT_WORLD):
    ax.fill(*np.vstack([poly, poly[:1]]).T, color="0.6" if "indoor" in name else "tan", lw=0)
sc = ax.scatter(df.x.to_numpy(), df.y.to_numpy(), c=df.wall_clear.clip(-0.05, 0.4).to_numpy(),
                cmap="RdYlGn", s=6, zorder=3)
plt.colorbar(sc, ax=ax, label="footprint→wall clearance (m)")
try:
    sp = np.loadtxt(f"{run}/last_spline.csv", delimiter=",")
    ax.plot(sp[:, 0], sp[:, 1], "b--", lw=1, label="last planned B-spline")
except OSError:
    pass
for i in range(0, len(df), max(1, len(df) // 40)):
    r = rect(df.x[i], df.y[i], FOOT_HALF_L, FOOT_HALF_W, df.yaw[i])
    ax.plot(*np.vstack([r, r[:1]]).T, color="k", lw=0.4, alpha=0.5)
for _, h in hits.iloc[:: max(1, len(hits) // 10)].iterrows():
    r = rect(h.x, h.y, FOOT_HALF_L, FOOT_HALF_W, h.yaw)
    ax.plot(*np.vstack([r, r[:1]]).T, color="r", lw=1.2)
ax.set_aspect("equal")
x0, x1 = df.x.min() - 1.0, df.x.max() + 1.0
y0, y1 = df.y.min() - 1.0, df.y.max() + 1.0
ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)
ax.set_title("Trajectory (red boxes = footprint touching walls)")
ax.legend(loc="upper right")

ax = axes[1]
T = df.t.to_numpy()
ax.plot(T, df.cmd_vx.to_numpy(), label="cmd vx"); ax.plot(T, df.cmd_vy.to_numpy(), label="cmd vy")
ax.plot(T, df.cmd_wz.to_numpy(), label="cmd wz")
ax.plot(T, spd.to_numpy(), "k", lw=1.5, label="actual speed")
ax.axhline(0.30, color="C0", ls=":"); ax.axhline(-0.25, color="C1", ls=":")
ax.axhline(0.25, color="C1", ls=":"); ax.axhline(0.5, color="C2", ls=":")
ax.axhline(-0.5, color="C2", ls=":")
ax2 = ax.twinx()
ax2.plot(T, df.wall_clear.to_numpy(), "r", lw=1, label="wall clearance")
ax2.axhline(0, color="r", ls="--", lw=0.8)
ax2.set_ylabel("clearance (m)", color="r")
ax.set_xlabel("sim time (s)"); ax.legend(loc="upper left", fontsize=8)
ax.set_title("Commands vs CHAMP caps (dotted)")
fig.tight_layout()
fig.savefig(f"{run}/report.png", dpi=110)
print(f"plot → {run}/report.png")
