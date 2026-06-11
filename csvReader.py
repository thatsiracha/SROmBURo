"""Simple CSV reader for PID torque log files.

Expected column order (11 columns) matching PID_Nathan_Vibe_IntegralUpdate.py
lines 621-625:

	tick,
	roll_f, pitch_f, rolldot_f, pitchdot_f, vel_roll_f, vel_pitch_f,
	tau_roll, tau_pitch, tau_motor2, tau_motor1

The module exposes `read_pid_csv(path)` which returns a dict of lists
for each column name, and a small CLI to print a summary.
"""

from __future__ import annotations

import csv
from typing import Dict, List


COLUMN_NAMES = [
	"tick",
	"roll_f", "pitch_f", "rolldot_f", "pitchdot_f", "vel_roll_f", "vel_pitch_f",
	"tau_roll", "tau_pitch", "tau_motor2", "tau_motor1",
]


def _is_header_row(row: List[str]) -> bool:
	if not row:
		return False
	# detect common header forms (e.g. first cell non-numeric or matches 'tick')
	first = row[0].strip()
	if first.lower() == "tick":
		return True
	try:
		float(first)
		return False
	except Exception:
		return True


def read_pid_csv(path: str) -> Dict[str, List[float]]:
	"""Read the PID log CSV at `path` and return columns as lists of floats.

	Non-numeric rows (headers, empty lines) are skipped. If a row has more
	columns than expected, the first 11 are used; if fewer, the row is skipped.
	"""
	cols: Dict[str, List[float]] = {name: [] for name in COLUMN_NAMES}

	with open(path, newline="") as f:
		reader = csv.reader(f)
		for row in reader:
			if not row:
				continue
			if _is_header_row(row):
				continue
			if len(row) < len(COLUMN_NAMES):
				# ignore short rows
				continue
			# parse first N columns
			values = []
			ok = True
			for v in row[: len(COLUMN_NAMES)]:
				try:
					values.append(float(v))
				except Exception:
					ok = False
					break
			if not ok:
				continue
			for name, val in zip(COLUMN_NAMES, values):
				cols[name].append(val)

	return cols


if __name__ == "__main__":
	import argparse
	import numpy as _np

	try:
		import matplotlib.pyplot as plt  # noqa: F401
	except Exception:
		plt = None

	def plot_pid_csv(data_or_path, hz: float = 500.0, start: int | None = None,
					 end: int | None = None, save: str | None = None, show: bool = True):
		"""Plot roll/pitch (degrees) and tau_roll/tau_pitch with two Y axes.

		`data_or_path` may be a path string or the dict returned by `read_pid_csv`.
		`hz` is the sampling frequency used to convert ticks to seconds.
		"""
		if isinstance(data_or_path, str):
			data = read_pid_csv(data_or_path)
		else:
			data = data_or_path

		if not any(data.values()):
			raise ValueError("No data to plot")

		ticks = data.get("tick", [])
		if ticks:
			t = _np.array(ticks, dtype=float) / float(hz)
		else:
			n = len(next(iter(data.values())))
			t = _np.arange(n) / float(hz)

		roll_deg  = _np.degrees(_np.array(data["roll_f"]))
		pitch_deg = _np.degrees(_np.array(data["pitch_f"]))
		tau_roll  = _np.array(data["tau_roll"]) if data["tau_roll"] else _np.array([])
		tau_pitch = _np.array(data["tau_pitch"]) if data["tau_pitch"] else _np.array([])

		if start is not None or end is not None:
			s = start or 0
			e = end or len(t)
			t = t[s:e]
			roll_deg = roll_deg[s:e]
			pitch_deg = pitch_deg[s:e]
			tau_roll = tau_roll[s:e]
			tau_pitch = tau_pitch[s:e]

		if plt is None:
			raise RuntimeError("matplotlib is required for plotting (pip install matplotlib)")

		fig, ax1 = plt.subplots(figsize=(10, 4))
		p1, = ax1.plot(t, roll_deg, label="roll_f (deg)", color="tab:blue")
		p2, = ax1.plot(t, pitch_deg, label="pitch_f (deg)", color="tab:red")
		ax1.set_xlabel("time (s)")
		ax1.set_ylabel("angle (deg)")

		ax2 = ax1.twinx()
		p3, = ax2.plot(t, tau_roll, label="tau_roll (Nm)", color="tab:blue", linestyle="--")
		p4, = ax2.plot(t, tau_pitch, label="tau_pitch (Nm)", color="tab:red", linestyle="--")
		ax2.set_ylabel("torque (Nm)")

		# combined legend
		handles = [p1, p2, p3, p4]
		labels = [h.get_label() for h in handles]
		ax1.legend(handles, labels, loc="upper right")

		fig.tight_layout()
		if save:
			fig.savefig(save)
		if show:
			plt.show()

	p = argparse.ArgumentParser(description="Read PID log CSV and show a brief summary / plot")
	p.add_argument("path", help="Path to CSV file")
	p.add_argument("--preview", "-n", type=int, default=3, help="Number of rows to preview")
	p.add_argument("--plot", action="store_true", help="Show time plots for roll/pitch and tau")
	p.add_argument("--hz", type=float, default=500.0, help="Sampling frequency (Hz) for ticks -> seconds")
	p.add_argument("--save", help="Path to save PNG of the plot (optional)")
	p.add_argument("--start", type=int, help="Slice start index for plotting")
	p.add_argument("--end", type=int, help="Slice end index for plotting")
	args = p.parse_args()

	data = read_pid_csv(args.path)
	total = len(next(iter(data.values()))) if data and any(data.values()) else 0
	print(f"Read {total} rows from: {args.path}")
	print("Columns:")
	for name in COLUMN_NAMES:
		print(f" - {name}: {len(data[name])} values")

	n = min(args.preview, total)
	if n > 0:
		print(f"\nFirst {n} rows:")
		for i in range(n):
			row = [data[name][i] for name in COLUMN_NAMES]
			print(row)

	if args.plot:
		plot_pid_csv(data, hz=args.hz, start=args.start, end=args.end, save=args.save, show=True)

