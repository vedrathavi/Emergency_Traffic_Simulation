import csv
import json
import shutil
from datetime import datetime
from pathlib import Path


def initialize_output(base_dir="output"):
	base_path = Path(base_dir)
	current_run = base_path / "current_run"
	history = base_path / "history"

	if current_run.exists() and any(current_run.iterdir()):
		history.mkdir(parents=True, exist_ok=True)
		timestamp = datetime.now().strftime("run_%Y-%m-%d_%H-%M-%S")
		shutil.move(str(current_run), str(history / timestamp))

	graphs_dir = current_run / "graphs"
	graphs_dir.mkdir(parents=True, exist_ok=True)

	paths = {
		"current_run": current_run,
		"logs": current_run / "logs.csv",
		"summary": current_run / "summary.json",
		"graphs": graphs_dir,
	}
	flush_logs([], paths["logs"])
	return paths


def write_json(path, data):
	path = Path(path)
	path.parent.mkdir(parents=True, exist_ok=True)
	with path.open("w", encoding="utf-8") as file_handle:
		json.dump(data, file_handle, indent=2)


def flush_logs(logs, logs_path):
	logs_path = Path(logs_path)
	logs_path.parent.mkdir(parents=True, exist_ok=True)

	if not logs:
		with logs_path.open("w", newline="", encoding="utf-8") as file_handle:
			file_handle.write("")
		return

	fieldnames = list(logs[0].keys())
	with logs_path.open("w", newline="", encoding="utf-8") as file_handle:
		writer = csv.DictWriter(file_handle, fieldnames=fieldnames)
		writer.writeheader()
		writer.writerows(logs)


def save_summary(summary_data, summary_path):
	write_json(summary_path, summary_data)


def generate_graphs(logs, graphs_dir):
	if not logs:
		return

	graphs_dir = Path(graphs_dir)
	graphs_dir.mkdir(parents=True, exist_ok=True)
	for existing_png in graphs_dir.glob("*.png"):
		existing_png.unlink(missing_ok=True)

	try:
		import matplotlib.pyplot as plt
	except ImportError:
		return

	times = [row["time"] for row in logs]
	ambulance_speeds = [row["ambulance_speed"] for row in logs]
	avg_speeds = [row["avg_speed"] for row in logs]
	speed_losses = [row.get("speed_loss", 0.0) for row in logs]
	same_lane_density = [row.get("same_lane_density", 0) for row in logs]
	adjacent_lane_density = [row.get("adjacent_lane_density", 0) for row in logs]
	controller_states = [row.get("controller_state", "CRUISING") for row in logs]

	plt.figure(figsize=(8, 4))
	plt.plot(times, ambulance_speeds, label="Ambulance speed", linewidth=2)
	plt.plot(times, avg_speeds, label="Avg traffic speed", linewidth=2)
	plt.xlabel("Time (s)")
	plt.ylabel("Speed (m/s)")
	plt.title("Speed vs Time")
	plt.grid(alpha=0.3)
	plt.legend()
	plt.tight_layout()
	plt.savefig(graphs_dir / "speed_vs_time.png", dpi=150)
	plt.close()

	plt.figure(figsize=(8, 4))
	plt.plot(times, speed_losses, label="Speed loss", color="tab:orange", linewidth=2)
	plt.xlabel("Time (s)")
	plt.ylabel("Speed Loss (m/s)")
	plt.title("Speed Loss vs Time")
	plt.grid(alpha=0.3)
	plt.legend()
	plt.tight_layout()
	plt.savefig(graphs_dir / "speed_loss_vs_time.png", dpi=150)
	plt.close()

	plt.figure(figsize=(8, 4))
	plt.plot(times, same_lane_density, label="Same lane density", linewidth=2)
	plt.plot(times, adjacent_lane_density, label="Adjacent lane density", linewidth=2)
	plt.xlabel("Time (s)")
	plt.ylabel("Vehicle count")
	plt.title("Lane Density vs Time")
	plt.grid(alpha=0.3)
	plt.legend()
	plt.tight_layout()
	plt.savefig(graphs_dir / "lane_density_vs_time.png", dpi=150)
	plt.close()

	state_to_y = {
		"CRUISING": 0,
		"CLEARING_PATH": 1,
		"INTERSECTION_APPROACH": 2,
		"INTERSECTION_CONTROL": 3,
		"EXIT_INTERSECTION": 4,
	}
	state_series = [state_to_y.get(state, 0) for state in controller_states]

	plt.figure(figsize=(10, 4))
	plt.step(times, state_series, where="post", linewidth=2, color="tab:brown")
	plt.yticks(list(state_to_y.values()), list(state_to_y.keys()))
	plt.xlabel("Time (s)")
	plt.ylabel("Controller state")
	plt.title("Intersection Events Timeline")
	plt.grid(alpha=0.3)
	plt.tight_layout()
	plt.savefig(graphs_dir / "intersection_timeline.png", dpi=150)
	plt.close()
