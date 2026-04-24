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

	try:
		import matplotlib.pyplot as plt
	except ImportError:
		return

	times = [row["time"] for row in logs]
	ambulance_speeds = [row["ambulance_speed"] for row in logs]
	avg_speeds = [row["avg_speed"] for row in logs]
	queue_lengths = [row["queue_length"] for row in logs]

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
	plt.plot(times, queue_lengths, label="Queue length", color="tab:red", linewidth=2)
	plt.xlabel("Time (s)")
	plt.ylabel("Vehicles")
	plt.title("Queue Length vs Time")
	plt.grid(alpha=0.3)
	plt.legend()
	plt.tight_layout()
	plt.savefig(graphs_dir / "queue_vs_time.png", dpi=150)
	plt.close()
