# Emergency_Traffic_Simulation

This project aims to simulate emergency traffic on city roads using SUMO (Simulation of Urban Mobility).

Features:

- Completely random vehicle types and attributes, resulting in dynamic traffic flow
- Real world image backgound (Dashikou, Zhenjiang, Jiangsu)
- Vehicles will change lanes in order to avoid emergency vehicle
- Emergency vehicle is over speed and ignores red lights

Problems:

- The traffic lights are not intelligent, often leading to congestion

![output](assets/output.gif)

![traffic_light](assets/traffic_light.gif)

![Screen Shot 2023-03-18 at 10.42.17](assets/Screen%20Shot%202023-03-18%20at%2010.42.17.jpg)

## Getting Started

To get started, you have to download sumo https://www.eclipse.org/sumo/

If you are using Linux and apt, you can just paste the following command in your terminal:

````bash
sudo add-apt-repository ppa:sumo/stable
sudo apt-get update
sudo apt-get install sumo sumo-tools
pip install traci  ## important! I didn't use the traditional
# Emergency_Traffic_Simulation

Emergency_Traffic_Simulation is a research / demonstration project that simulates emergency vehicle (ambulance) driving through urban traffic using SUMO (Simulation of Urban Mobility) and TraCI. The project implements a simple local controller that attempts to clear a corridor for the emergency vehicle by requesting lane changes, slowing/blocking conflicting traffic, and temporarily controlling traffic lights near intersections.

This README answers the 5Ws, provides a full setup guide, explains how the simulation works, and lists configuration and troubleshooting tips.

**Contents**
- **What**: what the project does
- **Why**: goals and use cases
- **Who**: intended users and authors
- **How**: architecture and core algorithm summary
- **Where / When**: supported platforms and minimum versions
- **Setup & Run**: step-by-step installation and run instructions
- **Configuration**: where to change routes, traffic, and TLS
- **Outputs**: logs, summaries and graphs
- **Troubleshooting & Notes**n+
---

**What**
- Simulates an emergency vehicle traveling through a SUMO road network while other traffic dynamically reacts.
- Includes an `EmergencyController` (in `src/algorithm.py`) that: requests lane changes, reduces speeds of conflicting vehicles, and temporarily alters traffic light (TLS) states to create a clear path through intersections.

**Why**
- Evaluate how simple decentralized controls affect ambulance travel time and traffic disruption.
- Useful for research experiments, visual demos, and prototyping traffic interventions for emergency scenarios.

**Who**
- Researchers, students, or developers who want a SUMO-based environment to test emergency vehicle routing and simple control strategies.

**How (high level)**
- SUMO runs the traffic simulation and exposes an API via TraCI.
- The Python runner `src/main.py` starts SUMO (`sumo` or `sumo-gui`) and steps the simulation loop.
- `EmergencyController` (in `src/algorithm.py`) inspects vehicles ahead of the ambulance and attempts corrective actions:
	- request nearby vehicles to change lane using `traci.vehicle.changeLaneRelative`
	- reduce speeds of conflicting vehicles using `traci.vehicle.setSpeed`
	- freeze or slow conflicting approaches at traffic lights and set TLS green for the ambulance's incoming link
- The `src/output.py` module collects logs, writes `logs.csv` and `summary.json`, and optionally generates graphs using `matplotlib`.

---

**Minimum technical requirements (works on versions >=)**
- **Operating System:** Windows 10 / Windows 11 or Linux (Ubuntu 20.04+ recommended)
- **Python:** >= 3.8
- **SUMO:** >= 1.12.0 (must provide `sumo` or `sumo-gui` on PATH and TraCI support)
- **TraCI Python client (`traci`)**: >= 1.1.0 (install via `pip install traci`)
- **Matplotlib:** >= 3.3.0 (optional — required only for graph generation)
- **Shell / Utilities:** POSIX shell for `cfg/autoGenTradic.sh` (Linux); Windows users can use PowerShell, WSL, or adapt scripts
- **Environment variable:** `SUMO_HOME` set to SUMO installation directory (recommended for SUMO tools)
- **Hardware:** 4 GB RAM minimum; 8+ GB recommended for larger simulations

All other libraries used in the code are part of the Python standard library (`csv`, `json`, `pathlib`, `shutil`, `datetime`, `zlib`).

---

**Quick Setup (Windows)**
1. Install SUMO: download and install from https://www.eclipse.org/sumo/ (or use package manager on Linux).
2. Ensure `sumo` and `sumo-gui` executables are on your PATH (or set `SUMO_HOME` environment variable).
3. Create and activate a Python 3.8+ virtual environment:

```powershell
python -m venv .venv
.\\.venv\\Scripts\\Activate.ps1   # PowerShell
````

4. Install Python dependencies:

```powershell
pip install --upgrade pip
pip install traci matplotlib
```

5. Run the simulation (GUI mode):

```powershell
cd src
python main.py
# or explicitly enable GUI:
set GUI=True; python main.py   # Windows cmd/powershell example (see code uses start_gui variable)
```

Notes: if you prefer headless (no GUI) set the `start_gui` variable in `src/main.py` to a non-"True" value or run with an environment variable wrapper.

**Quick Setup (Linux)**

```bash
sudo add-apt-repository ppa:sumo/stable
sudo apt-get update
sudo apt-get install sumo sumo-tools
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install traci matplotlib
cd src
python3 main.py
```

---

**How to run with/without the controller**

- When launched, `main.py` prompts to select simulation mode:
  1.  With algorithm controller — runs `EmergencyController` to actively clear a path.
  2.  Baseline — runs without the controller (ambulance behaves like any vehicle but may have different parameters).

You can also change the default by editing the `USE_CONTROLLER` variable in `src/main.py`.

---

**Key files and directories**

- `src/main.py` — main runner; starts SUMO and contains the simulation loop.
- `src/algorithm.py` — controller logic and helper functions.
- `src/output.py` — logging, summary, and optional graph generation.
- `cfg/` — SUMO network, route and configuration files (edit `cfg/emergency.city.sumo.cfg` to change scenario inputs).
- `assets/` — images and demo GIFs used for the project website/README.
- `output/` — runtime outputs; `current_run/` contains `logs.csv`, `summary.json`, and `graphs/`.

---

**Configuration & common edits**

- Change the ambulance route: edit `cfg/output.trips0.xml` (route id `em_route`). Use `netedit` if you need to inspect edge ids.
- Adjust traffic generation: edit `cfg/autoGenTradic.sh` or files in `cfg/vtype/` and re-run the script to generate `output.trips*.xml`.
- SUMO config: `cfg/emergency.city.sumo.cfg` references the network and routes. Modify `input` files there to add/remove trip files.

**Important runtime parameters (in code)**

- `src/main.py` contains variables such as `AMBULANCE_MAX_SPEED`, `em_vehicle_start_time`, `end_time`, and sampling rates — tweak them for experiments.

---

**Outputs**

- `output/current_run/logs.csv` — time series of logged metrics (ambulance speed, avg traffic speed, queue length, etc.).
- `output/current_run/summary.json` — aggregated summary for the run.
- `output/current_run/graphs/` — PNG graphs generated by `src/output.py` (requires `matplotlib`).

---

**Generating heatmaps / plots**

1. After a run completes, scripts in `src/output.py` will automatically generate plots if `matplotlib` is installed.
2. For the earlier project workflow, there are instructions to run `sh run.sh` (under `src`); this may invoke further processing or plotting. On Windows prefer PowerShell or WSL for POSIX shell scripts.

---

**Troubleshooting**

- If `traci` cannot connect: ensure SUMO is installed and the `sumo` or `sumo-gui` executable is available on PATH. Setting `SUMO_HOME` is recommended.
- If graphs are missing: install `matplotlib` inside the same Python environment used to run `main.py`.
- If a runtime TraCI exception occurs: logs are flushed at the end of simulation; inspect `output/current_run/logs.csv` and check for early SUMO exit.

---

**Development & experiments**

- To test controller logic, modify thresholds and radii in `src/algorithm.py` (e.g., `BROADCAST_RADIUS`, `CLEAR_RADIUS`, `INTERSECTION_RADIUS`, `SAFE_SPEED`) and re-run.
- Add instrumentation by logging values to `logs` in `src/main.py` or extend `src/output.py` with new plots.

---

**Contribution & License**

- The project contains a `LICENSE` file — consult it for reuse and contribution rules.
- Contributions welcome: open issues or PRs with focused changes (fixes, parameterization, better TLS handling, improved plotting).

---

If you'd like, I can next:

- (A) produce a `requirements.txt` with pinned minimum versions, or
- (B) add a short `docs/` folder with example experiments and recommended parameter settings.

File: [README.md](README.md)
