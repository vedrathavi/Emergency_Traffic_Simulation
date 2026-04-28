import traci
from traci.exceptions import FatalTraCIError
import zlib

from algorithm import (
    CLEAR_RADIUS,
    FREEZE_RADIUS,
    INTERSECTION_CONTROL,
    INTERSECTION_RADIUS,
    BROADCAST_RADIUS,
    RESTORE_DELAY,
    SAFE_SPEED,
    EmergencyController,
    configure_sumo,
    deploy_emergency_vehicle,
)
from output import flush_logs, generate_graphs, initialize_output, save_summary



start_gui = "True"
em_vid="eme"
em_vehicle_start_time = 250  # 100x, 1200 = 12s
end_time = 20000

road_id = "E6"
LOG_INTERVAL = 10
PRINT_INTERVAL = 50
LOG_FLUSH_INTERVAL = 50
SAMPLE_RATE = 0.2
STOP_SPEED_THRESHOLD = 0.1
AMBULANCE_MAX_SPEED = 33.0
USE_CONTROLLER = True


def prompt_run_mode(default_use_controller=USE_CONTROLLER):
    print("\nSelect simulation mode:")
    print("1) With algorithm controller")
    print("2) Baseline (without controller)")

    default_option = "1" if default_use_controller else "2"
    while True:
        choice = input(f"Enter choice [1/2] (default {default_option}): ").strip()
        if choice == "":
            choice = default_option
        if choice == "1":
            print("[INFO] Running WITH algorithm controller")
            return True
        if choice == "2":
            print("[INFO] Running BASELINE without controller")
            return False
        print("[WARN] Invalid choice. Please enter 1 or 2.")


def is_sampled_vehicle(vid, sample_rate=SAMPLE_RATE):
    bucket = zlib.crc32(vid.encode("utf-8")) % 100
    return bucket < int(sample_rate * 100)


def safe_mean(values):
    return sum(values) / len(values) if values else 0.0


def get_road_density(road_id, vehicle_count):
    lane_id = f"{road_id}_0"
    if lane_id not in traci.lane.getIDList():
        return 0.0
    road_length_km = traci.lane.getLength(lane_id) / 1000.0
    if road_length_km <= 0:
        return 0.0
    return vehicle_count / road_length_km


def main(road_id, use_controller=USE_CONTROLLER):
    run_type = "with_algo" if use_controller else "baseline"
    output_paths = initialize_output(f"output/{run_type}")
    logs = []
    step = 0
    em_started = False
    em_seen_in_network = False
    em_start_step = None
    em_end_step = None
    clearance_step = None
    affected_vehicles = set()
    sampled_waiting_total = 0.0
    sampled_waiting_count = 0
    ambulance_speeds = []
    stop_count = 0
    was_stopped = False
    ambulance_distance_start = 0.0
    ambulance_distance_end = 0.0
    previous_ambulance_speed = None
    previous_smoothed_acceleration = 0.0
    acceleration_alpha = 0.8
    last_control_update = {
        "state": "CRUISING",
        "same_lane_count": 0,
        "adjacent_lane_count": 0,
        "vehicles_in_radius": 0,
        "close_adjacent_vehicles": 0,
        "affected_vehicles": [],
    }
    intersection_control_steps = 0
    cumulative_vehicles_cleared = 0
    cumulative_cleared_vehicle_ids = set()
    summary = None
    controller = (
        EmergencyController(
            em_vid=em_vid,
            broadcast_radius=BROADCAST_RADIUS,
            intersection_radius=INTERSECTION_RADIUS,
            clear_radius=CLEAR_RADIUS,
            freeze_radius=FREEZE_RADIUS,
            restore_delay=RESTORE_DELAY,
            safe_speed=SAFE_SPEED,
        )
        if use_controller
        else None
    )

    print("[INFO] Simulation started")

    try:
        configure_sumo(start_gui, "cfg/emergency.city.sumo.cfg")
        while step < end_time:
            try:
                traci.simulationStep()
            except FatalTraCIError:
                print("[WARN] SUMO closed early; finalizing collected data.")
                break

            if step == em_vehicle_start_time:
                deploy_emergency_vehicle(em_vid, start_gui)
                if use_controller:
                    traci.vehicle.setParameter(em_vid, "device.bluelight.reactiondist", str(90))
                else:
                    traci.vehicle.setParameter(em_vid, "device.bluelight.reactiondist", str(0))
                em_started = True
                em_start_step = step

            em_in_network = em_vid in traci.vehicle.getIDList()
            if em_started and em_in_network and not em_seen_in_network:
                em_seen_in_network = True
                ambulance_distance_start = traci.vehicle.getDistance(em_vid)

            if em_started and em_seen_in_network and (not em_in_network) and em_end_step is None:
                em_end_step = step
                print(f"[EVENT] Ambulance reached destination at t={step}")
                break

            if em_in_network:
                ambulance_speed = traci.vehicle.getSpeed(em_vid)
                if not use_controller:
                    traci.vehicle.setSpeed(em_vid, -1)
                ambulance_speeds.append(ambulance_speed)
                if ambulance_speed <= STOP_SPEED_THRESHOLD:
                    if not was_stopped:
                        stop_count += 1
                        was_stopped = True
                else:
                    was_stopped = False
                ambulance_distance_end = traci.vehicle.getDistance(em_vid)
            else:
                ambulance_speed = 0.0

            if previous_ambulance_speed is None:
                raw_acceleration = 0.0
            else:
                raw_acceleration = ambulance_speed - previous_ambulance_speed
            smoothed_acceleration = (
                acceleration_alpha * previous_smoothed_acceleration
                + (1 - acceleration_alpha) * raw_acceleration
            )
            previous_ambulance_speed = ambulance_speed
            previous_smoothed_acceleration = smoothed_acceleration

            if use_controller:
                control_update = controller.update(step)
            else:
                # Baseline mode: still calculate vehicle counts for metrics (not for control)
                em_road_id = traci.vehicle.getRoadID(em_vid) if em_vid in traci.vehicle.getIDList() else None
                same_lane_count = 0
                adjacent_lane_count = 0
                close_adjacent_count = 0
                
                if em_road_id and em_vid in traci.vehicle.getIDList():
                    try:
                        em_lane_idx = traci.vehicle.getLaneIndex(em_vid)
                        em_pos = traci.vehicle.getLanePosition(em_vid)
                        
                        # Count vehicles ahead in same and adjacent lanes (broadcast_radius = 80)
                        for vid in traci.edge.getLastStepVehicleIDs(em_road_id):
                            if vid == em_vid:
                                continue
                            veh_pos = traci.vehicle.getLanePosition(vid)
                            delta = veh_pos - em_pos
                            if delta <= 0 or delta > BROADCAST_RADIUS:
                                continue
                            veh_lane_idx = traci.vehicle.getLaneIndex(vid)
                            if veh_lane_idx == em_lane_idx:
                                same_lane_count += 1
                            elif abs(veh_lane_idx - em_lane_idx) == 1:
                                adjacent_lane_count += 1
                                if delta < 30:
                                    close_adjacent_count += 1
                    except:
                        pass
                
                control_update = {
                    "road_id": em_road_id,
                    "affected_vehicles": [],
                    "path_cleared": False,
                    "state": "NO_CONTROL",
                    "same_lane_count": same_lane_count,
                    "adjacent_lane_count": adjacent_lane_count,
                    "vehicles_in_radius": same_lane_count + adjacent_lane_count,
                    "close_adjacent_vehicles": close_adjacent_count,
                }
            last_control_update = control_update
            if control_update["road_id"]:
                road_id = control_update["road_id"]
            
            # Always track ambulance's current road (for both modes) to get accurate density
            if em_vid in traci.vehicle.getIDList():
                road_id = traci.vehicle.getRoadID(em_vid)
            
            actual_affected = set(control_update["affected_vehicles"])
            if actual_affected:
                affected_vehicles.update(actual_affected)
                cumulative_cleared_vehicle_ids.update(actual_affected)
            cumulative_vehicles_cleared = len(cumulative_cleared_vehicle_ids)
            if em_started and control_update["path_cleared"] and clearance_step is None:
                clearance_step = step
                print(f"[EVENT] Path cleared at t={step}")
            if control_update["state"] == INTERSECTION_CONTROL:
                intersection_control_steps += 1

            if step % LOG_INTERVAL == 0 and step >= em_vehicle_start_time:
                car_list = traci.edge.getLastStepVehicleIDs(road_id)
                sampled_vehicles = [vid for vid in car_list if is_sampled_vehicle(vid)]
                sampled_speeds = [traci.vehicle.getSpeed(vid) for vid in sampled_vehicles]
                sampled_waiting = [traci.vehicle.getWaitingTime(vid) for vid in sampled_vehicles]
                sampled_waiting_total += sum(sampled_waiting)
                sampled_waiting_count += len(sampled_waiting)

                queue_length = sum(
                    1 for vid in car_list if traci.vehicle.getSpeed(vid) <= STOP_SPEED_THRESHOLD
                )
                
                # DEBUG: show which road and vehicle count is being measured
                if step % (LOG_INTERVAL * 5) == 0:
                    print(f"[DEBUG] Road: {road_id}, Vehicles on road: {len(car_list)}, Density: {get_road_density(road_id, len(car_list)):.3f}")

                log_entry = {
                    "time": step,
                    "ambulance_speed": round(ambulance_speed, 3),
                    "acceleration": round(smoothed_acceleration, 3),
                    "speed_loss": round(max(0.0, AMBULANCE_MAX_SPEED - ambulance_speed), 3),
                    "avg_speed": round(safe_mean(sampled_speeds), 3),
                    "density": round(get_road_density(road_id, len(car_list)), 3),
                    "avg_waiting_time": round(safe_mean(sampled_waiting), 3),
                    "queue_length": int(queue_length),
                    "vehicles_sampled": len(sampled_vehicles),
                    "vehicles_in_radius": int(last_control_update["vehicles_in_radius"]),
                    "vehicles_cleared": int(len(actual_affected)),
                    "cumulative_vehicles_cleared": int(cumulative_vehicles_cleared),
                    "same_lane_density": int(last_control_update["same_lane_count"]),
                    "adjacent_lane_density": int(last_control_update["adjacent_lane_count"]),
                    "close_adjacent_vehicles": int(last_control_update["close_adjacent_vehicles"]),
                    "intersection_delay": int(intersection_control_steps),
                    "controller_state": last_control_update["state"],
                }
                logs.append(log_entry)

                if step % LOG_FLUSH_INTERVAL == 0:
                    flush_logs(logs, output_paths["logs"])

                if step % PRINT_INTERVAL == 0:
                    print(
                        f"[STEP {step}] Ambulance speed: {log_entry['ambulance_speed']:.2f} m/s | "
                        f"Avg traffic: {log_entry['avg_speed']:.2f} m/s"
                    )

            step += 1

        if em_end_step is None and em_started:
            em_end_step = step

        travel_time = (em_end_step - em_start_step) if em_start_step is not None else 0
        ideal_time = 0.0
        distance_travelled = max(0.0, ambulance_distance_end - ambulance_distance_start)
        if distance_travelled > 0:
            ideal_time = distance_travelled / AMBULANCE_MAX_SPEED

        summary = {
            "total_time": step,
            "travel_time": travel_time,
            "ambulance_avg_speed": round(safe_mean(ambulance_speeds), 3),
            "stops_count": stop_count,
            "delay_time": round(max(0.0, travel_time - ideal_time), 3),
            "avg_vehicle_speed": round(safe_mean([row["avg_speed"] for row in logs]), 3),
            "avg_waiting_time": round(
                (sampled_waiting_total / sampled_waiting_count) if sampled_waiting_count else 0.0,
                3,
            ),
            "vehicles_affected": len(affected_vehicles),
            "avg_delay_other_vehicles": round(
                (sampled_waiting_total / sampled_waiting_count) if sampled_waiting_count else 0.0,
                3,
            ),
            "clearance_time": (
                (clearance_step - em_start_step)
                if clearance_step is not None and em_start_step is not None
                else None
            ),
            "intersection_delay": intersection_control_steps,
            "corridor_violations": sum(row["close_adjacent_vehicles"] for row in logs),
            "cumulative_vehicles_cleared": cumulative_vehicles_cleared,
        }

        print("[INFO] Simulation completed")
        print(
            f"[RESULT] Travel time: {summary['travel_time']}s | "
            f"Stops: {summary['stops_count']} | "
            f"Vehicles affected: {summary['vehicles_affected']}"
        )
    finally:
        flush_logs(logs, output_paths["logs"])
        if summary is not None:
            save_summary(summary, output_paths["summary"])
            generate_graphs(logs, output_paths["graphs"])
        try:
            traci.close()
        except Exception:
            pass


if __name__ == "__main__":
    selected_use_controller = prompt_run_mode(USE_CONTROLLER)
    main(road_id, use_controller=selected_use_controller)
