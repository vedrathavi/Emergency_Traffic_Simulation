import traci
from traci.exceptions import FatalTraCIError
import zlib

from algorithm import (
    apply_emergency_lane_changes,
    configure_sumo,
    deploy_emergency_vehicle,
    get_vid_info,
)
from output import flush_logs, generate_graphs, initialize_output, save_summary



start_gui = "True"
em_vid="eme"
em_vehicle_start_time = 2500  # 100x, 1200 = 12s
end_time = 20000  
detect_range = 90
lctime = 3
lcmode=0b011001000101

road_id = "E6"
LOG_INTERVAL = 10
PRINT_INTERVAL = 50
LOG_FLUSH_INTERVAL = 50
SAMPLE_RATE = 0.2
STOP_SPEED_THRESHOLD = 0.1
AMBULANCE_MAX_SPEED = 33.0


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


def main(road_id):
    output_paths = initialize_output("output")
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
    summary = None

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

            updated_road_id, newly_affected, path_cleared = apply_emergency_lane_changes(
                step,
                em_vehicle_start_time,
                em_vid,
                detect_range,
                lcmode,
                lctime,
            )
            if updated_road_id:
                road_id = updated_road_id
            if newly_affected:
                affected_vehicles.update(newly_affected)
            if em_started and path_cleared and clearance_step is None:
                clearance_step = step
                print(f"[EVENT] Path cleared at t={step}")

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

                log_entry = {
                    "time": step,
                    "ambulance_speed": round(ambulance_speed, 3),
                    "avg_speed": round(safe_mean(sampled_speeds), 3),
                    "density": round(get_road_density(road_id, len(car_list)), 3),
                    "avg_waiting_time": round(safe_mean(sampled_waiting), 3),
                    "queue_length": int(queue_length),
                    "vehicles_sampled": len(sampled_vehicles),
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
    main(road_id)
