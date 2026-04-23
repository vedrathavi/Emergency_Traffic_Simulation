import traci

from algorithm import (
    apply_emergency_lane_changes,
    configure_sumo,
    deploy_emergency_vehicle,
    get_vid_info,
)
from output import flush_output, open_csv_writer, write_road_snapshot



start_gui = "True"
em_vid="eme"
em_vehicle_start_time = 2000  # 100x, 1200 = 12s
end_time = 2000000
detect_range = 80
lctime = 3
lcmode=0b011001000101

road_id = "E6"


def main(road_id):
    f, writer = open_csv_writer("data/data.csv")
    step = 0 

    configure_sumo(start_gui, "cfg/emergency.city.sumo.cfg")
    while step < end_time:
        traci.simulationStep()
        if step == em_vehicle_start_time:
            deploy_emergency_vehicle(em_vid, start_gui)

        updated_road_id = apply_emergency_lane_changes(
            step,
            em_vehicle_start_time,
            em_vid,
            detect_range,
            lcmode,
            lctime,
        )
        if updated_road_id:
            road_id = updated_road_id

            
        if step % 10 == 0 and step > em_vehicle_start_time - 1000:
            write_road_snapshot(writer, road_id, step, get_vid_info)
                
        if step % 500 == 0 and step > em_vehicle_start_time - 1000:
            flush_output(f)



        step+=1

    f.close()
    traci.close()
if __name__ == "__main__":
    main(road_id)
