import traci


def get_vid_info(vid, step):
	acc = traci.vehicle.getAccel(vid)
	speed = traci.vehicle.getSpeed(vid)
	pos = traci.vehicle.getLanePosition(vid)
	lane = traci.vehicle.getLaneIndex(vid)
	return (step, vid, acc, speed, pos, lane)


def configure_sumo(start_gui, config_path="cfg/emergency.city.sumo.cfg"):
	sumo_exe = "sumo-gui"
	if start_gui != "True":
		sumo_exe = "sumo"

	traci.start(
		[
			sumo_exe,
			"-c",
			config_path,
			"--lanechange.duration",
			"2",
			"--random",
			"--tls.actuated.jam-threshold",
			"3",
			"--device.bluelight.explicit",
			"true",
		]
	)

	if start_gui == "True":
		traci.gui.setSchema("View #0", "real world")


def deploy_emergency_vehicle(em_vid, start_gui):
	traci.vehicle.add(
		em_vid,
		"em_route",
		typeID="emergency_v",
		departSpeed="19",
		departLane="0",
	)
	traci.vehicle.setParameter(em_vid, "emergency", "yes")
	traci.vehicle.setParameter(em_vid, "device.bluelight.reactiondist", str(90))
	traci.vehicle.setMaxSpeed(em_vid, 33)
	traci.vehicle.setSpeedMode(em_vid, 32)

	if start_gui == "True":
		traci.gui.trackVehicle("View #0", em_vid)
		traci.gui.setZoom("View #0", 3000)


def apply_emergency_lane_changes(
	step,
	em_vehicle_start_time,
	em_vid,
	detect_range,
	lcmode,
	lctime,
):
	if step % 20 != 0 or step <= em_vehicle_start_time + 800:
		return None

	if em_vid not in traci.vehicle.getIDList():
		return None

	road_id = traci.vehicle.getRoadID(em_vid)
	em_info = get_vid_info(em_vid, step)
	car_list = traci.edge.getLastStepVehicleIDs(road_id)

	if not car_list:
		return road_id

	for vid in car_list:
		res = get_vid_info(vid, step)
		traci.vehicle.setLaneChangeMode(vid, lcmode)

		if (
			(res[4] - em_info[4] < detect_range)
			and (res[4] - em_info[4] > 0)
			and res[5] == em_info[5]
			and res[3] > 3
		):
			lcsl = traci.vehicle.couldChangeLane(vid, 1)
			if lcsl:
				traci.vehicle.changeLaneRelative(vid, 1, lctime)
				print(f"vid:{vid},change left")
			else:
				traci.vehicle.changeLaneRelative(vid, -1, lctime)
				print(f"vid:{vid},change right")
		elif (
			(res[4] - em_info[4] < detect_range)
			and (res[4] - em_info[4] > 0)
			and res[3] > 3
		):
			if res[5] - em_info[5] > 0:
				traci.vehicle.changeLaneRelative(vid, 1, lctime)
			if res[5] - em_info[5] < 0:
				traci.vehicle.changeLaneRelative(vid, -1, lctime)

	return road_id
