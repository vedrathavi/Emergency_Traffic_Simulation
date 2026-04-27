import traci
from traci.exceptions import TraCIException


BROADCAST_RADIUS = 80
INTERSECTION_RADIUS = 50
CLEAR_RADIUS = 50
FREEZE_RADIUS = 30
RESTORE_DELAY = 10
SAFE_SPEED = 7
AMBULANCE_MAX_SPEED = 35

CRUISING = "CRUISING"
CLEARING_PATH = "CLEARING_PATH"
INTERSECTION_APPROACH = "INTERSECTION_APPROACH"
INTERSECTION_CONTROL = "INTERSECTION_CONTROL"
EXIT_INTERSECTION = "EXIT_INTERSECTION"


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
	traci.vehicle.setSpeedMode(em_vid, 31)
	traci.vehicle.setLaneChangeMode(em_vid, 0)

	if start_gui == "True":
		traci.gui.trackVehicle("View #0", em_vid)
		traci.gui.setZoom("View #0", 3000)


class EmergencyController:
	def __init__(
		self,
		em_vid,
		broadcast_radius=BROADCAST_RADIUS,
		intersection_radius=INTERSECTION_RADIUS,
		clear_radius=CLEAR_RADIUS,
		freeze_radius=FREEZE_RADIUS,
		restore_delay=RESTORE_DELAY,
		safe_speed=SAFE_SPEED,
	):
		self.em_vid = em_vid
		self.broadcast_radius = broadcast_radius
		self.intersection_radius = intersection_radius
		self.clear_radius = clear_radius
		self.freeze_radius = freeze_radius
		self.restore_delay = restore_delay
		self.safe_speed = safe_speed
		self._exit_counter = 0
		self.state = CRUISING
		self.current_tls_id = None
		self.original_tls_state = None
		self.original_tls_program = None
		self.frozen_vehicles = set()
		self.exit_started_step = None

	def update(self, step):
		result = {
			"road_id": None,
			"affected_vehicles": [],
			"path_cleared": False,
			"state": self.state,
			"same_lane_count": 0,
			"adjacent_lane_count": 0,
			"vehicles_in_radius": 0,
			"close_adjacent_vehicles": 0,
		}

		if self.em_vid not in traci.vehicle.getIDList():
			result["state"] = self.state
			return result

		road_id = traci.vehicle.getRoadID(self.em_vid)
		em_lane_id = traci.vehicle.getLaneID(self.em_vid)
		em_lane_idx = traci.vehicle.getLaneIndex(self.em_vid)
		em_pos = traci.vehicle.getLanePosition(self.em_vid)
		next_tls = traci.vehicle.getNextTLS(self.em_vid)

		result["road_id"] = road_id

		upcoming_tls_id = next_tls[0][0] if next_tls else None
		distance_to_tls = next_tls[0][2] if next_tls else None
		near_intersection = (
			distance_to_tls is not None and distance_to_tls < self.intersection_radius
		)

		ahead_same_lane, ahead_adjacent = self._get_ahead_vehicles(
			road_id,
			em_lane_idx,
			em_pos,
			self.broadcast_radius,
		)

		self._force_ambulance_flow(ahead_same_lane, ahead_adjacent)
		path_cleared = len(ahead_same_lane) == 0 and len(ahead_adjacent) < 2

		result["same_lane_count"] = len(ahead_same_lane)
		result["adjacent_lane_count"] = len(ahead_adjacent)
		result["vehicles_in_radius"] = len(ahead_same_lane) + len(ahead_adjacent)
		result["close_adjacent_vehicles"] = sum(1 for delta, _, _ in ahead_adjacent if delta < 30)

		if self.current_tls_id and self._has_exited_control_zone(upcoming_tls_id, distance_to_tls):
			self.state = EXIT_INTERSECTION
			self._restore_intersection()
			self.exit_started_step = step

		if self.state == EXIT_INTERSECTION:
			if self.exit_started_step is None or step - self.exit_started_step >= self.restore_delay:
				self.state = CRUISING
				self.exit_started_step = None

		if near_intersection and self.state != EXIT_INTERSECTION:
			if self.state not in (INTERSECTION_APPROACH, INTERSECTION_CONTROL):
				self.state = INTERSECTION_APPROACH

			if self.state == INTERSECTION_APPROACH:
				self._prepare_intersection_control(upcoming_tls_id)
				self.state = INTERSECTION_CONTROL

			if self.state == INTERSECTION_CONTROL:
				affected = self._clear_path_in_intersection_zone(
					road_id,
					em_lane_idx,
					em_pos,
				)
				self._freeze_conflicting_traffic(upcoming_tls_id, em_lane_id)
				self._set_tls_green(upcoming_tls_id, em_lane_id)
				result["affected_vehicles"] = affected
				result["path_cleared"] = path_cleared
				result["state"] = self.state
				return result

		if ahead_same_lane or ahead_adjacent:
			self.state = CLEARING_PATH
			affected = self._clear_path(ahead_same_lane, ahead_adjacent, em_lane_idx)
			result["affected_vehicles"] = affected
			result["path_cleared"] = path_cleared
		else:
			self.state = CRUISING
			result["path_cleared"] = True

		result["state"] = self.state
		return result

	def _force_ambulance_flow(self, ahead_same_lane, ahead_adjacent):
		try:
			traci.vehicle.setSpeedMode(self.em_vid, 31)

			is_constrained = False
			let_sumo_control = False
			target_speed = AMBULANCE_MAX_SPEED

			# --- SAME LANE (hard constraint) ---
			if ahead_same_lane:
				nearest_distance = ahead_same_lane[0][0]
				is_constrained = True

				if nearest_distance < 8:
					target_speed = 0   # WAIT, don’t push
				elif nearest_distance < 20:
					target_speed = min(target_speed, nearest_distance * 0.8)
				else:
					target_speed = min(target_speed, nearest_distance * 1.2)

			# --- ADJACENT LANE (soft constraint) ---
			if ahead_adjacent:
				closest_adjacent_distance = ahead_adjacent[0][0]

				if closest_adjacent_distance < 6:
					current_speed = traci.vehicle.getSpeed(self.em_vid)
					is_constrained = True
					target_speed = min(target_speed, max(15, current_speed * 0.8))

				elif closest_adjacent_distance < 10:
					let_sumo_control = True

			# --- FINAL DECISION ---
			if is_constrained:
				traci.vehicle.setSpeed(
					self.em_vid,
					max(0, min(target_speed, AMBULANCE_MAX_SPEED))
				)
			elif let_sumo_control:
				traci.vehicle.setSpeed(self.em_vid, -1)
			else:
				traci.vehicle.setSpeed(self.em_vid, -1)

		except TraCIException:
			pass

	def _get_ahead_vehicles(self, road_id, em_lane_idx, em_pos, radius):
		ahead_same_lane = []
		ahead_adjacent = []
		for vid in traci.edge.getLastStepVehicleIDs(road_id):
			if vid == self.em_vid:
				continue

			veh_pos = traci.vehicle.getLanePosition(vid)
			delta = veh_pos - em_pos
			if delta <= 0 or delta > radius:
				continue

			veh_lane_idx = traci.vehicle.getLaneIndex(vid)
			entry = (delta, vid, veh_lane_idx)
			if veh_lane_idx == em_lane_idx:
				ahead_same_lane.append(entry)
			elif abs(veh_lane_idx - em_lane_idx) == 1:
				ahead_adjacent.append(entry)

		ahead_same_lane.sort(key=lambda item: item[0])
		ahead_adjacent.sort(key=lambda item: item[0])
		return ahead_same_lane, ahead_adjacent

	def _clear_path(self, ahead_same_lane, ahead_adjacent, em_lane_idx):
		affected = []

		for _, vid, veh_lane_idx in ahead_same_lane:
			if self._try_lane_change(vid, -1):
				affected.append(vid)
				continue
			if self._try_lane_change(vid, 1):
				affected.append(vid)
				continue

			if self._reduce_vehicle_speed(vid):
				affected.append(vid)

		for delta, vid, veh_lane_idx in ahead_adjacent:
			# Very close adjacent vehicles are treated as direct blockers.
			if delta < 10:
				if self._reduce_vehicle_speed(vid):
					affected.append(vid)
				continue

			if veh_lane_idx < em_lane_idx:
				direction = -1   # go further left
			else:
				direction = 1    # go further right
			if self._try_lane_change(vid, direction):
				affected.append(vid)
				continue

			# Virtual corridor: force nearby adjacent lanes to slow down.
			if abs(veh_lane_idx - em_lane_idx) <= 1 and delta < 30:
				if self._reduce_vehicle_speed(vid):
					affected.append(vid)

		return list(dict.fromkeys(affected))

	def _reduce_vehicle_speed(self, vid):
		try:
			current_speed = traci.vehicle.getSpeed(vid)
			if current_speed <= self.safe_speed:
				return False
			traci.vehicle.setSpeed(vid, self.safe_speed)
			return True
		except TraCIException:
			return False

	def _try_lane_change(self, vid, direction):
		try:
			traci.vehicle.setLaneChangeMode(vid, 0)
			if traci.vehicle.couldChangeLane(vid, direction):
				traci.vehicle.changeLaneRelative(vid, direction, 2)
				return True
		except TraCIException:
			return False
		return False

	def _prepare_intersection_control(self, tls_id):
		if not tls_id:
			return
		if self.current_tls_id == tls_id:
			return

		self.current_tls_id = tls_id
		self.frozen_vehicles.clear()
		try:
			self.original_tls_state = traci.trafficlight.getRedYellowGreenState(tls_id)
			self.original_tls_program = traci.trafficlight.getProgram(tls_id)
		except TraCIException:
			self.original_tls_state = None
			self.original_tls_program = None

	def _clear_path_in_intersection_zone(self, road_id, em_lane_idx, em_pos):
		ahead_same_lane, ahead_adjacent = self._get_ahead_vehicles(
			road_id,
			em_lane_idx,
			em_pos,
			self.clear_radius,
		)
		return self._clear_path(ahead_same_lane, ahead_adjacent, em_lane_idx)

	def _freeze_conflicting_traffic(self, tls_id, em_lane_id):
		if not tls_id:
			return

		em_edge = em_lane_id.rsplit("_", 1)[0] if "_" in em_lane_id else em_lane_id
		conflicting_edges = set()

		try:
			controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
		except TraCIException:
			controlled_lanes = []

		for lane_id in controlled_lanes:
			edge_id = lane_id.rsplit("_", 1)[0] if "_" in lane_id else lane_id
			if edge_id != em_edge:
				conflicting_edges.add(edge_id)

		for edge_id in conflicting_edges:
			for vid in traci.edge.getLastStepVehicleIDs(edge_id):
				lane_id = traci.vehicle.getLaneID(vid)
				lane_len = traci.lane.getLength(lane_id)
				lane_pos = traci.vehicle.getLanePosition(vid)
				distance_to_intersection = lane_len - lane_pos

				# Do not freeze vehicles already at the stop line / intersection core.
				if distance_to_intersection <= 5:
					traci.vehicle.setSpeed(vid, -1)
				elif distance_to_intersection < 10:
					traci.vehicle.setSpeed(vid, self.safe_speed)
					self.frozen_vehicles.add(vid)

	def _set_tls_green(self, tls_id, em_lane_id):
		if not tls_id:
			return
		try:
			if not self.original_tls_state:
				return

			controlled_links = traci.trafficlight.getControlledLinks(tls_id)
			new_state = list(self.original_tls_state)

			for i, links in enumerate(controlled_links):
				is_ambulance_link = False

				for link in links:
					if not link:
						continue

					incoming_lane = link[0]

					# Ambulance lane → must be green
					if incoming_lane == em_lane_id:
						is_ambulance_link = True
						break

				if is_ambulance_link:
					new_state[i] = "G"
				else:
					# Only block if originally green (avoid unnecessary override)
					if new_state[i] in ("G", "g"):
						new_state[i] = "r"

			traci.trafficlight.setRedYellowGreenState(tls_id, "".join(new_state))

		except TraCIException:
			pass

	def _has_exited_control_zone(self, upcoming_tls_id, distance_to_tls):
		if not self.current_tls_id:
			return False

		if upcoming_tls_id == self.current_tls_id:
			self._exit_counter = 0
			return False

		self._exit_counter += 1
		return self._exit_counter >= 50

	def _restore_intersection(self):
		self.last_tls_id = self.current_tls_id
		self._exit_counter = 0
		# --- Restore vehicles gradually ---
		for vid in list(self.frozen_vehicles):
			if vid not in traci.vehicle.getIDList():
				continue

			try:
				current_speed = traci.vehicle.getSpeed(vid)

				# Smooth recovery instead of sudden jump
				if current_speed < self.safe_speed + 5:
					traci.vehicle.setSpeed(vid, self.safe_speed + 5)
				else:
					traci.vehicle.setSpeed(vid, -1)  # give control back to SUMO

			except TraCIException:
				pass

		self.frozen_vehicles.clear()

		# --- Restore traffic light ---
		if self.current_tls_id:
			try:
				if self.original_tls_program is not None:
					traci.trafficlight.setProgram(
						self.current_tls_id,
						self.original_tls_program
					)
				elif self.original_tls_state is not None:
					traci.trafficlight.setRedYellowGreenState(
						self.current_tls_id,
						self.original_tls_state,
					)
			except TraCIException:
				pass

		# --- Reset controller state ---
		self.current_tls_id = None
		self.original_tls_state = None
		self.original_tls_program = None
