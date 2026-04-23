import csv

import traci


def open_csv_writer(path="data/data.csv"):
	file_handle = open(path, "a+", newline="")
	return file_handle, csv.writer(file_handle)


def write_road_snapshot(writer, road_id, step, info_fn):
	car_list = traci.edge.getLastStepVehicleIDs(road_id)
	if not car_list:
		return

	for vid in car_list:
		writer.writerow(info_fn(vid, step))


def flush_output(file_handle):
	file_handle.flush()
