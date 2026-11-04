import numpy as np
import time
from rplidar import RPLidar, RPLidarException

class LIDAR:

	def __init__(self):
		print("Initializing Lidar...")
		retry = 0
		while retry < 10:
			try:
				self.lidar = RPLidar("/dev/ttyUSB0")
				self.info = self.lidar.get_info()
				print(self.info)
				break
			except RPLidarException as e:
				if "Incorrect descriptor" in e.args[0]:
					print(f"LIDAR busy. Restarting... ({retry}):", e)
					self.kill_lidar()
					retry += 1
				elif "could not open port /dev/ttyUSB0" in e.args[0]:
					print(f"ERROR: Could not open port /dev/ttyUSB0. Please make sure the lidar is plugged in.")
					exit(1)
				else:
					raise e
		if retry == 10:
			raise Exception("Lidar could not be initialized.")
		print("Lidar Initialized!\n")
		self.scan_data = np.zeros(360)
		self.lidar.clean_input()


	def kill_lidar(self):
		if hasattr(self, "lidar") and self.lidar is not None:
			self.lidar.stop()
			self.lidar.disconnect()

	def __del__(self):
		self.kill_lidar()

	def get_data(self):
		while 1:
			try:
				self.lidar.clean_input()
				for scan in self.lidar.iter_scans():
					min_angle, max_angle = np.inf, -np.inf
					scan_data = np.zeros(360)
					for (_, angle, distance_mm) in scan:
						# subtract 180 to reverse lidar direction
						angle = angle - 180

						# get angle in degrees
						scan_angle = min(359, int(angle % 360))
						# print(scan_angle, distance_mm)

						# convert millimeters to meters
						distance_m = distance_mm / 1000.0

						# save distance at that angle
						scan_data[scan_angle] = distance_m

						# record max and mins
						min_angle = min(min_angle, scan_angle)
						max_angle = max(max_angle, scan_angle)

					# print(min_angle, max_angle)
					yield scan_data
			except RPLidarException as e:
				continue
if __name__ == "__main__":
	# Test we can connect to lidar and print data
	lidar = LIDAR()
	i = 0
	for data in lidar.get_data():
		i += 1
		print(data)
		if i == 3:
			exit()
