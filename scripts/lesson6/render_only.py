from dvisd_autonomy.sensors.lidar import LIDAR
from dvisd_autonomy.networking.TcpSocket import TcpSocketSender
import numpy as np
from tqdm import tqdm

from dvisd_autonomy.pure_pursuit.navigator import AutonomousNavigator
from dvisd_autonomy.control.control import Control
import time
import numpy as np

import time


def normalize(v):
	# Normalize a vector. This means to make it have a length of 1, while keeping the same direction.
    n = np.linalg.norm(v)
    if n < 1e-8:
        return v
    return v / n

def closest_point_on_line(p1, p2, point):
	# Returns the closest point on the line defined by p1 and p2 to the given point.
	d = p2 - p1
	denom = np.dot(d,d)
	if denom < 1e-5:
		denom += 1
	t = np.dot(point - p1, d) / denom
	return p1 + t * d

def hallway_waypoint(wall1, wall2, step=1.0):
    # Given the walls, compute a waypoint that is "step" meters forward in the hallway.

    p1, p2 = np.array(wall1[0]), np.array(wall1[1])
    p3, p4 = np.array(wall2[0]), np.array(wall2[1])

    origin = np.array([0.0, 0.0])

    # closest points to robot
    c1 = closest_point_on_line(p1, p2, origin)
    c2 = closest_point_on_line(p3, p4, origin)

    # center of hallway near robot
    center = (c1 + c2) / 2.0

    # wall directions
    d1 = normalize(p2 - p1)
    d2 = normalize(p4 - p3)

    # make them roughly same orientation
    if np.dot(d1, d2) < 0:
        d2 = -d2

    hallway_dir = normalize(d1 + d2)

	# force direction to point toward +x
    if hallway_dir[0] < 0:
        hallway_dir = -hallway_dir

    waypoint = center + hallway_dir * step

    return waypoint


def line_of_best_fit(points):
	# Given points, finds the line that goes through most of the points

	# Remove points close to 0,0
	margin_of_error = 0.05 # 5 cm
	zero_points = (np.abs(points[:, 0]) < margin_of_error) & (np.abs(points[:, 1]) < margin_of_error)
	points = points[~zero_points]

	if points.shape[0] <= 1:
		return np.array([[0.0, 0.0], [0.0, 0.0]])

	# Removes points that are far from the middle of the data
	# This is necesarry because the lidar is noisy, and sometimes 
	# it hallucinates points that are not there.
	# our line of best fit is very sensitive to this noise
	median = np.median(points, axis=0) # compute middle points
	dists = np.linalg.norm(points - median, axis=1) # compute distances
	keep = dists < np.percentile(dists, 80) # remove points that are in the 80th percentile of distance or higher
	points = points[keep]

	if points.shape[0] <= 1:
		return np.array([[0.0, 0.0], [0.0, 0.0]])

	#  pull out x and y
	x = points[:, 0]
	y = points[:, 1]

	# fit y = mx + b
	m, b = np.polyfit(y, x, 1)

	# Find two points along the line
	y1, y2 = y.min(), y.max()
	x1 = m * y1 + b
	x2 = m * y2 + b

	# return two points along the wall. 
	p1 = [x1, y1]
	p2 = [x2, y2]
	return np.array([p1, p2])



if __name__ == "__main__":

	##### Tune these parameters! ######################
	# TODO !!!!
	# How wide of a range of points do you want to use to predict the wall?
	angle = 65 # degrees

	# How far ahead to place the waypoint.
	lookahead_distance = 5.75 # meters

	###################################################

	# Connect to lidar
	lidar = LIDAR()

	# Create server to send data to laptop
	# TCP is the communication protocol that most of the internet uses. 
	tcp_socket = TcpSocketSender()

	# fetch data in a loop
	# Press CTRL+C to stop the program
	# Other notes:
	# 	"with tqdm() as pbar:" Creates a progress bar to show what time it is. 
	with tqdm() as pbar:
		for distances in lidar.get_data():
			# Convert data to x,y coordinates. 
			angles = np.array([i for i in range(360)])
			radians = np.radians(angles)
			xs = distances * np.cos(radians)
			ys = distances * np.sin(radians)
			points = np.stack([xs,ys], axis=1)

			# Compute points that we assume belong to the left wall.
			# Remember 0 degrees is forward. 90 degrees is perfectly to the left.
			# Therefore, we assume 90 degrees plus or minus "angle" must belong to the wall
			front_left_angle = 90 - angle
			back_left_angle = 90 + angle
			belongs_to_left_wall = (angles > front_left_angle) & (angles < back_left_angle)

			# do the same thing for the right wall
			# right = 270 degrees
			front_right_angle = 270 + angle
			back_right_angle = 270 - angle
			belongs_to_right_wall = (angles > back_right_angle) & (angles < front_right_angle)

			# now create points that belong to each list
			# This assigns the point to left_wall_points if it belongs to the left wall, otherwise assigns it to 0.0
			left_wall_points = points[belongs_to_left_wall]
			
			# do the same thing for the right wall
			right_wall_points = points[belongs_to_right_wall]
			
			# now gather any points that do not belong to either wall.
			# Thats what this means: ~(belongs_to_left_wall | belongs_to_right_wall)
			# ~ means "NOT", and | means "OR"
			# so you can read this as:
			# NOT belongs to left wall OR belongs to right wall
			other_points = points[~(belongs_to_left_wall | belongs_to_right_wall)]


			# We will also draw lines from the car (origin) at the specified angle,
			# so that its easier to see whats happening
			# Feel free to change the "angle" variable (above) to get an idea of whats happening. 
			front_left_endpoint = [np.cos(np.radians(front_left_angle)), np.sin(np.radians(front_left_angle))]
			back_left_endpoint = [np.cos(np.radians(back_left_angle)), np.sin(np.radians(back_left_angle))]
			front_right_endpoint = [np.cos(np.radians(front_right_angle)), np.sin(np.radians(front_right_angle))]
			back_right_endpoint = [np.cos(np.radians(back_right_angle)), np.sin(np.radians(back_right_angle))]

			# combine into a single array
			endpoints = np.array([
				front_left_endpoint, 
				back_left_endpoint, 
				front_right_endpoint, 
				back_right_endpoint
			])
			endpoints *= 10 # move the endpoint out to 10 meters so the line is longer. 

			# Now we will compute the line of best fit for each wall, and draw that as well.
			# This function first calculates the line that goes through the data
			# then it returns two points along this line for plotting. 
			left_wall = line_of_best_fit(left_wall_points)
			right_wall = line_of_best_fit(right_wall_points)
			walls = np.stack([left_wall, right_wall])

			# Now use the wall locations to create a waypoint
			waypoint = hallway_waypoint(left_wall, right_wall, step=lookahead_distance)

			# Create a path to the waypoint. In this case, its just a straight line
			path = np.zeros((2,2))
			path[0] = [0.0, 0.0] # start at the origin
			path[1] = waypoint # end at the waypoint


			# send to renderer, using different colors for each wall
			tcp_socket.send({
				"points_red": other_points,
				"points_blue": left_wall_points,
				"points_green": right_wall_points,
				"endpoints": endpoints,
				"walls": walls,
				"waypoint": waypoint,
				"path": path,
			})

			# update progress bar.
			pbar.update()
				

