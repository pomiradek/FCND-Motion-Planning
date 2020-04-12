import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import *
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import matplotlib.pyplot as plt
from sampling import Sampler

import sys
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx

import re

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        filename = 'colliders.csv'
        data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
        sampler = Sampler(data)
        polygons = sampler._polygons
        print(len(polygons))
        nodes = sampler.sample(3000)
        print(len(nodes))

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values

        #
        # Read lat0, lon0 from colliders into floating point values
        """"
        with open(filename) as f:
            origin_pos_data = f.readline().split(',')
        lat0 = float(origin_pos_data[0].strip().split(' ')[1])
        lon0 = float(origin_pos_data[1].strip().split(' ')[1])
        # Set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
        """
        with open(filename) as f:
            # lat0 37.792480, lon0 -122.397450
            home = re.findall(r"[-+]?\d*\.*\d+", f.readline())
            print(home)
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(float(home[3]), float(home[1]), 0.0)

        # TODO: retrieve current global position
        global_position = [self._longitude, self._latitude, self._altitude]
 
        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
        print(data.shape)
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}" . format(north_offset, east_offset))

        """
        plt.imshow(grid, origin='lower')

        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        plt.show()
        """

        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(np.round(local_position[0]))-north_offset, int(np.round(local_position[1]))-east_offset)
        
        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_lon = -122.397938
        goal_lat = 37.793492

        goal_local = global_to_local((goal_lon, goal_lat, 0.0), self.global_home)
        grid_goal = (int(np.round(goal_local[0])) - north_offset, int(np.round(goal_local[1])) - east_offset)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        print('Sampling nodes...')
        t0 = time.time()
        print(data.shape)
        sampler = Sampler(data)
        polygons = sampler._polygons
        print(len(polygons))
        print('Sampler took {0} seconds to build'.format(time.time() - t0))
        # 1000 is veeery slow
        t0 = time.time()
        nodes = sampler.sample(100)
        print('Sampling took {0} seconds to build'.format(time.time() - t0))
        print('Number of nodes: ', len(nodes))

        print('Creating graph...')
        t0 = time.time()
        graph = create_graph(nodes, 5, sampler.polygons, grid)
        print('Graph took {0} seconds to build'.format(time.time() - t0))

        print("Number of edges", len(graph.edges))

        """
        fig = plt.figure()

        plt.imshow(grid, cmap='Greys', origin='lower')

        nmin = np.min(data[:, 0])
        emin = np.min(data[:, 1])

        # draw edges
        for (n1, n2) in graph.edges:
            plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black', alpha=0.5)

        # draw all nodes
        for n1 in nodes:
            plt.scatter(n1[1] - emin, n1[0] - nmin, c='blue')

        # draw connected nodes
        for n1 in graph.nodes:
            plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')

        plt.xlabel('NORTH')
        plt.ylabel('EAST')

        plt.show()
        """

        """
        alpha = 1.0
        beta = 2.0
        q_max = 10
        x, y, fx, fy = potential_field(grid, grid_goal, alpha, beta, q_max)
        plt.imshow(grid, cmap='Greys', origin='lower')
        plt.plot(grid_goal[1], grid_goal[0], 'ro')
        plt.quiver(y, x, fy, fx)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()
        """

        print('Local Start and Goal: ', grid_start, grid_goal)
        path, cost = a_star(graph, heuristic, grid_start, grid_goal)

        path_pairs = zip(path[:-1], path[1:])
        for (n1, n2) in path_pairs:
            print(n1, n2)

        fig = plt.figure()

        plt.imshow(grid, cmap='Greys', origin='lower')

        nmin = np.min(data[:, 0])
        emin = np.min(data[:, 1])

        # draw nodes
        for n1 in graph.nodes:
            plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')

        # draw edges
        for (n1, n2) in graph.edges:
            plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black')

        path_pairs = zip(path[:-1], path[1:])
        for (n1, n2) in path_pairs:
            plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'green')

        plt.xlabel('NORTH')
        plt.ylabel('EAST')

        plt.show()

        # TODO: prune path to minimize number of waypoints
        pruned_path = prune_path_bresenham(grid, path)
        print('Prunned path length: ', len(pruned_path))
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        print(pruned_path)

        plt.imshow(grid, cmap='Greys', origin='lower')

        plt.plot(grid_start[1], grid_start[0], 'x')
        plt.plot(grid_goal[1], grid_goal[0], 'x')

        pp = np.array(pruned_path)
        plt.plot(pp[:, 1], pp[:, 0], 'g')
        plt.scatter(pp[:, 1], pp[:, 0])

        plt.xlabel('EAST')
        plt.ylabel('NORTH')

        plt.show()

        print('Sending')

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
