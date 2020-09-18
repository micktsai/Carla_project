#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles.
The agent also responds to traffic lights. """
import math
import sys
import glob
import os
import numpy as np
try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/carla')
except IndexError:
    pass

import carla
from _agent import _Agent, _AgentState
from agents.navigation.local_planner import LocalPlanner
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

class TryAgent(_Agent):
    """
    BasicAgent implements a basic agent that navigates scenes to reach a given
    target destination. This agent respects traffic lights and other vehicles.
    """

    def __init__(self, vehicle, target_speed=20):
        """

        :param vehicle: actor to apply to local planner logic onto
        """
        super(TryAgent, self).__init__(vehicle)

        self._proximity_threshold = 10.0  # meters
        self._state = _AgentState.NAVIGATING
        args_lateral_dict = {
            'K_P': 1,
            'K_D': 0.02,
            'K_I': 0,
            'dt': 1.0/20.0}
        self._local_planner = LocalPlanner(
            self._vehicle, opt_dict={'target_speed' : target_speed,
            'lateral_control_dict':args_lateral_dict})
        self._hop_resolution = 2.0
        self._path_seperation_hop = 2
        self._path_seperation_threshold = 0.5
        self._target_speed = target_speed
        self._grp = None
        self.speed = 60
        self.matrix_transform = None
        self.last = False
        self.walker = None

    def set_destination(self, location):
        """
        This method creates a list of waypoints from agent's position to destination location
        based on the route returned by the global router
        """

        start_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        end_waypoint = self._map.get_waypoint(
            carla.Location(location[0], location[1], location[2]))

        route_trace = self._trace_route(start_waypoint, end_waypoint)
        assert route_trace

        self._local_planner.set_global_plan(route_trace)

    def _trace_route(self, start_waypoint, end_waypoint):
        """
        This method sets up a global router and returns the optimal route
        from start_waypoint to end_waypoint
        """

        # Setting up global router
        if self._grp is None:
            dao = GlobalRoutePlannerDAO(self._vehicle.get_world().get_map(), self._hop_resolution)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._grp = grp

        # Obtain route plan
        route = self._grp.trace_route(
            start_waypoint.transform.location,
            end_waypoint.transform.location)

        return route


    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """
        self.matrix_transform = self.get_matrix(self._vehicle.get_transform())
        matrix = get_matrix(self._vehicle.get_transform())
        velocity = np.dot(np.array([1,0,0]),np.linalg.inv(matrix))
        # is there an obstacle in front of us?
        hazard_detected = False

        # retrieve relevant elements for safe navigation, i.e.: traffic lights
        # and other vehicles
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")
        # pedestrian_list = actor_list.filter("*pedestrian*")
        walker_list = actor_list.filter("*walker*")
        angle = float(10/17)
        angles = np.array([0,0,0])
        for walker in walker_list:
            _distance = math.sqrt((self._vehicle.get_location().x - walker.get_location().x)**2 + (self._vehicle.get_location().y - walker.get_location().y)**2)
            x = -(self._vehicle.get_location().x - walker.get_location().x)
            y = -(self._vehicle.get_location().y - walker.get_location().y)
            z = -(self._vehicle.get_location().z - walker.get_location().z)
            _angles = np.dot(np.array([x,y,z]),matrix)
            _angle = _angles[1]/_angles[0]
            if _distance < self.speed or _distance < 10:
                hazard_detected = True
                distance = _distance
                if abs(_angle) < abs(angle):
                    angle = _angle
                    angles = _angles
                    ped = np.dot(np.linalg.inv(self.matrix_transform), np.array(
                        [walker.get_location().x, walker.get_location().y, walker.get_location().z, 1]))
                    self.walker = np.array([walker.get_location().x, walker.get_location().y, walker.get_location().z, 1])
        """
        x = self.GlobaltoLocalVehicle(self._vehicle)[0]
        end1 = self.LocaltoGlobal(np.array([x[0] + 10 + self.speed, x[1] + (10+self.speed)*float(10/17), x[2]+2, 1]))
        end2 = self.LocaltoGlobal(np.array([x[0] + 10 + self.speed, x[1] - (10+self.speed)*float(10/17), x[2]+2, 1]))
        self._world.debug.draw_line(self._vehicle.get_location(), carla.Location(end1), life_time = 0.0001)
        self._world.debug.draw_line(self._vehicle.get_location(), carla.Location(end2), life_time = 0.0001)
        """
        # check possible obstacles
        vehicle_state, vehicle = self._is_vehicle_hazard(vehicle_list)
        if vehicle_state:
            if debug:
                print('!!! VEHICLE BLOCKING AHEAD [{}])'.format(vehicle.id))

            self._state = _AgentState.BLOCKED_BY_VEHICLE
            hazard_detected = True

        # check for the state of the traffic lights
        """
        light_state, traffic_light = self._is_light_red(lights_list)
        if light_state:
            if debug:
                print('=== RED LIGHT AHEAD [{}])'.format(traffic_light.id))

            self._state = _AgentState.BLOCKED_RED_LIGHT
            hazard_detected = True
        """
        if hazard_detected and abs(angle) < 0.5 and angles[0] > 0:
            if self.speed < 15 and ped[1] != 0:
                self.speed = self.speed
            else:
                self.speed = self.speed-self.get_break(self.speed, distance)
                if self.speed < 0:
                    self.speed = 0
            velocity = (velocity/norm(velocity))*self.speed
            control = carla.Vector3D(velocity[0], velocity[1], velocity[2])
            self.last = hazard_detected
        else:
            if self.speed < 60:
                self.increase_speed()
            velocity = (velocity/norm(velocity))*self.speed
            control = carla.Vector3D(velocity[0], velocity[1], velocity[2])
            self.last = hazard_detected
            self.walker = None
        """
        if hazard_detected and self.speed >= 10 and abs(angle) < 0.3 and angles[0]>0:
            # control = self.emergency_stop()
            # if self.speed > 10:
            self.speed = self.speed-self.get_break(self.speed,distance)
            if self.speed < 0:
                self.speed = 0
            velocity = (velocity/norm(velocity))*self.speed
            control = carla.Vector3D(velocity[0],velocity[1],velocity[2])

        elif hazard_detected and distance<30 and distance>10 and self.speed >= 5 and abs(angle) < 0.3 and angles[0]>0:
            # if self.speed > 5:
            control = self.emergency_stop()
            self.speed = self.speed-self.get_break(self.speed,distance)*0.1
            if self.speed < 0:
                self.speed = 0
            velocity = (velocity/norm(velocity))*self.speed
            control = carla.Vector3D(velocity[0],velocity[1],velocity[2])

        elif hazard_detected and distance<10 and self.speed > 0 and abs(angle) < 0.3 and angles[0]>0:
            return carla.Vector3D(0,0,0)

        elif hazard_detected:
            velocity = (velocity/norm(velocity))*self.speed
            control = carla.Vector3D(velocity[0],velocity[1],velocity[2])
            return control

        else:
            if self.speed<60:
                self.increase_speed()
            # self._state = _AgentState.NAVIGATING
            # standard local planner behavior
            # control = self._local_planner.run_step(debug=debug)
            velocity = (velocity/norm(velocity))*self.speed
            control = carla.Vector3D(velocity[0],velocity[1],velocity[2])
            # control = carla.Vector3D(0,0,0)
        # print(self.speed)"""
        return control

    def done(self):
        """
        Check whether the agent has reached its destination.
        :return bool
        """
        return self._local_planner.done()

    def get_location(self):
        return self._vehicle.get_location()

    def check_end(self,location):
        # matrix = self.get_matrix(self._vehicle.get_transform())
        x = np.dot(np.linalg.inv(self.matrix_transform), np.array(
            [location.x, location.y, location.z, 1]))
        if x[0] < 0:
            return True
        else:
            return False
    def get_break(self,c,distance):
        f = 2*((20+c*100*0.2)/distance)**2
        if c>1:
            return f/200
        else:
            return 0

    def increase_speed(self):
        self.speed += 1

    def check_infront():
        pass

    def get_matrix(self, transform): #local transfer to global
        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))

        matrix = np.array(np.identity(4))

        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p*c_y
        matrix[0, 1] = c_y*s_p*s_r - s_y*c_r
        matrix[0, 2] = -c_y*s_p*c_r - s_y*s_r
        matrix[1, 0] = s_y*c_p
        matrix[1, 1] = s_y*s_p*s_r + c_y*c_r
        matrix[1, 2] = -s_y*s_p*c_r + c_y*s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p*s_r
        matrix[2, 2] = c_p*c_r
        return matrix

    def GlobaltoLocalVehicle(self, target):
        location = np.dot(np.linalg.inv(self.matrix_transform), np.array([target.get_location().x, target.get_location().y, target.get_location().z, 1]))
        velocity = np.dot(np.linalg.inv(self.matrix_transform), np.array([target.get_velocity().x, target.get_velocity().y, target.get_velocity().z, 0]))
        return (location, velocity)

    def LocaltoGlobal(self, velocity):
        trans = np.dot(self.matrix_transform, velocity)
        return carla.Vector3D(trans[0], trans[1], trans[2])

def get_matrix(transform): #local transfer to global
    rotation = transform.rotation
    c_y = np.cos(np.radians(rotation.yaw))
    s_y = np.sin(np.radians(rotation.yaw))
    c_r = np.cos(np.radians(rotation.roll))
    s_r = np.sin(np.radians(rotation.roll))
    c_p = np.cos(np.radians(rotation.pitch))
    s_p = np.sin(np.radians(rotation.pitch))

    matrix = np.array(np.identity(3))

    matrix[0, 0] = c_p*c_y
    matrix[0, 1] = c_y*s_p*s_r - s_y*c_r
    matrix[0, 2] = -c_y*s_p*c_r - s_y*s_r
    matrix[1, 0] = s_y*c_p
    matrix[1, 1] = s_y*s_p*s_r + c_y*c_r
    matrix[1, 2] = -s_y*s_p*c_r + c_y*s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p*s_r
    matrix[2, 2] = c_p*c_r
    return matrix

def global_to_local(transform):
    return np.inv(global_to_local(transform))

def norm(array):
    sum = 0
    for i in array:
        sum += i**2
    return math.sqrt(sum)
