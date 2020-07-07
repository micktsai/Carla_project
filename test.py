import glob
import os
import sys
import math
import random
import time
import pygame

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
from carla import Transform, Location, Rotation
from carla import Map
from carla import Vector3D
from carla import ColorConverter as cc
from try_agent import TryAgent
from agents.tools.misc import distance_vehicle, draw_waypoints

actor_list = []
agent_list = []

#--------------------------------Initialization-----------------------------------#
try:
	##General connection to server and get blue_print
    client = carla.Client('localhost',2000)
    client.set_timeout(20.0)

    # world = client.load_world('Town05')
    world = client.get_world()
    mp = world.get_map() #get the map of the current world.
    blueprint_library = world.get_blueprint_library()

    #weather
    weather = carla.WeatherParameters(cloudiness=0.0,precipitation=100.0,sun_altitude_angle=70.0)
    world.set_weather(weather)
    spawn_points = mp.get_spawn_points()
    """
    f=open('waypoint.txt','w')
    for w in spawn_points:
        f.write(str(w.location))
        f.write("\n")
    f.close()

    for i in range(len(spawn_points)):
        world.debug.draw_string(spawn_points[i].location, '%s'%i, draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000.0,persistent_lines=True)
    input()
    # """
    #make a car
    bp = blueprint_library.filter('model3')[0]
    bp.set_attribute('color', '0,0,0')
    waypoint = spawn_points[83] #84
    vehicle = world.spawn_actor(bp, waypoint)
    actor_list.append(vehicle)
    agent = TryAgent(vehicle)
    agent.set_destination([spawn_points[201].location.x,spawn_points[201].location.y,spawn_points[201].location.z])
    agent_list.append([vehicle, agent])

    # """
    bp = blueprint_library.filter('model3')[0]
    bp.set_attribute('color', '0,0,0')
    waypoint = spawn_points[57] #58
    vehicle = world.spawn_actor(bp, waypoint)
    actor_list.append(vehicle)
    agent = TryAgent(vehicle)
    agent_list.append([vehicle, agent])
    """
    #make pedestrian
    bp = blueprint_library.filter('walker.*')[0]
    waypoint = spawn_points[101] # 201 202 101 100
    pedestrian = world.try_spawn_actor(bp, waypoint)
    actor_list.append(pedestrian)

    bp = blueprint_library.filter('walker.*')[0]
    waypoint = spawn_points[103] # 102 103
    pedestrian = world.try_spawn_actor(bp, waypoint)
    actor_list.append(pedestrian)
    # """
#---------------------------------Control Part------------------------------------#
    # input()
    while True:
        for vehicle, agent in agent_list:
            #vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0, hand_brake=True))
            control = agent.run_step()
            #velocity control
            vehicle.set_velocity(control)
            #controller control
            # vehicle.set_controller(control)
            # vehicle.apply_control(control)

finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
