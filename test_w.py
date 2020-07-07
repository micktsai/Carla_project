import glob
import os
import sys
import math
import random
import time
import pygame
import argparse
import logging

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
        world.debug.draw_string(spawn_points[i].location, '%s'%i, draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=120.0,persistent_lines=True)
    input()
    """
    #make a car
    bp = blueprint_library.filter('model3')[0]
    bp.set_attribute('color', '0,0,0')
    waypoint = spawn_points[83] #84
    vehicle = world.spawn_actor(bp, waypoint)
    actor_list.append(vehicle)
    agent = TryAgent(vehicle)
    agent_list.append([vehicle, agent])

    bp = blueprint_library.filter('model3')[0]
    bp.set_attribute('color', '0,0,0')
    waypoint = spawn_points[57] #58
    vehicle = world.spawn_actor(bp, waypoint)
    actor_list.append(vehicle)
    agent = TryAgent(vehicle)
    agent_list.append([vehicle, agent])

    #make pedestrian
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=0,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=1,
        type=int,
        help='number of walkers (default: 50)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '-tm_p', '--tm_port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    args = argparser.parse_args()
    blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)
    percentagePedestriansRunning = 0.0      # how many pedestrians will run
    percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
    SpawnActor = carla.command.SpawnActor
    walkers_list = []
    all_id = []
    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(1):
        spawn_point = carla.Transform()
        #loc = world.get_random_location_from_navigation()
        loc = carla.Location(-113.6, 4, 1)
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put altogether the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)
	    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    if not args.sync or not synchronous_master:
        world.wait_for_tick()
    else:
        world.tick()
    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        #all_actors[i].go_to_location(world.get_random_location_from_navigation())
        all_actors[i].go_to_location(carla.Location(-113.6, -16.8, 1))
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))
#---------------------------------Control Part------------------------------------#
    # input()
    while True:
        for vehicle, agent in agent_list:
            #vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0, hand_brake=True))
            control = agent.run_step()
            vehicle.set_velocity(control)
            # control.manual_gear_shift = False
            # vehicle.apply_control(control)
            # vehicle.set_velocity(control)

finally:
    for actor in actor_list:
        actor.destroy()
    for actor in all_actors:
        actor.destroy()
    print("All cleaned up!")
