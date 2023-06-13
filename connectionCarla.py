#!/usr/bin/env python

import glob
import logging
import os
import random
import sys

import numpy as np

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import time


class CarlaConnection():
    def __int__(self, townName="Town03", host="localhost", reloadWorld=True,
                camaraLocation=carla.Location(x=-200, y=0, z=150), camRotation=None, render=True, syncMode=False, port=2000, delta=0.05, setCamerView=True):
        # === Carla ===
        self.host = host
        self.town = townName
        self.port = port
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(20.0)
        self.world = self.client.get_world()

        self.map = self.world.get_map()
        if not self.map.name.endswith(self.town):
            self.world = self.client.load_world(self.town)
            while not self.world.get_map().name.endswith(self.town):
                time.sleep(0.2)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
        time.sleep(2)
        if reloadWorld:
            self.client.reload_world(False)
        if syncMode:
            self.setSynchronous_mode(no_render=not render, reloadWorld=reloadWorld, delta=delta),
        if setCamerView:
            self.set_camara_view(camaraLocation, camRotation)

    def reinitialize(self):
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(20.0)
        self.world = self.client.get_world()
        time.sleep(1)

    def setSynchronous_mode(self, delta=0.05, no_render=True, reloadWorld=True):
        settings = self.world.get_settings()
        settings.synchronous_mode = True  # Enables synchronous mode
        settings.fixed_delta_seconds = delta
        settings.no_rendering_mode = no_render
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 5
        self.world.apply_settings(settings)
        if reloadWorld:
            self.client.reload_world(False)

    def set_camara_view(self, location, camRotation):
        # === Walker View Camera ===
        # X und Y sind gedreht so....
        spectator = self.world.get_spectator()
        location = location
        transform = carla.Transform(location, self.world.get_map().get_spawn_points()[0].rotation)
        if not camRotation:
            camRotation = carla.Rotation(pitch=-60)
        spectator.set_transform(carla.Transform(transform.location, camRotation))

    def draw_waypoint(self, location, index, life_time=120.0, intensity=1):
        if intensity < 0.33:
            color = carla.Color(r=0, g=255, b=0)
        elif intensity < 0.66:
            color = carla.Color(r=250, g=250, b=0)
        else:
            color = carla.Color(r=250, g=0, b=0)
        self.world.debug.draw_string(location, str(index), draw_shadow=False,
                                     color=color, life_time=life_time,
                                     persistent_lines=True)

    def getVehicleList(self):
        actors = self.world.get_actors().filter('vehicle.*')
        return [actor for actor in actors]

    def getWalkerList(self):
        actors = self.world.get_actors().filter('walker.*')
        return [actor for actor in actors]


class CarlaSpawning():
    def __int__(self, conection):
        self.client = conection.client
        self.world = conection.world
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicles_list = []
        self.walkers_list = []
        self.collision_Sensors = []
        self.collision_locations = []
        self.spawn_points = self.world.get_map().get_spawn_points()

    def spawnVehicles(self, number_of_cars=30):
        car_bp = self.world.get_blueprint_library().filter('vehicle.tesla.model3')[0]

        tm = self.client.get_trafficmanager(8000)

        # tm.global_percentage_speed_difference(100)

        tm_port = tm.get_port()
        for i in range(number_of_cars):
            bp = car_bp
            vehicle = self.world.spawn_actor(bp, self.spawn_points[-i])
            self.vehicles_list.append(vehicle)
        for vehicle in self.vehicles_list:
            vehicle.set_autopilot(True, tm_port)

    def spawnWalker(self, number_of_walkers=50):

        blueprintsWalkers = self.world.get_blueprint_library().filter('0012')[0]
        all_id = []
        walkers_list_local = []
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.50  # how many pedestrians will run
        percentagePedestriansCrossing = 0.3  # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(number_of_walkers):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            loc.z += 1
            for sp in spawn_points:
                x = np.linalg.norm(
                    np.array((loc.x, loc.y, loc.z)) - np.array((sp.location.x, sp.location.y, sp.location.z)))
                if x < 1.0:
                    print("near sp")

            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        max_speed = 15/3.5
        for spawn_point in spawn_points:
            walker_bp = blueprintsWalkers
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                    walker_speed.append(max_speed * ((random.random()+1)/2))
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = self.client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list_local.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list_local)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list_local[i]["id"]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list_local[i]["con"] = results[i].actor_id
        # 4. we put altogether the walkers and controllers id to get the objects from their id

        for i in range(len(walkers_list_local)):
            all_id.append(walkers_list_local[i]["con"])
            all_id.append(walkers_list_local[i]["id"])
        all_actors = self.world.get_actors(all_id)
        self.walkers_list = all_actors
        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        # if not args.sync or not synchronous_master:
        #     world.wait_for_tick()
        # else:
        #     world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (
            len(self.vehicles_list), len(walkers_list_local)))

        # example of how to use parameters
        # traffic_manager.global_percentage_speed_difference(30.0)
