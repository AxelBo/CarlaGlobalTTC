#!/usr/bin/env python

import glob
import math
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import random
import time
import numpy as np
import logging
import connectionCarla


class RecordingMode:

    def __int__(self):
        conection = connectionCarla.CarlaConnection()
        conection.__int__()

        self.client = conection.client
        self.world = conection.world
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicles_list = []
        self.walkers_list = []
        self.collision_Sensors = []
        self.collision_locations = []
        self.spawn_points = self.world.get_map().get_spawn_points()

    def distance_p1_p2(self, a, b):
        x = (b[0] - a[0]) ** 2
        y = (b[1] - a[1]) ** 2
        return math.sqrt(x + y)

    def collision_handler(self, event):
        # 'actor', 'frame', 'frame_number', 'normal_impulse', 'other_actor', 'timestamp', 'transform']
        """ handle collisions and calculate extra reward """
        actor = event.actor
        actor_we_collide_against = event.other_actor

        # To ensure that the initial force does not count as collision
        location = event.transform.location

        frame_number = event.frame_number

        collision = (actor.id, location, actor_we_collide_against, frame_number)
        print(collision)
        add_coll = True

        for col in self.collision_locations:
            if add_coll == False:
                continue
            if col[0] == actor.id:
                # if self.distance_p1_p2([col[1].x, col[1].y], [location.x, location.y]) < 20.0:
                add_coll = False
        if add_coll:
            self.collision_locations.append(collision)

    def spawnVehicles(self, number_of_cars=30):
        car_bp = self.blueprint_library.filter('vehicle.*.*')
        tm = self.client.get_trafficmanager(8000)
        tm_port = tm.get_port()
        for i in range(number_of_cars):
            bp = random.choice(car_bp)
            vehicle = self.world.spawn_actor(bp, self.spawn_points[-i])
            self.vehicles_list.append(vehicle)
        for vehicle in self.vehicles_list:
            vehicle.set_autopilot(True, tm_port)

        for vehicle in self.vehicles_list:
            collision_sensor_car = self.world.spawn_actor(
                self.blueprint_library.find('sensor.other.collision'),
                carla.Transform(), attach_to=vehicle)
            collision_sensor_car.listen(lambda event: self.collision_handler(event))
            self.collision_Sensors.append(collision_sensor_car)

    def spawnWalker(self, number_of_walkers=50):

        blueprintsWalkers = self.blueprint_library.filter('walker.*.*')
        all_id = []

        SpawnActor = carla.command.SpawnActor

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 1.0  # how many pedestrians will run
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
        results = self.client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                self.walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(self.walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                self.walkers_list[i]["con"] = results[i].actor_id
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(self.walkers_list)):
            all_id.append(self.walkers_list[i]["con"])
            all_id.append(self.walkers_list[i]["id"])
        all_actors = self.world.get_actors(all_id)

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
        len(self.vehicles_list), len(self.walkers_list)))

        # example of how to use parameters
        # traffic_manager.global_percentage_speed_difference(30.0)

    def printCollisionLocations(self):
        time.sleep(2)
        print("============================================================== ")

        if self.collision_locations is None:
            print("No Collisions")
        else:
            print("Print all Collisions: ")
            print("--------------------------------------------------------------- ")
            for col in self.collision_locations:
                print(col[0], col[1])

    def identMap(self):
        location = carla.Location(x=100, y=100, z=5)
        self.draw_waypoint(location, "x100, y100")

        location = carla.Location(x=0, y=0, z=5)
        self.draw_waypoint(location, "0, 0")

        location = carla.Location(x=200, y=200, z=5)
        self.draw_waypoint(location, "200, 200")

    def draw_waypoint(self, location, index, life_time=120.0):

        self.world.debug.draw_string(location, str(index), draw_shadow=False,
                                     color=carla.Color(r=255, g=0, b=0), life_time=life_time,
                                     persistent_lines=True)

    def recordingCarla(self):
        print("Recording on file: ",
              self.client.start_recorder("/home/imech031/.config/Epic/CarlaUE4/Saved/recorder4.log", True))

    def saveCollionDataJSON(self):
        f = open("./collisionData.log", "w")
        f.write("{\n")
        for action in self.collision_locations:
            f.write("{\n")
            f.write('"id": %s,\n' % action[0])
            f.write('"x": %s,\n' % action[1].x)
            f.write('"y": %s\n' % action[1].y)
            f.write("},")
        f.write("}")
        f.close()

    def saveCollionData(self):
        f = open("./collisionData4.log", "w")
        for action in self.collision_locations:
            f.write('%s,\n' % action[0])
            f.write('%s,\n' % action[1].x)
            f.write('%s\n' % action[1].y)
            f.write("\n")

        f.close()


if __name__ == '__main__':
    cc = RecordingMode()
    cc.__int__()
    cc.spawnVehicles(80)
    cc.spawnWalker(60)
    cc.recordingCarla()
    time.sleep(7200)
    cc.client.stop_recorder()
    print("Stop recording")
    cc.printCollisionLocations()
    cc.saveCollionData()
