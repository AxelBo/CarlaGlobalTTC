import math
import time
from shapely.geometry import Polygon
import carla
import datetime
import connectionCarla

import subprocess


def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return 9999, 9999
        # raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


def calc_bounding_box(p0, angle, length, width):
    # This function calculates the bounding box of a rectangle given its starting point (p0), orientation angle, length,
    # and width. It returns a Shapely Polygon representing the bounding box.
    if abs(length) < 0.1:
        length = 0.1
    x = p0[0] + length * math.cos(angle)
    y = p0[1] + length * math.sin(angle)
    p_mid_ahead = (x, y)

    x = p0[0] + width * math.cos(angle - math.pi / 2)
    y = p0[1] + width * math.sin(angle - math.pi / 2)
    point1 = (x, y)

    x = p0[0] + width * math.cos(angle + math.pi / 2)
    y = p0[1] + width * math.sin(angle + math.pi / 2)
    point2 = (x, y)

    x = p_mid_ahead[0] + width * math.cos(angle - math.pi / 2)
    y = p_mid_ahead[1] + width * math.sin(angle - math.pi / 2)
    point3 = (x, y)

    x = p_mid_ahead[0] + width * math.cos(angle + math.pi / 2)
    y = p_mid_ahead[1] + width * math.sin(angle + math.pi / 2)
    point4 = (x, y)
    return Polygon([point1, point2, point4, point3])


def dist_xy(pos1, pos2):
    return math.sqrt(
        (pos1[0] - pos2[0]) ** 2 +
        (pos1[1] - pos2[1]) ** 2
    )


def velocity_3d_to_ms(velocity3d):
    return math.sqrt(velocity3d.x ** 2 + velocity3d.y ** 2 + velocity3d.z ** 2)


def velocity_3d_diff(velocity3d, velocity3d2):
    v_x = velocity3d.x - velocity3d2.x
    v_y = velocity3d.y - velocity3d2.y
    v_z = velocity3d.z - velocity3d2.z
    return math.sqrt(v_x ** 2 + v_y ** 2 + v_z ** 2)


def on_collision(event):
    other_actor = event.other_actor
    print("Collison")
    if 'walker' in other_actor.type_id and 'vehicle' in event.self_actor.type_id:
        print(f"Collision between {event.self_actor.type_id} and {other_actor.type_id}")


def distance_actor(actor1, actor2):
    try:
        pos1 = actor1.get_location()
        pos2 = actor2.get_location()
        distance = math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)
        return distance
    except:
        print("Fail")
        return 999


class CarlaTimeToCollision:

    def __int__(self, vehicles_list, walkers_list, connection, run):
        self.walkers_list = walkers_list
        self.run_id = run
        self.vehicles_list = []
        for vehicle in vehicles_list:
            self.vehicles_list.append([vehicle, True])
        # self.vehicles_list = vehicles_list
        self.connection = connection
        self.ttcList = []
        self.max_distance_ttc = 50  # in meter
        self.timeframe = 0
        self.faildedCars = 0
        self.faildedWaler = 0
        self.sensorlist = []
        self.pathList = []
        self.fotoCount = 0
        self.collision_sensor_bp = connection.world.get_blueprint_library().find('sensor.other.collision')
        # === TTC Calculation ===
        self.timeLookAhead = 1.5
        self.lengthWalker = 0.187679 * 2
        self.widthWalker = 0.187679
        self.lengthVehicle = 2.395890
        self.widthVehicle = 1.081725

    def addCollisionSensors(self):
        for actor in self.walkers_list:
            if 'walker' in actor.type_id:
                collision_sensor = self.connection.world.spawn_actor(self.collision_sensor_bp, carla.Transform(),
                                                                     attach_to=actor)
                collision_sensor.listen(lambda event: on_collision(event))

    def ttc_collision_search(self, timetoCollisionSearch, timeLookAhead, vel1, vel2, rot1, rot2, posWalker, posVehicle):
        while timetoCollisionSearch < timeLookAhead:
            length = vel1 * timetoCollisionSearch
            length2 = vel2 * timetoCollisionSearch
            x = posWalker[0] + length * math.cos(rot1)
            y = posWalker[1] + length * math.sin(rot1)
            x2 = posVehicle[0] + length2 * math.cos(rot2)
            y2 = posVehicle[1] + length2 * math.sin(rot2)

            polyWalker = calc_bounding_box(p0=[x, y], angle=rot1, length=self.lengthWalker, width=self.widthWalker)
            polyCar = calc_bounding_box(p0=[x2, y2], angle=rot2, length=self.lengthVehicle, width=self.widthVehicle)

            intersectionAhead = polyWalker.intersects(polyCar)

            if intersectionAhead:
                return timetoCollisionSearch

            if timetoCollisionSearch < 0.1:
                timetoCollisionSearch += 0.01
            else:
                timetoCollisionSearch += 0.1
        return 999

    def ttc(self):
        for index in range(len(self.vehicles_list)):
            ### Find all near by vehicles
            vehicle, alive = self.vehicles_list[index]
            if not alive:
                continue
            if not vehicle.is_alive:
                print("Vehicle not Alive :(")
                self.vehicles_list[index][1] = False
                continue
            near_walkers = [walker for walker in self.walkers_list if
                            distance_actor(vehicle, walker) < self.max_distance_ttc]
            for actor in near_walkers:
                try:
                    posWalker = (actor.get_location().x, actor.get_location().y)
                    posVehicle = (vehicle.get_location().x, vehicle.get_location().y)

                    rotWalker = math.radians(actor.get_transform().rotation.yaw)
                    rotVehicle = math.radians(vehicle.get_transform().rotation.yaw)

                    velWalker = velocity_3d_to_ms(actor.get_velocity())
                    velVehicle = velocity_3d_to_ms(vehicle.get_velocity())

                    # New Pos for Walker, where center of Walker is in the back middle
                    posWalker = [posWalker[0] - self.lengthWalker / 2 * math.cos(rotWalker),
                                 posWalker[1] - self.lengthWalker / 2 * math.sin(rotWalker)]

                    length = velWalker * self.timeLookAhead + self.lengthWalker
                    length2 = velVehicle * self.timeLookAhead + self.lengthVehicle
                    poly1 = calc_bounding_box(p0=posWalker, angle=rotWalker, length=length, width=self.widthWalker)
                    poly2 = calc_bounding_box(p0=posVehicle, angle=rotVehicle, length=length2,
                                              width=self.widthVehicle)
                    intersection = poly1.intersects(poly2)
                    time_to_collision = 0
                    if intersection:
                        # Check if collision in first halt
                        length = velWalker * self.timeLookAhead / 2 + self.lengthWalker
                        length2 = velVehicle * self.timeLookAhead / 2 + self.lengthVehicle
                        poly1 = calc_bounding_box(p0=posWalker, angle=rotWalker, length=length,
                                                  width=self.widthWalker)
                        poly2 = calc_bounding_box(p0=posVehicle, angle=rotVehicle, length=length2,
                                                  width=self.widthVehicle)
                        if poly1.intersects(poly2):
                            time_to_collision = self.ttc_collision_search(0, self.timeLookAhead / 2, velWalker,
                                                                          velVehicle, rotWalker, rotVehicle,
                                                                          posWalker,
                                                                          posVehicle)
                        else:
                            time_to_collision = self.ttc_collision_search(self.timeLookAhead / 2,
                                                                          self.timeLookAhead, velWalker, velVehicle,
                                                                          rotWalker,
                                                                          rotVehicle,
                                                                          posWalker,
                                                                          posVehicle)
                    else:
                        time_to_collision = 999
                    if time_to_collision < 1.5:
                        w_time = self.connection.world.get_snapshot().timestamp.elapsed_seconds
                        # --------------
                        # Spawn attached RGB camera
                        # --------------
                        cam_bp = None
                        cam_bp = self.connection.world.get_blueprint_library().find('sensor.camera.rgb')
                        cam_bp.set_attribute("image_size_x", str(1920))
                        cam_bp.set_attribute("image_size_y", str(1080))
                        cam_bp.set_attribute("fov", '110')
                        cam_location = carla.Location(x=-2, y=0, z=3)
                        cam_transform = carla.Transform(cam_location)
                        # try:

                        velocity = vehicle.get_velocity()
                        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
                        if speed > 0.5:
                            path = 'tmp/'+ self.run_id +'_fotos/' + self.run_id + "_" + str(time_to_collision) + "_" + str(w_time) + "_" + str(
                                vehicle.id) + ".png"
                            if not sysncModeEnable:
                                camera = CarlaCamera(vehicle, path, self.connection)


                            # ego_cam = self.connection.world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle,
                            #                                             attachment_type=carla.AttachmentType.Rigid)

                            # self.pathList.append(path)
                            # self.sensorlist.append(ego_cam)
                            # ego_cam.listen(lambda image: self.saveImage(image))
                            filename = self.run_id+"_array_data.txt"
                            with open(filename, "a") as file:
                                file.write(str([
                                    vehicle.id,
                                    vehicle.get_location().x,
                                    vehicle.get_location().y,
                                    actor.id,
                                    actor.get_location().x,
                                    actor.get_location().y,
                                    time_to_collision,
                                    self.timeframe,
                                    speed
                                ]) + "\n")
                            self.ttcList.append([
                                vehicle.id,
                                vehicle.get_location().x,
                                vehicle.get_location().y,
                                actor.id,
                                actor.get_location().x,
                                actor.get_location().y,
                                time_to_collision,
                                self.timeframe,
                                speed
                                # w_time,
                            ])
                        # location = carla.Location(vehicle.get_location().x, vehicle.get_location().y, 3)
                        # self.connection.draw_waypoint(location=location, index='x', life_time=60)
                except Exception as e:
                    # handle the exception by printing its name
                    print("Caught an exception:", type(e).__name__)
                    print("Error message:", str(e))

    def saveImage(self, image):
        if (self.fotoCount % 1 == 0):
            print('foto?')
        # np.image_converter.to_rgb_array(image)
        self.fotoCount += 1
        path = self.pathList.pop(0)
        self.sensorlist[0].destroy()
        del self.sensorlist[0]
        image.save_to_disk(path)

    def getAllActorsFromWorld(self):
        allAcotrs = self.connection.world.get_actors()
        self.vehicles_list = []
        self.walkers_list = []
        otherAcots = []

        for actor in allAcotrs:
            if "vehicle" in actor.type_id:
                self.vehicles_list.append(actor)
            elif "walker" in actor.type_id:
                self.walkers_list.append(actor)
            else:
                otherAcots.append(actor)
        print("Found: ",
              len(self.vehicles_list), "vehicles and ",
              len(self.walkers_list) / 2, "walkers and ",
              len(otherAcots), "other Acors")

    def recordingCarla(self, savepath):
        savepath = "/" + savepath
        print("Recording on file: ",
              self.connection.client.start_recorder(
                  savepath, True))

class CarlaCamera:

    def __init__(self, vehicle, path, con):
        self.vehicle = vehicle
        self.camera_bp = None
        self.camera = None
        self.path = path
        self.connection = con
        if self.camera is None:
            self.attach_camera()
        self.camera.listen(lambda data: self.process_image(data))

    def attach_camera(self):
        self.camera_bp = self.connection.world.get_blueprint_library().find('sensor.camera.rgb')
        self.camera_bp.set_attribute("image_size_x", str(1920))
        self.camera_bp.set_attribute("image_size_y", str(1080))
        self.camera_bp.set_attribute("fov", '110')
        cam_location = carla.Location(x=-2, y=0, z=3)
        cam_transform = carla.Transform(cam_location)
        self.camera = self.connection.world.spawn_actor(self.camera_bp, cam_transform, attach_to=self.vehicle,
                                                        attachment_type=carla.AttachmentType.Rigid)

    def process_image(self, image):
        image.save_to_disk(self.path)
        self.destroy()

    def destroy(self):
        if self.camera is not None:
            self.camera.destroy()


if __name__ == '__main__':

    for run in range(6):
        x = datetime.datetime.now()
        timestamp = str(x.year) + "_" + str(x.month) + "_" + str(x.day) + "_" + str(x.hour) + "_" + str(x.minute)

        cars = 50
        walker = 60
        simulationTime = 1 * 60
        stepsCalculations = 0.05
        #### Connection to Carla
        conncetion = connectionCarla.CarlaConnection()
        sysncModeEnable = True
        conncetion.__int__(reloadWorld=True, syncMode=sysncModeEnable, render=False, delta=0.05)

        #### Spawn Walker and Cars
        spawn = connectionCarla.CarlaSpawning()
        spawn.__int__(conncetion)
        spawn.spawnVehicles(cars)
        spawn.spawnWalker(walker)

        ttc = CarlaTimeToCollision()
        ttc.__int__(spawn.vehicles_list, spawn.walkers_list, conncetion, str(run))

        savepath = str(run) + "_" +timestamp + "_steps-" + str(int(simulationTime / stepsCalculations)) + "_Car-" + str(
            cars) + "_Walk-" + str(walker) + ".log"
        time.sleep(5)

        ttc.recordingCarla(savepath)
        for i in range(int(simulationTime / stepsCalculations)):
            # try:

            ttc.ttc()
            if sysncModeEnable:
                conncetion.world.tick()
            else:
                # time.sleep(stepsCalculations)
                ...
            # Print time every 60 secound
            if i % int(600) == 0:
                # conncetion.reinitialize()
                print("Timeframe: ", ttc.timeframe)
            ttc.timeframe += stepsCalculations
        print(ttc.ttcList)
        savepath = "./ttc_list_" + savepath
        f = open(savepath, "w")
        f.write('%s' % ttc.ttcList)
        f.close()
        ttc.connection.client.stop_recorder()
        try:
            # Beenden der Anwendung auf Windows
            subprocess.call(['taskkill', '/f', '/im', 'CarlaUE4-Win64-Shipping.exe'])
            time.sleep(1)
            subprocess.call(['taskkill', '/f', '/im', 'CarlaUE4.exe'])
        except:
            print("Fehler beim Beenden von Carla")
        finally:
            time.sleep(5)
    # Beenden der Anwendung auf Windows
    subprocess.call(['taskkill', '/f', '/im', 'CarlaUE4-Win64-Shipping.exe'])
    print("All Done!!!")

    exit()
