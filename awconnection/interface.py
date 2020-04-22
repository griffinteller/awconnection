import threading
import time
import json
import getpass
import platform
import math


class Vector3:

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z

    def __init___(self, vector_dict):

        self.x = vector_dict["x"]
        self.y = vector_dict["y"]
        self.z = vector_dict["z"]

    def magnitude(self):
        return math.sqrt(float(self.x ** 2.0 + self.y ** 2.0 + self.z ** 2.0))

    def normalized(self):
        magnitude = self.magnitude()
        return Vector3(self.x / magnitude, self.y / magnitude, self.z / magnitude)


class Sensor:

    def __init__(self, info_dict):
        self.update(info_dict)

    def update(self, info_dict):
        pass


class Altimeter(Sensor):

    def __init__(self, info_dict):

        self.altitude = None
        super().__init__(info_dict)

    def update(self, info_dict):

        self.altitude = info_dict["altimeter"]["altitude"]


class GPS(Sensor):

    def __init__(self, info_dict):

        self.position = None
        super().__init__(info_dict)

    def update(self, info_dict):

        position_dict = info_dict["gps"]["position"]
        self.position = Vector3(position_dict)


class Gyroscope(Sensor):

    def __init__(self, info_dict):

        self.forward = None
        self.up = None
        self.right = None
        super().__init__(info_dict)

    def update(self, info_dict):

        gyroscope_dict = info_dict["gyroscope"]
        self.forward = Vector3(gyroscope_dict["forward"])
        self.up = Vector3(gyroscope_dict["up"])
        self.right = Vector3(gyroscope_dict["right"])

class Radar(Sensor):

    def __init__(self, info_dict):

        self.pings = None
        self.it_ping = None
        super().__init__(info_dict)

    def update(self, info_dict):

        radar_dict = info_dict["radar"]

        self.pings = []

        for ping in radar_dict["pings"]:

            self.pings.append(Vector3(ping))

        self.it_ping = radar_dict["itPing"]

class LiDAR(Sensor):

    def __init__(self, info_dict):

        self.distance_matrix = None
        super().__init__(info_dict)

    def update(self, info_dict):

        container_array = info_dict["lidar"]["distanceMatrix"]  # unity can't serialize 2D arrays (grr) so we have
                                                                # to create an array of ArrayContainers, which each
                                                                # each have a field "array" containing a row.

        self.distance_matrix = []
        for container in container_array:

            self.distance_matrix.append(container["array"])


class RobotInfo:

    # I know this looks like bad programming, but until the two functions are actually different, there is no point
    # defining everything as None and then initializing it. If I just call update, python gets cranky.

    def __init__(self, info_dict):

        self.altimiter = Altimeter(info_dict)
        self.gps = GPS(info_dict)
        self.gyroscope = Gyroscope(info_dict)
        self.lidar = LiDAR(info_dict)
        self.radar = Radar(info_dict)

        self.timestamp = info_dict["timestamp"]
        self.isIt = info_dict["isIt"]
        self.gamemode = info_dict["gameMode"]

    def update(self, info_dict):

        self.altimiter.update(info_dict)
        self.gps.update(info_dict)
        self.gyroscope.update(info_dict)
        self.lidar.update(info_dict)
        self.radar.update(info_dict)

        self.timestamp = info_dict["timestamp"]
        self.isIt = info_dict["isIt"]
        self.gamemode = info_dict["gameMode"]


class RobotConnection:

    SEND_INTERVAL = 0.01  # in seconds
    GET_INTERVAL = 0.01
    QUEUE_INTERVAL = 0.01

    def __init__(self):

        if platform.system() == "Windows":
            data_dir = "C:\\Users\\" + getpass.getuser() + "\\AppData\\Local\\ABR\\"

        else:
            data_dir = "/Users/" + getpass.getuser() + "/.local/share/ABR/"

        self.queue_path = data_dir + "EventQueue"
        self.info_path = data_dir + "RobotState.json"
        self.event_buffer = []  # where events that need to be sent are stored
        self.info = None  # type: RobotInfo
        self.should_destroy = False  # should this connection end
        self.event_buffer_lock = threading.Lock()  # lock for threads wanting to access event_buffer
        self.info_lock = threading.Lock()

    def queue_event(self, event_text):

        with self.event_buffer_lock:
            self.event_buffer.append(event_text + "\n")

        time.sleep(RobotConnection.QUEUE_INTERVAL)

    def flip_coordinates(self):
        self.queue_event("COORDINATES flip")

    def set_tire_torque(self, tire_name, torque):
        self.queue_event("SET tire " + tire_name + " " + str(torque))

    def set_tire_steering(self, tire_name, bering):
        self.queue_event("SET steering " + tire_name + " " + str(bering))

    def disconnect(self):
        self.should_destroy = True

    def send_buffer_thread(self):

        while len(self.event_buffer) > 0 or not self.should_destroy:  # continue until connection is ended

            try:  # in case game is currently cleaning file
                queue_file = open(self.queue_path, "a")

            except EnvironmentError:  # any sort of io error
                continue  # try again until unity is done

            with self.event_buffer_lock:  # acquire buffer lock

                for event in self.event_buffer:
                    queue_file.write(event)

                self.event_buffer = []  # reset buffer

            queue_file.close()  # flush file buffer and hand over lock to game
            time.sleep(RobotConnection.SEND_INTERVAL)  # we don't need to do this too often (1-2 times per frame)

    def get_robot_state_thread(self):

        while not self.should_destroy:

            try:

                state_file = open(self.info_path, "r")

                with self.info_lock:

                    info_dict = json.load(state_file)
                    if self.info:
                        self.info.update(info_dict)
                    else:
                        self.info = RobotInfo(info_dict)

            except (EnvironmentError, json.JSONDecodeError) as e:
                continue

            state_file.close()
            time.sleep(RobotConnection.GET_INTERVAL)

    def connect(self):

        send_thread = threading.Thread(target=self.send_buffer_thread)
        get_thread = threading.Thread(target=self.get_robot_state_thread)
        send_thread.start()
        get_thread.start()
        time.sleep(1)

    def set_all_tire_torques(self, torque):
        self.queue_event("SET all_tire " + str(torque))
