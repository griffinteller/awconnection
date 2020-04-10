import threading
import time
import json
import getpass
import platform


class RobotConnection(object):

    SEND_INTERVAL = 0.01  # in seconds
    GET_INTERVAL = 0.01
    QUEUE_INTERVAL = 0.01

    def __init__(self):

        if platform.system() == "Windows":

            data_dir = "C:\\Users\\" + getpass.getuser() + "\\AppData\\Local\\ABR\\"

        else:

            data_dir = "/Users/" + getpass.getuser() + "/.local/share/ABR/"

        self.queue_path = data_dir + "EventQueue"
        self.state_path = data_dir + "RobotState.json"
        self.event_buffer = []  # where events that need to be sent are stored
        self.sensors = dict()  # where robot state json is parsed into
        self.should_destroy = False  # should this connection end
        self.event_buffer_lock = threading.Lock()  # lock for threads wanting to access event_buffer
        self.sensors_lock = threading.Lock()

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

                state_file = open(self.state_path, "r")
                with self.sensors_lock:

                    self.sensors = json.load(state_file)

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
