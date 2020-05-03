from awconnection import RobotConnection
import time

r = RobotConnection()
r.connect()

while True:

    r.lock_info()

    print(r.info.timestamp)

    r.unlock_info()

    """now_pos = info.gps.position
    now_time = info.timestamp

    delta_millis = now_time - last_time

    if (delta_millis >= 1):

        speed = (now_pos - last_pos).magnitude() / delta_millis * 1000
        latency = str(time.time() * 1000 - now_time)
        print("SPEED: " + str(speed))
        print("LATENCY: " + latency)


        last_pos = now_pos
        last_time = now_time"""

r.disconnect()
