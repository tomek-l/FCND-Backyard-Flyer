import argparse
import time
from enum import Enum

import numpy as np
import math

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMED = 1
    TAKEOFF_COMPLETE = 2
    WAYPOINTS = 3
    LANDED = 4
    DISARMED = 5


class BackyardFlyer(Drone):
    def __init__(self, connection):
        super().__init__(connection)
        self.in_mission = True
        self.check_state = {}
        self.wp_list = self.calculate_box()
        self.wp_idx = 0
        self.target_position = self.wp_list[0]
        self.state = States.MANUAL
        self.waypoints_completed = False
        self.eps = 1.0  # m

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    @staticmethod
    def euc_dist(a, b):
        sumsq = (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] + b[2]) ** 2
        return math.sqrt(sumsq)

    @staticmethod
    def calculate_box(size=10, altitude=5):
        return [
            [0, 0, altitude],
            [size, 0, altitude],
            [size, size, altitude],
            [0, size, altitude],
            [0, 0, altitude],
        ]

    def local_position_callback(self):
        # wasn't needed
        pass

    def velocity_callback(self):
        # wasn't needed
        pass

    def state_callback(self):
        if not self.in_mission:
            return
        if self.state == States.MANUAL:
            self.arming_transition()
        elif self.state == States.ARMED:
            self.takeoff_transition()
        elif self.state == States.TAKEOFF_COMPLETE:
            self.waypoint_transition()
        elif self.state == States.WAYPOINTS:
            self.waypoint_transition()
            self.landing_transition()
        elif self.state == States.LANDED:
            self.disarming_transition()
        elif self.state == States.DISARMED:
            self.manual_transition()

    def arming_transition(self):
        self.take_control()
        self.set_home_position(
            self.global_position[0], self.global_position[1], self.global_position[2]
        )

        self.arm()
        self.state = States.ARMED
        print("arming transition")

    def takeoff_transition(self):
        alt = 5
        self.target_position[2] = alt
        self.takeoff(alt)
        self.state = States.TAKEOFF_COMPLETE
        print("takeoff transition")

    def waypoint_transition(self):
        if (self.euc_dist(self.local_position, self.target_position) < self.eps):  # if reached waypoint

            if self.wp_idx + 1 == len(self.wp_list):  # if reached last waypoint
                pass  # let landing_transition take care of this
            else:
                self.wp_idx += 1
                self.target_position = self.wp_list[self.wp_idx]
                self.cmd_position(*self.target_position, heading=0)
                print(f"set target to {self.target_position}")
                self.state = States.WAYPOINTS  # stay in waypoint
        print("waypoint transition")

    def landing_transition(self):
        if (
            self.euc_dist(self.local_position, self.target_position) < self.eps
        ):  # if reached waypoint
            if self.wp_idx + 1 == len(self.wp_list):  # if reached last waypoint
                self.land()
                self.state = States.LANDED
                self.target_position = [0, 0, 0]
        print("landing transition")

    def disarming_transition(self):
        if (
            self.euc_dist(self.local_position, self.target_position) < self.eps
        ):  # if reached waypoint
            self.disarm()
            self.state = States.DISARMED
        print("disarm transition")

    def manual_transition(self):
        self.release_control()
        self.stop()
        self.in_mission = False
        self.state = States.MANUAL
        print("manual transition")

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5760, help="Port number")
    parser.add_argument(
        "--host", type=str, default="127.0.0.1", help="host address, i.e. '127.0.0.1'"
    )
    args = parser.parse_args()

    conn = MavlinkConnection(
        "tcp:{0}:{1}".format(args.host, args.port), threaded=False, PX4=False
    )
    # conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
