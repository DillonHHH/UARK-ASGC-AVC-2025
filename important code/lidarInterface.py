from dataclasses import dataclass
import custom_rplidar
import numpy as np
import math
import warnings
from dataTypes import *
import time
from typing import Callable, Union

PORT_NAME = "/dev/ttyUSB0"
DMAX = 4000
IMIN = 0
IMAX = 50

DISTANCE_MULTIPLIER = 1 / 1000  # convert from mm to m


@dataclass
class DAQ:
    """Contains the 3 components of a lidar scan; Distance, Angle, Quality"""

    distance: float
    angle_degrees: float
    quality: float

    def to_relative_location(self) -> tuple[float, float]:
        """Returns location of scan in relative coordinates, should always be used when comparing locations to the camera.\n
        Location is offset because the camera is considered the "center" of our vehicle
        """
        return (
            self.distance * math.sin(math.radians(self.angle_degrees)),
            # - CAMERA_OFFSET_FROM_LIDAR_CM[0] / 100,
            self.distance * math.cos(math.radians(self.angle_degrees)),
            # - CAMERA_OFFSET_FROM_LIDAR_CM[1] / 100,
        )

    def to_global_location(
        self, car_x: float, car_y: float, car_theta: float
    ) -> tuple[float, float]:
        """Transforms the LIDAR scan from relative coordinates to global coordinates using the car's pose."""
        x_rel, y_rel = self.to_relative_location()  # Get relative coordinates

        # Apply 2D rotation and translation to convert to global coordinates
        x_global = car_x + x_rel * math.cos(car_theta) - y_rel * math.sin(car_theta)
        y_global = car_y + x_rel * math.sin(car_theta) + y_rel * math.cos(car_theta)

        # print(f"carxglob: {car_x}")
        # print(f"caryglob: {car_y}")
        # print(f"xglob: {x_global}")
        # print(f"yglob: {y_global}")

        return x_global, y_global

    def is_within_range(self, min_angle_degrees: float, max_angle_degrees: float):
        """Returns true if lidar scan is between min and max, where min values can be > max values, eg:\n
        returns true when self.angle_degrees is 5 when min = 350 and max = 10"""
        if min_angle_degrees <= 0 or min_angle_degrees >= 360:
            raise Exception(f"Min must be between 0 and 360, was {min_angle_degrees}")
        if max_angle_degrees <= 0 or max_angle_degrees >= 360:
            raise Exception(f"Max must be between 0 and 360, was {max_angle_degrees}")

        if min_angle_degrees > max_angle_degrees:
            return (
                self.angle_degrees > min_angle_degrees
                or self.angle_degrees < max_angle_degrees
            )
        else:
            return (
                self.angle_degrees < min_angle_degrees
                or self.angle_degrees > max_angle_degrees
            )


def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return (line,)


class LidarInterface:
    def __init__(self) -> None:
        self.reconnect()

    def get_scans(
        self,
        callback_on_fail: Union[Callable, None] = None,
        callback_on_restart: Union[Callable, None] = None,
    ) -> list[DAQ]:
        while True:
            try:
                assert self.lidar._serial is not None
                data_in_buffer = self.lidar._serial.in_waiting
                return_data: list[DAQ] = []
                for _ in range(0, math.floor(data_in_buffer / self.dsize)):
                    raw = self.lidar._read_response(self.dsize)
                    scan = custom_rplidar._process_scan(raw)
                    return_data.append(
                        DAQ(scan[3] * DISTANCE_MULTIPLIER, scan[2], scan[1])
                    )
                return return_data
            except Exception as e:
                if callback_on_fail:
                    callback_on_fail()
                self.reconnect()
                if callback_on_restart:
                    callback_on_restart()

    def reconnect(self):
        while True:
            try:
                self.lidar = custom_rplidar.RPLidar(PORT_NAME)
                print("starting motor attempt")
                self.lidar.start_motor()
                print("starting")
                self.lidar.start()
                self.dsize = self.lidar.scanning[1]
                break
            except Exception as e:
                print(f"Lidar setup failed due to: {e}\nRetrying...")
                # time.sleep(1)
                self.disconnect()
                # time.sleep(1)
                continue

    def disconnect(self):
        try:
            self.lidar.stop_motor()
        except Exception as e:
            print(e)
        try:
            self.lidar.stop()
        except Exception as e:
            print(e)
        try:
            self.lidar.disconnect()
        except Exception as e:
            print(e)


# def run():

#     # lidar.start_motor()
#     # fig = plt.figure()
#     # ax = plt.subplot(111, projection="polar")
#     # line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX], cmap=plt.cm.Greys_r, lw=0)
#     # ax.set_rmax(DMAX)
#     # ax.grid(True)

#     # iterator = lidar.iter_scans()
#     # ani = animation.FuncAnimation(fig, update_line, fargs=(iterator, line), interval=50)
#     # plt.show()
#     # lidar.stop()
#     # lidar.disconnect()

#     print("new loop")

#     LidarInterface.start_motor()

#     LidarInterface.start()

#     dsize = LidarInterface.scanning[1]

#     while True:
#         # lidar.stop_motor()


#         # for scan in lidar.iter_measures(scan_type="normal", max_buf_meas=500):
#         #     if scan[2] > 0 and scan[2] < 5:
#         #         # print(f"q: {scan[1]}, a: {scan[2]}, d: \n{scan[3]}, ")
#         #         pass


# if __name__ == "__main__":
#     run()
