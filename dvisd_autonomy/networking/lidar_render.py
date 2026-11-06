import numpy as np
import matplotlib.pyplot as plt
import time
from dvisd_autonomy.networking.tcp_socket import TcpSocketReceiver


def is_raspberry_pi():
    try:
        with open("/proc/device-tree/model", "r") as f:
            model = f.read().lower()
        return "raspberry pi" in model
    except FileNotFoundError:
        return False


# terminate if accidentally running on the car
if is_raspberry_pi():
    raise Exception("You are running this code on the pi, but we can only render on a laptop!")


class LidarTopDownVisualizer:
    def __init__(self, x_lims=(-5, 5), y_lims=(-5, 5)):
        self.x_lims = x_lims
        self.y_lims = y_lims

        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.ax.set_xlim(*x_lims)
        self.ax.set_ylim(*y_lims)
        self.ax.set_aspect('equal')
        self.ax.set_title("Top-Down LIDAR View")

        # scatter plots
        self.scatter = self.ax.scatter([], [], s=10)

        # draw origin (car position)
        self.ax.plot(0, 0, 'ko')  # black dot at origin

        self.heading = self.ax.quiver(
            0, 0,   # origin
            0.5, 0,   # direction (x=1, y=0)
            angles='xy',
            scale_units='xy',
            scale=0.75,
            color='r',
            width=0.0075
        )

        plt.grid()

        plt.show()

    def update_points(self, points, scatter):
        if points is None or len(points) == 0:
            scatter.set_offsets(np.empty((0, 2)))
            return

        points = np.asarray(points)

        # ensure Nx2
        if points.shape[1] > 2:
            points = points[:, :2]

        # mirror y (keep your original convention)
        points[:, 1] *= -1

        scatter.set_offsets(points)

    def update(self, points=None):
        self.update_points(points, self.scatter)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


hostname_map = {
    1: {
        "ut": "b827eb777a0f.dynamic.utexas.edu",
        "radio": "192.168.168.11"
    },
    2: {
        "ut": "b827eb2f3fa5.dynamic.utexas.edu",
        "radio": "192.168.168.12"
    },
    3: {
        "ut": "b827ebfbf87a.dynamic.utexas.edu",
        "radio": "192.168.168.13"
    },
}


if __name__ == "__main__":
    car = 3
    wifi_type = "radio"
    assert wifi_type in ["ut", "radio"]

    viz = LidarTopDownVisualizer()

    hostname = hostname_map[car][wifi_type]
    receiver = TcpSocketReceiver(hostname)

    while True:
        msg = receiver.receive()

        if len(msg) > 0:
            viz.update(
                points=msg.get("points"),
            )

        time.sleep(0.03)