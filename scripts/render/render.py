def is_raspberry_pi():
    try:
        with open("/proc/device-tree/model", "r") as f:
            model = f.read().lower()
        return "raspberry pi" in model
    except FileNotFoundError:
        return False

# terminate if accidently running on the car
if is_raspberry_pi():
	raise Exception("You are running this code on the pi, but we can only render on a laptop!")


# laptop_render.py
import zmq
import numpy as np

import pyvista as pv
import numpy as np
import time
import matplotlib.pyplot as plt
from dvisd_autonomy.networking.TcpSocket import TcpSocketReceiver



def points_to_plane(points, wall_height=1):
    point1 = points[0]
    point2 = points[1]
    lower1 = [point1[0], point1[1], 0]
    lower2 = [point2[0], point2[1], 0]
    upper1 = [point1[0], point1[1], wall_height]
    upper2 = [point2[0], point2[1], wall_height]
    return np.array([lower1, lower2, upper1, upper2])


def extend_points(point1, point2, max_render_distance=5):
    point1 = np.array(point1, dtype=float)
    point2 = np.array(point2, dtype=float)

    direction = point2 - point1
    norm_dir = np.linalg.norm(direction)
    if norm_dir == 0:
        return point1, point2

    direction /= norm_dir  # normalize

    # Quadratic coefficients: ||p + t*d||^2 = R^2
    def solve_t(p, d):
        a = 1  # because d is normalized
        b = 2 * np.dot(p, d)
        c = np.dot(p, p) - max_render_distance ** 2
        discriminant = b ** 2 - 4 * a * c
        if discriminant < 0:
            return 0, 0
        t1 = (-b + np.sqrt(discriminant)) / 2
        t2 = (-b - np.sqrt(discriminant)) / 2
        return t1, t2

    t1, t2 = solve_t(point1, direction)
    # Use one t for point1 in negative direction, other t for point2 in positive
    new_point1 = point1 + t2 * direction
    new_point2 = point1 + t1 * direction

    return new_point1, new_point2

class CarVisualizer:
    def __init__(self,
                 car_length=0.4,
                 car_width=0.2,
                 base_height=0.1,
                 x_lims=(-10, 10),
                 y_lims=(-10, 10),
                 z_lims=(0, 5)):
        # Hyperparameters
        self.car_length = car_length
        self.car_width = car_width
        self.base_height = base_height
        self.x_lims = x_lims
        self.y_lims = y_lims
        self.z_lims = z_lims

        # Create PyVista plotter
        self.plotter = pv.Plotter()
        self.plotter.set_background("white")
        # self.plotter.show_grid()

        # --- Add ground ---
        self._render_ground()

        # --- Add car mesh ---
        self.car_mesh = self._create_car_mesh()
        self.car_actor = self.plotter.add_mesh(self.car_mesh, color="blue")

        # --- Add empty point cloud ---
        self.pc_mesh_red = self._create_pc_mesh()
        self.pc_actor_red = self.plotter.add_points(self.pc_mesh_red, color="red", point_size=5)
        self.pc_mesh_blue = self._create_pc_mesh()
        self.pc_actor_blue = self.plotter.add_points(self.pc_mesh_blue, color="blue", point_size=5)
        self.pc_mesh_green = self._create_pc_mesh()
        self.pc_actor_green = self.plotter.add_points(self.pc_mesh_green, color="green", point_size=5)
        self.hidden_points = np.tile(np.array([[0.0, 0.0, 0.0]]), (360, 1))

        # --- Create waypoint mesh ---
        self.waypoint_mesh = self._create_pc_mesh(count=1)
        self.waypoint_actor = self.plotter.add_points(self.waypoint_mesh, color="purple", point_size=10)

        # ---  Add guiding lines mesh ---
        self.gl_meshes = [self._create_line_mesh() for _ in range(20)]
        self.gl_actors = [self.plotter.add_mesh(mesh, color="grey", line_width=3) for mesh in self.gl_meshes]

        # --- Add path line meshes
        self.path_meshes = [self._create_line_mesh() for _ in range(20)]
        self.path_actors = [self.plotter.add_mesh(mesh, color="purple", line_width=3) for mesh in self.path_meshes]


        # --- Add estimated walls mesh ---
        self.wall_mesh1 = self._create_wall_mesh()
        self.wall_actor1 = self.plotter.add_mesh(self.wall_mesh1, color="blue", opacity=0.5)
        self.wall_mesh2 = self._create_wall_mesh()
        self.wall_actor2 = self.plotter.add_mesh(self.wall_mesh2, color="green", opacity=0.5)

        # Set camera
        self.plotter.camera_position = [( -6, 0, 3), (0,0,0), (0,0,3)]

        # Show plotter window
        self.plotter.show(auto_close=False, interactive_update=True)

    # ------------------ Modular rendering ------------------ #
    def _render_ground(self):
        # Ground plane at z=0
        x = np.linspace(self.x_lims[0], self.x_lims[1], 2)
        y = np.linspace(self.y_lims[0], self.y_lims[1], 2)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)
        grid = pv.StructuredGrid(X, Y, Z)
        self.plotter.add_mesh(grid, color="lightgray", opacity=0.5)

    def _create_line_mesh(self):
        start = [0, 0, self.base_height]
        end = [0, 0, self.base_height]
        line_mesh = pv.Line(start, end)
        return line_mesh

    def _create_pc_mesh(self, count=360):
        """Create an empty point cloud mesh"""
        points = np.zeros((count, 3))
        pc_mesh = pv.PolyData(points)
        return pc_mesh
    def _create_car_mesh(self):
        """Create a triangular prism mesh for the car at origin"""
        l = self.car_length / 2
        w = self.car_width / 2
        h = self.base_height

        # Vertices: front top/bottom, rear left/right top/bottom
        points = np.array([
            [ l, 0, 0],       # front bottom
            [-l,  w, 0],      # rear left bottom
            [-l, -w, 0],      # rear right bottom
            [ l, 0, h],       # front top
            [-l,  w, h],      # rear left top
            [-l, -w, h],      # rear right top
        ])

        # Faces: PyVista needs a special format: [n, p0, p1, ..., pn-1]
        faces = [
            [3, 0,1,2],  # bottom triangle
            [3, 3,5,4],  # top triangle (note flipped for correct normal)
            [4, 0,1,4,3],  # left side
            [4, 0,2,5,3],  # right side
            [4, 1,2,5,4]   # rear face
        ]
        # Flatten faces for PyVista
        faces_flat = np.hstack(faces)
        mesh = pv.PolyData(points, faces_flat)
        return mesh

    def _create_wall_mesh(self):
        point1 = [0, 0]
        point2 = [0, 0]
        points = points_to_plane([point1, point2])
        faces = [4,0,1,3,2]
        faces_flat = np.hstack(faces)
        mesh = pv.PolyData(points, faces_flat)
        return mesh


    # ------------------ Update functions ------------------ #
    def clear_old_points(self, color):
        if color == "red":
            self.pc_mesh_red.points = self.hidden_points
        elif color == "blue":
            self.pc_mesh_blue.points = self.hidden_points
        elif color == "green":
            self.pc_mesh_green.points = self.hidden_points


    def update_pointcloud(self, points, color):
        assert type(points) == np.ndarray, "Points must be a numpy array"
        assert points.ndim == 2
        """Update LIDAR point cloud"""
        # Ensure points have 3 columns
        if points.shape[1] == 2:
            points_3d = np.hstack([
                points,
                np.full((points.shape[0],1), self.base_height),
            ])
        else:
            points_3d = points

        # mirror along y (side to side dimension)
        points_3d[:, 1] *= -1

        # the number of points in points_3d varies, so we need to extend it with hidden points
        new_points = self.hidden_points.copy()
        new_points[:points_3d.shape[0]] = points_3d

        # Update points
        if color == "red":
            self.pc_mesh_red.points = new_points
        elif color == "blue":
            self.pc_mesh_blue.points = new_points
        elif color == "green":
            self.pc_mesh_green.points = new_points


    def update_waypoint(self, new_waypoint):
        assert type(new_waypoint) == np.ndarray, "Points must be a numpy array"
        assert new_waypoint.ndim == 1
        if new_waypoint.shape[0] == 2:
            points_3d = np.concatenate([
                new_waypoint,
                np.array([self.base_height]),
            ])
        else:
            points_3d = new_waypoint
        # mirror along y (side to side dimension)
        points_3d[1] *= -1
        self.waypoint_mesh.points = points_3d

    def update_guiding_lines(self, endpoints):
        assert type(endpoints) == np.ndarray, "Endpoints must be a numpy array"
        assert endpoints.ndim == 2, "Endpoints must be Nx2 array"
        assert endpoints.shape[0] <= len(self.gl_meshes)
        assert endpoints.shape[1] == 2, "Endpoints must be 2D (x,y)"

        # mirror along y (side to side dimension)
        endpoints[..., 1] *= -1

        """Update guiding lines"""
        # update seen lines
        for i in range(endpoints.shape[0]):
            start = [0, 0, self.base_height]
            end = [endpoints[i][0], endpoints[i][1], self.base_height]
            self.gl_meshes[i].points = np.array([start, end])

        # hide unused lines by making them have zero length.
        for i in range(endpoints.shape[0], len(self.gl_meshes)):
            start = [0, 0, 0]
            end = [0,0,0]
            self.gl_meshes[i].points = np.array([start, end])

    def update_path(self, path):
        assert type(path) == np.ndarray, "Endpoints must be a numpy array"
        assert path.ndim == 2, "Endpoints must be Nx2 array"
        assert path.shape[0] - 1 <= len(self.path_meshes)
        assert path.shape[1] == 2, "Endpoints must be 2D (x,y)"

        # mirror along y (side to side dimension)
        path[..., 1] *= -1

        """Update guiding lines"""
        # update seen lines
        for i in range(0, path.shape[0] - 1):
            start = [path[i, 0], path[i, 1], self.base_height]
            end = [path[i+1, 0], path[i+1, 1], self.base_height]
            self.path_meshes[i].points = np.array([start, end])

        # hide unused lines by making them have zero length.
        for i in range(path.shape[0] - 1, len(self.path_meshes)):
            start = [0, 0, 0]
            end = [0, 0, 0]
            self.path_meshes[i].points = np.array([start, end])


    def update_estimated_walls(self, walls):
        """Placeholder for updating estimated walls (not implemented)"""
        assert type(walls) == np.ndarray, "Walls must be a numpy array"
        assert walls.ndim == 3, "Walls must be a 3D array of shape (2, 2, 2)"
        assert walls.shape[0] == 2, "There must be exactly 2 walls"
        assert walls.shape[1] == 2, "Each wall must have 2 endpoints"
        assert walls.shape[2] == 2, "Endpoints must be 2D (x,y)"
        # mirror along y (side to side dimension)
        walls[..., 1] *= -1

        # Do first wall
        wall1 = walls[0]
        point1 = wall1[0]
        point2 = wall1[1]
        point1, point2 = extend_points(point1, point2)
        points = points_to_plane([point1, point2])
        self.wall_mesh1.points = points

        # Do second wall
        wall2 = walls[1]
        point1 = wall2[0]
        point2 = wall2[1]
        point1, point2 = extend_points(point1, point2)
        points = points_to_plane([point1, point2])
        self.wall_mesh2.points = points

    def update(
            self,
            points_red=None,
            points_blue=None,
            points_green=None,
            endpoints=None,
            walls=None,
            waypoint=None,
            path=None,
    ):
        """Convenience method to update both"""

        if points_red is not None:
            self.update_pointcloud(points_red, "red")


        if points_blue is not None:
            self.update_pointcloud(points_blue, "blue")


        if points_green is not None:
            self.update_pointcloud(points_green, "green")



        if endpoints is not None:
            self.update_guiding_lines(endpoints)


        if walls is not None:
            self.update_estimated_walls(walls)

        if waypoint is not None:
            self.update_waypoint(waypoint)

        if path is not None:
            self.update_path(path)

        self.plotter.update()


hostname = {
    1: {
        "ut": "b827eb777a0f.dynamic.utexas.edu",
        "radio": "192.168.168.11",
         "philip": "192.168.0.101"
    },
    2: {
        "ut": "b827eb2f3fa5.dynamic.utexas.edu",
        "radio": "192.168.168.12",
         "philip": "192.168.0.102",
    },
    3: {
        "ut": "b827ebfbf87a.dynamic.utexas.edu",
        "radio": "192.168.168.13",
        "philip": "192.168.0.103"
    },
    4: {
        "ut": "nan",
        "radio": "nan",
        "philip": "192.168.0.104"
    },
}


if __name__ == "__main__":
    # Chosen car number, and where we are for deciding on hostname
    car = 1
    wifi_type = "philip"
    assert wifi_type in ["ut", "radio", "philip"], "You must select either 'ut' or 'radio' or 'philip'. Case sensitive. "

    # create renderer
    viz = CarVisualizer()

    # create receiver
    hostname = hostname[car][wifi_type]

    # Create reciever. Listens for messages from the provided hostname.
    receiver = TcpSocketReceiver(hostname)

    # constantly render
    while 1:
        # Receive frame bytes
        msg = receiver.receive()

        # Update visualization
        viz.update(**msg)
        time.sleep(0.03)


