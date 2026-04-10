import warnings
warnings.filterwarnings("ignore", message="pkg_resources is deprecated")
import pygame 
import shapely.geometry as geom
from shapely.prepared import prep
import numpy as np
import Formula_E
import pandas as pd
rand = np.random.default_rng()


class Track():
    def __init__(self, track_name="IMS", angle=88.5, window_size=(1200, 800)):
        """Tracks are loaded based on GPS Data and physical orientation. Some need to be rotated to fit well.
            IMS: 88.5, Monza: 90, Silverstone: -80 (Automatically taken care of. Other tracks default to 88.5)
            IMS is Indianapolis Motor Speedway, 4000 (ish) meter, Rounded-corner square track used for nascar and indycar.
            We will use it for testing."""
        self.window_size = window_size

        if track_name == "Monza":
            angle = 90
        elif track_name == "Silverstone":
            angle = -80

        track_data = self.load_track(track_name, angle)

        self.centerline = geom.linestring.LineString(track_data[0])
        self.outer_coords = track_data[1] 
        self.inner_coords = track_data[2] 
        self.gradients = track_data[3] 
        self.scale = track_data[4]

        self.prepared = prep(geom.Polygon(shell=self.outer_coords, holes=[self.inner_coords]))
        self.start = self.centerline.coords[0]
        self.start_ang = np.atan2(self.gradients[0,1], self.gradients[0,0])

        self.goal = geom.Polygon([self.inner_coords[-1], self.inner_coords[-2], self.outer_coords[-2], self.outer_coords[-1]])

    def load_track(self, track_name:str, angle=0.):
        """
        Load from TU Munich racetrack-database format.
        Columns: x_m, y_m, w_tr_right_m, w_tr_left_m
        """
        df = pd.read_csv("track_info/tracks/" + track_name + ".csv", comment='#', header=None, names=['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])    
        centerline = list(zip(df['x_m'], df['y_m']))
        
        # Reconstruct inner/outer boundaries
        # (requires computing track normal vectors)
        coords = np.array(centerline)
        coords[:, 1] *= -1 # Flip Y for pygame coordinates.

        angle = np.radians(angle)
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle),  np.cos(angle)]])
        coords = coords @ rotation_matrix.T  # Apply rotation
        
        # Translate to origin (shift min x/y to 0)
        coords -= np.min(coords, axis=0)

        # Compute bounding box after initial translation
        minx, miny = np.min(coords, axis=0)
        maxx, maxy = np.max(coords, axis=0)
        width = maxx - minx
        height = maxy - miny

        # Define screen and margins
        screen_width, screen_height = self.window_size
        margin_x, margin_y = 20, 20  # Adjust as needed for padding

        # Compute scale factor (uniform, preserving aspect ratio)
        scale_x = (screen_width - 2 * margin_x) / width
        scale_y = (screen_height - 2 * margin_y) / height
        scale = min(scale_x, scale_y)

        # Apply scaling
        coords *= scale
        # coords[:, 1] *= -1

        # Center the track on the screen
        bbox_center_x = (np.min(coords[:, 0]) + np.max(coords[:, 0])) / 2
        bbox_center_y = (np.min(coords[:, 1]) + np.max(coords[:, 1])) / 2
        screen_center_x = screen_width / 2
        screen_center_y = screen_height / 2
        offset_x = screen_center_x - bbox_center_x
        offset_y = screen_center_y - bbox_center_y
        coords += np.array([offset_x, offset_y])


        gradients = np.gradient(coords, axis=0)
        normals = np.column_stack([gradients[:,1], -gradients[:,0]])
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = normals / norms

        outer = coords + normals * df['w_tr_right_m'].values[:,None]
        inner = coords - normals * df['w_tr_left_m'].values[:,None]

        return coords, outer.tolist(), inner.tolist(), gradients, scale

    def is_colliding(self, hitbox:geom.Polygon):
        return not self.prepared_track.contains(hitbox)
    
    def goal_reached(self, hitbox:geom.Polygon):
        return self.goal.intersects(hitbox)
    
    def sample_goal(self):
        minx, miny, maxx, maxy = self.goal.bounds()
        x = rand.uniform(minx, maxx)
        y = rand.uniform(miny, maxy)
        sample = geom.point.Point((x, y))

        if self.goal.contains(sample):
            return sample
        else:
            return self.sample_goal()
        
    def sample_state(self):
        x = rand.uniform(0, self.window_size[0])
        y = rand.uniform(0, self.window_size[1])

        return geom.point.Point((x, y))
        




