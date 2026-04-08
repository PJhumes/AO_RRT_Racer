import warnings
warnings.filterwarnings("ignore", message="pkg_resources is deprecated")
import pygame 
import shapely.geometry as geom
from shapely.prepared import prep
import numpy as np
import Formula_E
import pandas as pd

def load_track(track_name, angle=0):
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
    screen_width, screen_height = 1200, 800
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

    # Optional: Rotate by a fixed angle (e.g., 45 degrees) if needed for alignment
    # Uncomment and adjust 'angle' as needed
    

    tangents = np.gradient(coords, axis=0)
    normals = np.column_stack([tangents[:,1], -tangents[:,0]])
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / norms

    outer = coords + normals * df['w_tr_right_m'].values[:,None]
    inner = coords - normals * df['w_tr_left_m'].values[:,None]

    return centerline, outer.tolist(), inner.tolist(), scale


def generate_oval_track(center_x, center_y, straight, r, num_points=100):
    points = []
    for i in range(num_points):
        angle = 2 * np.pi * i / num_points
        x = center_x + r * np.cos(angle)
        y = center_y + r * np.sin(angle)
        # print(f"({x}, {y}) {angle}")
        # input()
        if angle < np.pi/2 or angle > 3*np.pi/2:
            x += straight/2
        else:
            x -= straight/2
        points.append((x, y))
    return points


# --- 1. TRACK DESIGN (Shapely) ---
# # Outer boundary and inner hole (the grass)
# outer_coords = generate_oval_track(400, 300, 250, 250)
# inner_coords = generate_oval_track(400, 300, 200, 150)

# Tracks are loaded based on GPS Data and physical orientation. Some need to be rotated to fit well.
# IMS: 88.5, Monza: 90, Silverstone: -80 
# IMS is Indianapolis Motor Speedway, Rounded-corner square track used for nascar and indycar.
centerline, outer_coords, inner_coords, scale = load_track("Silverstone", -80)

print(scale)

# The 'track' is the area between the outer wall and the inner hole
track_poly = geom.Polygon(shell=outer_coords, holes=[inner_coords])
# Prep the geometry for 50x faster performance
prepared_track = prep(track_poly)

# --- 2. PYGAME SETUP ---
pygame.init()
screen = pygame.display.set_mode((1200, 800))
clock = pygame.time.Clock()

racecar = Formula_E.Formula_E(100, 100, 0, scale)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Move car with arrow keys
    keys = pygame.key.get_pressed()
    # Turning Motion
    if keys[pygame.K_LEFT]:  racecar.phi -= .01
    elif keys[pygame.K_RIGHT]: racecar.phi += .01
    else:
        if racecar.phi > 0: racecar.phi = max(0, racecar.phi-0.05)
        if racecar.phi < 0: racecar.phi = min(0, racecar.phi+0.05)
        
    # Throttle/Brake
    if keys[pygame.K_UP]:
        racecar.v = min(racecar.v_max * scale, racecar.v+0.1)  

    elif keys[pygame.K_DOWN]:  
        racecar.v = max(-racecar.v_max/5 * scale, racecar.v-0.2)

    else: # Slowly coast to a stop
        if racecar.v > 0: racecar.v = max(0, racecar.v-0.03)
        if racecar.v < 0: racecar.v = min(0, racecar.v+0.03)
    

    car_box = racecar.update_pos()
    

    # --- 3. COLLISION LOGIC ---
    # Create a small square hitbox for the car
    # CAR_Litbox = geom.box(car[0,0], car[0,1], car[1, 0], car[1, 1]) 
    car_hitbox = geom.polygon.Polygon(car_box)

    # Check if the car is entirely WITHIN the track polygon
    is_colliding = not prepared_track.contains(car_hitbox)
    car_color = (255, 0, 0) if is_colliding else (0, 255, 0)

    # --- 4. DRAWING ---
    screen.fill((30, 30, 30)) # Background
    
    # Draw Track: Outer Wall (Gray) and Inner Wall (Green Grass)
    pygame.draw.polygon(screen, (80, 80, 80), outer_coords)
    pygame.draw.polygon(screen, (34, 139, 34), inner_coords)

    # Draw Car
    pygame.draw.polygon(screen, car_color, car_box)
    # pygame.draw.circle(screen, (0, 0, 200), racecar.state.pos(), 1*scale)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
