import warnings
warnings.filterwarnings("ignore", message="pkg_resources is deprecated")
import pygame 
import shapely.geometry as geom
from shapely.prepared import prep
import numpy as np
import Formula_E


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
# Outer boundary and inner hole (the grass)
outer_coords = generate_oval_track(400, 300, 250, 250)
inner_coords = generate_oval_track(400, 300, 200, 150)

# The 'track' is the area between the outer wall and the inner hole
track_poly = geom.Polygon(shell=outer_coords, holes=[inner_coords])
# Prep the geometry for 50x faster performance
prepared_track = prep(track_poly)

# --- 2. PYGAME SETUP ---
pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()

racecar = Formula_E.Formula_E(100, 100, 0)

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
        racecar.v = min(5, racecar.v+0.1)  

    elif keys[pygame.K_DOWN]:  
        racecar.v = max(-5/2, racecar.v-0.2)

    else: # Slowly coast to a stop
        if racecar.v > 0: racecar.v = max(0, racecar.v-0.03)
        if racecar.v < 0: racecar.v = min(0, racecar.v+0.03)
    

    # theta_dot = car_v/CAR_L*np.atan(phi)
    # pos[0] += car_v*np.cos(theta)
    # pos[1] += car_v*np.sin(theta)
    # theta += theta_dot

    car_box = racecar.update_pos()
    
    # r_mat = np.array([[np.cos(theta), -np.sin(theta)],
    #                   [np.sin(theta),  np.cos(theta)]])

    # car =  [
    #         r_mat @ np.array([0,  CAR_W]) + pos, # Rear left
    #         r_mat @ np.array([0, -CAR_W]) + pos, # Rear right
    #         r_mat @ np.array([2*CAR_L, -CAR_W]) + pos, # Front left
    #         r_mat @ np.array([2*CAR_L,  CAR_W]) + pos, # Front right
    #        ]

        
    

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
    pygame.draw.circle(screen, (0, 0, 200), racecar.state.pos(), 5)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
