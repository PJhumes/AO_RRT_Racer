import warnings
warnings.filterwarnings("ignore", message="pkg_resources is deprecated")
import pygame 
import shapely.geometry as geom
from shapely.prepared import prep
import numpy as np
import Formula_E
import Track_Collisions
import pandas as pd

window_size = (1200, 800)
track = Track_Collisions.Track("IMS", window_size=window_size)
display_stats = True

print(f"Track Scale: {track.scale} px/m (<- check units)")
print(f"Scaled Track Length: {track.centerline.length / track.scale}")


# --- 2. PYGAME SETUP ---
pygame.init()
screen = pygame.display.set_mode(window_size)
clock = pygame.time.Clock()
text = pygame.font.SysFont('Arial', 24)
framerate = 60

racecar = Formula_E.Formula_E(track.start[0], track.start[1], track.start_ang, track.scale, framerate)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Move car with arrow keys
    keys = pygame.key.get_pressed()
    # Turning Motion
    if keys[pygame.K_LEFT]:  racecar.phi = np.radians(-1)
    elif keys[pygame.K_RIGHT]: racecar.phi = np.radians(1)
    else:
        if racecar.phi > 0: racecar.phi = max(0, racecar.phi-0.05)
        if racecar.phi < 0: racecar.phi = min(0, racecar.phi+0.05)
        
    # Throttle/Brake
    if keys[pygame.K_UP]:
        f, p_pos = racecar.f_acc(1)
    elif keys[pygame.K_DOWN]:  
        f, p_pos = racecar.f_acc(-1)

    else: # Slowly coast to a stop
        f = -(racecar.f_drag(racecar.v) + racecar.f_roll(racecar.v))
        p_pos = 0
        
    racecar.accelerate(f)

    # --- 3. COLLISION LOGIC ---
    car_box = racecar.update_pos() # Apply euler timestep and recalculate hitbox
    car_hitbox = geom.polygon.Polygon(car_box)

    # Check if the car is entirely WITHIN the track polygon
    is_colliding = not track.prepared.contains(car_hitbox)
    car_color = (255, 0, 0) if is_colliding else (0, 255, 0)

    # --- 4. DRAWING ---
    screen.fill((30, 30, 30)) # Background
    
    # Draw Track: Outer Wall (Gray) and Inner Wall (Green Grass)
    pygame.draw.polygon(screen, (80, 80, 80), track.outer_coords)
    pygame.draw.polygon(screen, (34, 139, 34), track.inner_coords)
    pygame.draw.polygon(screen, (0, 0, 0), track.goal.exterior.coords)

    # Draw Car
    pygame.draw.polygon(screen, car_color, car_box)
    # pygame.draw.circle(screen, (0, 0, 200), racecar.state.pos(), 1*scale)

    if display_stats:
        velocity_display = text.render("Vel: " + str(round(racecar.v, 3)), True, (255, 255, 255))
        force_display = text.render("Force: " + str(round(f, 3)), True, (255, 255, 255))
        goal_display = text.render("Goal Reached: " + str(track.goal_reached(car_hitbox)), True, (255, 255, 255))
        
        screen.blit(velocity_display, (10, 10))
        screen.blit(force_display, (10, 40))
        screen.blit(goal_display, (10, 70))

    pygame.display.flip()
    clock.tick(framerate)

pygame.quit()
