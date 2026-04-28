from PIL import Image
import glob

# Create a list of image objects
frames = []
imgs = sorted(glob.glob("./figs/Schumacher_anim/*.png")) # Get and sort images

for i in imgs:
    new_frame = Image.open(i)
    frames.append(new_frame)

# Save into a GIF file
frames[0].save('Schumacher_animation.gif', format='GIF',
               append_images=frames[1:],
               save_all=True,
               duration=15, # Duration of each frame in milliseconds
               loop=0)       # 0 means loop forever
