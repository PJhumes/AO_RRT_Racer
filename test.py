import matplotlib.pyplot as plt
import shapely.plotting
from shapely.geometry import Polygon

# Define a square with a square hole
polygon = Polygon(
    shell=[(0, 0), (10, 0), (10, 10), (0, 10)],
    holes=[[(2, 2), (2, 8), (8, 8), (8, 2)]]
)

fig, ax = plt.subplots()
shapely.plotting.plot_polygon(polygon, ax=ax, facecolor='lightblue', edgecolor='blue')
plt.show()