import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as spline

# Read waypoints from CSV
data = np.loadtxt("/sim_ws/src/pure_pursuit/waypoints/waypoints.csv", delimiter=',')
x_values = data[:, 0]
y_values = data[:, 1]

# Make spline of waypoints
tck, u = spline.splprep([x_values, y_values], k=2, s=1)
u_new = np.linspace(u.min(), u.max(), 1000)
interpolated_points = spline.splev(u_new, tck)
spline_points = np.array(spline.splev(u_new, tck))

# print("Spline Points:")
# print(spline_points)
# print("X values:", spline_points[0])
# print("Y values:", spline_points[1])

spline_x_values, spline_y_values = spline_points

# Plot the waypoints and the spline
plt.scatter(x_values, y_values, label='Waypoints')
plt.plot(spline_x_values, spline_y_values, label='Spline')
plt.title('Waypoints and Spline')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()

# Save the plot as a PNG file
plt.savefig('/sim_ws/src/pure_pursuit/waypoints/waypoints_and_spline.png')
print("Done printing the waypoint and the spline.")
# Show the plot if needed (uncomment the line below)
# plt.show()