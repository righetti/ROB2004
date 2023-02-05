import numpy as np
import matplotlib.pyplot as plt

from matrices_plotting import plot_two_axes, get_rotation_matrix_3d, plot_scatter_point, plot_vector

Aorigin = np.array([0, 0, 0]) # Aorigin is the origin of frame A
# The size of the frame is 1 unit in each direction
X_A = np.array([1, 0, 0])
Y_A = np.array([0, 1, 0])
Z_A = np.array([0, 0, 1])

# ========================

# * offset of B origin from A origin (aka position vector, p)
p = o_9in5 = np.array([[0.55440778], [-0.76436229], [-0.78469475]])

# the code below is to flatten the list of lists for matplotlib plotting requirements 
Borigin_A = []
for sublist in p:
    Borigin_A.extend(sublist)
Borigin_A = Borigin_A + Aorigin


# Rotation matrix provided in the problem
R_x = R_9in5 = np.array([[0.07155755, 0.13053746, 0.98885767], [0.99609913, -0.06067355, -0.06407215], [0.05163369, 0.98958511, -0.13436990]])

# ! Theta and axis provided in the problem
# axis = 'x'
# theta = np.pi/3
# R_x = R_9in5 = get_rotation_matrix_3d(axis, theta)

# point = q_44 = np.array([[-0.39428019], [-0.92702154], [-0.24232918]])

vector = v_5in5 = np.array([[-0.35180568], [0.16130314], [-0.31815458]])

R_5in9 = R_9in5.T # find inverse of R_9in5 by transpose

# translate evertyhing to cooordinate 9
v_9in5 = np.dot(R_5in9, v_5in5)

v_9in5

# ========================

X_B, Y_B, Z_B = np.dot(R_x, [X_A, Y_A, Z_A])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plot_two_axes(ax, Aorigin, X_A, Y_A, Z_A, Borigin_A, X_B, Y_B, Z_B)
# plot_scatter_point(ax, point_1, True, Aorigin, Borigin_A, 'g', marker_shape='o')
# plot_scatter_point(ax, point_1, False, Aorigin, Borigin_A, 'pink', marker_shape='x')
# ".": point.
# "o": circle.
# "s": square.
# "^": triangle.
# "v": upside down triangle.
# "+": plus.
# "x": X.
plot_vector(ax, Aorigin, vector, 'c')
plot_vector(ax, Aorigin, v_9in5, 'y')


# ax.set_xlim([-2, 2])
# ax.set_ylim([-2, 2])
# ax.set_zlim([-2, 2])

plt.show()