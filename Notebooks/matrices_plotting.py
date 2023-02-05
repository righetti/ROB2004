import matplotlib.pyplot as plt
import numpy as np

def plot_two_axes(ax, Aorigin, X_A, Y_A, Z_A, Borigin_A, X_B, Y_B, Z_B):
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # # plot the point using scatter method
    # if is_point_in_A:
    #     ax.scatter(point[0]+Aorigin[0], point[1]+Aorigin[1], point[2]+Aorigin[2], c='r', marker='o')
    # else:
    #     ax.scatter(point[0]+Borigin_A[0], point[1]+Borigin_A[1], point[2]+Borigin_A[2], c='r', marker='o')


    ax.plot([Aorigin[0], Aorigin[0]+X_A[0]], [Aorigin[1], Aorigin[1]+X_A[1]], [Aorigin[2], Aorigin[2]+X_A[2]], 'r')
    ax.plot([Aorigin[0], Aorigin[0]+Y_A[0]], [Aorigin[1], Aorigin[1]+Y_A[1]], [Aorigin[2], Aorigin[2]+Y_A[2]], 'g')
    ax.plot([Aorigin[0], Aorigin[0]+Z_A[0]], [Aorigin[1], Aorigin[1]+Z_A[1]], [Aorigin[2], Aorigin[2]+Z_A[2]], 'b')
    ax.plot([Borigin_A[0], Borigin_A[0]+X_B[0]], [Borigin_A[1], Borigin_A[1]+X_B[1]], [Borigin_A[2], Borigin_A[2]+X_B[2]], 'r')
    ax.plot([Borigin_A[0], Borigin_A[0]+Y_B[0]], [Borigin_A[1], Borigin_A[1]+Y_B[1]], [Borigin_A[2], Borigin_A[2]+Y_B[2]], 'g')
    ax.plot([Borigin_A[0], Borigin_A[0]+Z_B[0]], [Borigin_A[1], Borigin_A[1]+Z_B[1]], [Borigin_A[2], Borigin_A[2]+Z_B[2]], 'b')
    ax.set_xlabel('X_A')
    ax.set_ylabel('Y_A')
    ax.set_zlabel('Z_A')

    ax.text(Aorigin[0], Aorigin[1], Aorigin[2], 'Aorigin')
    ax.text(Borigin_A[0], Borigin_A[1], Borigin_A[2], 'Borigin_A')
    # plt.show()
    
def plot_scatter_point(ax, point, is_point_in_A, Aorigin, Borigin_A, marker_color, marker_shape):
    if not marker_color:
        marker_color = 'r'
        
    if not marker_shape:
        marker_shape = 'o'
        
    # plot the point using scatter method
    if is_point_in_A:
        ax.scatter(point[0]+Aorigin[0], point[1]+Aorigin[1], point[2]+Aorigin[2], c= marker_color, marker=marker_shape)
    else:
        ax.scatter(point[0]+Borigin_A[0], point[1]+Borigin_A[1], point[2]+Borigin_A[2], c= marker_color, marker=marker_shape)
    
    # plt.show()

def plot_vector(ax, origin, vector, vec_color):
    if not vec_color:
        vec_color = 'c'

    ax.quiver(origin[0], origin[1], origin[2], vector[0], vector[1], vector[2], color=vec_color)

    
def get_rotation_matrix_3d(axis, theta):
    if axis == 'x':
        # Rotation matrix about the x-axis
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(theta), -np.sin(theta)],
                        [0, np.sin(theta), np.cos(theta)]])
        return R_x
    elif axis == 'y':
        # Rotation matrix about the y-axis
        R_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])
        return R_y
    elif axis == 'z':
        # Rotation matrix about the z-axis
        R_z = np.array([[np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1]])
        return R_z
    else:
        raise ValueError("Invalid axis. Choose 'x', 'y', or 'z'.")

