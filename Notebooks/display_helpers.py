import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

# be careful we will only print the first 5 digits and round small numbers in arrays
np.set_printoptions(suppress=True, precision=5)

# libraries to make things interactive
from ipywidgets import interact, interactive, fixed, interact_manual
import ipywidgets as widgets
from IPython.display import display, Latex, Markdown

def get_rotation_2D(theta):
    """This function gets an angle and returns a rotation matrix representing a 2D rotation of theta"""
    return np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])



# a nice function to print pretty matrices in latex
def bmatrix(a):
    """Returns a LaTeX bmatrix

    :a: numpy array
    :returns: LaTeX bmatrix as a string
    """
    if len(a.shape) > 2:
        raise ValueError('bmatrix can at most display two dimensions')
    lines = str(a).replace('[', '').replace(']', '').splitlines()
    rv = [r'\begin{bmatrix}']
    rv += ['  ' + ' & '.join(l.split()) + r'\\' for l in lines]
    rv +=  [r'\end{bmatrix}']
    return str(r''.join(rv))

def pmatrix(a):
    """Returns a LaTeX bmatrix

    :a: numpy array
    :returns: LaTeX bmatrix as a string
    """
    if len(a.shape) > 2:
        raise ValueError('bmatrix can at most display two dimensions')
    lines = str(a).replace('[', '').replace(']', '').splitlines()
    rv = [r'\begin{pmatrix}']
    rv += ['  ' + ' & '.join(l.split()) + r'\\' for l in lines]
    rv +=  [r'\end{pmatrix}']
    return str(r''.join(rv))



def rotated_frame_widget():
    def display_frame2d(ax, theta):
        R = get_rotation_2D(theta)
        # x-y axes of the base frame
        x_base = [1,0]
        y_base = [1,0]
        x_new = R[:,0]
        y_new = R[:,1]

        p = [1,1]
        p_new = R.dot(p)

        ax.clear()
        ax.set_xlim([-2,2])
        ax.set_ylim([-2,2])
        ax.arrow(0,0,1,0,width=0.05, color='k')
        ax.arrow(0,0,0,1,width=0.05, color='k')
        ax.arrow(0,0,x_new[0],x_new[1],width=0.05, color='r')
        ax.arrow(0,0,y_new[0],y_new[1],width=0.05, color='r')
        ax.plot(p_new[0],p_new[1], '-rX', ms=10, lw=4)
        display(Markdown('$R_{SB} = ' + bmatrix(R) + '$'))
        return display(Markdown(r'The red cross has coordinates $\begin{pmatrix}1\\1\end{pmatrix}$ in the red frame and coordinates $R_{SB}\cdot \begin{pmatrix}1\\1\end{pmatrix}=' + pmatrix(np.array([[p_new[0]],[p_new[1]]])) + '$ in the black frame'))
    
    fig = plt.figure(figsize=[4,4])
    ax = fig.add_subplot(111)
    display(Markdown('### Illustration of orientation of a frame B with respect to a frame S'))
    display(Markdown('The frame S is displayed in black and the frame B in red'))
    display(Markdown('The frame B is displayed by plotting the x and y vectors as defined by the columns of $R_{SB}$'))
    display(Markdown('Use the slider to change the orientation theta'))
    interact(lambda theta: display_frame2d(ax, theta), theta=(-6.3,6.3,0.01))
    
    
def display_rotated_body_widget(body):
    def display_rotated_body(ax, theta):
        R = get_rotation_2D(theta)
        
        ## coordinates of the rotated body
        body_prime = R @ body

        ax.clear()
        ax.set_xlim([-2,2])
        ax.set_ylim([-2,2])
        ax.arrow(0,0,1,0,width=0.05, color='k')
        ax.arrow(0,0,0,1,width=0.05, color='k')

        ax.plot(body[0,:], body[1,:], '-bX', ms=10, lw=4)
        ax.plot(body_prime[0,:], body_prime[1,:], '-rX', ms=10, lw=4)

        display(Markdown('The rotation matrix is $R = ' + bmatrix(R) + '$'))
        display(Markdown('The coordinates of points (each column is one point) for the original body are:\n\n' + '$ p^0 = ' + bmatrix(body) + '$'))
        return display(Markdown('The coordinates of the points of the rotated body are computed as $p^1 = R \cdot p^0$ which gives:\n\n $p^1 = '+ bmatrix(R) +'\cdot'+bmatrix(body)+' = '+bmatrix(body_prime)+'$'))


    fig = plt.figure(figsize=[4,4])
    ax2 = fig.add_subplot(111)
    display(Markdown('### Illustration of the rotation of an object'))
    display(Markdown('The original body is displayed in blue and the rotated one in red'))
    display(Markdown('Use the slider to change theta to rotate the body with respect to the origin of the coordinate frame'))
    interact(lambda theta: display_rotated_body(ax2, theta), theta=(-6.3,6.3,0.01))
    
    
    
def transform_vertices_3D(vertices, T):
    """
        takes an array of vertices (nx3) and homogeneous transform T
        returns each row (treated as a column vector) transformed by T
    """
    v_new = vertices.copy()

    for i in range(len(vertices)):
        v_aug = np.ones([4,1])
        v_aug[0:3,0] = vertices[i,:].transpose()
        v_aug_new = T.dot(v_aug)
        v_new[i,:] = v_aug_new[0:3,0]
    return v_new

def update_plot_cube3D(ax, vertices, T, plot_scale=1.):
    """
        this function plots a 3D cube described by vertices (in spatial frame) and transformed by T
        it also shows the spatial frame (origin) and the frame attached to the cube
        ax: matplotlib axes handle
        plot_scale: defines the x,y,z-lim of the plot
    """
    # move the cube
    moved_vertices = transform_vertices_3D(vertices,T)
    
    # generate list of sides' polygons of our cube
    sides = [ [moved_vertices[0],moved_vertices[1],moved_vertices[2],moved_vertices[3]],
              [moved_vertices[0],moved_vertices[1],moved_vertices[5],moved_vertices[4]],
              [moved_vertices[2],moved_vertices[3],moved_vertices[7],moved_vertices[6]],
              [moved_vertices[7],moved_vertices[6],moved_vertices[5],moved_vertices[4]],
           [moved_vertices[0],moved_vertices[3],moved_vertices[7],moved_vertices[4]],
           [moved_vertices[1],moved_vertices[2],moved_vertices[6],moved_vertices[5]]]

    ax.clear()
    ax.scatter3D(moved_vertices[:, 0], moved_vertices[:, 1], moved_vertices[:, 2], lw=2)
    # plot sides
    ax.add_collection3d(Poly3DCollection(sides, facecolors='red', linewidths=2, edgecolors='blue', alpha=.25))

    # we use plot_scale to define the display limits
    ax.set_xlim3d([-plot_scale,plot_scale])
    ax.set_ylim3d([-plot_scale,plot_scale])
    ax.set_zlim3d([-plot_scale,plot_scale])
    ax.quiver3D(0,0,0,1,0,0,ls='--',color='red',lw=2)
    ax.quiver3D(0,0,0,0,1,0,ls='--',color='green',lw=2)
    ax.quiver3D(0,0,0,0,0,1,ls='--',color='blue',lw=2)
    
    x1 = T.dot(np.array([1,0,0,0]))
    y1 = T.dot(np.array([0,1,0,0]))
    z1 = T.dot(np.array([0,0,1,0]))
    
    ax.quiver3D(T[0,3],T[1,3],T[2,3],x1[0],x1[1],x1[2],ls='-',color='red',lw=2)
    ax.quiver3D(T[0,3],T[1,3],T[2,3],y1[0],y1[1],y1[2],ls='-',color='green',lw=2)
    ax.quiver3D(T[0,3],T[1,3],T[2,3],z1[0],z1[1],z1[2],ls='-',color='blue',lw=2)