import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def rotation_matrix(point):
    """
    point - is a 2D vector - np.array of 2X1
    """
    R_i = np.identity(3)
    alpha_i = np.arctan2(point[1],point[0])
    if alpha_i < 0:
        alpha_i += 2*np.pi
    theta_i = alpha_i - np.pi/2 
    R_i[0:2,0:2] = np.array([[np.cos(theta_i),np.sin(theta_i)],
                                 [-np.sin(theta_i),np.cos(theta_i)]])
    return R_i

def test_rotation_matrix():
    r = rotation_matrix(np.array([1,0]))
    print(r)

def gen_grasp_matrix(object_center,contact_locations):
    """
    function is for planar case
    object_center - is a vector -- np.array of 2X1
    N contact_locations - list of vectors -- np.array of 2xN
    """
    contact_normal_vectors = contact_locations - object_center

    G_T = None
    # assuming hard finger contact
    H = np.identity(3)
    H[2,2] = 0
    for point in contact_normal_vectors.T:
        P_i = np.identity(3)
        P_i[0:2,2] = np.array([[-point[1],point[0]]])
        R_i = rotation_matrix(point)

        G_T_i = H @ ((R_i @ P_i))
#        print(f"point:\n{point}")
#        print(f"theta:\n{theta_i}")
#        print(f"rotation:\n{R_i}")
#        print(f"P_i:\n{P_i}")
#        print(f"H_i:\n{H}")
#        print(f"G_T_i:\n{G_T_i}")
        if G_T is None:
            G_T = G_T_i
            continue
        G_T = np.concatenate((G_T,G_T_i))
    G = G_T.T
    return G

def test_gen_grasp_matrix():
    c=np.array([[1,2],[3,4],[5,6],[7,8]]).T
    c=np.array([[0,2]]).T
    o=np.array([[3,1.5]]).T
    gen_grasp_matrix(o,c)
    
def five_vectors_questions(fifth_vector,rect_object_coords):
    """
    fifth_vector - is a 2D vector - np.array of 2X1
    rect_object - is 2 2D vectors 
                - np.array of 2X2
                - 0 - first point
                - 1 - diagonally opposite to the first point
                    
    """
    #given_contacts = np.array([[3,0],[4,3],[2,3],[0,2]]).T
    given_contacts = np.array([[0,2],[2,3],[4,3],[3,0]]).T
    given_center = np.mean(rect_object_coords,axis=1,keepdims=True)

    five_vectors = np.concatenate((given_contacts,fifth_vector),axis=1)
    G = gen_grasp_matrix(given_center,five_vectors)
    rank = np.linalg.matrix_rank(G)
    if rank != 3:
        print(f"rank:{rank} is incorrect not proceeding further")
        return None,None,None
        
    eigen_values = np.linalg.eigvals(G @ G.T)
    singular_values = eigen_values ** 0.5

    # minimum singular quality computation
    min_singular = np.min(singular_values)

    # volume of ellipsoid quality computation
    volume = np.prod(singular_values,axis=0)

    # isotropy quality computation
    max_singular = np.max(singular_values)
    isotropy = min_singular/max_singular

    return [min_singular,volume,isotropy]

rect_object_coords = np.array([[0,0],[6,3]]).T
edge_x_0_6 = np.arange(0,6.1,0.1)
edge_y_0_3 = np.arange(0,3.1,0.1)

edge_x_A = edge_x_0_6
edge_y_A = np.zeros_like(edge_x_A)
edge_A = np.vstack((edge_x_A,edge_y_A))
#print(f"edge_A:{edge_A}")

edge_x_B = np.full(shape=edge_y_0_3.shape,fill_value=6)
edge_y_B = edge_y_0_3
edge_B = np.vstack((edge_x_B,edge_y_B))
#print(f"edge_B:{edge_B}")

edge_x_C = np.flip(edge_x_0_6,axis=0)
edge_y_C = np.full(shape=edge_x_C.shape,fill_value=3)
edge_C = np.vstack((edge_x_C,edge_y_C))
#print(f"edge_C:{edge_C}")

edge_x_D = np.zeros_like(edge_y_0_3)
edge_y_D = np.flip(edge_y_0_3,axis=0)
edge_D = np.vstack((edge_x_D,edge_y_D))
#print(f"edge_D:{edge_D}")

all_edges = np.concatenate((edge_A,edge_B,edge_C,edge_D),axis=1)
#print(f"all_edges:{all_edges}")
#print(f"all_edges_shape:{all_edges.shape}")
min_singular_values = []
volume_values = []
isotropy_values = []
for force_point in all_edges.T[:,None]:
    min_singular,volume, isotropy = five_vectors_questions(force_point.T,rect_object_coords)
    if min_singular is None or volume is None or isotropy is None:
        continue
    min_singular_values.append(min_singular)
    volume_values.append(volume)
    isotropy_values.append(isotropy)

min_singular_values = np.array(min_singular_values)
max_min_singular_value = None
for x,y,quality in zip(all_edges[0,:],all_edges[1,:],min_singular_values):
    if max_min_singular_value is None or max_min_singular_value[2] < quality:
        max_min_singular_value = (x,y,quality)
print(f"maximum minimum singular value:{max_min_singular_value}")
min_singular_values = min_singular_values/np.max(min_singular_values)

volume_values = np.array(volume_values)
volume_value = None
for x,y,quality in zip(all_edges[0,:],all_edges[1,:],volume_values):
    if volume_value is None or volume_value[2] < quality:
        volume_value = (x,y,quality)
print(f"maximum volume of ellipsoid value:{volume_value}")
volume_values = volume_values/np.max(volume_values)

isotropy_values = np.array(isotropy_values)
isotropy_value = None
for x,y,quality in zip(all_edges[0,:],all_edges[1,:],isotropy_values):
    if isotropy_value is None or isotropy_value[2] < quality:
        isotropy_value = (x,y,quality)
print(f"maximum isotropy value:{isotropy_value}")
isotropy_values = isotropy_values/np.max(isotropy_values)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(all_edges[0,:],all_edges[1,:],min_singular_values,label='minimum singular value')
ax.plot3D(all_edges[0,:],all_edges[1,:],volume_values,color='magenta',label='volume of ellipsoid')
ax.plot3D(all_edges[0,:],all_edges[1,:],isotropy_values,color='darkviolet',label='isotropy of ellipsoid')
ax.plot3D(all_edges[0,:],all_edges[1,:],0,color="red",label="rectangle")
ax.set_zlabel('quality')
plt.title('quality metrics varying on rectangle')
plt.legend(loc=3)
plt.show()
