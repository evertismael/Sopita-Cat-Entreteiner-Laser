import cv2
import numpy as np
from scipy.optimize import Bounds
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from libs.cam_calib_utils import inv_svd
'''
Mechanical parts modeling:
'''
def make_C1(theta_deg):
    M_L_C1 = np.eye(4)
    M_L_C1[:3,:3], _ = cv2.Rodrigues(np.array([0,0,np.deg2rad(theta_deg)]))
    return M_L_C1

def make_C2(Lx):
    M_C1_C2 = np.eye(4)
    #R
    M_C1_C2[:3,:3] = np.array([[0,0,-1],[0,1,0],[1,0,0]]).T
    
    #T
    M_C1_C2[:3,3] = np.array([Lx,0,0])
    return M_C1_C2

def make_C3(phi_deg):
    M_C2_C3 = np.eye(4)
    M_C2_C3[:3,:3], _ = cv2.Rodrigues(np.array([0,0,np.deg2rad(phi_deg)]))
    return M_C2_C3

def move_set_up(theta_deg, phi_deg, M_W_L, Lx):
    # update C1 coordinate sys and compute M to the world
    M_L_C1 = make_C1(theta_deg)
    M_W_C1 = M_W_L.dot(M_L_C1)

    # update C2 coordinate sys and compute M to the world
    M_C1_C2 = make_C2(Lx=Lx)
    M_W_C2 = M_W_C1.dot(M_C1_C2)

    # update C3 coordinate sys and compute M to the world
    M_C2_C3 = make_C3(phi_deg)
    M_W_C3 = M_W_C2.dot(M_C2_C3)

    return M_W_C1, M_W_C2, M_W_C3

def find_laser_base_R_T_error(ang_and_T, P1_W, M_L_C3_list, T_W_L_0):
    # convert P1_W to Laser coords.
    angles_hat = np.array(ang_and_T[0:3])
    T_hat = np.array(ang_and_T[3:])
    R_hat,_ = cv2.Rodrigues(np.deg2rad(angles_hat))

    M_L_W_hat = np.eye(4)
    M_L_W_hat[:3,:3] = R_hat
    M_L_W_hat[:3, 3] = T_hat
    M_W_L_hat = inv_svd(M_L_W_hat)

    P1_L_hat = M_L_W_hat.dot(P1_W)

    w = 1*np.array([[1, 1, 1]]).T

    # Each point
    P1_L = np.zeros_like(P1_L_hat)
    for idx, M_L_C3 in enumerate(M_L_C3_list):
        q_L_i = M_L_C3.dot(np.array([[0,0,0,1]]).T)
        rho_i = np.sqrt(np.sum((P1_L_hat[:3,idx] - q_L_i[:3,0])**2,axis=0))
        P1_L[:,idx,None] = M_L_C3.dot(np.array([[0,rho_i,0,1]]).T)
    
    T_W_L_diff = (M_W_L_hat[:3,3,None] - T_W_L_0)**2
    error = np.mean(np.sum((P1_L - P1_L_hat)**2,axis=0,keepdims=True), axis=1, keepdims=True) + np.sum(w*T_W_L_diff)
    return error

def estimate_R_l_w_and_T_l_w(P1_W, M_L_C3_list, T_W_L_0, options={}):

    # define bounds for optimization
    x0mins = [-90,-90,-90,   -100,-100,-100]
    x0max =  [ 90, 90, 90,    100, 100, 100]
    bounds = Bounds(x0mins, x0max)

    # init value:
    ang_and_T0 = np.random.uniform(-80,80,6)

    res = minimize(find_laser_base_R_T_error,ang_and_T0,args=(P1_W, M_L_C3_list, T_W_L_0),method='nelder-mead', tol=1e-10, 
                bounds=bounds,options=options)

    angles_hat = np.array(res.x[0:3])
    T_hat = np.array(res.x[3:])
    R_hat,_ = cv2.Rodrigues(np.deg2rad(angles_hat))

    return angles_hat, R_hat, T_hat

def plot_coord_sys(M_world_sys, scale, sys_name, ax, alpha):
    # Define axes in sys-coord_system
    xyz_s = np.array([[0, 0, 0],[scale, 0, 0], [0, 0, 0],[0, scale, 0], [0, 0, 0],[0, 0, scale]]).T

    # prepare to plot: homogeneous coords:
    xyz1_s = np.row_stack((xyz_s,np.ones((1,xyz_s.shape[1]))))

    # convert to world coords:
    xyz1_w = M_world_sys.dot(xyz1_s)

    # plot axes: x->red, y->green, z->blue.
    ax.plot(xyz1_w[0,:2], xyz1_w[1,:2], xyz1_w[2,:2], color='red', alpha=alpha)
    ax.plot(xyz1_w[0,2:4], xyz1_w[1,2:4], xyz1_w[2,2:4], color='green', alpha=alpha)
    ax.plot(xyz1_w[0,4:], xyz1_w[1,4:], xyz1_w[2,4:], color='blue', alpha=alpha)
    ax.text(xyz1_w[0,0], xyz1_w[1,0], xyz1_w[2,0], sys_name, 'x', color='black', alpha=alpha)
    
def make_figure(x_lim, y_lim, z_lim):
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim(x_lim)
    ax.set_ylim(y_lim)
    ax.set_zlim(z_lim)
    plt.gca().set_aspect('equal', adjustable='box')
    return ax

def get_theta(Pxy_L, Lx):
    QP = np.sqrt(np.sum(Pxy_L**2) - np.abs(Lx)**2)
    Px = Pxy_L[0]
    Py = Pxy_L[1]

    a = Px**2 - QP**2
    b = -2*Px*Py
    c = Py**2 - QP**2
    
    d = b**2-4*a*c # discriminant
    
    if d < 0:
        raise('discriminant negative')
    elif d == 0:
        x1 = (-b+np.sqrt(b**2-4*a*c))/2*a
        x2 = x1
    else:
        x1 = (-b+np.sqrt((b**2)-(4*(a*c))))/(2*a)
        x2 = (-b-np.sqrt((b**2)-(4*(a*c))))/(2*a)

    theta1 = np.rad2deg(np.arctan(x1))
    theta2 = np.rad2deg(np.arctan(x2))
    return theta1, theta2

def get_phi(Pxyz1_L, Lx, theta_deg):
    _, M_L_C2, _ = move_set_up(theta_deg, 0, M_W_L=np.eye(4), Lx=Lx)
    M_C2_L = inv_svd(M_L_C2)

    Pxyz1_C2 = M_C2_L.dot(Pxyz1_L)
    phi = np.rad2deg(np.arctan2(-Pxyz1_C2[0], Pxyz1_C2[1]))
    return phi[0]