import modern_robotics as mr
import numpy as np 
import csv

def FeedbackControl(Tse, Tse_d, Tse_d_nx, Kp, Ki, dt, robo_config, integral):
    """ Calculates the feedforward and feedback control law for the task-space
    Input:
    :param Tse: The current actual end-effector configuration
    :param Tse_d: The current end-effector reference configuration Xd (i.e., Tse,d).
    "param Tse_d_nx: The end-effector reference configuration at the next timestep in the reference trajectory at a time Δt later.
    :param Kp, param Ki: The PI gain matrices Kp and Ki.
    :param dt: The timestep Δt between reference trajectory configurations.

    Output:

    :param V_e: The commanded end-effector twist \mathcal{V} expressed in the end-effector frame {e}.
    """
    # The fixed offset from the chassis frame {b} to the base frame of the arm {0}
    Tb0 = np.array([[ 1, 0, 0, 0.1662],
                    [ 0, 1, 0,      0], 
                    [ 0, 0, 1, 0.0026], 
                    [ 0, 0, 0,      1]])
    
    # For home configuration...
    M0e = np.array([[ 1, 0, 0,  0.033],
                    [ 0, 1, 0,      0], 
                    [ 0, 0, 1, 0.6546], 
                    [ 0, 0, 0,      1]])
    
    B_list = np.array([[0,  0,  1,       0, 0.033, 0], 
                       [0, -1,  0, -0.5076,     0, 0], 
                       [0, -1,  0, -0.3526,     0, 0], 
                       [0, -1,  0, -0.2176,     0, 0], 
                       [0,  0,  1,       0,     0, 0]]).T

    r = 0.0475 # The radius of each wheel is r = 0.0475 meters.
    w = 0.3/2 # The side-to-side distance between wheels is 2w = 0.3 meters
    l = 0.47/2 # The forward-backward distance between the wheels is 2l = 0.47 meters

    theta_list = robo_config[3:8]
    T0e = mr.FKinBody(M0e, B_list, theta_list)

    time = 1/dt
    T_next = np.dot(mr.TransInv(Tse_d), Tse_d_nx)
    mat_log = mr.MatrixLog6(T_next)
    V_d = mr.se3ToVec(time*mat_log)
    # print(f"V_d is {V_d}")

    T_ee_d = np.dot(mr.TransInv(Tse), Tse_d)
    Ad_xxd = mr.Adjoint(T_ee_d)
    Ad_V = np.dot(Ad_xxd, V_d)
    # print("")
    # print("Adjoint times Vd is")
    # print(Ad_V)

    mat_log2 = mr.MatrixLog6(T_ee_d)
    X_err = mr.se3ToVec(mat_log2)
    # print("")
    # print("X error is")
    # print(X_err)

    X_err_int = integral + (X_err * dt)
    integral += X_err * dt

    V = Ad_V + np.dot(X_err, Kp) + np.dot(X_err_int, Ki)
    # print("")
    # print("V is")
    # print(V)

    J_arm = mr.JacobianBody(B_list, theta_list)

    T_00 = np.dot(mr.TransInv(T0e), mr.TransInv(Tb0))
    F = (r/4) * np.array([[           0,           0,           0,            0],
                          [           0,           0,           0,            0],
                          [(-1 / (l+w)), (1 / (l+w)), (1 / (l+w)), (-1 / (l+w))], 
                          [           1,           1,           1,            1], 
                          [          -1,           1,          -1,            1],
                          [           0,           0,           0,            0]])
    J_base = np.dot(mr.Adjoint(T_00), F)
    J_e = np.concatenate((J_base, J_arm), axis = 1)
    # print("")
    # print("J_e is")
    # print(J_e)

    Je_p_inv = np.linalg.pinv(J_e)
    speeds = np.dot(Je_p_inv, V)
    # print("")
    # print("speeds are")
    # print(speeds)

    return speeds, X_err

# Test values
robo_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
Xd = np.array([[ 0, 0, 1, 0.5], 
               [ 0, 1, 0,   0], 
               [-1, 0, 0, 0.5], 
               [ 0, 0, 0,   1]])
Xd_nx = np.array([[ 0, 0, 1, 0.6],
                  [ 0, 1, 0,   0], 
                  [-1, 0, 0, 0.3], 
                  [ 0, 0, 0,   1]])
X = np.array([[ 0.170, 0, 0.985, 0.387],
              [     0, 1,     0,     0], 
              [-0.985, 0, 0.170, 0.570], 
              [     0, 0,     0,     1]])
        
# Kp and Ki are zero matracies for testing
Kp = Ki = np.zeros(6)
integral = np.zeros((6,), dtype = float)
dt = 0.01

FeedbackControl(X, Xd, Xd_nx, Kp, Ki, dt, robo_config, integral)