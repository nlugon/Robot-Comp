### Extended Kalman Filter ###
import numpy as np

def input_conversion(u):
    """
    Make the conversion from the input to the state space

    --------------------------------------------------------------
    Input:
        u (float) : input value [motor speed (0-100)]

    --------------------------------------------------------------
    Output: 
        u (float) : speed [m/s]
    
    """

    # conversion factors
    # Vitesse :  5.5m pour 27.3s 0.20m/s
    speed_conv_factor = 1/495 #0.01403*mm2pxl 

    # conversion 
    u = speed_conv_factor*u

    return u


def prediction(x_prev, P_prev, u, A, B, dt):
    """
    Make the prediction step of the Kalman filter

    --------------------------------------------------------------
    Input:
        x_prev (float) : previous state
        P_prev (float) : previous covariance
        u (float) : input value
        A (float) : state transition matrix
        B (float) : input matrix
        dt (float) : time step

    --------------------------------------------------------------
    Output: 
        x_priori (float) : predicted state
        P_priori (float) : predicted covariance
    
    """
    ## Prediction step
    q = [0.1,0.1,np.pi/6]
    Q = np.diag(q)
    # estimated mean of the state
    x_priori = np.dot(A, x_prev) + np.dot(B, u)
    # jacobian of function f(x,u)
    Fk = np.array([[1, 0, -dt*np.sin(-x_priori[2])*(u[0]-u[1])/2], 
                   [0, 1,  dt*np.cos(-x_priori[2])*(u[0]+u[1])/2], 
                   [0, 0, 1]], float)
    # estimated covariance of the state
    P_priori = np.dot(Fk, np.dot(P_prev, Fk.T)) + Q
    
    return x_priori, P_priori
    
def update(x_priori, P_priori, z):
    """
    Make the update step of the Kalman filter

    --------------------------------------------------------------
    Input:
        x_priori (float) : predicted state
        P_priori (float) : predicted covariance
        z (float) : measurement

    --------------------------------------------------------------
    Output: 
        x_posteriori (float) : updated state
        P_posteriori (float) : updated covariance
    
    """
    ## Update step          
    H = np.identity(3)
    r = [0.2,0.2,np.pi/12]
    R = np.diag(r)

    # innovation / measurement residual
    i = z - np.dot(H, x_priori) 
       
    # measurement prediction covariance
    S = np.dot(H, np.dot(P_priori, H.T)) + R
             
    # Kalman gain (tells how much the predictions should be corrected based on the measurements)
    K = np.dot(P_priori, np.dot(H.T, np.linalg.inv(S)))

    # a posteriori estimate
    x_posteriori = x_priori + np.dot(K,i)
    P_posteriori = P_priori - np.dot(K,np.dot(H, P_priori))  

    return x_posteriori, P_posteriori

## Main function
def calcKalmanFilter(state_prev, cov_prev, camera_measure, motor_input, flag_pos, dt):
    """
    Main function of the Kalman filter
    Estimates the current state using input sensor data and the previous state

    --------------------------------------------------------------
    Input:
        state_prev (float) : previous state (x,y,angle)
        cov_prev (float) : previous covariance (3*3)
        camera_measure (float) : measured state (x,y,angle)
        motor_input (float) : motor input value (wr,wl)
        flag_pos (float) : boolean camera used (True,False)

    --------------------------------------------------------------
    Output: 
        state_posteriori (float) : new a posteriori state estimation (x,y,angle)
        cov_posteriori (float) : new a posteriori state covariance (3*3)
    
    """
    
    ## Initialising the needed constants
    # units: length [m], angle [rad], time [s]
    axle = 0.376
    
    motor_input = input_conversion(motor_input)
    
    # State-space matrices
    A = np.identity(3)
    B = dt*np.array([[np.cos(-state_prev[2])/2, np.cos(-state_prev[2])/2],
                    [np.sin(-state_prev[2])/2, np.sin(-state_prev[2])/2],
                    [1/axle, -1/axle]], float)

    # bycicle model (not used)
    # turning_radius = 0.5*axle*(motor_input[0]+motor_input[1])/(motor_input[0]-motor_input[1])
    # delta_theta = dt*(motor_input[0]-motor_input[1])/axle
    # delta_x = turning_radius*(np.sin(state_prev[2]+delta_theta)-np.sin(state_prev[2]))
    # delta_y = turning_radius*(np.cos(state_prev[2])-np.cos(state_prev[2]+delta_theta))
    # B = np.array([[delta_x, delta_x],
    #               [delta_y, delta_y],
    #               [1/axle, -1/axle]], float)
    
    state_priori, cov_priori = prediction(state_prev, cov_prev, motor_input, A, B, dt) 
    
    # If camera does not send measurements, then return a priori estimate
    if (flag_pos) :
        return state_priori, cov_priori
    # Else go to update step
    state_posteriori, cov_posteriori = update(state_priori, cov_priori, camera_measure)
    
    return state_posteriori, cov_posteriori