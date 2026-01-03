def KalmanFilter(z, number, dt, z0):
    import numpy as np
    g = -9.81
    if number == 1:

        x1 = z[0][0]
        y1 = z[1][0]
        z1 = z[2][0]

        xi = z0[0][0]
        yi = z0[1][0]
        zi = z0[2][0]
        
        dx = x1 - xi
        dy = y1 - yi
        dz = z1 - zi

        x_angle = np.atan(dz/dx)
        ele_angle = np.atan(dy / ((dx**2 + dz**2)**(1/2)))

        v_MaxMag = 125 * 1.2

        vx = v_MaxMag * ((np.cos(ele_angle)**2) / (1 + (np.tan(x_angle))**2))**(1/2)
        vy = v_MaxMag * np.sin(ele_angle)
        vz = v_MaxMag * np.cos(ele_angle) * np.tan(x_angle) * ((np.tan(x_angle))**2 + 1)**(-1/2)
        
        KalmanFilter.x = np.array([[xi],
                              [vx],
                              [yi],
                              [abs(vy)],
                              [zi],
                              [vz]])
        
        KalmanFilter.P = np.diag((5,5,5,5,5,5))

        KalmanFilter.A = np.array([[1, dt, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0],
                               [0, 0, 1, dt, 0, 0],
                               [0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 1, dt],
                               [0, 0, 0, 0, 0, 1]])

        KalmanFilter.H = np.array([[1, 0, 0, 0, 0, 0],
                               [0, 0, 1, 0, 0, 0],
                               [0, 0, 0, 0, 1, 0]])

        KalmanFilter.HT = KalmanFilter.H.T

        KalmanFilter.R = np.diag((20, 20, 20))

        KalmanFilter.Q = np.diag((0,0,0,0,0,0))

        KalmanFilter.U = np.array([[0],
                               [0],
                               [0],
                               [g],
                               [0],
                               [0]])

        KalmanFilter.B =np.array([[0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, dt, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0]])

    x_p = KalmanFilter.A @ KalmanFilter.x + KalmanFilter.B @ KalmanFilter.U

    P_p = KalmanFilter.A @ KalmanFilter.P @ KalmanFilter.A.T + KalmanFilter.Q

    S = KalmanFilter.H @ P_p @ KalmanFilter.HT + KalmanFilter.R

    K = (P_p @ KalmanFilter.HT) @ (np.linalg.inv(S))

    residual = z - KalmanFilter.H @ x_p

    KalmanFilter.x = x_p + K @ residual

    KalmanFilter.P = (np.eye(6,6) - K @ KalmanFilter.H) @ P_p

    x_p = KalmanFilter.x

    return [KalmanFilter.x, KalmanFilter.P]
