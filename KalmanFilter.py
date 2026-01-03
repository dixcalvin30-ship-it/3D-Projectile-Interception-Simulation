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
        
        K_filter.x = np.array([[xi],
                              [vx],
                              [yi],
                              [abs(vy)],
                              [zi],
                              [vz]])
        
        K_filter.P = np.diag((5,5,5,5,5,5))

        K_filter.A = np.array([[1, dt, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0],
                               [0, 0, 1, dt, 0, 0],
                               [0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 1, dt],
                               [0, 0, 0, 0, 0, 1]])

        K_filter.H = np.array([[1, 0, 0, 0, 0, 0],
                               [0, 0, 1, 0, 0, 0],
                               [0, 0, 0, 0, 1, 0]])

        K_filter.HT = K_filter.H.T

        K_filter.R = np.diag((20, 20, 20))

        K_filter.Q = np.diag((0,0,0,0,0,0))

        K_filter.U = np.array([[0],
                               [0],
                               [0],
                               [g],
                               [0],
                               [0]])

        K_filter.B =np.array([[0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, dt, 0, 0],
                               [0, 0, 0, 0, 0, 0],
                               [0, 0, 0, 0, 0, 0]])

    x_p = K_filter.A @ K_filter.x + K_filter.B @ K_filter.U

    P_p = K_filter.A @ K_filter.P @ K_filter.A.T + K_filter.Q

    S = K_filter.H @ P_p @ K_filter.HT + K_filter.R

    K = (P_p @ K_filter.HT) @ (np.linalg.inv(S))

    residual = z - K_filter.H @ x_p

    K_filter.x = x_p + K @ residual

    K_filter.P = (np.eye(6,6) - K @ K_filter.H) @ P_p

    x_p = K_filter.x

    return [K_filter.x, K_filter.P]
