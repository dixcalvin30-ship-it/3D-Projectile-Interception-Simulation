def K_filter(z, number):
    dt = 0.01

    if number == 1:
        
        K_filter.x = np.array([[z[0][0]],
                              [50],
                              [z[1][0]],
                              [50],
                              [z[2][0]],
                              [50]])
        
        K_filter.P = np.diag((4,4,4,4,4,4))

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

        K_filter.R = np.diag((12, 12, 12))

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

    K_filter.P = P_p - K @ K_filter.H @ P_p

    x_p = K_filter.x

    return [K_filter.x[0], K_filter.x[1], K_filter.x[2], K_filter.x[3], K_filter.x[4], K_filter.x[5], K_filter.P, x_p]
