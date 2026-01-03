def FilterOutput():
    
    import pandas as pd
    import numpy as np
    from KalmanFilter import KalmanFilter

    df = pd.read_csv('Sensor_readings.csv')

    reading_x = df["x reading (m)"]
    reading_y = df["y reading (m)"]
    reading_z = df["z reading (m)"]
    times = df["time (s)"]

    signal_x = [reading_x[0]]
    signal_z = [reading_z[0]]
    signal_y = [reading_y[0]]

    estimate_x = [reading_x[0]]
    estimate_z = [reading_z[0]]
    estimate_y = [reading_y[0]]

    variance_pos = [5]
    variance_vel = [5]

    t = 0
    t_signal = [times[0]]

    for i in range(1,len(times)):
        
        z = np.array([[reading_x[i]],
                 [reading_y[i]],
                 [reading_z[i]]])
        
        z0 = np.array([[reading_x[i-1]],
                 [reading_y[i-1]],
                 [reading_z[i-1]]])
        
        dt = times[1]
        f = KalmanFilter(z,i,dt,z0)
    
        KalmanEstimate = f[0]

        signal_x.append(reading_x[i])
        signal_y.append(reading_y[i])
        signal_z.append(reading_z[i])

        estimate_x.append(KalmanEstimate[0][0])
        estimate_y.append(KalmanEstimate[2][0])
        estimate_z.append(KalmanEstimate[4][0])

        variance = np.diag(f[1])
        variance_pos.append(variance[0])
        variance_vel.append(variance[1])
    
        t = times[i]

        t_signal.append(t)
        
        if variance[0] <= .05 and variance[1] <= .005:
            
            break

    return [KalmanEstimate, signal_x, signal_y, signal_z, t, t_signal, estimate_x, estimate_y, estimate_z, variance_pos, variance_vel]
