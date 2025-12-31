def FilterOutput():
    
    import pandas as pd
    import numpy as np
    from Filter import K_filter

    df = pd.read_csv('Sensor_readings.csv')

    reading_x = df["x reading (m)"]
    reading_y = df["y reading (m)"]
    reading_z = df["z reading (m)"]
    times_1 = df["time (s)"]

    signal_x = []
    signal_z = []
    signal_y = []

    t_1 = 0
    t_signal = []

    for i in range(1,len(times_1)):
        
        z = np.array([[reading_x[i]],
                 [reading_y[i]],
                 [reading_z[i]]])
        
        z0 = np.array([[reading_x[i-1]],
                 [reading_y[i-1]],
                 [reading_z[i-1]]])
        
        dt = times_1[1]
        f = K_filter(z,i,dt,z0)
    
        KalmanEstimate = f[0]

        signal_x.append(reading_x[i])
        signal_y.append(reading_y[i])
        signal_z.append(reading_z[i])

        variance = np.diag(f[1])
    
        t_1 = times_1[i]

        t_signal.append(t_1)
        
        if variance[0] <= .05 and variance[1] <= .005:
            
            break

    return [KalmanEstimate, signal_x, signal_y, signal_z, variance, t_1, t_signal]
