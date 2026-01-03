import numpy as np
import pandas as pd

x_0 = -300
y_0 = 0
z_0 = -300
v_x = 60
v_y = 100
v_z = 40
g = -9.81

rng = np.random.default_rng()

t_f = (v_y + (v_y**2 + 2*-g*y_0)**(1/2)) / -g

t_1 = np.linspace(0,t_f,int((t_f / .005) + 1))
x_t = x_0 + v_x * t_1
y_t = y_0 + v_y * t_1 + .5 * g * t_1**2
z_t = z_0 + v_z * t_1 

samples = rng.normal(loc=0.0, scale=5.0, size=len(t_1))

x_signal = x_t + samples
y_signal = y_t + samples
z_signal = z_t + samples

df = pd.DataFrame({'x reading (m)':x_signal, 'y reading (m)':y_signal, 'z reading (m)':z_signal, 'time (s)':t_1})

df.to_csv("Sensor_readings.csv", index=False)
