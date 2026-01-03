from FilterOutput import FilterOutput
import numpy as np
from scipy.optimize import fsolve
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define variables
g = -9.81
accel = g
accel_2 = 0

# Projectile 1
KalmanEstimate = FilterOutput()

xi_1 = KalmanEstimate[0][0][0]
vi_1_x = KalmanEstimate[0][1][0]
yi_1 = KalmanEstimate[0][2][0]
vi_1_y = KalmanEstimate[0][3][0]
zi_1 = KalmanEstimate[0][4][0]
vi_1_z = KalmanEstimate[0][5][0]
estimate_x = KalmanEstimate[6]
estimate_y = KalmanEstimate[7]
estimate_z = KalmanEstimate[8]
variance = KalmanEstimate[4]
TimeOfPrediction = KalmanEstimate[4]

t_f = (vi_1_y + (vi_1_y**2 + 2*-g*yi_1)**(1/2)) / -g
ri_1 = np.array([xi_1, yi_1, zi_1])
vi_1 = np.array([vi_1_x, vi_1_y, vi_1_z])

# Speed of Projectile 2
v2_mag = 760

# Starting Position of Projectile 2
ri_2 = np.array([0,0,0])
xi_2 = ri_2[0]
yi_2 = ri_2[1]
zi_2 = ri_2[2]

# Calculate position of Projectile 1 with respect to time from t = 0 to t = t_f
step = int(round(t_f*100))
t = np.linspace(0,t_f,step + 1)
x_1 = xi_1 + vi_1_x * t
y_1 = yi_1 + vi_1_y * t + accel * .5 * t**2
z_1 = z1_t = zi_1 + vi_1_z * t

# Calculate possible times of impact where s is time variable of Projectile 2           
t_impact_after = np.array([])
s_after = np.array([])
times = 0
r_t = np.hstack((x_1.reshape(len(x_1),1),y_1.reshape(len(y_1),1),z_1.reshape(len(z_1),1)))

def equations(q, x, y, z):
    a, b, c, s = q
    return (xi_2 + v2_mag * s * a + .5 * accel_2 * a * s**2 - x, 
            yi_2 + v2_mag * s * b + .5 * (g + accel_2 * b) * s**2 - y,
            zi_2 + v2_mag * s * c + .5 * accel_2 * c * s**2 - z,
            a**2 + b**2 + c**2 - 1)
    
for f1, f2, f3 in r_t:
    r_t_test = np.array([f1,f2,f3]) - ri_2
    r_t_unit = r_t_test / np.linalg.norm(r_t_test)
    initial_guess = np.hstack((r_t_unit,times))
    sol_test, infor, ier, msg = fsolve(equations, initial_guess, args=(f1,f2,f3), full_output=True)
    if ier == 1 and sol_test[3] > 0 and sol_test[3] <= times:
        t_impact_after = np.append(t_impact_after, [times])
        s_after = np.append(s_after, [sol_test[3]])
    times += t[1]
            
# Dsiplay possible times of impact and recieve input
if len(t_impact_after) > 0:
    while True:
        print(f"Possible times for interseption are from {math.ceil((t_impact_after[0] + TimeOfPrediction) * 100) / 100}s to {math.floor((t_impact_after[-1] + TimeOfPrediction) * 100) / 100}s")
        t_collide = float(input('Enter desired time for impact (s):')) - TimeOfPrediction
        if t_collide >= (t_impact_after[0]) and t_collide <= (t_impact_after[-1]):
            break

    # Calculate postion at desired time, t0-t1
    t1 = np.linspace(0,t_collide,int(round(t_collide*100)))
    x1_t0_t1 = xi_1 + vi_1_x * t1
    y1_t0_t1 = yi_1 + vi_1_y * t1 + accel * .5* t1**2
    z1_t0_t1 = zi_1 + vi_1_z * t1

    # Calculate s for desired time 
    r1_f = np.array([x1_t0_t1[-1],y1_t0_t1[-1],z1_t0_t1[-1]]) - ri_2
    r1_f_unit = r1_f / np.linalg.norm(r1_f)
    inital_guess = np.hstack((r1_f_unit,t_collide)) 
    sol, infor, ier, msg = fsolve(equations, inital_guess, args=(x1_t0_t1[-1],y1_t0_t1[-1],z1_t0_t1[-1]), full_output=True)
    s = sol[3]

    # Generate data for postition of Projectile 2
    unit_vx2 = sol[0]
    unit_vy2 = sol[1]
    unit_vz2 = sol[2]
    v2_unit = np.array([unit_vx2,unit_vy2,unit_vz2])
    vi_2 = v2_mag * v2_unit
    vi_2_x = vi_2[0]
    vi_2_y = vi_2[1]
    vi_2_z = vi_2[2]
    accel_2_v = accel_2 * v2_unit
    accel_2_x = accel_2_v[0]
    accel_2_y = accel_2_v[1]
    accel_2_z = accel_2_v[2]

    # Calculate launch angles
    ele_angle_2 = np.degrees(np.atan(unit_vy2 / (unit_vz2**2 + unit_vx2**2)**(1/2)))
    x_angle_2 = np.degrees(np.atan(unit_vz2 / unit_vx2))
    
    # Create time vector for projectile 2 where position values before time of launch are zero
    t2 = t1 - (t_collide - s)
    t2[t2 < 0] = 0

    # Create positional data for projectile 2 with respect to time 
    x_2 = xi_2 + vi_2_x * t2 + accel_2_x * .5 * t2**2
    y_2 = yi_2 + vi_2_y * t2 + (g + accel_2_y) * .5* t2**2
    z_2 = zi_2 + vi_2_z * t2 + accel_2_z * .5 * t2**2

    # Create 3D graph
    fig = plt.figure(figsize=(5,5))
    ax = fig.add_subplot(projection='3d')
    ax.plot(x1_t0_t1, 
            z1_t0_t1, 
            y1_t0_t1, 
            label='Projectile 1', 
            color ='blue')
    ax.plot(x_2, 
            z_2, 
            y_2, 
            label='Projectile 2', 
            color ="red")
    ax.plot(x_2[-1],
            z_2[-1],
            y_2[-1],
            'y*',
            label='Point of Impact',
            markersize=10)
    ax.plot(x_2[0],
            z_2[0],
            y_2[0],
            'ro',
            markersize=5)
    ax.plot(x1_t0_t1[0],
            z1_t0_t1[0],
            y1_t0_t1[0],
            'bo',
            markersize=5)
    ax.plot(estimate_x, 
            estimate_z, 
            estimate_y, 
            'co--', 
            markersize=1, 
            label='Kalman Estimation')
    plt.title("Projectile Interseption")
    plt.legend()
    plt.tight_layout()
    plt.savefig('projectile.png')
    plt.show()

    # Display calculated data
    print(f"Projectile 1")
    print(f"Position at time of prediction (x,y,z)(m): {np.round(ri_1,2)}")
    print(f"Velcotiy at time of prediction (vx,vy,vz)(m/s): {np.round(vi_1,2)}")
    print(f"Time of flight without interseption (s): {t_f + TimeOfPrediction:.2f}")
    print(f"Time of prediction (s): {TimeOfPrediction:.2f}")
    print(" ")
    print(f"Projectile 2")
    print(f"Inital position (x,y,z)(m): {np.round(ri_2,2)}")
    print(f"Launch speed (m/s): {v2_mag:.2f}")
    print(f"Inital velocity (vx,vy,vz)(m/s): {np.round(vi_2,2)}")
    print(f"Launch direction as unit vector: {np.round(v2_unit,2)}")
    print(f"Inital elevation angle (degress): {ele_angle_2:.2f}")
    print(f"Inital angle from x-axis (degrees): {x_angle_2:.2f}")
    print(f"Time of flight(s): {s:.2f}")
    print(f"Time of launch (s): {(t_collide - s + TimeOfPrediction):.2f}") 
    print(" ")
    print(f"Time of interseption (s): {(t_collide + TimeOfPrediction):.2f}")
    print(f"Location of interseption (x,y,z)(m): [{x_2[-1]:.2f} {y_2[-1]:.2f} {z_2[-1]:.2f}]")

# Display results if no interseption is possible
else:
    print(f"No possible interseption within given parameters")
    
