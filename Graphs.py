import matplotlib.pyplot as plt
from FilterOutput import FilterOutput

KalmanEstimate = FilterOutput()

signal_x = KalmanEstimate[1]
signal_y = KalmanEstimate[2]
signal_z = KalmanEstimate[3]

estimate_x = KalmanEstimate[6]
estimate_y = KalmanEstimate[7]
estimate_z = KalmanEstimate[8]

variance_pos = KalmanEstimate[9]
variance_vel = KalmanEstimate[10]

t = KalmanEstimate[5]

fig = plt.figure(figsize=(10,10))

plt.suptitle("Kalman Estimation and Signal")
plt.subplot(3,1,1)
plt.title("x position filter")
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.plot(t[-200::],
         signal_x[-200::],
         'r--o',
         markersize=3,
         label='Signal')
plt.plot(t[-200::],
         estimate_x[-200::],
         'k:*',
         markersize=3,
         label='Kalman Estimate')
plt.legend()

plt.subplot(3,1,2)
plt.title("y position filter")
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.plot(t[-200::],
         signal_y[-200::],
         'b--o',
         markersize=3,
         label='Signal')
plt.plot(t[-200::],
         estimate_y[-200::],
         'k:*',
         markersize=3,
         label='Kalman Estimate')
plt.legend()

plt.subplot(3,1,3)
plt.title("z position filter")
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.plot(t[-200::],
         signal_z[-200::],
         'g--o',
         markersize=3,
         label='Signal')
plt.plot(t[-200::],
         estimate_z[-200::],
         'k:*',
         markersize=3,
         label='Kalman Estimate')
plt.legend()

plt.tight_layout()
plt.savefig("KalmanEstimationAndSignalLast.png")
plt.close('all')

fig = plt.figure(figsize=(10,10))

plt.suptitle("Kalman Estimation and Signal")
plt.subplot(3,1,1)
plt.title("x position filter")
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.plot(t[:200:],
         signal_x[:200:],
         'r--o',
         markersize=3,
         label='Signal')
plt.plot(t[:200:],
         estimate_x[:200:],
         'k:*',
         markersize=3,
         label='Kalman Estimate')
plt.legend()

plt.subplot(3,1,2)
plt.title("y position filter")
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.plot(t[:200:],
         signal_y[:200:],
         'b--o',
         markersize=3,
         label='Signal')
plt.plot(t[:200:],

         estimate_y[:200:],
         'k:*',
         markersize=3,
         label='Kalman Estimate')
plt.legend()

plt.subplot(3,1,3)
plt.title("z position filter")
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.plot(t[:200:],
         signal_z[:200:],
         'g--o',
         markersize=3,
         label='Signal')
plt.plot(t[:200:],
         estimate_z[:200:],

         'k:*',
         markersize=3,
         label='Kalman Estimate')
plt.legend()

plt.tight_layout()
plt.savefig("KalmanEstimationAndSignalFirst.png")
plt.close('all')

fig = plt.figure(figsize=(6,6))
plt.title("Position Variance with Iterations")
plt.xlabel("Iteration")
plt.ylabel("Variance (m^2)")
plt.grid(True)
plt.plot(range(0,len(t))[::3], variance_pos[::3]) 
plt.savefig("PositionVariance")
plt.close()

fig = plt.figure(figsize=(6,6))
plt.title("Velocity Variance with Iterations")
plt.xlabel("Iteration")
plt.ylabel("Variance ((m/s)^2)")
plt.grid(True)
plt.plot(range(0,len(t))[::3], variance_vel[::3]) 
plt.savefig("VelocityVariance")
plt.close()
