import numpy as np
import time
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

def jacobian_inverse(q):
    J = jacobian(q)
    return np.linalg.pinv(J)

def jacobian(q):
    return np.array([
        [-0.1 * (np.cos(q[1]) + np.cos(q[1] + q[2])) * np.sin(q[0]), -0.1 * (np.sin(q[1]) + np.sin(q[1] + q[2])) * np.cos(q[0]), -0.1 * np.sin(q[1] + q[2]) * np.cos(q[0])],
        [0.1 * (np.cos(q[1]) + np.cos(q[1] + q[2])) * np.cos(q[0]), -0.1 * (np.sin(q[1]) + np.sin(q[1] + q[2])) * np.sin(q[0]), -0.1 * np.sin(q[0]) * np.sin(q[1] + q[2])],
        [0, 0.1 * (np.cos(q[1]) + np.cos(q[1] + q[2])), 0.1 * np.cos(q[1] + q[2])]
    ])

def fkine(q):
    return np.array([
        0.1 * (np.cos(q[1]) + np.cos(q[1] + q[2])) * np.cos(q[0]),
        0.1 * (np.cos(q[1]) + np.cos(q[1] + q[2])) * np.sin(q[0]),
        0.1 * np.sin(q[1]) + 0.1 * np.sin(q[1] + q[2]) + 0.05
    ])
# Time parameters
start_time = time.time()
current_time = 0
plot_xd = []
plot_xa = []
plot_t = []
q = np.array([0,np.pi/4,-np.pi/2])
K = np.eye(3)  # Gain matrix
dt = 0.5
# Loop for 5 seconds
while current_time < 5:
    current_time = time.time() - start_time
    t = current_time

    xa = fkine(q)
    # Compute the Jacobian inverse
    J_i = jacobian_inverse(q)
    # Desired trajectory (example: a circular trajectory in the XY plane)
    xd = np.array([
        0.1 * np.cos(t),  # X coordinates
        0.1,  # Y coordinates
        0.1  # Z coordinates (constant height)
    ]).T

    # Error
    e = xd - xa

    # Desired velocity (derivative of the desired trajectory)
    dxd = np.array([
        -0.1 * np.sin(t),  # dX/dt
        0,   # dY/dt
        0   # dZ/dt (no change in height)
    ]).T

    # Control
    u = K @ e + dxd
    # Update the joint angles
    q_update = J_i @ u * dt

    q = q + q_update
    print(q_update*180/np.pi)
    print(q*180/np.pi)

    plot_xd.append(xd)
    plot_xa.append(xa)
    plot_t.append(t)
    # Sleep for a short duration to simulate real-time loop
    time.sleep(dt)

plot_xd = np.array(plot_xd)
plot_t = np.array(plot_t)

# Plot the desired trajectory
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
ax1.plot(plot_t, plot_xd[:, 0], label='X Trajectory')
ax1.plot(plot_t, plot_xd[:, 1], label='Y Trajectory')
ax1.plot(plot_t, plot_xd[:, 2], label='Z Trajectory')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Position')
ax1.set_title('Desired Trajectory')
ax1.legend()

# Plot actual trajectory
plot_xa = np.array(plot_xa)  # Ensure plot_xa is a numpy array
ax2.plot(plot_t, plot_xa[:, 0], label='X Actual')
ax2.plot(plot_t, plot_xa[:, 1], label='Y Actual')
ax2.plot(plot_t, plot_xa[:, 2], label='Z Actual')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Position')
ax2.set_title('Actual Trajectory')
ax2.legend()

plt.tight_layout()
plt.show()