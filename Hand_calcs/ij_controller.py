import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SCARAKinematics:
    def __init__(self):
        # Initial joint angles
        self.q = np.array([0.0, np.pi/4, -np.pi/2])
        self.t = 0.0
        self.dt = 0.01
        
        # Desired end-effector position
        self.pd = np.array([0.097, 0.096, 0.058])
        
        # Proportional gain matrix
        self.K = np.eye(3)
        
    def direct_kinematics_SCARA(self, q):
        """Calculate end-effector position using direct kinematics"""
        xe = np.zeros(3)
        xe[0] = (0.048 * np.cos(q[0] - q[1]) + 
                 0.048 * np.cos(q[0] + q[1]) + 
                 0.049 * np.cos(-q[0] + q[1] + q[2]) + 
                 0.049 * np.cos(q[0] + q[1] + q[2]))
        
        xe[1] = (0.048 * np.sin(q[0] - q[1]) + 
                 0.048 * np.sin(q[0] + q[1]) - 
                 0.049 * np.sin(-q[0] + q[1] + q[2]) + 
                 0.049 * np.sin(q[0] + q[1] + q[2]))
        
        xe[2] = (0.096 * np.sin(q[1]) + 
                 0.098 * np.sin(q[1] + q[2]) + 
                 0.06)
        
        return xe
    
    def analytical_jacobian(self, q):
        """Calculate the analytical Jacobian matrix"""
        J_A = np.zeros((3, 3))
        
        # First row
        J_A[0, 0] = (-0.048 * np.sin(q[0] - q[1]) - 
                     0.048 * np.sin(q[0] + q[1]) + 
                     0.049 * np.sin(-q[0] + q[1] + q[2]) - 
                     0.049 * np.sin(q[0] + q[1] + q[2]))
        J_A[0, 1] = (0.048 * np.sin(q[0] - q[1]) - 
                     0.048 * np.sin(q[0] + q[1]) - 
                     0.049 * np.sin(-q[0] + q[1] + q[2]) - 
                     0.049 * np.sin(q[0] + q[1] + q[2]))
        J_A[0, 2] = (-0.049 * np.sin(-q[0] + q[1] + q[2]) - 
                     0.049 * np.sin(q[0] + q[1] + q[2]))
        
        # Second row
        J_A[1, 0] = (0.048 * np.cos(q[0] - q[1]) + 
                     0.048 * np.cos(q[0] + q[1]) + 
                     0.049 * np.cos(-q[0] + q[1] + q[2]) + 
                     0.049 * np.cos(q[0] + q[1] + q[2]))
        J_A[1, 1] = (-0.048 * np.cos(q[0] - q[1]) + 
                     0.048 * np.cos(q[0] + q[1]) - 
                     0.049 * np.cos(-q[0] + q[1] + q[2]) + 
                     0.049 * np.cos(q[0] + q[1] + q[2]))
        J_A[1, 2] = (-0.049 * np.cos(-q[0] + q[1] + q[2]) + 
                     0.049 * np.cos(q[0] + q[1] + q[2]))
        
        # Third row
        J_A[2, 0] = 0
        J_A[2, 1] = (0.096 * np.cos(q[1]) + 
                     0.098 * np.cos(q[1] + q[2]))
        J_A[2, 2] = 0.098 * np.cos(q[1] + q[2])
        
        return J_A
    
    def compute_joint_angle_update(self):
        """Compute joint angle updates using Jacobian pseudoinverse"""
        pd_dot = np.array([0.0, 0.0, 0])
        
        x_e = self.direct_kinematics_SCARA(self.q)
        J_A = self.analytical_jacobian(self.q)
        
        # Error calculation
        e = self.pd - x_e
        
        # Pseudoinverse using numpy's built-in method
        J_A_pseudo_inverse = np.linalg.pinv(J_A)
        
        J_A_inverse = np.linalg.inv(J_A)
        # Joint velocity calculation
        qdot = J_A_inverse @ (pd_dot + self.K @ e)
        
        return qdot
    
    def simulate(self, max_iterations=500):
        """Simulate robot movement and track positions"""
        positions = []
        joint_angles = []
        
        for _ in range(max_iterations):
            # Compute joint angle updates
            qdot = self.compute_joint_angle_update()
            
            # Update joint angles
            self.q += qdot * self.dt
            
            # Track positions
            current_pos = self.direct_kinematics_SCARA(self.q)
            positions.append(current_pos)
            joint_angles.append(self.q.copy())
            
            # Check convergence
            error = np.linalg.norm(self.pd - current_pos)
            if error < 0.005:
                break
        
        return np.array(positions), np.array(joint_angles)
    
    def plot_results(self, positions, joint_angles):
        """Create multiple plots for visualization"""
        plt.figure(figsize=(15, 5))
        
        # End-effector position trajectory
        plt.subplot(131)
        plt.plot(positions[:, 0], positions[:, 1], label='XY Trajectory')
        plt.scatter(self.pd[0], self.pd[1], color='red', label='Target')
        plt.title('End-Effector XY Trajectory')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        
        # Z position trajectory
        plt.subplot(132)
        plt.plot(positions[:, 2], label='Z Position')
        plt.title('End-Effector Z Position')
        plt.xlabel('Iteration')
        plt.ylabel('Z Position')
        plt.legend()
        
        # Joint angles
        plt.subplot(133)
        plt.plot(joint_angles[:, 0] * 180/np.pi, label='Joint 1')
        plt.plot(joint_angles[:, 1] * 180/np.pi, label='Joint 2')
        plt.plot(joint_angles[:, 2] * 180/np.pi, label='Joint 3')
        plt.title('Joint Angles')
        plt.xlabel('Iteration')
        plt.ylabel('Angle (degrees)')
        plt.legend()
        
        plt.tight_layout()
        plt.show()

def main():
    # Create SCARA kinematics simulation
    scara = SCARAKinematics()
    
    # Run simulation
    positions, joint_angles = scara.simulate()
    
    # Plot results
    scara.plot_results(positions, joint_angles)

if __name__ == "__main__":
    main()