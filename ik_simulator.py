import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button

import matplotlib
matplotlib.use('TkAgg') # The most stable backend for Mac UI widgets

class SpatialArm:
    def __init__(self, lengths):
        self.lengths = lengths
        self.num_links = len(lengths)
        self.angles = np.array([0.5, 0.4, 0.8, 0.4, 0.2])

    def forward_kinematics(self, angles=None):
        if angles is None: angles = self.angles
        pts = np.zeros((self.num_links + 1, 3))
        yaw, curr_pitch, curr_pos = angles[0], 0, np.zeros(3)
        for i in range(1, self.num_links + 1):
            curr_pitch += angles[i]
            r = self.lengths[i-1] * np.cos(curr_pitch)
            curr_pos[0] += r * np.cos(yaw)
            curr_pos[1] += r * np.sin(yaw)
            curr_pos[2] += self.lengths[i-1] * np.sin(curr_pitch)
            pts[i] = curr_pos.copy()
        return pts

    def update_ik(self, target):
        for _ in range(10):
            pos = self.forward_kinematics()[-1]
            error = target - pos
            if np.linalg.norm(error) < 0.001: break
            J = self.get_jacobian()
            damping = 0.12
            J_dls = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(3))
            self.angles += np.clip(J_dls.dot(error) * 0.5, -0.2, 0.2)

    def get_jacobian(self):
        num_vars = len(self.angles)
        J = np.zeros((3, num_vars))
        delta = 1e-6
        pos = self.forward_kinematics()[-1]
        for i in range(num_vars):
            temp = self.angles[i]
            self.angles[i] += delta
            J[:, i] = (self.forward_kinematics()[-1] - pos) / delta
            self.angles[i] = temp
        return J

class RoboticDashboard:
    def __init__(self):
        self.arm = SpatialArm([1.2, 1.0, 0.8, 0.6])
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d', position=[0.05, 0.15, 0.65, 0.8])
        
        # Define Sliders
        self.ax_x = plt.axes([0.75, 0.4, 0.15, 0.03])
        self.ax_y = plt.axes([0.75, 0.3, 0.15, 0.03])
        self.ax_z = plt.axes([0.75, 0.2, 0.15, 0.03])
        self.ax_res = plt.axes([0.75, 0.1, 0.1, 0.05])

        self.s_x = Slider(self.ax_x, 'X', -3.0, 3.0, valinit=2.0, color='#e74c3c')
        self.s_y = Slider(self.ax_y, 'Y', -3.0, 3.0, valinit=0.0, color='#3498db')
        self.s_z = Slider(self.ax_z, 'Z', 0.1, 3.5, valinit=1.5, color='#2ecc71')
        self.btn = Button(self.ax_res, 'HOME POSE', color='#bdc3c7')

        # Connect Events
        self.s_x.on_changed(self.update)
        self.s_y.on_changed(self.update)
        self.s_z.on_changed(self.update)
        self.btn.on_clicked(self.reset)

        self.update(None) # Initial Draw

    def update(self, val):
        target = np.array([self.s_x.val, self.s_y.val, self.s_z.val])
        self.ax.clear()
        self.ax.set_xlim(-3, 3); self.ax.set_ylim(-3, 3); self.ax.set_zlim(0, 4)
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        
        self.arm.update_ik(target)
        pts = self.arm.forward_kinematics()
        
        # High-Vis Render
        self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 'o-', lw=8, color='#2c3e50', markersize=10)
        self.ax.scatter(target[0], target[1], target[2], color='red', s=200, alpha=0.5)
        
        # HUD Data
        error = np.linalg.norm(target - pts[-1])
        self.ax.text2D(0.05, 0.95, f"Tracking Error: {error:.4f}m", transform=self.ax.transAxes, fontweight='bold')
        
        self.fig.canvas.draw_idle()

    def reset(self, event):
        # 1. Block signals to prevent math jitters during reset
        self.s_x.eventson = self.s_y.eventson = self.s_z.eventson = False
        
        # 2. Reset values
        self.s_x.reset()
        self.s_y.reset()
        self.s_z.reset()
        
        # 3. Hard reset arm state
        self.arm.angles = np.array([0.5, 0.4, 0.8, 0.4, 0.2])
        
        # 4. Restore signals and redraw
        self.s_x.eventson = self.s_y.eventson = self.s_z.eventson = True
        self.update(None)

if __name__ == "__main__":
    print("Launching Class-Based Control Dashboard...")
    dashboard = RoboticDashboard()
    plt.suptitle("Precision 5-DOF Spatial Control Interface", fontsize=14, y=0.95)
    plt.show()