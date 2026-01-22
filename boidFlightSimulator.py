import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

class BoidFlightSimulator:
    def __init__(self, spaceSize=10, numBoids=50, maxSpeed=2.0, visionR=3.0, dt=0.05):
        self.spaceSize = spaceSize
        self.numBoids = numBoids
        self.maxSpeed = maxSpeed
        self.visionR = visionR
        self.dt = dt
        
        self.pos = np.random.rand(numBoids, 2) * spaceSize
        self.angles = 2 * np.pi * np.random.rand(numBoids)
        self.vel = np.stack([np.cos(self.angles), np.sin(self.angles)], axis=1) * maxSpeed
        
        self.w_align = 0.8
        self.w_coh = 0.8
        self.w_sep = 0.6
        self.w_rand = 0.1
        
        self.set_visualizer()
        
    def set_visualizer(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.quiver = self.ax.quiver(self.pos[:,0], self.pos[:,1],
                                     self.vel[:,0], self.vel[:,1],
                                     angles='xy', scale_units='xy', scale=7)
        self.ax.set_xlim(0, self.spaceSize)
        self.ax.set_ylim(0, self.spaceSize)
        self.ax.set_aspect('equal', adjustable='box')
        
        plt.subplots_adjust(bottom=0.35)
        ax_align = plt.axes([0.2, 0.25, 0.6, 0.03])
        ax_coh   = plt.axes([0.2, 0.20, 0.6, 0.03])
        ax_sep   = plt.axes([0.2, 0.15, 0.6, 0.03])
        ax_rand  = plt.axes([0.2, 0.10, 0.6, 0.03])
        ax_vis   = plt.axes([0.2, 0.05, 0.6, 0.03])

        self.s_align = Slider(ax_align, "Align", 0.0, 2.0, valinit=self.w_align)
        self.s_coh   = Slider(ax_coh,   "Cohesion", 0.0, 2.0, valinit=self.w_coh)
        self.s_sep   = Slider(ax_sep,   "Separation", 0.0, 2.0, valinit=self.w_sep)
        self.s_rand  = Slider(ax_rand,  "Random", 0.0, 2.0, valinit=self.w_rand)
        self.s_vis   = Slider(ax_vis,   "VisionR", 0.1, self.spaceSize, valinit=self.visionR)

        for s in (self.s_align, self.s_coh, self.s_sep, self.s_rand, self.s_vis):
            s.on_changed(self.updateSliders)

    def updateSliders(self):
        self.w_align = self.s_align.val
        self.w_coh = self.s_coh.val
        self.w_sep = self.s_sep.val
        self.w_rand = self.s_rand.val
        self.visionR = self.s_vis.val
        
    def periodic_diff(self, a):
        dx = a[:,None] - a[None,:]
        L = self.spaceSize
        dx = (dx + L/2) % L - L/2
        return dx

    def get_neighbors(self):
        dx = self.periodic_diff(self.pos[:,0])
        dy = self.periodic_diff(self.pos[:,1])
        dist = np.sqrt(dx**2 + dy**2)
        neighborsAngle = np.arctan2(dy,dx)
        angleVel = self.angles
        angleDiff = neighborsAngle - angleVel
        angleDiff = (angleDiff + np.pi) % (2*np.pi) - np.pi
        fov = angleDiff < np.pi / 6
        neighbors = (dist < self.visionR) & (dist > 0) & fov
        return dist, neighbors

    def limit_speed(self):
        speed = np.linalg.norm(self.vel, axis=1)
        mask = speed > self.maxSpeed
        self.vel[mask] *= (self.maxSpeed / speed[mask])[:,None]
        
    def alignment(self, neighbors):
        acc = np.zeros_like(self.vel)
        for i in range(self.numBoids):
            idx = neighbors[i]
            if np.any(idx):
                mean_vel = np.mean(self.vel[idx], axis=0)
                acc[i] = mean_vel - self.vel[i]
        return acc


    def cohesion(self, neighbors):
        acc = np.zeros_like(self.vel)
        L = self.spaceSize

        for i in range(self.numBoids):
            idx = neighbors[i]
            if np.any(idx):
                diff = self.pos[idx] - self.pos[i]
                diff = (diff + L/2) % L - L/2
                acc[i] = np.mean(diff, axis=0)

        return acc


    def separation(self, neighbors, dist):
        acc = np.zeros_like(self.vel)
        for i in range(self.numBoids):
            idx = neighbors[i] & (dist[i] < self.visionR/3)
            if np.any(idx):
                diff = self.pos[i] - self.pos[idx]
                diff = (diff + self.spaceSize/2) % self.spaceSize - self.spaceSize/2
                acc[i] = np.sum(diff / (np.linalg.norm(diff, axis=1)[:,None] + 1e-6), axis=0)
        return acc

    def random_force(self):
        angles = 2*np.pi*np.random.rand(self.numBoids)
        return np.stack([np.cos(angles), np.sin(angles)], axis=1)
    
    def updateBoids(self):
        dist, neighbors = self.get_neighbors()
        acc = (self.w_align * self.alignment(neighbors) +
               self.w_coh * self.cohesion(neighbors) +
               self.w_sep * self.separation(neighbors, dist) +
               self.w_rand * self.random_force())

        self.vel += acc * self.dt
        self.limit_speed()
        self.pos = (self.pos + self.vel * self.dt) % self.spaceSize
        
    def plotting(self):
        angles = np.arctan2(self.vel[:,1], self.vel[:,0])
        self.quiver.set_offsets(self.pos)
        self.quiver.set_UVC(np.cos(angles), np.sin(angles))
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def startSimulation(self, steps=300):
        for _ in range(steps):
            self.updateBoids()
            self.plotting()
            plt.pause(0.001)


boidRun = BoidFlightSimulator(spaceSize=10, numBoids=500, maxSpeed=5.0, visionR=3.0, dt=0.05)
boidRun.startSimulation(steps=2000)