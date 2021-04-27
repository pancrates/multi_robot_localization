import numpy as np
from scipy import stats
import matplotlib.pyplot as plt

# normalize angle in radians to [-np.pi, np.pi]
def anorm(phi):
    return (phi + np.pi) % (2 * np.pi) - np.pi

# robot part

class Robot:
    def __init__(self, pos=None):
        self.MCL = MCL(5, 0.1, pos)

    def get_particles(self):
        return self.MCL.x, self.MCL.y, self.MCL.phi

    def get_relative_error(self):
        return self.MCL.relative_error

    def move_step(self):
        u, v = 0.5, np.random.uniform(low=-0.5, high=0.5)
        self.MCL.apply_motion_model(u, v)
        return u, v

    def localization_step(self, msgs, real_phi):
        self.MCL.apply_detection_model(msgs, real_phi)

    def localization_step_abs(self, real_x, real_y, real_phi):
        self.MCL.apply_fixed_localization(real_x, real_y, real_phi)

class MCL:
    def __init__(self, size, alpha=0.2, pos=None):
        self.size = size
        self.alpha = alpha
        self.relative_error = np.zeros(4)
        if pos is None:
            self.x = np.random.uniform(low=-8, high=8, size=size)
            self.y = np.random.uniform(low=-8, high=8, size=size)
            self.phi = np.random.uniform(low=-np.pi, high=np.pi, size=size)
        else:
            self.x = np.random.normal(loc=pos[0], size=size)
            self.y = np.random.normal(loc=pos[1], size=size)
            self.phi = np.random.normal(loc=pos[2], scale=0.1, size=size)

    def apply_motion_model(self, u, v):
        dx = np.cos(self.phi) * u + np.random.normal(scale=0.1, size=self.size)
        dy = np.sin(self.phi) * u + np.random.normal(scale=0.1, size=self.size)
        dphi = v + np.random.normal(scale=0.1)
        self.x = (self.x + dx)
        self.y = (self.y + dy)
        self.phi = anorm(self.phi + dphi)

    def apply_fixed_localization(self, real_x, real_y, real_phi):
        self.x = np.random.normal(loc=real_x, scale=0.05, size=self.size)
        self.y = np.random.normal(loc=real_y, scale=0.05, size=self.size)
        self.phi = np.random.normal(loc=real_phi, scale=0.05, size=self.size)

    def apply_detection_model(self, msgs, real_phi):
        if len(msgs) == 0:
            return
        size = self.size
        weights = self.get_simple_weights(msgs, real_phi)
        recp_x, recp_y, recp_phi = self.get_reciprocal_particles(msgs)
        new_x = np.zeros(size)
        new_y = np.zeros(size)
        new_phi = np.zeros(size)
        for i_new in range(size):
            if np.random.uniform() < self.alpha:
                i_old = np.random.choice(len(recp_phi))
                new_x[i_new] = recp_x[i_old]
                new_y[i_new] = recp_y[i_old]
                new_phi[i_new] = recp_phi[i_old]
            else:
                i_old = np.random.choice(size, p=weights)
                new_x[i_new] = self.x[i_old]
                new_y[i_new] = self.y[i_old]
                new_phi[i_new] = self.phi[i_old]
        self.x = new_x
        self.y = new_y
        self.phi = new_phi

    def get_simple_weights(self, msgs, real_phi):
        self.relative_error = np.zeros(4)
        weights = np.zeros(self.size)
        for i in range(self.size):
            w = 0
            for msg in msgs:
                w += self.particle_weight(self.x[i], self.y[i], self.phi[i], msg, real_phi)
            weights[i] = w
        return weights / np.sum(weights)

    def get_reciprocal_particles(self, msgs):
        recp_x = []
        recp_y = []
        recp_phi = []
        for msg in msgs:
            angle = anorm(msg['pphi'] + msg['theta'])
            new_x = msg['px'] + np.cos(angle) * msg['r']
            new_y = msg['py'] + np.sin(angle) * msg['r']
            new_phi = anorm(np.pi + msg['pphi'] + msg['theta'] - msg['theta2'])
            recp_x.append(new_x)
            recp_y.append(new_y)
            recp_phi.append(new_phi)
        return np.concatenate(recp_x), np.concatenate(recp_y), np.concatenate(recp_phi)

    def particle_weight(self, x, y, phi, msg, real_phi):
        dx = msg['px'] - x
        dy = msg['py'] - y
        dr = np.hypot(dx, dy) - msg['r']
        dtheta = anorm(np.arctan2(dy, dx) - (msg['pphi'] + msg['theta']))
        dphi = anorm(np.pi + msg['pphi'] - phi + msg['theta'] - msg['theta2'])
        drealphi = anorm(phi - real_phi)
        prob = stats.norm.pdf(dr, scale=0.1) * stats.norm.pdf(dtheta, scale=0.1) *stats.norm.pdf(dphi, scale=0.1) * stats.norm.pdf(drealphi, scale=0.1)
        self.relative_error[0] += np.sum(np.abs(dr))
        self.relative_error[1] += np.sum(np.abs(dtheta))
        self.relative_error[2] += np.sum(np.abs(dphi))
        self.relative_error[3] += np.sum(np.abs(drealphi))
        return np.average(prob)

# simulator part

class Simulator:
    def __init__(self, nrobots=2, loc=False):
        self.nrobots = nrobots
        if loc:
            self.robots = [Robot((0,0,0))] + [Robot() for _ in range(nrobots-1)]
        else:
            self.robots = [Robot() for _ in range(nrobots)]
        self.positions = np.zeros((nrobots, 3))
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#a02c2c']
        self.history = []
        self.positions[:,0] = np.random.uniform(low=-5, high=5, size=nrobots)
        self.positions[:,1] = np.random.uniform(low=-5, high=5, size=nrobots)
        self.positions[:,2] = np.random.uniform(low=-np.pi, high=np.pi, size=nrobots)
        if loc:
            self.positions[0] = (0,0,0)
        self.cont_loc = loc
        self.max_distance = 30

    def get_sensor_readings(self):
        nrobots = self.nrobots
        xs = self.positions[:,0]
        ys = self.positions[:,1]
        phis = self.positions[:,2]
        dx = xs[np.newaxis, :] - xs[:, np.newaxis]
        dy = ys[np.newaxis, :] - ys[:, np.newaxis]
        dist = np.hypot(dx, dy)
        theta = np.arctan2(dy, dx) - phis[:, np.newaxis]
        dist_noise = np.random.normal(scale=0.01, size=(nrobots, nrobots))
        theta_noise = np.random.normal(scale=0.01, size=(nrobots, nrobots))
        sensor_dist = np.clip(dist + dist_noise, 0, None)
        sensor_theta = anorm(theta + theta_noise)
        return sensor_dist, sensor_theta

    def run_motion_step(self):
        for i_robot in range(self.nrobots):
            robot = self.robots[i_robot]
            u, v = robot.move_step()
            x, y, phi = self.positions[i_robot]
            x += np.cos(phi) * u + np.random.normal(scale=0.01)
            y += np.sin(phi) * u + np.random.normal(scale=0.01)
            phi = anorm(phi + v)
            self.positions[i_robot] = (x, y, phi)

    def run_localization_step(self):
        s_dist, s_theta = self.get_sensor_readings()
        all_messages = []
        for i_to in range(self.nrobots):
            msgs = []
            for i_from in range(self.nrobots):
                if i_to == i_from: continue
                if s_dist[i_from,i_to] > self.max_distance:
                    continue
                p_x, p_y, p_phi = self.robots[i_from].get_particles()
                msg = {
                    'from': i_from,
                    'r': s_dist[i_from,i_to],
                    'theta': s_theta[i_from,i_to],
                    'theta2': s_theta[i_to,i_from],
                    'px': np.copy(p_x),
                    'py': np.copy(p_y),
                    'pphi': np.copy(p_phi),
                }
                msgs.append(msg)
            all_messages.append(msgs)
        entry = {'px':[],'py':[],'pphi':[],'positions':np.copy(self.positions),'relative':[]}
        for i_to in range(self.nrobots):
            robot = self.robots[i_to]
            if i_to == 0 and self.cont_loc:
                real_x = self.positions[i_to,0] + np.random.normal(scale=0.1)
                real_y = self.positions[i_to,1] + np.random.normal(scale=0.1)
                real_phi = anorm(self.positions[i_to,2] + np.random.normal(scale=0.05))
                robot.localization_step_abs(real_x, real_y, real_phi)
            else:
                real_phi = anorm(self.positions[i_to,2] + np.random.normal(scale=0.1))
                robot.localization_step(all_messages[i_to], real_phi)
            p_x, p_y, p_phi = robot.get_particles()
            entry['px'].append(p_x)
            entry['py'].append(p_y)
            entry['pphi'].append(p_phi)
            entry['relative'].append(robot.get_relative_error())
        self.history.append(entry)

    def run_simulation(self):
        for _ in range(100):
            self.run_motion_step()
            self.run_localization_step()
        self.show()

    def show_errors(self):
        fig3, ax3 = plt.subplots()
        fig4, ax4 = plt.subplots()
        nrobots = len(self.robots)
        cum_err = []
        rel_err = []
        for i, entry in enumerate(self.history):
            sum_e = 0
            sum_r = 0
            for i_robot in range(nrobots):
                p_x = entry['px'][i_robot]
                p_y = entry['py'][i_robot]
                pos = entry['positions'][i_robot]
                e = np.hypot(p_x - pos[0], p_y - pos[1])
                sum_e += e
                rel =entry['relative'][i_robot]
                sum_r +=  np.array([rel[0],rel[2],rel[3]])
            cum_err.append(sum_e/len(p_x))
            rel_err.append(sum_r/len(p_x))
        ax3.plot(range(len(cum_err)), cum_err)
        ax4.plot(range(len(rel_err)), rel_err)
        print('cumulative error:')
        print('  start:', cum_err[0],sum(cum_err[0]))
        print('    end:', cum_err[-1],sum(cum_err[-1]))
        print('relative error:')
        print('  start:', rel_err[0], sum(rel_err[0]))
        print('    end:', rel_err[-1],sum(rel_err[-1]))

    def show(self):
        nrobots = len(self.robots)
        fig1, ax1 = plt.subplots()
        for entry in self.history:
            for i_robot in range(nrobots):
                color = self.colors[i_robot]
                p_x = entry['px'][i_robot]
                p_y = entry['py'][i_robot]
                p_phi = entry['pphi'][i_robot]
                for i_p in range(len(p_x)):
                    angle = (p_phi[i_p] * 180 / np.pi) - 90
                    ax1.plot(p_x[i_p], p_y[i_p], marker=(2, 0, angle), color=color)
        fig2, ax2 = plt.subplots()
        for i_robot in range(nrobots):
            color = self.colors[i_robot]
            pos = [h['positions'][i_robot] for h in self.history]
            pos_x = [p[0] for p in pos]
            pos_y = [p[1] for p in pos]
            ax2.plot(pos_x, pos_y, color=color)
        self.show_errors()
        plt.show()

sim = Simulator(2,True)
sim.run_simulation()
