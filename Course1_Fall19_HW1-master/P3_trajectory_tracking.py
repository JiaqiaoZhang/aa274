import numpy as np
from numpy import linalg

V_PREV_THRES = 0.0001


class TrajectoryTracker:
    """ Trajectory tracking controller using differential flatness """

    def __init__(self, kpx, kpy, kdx, kdy, V_max=0.5, om_max=1):
        self.kpx = kpx
        self.kpy = kpy
        self.kdx = kdx
        self.kdy = kdy

        self.V_max = V_max
        self.om_max = om_max

        self.coeffs = np.zeros(8)  # Polynomial coefficients for x(t) and y(t) as
        # returned by the differential flatness code

    def reset(self):
        self.V_prev = 0
        self.om_prev = 0
        self.t_prev = 0

    def load_traj(self, times, traj):
        """ Loads in a new trajectory to follow, and resets the time """
        self.reset()
        self.traj_times = times
        self.traj = traj

    def get_desired_state(self, t):
        """
        Input:
            t: Current time
        Output:
            x_d, xd_d, xdd_d, y_d, yd_d, ydd_d: Desired state and derivatives
                at time t according to self.coeffs
        """
        x_d = np.interp(t, self.traj_times, self.traj[:, 0])
        y_d = np.interp(t, self.traj_times, self.traj[:, 1])
        xd_d = np.interp(t, self.traj_times, self.traj[:, 3])
        yd_d = np.interp(t, self.traj_times, self.traj[:, 4])
        xdd_d = np.interp(t, self.traj_times, self.traj[:, 5])
        ydd_d = np.interp(t, self.traj_times, self.traj[:, 6])

        return x_d, xd_d, xdd_d, y_d, yd_d, ydd_d

    def compute_control(self, x, y, th, t):
        """
        Inputs:
            x,y,th: Current state
            t: Current time
        Outputs: 
            V, om: Control actions
        """

        dt = t - self.t_prev
        x_d, xd_d, xdd_d, y_d, yd_d, ydd_d = self.get_desired_state(t)

        ########## Code starts here ##########
        V = 0
        om = 0
        if np.absolute(V) < V_PREV_THRES:
            V = np.sign(V) * V_PREV_THRES
        xd = self.V_prev * np.cos(th)
        yd = self.V_prev * np.sin(th)
        u1 = xdd_d + self.kpx * (x_d - x) + self.kdx * (xd_d - xd)
        u2 = ydd_d + self.kpy * (y_d - y) + self.kdy * (yd_d - yd)
        J = np.array([[np.cos(th), -np.sin(th)],
                      [np.sin(th), np.cos(th)]])
        U = np.array([[u1], [u2]])

        X = np.linalg.solve(J, U)  # store the control input Vdot and V*om
        V = X[0][0] * dt + self.V_prev
        om = X[1][0] / self.V_prev

        ########## Code ends here ##########

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
        om = np.clip(om, -self.om_max, self.om_max)

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        return V, om
