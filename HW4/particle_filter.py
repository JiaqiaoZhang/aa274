import numpy as np
import scipy.linalg  # You may find scipy.linalg.block_diag useful
import scipy.stats  # You may find scipy.stats.multivariate_normal.pdf useful
import turtlebot_model as tb

EPSILON_OMEGA = 1e-3

class ParticleFilter(object):
    """
    Base class for Monte Carlo localization and FastSLAM.

    Usage:
        pf = ParticleFilter(x0, R)
        while True:
            pf.transition_update(u, dt)
            pf.measurement_update(z, Q)
            localized_state = pf.x
    """

    def __init__(self, x0, R):
        """
        ParticleFilter constructor.

        Inputs:
            x0: np.array[M,3] - initial particle states.
             R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
        """
        self.M = x0.shape[0]  # Number of particles
        self.xs = x0  # Particle set [M x 3]
        self.ws = np.repeat(1. / self.M, self.M)  # Particle weights (initialize to uniform) [M]
        self.R = R  # Control noise covariance (corresponding to dt = 1 second) [2 x 2]

    @property
    def x(self):
        """
        Returns the particle with the maximum weight for visualization.

        Output:
            x: np.array[3,] - particle with the maximum weight.
        """
        idx = self.ws == self.ws.max()
        x = np.zeros(self.xs.shape[1:])
        x[:2] = self.xs[idx,:2].mean(axis=0)
        th = self.xs[idx,2]
        x[2] = np.arctan2(np.sin(th).mean(), np.cos(th).mean())
        return x

    def transition_update(self, u, dt):
        """
        Performs the transition update step by updating self.xs.

        Inputs:
             u: np.array[2,] - zero-order hold control input.
            dt: float        - duration of discrete time step.
        Output:
            None - internal belief state (self.xs) should be updated.
        """
        ########## Code starts here ##########
        # TODO: Update self.xs.
        # Hint: Call self.transition_model().
        us = np.ones((self.M, 2))
        us[:, 0] = np.random.normal(u[0], scale=self.R[0, 0], size=self.M)
        us[:, 1] = np.random.normal(u[1], scale=self.R[1, 1], size=self.M)
        self.xs = self.transition_model(us, dt)

        ########## Code ends here ##########

    def transition_model(self, us, dt):
        """
        Propagates exact (nonlinear) state dynamics.

        Inputs:
            us: np.array[M,2] - zero-order hold control input for each particle.
            dt: float         - duration of discrete time step.
        Output:
            g: np.array[M,3] - result of belief mean for each particle
                               propagated according to the system dynamics with
                               control u for dt seconds.
        """
        raise NotImplementedError("transition_model must be overridden by a subclass of EKF")

    def measurement_update(self, z_raw, Q_raw):
        """
        Updates belief state according to the given measurement.

        Inputs:
            z_raw: np.array[2,I]   - matrix of I columns containing (alpha, r)
                                     for each line extracted from the scanner
                                     data in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Output:
            None - internal belief state (self.x, self.ws) is updated in self.resample().
        """
        raise NotImplementedError("measurement_update must be overridden by a subclass of EKF")

    def resample(self, xs, ws):
        """
        Resamples the particles according to the updated particle weights.

        Inputs:
            xs: np.array[M,3] - matrix of particle states.
            ws: np.array[M,]  - particle weights.

        Output:
            None - internal belief state (self.xs, self.ws) should be updated.
        """
        r = np.random.rand() / self.M

        ########## Code starts here ##########
        # TODO: Update self.xs, self.ws.
        # Note: Assign the weights in self.ws to the corresponding weights in ws
        #       when resampling xs instead of resetting them to a uniform
        #       distribution. This allows us to keep track of the most likely
        #       particle and use it to visualize the robot's pose with self.x.
        # Hint: To maximize speed, try to implement the resampling algorithm
        #       without for loops. You may find np.linspace(), np.cumsum(), and
        #       np.searchsorted() useful. This results in a ~10x speedup.

        m = np.linspace(0, self.M - 1, num=self.M)
        u = np.sum(ws) * (r + m / self.M)

        ws_sum = np.cumsum(ws)
        idx = np.searchsorted(ws_sum, u)
        self.xs = xs[idx]
        self.ws = ws[idx]


        ########## Code ends here ##########

    def measurement_model(self, z_raw, Q_raw):
        """
        Converts raw measurements into the relevant Gaussian form (e.g., a
        dimensionality reduction).

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            z: np.array[2I,]   - joint measurement mean.
            Q: np.array[2I,2I] - joint measurement covariance.
        """
        raise NotImplementedError("measurement_model must be overridden by a subclass of EKF")


class MonteCarloLocalization(ParticleFilter):

    def __init__(self, x0, R, map_lines, tf_base_to_camera, g):
        """
        MonteCarloLocalization constructor.

        Inputs:
                       x0: np.array[M,3] - initial particle states.
                        R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
                map_lines: np.array[2,J] - J map lines in columns representing (alpha, r).
        tf_base_to_camera: np.array[3,]  - (x, y, theta) transform from the
                                           robot base to camera frame.
                        g: float         - validation gate.
        """
        self.map_lines = map_lines  # Matrix of J map lines with (alpha, r) as columns
        self.tf_base_to_camera = tf_base_to_camera  # (x, y, theta) transform
        self.g = g  # Validation gate
        super(self.__class__, self).__init__(x0, R)

    def transition_model(self, us, dt):
        """
        Unicycle model dynamics.

        Inputs:
            us: np.array[M,2] - zero-order hold control input for each particle.
            dt: float         - duration of discrete time step.
        Output:
            g: np.array[M,3] - result of belief mean for each particle
                               propagated according to the system dynamics with
                               control u for dt seconds.
        """

        ########## Code starts here ##########
        # TODO: Compute g.
        # Hint: We don't need Jacobians for particle filtering.
        # Hint: To maximize speed, try to compute the dynamics without looping
        #       over the particles. If you do this, you should implement
        #       vectorized versions of the dynamics computations directly here
        #       (instead of modifying turtlebot_model). This results in a
        #       ~10x speedup.
        x = self.xs[:, 0]  # [M, 1]
        y = self.xs[:, 1]  # [M, 1]
        th = self.xs[:, 2]  # [M, 1]
        V = us[:, 0]  # [M, 1]
        om = us[:, 1]  # [M, 1]

        th_t = th + om * dt
        # for om < Epsilon
        x_t1 = x + V * dt * np.cos(th_t)
        y_t1 = y + V * dt * np.sin(th_t)

        x_t1 = np.where(abs(om) >= EPSILON_OMEGA, 0, x_t1)
        y_t1 = np.where(abs(om) >= EPSILON_OMEGA, 0, y_t1)

        # for om > Epsilon
        x_t2 = (V / om) * (np.sin(th_t) - np.sin(th)) + x
        y_t2 = (-V / om) * (np.cos(th_t) - np.cos(th)) + y

        x_t2 = np.where(abs(om) < EPSILON_OMEGA, 0, x_t2)
        y_t2 = np.where(abs(om) < EPSILON_OMEGA, 0, y_t2)

        x_t = x_t1 + x_t2
        y_t = y_t1 + y_t2

        g = np.c_[x_t, y_t, th_t]
        ########## Code ends here ##########

        return g

    def measurement_update(self, z_raw, Q_raw):
        """
        Updates belief state according to the given measurement.

        Inputs:
            z_raw: np.array[2,I]   - matrix of I columns containing (alpha, r)
                                     for each line extracted from the scanner
                                     data in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Output:
            None - internal belief state (self.x, self.ws) is updated in self.resample().
        """
        xs = np.copy(self.xs)
        ws = np.zeros_like(self.ws)

        ########## Code starts here ##########
        # TODO: Compute new particles (xs, ws) with updated measurement weights.
        # Hint: To maximize speed, implement this without looping over the
        #       particles. You may find scipy.stats.multivariate_normal.pdf()
        #       useful.
        vs, Q = self.measurement_model(z_raw, Q_raw)
        ws = scipy.stats.multivariate_normal.pdf(vs, cov=Q)

        ########## Code ends here ##########

        self.resample(xs, ws)

    def measurement_model(self, z_raw, Q_raw):
        """
        Assemble one joint measurement and covariance from the individual values
        corresponding to each matched line feature for each particle.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            z: np.array[2I,]   - joint measurement mean.
            Q: np.array[2I,2I] - joint measurement covariance.
        """
        vs = self.compute_innovations(z_raw, np.array(Q_raw))  # [M, 2I]

        ########## Code starts here ##########
        # TODO: Compute Q.

        Q = scipy.linalg.block_diag(*Q_raw)

        ########## Code ends here ##########

        return vs, Q

    def compute_innovations(self, z_raw, Q_raw):
        """
        Given lines extracted from the scanner data, tries to associate each one
        to the closest map entry measured by Mahalanobis distance.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: np.array[I,2,2] - I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            vs: np.array[M,2I] - M innovation vectors of size 2I
                                 (predicted map measurement - scanner measurement).
        """
        def angle_diff(a, b):
            a = a % (2. * np.pi)
            b = b % (2. * np.pi)
            diff = a - b
            idx = np.abs(diff) > np.pi
            sign = 2. * (diff[idx] < 0.) - 1.
            diff[idx] += sign * 2. * np.pi
            return diff

        ########## Code starts here ##########
        # TODO: Compute vs (with shape [M x I x 2]).
        # Hint: To maximize speed, try to eliminate all for loops, or at least
        #       for loops over J. It is possible to solve multiple systems with
        #       np.linalg.solve() and swap arbitrary axes with np.transpose().
        #       Eliminating loops over J results in a ~10x speedup.
        #       Eliminating loops over I results in a ~2x speedup.
        #       Eliminating loops over M results in a ~5x speedup.
        #       Overall, that's 100x!

        hs = self.compute_predicted_measurements()  # [M, 2, J]
        I = z_raw.shape[1]
        J = hs.shape[2]  # num of predict line
        vs = np.zeros((self.M, I, 2))

        for i in range(I):
            z_i = z_raw[:, i]  # (2, 1)
            Q_i = Q_raw[i]  # (2, 2)
            for m in range(self.M):
                d_min = float("inf")
                for j in range(J):
                    v = z_i - hs[m, :, j]  # [2, 1]
                    # d_ij.append((np.matmul(np.matmul(v.T, np.linalg.inv(Q_i)), v)))  # [1, 1]
                    d_ij = np.matmul(np.matmul(v.T, np.linalg.inv(Q_i)), v)  # [1, 1]
                    if d_ij < d_min:
                        v_min = v
                        d_min = d_ij

                vs[m, i, :] = v_min

        ########## Code ends here ##########

        # Reshape [M x I x 2] array to [M x 2I]
        return vs.reshape((self.M,-1))  # [M x 2I]

    def compute_predicted_measurements(self):
        """
        Given a single map line in the world frame, outputs the line parameters
        in the scanner frame so it can be associated with the lines extracted
        from the scanner measurements.

        Input:
            None
        Output:
            hs: np.array[M,2,J] - J line parameters in the scanner (camera) frame for M particles.
        """
        ########## Code starts here ##########
        # TODO: Compute hs.
        # Hint: We don't need Jacobians for particle filtering.
        # Hint: To maximize speed, try to compute the predicted measurements
        #       without looping over the map lines. You can implement vectorized
        #       versions of turtlebod_model functions directly here. This
        #       results in a ~10x speedup.

        alpha = self.map_lines[0, :]  # [1, J]
        r = self.map_lines[1, :]  # [1, J]

        x_cam, y_cam, th_cam = self.tf_base_to_camera  # float
        # compute pose of camera in world frame

        hs = np.zeros((self.M, 2, self.map_lines.shape[1]))
        for i in range(self.M):
            x = self.xs[i, 0]  # [M, 1]
            y = self.xs[i, 1]  # [M, 1]
            th = self.xs[i, 2]  # [M, 1]
            x_cam_w = x_cam * np.cos(th) - y_cam * np.sin(th) + x  # [1, 1]
            y_cam_w = x_cam * np.sin(th) + y_cam * np.cos(th) + y

            alpha_pre = alpha - th - th_cam
            r_pre = r - x_cam_w * np.cos(alpha) - y_cam_w * np.sin(alpha)

            # normalize
            idx = r_pre < 0
            r_pre[idx] *= -1
            alpha_pre[idx] += np.pi
            alpha_pre = (alpha_pre + np.pi) % (2 * np.pi) - np.pi

            hs[i] = np.array([alpha_pre, r_pre])
        ########## Code ends here ##########

        return hs

