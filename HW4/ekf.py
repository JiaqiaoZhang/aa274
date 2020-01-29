import numpy as np
import scipy.linalg    # you may find scipy.linalg.block_diag useful
import turtlebot_model as tb
from ExtractLines import angle_difference

class Ekf(object):
    """
    Base class for EKF Localization and SLAM.

    Usage:
        ekf = EKF(x0, Sigma0, R)
        while True:
            ekf.transition_update(u, dt)
            ekf.measurement_update(z, Q)
            localized_state = ekf.x
    """

    def __init__(self, x0, Sigma0, R):
        """
        EKF constructor.

        Inputs:
                x0: np.array[n,]  - initial belief mean.
            Sigma0: np.array[n,n] - initial belief covariance.
                 R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
        """
        self.x = x0  # Gaussian belief mean
        self.Sigma = Sigma0  # Gaussian belief covariance
        self.R = R  # Control noise covariance (corresponding to dt = 1 second)

    def transition_update(self, u, dt):
        """
        Performs the transition update step by updating (self.x, self.Sigma).

        Inputs:
             u: np.array[2,] - zero-order hold control input.
            dt: float        - duration of discrete time step.
        Output:
            None - internal belief state (self.x, self.Sigma) should be updated.
        """
        g, Gx, Gu = self.transition_model(u, dt)

        ########## Code starts here ##########
        # TODO: Update self.x, self.Sigma.
        self.x = g
        self.Sigma = np.matmul(np.matmul(Gx, self.Sigma), Gx.T) + dt * np.matmul(np.matmul(Gu, self.R), Gu.T)

        ########## Code ends here ##########

    def transition_model(self, u, dt):
        """
        Propagates exact (nonlinear) state dynamics.

        Inputs:
             u: np.array[2,] - zero-order hold control input.
            dt: float        - duration of discrete time step.
        Outputs:
             g: np.array[n,]  - result of belief mean propagated according to the
                                system dynamics with control u for dt seconds.
            Gx: np.array[n,n] - Jacobian of g with respect to belief mean self.x.
            Gu: np.array[n,2] - Jacobian of g with respect to control u.
        """
        raise NotImplementedError("transition_model must be overriden by a subclass of EKF")

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
            None - internal belief state (self.x, self.Sigma) should be updated.
        """
        z, Q, H = self.measurement_model(z_raw, Q_raw)
        if z is None:
            # Don't update if measurement is invalid
            # (e.g., no line matches for line-based EKF localization)
            return

        ########## Code starts here ##########
        # TODO: Update self.x, self.Sigma.
        St = np.matmul(np.matmul(H, self.Sigma), H.T) + Q
        Kt = np.matmul(np.matmul(self.Sigma, H.T), np.linalg.inv(St))

        self.x = self.x + np.hstack(Kt.dot(z))
        self.Sigma = self.Sigma - np.matmul(np.matmul(Kt, St), Kt.T)

        ########## Code ends here ##########

    def measurement_model(self, z_raw, Q_raw):
        """
        Converts raw measurements into the relevant Gaussian form (e.g., a
        dimensionality reduction). Also returns the associated Jacobian for EKF
        linearization.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            z: np.array[2K,]   - measurement mean.
            Q: np.array[2K,2K] - measurement covariance.
            H: np.array[2K,n]  - Jacobian of z with respect to the belief mean self.x.
        """
        raise NotImplementedError("measurement_model must be overriden by a subclass of EKF")


class EkfLocalization(Ekf):
    """
    EKF Localization.
    """

    def __init__(self, x0, Sigma0, R, map_lines, tf_base_to_camera, g):
        """
        EkfLocalization constructor.

        Inputs:
                       x0: np.array[3,]  - initial belief mean.
                   Sigma0: np.array[3,3] - initial belief covariance.
                        R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
                map_lines: np.array[2,J] - J map lines in columns representing (alpha, r).
        tf_base_to_camera: np.array[3,]  - (x, y, theta) transform from the
                                           robot base to camera frame.
                        g: float         - validation gate.
        """
        self.map_lines = map_lines  # Matrix of J map lines with (alpha, r) as columns
        self.tf_base_to_camera = tf_base_to_camera  # (x, y, theta) transform
        self.g = g  # Validation gate
        super(self.__class__, self).__init__(x0, Sigma0, R)

    def transition_model(self, u, dt):
        """
        Turtlebot dynamics (unicycle model).
        """

        ########## Code starts here ##########
        # TODO: Compute g, Gx, Gu using tb.compute_dynamics().
        g, Gx, Gu = tb.compute_dynamics(self.x, u, dt)

        ########## Code ends here ##########

        return g, Gx, Gu

    def measurement_model(self, z_raw, Q_raw):
        """
        Assemble one joint measurement and covariance from the individual values
        corresponding to each matched line feature.
        """
        v_list, Q_list, H_list = self.compute_innovations(z_raw, Q_raw)
        if not v_list:
            print("Scanner sees {} lines but can't associate them with any map entries."
                  .format(z_raw.shape[1]))
            return None, None, None

        ########## Code starts here ##########
        # TODO: Compute z, Q.
        z = np.array(v_list).reshape(-1, 1)
        Q = scipy.linalg.block_diag(*Q_list)
        H = np.array(H_list).reshape(-1, H_list[0].shape[1])
 
        ########## Code ends here ##########

        return z, Q, H

    def compute_innovations(self, z_raw, Q_raw):
        """
        Given lines extracted from the scanner data, tries to associate each one
        to the closest map entry measured by Mahalanobis distance.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            v_list: [np.array[2,]]  - list of at most I innovation vectors
                                      (predicted map measurement - scanner measurement).
            Q_list: [np.array[2,2]] - list of covariance matrices of the
                                      innovation vectors (from scanner uncertainty).
            H_list: [np.array[2,3]] - list of Jacobians of the innovation
                                      vectors with respect to the belief mean self.x.
        """
        def angle_diff(a, b):
            a = a % (2. * np.pi)
            b = b % (2. * np.pi)
            diff = a - b
            if np.size(diff) == 1:
                if np.abs(diff) > np.pi:
                    diff -= 2. * np.pi
                else:
                    diff += 2. * np.pi
            else:
                idx = np.abs(diff) > np.pi
                sign = 2. * (diff[idx] < 0.) - 1.
                diff[idx] += sign * 2. * np.pi
            return diff

        hs, Hs = self.compute_predicted_measurements()

        ########## Code starts here ##########
        # TODO: Compute v_list, Q_list, H_list
        hs, Hx_list = self.compute_predicted_measurements()
        num_measure = z_raw.shape[1]
        num_lines = hs.shape[1]
        v_list, Q_list, H_list = [], [], []

        for i in range(num_measure):
            z_i = z_raw[:, i]
            Q_i = Q_raw[i]
            d_min = self.g ** 2
            v, Q, H = None, None, None
            for j in range(num_lines):
                h_j = hs[:, j]
                H_j = Hx_list[j]

                vij = z_i - h_j
                Sij = np.matmul(np.matmul(H_j, self.Sigma), H_j.T) + Q_i
                dij = np.matmul(np.matmul(vij.T, np.linalg.inv(Sij)), vij)

                if dij < d_min:
                    d_min = dij
                    v, Q, H = vij, Q_i, H_j
            if d_min < self.g ** 2:
                v_list.append(v)
                Q_list.append(Q)
                H_list.append(H)

        ########## Code ends here ##########

        return v_list, Q_list, H_list

    def compute_predicted_measurements(self):
        """
        Given a single map line in the world frame, outputs the line parameters
        in the scanner frame so it can be associated with the lines extracted
        from the scanner measurements.

        Input:
            None
        Outputs:
                 hs: np.array[2,J]  - J line parameters in the scanner (camera) frame.
            Hx_list: [np.array[2,3]] - list of Jacobians of h with respect to the belief mean self.x.
        """
        hs = np.zeros_like(self.map_lines)
        Hx_list = []
        for j in range(self.map_lines.shape[1]):
            ########## Code starts here ##########
            # TODO: Compute h, Hx using tb.transform_line_to_scanner_frame().
            h, Hx = tb.transform_line_to_scanner_frame(self.map_lines[:, j], self.x, self.tf_base_to_camera)

            ########## Code ends here ##########

            h, Hx = tb.normalize_line_parameters(h, Hx)

            hs[:,j] = h
            Hx_list.append(Hx)

        return hs, Hx_list


class EkfSlam(Ekf):
    """
    EKF SLAM.
    """

    def __init__(self, x0, Sigma0, R, tf_base_to_camera, g):
        """
        EKFSLAM constructor.

        Inputs:
                       x0: np.array[3+2J,]     - initial belief mean.
                   Sigma0: np.array[3+2J,3+2J] - initial belief covariance.
                        R: np.array[2,2]       - control noise covariance
                                                 (corresponding to dt = 1 second).
        tf_base_to_camera: np.array[3,]  - (x, y, theta) transform from the
                                           robot base to camera frame.
                        g: float         - validation gate.
        """
        self.tf_base_to_camera = tf_base_to_camera  # (x, y, theta) transform
        self.g = g  # Validation gate
        super(self.__class__, self).__init__(x0, Sigma0, R)

    def transition_model(self, u, dt):
        """
        Combined Turtlebot + map dynamics.
        Adapt this method from EkfLocalization.transition_model().
        """
        g = np.copy(self.x)
        Gx = np.eye(self.x.size)
        Gu = np.zeros((self.x.size, 2))

        ########## Code starts here ##########
        V, om = u
        x, y, th = self.x[:3]
        # TODO: Compute g, Gx, Gu.
        if abs(om) < 1e-3:
            x_t = x + V * np.cos(th) * dt
            y_t = y + V * np.sin(th) * dt
            th_t = th + om * dt
            g[:3] = [x_t, y_t, th_t]

            Gx[0, 2] = -V * np.sin(th) * dt
            Gx[1, 2] = V * np.cos(th) * dt

            Gu[0, 0] = np.cos(th) * dt
            Gu[0, 1] =  -V / 2 * (dt ** 2) * np.sin(th)
            Gu[1, 0] = np.sin(th) * dt
            Gu[1, 1] = V / 2 * (dt ** 2) * np.cos(th)
            Gu[2, 1] = dt

        else:
            x_t = x + V / om * (np.sin(th + om * dt) - np.sin(th))
            y_t = y - V / om * (np.cos(th + om * dt) - np.cos(th))
            th_t = th + om * dt
            g[:3] = [x_t, y_t, th_t]
            Gx[0, 2] = V / om * (np.cos(th + om * dt) - np.cos(th))
            Gx[1, 2] = V / om * (np.sin(th + om * dt) - np.sin(th))

            dx_dom = V / (om ** 2) * (np.sin(th) - np.sin(th + om * dt)) + V * dt / om * np.cos(th + om * dt)
            dy_dom = V / (om ** 2) * (np.cos(th + om * dt) - np.cos(th)) + V * dt / om * np.sin(th + om * dt)
            Gu[0, 0] = 1. / om * (np.sin(th + om * dt) - np.sin(th))
            Gu[0, 1] = dx_dom
            Gu[1, 0] = -1. / om * (np.cos(th + om * dt))
            Gu[1, 1] = dy_dom
            Gu[2, 1] = dt

        ########## Code ends here ##########

        return g, Gx, Gu

    def measurement_model(self, z_raw, Q_raw):
        """
        Combined Turtlebot + map measurement model.
        Adapt this method from EkfLocalization.measurement_model().
        
        The ingredients for this model should look very similar to those for
        EkfLocalization. In particular, essentially the only thing that needs to
        change is the computation of Hx in self.compute_predicted_measurements()
        and how that method is called in self.compute_innovations() (i.e.,
        instead of getting world-frame line parameters from self.map_lines, you
        must extract them from the state self.x).
        """
        v_list, Q_list, H_list = self.compute_innovations(z_raw, Q_raw)
        if not v_list:
            print("Scanner sees {} lines but can't associate them with any map entries."
                  .format(z_raw.shape[1]))
            return None, None, None

        ########## Code starts here ##########
        # TODO: Compute z, Q, H.
        # Hint: Should be identical to EkfLocalization.measurement_model().
        z = np.array(v_list).reshape(-1, 1)
        Q = scipy.linalg.block_diag(*Q_list)
        H = np.array(H_list).reshape(-1, H_list[0].shape[1])
        ########## Code ends here ##########

        return z, Q, H

    def compute_innovations(self, z_raw, Q_raw):
        """
        Adapt this method from EkfLocalization.compute_innovations().
        """
        def angle_diff(a, b):
            a = a % (2. * np.pi)
            b = b % (2. * np.pi)
            diff = a - b
            idx = np.abs(diff) > np.pi
            sign = 2. * (diff[idx] < 0.) - 1.
            diff[idx] += sign * 2. * np.pi
            return diff

        hs, Hs = self.compute_predicted_measurements()

        ########## Code starts here ##########
        # TODO: Compute v_list, Q_list, H_list.

        num_measure = z_raw.shape[1]
        num_lines = hs.shape[1]
        v_list, Q_list, H_list = [], [], []

        for i in range(num_measure):
            z_i = z_raw[:, i]
            Q_i = Q_raw[i]
            d_min = self.g ** 2
            v, Q, H = None, None, None
            for j in range(num_lines):
                h_j = hs[:, j]
                H_j = Hs[j]

                vij = z_i - h_j
                Sij = np.matmul(np.matmul(H_j, self.Sigma), H_j.T) + Q_i
                dij = np.matmul(np.matmul(vij.T, np.linalg.inv(Sij)), vij)

                if dij < d_min:
                    d_min = dij
                    v, Q, H = vij, Q_i, H_j
            if d_min < self.g ** 2:
                v_list.append(v)
                Q_list.append(Q)
                H_list.append(H)


        ########## Code ends here ##########

        return v_list, Q_list, H_list

    def compute_predicted_measurements(self):
        """
        Adapt this method from EkfLocalization.compute_predicted_measurements().
        """
        J = (self.x.size - 3) // 2
        hs = np.zeros((2, J))
        Hx_list = []
        for j in range(J):
            idx_j = 3 + 2 * j
            alpha, r = self.x[idx_j:idx_j+2]

            Hx = np.zeros((2,self.x.size))

            ########## Code starts here ##########
            # TODO: Compute h, Hx.
            x, y, th = self.x[:3]
            x_cam, y_cam, th_cam = self.tf_base_to_camera
            # compute pose of camera in world frame
            x_cam_w = x_cam * np.cos(th) - y_cam * np.sin(th) + x
            y_cam_w = x_cam * np.sin(th) + y_cam * np.cos(th) + y

            h = np.array([alpha - th - th_cam,
                          r - x_cam_w * np.cos(alpha) - y_cam_w * np.sin(alpha)])

            dh_dth = (x_cam * np.sin(th) + y_cam * np.cos(th)) * np.cos(alpha) - \
                     (x_cam * np.cos(th) - y_cam * np.sin(th)) * np.sin(alpha)
            Hx[:, :3]= np.array([[0, 0, -1],
                           [-np.cos(alpha), -np.sin(alpha), dh_dth]])

            # First two map lines are assumed fixed so we don't want to propagate
            # any measurement correction to them.
            if j >= 2:
                Hx[:,idx_j:idx_j+2] = np.eye(2)  # FIX ME!
                Hx[1, idx_j] = x_cam_w * np.sin(alpha) - y_cam_w * np.cos(alpha)
            ########## Code ends here ##########

            h, Hx = tb.normalize_line_parameters(h, Hx)
            hs[:,j] = h
            Hx_list.append(Hx)

        return hs, Hx_list
