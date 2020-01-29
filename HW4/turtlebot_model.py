import numpy as np

EPSILON_OMEGA = 1e-3

def compute_dynamics(x, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                        x: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to x.
        Gu: np.array[3,2] - Jacobian of g with respect ot u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    V, om = u
    x, y, th = x
    if abs(om) < EPSILON_OMEGA:
        x_t = x + V * np.cos(th) * dt
        y_t = y + V * np.sin(th) * dt
        th_t = th + om * dt
        g = np.array([x_t, y_t, th_t])

        Gx = np.array([[1, 0, -V * np.sin(th) * dt],
                       [0, 1, V * np.cos(th) * dt],
                       [0, 0, 1]])

        Gu = np.array([[np.cos(th) * dt, -V / 2 * (dt ** 2) * np.sin(th)],
                       [np.sin(th) * dt, V / 2 * (dt ** 2) * np.cos(th)],
                       [0, dt]])
    else:
        x_t = x + V / om * (np.sin(th + om * dt) - np.sin(th))
        y_t = y - V / om * (np.cos(th + om * dt) - np.cos(th))
        th_t = th + om * dt
        g = np.array([x_t, y_t, th_t])

        Gx = np.array([[1., 0., V / om * (np.cos(th + om * dt) - np.cos(th))],
                       [0., 1., V / om * (np.sin(th + om * dt) - np.sin(th))],
                       [0., 0., 1.]])

        dx_dom = V / (om ** 2) * (np.sin(th) - np.sin(th + om * dt)) + V * dt / om * np.cos(th + om * dt)
        dy_dom = V / (om ** 2) * (np.cos(th + om * dt) - np.cos(th)) + V * dt / om * np.sin(th + om * dt)
        Gu = np.array([[1. / om * (np.sin(th + om * dt) - np.sin(th)), dx_dom],
                       [-1. / om * (np.cos(th + om * dt) - np.cos(th)), dy_dom],
                       [0., dt]])
    ########## Code ends here ##########

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
             hs: [np.array[2,J]] - J line parameters in the scanner (camera) frame.
        Hx_list: [np.array[2,3]] - list of Jacobians of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    x, y ,th = x
    x_cam, y_cam, th_cam = tf_base_to_camera
    # compute pose of camera in world frame
    x_cam_w = x_cam * np.cos(th) - y_cam * np.sin(th) + x
    y_cam_w = x_cam * np.sin(th) + y_cam * np.cos(th) + y

    h = np.array([alpha - th - th_cam,
                  r - x_cam_w * np.cos(alpha) - y_cam_w * np.sin(alpha)])

    dh_dth = (x_cam * np.sin(th) + y_cam * np.cos(th)) * np.cos(alpha) - \
             (x_cam * np.cos(th) - y_cam * np.sin(th)) * np.sin(alpha)
    Hx = np.array([[0, 0, -1],
                   [-np.cos(alpha), -np.sin(alpha), dh_dth]])

    ########## Code ends here ##########

    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h
