# This adds the derivative of g with respect to the state (x, y and heading)
# to the class.
#
# slam_07_b_state_derivative
# Claus Brenner, 11.12.2012
from lego_robot import *
from math import sin, cos, pi
from numpy import *

class ExtendedKalmanFilter:

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):
        theta = state[2]
        l, r = control
        if r != l:

            # --->>> Put your code here.
            # This is for the case r != l.
            # g has 3 components and the state has 3 components, so the
            # derivative of g with respect to all state variables is a
            # 3x3 matrix. To construct such a matrix in Python/Numpy,
            # use: m = array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
            # where 1, 2, 3 are the values of the first row of the matrix.
            # Don't forget to return this matrix.
            #m = array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])  # Replace this.

            alpha = (r - l) / w
            R = l/alpha

            g1 = (R + (w/2)) * (cos(theta + alpha) - cos(theta))
            g2 = (R + (w/2)) * (sin(theta + alpha) - sin(theta))
            m = array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])
        else:

            # --->>> Put your code here.
            # This is for the special case r == l.
            #m = array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])  # Replace this.
            m = array([[1.0, 0.0, -l * sin(theta)], [0.0, 1.0, l* cos(theta)], [0.0, 0.0, 1.0]])
        return m


if __name__ == '__main__':
    # If the partial derivative with respect to x, y and theta (the state)
    # are correct, then the numerical derivative and the analytical
    # derivative should be the same.

    # Set some variables. Try other variables as well.
    # In particular, you should check cases with l == r and l != r.
    x = 10.0
    y = 20.0
    theta = 35. / 180. * pi
    state = array([x, y, theta])
    l = 50.0
    r = 54.32
    control = array([l, r])
    w = 150.0

    # Compute derivative numerically.
    print "Numeric differentiation dx, dy, dtheta:"
    delta = 1e-7
    state_x = array([x + delta, y, theta])
    state_y = array([x, y + delta, theta])
    state_theta = array([x, y, theta + delta])
    dg_dx = (ExtendedKalmanFilter.g(state_x, control, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dy = (ExtendedKalmanFilter.g(state_y, control, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dtheta = (ExtendedKalmanFilter.g(state_theta, control, w) -\
                 ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dstate_numeric = column_stack([dg_dx, dg_dy, dg_dtheta])
    print dg_dstate_numeric

    # Use the above code to compute the derivative analytically.
    print "Analytic differentiation dx, dy, dtheta:"
    dg_dstate_analytic = ExtendedKalmanFilter.dg_dstate(state, control, w)
    print dg_dstate_analytic

    # The difference should be close to zero (depending on the setting of
    # delta, above).
    print "Difference:"
    print dg_dstate_numeric - dg_dstate_analytic
    print "Seems correct:", allclose(dg_dstate_numeric, dg_dstate_analytic)
