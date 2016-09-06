# The complete Kalman prediction step (without the correction step).
#
# slam_07_d_kalman_predict_solution
# Claus Brenner, 12.12.2012
from lego_robot import *
from math import sin, cos, pi, atan2
from numpy import *


class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width,
                 control_motion_factor, control_turn_factor):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor

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


    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)
        if r != l:

            # --->>> Put your code here.
            # This is for the case l != r.
            # Note g has 3 components and control has 2, so the result
            # will be a 3x2 (rows x columns) matrix.

            alpha = (r - l) / w

            wr = (w*r)/((r-l)**2)
            wl = (w*l)/((r-l)**2)
            r2l = (r+l)/(2*(r-l))

            g1_l = wr * (sin(theta+alpha)-sin(theta)) - r2l * cos(theta+alpha)
            g2_l = wr * (-cos(theta+alpha)+cos(theta)) - r2l * sin(theta+alpha)
            g3_l = - (1/w)

            g1_r = -wl * (sin(theta+alpha)-sin(theta)) + r2l * cos(theta+alpha)
            g2_r = -wl * (-cos(theta+alpha)+cos(theta)) + r2l * sin(theta+alpha)
            g3_r = 1 / w 

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])
            
            
        else:

            # --->>> Put your code here.
            # This is for the special case l == r.
            g1_l = .5 * (cos(theta) + (l/w)*sin(theta))
            g2_l = .5 * (sin(theta) - (l/w)*cos(theta))
            g3_l = - 1/w

            g1_r = .5 * ((-l/w)*sin(theta) + cos(theta))
            g2_r = .5 * ((l/w)*cos(theta) + sin(theta))
            g3_r = 1 / w 

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])            

        #m = array([[1, 2], [3, 4], [5, 6]])  # Remove this.
            
        return m

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):
        """The prediction step of the Kalman filter."""
        # covariance' = G * covariance * GT + R
        # where R = V * (covariance in control space) * VT.
        # Covariance in control space depends on move distance.
        left, right = control

        alpha_1 = self.control_motion_factor
        alpha_2 = self.control_turn_factor

        g2l = (alpha_1 * left)**2 + (alpha_2 * (left-right))**2
        g2r = (alpha_1 * right)**2 + (alpha_2 * (left-right))**2

        Sigma_control = diag([g2l,g2r])
        Vt = self.dg_dcontrol( self.state,control,self.robot_width )#!!
        VtT = Vt.T

        Sigma_covariance = self.covariance
        Gt = self.dg_dstate( self.state,control,self.robot_width )#!!
        GtT = Gt.T

        self.covariance = dot(dot(Gt,Sigma_covariance),GtT) + dot(dot(Vt,Sigma_control),VtT)

        # --->>> Put your code to compute the new self.covariance here.
        # First, construct the control_covariance, which is a diagonal matrix.
        # In Python/Numpy, you may use diag([a, b]) to get
        # [[ a, 0 ],
        #  [ 0, b ]].
        # Then, compute G using dg_dstate and V using dg_dcontrol.
        # Then, compute the new self.covariance.
        # Note that the transpose of a Numpy array G is expressed as G.T,
        # and the matrix product of A and B is written as dot(A, B).
        # Writing A*B instead will give you the element-wise product, which
        # is not intended here.

        # state' = g(state, control)

        # --->>> Put your code to compute the new self.state here.
        self.state = self.g(self.state,control,self.robot_width)



if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.

    # Measured start position.
    initial_state = array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
                              robot_width,
                              control_motion_factor, control_turn_factor)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records and generate filtered positions and
    # covariances.
    # This is the Kalman filter loop, without the correction step.
    states = []
    covariances = []
    for m in logfile.motor_ticks:
        # Prediction.
        control = array(m) * ticks_to_mm
        kf.predict(control)

        # Log state and covariance.
        states.append(kf.state)
        covariances.append(kf.covariance)

    # Write all states, all state covariances, and matched cylinders to file.
    f = open("kalman_prediction.txt", "w")
    for i in xrange(len(states)):
        # Output the center of the scanner, not the center of the robot.
        print >> f, "F %f %f %f" % \
            tuple(states[i] + [scanner_displacement * cos(states[i][2]),
                               scanner_displacement * sin(states[i][2]),
                               0.0])
        # Convert covariance matrix to angle stddev1 stddev2 stddev-heading form
        e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
        print >> f, "E %f %f %f %f" % (e + (sqrt(covariances[i][2,2]),))

    f.close()
