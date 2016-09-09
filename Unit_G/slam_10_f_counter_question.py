# FastSLAM.
# The fully functional SLAM is extended by a mechanism to exclude
# spurious observations of landmarks.
#
# Search for the term 'Added' to find all locations in the program where
# a modification is made to support the 'observation count' and removal
# of spurious landmarks.
#
# slam_10_f_counter
# Claus Brenner, 20.02.2013
from lego_robot import *
from slam_g_library import get_cylinders_from_scan, write_cylinders,\
     write_error_ellipses, get_mean, get_error_ellipse_and_heading_variance,\
     print_particles
from math import sin, cos, pi, atan2, sqrt, exp
import copy
import random
import numpy as np


class Particle:
    def __init__(self, pose):
        self.pose = pose
        self.landmark_positions = []
        self.landmark_covariances = []
        self.landmark_counters = []  # Added: counter for each landmark.

    def number_of_landmarks(self):
        """Utility: return current number of landmarks in this particle."""
        return len(self.landmark_positions)

    @staticmethod
    def g(state, control, w):
        """State transition. This is exactly the same method as in the Kalman
           filter."""
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
        return np.array([g1, g2, g3])

    def move(self, left, right, w):
        """Given left, right control and robot width, move the robot."""
        self.pose = self.g(self.pose, (left, right), w)

    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Measurement function. Takes a (x, y, theta) state and a (x, y)
           landmark, and returns the corresponding (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return np.array([r, alpha])

    @staticmethod
    def dh_dlandmark(state, landmark, scanner_displacement):
        """Derivative with respect to the landmark coordinates. This is related
           to the dh_dstate function we used earlier (it is:
           -dh_dstate[0:2,0:2])."""
        theta = state[2]
        cost, sint = cos(theta), sin(theta)
        dx = landmark[0] - (state[0] + scanner_displacement * cost)
        dy = landmark[1] - (state[1] + scanner_displacement * sint)
        q = dx * dx + dy * dy
        sqrtq = sqrt(q)
        dr_dmx = dx / sqrtq
        dr_dmy = dy / sqrtq
        dalpha_dmx = -dy / q
        dalpha_dmy =  dx / q

        return np.array([[dr_dmx, dr_dmy],
                         [dalpha_dmx, dalpha_dmy]])

    def h_expected_measurement_for_landmark(self, landmark_number,
                                            scanner_displacement):
        """Returns the expected distance and bearing measurement for a given
           landmark number and the pose of this particle."""
        return self.h(self.pose,self.landmark_positions[landmark_number], scanner_displacement)

    def H_Ql_jacobian_and_measurement_covariance_for_landmark(
        self, landmark_number, Qt_measurement_covariance, scanner_displacement):
        """Computes Jacobian H of measurement function at the particle's
           position and the landmark given by landmark_number. Also computes the
           measurement covariance matrix."""
        H = self.dh_dlandmark(self.pose,self.landmark_positions[landmark_number], scanner_displacement)
        Sigma_k = self.landmark_covariances[landmark_number]
        Ql = np.dot(np.dot(H,Sigma_k),H.T) + Qt_measurement_covariance
        
        return (H, Ql)

    def wl_likelihood_of_correspondence(self, measurement,
                                        landmark_number,
                                        Qt_measurement_covariance,
                                        scanner_displacement):
        """For a given measurement and landmark_number, returns the likelihood
           that the measurement corresponds to the landmark."""
        # - You will need delta_z, which is the measurement minus the
        #   expected_measurement_for_landmark()
        delta_z = measurement - self.h_expected_measurement_for_landmark( landmark_number, scanner_displacement )

        # - Ql is obtained using a call to
        #   H_Ql_jacobian_and_measurement_covariance_for_landmark(). You
        #   will only need Ql, not H
        H,Ql = self.H_Ql_jacobian_and_measurement_covariance_for_landmark( landmark_number,Qt_measurement_covariance,scanner_displacement )

        # - np.linalg.det(A) computes the determinant of A
        # - np.dot() does not distinguish between row and column vectors.
        detQl = np.linalg.det(Ql)
        e_factor = -0.5 * np.dot(np.dot(delta_z.T, np.linalg.inv(Ql)), delta_z)
        l = exp(e_factor)/(2*pi*sqrt(detQl))

        return l

    def compute_correspondence_likelihoods(self, measurement,
                                           number_of_landmarks,
                                           Qt_measurement_covariance,
                                           scanner_displacement):
        """For a given measurement, returns a list of all correspondence
           likelihoods (from index 0 to number_of_landmarks-1)."""
        likelihoods = []
        for i in xrange(number_of_landmarks):
            likelihoods.append(
                self.wl_likelihood_of_correspondence(
                    measurement, i, Qt_measurement_covariance,
                    scanner_displacement))
        return likelihoods

    def initialize_new_landmark(self, measurement_in_scanner_system,
                                Qt_measurement_covariance,
                                scanner_displacement):
        """Given a (x, y) measurement in the scanner's system, initializes a
           new landmark and its covariance."""
        scanner_pose = (self.pose[0] + cos(self.pose[2]) * scanner_displacement,
                        self.pose[1] + sin(self.pose[2]) * scanner_displacement,
                        self.pose[2])
        # - LegoLogfile.scanner_to_world() (from lego_robot.py) will return
        #   the world coordinate, given the scanner pose and the coordinate in
        #   the scanner's system.
        m = LegoLogfile.scanner_to_world(scanner_pose,measurement_in_scanner_system)
        # - H is obtained from dh_dlandmark()
        H = self.dh_dlandmark(scanner_pose, m, scanner_displacement)
        # - Use np.linalg.inv(A) to invert matrix A
        H_inv = np.linalg.inv(H)
        Qt = Qt_measurement_covariance
        # - As usual, np.dot(A,B) is the matrix product of A and B.
        # self.landmark_positions.append(np.array([0.0, 0.0]))  # Replace this.
        # self.landmark_covariances.append(np.eye(2))  # Replace this.
        self.landmark_positions.append(m)
        self.landmark_covariances.append(np.dot(np.dot(H_inv,Qt),H_inv.T))

    def update_landmark(self, landmark_number, measurement,
                        Qt_measurement_covariance, scanner_displacement):
        """Update a landmark's estimated position and covariance."""
        # - H and Ql can be computed using
        #   H_Ql_jacobian_and_measurement_covariance_for_landmark()
        H, Ql = self.H_Ql_jacobian_and_measurement_covariance_for_landmark( landmark_number,Qt_measurement_covariance,scanner_displacement )
        
        # - Use np.linalg.inv(A) to compute the inverse of A
        Ql_inv = np.linalg.inv(Ql)
        
        # - Delta z is measurement minus expected measurement
        # - Expected measurement can be computed using
        #   h_expected_measurement_for_landmark()
        h = self.h_expected_measurement_for_landmark( landmark_number, scanner_displacement )
        delta_z = measurement - h

        # - Remember to update landmark_positions[landmark_number] as well
        #   as landmark_covariances[landmark_number].
        Sigma_old = self.landmark_covariances[landmark_number]
        K = np.dot(np.dot(Sigma_old,H.T),Ql_inv)
        KH = np.dot(K,H)
        
        self.landmark_positions[landmark_number] += np.dot(K, delta_z)
        self.landmark_covariances[landmark_number] = np.dot((np.eye(KH.shape[0]) - KH), Sigma_old)

    def update_particle(self, measurement, measurement_in_scanner_system,
                        number_of_landmarks,
                        minimum_correspondence_likelihood,
                        Qt_measurement_covariance, scanner_displacement):
        """Given a measurement, computes the likelihood that it belongs to any
           of the landmarks in the particle. If there are none, or if all
           likelihoods are below the minimum_correspondence_likelihood
           threshold, add a landmark to the particle. Otherwise, update the
           (existing) landmark with the largest likelihood."""
        # --->>> Insert your previous code here.
        # --->>> Then, modify as follows:
        # If a new landmark is initialized, append 1 to landmark_counters.
        # If an existing landmark is updated, add 2 to the corresponding
        #  landmark counter.

        # Compute likelihood of correspondence of measurement to all landmarks
        # (from 0 to number_of_landmarks-1).
        likelihoods = self.compute_correspondence_likelihoods(measurement,number_of_landmarks,\
            Qt_measurement_covariance,scanner_displacement)

        # If the likelihood list is empty, or the max correspondence likelihood
        # is still smaller than minimum_correspondence_likelihood, setup
        # a new landmark.
        if not likelihoods or\
           max(likelihoods) < minimum_correspondence_likelihood:
            self.initialize_new_landmark(measurement_in_scanner_system,Qt_measurement_covariance,scanner_displacement)
            self.landmark_counters.append(1)
            #self.initialize_new_landmark(measurement,Qt_measurement_covariance,scanner_displacement)
            return minimum_correspondence_likelihood

        # Else update the particle's EKF for the corresponding particle.
        else:
            # This computes (max, argmax) of measurement_likelihoods.

            # --->>> Add code to find w, the maximum likelihood,
            # and the corresponding landmark index.
            w = max(likelihoods)
            index =  np.argmax(likelihoods) # Replace this by a search for max, argmax.
            self.update_landmark(index,measurement,Qt_measurement_covariance, scanner_displacement)
            self.landmark_counters[index] += 2
            return w

    # Added: Counter decrement for visible landmarks.
    def decrement_visible_landmark_counters(self, scanner_displacement):
        """Decrements the counter for every landmark which is potentially
           visible. This uses a simplified test: it is only checked if the
           bearing of the expected measurement is within the laser scanners
           range."""
        # --->>> Insert your new code here.
        # Hints:
        # - We only check the bearing angle of the landmarks.
        # - Min and max bearing can be obtained from
        #   LegoLogfile.min_max_bearing()
        min_b,max_b = LegoLogfile.min_max_bearing()
        # - The bearing for any landmark can be computed using
        #   h_expected_measurement_for_landmark()
        for landmark_ix in range(len(self.landmark_positions)):
            distance, bearing = self.h_expected_measurement_for_landmark(landmark_ix, scanner_displacement)
            if bearing >= min_b and bearing <= max_b:
                self.landmark_counters[landmark_ix] -= 1

        # - If the bearing is within the range, decrement the corresponding
        #   self.landmark_counters[].
        #pass  # Replace this.

    # Added: Removal of landmarks with negative counter.
    def remove_spurious_landmarks(self):
        """Remove all landmarks which have a counter less than zero."""
        # --->>> Insert your new code here.
        # Remove any landmark for which the landmark_counters[] is (strictly)
        # smaller than zero.
        # Note: deleting elements of a list while iterating over the list
        # will not work properly. One relatively simple and elegant solution is
        # to make a new list which contains all required elements (c.f. list
        # comprehensions with if clause).
        # Remember to process landmark_positions, landmark_covariances and
        # landmark_counters.
        new_landmark_positions = []
        new_landmark_covariances = []
        new_landmark_counters = []

        for ix in range(len(self.landmark_counters)):
            if self.landmark_counters[ix] >= 0:
                new_landmark_positions.append(self.landmark_positions[ix])
                new_landmark_covariances.append(self.landmark_covariances[ix])
                new_landmark_counters.append(self.landmark_counters[ix])

        self.landmark_positions = new_landmark_positions
        self.landmark_covariances = new_landmark_covariances
        self.landmark_counters = new_landmark_counters


class FastSLAM:
    def __init__(self, initial_particles,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev,
                 minimum_correspondence_likelihood):
        # The particles.
        self.particles = initial_particles

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev
        self.minimum_correspondence_likelihood = \
            minimum_correspondence_likelihood

    def predict(self, control):
        """The prediction step of the particle filter."""
        left, right = control
        left_std  = sqrt((self.control_motion_factor * left)**2 +\
                        (self.control_turn_factor * (left-right))**2)
        right_std = sqrt((self.control_motion_factor * right)**2 +\
                         (self.control_turn_factor * (left-right))**2)
        # Modify list of particles: for each particle, predict its new position.
        for p in self.particles:
            l = random.gauss(left, left_std)
            r = random.gauss(right, right_std)
            p.move(l, r, self.robot_width)

    def update_and_compute_weights(self, cylinders):
        """Updates all particles and returns a list of their weights."""
        Qt_measurement_covariance = \
            np.diag([self.measurement_distance_stddev**2,
                     self.measurement_angle_stddev**2])
        weights = []
        for p in self.particles:
            # Added: decrement landmark counter for any landmark that should be
            # visible.
            p.decrement_visible_landmark_counters(self.scanner_displacement)

            # Loop over all measurements.
            number_of_landmarks = p.number_of_landmarks()
            weight = 1.0
            for measurement, measurement_in_scanner_system in cylinders:
                weight *= p.update_particle(
                    measurement, measurement_in_scanner_system,
                    number_of_landmarks,
                    self.minimum_correspondence_likelihood,
                    Qt_measurement_covariance, scanner_displacement)

            # Append overall weight of this particle to weight list.
            weights.append(weight)
            
            # Added: remove spurious landmarks (with negative counter).
            p.remove_spurious_landmarks()

        return weights

    def resample(self, weights):
        """Return a list of particles which have been resampled, proportional
           to the given weights."""
        new_particles = []
        max_weight = max(weights)
        index = random.randint(0, len(self.particles) - 1)
        offset = 0.0
        for i in xrange(len(self.particles)):
            offset += random.uniform(0, 2.0 * max_weight)
            while offset > weights[index]:
                offset -= weights[index]
                index = (index + 1) % len(weights)
            new_particles.append(copy.deepcopy(self.particles[index]))
        return new_particles

    def correct(self, cylinders):
        """The correction step of FastSLAM."""
        # Update all particles and compute their weights.
        weights = self.update_and_compute_weights(cylinders)
        # Then resample, based on the weight array.
        self.particles = self.resample(weights)


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.
    minimum_correspondence_likelihood = 0.001  # Min likelihood of correspondence.

    # Generate initial particles. Each particle is (x, y, theta).
    number_of_particles = 200 #25
    start_state = np.array([500.0, 0.0, 45.0 / 180.0 * pi])
    initial_particles = [copy.copy(Particle(start_state))
                         for _ in xrange(number_of_particles)]

    # Setup filter.
    fs = FastSLAM(initial_particles,
                  robot_width, scanner_displacement,
                  control_motion_factor, control_turn_factor,
                  measurement_distance_stddev,
                  measurement_angle_stddev,
                  minimum_correspondence_likelihood)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Loop over all motor tick records.
    # This is the FastSLAM filter loop, with prediction and correction.
    f = open("fast_slam_counter.txt", "w")
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i])
        fs.predict(control)

        # Correction.
        cylinders = get_cylinders_from_scan(logfile.scan_data[i], depth_jump,
            minimum_valid_distance, cylinder_offset)
        fs.correct(cylinders)

        # Output particles.
        print_particles(fs.particles, f)

        # Output state estimated from all particles.
        mean = get_mean(fs.particles)
        print >> f, "F %.0f %.0f %.3f" %\
              (mean[0] + scanner_displacement * cos(mean[2]),
               mean[1] + scanner_displacement * sin(mean[2]),
               mean[2])

        # Output error ellipse and standard deviation of heading.
        errors = get_error_ellipse_and_heading_variance(fs.particles, mean)
        print >> f, "E %.3f %.0f %.0f %.3f" % errors

        # Output landmarks of particle which is closest to the mean position.
        output_particle = min([
            (np.linalg.norm(mean[0:2] - fs.particles[i].pose[0:2]),i)
            for i in xrange(len(fs.particles)) ])[1]
        # Write estimates of landmarks.
        write_cylinders(f, "W C",
                        fs.particles[output_particle].landmark_positions)
        # Write covariance matrices.
        write_error_ellipses(f, "W E",
                             fs.particles[output_particle].landmark_covariances)

    f.close()
