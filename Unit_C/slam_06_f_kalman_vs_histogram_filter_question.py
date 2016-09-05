# Comparison of the Kalman filter and the histogram filter.
# 06_f_kalman_vs_histogram_filter
# Claus Brenner, 29 NOV 2012
from distribution import *
from math import sqrt
from matplotlib.mlab import normpdf
from pylab import plot, show, ylim

# Import the helper functions from a previous file.
# If you implemented these functions in another file, put the filename here.
from slam_06_d_histogram_filter import move, convolve, multiply

#
# Helpers.
#
class Density:
    def __init__(self, mu, sigma2):
        self.mu = float(mu)
        self.sigma2 = float(sigma2)

def histogram_plot(prediction, measurement, correction):
    """Helper to draw all curves in each filter step."""
    plot(prediction.plotlists(*arena)[0], prediction.plotlists(*arena)[1],
         color='#C0C0FF', linestyle='steps', linewidth=5)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='#C0FFC0', linestyle='steps', linewidth=5)    
    plot(correction.plotlists(*arena)[0], correction.plotlists(*arena)[1],
         color='#FFC0C0', linestyle='steps', linewidth=5)

def kalman_plot(prediction, measurement, correction):
    """Helper to draw all curves in each filter step."""
    plot([normpdf(x, prediction.mu, sqrt(prediction.sigma2))
          for x in range(*arena)], color = 'b', linewidth=2)
    plot([normpdf(x, measurement.mu, sqrt(measurement.sigma2))
          for x in range(*arena)], color = 'g', linewidth=2)
    plot([normpdf(x, correction.mu, sqrt(correction.sigma2))
          for x in range(*arena)], color = 'r', linewidth=2)

#
# Histogram filter step.
#
def histogram_filter_step(belief, control, measurement):
    """Bayes filter step implementation: histogram filter."""
    # These two lines is the entire filter!
    prediction = convolve(belief, control)
    correction = multiply(prediction, measurement)

    return (prediction, correction)

#
# Kalman filter step.
#
def kalman_filter_step(belief, control, measurement):
    """Bayes filter step implementation: Kalman filter."""

    a = 1
    c = 1

    # --->>> Put your code here.
    mu_t_1 = belief.mu
    sigma2_t_1 = belief.sigma2

    u_t = control.mu
    sigma2_R = control.sigma2

    Z = measurement.mu
    sigma2_Q = measurement.sigma2
    
    # Prediction.
    #prediction = Density(belief.mu + 10.0, belief.sigma2 + 100.0)  # Replace
    mu_t = (a * mu_t_1) + u_t
    sigma2_t = (a * a * sigma2_t_1)  + sigma2_R

    prediction = Density(mu_t,sigma2_t)

    # Correction.
    #correction = prediction  # Replace
    K = (c * sigma2_t) / (c*c*sigma2_t + sigma2_Q)
    mu = mu_t + (K * (Z - (c * mu_t)))
    sigma2 = (1 - (K * c)) * sigma2_t

    correction =  Density(mu, sigma2)

    return (prediction, correction)

#
# Main
#
if __name__ == '__main__':
    arena = (0,200)
    Dist = Distribution.gaussian  # Distribution.triangle or Distribution.gaussian.

    # Start position. Well known, so the distribution is narrow.
    position = Dist(10, 1)      # Histogram
    position_ = Density(10, 1)  # Kalman

    # Controls and measurements.
    controls = [ Dist(40, 10), Dist(70, 10) ]               # Histogram
    controls_ = [ Density(40, 10**2), Density(70, 10**2) ]  # Kalman
    measurements = [ Dist(60, 10), Dist(140, 20) ]               # Histogram
    measurements_ = [ Density(60, 10**2), Density(140, 20**2) ]  # Kalman

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Histogram
        (prediction, position) = histogram_filter_step(position, controls[i], measurements[i])
        histogram_plot(prediction, measurements[i], position)
        # Kalman
        (prediction_, position_) = kalman_filter_step(position_, controls_[i], measurements_[i])
        kalman_plot(prediction_, measurements_[i], position_)

    ylim(0.0, 0.06)
    show()
