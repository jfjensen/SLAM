import random

def generate_new_particles(old_particles, weights):
    N = len(old_particles)
    new_particles = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(weights)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        print "beta =", beta
        while beta > weights[index]:
            beta -= weights[index]
            index = (index + 1) % N
            print "\tbeta= %f, index = %d, weight = %f" % (beta, index, weights[index])
        new_particles.append(old_particles[index])
    return new_particles

if __name__ == "__main__":
    old_particles = [1, 2, 3, 4]
    weights = [.3, 0, .4, .3]
    new_particles = generate_new_particles(old_particles, weights)

    print "old particles =", old_particles
    print "weights =", weights
    print "new particles =", new_particles