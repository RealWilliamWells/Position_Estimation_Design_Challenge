import math
from math import sqrt

import matplotlib.pyplot as plt
import numpy as np

# Global constants
A = 3  # Assume cross section area is 10m^2
C = 0.35
P = 1.225  # 1.225g/L air density at 15 degrees Celsius
M = 120.00  # 100kg + 20kg = 120kg

def xDisplacementFunction(tMax):
    # Constants
    Vwa = -5.00  # Wind velocity 1000m and below
    Vwb = 5.00  # Wind velocity above 1000m

    Vw = Vwa

    d = []

    for t in np.arange(0.1, tMax, 0.001):
        if 2 * A * C * P * Vw * t + M >= 0:
            d.append((1 / (A * C * P)) * (-2 * sqrt(M * (2 * A * C * P * Vw * t + M)) + 2 * M
                                     * np.arctanh((sqrt(M * (2 * A * C * P * Vw * t + M))) / M) + A * C * P * Vw * t + M * np.log(t)))
        else:
            d.append(Vw*0.001 + d[len(d)-1])

    return d

def zDisplacementFunction(tMax):
    # Constants
    F = 1500.00
    G = 9.81

    d = []

    for t in np.arange(0.1, tMax, 0.001):
        d.append((1 / (A * C * P)) * (sqrt(2 * A * C * P * (t ** 2) * (F - G * M) + (M ** 2)) - M
                                     * np.arctanh(((sqrt(2 * A * C * P * (t ** 2) * (F - G * M) + (M ** 2))) / M) + 0j) - M * np.log(t)))

    return d

if __name__ == "__main__":
    plt.ylabel('x (m)')
    plt.xlabel('z (m)')

    x = xDisplacementFunction(38.561)
    z = zDisplacementFunction(38.561)

    # file = open('true_dynamics.dat', 'w')
    # for i in z:
    #     file.write('z: ' + str(i) + ' x: ' + str(xDisplacementFunction(i)) + '\n')
    #
    # file.close()

    plt.plot(z, x)

    plt.show()
