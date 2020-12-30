import math
from math import sqrt

import matplotlib.pyplot as plt
import numpy as np

# Global constants
A = 3  # Assume cross section area is 10m^2
C = 0.35
P = 1.225  # 1.225g/L air density at 15 degrees Celsius
M = 120.00  # 100kg + 20kg = 120kg

def xDisplacementFunction(z):
    # Constants
    Vwa = -5.00  # Wind velocity 1000m and below
    Vwb = 5.00  # Wind velocity above 1000m

    # Initial conditions
    Vw = Vwa
    startingD = 0
    newConditions = False

    d = []

    for k in z:
        if (k>1000):
            k = k - 1000

            if not newConditions:
                # Conditions above 1000m
                Vw = Vwb
                startingD = d[len(d)-1]
                newConditions = True

        currentD = Vw*math.exp((-C*P*A*k)/(2*M))*((2*M)/(C*P*A)) + Vw*k - Vw*((2*M) / (C*P*A)) + startingD
        d.append(currentD)

    return d

if __name__ == "__main__":
    plt.ylabel('x (m)')
    plt.xlabel('z (m)')

    z = np.arange(0, 2000, 0.001)
    x = xDisplacementFunction(z)

    # file = open('true_dynamics.dat', 'w')
    # for i in z:
    #     file.write('z: ' + str(i) + ' x: ' + str(xDisplacementFunction(i)) + '\n')
    #
    # file.close()

    plt.plot(z, x)

    plt.show()
