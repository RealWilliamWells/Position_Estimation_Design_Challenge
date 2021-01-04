import math
from math import sqrt

import matplotlib.pyplot as plt
import numpy as np

# Global constants
A = 3.00  # Assume cross section area is 10m^2
C = 0.35
P = 1.225  # 1.225g/L air density at 15 degrees Celsius
M = 120.00  # 100kg + 20kg = 120kg

def xDisplacementFunction(z):
    # Constants
    Vwa = -5.00  # Wind velocity 1000m and below
    Vwb = 5.00  # Wind velocity above 1000m

    # Initial conditions
    Vw = Vwa
    startingV = 0
    startingD = 0
    newConditions = False

    d = []

    for k in z:
        if (k>1000):
            if not newConditions:
                # Conditions above 1000m
                Vw = Vwb
                startingV = xVelocityFunction(k)
                startingD = d[len(d)-1]
                newConditions = True

            k = k - 1000

        currentD = Vw*math.exp((-C*P*A*k)/(2*M))*((2*M)/(C*P*A)) + Vw*k - Vw*((2*M) / (C*P*A)) + startingV * k + startingD
        d.append(currentD)

    return d

def xVelocityFunction(z):
    # Constants
    Vw = -5.00  # Wind velocity 1000m and below

    Vx = -Vw*math.exp((-C*P*A*z)/(2*M)) + Vw

    return Vx

if __name__ == "__main__":
    plt.ylabel('x (m)')
    plt.xlabel('z (m)')

    z = np.arange(0, 2000, 0.1)
    x = xDisplacementFunction(z)

    plt.plot(z, x)

    plt.savefig('true_dynamics_graph.png')

    plt.show()
