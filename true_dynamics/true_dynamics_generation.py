import matplotlib.pyplot as plt
import numpy as np

plt.ylabel('x (m)')
plt.xlabel('z (m)')

z = np.array(np.arange(0, 2000, 0.001))
x = z ** 2

file = open('dynamics.dat', 'w')
for r in z:
    file.write('z: ' + str(r) + ' x: ' + str(r ** 2) + '\n')

file.close()

plt.plot(z, x)

plt.show()
