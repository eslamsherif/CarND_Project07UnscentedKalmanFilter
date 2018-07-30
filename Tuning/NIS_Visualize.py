import sys
import numpy as np
import matplotlib.pyplot as plt

## TODO: Some consistency checks on the input files would be nice
InFile = sys.argv[1]

file = open(InFile,"r")

Dlines = file.read()
lines = Dlines.splitlines()

Lvals = []
Rvals = []

for line in lines:
    if ( line == "Listening to port 4567" ) or (line == "Connected!!!"):
        continue
    else:
        if line[0] == "L":
            Lvals.append(float(line[11:]))
        elif line[0] == "R":
            Rvals.append(float(line[11:]))

file.close()


LCHI_SQ_95_PER_VAL = np.array([6.0 for i in range(len(Lvals))])
RCHI_SQ_95_PER_VAL = np.array([7.8 for i in range(len(Rvals))])

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(Lvals, 'g')
ax1.plot(LCHI_SQ_95_PER_VAL, 'r')
ax1.set_title('LIDAR NIS')

ax2.plot(Rvals, 'g')
ax2.plot(RCHI_SQ_95_PER_VAL, 'r')
ax2.set_title('RADAR NIS')

plt.show()