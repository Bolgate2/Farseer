from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from plotting.plotting import plot3dTrajectory, plot_kinematics

fname = "26"
rootDir = Path(__file__).parent.parent
resPth = Path(rootDir, "results", fname)
print(resPth)
heads = np.genfromtxt(resPth, str, max_rows=1, delimiter=',') # loading headers
res = np.genfromtxt(resPth, float, skip_header=1, delimiter=',') # loading data
heads = [ x.strip() for x in heads ]  # stripping white space
print(res.shape)
print(heads)
print(len(heads))

# packing data into dictionary
data = {}
for i,name in enumerate(heads):
    data[name] = res[:,i]

data["Xa"] = np.concatenate([[0],np.diff(data["Xv"])])
data["Ya"] = np.concatenate([[0],np.diff(data["Yv"])])
data["Za"] = np.concatenate([[0],np.diff(data["Zv"])])

plot3dTrajectory(data["Xp"], data["Yp"], data["Zp"])

plot_kinematics(data["t"], data["Zp"], data["Zv"], data["Za"], "z")
plt.show()


