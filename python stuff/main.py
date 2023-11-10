from os import PathLike
from pathlib import Path
import numpy as np
from numpy.typing import NDArray
import matplotlib.pyplot as plt
import sys

from plotting.plotting import plot3dTrajectory, plot_kinematics

def load_data(dataPath:PathLike) -> dict[str, NDArray]:
    heads = np.genfromtxt(dataPath, str, max_rows=1, delimiter=',') # loading headers
    res = np.genfromtxt(dataPath, float, skip_header=1, delimiter=',') # loading data
    heads = [ x.strip() for x in heads ]  # stripping white space
    data = {}
    for i,name in enumerate(heads):
        data[name] = res[:,i]
    return data

def same_shape_data(ork_times:NDArray, far_times:NDArray, ork_data:NDArray, far_data:NDArray) -> tuple[NDArray, NDArray, NDArray, NDArray]:
    time_concat = np.concatenate([far_times, ork_times])
    times = np.arange(np.min(time_concat),np.max(time_concat), 0.001)
    interpd_ork = np.interp(times, ork_times, ork_data)
    interpd_far = np.interp(times, far_times, far_data)
    data_diffs = interpd_ork - interpd_far
    return times, interpd_ork, interpd_far, data_diffs

def plot_mass_data(times:NDArray, orkMasses:NDArray, farMasses:NDArray, massDiffs:NDArray):
    fig, (ax1, ax2) = plt.subplots(1,2)
    ax1.plot(times, orkMasses)
    ax1.plot(times, farMasses)
    ax2.plot(times, massDiffs)
    fig.legend(["ork data", "Farseer data", "mdiffs"])
    ax1.grid(True)
    ax2.grid(True)
    plt.show()

def plot_Cn_data(orkAoAs, orkMachs, orkCNs, farAoAs, farMachs, farCNs):
    fig, (ax1, ax2) = plt.subplots(1,2)
    ax1.plot(orkAoAs, orkCNs)
    ax1.plot(farAoAs, farCNs)
    ax1.set_title(r"$C_N$ vs $\alpha$")
    ax2.plot(orkMachs, orkCNs)
    ax2.plot(farMachs, farCNs)
    ax2.set_title(r"$C_N$ vs $M$")
    fig.legend(["ork data", "Farseer data"])
    ax1.grid(True)
    ax2.grid(True)
    plt.show()
    

print(sys.argv)
fname = sys.argv[1]
orkfname = "ork6030res.csv"
rootDir = Path(__file__).parent.parent
resPth = Path(rootDir, "results", fname)
orkResPth = Path(rootDir, "results", orkfname)
data = load_data(resPth)
orkData = load_data(orkResPth)

data["Xa"] = np.concatenate([[0],np.diff(data["Xv"])/np.diff(data["t"])])
data["Ya"] = np.concatenate([[0],np.diff(data["Yv"])/np.diff(data["t"])])
data["Za"] = np.concatenate([[0],np.diff(data["Zv"])/np.diff(data["t"])])

data["ddPsi"] = np.concatenate([[0],np.diff(data["dPsi"])/np.diff(data["t"])])
data["ddTheta"] = np.concatenate([[0],np.diff(data["dTheta"])/np.diff(data["t"])])
data["ddPhi"] = np.concatenate([[0],np.diff(data["dPhi"])/np.diff(data["t"])])

data["ptot"] = np.sqrt( data["Xp"]**2 + data["Yp"]**2 + data["Zp"]**2 )
data["vtot"] = np.sqrt( data["Xv"]**2 + data["Yv"]**2 + data["Zv"]**2 )
data["atot"] = np.sqrt( data["Xa"]**2 + data["Ya"]**2 + data["Za"]**2 )

print(orkData.keys())

'''
fig, ax1 = plt.subplots()
ax1.plot(orkData['Time (s)'], orkData["Total acceleration (m/sÂ²)"])
ax1.plot(data["t"], data["atot"])
fig.legend(["ork data", "Farseer data"])
ax1.grid(True)
plt.show()
'''

times = np.arange(0,np.max(np.concatenate([data["t"], orkData['Time (s)']])), 0.001)
orkMasses = np.interp(times, orkData['Time (s)'], orkData["Mass (g)"])
farMasses = np.interp(times, data["t"], data["Mass"]*1000)

times, orkMasses, farMasses, massDiffs = same_shape_data(orkData['Time (s)'], data["t"], orkData["Mass (g)"], data["Mass"]*1000)

plot_mass_data(times, orkMasses, farMasses, massDiffs)

orkAoAs = orkData["Angle of attack (Â°)"]
orkMachs = orkData["Mach number (â€‹)"]
orkCNs = orkData["Normal force coefficient (â€‹)"]
farAoAs = data["AoA"]
farMachs = data["M"]
farCNs = data["CN"]

plot_Cn_data(orkAoAs, orkMachs, orkCNs, farAoAs, farMachs, farCNs)

#plot3dTrajectory(data["Xp"], data["Yp"], data["Zp"])


#plt.plot(data["t"][1:], np.diff(data["t"]))
#print(min(np.diff(data["t"])))
#plt.show()

#plot_kinematics(data["t"], data["Zp"], data["Zv"], data["Za"], "z")
#plot_kinematics(data["t"], data["Yp"], data["Yv"], data["Ya"], "y")
#plot_kinematics(data["t"], data["Xp"], data["Xv"], data["Xa"], "x")

#plot_kinematics(data["t"], data["Psi"], data["dPsi"], data["ddPsi"], "psi")
#plot_kinematics(data["t"], data["Theta"], data["dTheta"], data["ddTheta"], "theta")
#plot_kinematics(data["t"], data["Phi"], data["dPhi"], data["ddPhi"], "phi")

#plot_kinematics(data["t"], data["ptot"], data["vtot"], data["atot"], "total")


plt.show()


