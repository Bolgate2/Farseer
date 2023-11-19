from os import PathLike
from pathlib import Path
import numpy as np
from numpy.typing import NDArray
import matplotlib
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

def plot_time_data_with_diffs(ork_times:NDArray, far_times:NDArray, ork_data:NDArray, far_data:NDArray, yname:str, xname:str="Time (s)"):
    times, orkDataDense, farDataDense, dataDiffs = same_shape_data(ork_times, far_times, ork_data, far_data)
    dataPercent = dataDiffs/orkDataDense*100
    dataPercent[np.isnan(dataPercent)] = 0.0
    fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
    ax1.plot(times, orkDataDense)
    ax1.plot(times, farDataDense)
    ax2.plot(times, dataDiffs)
    ax3.plot(times, dataPercent)
    fig.legend(["ork data", "Farseer data"])
    
    ax1.set_ylabel(fr"{yname}")
    ax2.set_ylabel(fr"$\Delta${yname}")
    ax3.set_ylabel(fr"% Err")
    ax3.set_xlabel(xname)
    ax1.grid(True)
    ax2.grid(True)
    ax3.grid(True)
    fig.tight_layout()
    plt.show()
    

def plot_Cn_data(orkAoAs:NDArray, orkMachs:NDArray, orkCNs:NDArray, farAoAs:NDArray, farMachs:NDArray, farCNs:NDArray):
    fig, (ax1, ax2) = plt.subplots(1,2)
    ax1.plot(orkAoAs, orkCNs, )
    ax1.plot(farAoAs, farCNs, )
    ax1.set_ylabel(r"$C_N$")
    ax1.set_xlabel(r"$\alpha$ ($^\circ$)")
    
    ax2.plot(orkMachs, orkCNs/np.deg2rad(orkAoAs), )
    ax2.plot(farMachs, farCNs/np.deg2rad(farAoAs), )
    ax2.set_ylabel(r"$C_{N_\alpha}$ (rad$^{-1}$)")
    ax2.set_xlabel(r"$M$")
    
    fig.legend(["ork data", "Farseer data"])
    ax1.grid(True)
    ax2.grid(True)
    
    
    fig.tight_layout()
    plt.show()
    

print(sys.argv)
fname = sys.argv[1]
orkfname = sys.argv[2]
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

orkTimes = orkData['Time (s)']
farTimes = data["t"]

orkAlts = orkData["Altitude (m)"]
farAlts = data["Altitude"]

orkMasses = orkData["Mass (g)"]/1000
farMasses = data["Mass"]
orkThrusts = orkData["Thrust (N)"]
farThrusts = data["Thrust"]
orkGs = orkData["Gravitational acceleration (m/sÂ²)"]
farGs = data["g"]

orkLongInert = orkData["Longitudinal moment of inertia (kgÂ·mÂ²)"]
orkRotInert = orkData["Rotational moment of inertia (kgÂ·mÂ²)"]
farIxx = data["Ixx"]
farIyy = data["Iyy"]
farIzz = data["Izz"]

#plot_time_data_with_diffs(orkTimes, farTimes, orkLongInert, farIxx, "Long Inert")
#plot_time_data_with_diffs(orkTimes, farTimes, orkRotInert, farIzz, "Rot Inert")
#plot_time_data_with_diffs(orkTimes, farTimes, orkMasses, farMasses, "Mass (kg)")
plot_time_data_with_diffs(orkTimes, farTimes, orkThrusts, farThrusts, "Thrust (N)")
#plot_time_data_with_diffs(orkAlts, farAlts, orkGs, farGs, r"g (m/s$^2$)", "Altitude (m)")

# ATMOSPHERIC CONDITIONS
orkTemps = orkData["Air temperature (Â°C)"]
orkTempsK = orkTemps + 273.15
orkPres = orkData['Air pressure (mbar)']
orkPresPa = orkPres*100.0
orkDens = orkPresPa/(287.053*orkTempsK)
farPres = data["Pressure"]
farDens = data["Density"]

#plot_time_data_with_diffs(orkAlts, farAlts, orkPresPa, farPres, r"P (Pa)", "Altitude (m)")
#plot_time_data_with_diffs(orkAlts, farAlts, orkDens, farDens, r"$\rho$ (kg/m$^3$)", "Altitude (m)")

orkAoAs = orkData["Angle of attack (Â°)"]
orkMachs = orkData["Mach number (â€‹)"]
orkCNs = orkData["Normal force coefficient (â€‹)"]
farAoAs = data["AoA"]
farMachs = data["M"]
farCNs = data["CN"]

plot_time_data_with_diffs(orkTimes, farTimes, orkAoAs, farAoAs, r"$\alpha (^\circ$)")
plot_time_data_with_diffs(orkTimes, farTimes, orkMachs, farMachs, r"M")
plot_time_data_with_diffs(orkTimes, farTimes, orkCNs, farCNs, r"$C_N$")

orkCPx = orkData["CP location (cm)"]/100
farCPx = data["CPx"]
orkCGx = orkData["CG location (cm)"]/100
farCGx = data["CGx"]

#plot_time_data_with_diffs(orkTimes, farTimes, orkCPx, farCPx, r"$CP_{x}$ (m)")
#plot_time_data_with_diffs(orkTimes, farTimes, orkCGx, farCGx, r"$CG_{x}$ (m)")

#plot_Cn_data(orkAoAs, orkMachs, orkCPx, farAoAs, farMachs, farCPx)
plot_Cn_data(orkAoAs, orkMachs, orkCNs, farAoAs, farMachs, farCNs)

orkVelTotal = orkData["Total velocity (m/s)"]
farVelTotal = data["vtot"]

farReynL = data["ReL"]
orkReyn = orkData["Reynolds number (â€‹)"]
orkReynL = orkReyn/0.79 # total length of the rocket

orkKinViscInv = orkReynL/orkVelTotal # i/kinematicViscosity
farKinViscInv = farReynL/farVelTotal
'''
fig, ax1 = plt.subplots()
ax1.plot(orkAlts, orkReynU, )
ax1.plot(farAlts, farReynU, )
ax1.set_yscale("log")
ax1.grid(True)

ax1.set_ylabel(r" $\frac{\text{Re}}{L.u}$ ")
ax1.set_xlabel(r"altitude")
fig.legend(["ork data", "Farseer data"])
fig.tight_layout()
'''


farCdf = data["Cdf"]
orkCdf = orkData["Friction drag coefficient (â€‹)"]
farCdp = data["Cdp"]
orkCdp = orkData["Pressure drag coefficient (â€‹)"]
farCdb = data["Cdb"]
orkCdb = orkData["Base drag coefficient (â€‹)"]
farCd = data["Cd"]
plot_time_data_with_diffs(orkTimes, farTimes, orkCdf, farCdf, r"$C_{D_f}$")
plot_time_data_with_diffs(orkTimes, farTimes, orkCdp, farCdp, r"$C_{D_P}$")
plot_time_data_with_diffs(orkTimes, farTimes, orkCdb, farCdb, r"$C_{D_B}$")
'''
fig, ax1 = plt.subplots()
ax1.plot(orkReynL, orkCdf, )
ax1.plot(farReynL, farCdf, )
ax1.set_xscale("log")
ax1.grid(True)

ax1.set_ylabel(r" $C_{D_f}$")
ax1.set_xlabel(r"$\frac{\text{Re}}{L}$ (m$^{-1}$)")
fig.legend(["ork data", "Farseer data"])
fig.tight_layout()
'''

'''
fig, ax1 = plt.subplots()
ax1.plot(orkMachs, orkCdp, )
ax1.plot(farMachs, farCdp, )
ax1.grid(True)

ax1.set_ylabel(r" $C_{D_p}$")
ax1.set_xlabel(r"$M$")
fig.legend(["ork data", "Farseer data"])
fig.tight_layout()
'''

#plot3dTrajectory(data["Xp"], data["Yp"], data["Zp"])

#plt.plot(data["t"][1:], np.diff(data["t"]))
#print(min(np.diff(data["t"])))
#plt.show()
'''
plot_kinematics(data["t"], data["Zp"], data["Zv"], data["Za"], "z")
plot_kinematics(data["t"], data["Yp"], data["Yv"], data["Ya"], "y")
plot_kinematics(data["t"], data["Xp"], data["Xv"], data["Xa"], "x")

plot_kinematics(data["t"], data["Psi"], data["dPsi"], data["ddPsi"], "psi")
plot_kinematics(data["t"], data["Theta"], data["dTheta"], data["ddTheta"], "theta")
plot_kinematics(data["t"], data["Phi"], data["dPhi"], data["ddPhi"], "phi")

plot_kinematics(data["t"], data["ptot"], data["vtot"], data["atot"], "total")
'''

plt.show()


