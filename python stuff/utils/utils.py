from os import PathLike
import os
from pathlib import Path
import numpy as np
from numpy.typing import NDArray, ArrayLike
from typing import TypedDict, NamedTuple
from scipy.interpolate import CubicSpline

def load_data(dataPath:PathLike) -> dict[str, NDArray]:
    heads = np.genfromtxt(dataPath, str, max_rows=1, delimiter=',') # loading headers
    res = np.genfromtxt(dataPath, float, skip_header=1, delimiter=',') # loading data
    heads = [ x.strip() for x in heads ]  # stripping white space
    data = {}
    for i,name in enumerate(heads):
        data[name] = res[:,i]
    return data

class ORKDat(TypedDict):
    Time : list[float]
    Altitude : list[float]
    Vertical_velocity : list[float]
    Vertical_acceleration : list[float]
    Total_velocity : list[float]
    Total_acceleration : list[float]
    Position_East_of_launch : list[float]
    Position_North_of_launch : list[float]
    Lateral_distance : list[float]
    Lateral_direction : list[float]
    Lateral_velocity : list[float]
    Lateral_acceleration : list[float]
    Latitude : list[float]
    Longitude : list[float]
    Gravitational_acceleration : list[float]
    Angle_of_attack : list[float]
    Roll_rate : list[float]
    Pitch_rate : list[float]
    Yaw_rate : list[float]
    Mass : list[float]
    Motor_mass : list[float]
    Longitudinal_moment_of_inertia : list[float]
    Rotational_moment_of_inertia : list[float]
    CP_location : list[float]
    CG_location : list[float]
    Stability_margin_calibers : list[float]
    Mach_number : list[float]
    Reynolds_number : list[float]
    Thrust : list[float]
    Drag_force : list[float]
    Drag_coefficient : list[float]
    Axial_drag_coefficient : list[float]
    Friction_drag_coefficient : list[float]
    Pressure_drag_coefficient : list[float]
    Base_drag_coefficient : list[float]
    Normal_force_coefficient : list[float]
    Pitch_moment_coefficient : list[float]
    Yaw_moment_coefficient : list[float]
    Side_force_coefficient : list[float]
    Roll_moment_coefficient : list[float]
    Roll_forcing_coefficient : list[float]
    Roll_damping_coefficient : list[float]
    Pitch_damping_coefficient : list[float]
    Reference_length : list[float]
    Reference_area : list[float]
    Vertical_orientation : list[float]
    Lateral_orientation : list[float]
    Wind_velocity : list[float]
    Air_temperature : list[float]
    Air_pressure : list[float]
    Speed_of_sound : list[float]
    Simulation_time_step : list[float]
    Computation_time : list[float]
    Density : list[float]

class FARDat(TypedDict):
    Xp: list[float]
    Xv: list[float]
    Yp: list[float]
    Yv: list[float]
    Zp: list[float]
    Zv: list[float]
    Phi: list[float]
    dPhi: list[float]
    Theta: list[float]
    dTheta: list[float]
    Psi: list[float]
    dPsi: list[float]
    t: list[float]
    Cdf: list[float]
    Iyy: list[float]
    g: list[float]
    AoA: list[float]
    Cd: list[float]
    Ixx: list[float]
    Pressure: list[float]
    Yaw_Damping: list[float]
    Density: list[float]
    CPx: list[float]
    Mass: list[float]
    ctime: list[float]
    Cdp: list[float]
    CGx: list[float]
    Izz: list[float]
    Altitude: list[float]
    Thrust: list[float]
    Cdb: list[float]
    CN: list[float]
    ReL: list[float]
    M: list[float]
    Pitch_Damping: list[float]

def load_ork_data(dataPath:PathLike) -> ORKDat:
    heads = np.genfromtxt(dataPath, str, max_rows=1, delimiter=',') # loading headers
    res = np.genfromtxt(dataPath, float, skip_header=1, delimiter=',') # loading data
    heads = [ x.strip() for x in heads ]  # stripping white space
    data = {}
    for i,name in enumerate(heads):
        data[name] = res[:,i]
    data2 = {}
    for k,v in data.items():
        newK = "_".join( k.split(" ")[:-1])
        data2[newK] = v
    dat = ORKDat(**data2)
    
    orkTemps = dat["Air_temperature"]
    orkTempsK = orkTemps + 273.15
    orkPres = dat["Air_pressure"]
    orkPresPa = orkPres*100.0
    orkDens = orkPresPa/(287.053*orkTempsK)
    dat["Density"] = orkDens
    return dat


def load_far_data(dataPath:PathLike) -> FARDat:
    heads = np.genfromtxt(dataPath, str, max_rows=1, delimiter=',') # loading headers
    res = np.genfromtxt(dataPath, float, skip_header=1, delimiter=',') # loading data
    heads = [ x.strip() for x in heads ]  # stripping white space
    data = {}
    for i,name in enumerate(heads):
        data[name] = res[:,i]
    data2 = {}
    for k,v in data.items():
        k:str = k
        newK = k.replace(" ", "_")
        data2[newK] = v
    dat = FARDat(**data2)
    return dat


def same_shape_data(ork_data:ORKDat, far_data:FARDat, ork_key, far_key) -> tuple[NDArray, NDArray, NDArray, NDArray]:
    time_concat = np.concatenate([ork_data['Time'], far_data['t']])
    times = np.arange(np.min(time_concat),np.max(time_concat), 0.001)
    xspl_ork = CubicSpline()
    interpd_ork = np.interp(times, ork_data['Time'], ork_data[ork_key])
    interpd_far = np.interp(times, far_data['t'], far_data[far_key])
    data_diffs = interpd_ork - interpd_far
    return times, interpd_ork, interpd_far, data_diffs


def data_to_splines(ork_data:ORKDat, far_data:FARDat, ork_key, far_key) -> tuple[CubicSpline, CubicSpline]:
    """creates cubic splines of ork and far data points with respect to time

    Args:
        ork_data (ORKDat): ork data object
        far_data (dict[str,ArrayLike[float]]): farseer data object
        ork_key (str): data key for ork
        far_key (str): data key for far

    Returns:
        tuple[CubicSpline, CubicSpline]: (ork spline, far spline)
    """
    ork_times = np.array(ork_data["Time"])
    ork_dat = np.array(ork_data[ork_key])
    ork_times = ork_times[ np.isnan(ork_dat) == False ]
    ork_dat = ork_dat[ np.isnan(ork_dat) == False ]
    
    if ork_key == "Angle_of_attack":
        xspl_ork = CubicSpline(ork_times, ork_dat % 90)
    else:
        xspl_ork = CubicSpline(ork_times, ork_dat)
    if far_key == "AoA":
        xspl_far = CubicSpline(far_data["t"], far_data[far_key] % 90)
    else:
        xspl_far = CubicSpline(far_data["t"], far_data[far_key])
    return xspl_ork, xspl_far

def spline_max(spl:CubicSpline) -> tuple[float, float]:
    r = spl.derivative().roots()
    spl_max = None
    max_t = None
    for root in r:
        val = spl(root)
        if spl_max is None:
            spl_max = val
            max_t = root
        elif val > spl_max:
            spl_max = val
            max_t = root
    return max_t, spl_max

def spline_min(spl:CubicSpline):
    pass


def offset_fil_path(path:PathLike):
    p = Path(path)
    basename = p
    count = 0
    while p.exists():
        count = count+1
        if p.name.split(".")[0][-1].isnumeric():
            filname = p.name.split("_")
            filname = "".join(filname[:-1]) + "_" + str(count) + "." + filname[-1].split(".")[-1]
            p = Path(basename.parent, filname)
        else:
            filspl = p.name.split(".")
            filname = filspl[0] + "_" + str(count) + "." + filspl[-1]
            p = Path(basename.parent, filname)
    return p

def cd_to_cda_mul(alphas:NDArray):
    a = np.deg2rad(alphas)
    out_mul = np.where(alphas <= 17,
                       -22.971*np.power(a,3) + 10.223*np.power(a,2) + 1,
                       -1.4800*np.power(a,4) + 6.7849*np.power(a,3) - 10.063*np.power(a,2) + 4.3340*a + 0.7342
                       )
    return out_mul