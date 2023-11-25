import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import numpy as np
from utils.utils import ORKDat, FARDat, data_to_splines, spline_max
from matplotlib import cm
import tqdm

plt.rcParams["font.family"] = "serif"
plt.rcParams["mathtext.fontset"] = "dejavuserif"

farStyle = {
    "linewidth": 2,
    "color": "blue",
}

farMarker = {
    "marker": "x",
    "markeredgewidth":1,
    "linestyle": "",
    "markersize":20,
    "color":"purple",
}

orkStyle = {
    "linewidth": 2,
    "color": "red",
    "linestyle": "--"
}

orkMarker = {
    "marker": "+",
    "markeredgewidth":2,
    "linestyle": "",
    "markersize":20,
    "color": "orange",
}


def plot_altitudes(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting

    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    t_min = min(_tdata)
    t_max = max(_tdata)
    times = np.linspace(t_min, t_max, 100)

    ork_alt_spl, far_alt_spl = data_to_splines(ork_data, far_data, "Altitude", "Altitude")
    
    far_ap_t, far_apogee = spline_max(far_alt_spl)
    ork_ap_t, ork_apogee = spline_max(ork_alt_spl)
    
    far_al = axs[0].plot(times, far_alt_spl(times), label="FAR", **farStyle)
    ork_al = axs[0].plot(times, ork_alt_spl(times), label="ORK", **orkStyle)
    far_ap = axs[0].plot(far_ap_t, far_apogee, label="FAR\nApogee", **farMarker)
    ork_ap = axs[0].plot(ork_ap_t, ork_apogee, label="ORK\nApogee", **orkMarker)
    diffys = far_alt_spl(times)-ork_alt_spl(times)
    diff = axs[1].plot(times, diffys, color='green', label="$\Delta$Alt")
    
    axs[0].set_ylim(0, max(far_apogee, ork_apogee)*1.1)
    axs[1].set_ylim(min(diffys)*1.1, max(diffys)*1.1)
    
    fig.text(0.02, 0.5, "Altitude (m)", ha="left", va="center", rotation="vertical")
    axs[1].set_xlabel("Time (s)")
    
    # settings to apply to both axes
    for a in axs:
        a.set_xlim(t_min,t_max)
        a.grid(True)
    
    leg_x = 0.8
    leg = fig.legend(loc='center left', bbox_to_anchor=(leg_x, 0.5), fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[0,0,leg_x,1])
    return fig, axs


def plot_lat_motion(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting

    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    t_min = min(_tdata)
    t_max = max(_tdata)
    times = np.linspace(t_min, t_max, 100)
    
    ork_lat, far_lat_x = data_to_splines(ork_data, far_data, "Lateral_distance", "Xp")
    _, far_lat_y = data_to_splines(ork_data, far_data, "Lateral_distance", "Yp")
    
    far_lat_plt = axs[0].plot(times, np.hypot(far_lat_x(times), far_lat_y(times)), linewidth=2, color='blue', label="FAR Lat")
    ork_lat_plt = axs[0].plot(times, ork_lat(times), "--", linewidth=2, color='orange', label="ORK Lat")
    diffys = np.hypot(far_lat_x(times), far_lat_y(times))-ork_lat(times)
    diff = axs[1].plot(times, diffys, color='green', label="$\Delta$Lat")
    
    axs[1].set_ylim(min(diffys)*1.1, max(diffys)*1.1)
    
    fig.text(0.02, 0.5, "Altitude (m)", ha="left", va="center", rotation="vertical")
    axs[1].set_xlabel("Time (s)")
    
    # settings to apply to both axes
    for a in axs:
        a.set_xlim(t_min,t_max)
        a.grid(True)
    
    leg_x = 0.8
    leg = fig.legend(loc='center left', bbox_to_anchor=(leg_x, 0.5))
    leg.set_in_layout(False)
    fig.tight_layout(rect=[0,0,leg_x,1])
    return fig, axs


def plot_both_3d(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[Axes3D]]:
    fig = plt.figure()
    ax:Axes3D = plt.subplot(111,projection="3d")
    
    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    t_min = min(_tdata)
    t_max = max(_tdata)
    times = np.linspace(t_min, t_max, 1000)
    
    ork_x_spl, far_x_spl = data_to_splines(ork_data, far_data, "Position_East_of_launch", "Xp")
    ork_y_spl, far_y_spl = data_to_splines(ork_data, far_data, "Position_North_of_launch", "Yp")
    ork_z_spl, far_z_spl = data_to_splines(ork_data, far_data, "Altitude", "Zp")
    
    end_pt = [
        max(abs(far_x_spl(times))),
        max(abs(far_y_spl(times))),
        0
    ]
    
    camera_azm = np.rad2deg(np.arccos(np.dot(end_pt, [1,0,0])/( np.linalg.norm(end_pt) )))
    
    if camera_azm % 90 > 45:
        camera_azm -= 10
    else:
        camera_azm +=10
    
    ax.plot3D(far_x_spl(times), far_y_spl(times), far_z_spl(times), label="FAR", **farStyle)
    ax.plot3D(ork_x_spl(times), ork_y_spl(times), ork_z_spl(times), label="ORK", **orkStyle)
    
    # plotting origin
    ax.scatter(0,0,0,color = "green")
    # plotting x-y path
    xy_path_z = 0
    ax.plot3D(far_x_spl(times), far_y_spl(times), np.ones_like(times)*xy_path_z, color="blue", linestyle="--", linewidth=1)
    ax.plot3D(ork_x_spl(times), ork_y_spl(times), np.ones_like(times)*xy_path_z, color="red", linestyle=":", linewidth=1)
    
    # plotting x-z path
    xz_path_y = 0 # np.max([ far_y_spl(times), ork_y_spl(times) ])
    ax.plot3D(far_x_spl(times), np.ones_like(times)*xz_path_y, far_z_spl(times), color="blue", linestyle="--", linewidth=1)
    ax.plot3D(ork_x_spl(times), np.ones_like(times)*xz_path_y, far_z_spl(times), color="red", linestyle=":", linewidth=1)
    
    # plotting y-z path
    yz_path_x = 0 # np.max([ far_y_spl(times), ork_y_spl(times) ])
    ax.plot3D(np.ones_like(times)*yz_path_x, far_y_spl(times), far_z_spl(times), color="blue", linestyle="--", linewidth=1)
    ax.plot3D(np.ones_like(times)*yz_path_x, ork_y_spl(times), far_z_spl(times), color="red", linestyle=":", linewidth=1)
    
    ax.set_xlim(0, max(far_x_spl(times)))
    ax.set_ylim(0, max(far_y_spl(times)))
    ax.set_zlim(0, max(far_z_spl(times)))
    
    ax.set_xlabel("Position East of Launch, $x$ (m)")
    ax.set_ylabel("Position North of Launch, $y$ (m)")
    ax.set_zlabel("Altitude, $z$ (m)")
    ax.set_aspect("equal")
    ax.view_init(azim=camera_azm, elev=20)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 0), ncols=2, fancybox=False)
    fig.tight_layout()
    
    return fig, ax

def plot_thrusts(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting

    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    t_min = min(_tdata)
    t_max = max(_tdata)
    times = np.linspace(t_min, t_max, 100)
    
    ork_th_spl, far_th_spl = data_to_splines(ork_data, far_data, "Thrust", "Thrust")
    
    burnout = min([x for x in far_th_spl.roots() if x > 1])
    
    far_al = axs[0].plot(times, far_th_spl(times), label="FAR", **farStyle)
    ork_al = axs[0].plot(times, ork_th_spl(times), label="ORK", **orkStyle)
    diffys = far_th_spl(times)-ork_th_spl(times)
    diff = axs[1].plot(times, diffys, color='green', label="$\Delta$T")
    
    axs[1].set_xlabel("Time (s)")
    
    # settings to apply to both axes
    for a in axs:
        a.set_xlim(t_min,burnout)
        a.grid(True)
    
    txt_x = 0.05
    leg_y = 0.1
    x_lab = fig.text(txt_x, 0.5, "Thrust (N)", ha="right", va="center", rotation="vertical")
    x_lab.set_in_layout(False)
    leg = fig.legend(loc='lower center', bbox_to_anchor=(0.5, 1-leg_y), ncols=3, fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[txt_x,0,1,1-leg_y*2/3])
    return fig, axs


def plot_masses(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting

    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    t_min = min(_tdata)
    t_max = max(_tdata)
    times = np.linspace(t_min, t_max, 100)
    
    ork_th_spl, far_th_spl = data_to_splines(ork_data, far_data, "Thrust", "Thrust")
    
    ork_m_spl, far_m_spl = data_to_splines(ork_data, far_data, "Mass", "Mass")
    
    burnout = min([x for x in far_th_spl.roots() if x > 1])
    
    far_al = axs[0].plot(times, far_m_spl(times), label="FAR", **farStyle)
    ork_al = axs[0].plot(times, ork_m_spl(times)/1000, label="ORK", **orkStyle)
    diffys = far_m_spl(times)-ork_m_spl(times)/1000
    diff = axs[1].plot(times, diffys, color='green', label="$\Delta m$")
    
    axs[1].set_xlabel("Time (s)")
    
    # settings to apply to both axes
    for a in axs:
        a.set_xlim(t_min,burnout)
        a.grid(True)
    
    txt_x = 0.05
    leg_y = 0.1
    x_lab = fig.text(txt_x, 0.5, "Mass (kg)", ha="right", va="center", rotation="vertical")
    x_lab.set_in_layout(False)
    leg = fig.legend(loc='lower center', bbox_to_anchor=(0.5, 1-leg_y), ncols=3, fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[txt_x,0,1,1-leg_y*2/3])
    return fig, axs


def plot_rot_inert(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting

    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    t_min = min(_tdata)
    t_max = max(_tdata)
    times = np.linspace(t_min, t_max, 100)
    
    ork_th_spl, far_th_spl = data_to_splines(ork_data, far_data, "Thrust", "Thrust")
    
    ork_m_spl, far_m_spl = data_to_splines(ork_data, far_data, "Rotational_moment_of_inertia", "Izz")
    
    burnout = min([x for x in far_th_spl.roots() if x > 1])
    
    far_al = axs[0].plot(times, far_m_spl(times), label="FAR", **farStyle)
    ork_al = axs[0].plot(times, ork_m_spl(times), label="ORK", **orkStyle)
    diffys = far_m_spl(times)-ork_m_spl(times)
    diff = axs[1].plot(times, diffys, color='green', label="$\Delta J$")
    
    axs[1].set_xlabel("Time (s)")
    
    # settings to apply to both axes
    for a in axs:
        a.set_xlim(t_min,burnout)
        a.grid(True)
    
    txt_x = 0.05
    leg_y = 0.1
    x_lab = fig.text(txt_x, 0.5, "Rotational Moment of Inertia (kg.m$^2$)", ha="right", va="center", rotation="vertical")
    x_lab.set_in_layout(False)
    leg = fig.legend(loc='lower center', bbox_to_anchor=(0.5, 1-leg_y), ncols=3, fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[txt_x,0,1,1-leg_y*2/3])
    return fig, axs

def plot_long_inert(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting

    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    t_min = min(_tdata)
    t_max = max(_tdata)
    times = np.linspace(t_min, t_max, 100)
    
    ork_th_spl, far_th_spl = data_to_splines(ork_data, far_data, "Thrust", "Thrust")
    
    ork_m_spl, far_m_spl = data_to_splines(ork_data, far_data, "Longitudinal_moment_of_inertia", "Ixx")
    
    burnout = min([x for x in far_th_spl.roots() if x > 1])
    
    far_al = axs[0].plot(times, far_m_spl(times), label="FAR", **farStyle)
    ork_al = axs[0].plot(times, ork_m_spl(times), label="ORK", **orkStyle)
    diffys = far_m_spl(times)-ork_m_spl(times)
    diff = axs[1].plot(times, diffys, color='green', label="$\Delta I$")
    
    axs[1].set_xlabel("Time (s)")
    
    # settings to apply to both axes
    for a in axs:
        a.set_xlim(t_min,burnout)
        a.grid(True)
    
    txt_x = 0.05
    leg_y = 0.1
    x_lab = fig.text(txt_x, 0.5, "Longitudinal Moment of Inertia (kg.m$^2$)", ha="right", va="center", rotation="vertical")
    x_lab.set_in_layout(False)
    leg = fig.legend(loc='lower center', bbox_to_anchor=(0.5, 1-leg_y), ncols=3, fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[txt_x,0,1,1-leg_y*2/3])
    return fig, axs


def plot_g(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(1)
    axs : plt.Axes = axs # type hinting
    
    ork_g_spl, far_g_spl = data_to_splines(ork_data, far_data, "Gravitational_acceleration", "g")
    ork_alt_spl, far_alt_spl = data_to_splines(ork_data, far_data, "Altitude", "Altitude")
    
    t_apogee_far, _ = spline_max(far_alt_spl)
    t_apogee_ork, _ = spline_max(ork_alt_spl)
    far_times = np.linspace(0,t_apogee_far,1000)
    ork_times = np.linspace(0,t_apogee_far,1000)
    
    far_al = axs.plot(far_alt_spl(far_times), far_g_spl(far_times), label="FAR", **farStyle)
    ork_al = axs.plot(ork_alt_spl(ork_times), ork_g_spl(ork_times), label="ORK", **orkStyle)
    
    axs.grid(True)
    axs.set_xlabel("Altitude (m)")
    
    figa = fig.get_figwidth() * fig.get_figheight()
    fig_h = np.sqrt(figa/2)
    fig_w = 2*fig_h
    fig.set_figwidth(fig_w)
    fig.set_figheight(fig_h)
    
    txt_x = 0.05
    leg_y = 0.1
    x_lab = fig.text(txt_x, 0.5, "Gravitational Acceleration (m/s$^2$)", ha="right", va="center", rotation="vertical")
    x_lab.set_in_layout(False)
    leg = fig.legend(loc='lower center', bbox_to_anchor=(0.5, 1-leg_y), ncols=2, fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[txt_x,0,1,1-leg_y*2/3])
    return fig, axs


def plot_alpha(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting

    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    t_min = min(_tdata)
    t_max = max(_tdata)
    times = np.linspace(t_min, t_max, 100)
    
    ork_th_spl, far_th_spl = data_to_splines(ork_data, far_data,"Angle_of_attack","AoA"  )
    
    ork_time_ran = [min(ork_data["Time"][1:]), 2]
    ork_ts = np.linspace(ork_time_ran[0],ork_time_ran[1], 1000)
    far_time_ran = [min(far_data["t"]), 2]
    far_ts = np.linspace(far_time_ran[0],far_time_ran[1], 1000) # heh heh
    
    far_al = axs[0].plot(far_ts, far_th_spl(far_ts), label="FAR", **farStyle)
    ork_al = axs[0].plot(ork_ts, np.interp(ork_ts, ork_data["Time"][1:], ork_data["Angle_of_attack"][1:]), label="ORK", **orkStyle)
    
    min_time = min(ork_data["Time"][1:])
    max_time = 2
    intervals = np.linspace(min_time, max_time, 10000)
    diffys = np.interp(intervals, ork_data["Time"][1:], ork_data["Angle_of_attack"][1:]) - far_th_spl(intervals)
    # diffys = far_th_spl(times)-ork_th_spl(times)
    diff = axs[1].plot(intervals, diffys, color='green', label="$\Delta\\alpha$")
    
    #axs[1].set_xlabel("Time (s)")
    
    # settings to apply to both axes
    for a in axs:
        a.set_xlim(t_min, 2)
        a.grid(True)
    
    txt_x = 0.05
    leg_y = 0.1
    note_lab = axs[0].text(0.95, 0.98, "Note: the remainder of the flight is stable\nwith only \'noise\' in $\\alpha$", ha="right", va="top",transform=axs[0].transAxes)
    x_lab = fig.text(txt_x, 0.5, "$\\alpha$ (rad)", ha="right", va="center", rotation="vertical")
    x_lab.set_in_layout(False)
    leg = fig.legend(loc='lower center', bbox_to_anchor=(0.5, 1-leg_y), ncols=3, fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[txt_x,0,1,1-leg_y*2/3])
    return fig, axs


def plot_dens(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting
    
    ork_g_spl, far_g_spl = data_to_splines(ork_data, far_data, "Density", "Density")
    ork_alt_spl, far_alt_spl = data_to_splines(ork_data, far_data, "Altitude", "Altitude")
    t_apogee_far, far_ap = spline_max(far_alt_spl)
    t_apogee_ork, ork_ap = spline_max(ork_alt_spl)
    
    far_times = np.linspace(0,t_apogee_far,1000)
    ork_times = np.linspace(0,t_apogee_ork,1000)
    
    far_al = axs[0].plot(far_alt_spl(far_times), far_g_spl(far_times), label="FAR", **farStyle)
    ork_al = axs[0].plot(ork_alt_spl(ork_times), ork_g_spl(ork_times), label="ORK", **orkStyle)
    
    intervals = np.linspace(0,min(t_apogee_far, t_apogee_ork),10000)
    far_xvals = np.interp(intervals, far_data["t"], far_data["Altitude"])
    ork_xvals = np.interp(intervals, ork_data["Time"], ork_data["Altitude"])
    far_yvals = np.interp(intervals, far_data["t"], far_data["Density"])
    ork_yvals = np.interp(intervals, ork_data["Time"], ork_data["Density"])
    
    max_height = min(far_ap, ork_ap)
    hs = np.linspace(0, max_height,10000)
    
    diffs = np.interp(hs, far_xvals, far_yvals) - np.interp(hs, ork_xvals, ork_yvals)
    
    diff = axs[1].plot(hs, diffs, color='green', label="$\Delta\\rho$")
    
    axs[0].grid(True)
    axs[1].set_xlabel("Altitude (m)")
    
    #figa = fig.get_figwidth() * fig.get_figheight()
    #fig_h = np.sqrt(figa/2)
    #fig_w = 2*fig_h
    #fig.set_figwidth(fig_w)
    #fig.set_figheight(fig_h)# settings to apply to both axes
    for a in axs:
        a.grid(True)
    
    txt_x = 0.05
    leg_y = 0.1
    x_lab = fig.text(txt_x, 0.5, "Air Density (kg/m$^3$)", ha="right", va="center", rotation="vertical")
    x_lab.set_in_layout(False)
    leg = fig.legend(loc='lower center', bbox_to_anchor=(0.5, 1-leg_y), ncols=3, fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[txt_x,0,1,1-leg_y*2/3])
    return fig, axs


def plot_cn(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting

    ork_times = ork_data["Time"]
    far_times = far_data["t"]
    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    
    ork_cn_dat = ork_data["Normal_force_coefficient"]
    far_cn_dat = far_data["CN"]
    
    times = np.linspace(0, 2, 1000)
    ork_y = np.interp(times, ork_times, ork_cn_dat) # propa orky
    far_y = np.interp(times, far_times, far_cn_dat)
    
    far_al = axs[0].plot(times, far_y, label="FAR", **farStyle)
    ork_al = axs[0].plot(times, ork_y, label="ORK", **orkStyle)
    
    diffys = far_y - ork_y
    # diffys = far_th_spl(times)-ork_th_spl(times)
    diff = axs[1].plot(times, diffys, color='green', label="$\Delta C_{N}$")
    
    #axs[1].set_xlabel("Time (s)")
    
    # settings to apply to both axes
    for a in axs:
        a.set_xlim(0, 2)
        a.grid(True)
    
    txt_x = 0.05
    leg_y = 0.1
    note_lab = axs[0].text(0.95, 0.98, "Note: the remainder of the flight is stable\nwith only \'noise\' in $C_{N}$", ha="right", va="top",transform=axs[0].transAxes)
    x_lab = fig.text(txt_x, 0.5, "$C_N$", ha="right", va="center", rotation="vertical")
    x_lab.set_in_layout(False)
    leg = fig.legend(loc='lower center', bbox_to_anchor=(0.5, 1-leg_y), ncols=3, fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[txt_x,0,1,1-leg_y*2/3])
    return fig, axs


def plot_cp(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig, axs = plt.subplots(2, sharex=True)
    axs : tuple[plt.Axes] = axs # type hinting

    ork_times = ork_data["Time"]
    far_times = far_data["t"]
    _tdata = np.concatenate([ ork_data["Time"], far_data["t"] ])
    
    ork_cn_dat = ork_data["CP_location"]/100
    far_cn_dat = far_data["CPx"]
    
    times = np.linspace(0, 2, 1000)
    ork_y = np.interp(times, ork_times, ork_cn_dat) # propa orky
    far_y = np.interp(times, far_times, far_cn_dat)
    
    far_al = axs[0].plot(times, far_y, label="FAR", **farStyle)
    ork_al = axs[0].plot(times, ork_y, label="ORK", **orkStyle)
    
    diffys = far_y - ork_y
    # diffys = far_th_spl(times)-ork_th_spl(times)
    diff = axs[1].plot(times, diffys, color='green', label="$\Delta$ CP$_x$")
    
    #axs[1].set_xlabel("Time (s)")
    
    # settings to apply to both axes
    for a in axs:
        a.set_xlim(0, 2)
        a.grid(True)
    
    txt_x = 0.05
    leg_y = 0.1
    note_lab = axs[0].text(0.95, 0.98, "Note: the remainder of the flight is stable\nwith only \'noise\' in CP$_x$", ha="right", va="top",transform=axs[0].transAxes)
    x_lab = fig.text(txt_x, 0.5, "CP$_x$ (m)", ha="right", va="center", rotation="vertical")
    x_lab.set_in_layout(False)
    leg = fig.legend(loc='lower center', bbox_to_anchor=(0.5, 1-leg_y), ncols=3, fancybox=False)
    leg.set_in_layout(False)
    fig.tight_layout(rect=[txt_x,0,1,1-leg_y*2/3])
    return fig, axs


def plot_cn_m_alpha(ork_data:ORKDat, far_data:FARDat) -> tuple[plt.Figure, tuple[plt.Axes]]:
    fig = plt.figure()
    ax:Axes3D = plt.subplot(111,projection="3d")
    
    ork_cn_dat = ork_data["Normal_force_coefficient"]
    ork_mach_dat = ork_data["Mach_number"]
    ork_alpha_dat = ork_data["Angle_of_attack"]
    
    far_cn_dat = far_data["CN"]
    far_mach_dat = far_data["M"]
    far_alpha_dat = far_data["AoA"]
    
    ork_mach_grid, ork_alpha_grid = np.meshgrid(ork_cn_dat, ork_alpha_dat, sparse=True)
    cn_grid = np.empty((ork_mach_grid.shape[1], ork_alpha_grid.shape[0]))
    
    # this takes ages
    '''
    print("generating grid")
    for i in tqdm.tqdm(range(cn_grid.shape[0]),position=0):
        for j in tqdm.tqdm(range(cn_grid.shape[1]),position=1, leave=False):
            cn_grid[i,j] = float("nan")
            for k in tqdm.tqdm(range(len(ork_cn_dat)),position=2, leave=False):
                if ork_mach_dat[k] == ork_mach_grid[0,i] and ork_alpha_dat[k] == ork_alpha_grid[j,0]:
                    cn_grid[i,j] = ork_cn_dat[k]
    ax.plot_surface(ork_mach_grid, ork_alpha_grid, cn_grid)
    '''
    
    ax.plot3D(ork_alpha_dat, ork_mach_dat, ork_cn_dat)
    ax.plot3D(far_alpha_dat, far_mach_dat, far_cn_dat)
    
    ax.set_xlabel("$\alpha$ (rad)")
    ax.set_ylabel("Mach Number")
    ax.set_zlabel("")
    
    #ax.plot_surface(ork_alpha_dat, ork_mach_dat, ork_cn_dat)
    
    return fig, ax