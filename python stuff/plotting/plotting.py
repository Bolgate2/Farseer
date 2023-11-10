import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

def plot3dTrajectory(x,y,z):
    """Plot a 3D graph of the trajectory

    Parameters
    ----------
    None

    Return
    ------
    None
    """
    # shamelessly stolen from rocketpy
    elevation = 0
    # Get max and min x and y
    maxZ = max(z[:] - elevation)
    maxX = max(x[:])
    minX = min(x[:])
    maxY = max(y[:])
    minY = min(y[:])
    maxXY = max(maxX, maxY)
    minXY = min(minX, minY)

    # Create figure
    fig1 = plt.figure(figsize=(9, 9))
    ax1 = plt.subplot(111, projection="3d")
    ax1.plot(x[:], y[:], zs=0, zdir="z", linestyle="--")
    ax1.plot(
        x[:],
        z[:] - elevation,
        zs=minY,
        zdir="y",
        linestyle="--",
    )
    ax1.plot(
        y[:],
        z[:] - elevation,
        zs=minX,
        zdir="x",
        linestyle="--",
    )
    ax1.plot(
        x[:], y[:], z[:] - elevation, linewidth="2"
    )
    ax1.scatter(0, 0, 0)
    ax1.set_xlabel("X - East (m)")
    ax1.set_ylabel("Y - North (m)")
    ax1.set_zlabel("Z - Altitude Above Ground Level (m)")
    ax1.set_title("Flight Trajectory")
    ax1.set_zlim([0, maxZ])
    ax1.set_ylim([minY, maxY])
    ax1.set_xlim([minX, maxX])
    
    #if np.all( maxZ/np.array([maxX, maxY]) < 10):
    #    ax1.set_aspect('equal', adjustable='box')
    ax1.view_init(15, 55)
    fig1.tight_layout()
    return fig1, ax1

def get_lims_and_ticks(data:NDArray) -> tuple[NDArray, NDArray]:
    if np.min(data) == np.max(data):
        return np.array([-10, 10]), np.arange(-10,11,5)
    ax_max = np.max(np.abs(data))
    ax_pow_10 = np.power( 10, np.ceil( np.log10(ax_max) )-1 )
    ax1_tick_space =  np.ceil(ax_max/ax_pow_10)*ax_pow_10/10
    
    lims = np.array([np.floor(np.min(data/ax1_tick_space))-0.5, np.ceil(np.max(data/ax1_tick_space))+0.5])*ax1_tick_space
        
    ax1_ticks = np.concatenate(
        [np.arange( -ax1_tick_space, lims[0], -ax1_tick_space), np.arange(0, lims[1], ax1_tick_space)]
    )
        
    return lims, ax1_ticks

def plot_kinematics(times:NDArray, position:NDArray, velocity:NDArray, acceleration:NDArray, name:str):
    x_concat = np.concatenate([position,velocity])
    
    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    ax1.plot(times, position, color='green')
    ax1.plot(times, velocity, color='blue')
    ax2.plot(times, acceleration, color='red')
    fig.legend(
        [fr'${name}$', fr'$v_{{ {name} }}$', fr'$a_{{ {name} }}$'],
        loc='upper center',
        ncol=3,
        framealpha=0,
        bbox_to_anchor=(0.5, 1),
    )
    ax1.grid(True)
    '''
    x_lim = [-0.5,np.round(np.max(times))+0.5]
    ax1.set_xlim(x_lim)
    
    ax_1_lim, ax_1_ticks = get_lims_and_ticks(x_concat)
    ax1.set_ylim(ax_1_lim)
    # doing the limits like this aligns the zeroes on both axes
    if np.max(x_concat) != 0:
        ratio = np.max(np.abs(acceleration))/np.max(np.abs(x_concat))
        ax_2_lim = ax_1_lim*ratio
        _, ax_2_ticks = get_lims_and_ticks(acceleration)
    else:
        ax_2_lim, ax_2_ticks = get_lims_and_ticks(acceleration)
    
    ax2.set_ylim(ax_2_lim)
    
    ax1.set_yticks(ax_1_ticks)
    ax2.set_yticks(ax_2_ticks)
    
    x_tick_space = 2
    x_ticks = np.arange(0, x_lim[1], x_tick_space)
    ax1.set_xticks(x_ticks)
    '''
    ax1.set_xlabel(r'Time (s)')
    ax1.set_ylabel(fr'${name}$ (m)$\quad$$v_{name}$ (m/s)')
    ax2.set_ylabel(fr'$a_{name}$ (m/s$^2$)')
    
    fig.tight_layout()
    fig.subplots_adjust(top=0.94)
    return fig, ax1, ax2