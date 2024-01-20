from pathlib import Path
from os import PathLike
from typing import Callable
import matplotlib.pyplot as plt
import numpy as np
import os
    
from utils.utils import load_ork_data, offset_fil_path, load_far_data, ORKDat, FARDat
from plotting.far_ork_plots import (
    plot_altitudes, plot_lat_motion, plot_both_3d, plot_thrusts, plot_masses, plot_rot_inert, plot_long_inert, plot_g,
    plot_alpha, plot_dens, plot_cn, plot_cp, plot_cn_m_alpha, plot_cp_m_alpha,
    plot_cdf_vs_re_m, plot_cdf, plot_cdp, plot_cdb, plot_cdb_mach, plot_cdp_mach
    )


try:
    res_fol = Path(os.environ["FARPLOTRES"])
except KeyError as e:
    print("Please define the folder you wish to save plots in, in your environment variables as with the key \"FARPLOTRES\"")
    raise e

def save_fig(fig : plt.Figure, path : PathLike, overwrite=False, **kwargs):
    alt_pth = Path(res_fol, path)
    if overwrite:
        res_pth = alt_pth
    else:
        res_pth = offset_fil_path(alt_pth)
    fig.savefig(res_pth, dpi=450,**kwargs)
    print(f"Saved figure to {res_pth.as_posix()}")

def plot_proj(orkdat:ORKDat, fardat:FARDat):
    plots = {
        #"altitude",
        #"lateral motion",
        #"3dpath",
        #"Thrust",
        #"Mass",
        "rotinert",
        "longinert",
        #"g",
        }
    
    if "altitude" in plots:
        alt_fig, alt_axs = plot_altitudes(orkdat, fardat)
        alt_path = Path("Projectile Motion", "altitude.png")
        save_fig(alt_fig, alt_path)
    
    if "lateral motion" in plots:
        lat_fig, lat_axs = plot_lat_motion(orkdat, fardat)
    
    if "3dpath" in plots:
        threed_fig, threed_ax = plot_both_3d(orkdat, fardat)
        threed_path = Path("Projectile Motion", "3d_flight.png")
        save_fig(threed_fig, threed_path)
    
    if "Thrust" in plots:
        thrust_fig, thrust_ax = plot_thrusts(orkdat, fardat)
        thrust_path = Path("Projectile Motion", "thrust.png")
        save_fig(thrust_fig, thrust_path)
    
    if "Mass" in plots:
        mass_fig, mass_ax = plot_masses(orkdat, fardat)
        mass_path = Path("Projectile Motion", "mass.png")
        save_fig(mass_fig, mass_path)
    
    if "rotinert" in plots:
        roti_fig, roti_ax = plot_rot_inert(orkdat, fardat)
        roti_path = Path("Projectile Motion", "rotinert.png")
        save_fig(roti_fig, roti_path)
    
    if "longinert" in plots:
        lon_fig, lon_ax = plot_long_inert(orkdat, fardat)
        lon_path = Path("Projectile Motion", "longinert.png")
        save_fig(lon_fig, lon_path)
    
    if "g" in plots:
        g_fig, g_ax = plot_g(orkdat, fardat)
        g_path = Path("Projectile Motion", "grav.png")
        save_fig(g_fig, g_path)

def plot_norm(orksubdat:ORKDat, farsubdat:FARDat, orksuperdat:ORKDat, farsuperdat:ORKDat):
    plots:set[tuple[str,callable]] = {
        #("altitude", plot_altitudes),
        #("3d_flight", plot_both_3d),
        #("alpha", plot_alpha),
        #("dens", plot_dens),
        #("CN", plot_cn),
        #("CP", plot_cp),
        ("cn_m_alpha", plot_cn_m_alpha),
        ("cn_m_cp", plot_cp_m_alpha),
        }
    
    fol = "Norm"
    
    for name, func in plots:
        for dat, tag in zip([[orksubdat, farsubdat], [orksuperdat, farsuperdat]], ["sub", "super"]):
            alt_fig, alt_sub_axs = func(*dat)
            alt_path = Path(fol, f"{name}_{tag}.png")
            save_fig(alt_fig, alt_path)
    
def plot_drag(orksubdat:ORKDat, farsubdat:FARDat, orksuperdat:ORKDat, farsuperdat:ORKDat):
    plots:set[tuple[str,callable]] = {
        #("altitude", plot_altitudes),
        #("3d_flight", plot_both_3d),
        #("alpha", plot_alpha),
        #("dens", plot_dens),
        #("CN", plot_cn),
        #("CP", plot_cp),
        #("cn_m_alpha", plot_cn_m_alpha),
        #("cn_m_cp", plot_cp_m_alpha),
        #("cf_m_re", plot_cdf_vs_re_m)
        ("cdf", plot_cdf),
        #("cdp", plot_cdp),
        ("cdb", plot_cdb),
        ("cdb_m", plot_cdb_mach),
        ("cdp_m", plot_cdp_mach)
        }
    overwrite = True
    fol = "Drag"
    
    for name, func in plots:
        for dat, tag in zip([[orksubdat, farsubdat], [orksuperdat, farsuperdat]], ["sub", "super"]):
            alt_fig, alt_sub_axs = func(*dat)
            alt_path = Path(fol, f"{name}_{tag}.png")
            save_fig(alt_fig, alt_path, overwrite=overwrite)

def main():
    p = Path(Path(__file__).parent.parent, "results/6030sub")
    psuper = Path(Path(__file__).parent.parent, "results/6030super")
    
    op = Path(p.parent, "ork6030res.csv")
    opsuper = Path(p.parent, "ork6030resBIG.csv")
    print("loading far data")
    fardat = load_far_data(p)
    farsuperdat = load_far_data(psuper)
    print("loading ork data")
    orkdat = load_ork_data(op)
    orksuperdat = load_ork_data(opsuper)
    
    fardat["ReL"] *= 0.79
    farsuperdat["ReL"] *= 0.48
    
    print(max(fardat["Xp"]), max(orkdat["Position_East_of_launch"]))
    print(max(farsuperdat["Xp"]), max(orksuperdat["Position_East_of_launch"]))
    exit()
    
    # plot_proj(orkdat, fardat)
    
    # plot_norm(orkdat, fardat, orksuperdat, farsuperdat)
    
    plot_drag(orkdat, fardat, orksuperdat, farsuperdat)

    plt.show()

if __name__ == "__main__":
    main()