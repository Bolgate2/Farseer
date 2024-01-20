from os import PathLike
import numpy as np
from numpy.typing import NDArray
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from utils.utils import load_data


def main():
    this_path = Path(__file__)

    int_fols = ["euler", "rk4", "rk45", "rk56"]

    s1a = "**/subsonic*a"
    s2a = "**/supersonic*a"
    s1c = "**/subsonic*c"
    s2c = "**/supersonic*c"
    fil_groups = [s1a, s1c, s2a, s2c]
    titles = ["subsonic alone", "subsonic combined", "supersonic alone", "supersonic combined"]

    # print("\\hline Simulation", f"\\multirow{{2}}{{*}}{{Rocket}}", f"\\multirow{{2}}{{*}}{{Threading}}", f"\\multirow{{2}}{{*}}{{Apogee ($m$)}}","\\multirow{2}{*}{Steps}","Average Sim ","$\\Delta$ Sim\\\\", sep=" & ")
    # print("Type",                                              "",                                   "",                                    "",                       "",  "Time ($s$)",  "Time ($s$)\\\\\\hline", sep=" & ")

    print("loading files")
    numFil = len(fil_groups)*len(int_fols)
    print(f"0/{numFil}", end="")
    timDict : dict[str,list[float]] = {}

    for i,res_fol in enumerate(int_fols):
        res_path = Path(this_path.parent.parent, "results", res_fol)
        # print(f"\\multirow{{4}}{{*}}{{{res_fol.upper()}}}")
        for j,gl in enumerate(fil_groups):
            print(f"\r{i*len(fil_groups)+j}/{numFil}", end="")
            times = []
            datLen = 0
            apogees = []
            printed = False
            
            for f in res_path.glob(gl):
                d = load_data(f)
                dat_times = d["ctime"]
                datLen = len(d["ctime"])
                times.append(sum(dat_times)/1e6)
                apogees.append( max(d["Zp"]))

            timDict[str(res_path.name) + "_" + titles[j].replace(" ", "_")] = times
            #t_split = titles[j].split(" ")
            '''
            print("",
                f"\\multirow{{2}}{{*}}{{{t_split[0]}}}" if j%2==0 else "",
                f"{t_split[1]}",
                "\\multirow{{2}}{{*}}{{{:.2f}}}".format(sum(apogees)/len(apogees)) if j%2==0 else "",
                "\\multirow{{2}}{{*}}{{{}}}".format(datLen) if j%2==0 else "",
                "{:.5f}".format(sum(times)/len(times)),
                u"{:.5f}".format((max(times)-min(times))/2),
                sep=" & ", end="\\\\")
            print("\\cline{1-1}" if j == len(fil_groups)-1 else "", "\\cline{2-2}\\cline{4-5}" if j%2!=0 else "","\\cline{3-3}\\cline{6-7}",sep="")
            '''

            #print(res_fol,titles[j], "&", "{:.5f}".format(sum(times)/len(times)), "&",  "{:.5f}".format(max(times)-min(times)), "&", "{:.5f}".format(sum(apogees)/len(apogees)), "\\\\\\hline")
            #print("times:", ["{:.5f}".format(t) for t in times], "avg: {:.5f}".format(sum(times)/len(times)), "range: {:.5f}".format(max(times)-min(times)), "steps", datLen)
            #print("apogees", ["{:.3f}".format(t) for t in apogees], "avg: {:.5f}".format(sum(apogees)/len(apogees)), "range: {:.5f}".format(max(apogees)-min(apogees)))

    print(f"\r{numFil}/{numFil}")
    print("loaded data")
    print(timDict)
    fig, (ax1, ax2) = plt.subplots(1,2)
    ax1:plt.Axes = ax1
    ax2:plt.Axes = ax2
    ax1.grid(True)
    ax2.grid(True)
    subx = 0
    superx = 0
    spacing = 0.25
    max_val = max([ max(v) for v in timDict.values() ])
    min_val = min([ min(v) for v in timDict.values() ])
    foundNames = {}
    
    legElems = [Patch(color='blue', label='Alone'), Patch(color='red', label='Combined')]
    
    for k,v in timDict.items():
        bprops = {}
        int_scheme, speed, thread = k.split("_")
        xpos = None
        col = None
        med_props = dict(linewidth=1)
        plt_ax = None
        if speed == "supersonic":
            plt_ax = ax2
            superx -=- spacing
            xpos = superx
        else:
            plt_ax = ax1
            subx -=- spacing
            xpos = subx
            
        if thread == "combined":
            med_props['color'] = 'blue'
            col = 'red'
        else:
            med_props['color'] = 'magenta'
            col = 'blue'
        
        bp = plt_ax.boxplot(v, positions=[xpos], patch_artist=True, labels=[''], medianprops=med_props)
        plt.setp(bp['fliers'], marker="x")
        if " ".join([int_scheme, speed]) in foundNames.keys():
            otherVal = foundNames[" ".join([int_scheme, speed])]
            textXpos = (xpos + otherVal["xpos"])/2
            textYpos = max(max(v),otherVal["max"]) + 0.05
            txt = int_scheme
            if any([char.isdigit() for char in txt]):
                txt = txt.upper()
            else:
                txt = txt.capitalize()
            plt_ax.text(textXpos,textYpos,txt, ha="center", va="bottom")
        else:
            foundNames[" ".join([int_scheme, speed])] = {"xpos": xpos, "max": max(v)}
        for b in bp["boxes"]:
            b.set_facecolor(col)
    for ax in (ax1, ax2):
        curr = ax.get_ylim()
        ax.set_ylim( curr[0]-0.05, curr[1]+0.15  )
    ax1.set_ylabel(r"Time ($s$)")
    ax1.set_title("Subsonic")
    ax2.set_title("Supersonic")

    ax1.set_xticks([])
    ax2.set_xticks([])
    ax1.set_yticks(np.arange(0.1,1.01,0.1))
    #ax1.set_yticklabels([str(i) for i in np.arange(0.1,1,0.1)])
    ax2.set_yticks(np.arange(0.7,1.71,0.1))
    #ax2.set_yticklabels([str(i) for i in np.arange(0.7,1.7,0.1)])
    leg = fig.legend(handles=legElems, loc='upper center', bbox_to_anchor=(0.5, 0.05), ncol=2)
    leg.set_in_layout(False)
    leg.set_loc("lower center")
    fig.tight_layout(rect=[0, 0.1, 1, 1])
    plt.show()

if __name__ == "__main__":
    main()