import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def drawSphere(ax,pos,r,c):
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = pos[0] + r * np.outer(np.cos(u), np.sin(v))
    y = pos[1] + r * np.outer(np.sin(u), np.sin(v))
    z = pos[2] + r * np.outer(np.ones(np.size(u)), np.cos(v))

    ax.plot_surface(x, y, z,  rstride=4, cstride=4, color=c, linewidth=0, alpha=0.5)

def putText(ax, pos, text):
    ax.text(pos[0], pos[1], pos[2]+0.2, text, size=10, zorder=1, color='k')

