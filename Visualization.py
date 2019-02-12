import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection


def region_plot(regions, flag, ax):
    """
    plot the workspace
    :param regions: regions
    :param flag: regions or obstacle
    :param ax: figure axis
    :param d: 2D or 3D
    :return: none
    """

    ax.set_xlim((0, 1))
    ax.set_ylim((0, 1))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(b=True, which='major', color='k', linestyle='--')
    for key in regions.keys():
        coords = list(regions[key].exterior.coords)
        if len(coords)>10:
            color = '0.75' if flag != 'region' else 'c'
            circle = plt.Circle(regions[key].centroid.coords[0],  np.fabs(coords[0][0] - regions[key].centroid.coords[0][0]), color=color, fill=(flag!='region'))
            ax.add_artist(circle)
            ax.text(regions[key].centroid.x, regions[key].centroid.y, r'${}_{}$'.format(key[0], key[1:]), fontsize=16)
        else:
            color = '0.75' if flag != 'region' else 'c'
            x = []
            y = []
            patches = []
            for point in list(regions[key].exterior.coords)[:-1]:
                x.append(point[0])
                y.append(point[1])
            polygon = Polygon(np.column_stack((x, y)), True)
            patches.append(polygon)
            p = PatchCollection(patches, facecolors=color, edgecolors=color)
            ax.add_collection(p)
            ax.text(np.mean(x), np.mean(y), r'${}_{}$'.format(key[0], key[1:]), fontsize=16)


def path_plot(path, regions, obs):
    """
    plot the optimal path in the 2D and 3D
    :param path: ([pre_path], [suf_path])
    :param regions: regions
    :param obs: obstacle
    :return: none
    """

    ax = plt.figure(1).gca()
    region_plot(regions, 'region', ax)
    region_plot(obs, 'obs', ax)

    # prefix path
    x_pre = np.asarray([point[0][0] for point in path])
    y_pre = np.asarray([point[0][1] for point in path])
    pre = plt.quiver(x_pre[:-1], y_pre[:-1], x_pre[1:] - x_pre[:-1], y_pre[1:] - y_pre[:-1], color='r',
                     scale_units='xy', angles='xy', scale=1, label=' path')

    plt.legend(handles=[pre])
    plt.savefig('path.png', bbox_inches='tight', dpi=600)
    plt.show()