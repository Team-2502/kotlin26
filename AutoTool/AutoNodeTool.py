#!/usr/bin/env python3
import argparse
import csv
import pathlib

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from PIL import Image

graph = {
    # Blue
    "A": ((1.673, 2.582), ["B", "F", "C", "E"]),
    "B": ((3.462, 0.644), []),
    "C": ((1.673, 4.909), ["B", "E"]),
    "D": ((1.253, 7.063), ["B", "C", "E", "F"]),
    "E": ((3.462, 7.426), []),
    "F": ((3.462, 4.035), ["B", "C", "E"]),
    # Middle
    "G": ((5.789, 7.426), ["E", "J", "I"]),
    "H": ((5.789, 0.644), ["B", "G", "J", "I"]),
    "I": ((10.752, 0.644), ["J", "M"]),
    "J": ((10.752, 7.426), ["K"]),
    # Red
    "K": ((13.079, 7.426), ["N", "L", "O", "P"]),
    "L": ((13.079, 4.035), ["N", "P"]),
    "M": ((13.079, 0.644), ["L", "P", "N"]),
    "N": ((14.868, 5.487), []),
    "O": ((14.868, 3.160), ["K", "L", "M", "N", "P"]),
    "P": ((15.288, 1.006), []),
}
for k, v in graph.items():
    other_nodes = v[1]
    for node in other_nodes:
        assert k not in graph[node]
robot_edge = 34.65 / 12 / 3.28084
ramp_half_width = 0.6
keepout = [
    [
        (4.029, 6.782),
        (5.222, 6.782),
        (5.222, 1.287),
        (4.029, 1.287),
    ],
    [
        (0.0, 6.497),
        (0.0, 5.430),
        (0.686, 5.430),
        (0.686, 6.497),
    ],
    [
        (0.0, 4.343),
        (0.0, 3.149),
        (1.106, 3.149),
        (1.106, 4.343),
    ],
    [
        (11.318, 6.782),
        (12.512, 6.782),
        (12.512, 1.287),
        (11.318, 1.287),
    ],
    [
        (15.435, 4.921),
        (15.435, 3.727),
        (16.541, 3.727),
        (16.541, 4.921),
    ],
    [
        (15.855, 2.639),
        (15.855, 1.573),
        (16.541, 1.573),
        (16.541, 2.639),
    ]
]


def clip_half_plane(polygon, normal, limit):
    if len(polygon) == 0:
        return polygon
    output = []
    previous = polygon[-1]
    prev_dist = np.dot(normal, previous) - limit
    for current in polygon:
        curr_dist = np.dot(normal, current) - limit
        prev_inside = prev_dist <= 1e-9
        curr_inside = curr_dist <= 1e-9
        if prev_inside != curr_inside:
            denom = prev_dist - curr_dist
            t = prev_dist / denom
            output.append(previous + t * (current - previous))
        if curr_inside:
            output.append(current)
        previous, prev_dist = current, curr_dist
    return np.array(output, dtype=float).reshape(-1, 2)

def generateKotlinCode():
    for k, v in graph.items():
        print(k + "(Translation2d(" + str(v[0][0]) + ", " + str(v[0][1]) + "), Rotation2d(PI)),")
    print()
    for k, v in graph.items():
        if len(v[1])>= 1:
            print(k + ".connectTo(" + ", ".join(v[1]) + ")")

def draw(image, graph_points, keepout_points, alpha=0.23):
    width, height = image.size
    fig, ax = plt.subplots(figsize=(16, 16 * height / width), dpi=70)
    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    ax.imshow(image, extent=(0, width, height, 0), origin="upper")
    for label, ((x, y), connected) in graph_points.items():
        ax.scatter(x, y, s=68, c="black", edgecolors="black", linewidths=1.4, zorder=5)
        ax.annotate(label, (x, y), xytext=(9, -9),
                    textcoords="offset points", fontsize=11, weight="bold",
                    color="black", bbox=dict(facecolor="white", alpha=0.8,
                                             edgecolor="none", pad=2), zorder=6)
        for other_node in connected:
            (dx, dy), _ = graph_points[other_node]
            ax.plot(
                [x, dx],
                [y, dy],
                color="black",
                linewidth=4,
                linestyle="-",
                zorder=10
            )
    for poly in keepout_points:
        for i in range(len(poly)):
            (x1, y1) = poly[i];
            (x2, y2) = poly[(i + 1) % len(poly)]
            ax.plot(
                [x1, x2],
                [y1, y2],
                color="red",
                linewidth=4,
                linestyle="-",
                zorder=10
            )
    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.set_aspect("equal")
    ax.axis("off")
    plt.show()


def main():
#     image = Image.open(pathlib.Path(__file__).parent / "image.png").convert("RGB")
#     keepout_points = []
#     graph_points = dict()
#     field_width = 650.12 / 12 / 3.28084
#     field_height = 316.64 / 12 / 3.28084
#     for points in keepout:
#         poly = []
#         for (x, y) in points:
#             new_loc = (1.0 - x / field_width) * image.width, y / field_height * image.height
#             poly.append(new_loc)
#         keepout_points.append(poly)
#     for k, ((x, y), connected) in graph.items():
#         new_loc = (1.0 - x / field_width) * image.width, y / field_height * image.height
#         graph_points[k] = (new_loc, connected)
#     draw(image, graph_points, keepout_points, 0.23)
    generateKotlinCode()


if __name__ == "__main__":
    main()