# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json
from wpimath.geometry import *
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
import argparse

# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument("trajectory", help='The ".traj" file to analyze')
parser.add_argument("--layout", default="2025-reefscape.json", help="The AprilTag field layout JSON")
parser.add_argument("--slice", type=float, default=10, help="The size of each slice in degrees")
args = parser.parse_args()

# Load AprilTag poses
apriltags = json.loads(open(args.layout).read())
tag_poses: list[Pose2d] = []
for tag_json in apriltags["tags"]:
    translation_3d = Translation3d(
        tag_json["pose"]["translation"]["x"], tag_json["pose"]["translation"]["y"], tag_json["pose"]["translation"]["z"]
    )
    rotation_3d = Rotation3d(
        Quaternion(
            tag_json["pose"]["rotation"]["quaternion"]["W"],
            tag_json["pose"]["rotation"]["quaternion"]["X"],
            tag_json["pose"]["rotation"]["quaternion"]["Y"],
            tag_json["pose"]["rotation"]["quaternion"]["Z"],
        )
    )
    tag_poses.append(Pose3d(translation_3d, rotation_3d).toPose2d())

# Initialize results
total_tag_secs: list[float] = []
for i in range(math.floor(math.radians(360) / math.radians(args.slice))):
    total_tag_secs.append(0)

# Load trajectory
traj_json = json.loads(open(args.trajectory).read())
last_time = 0
for sample_json in traj_json["trajectory"]["samples"]:
    sample_pose = Pose2d(sample_json["x"], sample_json["y"], Rotation2d(sample_json["heading"]))
    dt = sample_json["t"] - last_time
    last_time = sample_json["t"]

    # Check each tag
    for tag_index, tag_pose in enumerate(tag_poses):
        robot_to_tag_rotation = (tag_pose.translation() - sample_pose.translation()).angle()

        # Skip if facing wrong direction
        rotation_diff = tag_pose.rotation() - robot_to_tag_rotation
        if abs(rotation_diff.radians()) < math.radians(90):
            continue

        # Add to results
        rotation_value = (robot_to_tag_rotation - sample_pose.rotation()).radians()
        if rotation_value < 0:
            rotation_value += math.pi * 2
        result_index = math.floor(rotation_value / math.radians(args.slice))
        if result_index < len(total_tag_secs):
            total_tag_secs[result_index] += dt

# Initialize plot
center = (0, 0)
radius = 5
fig, ax = plt.subplots()
circle = Circle(center, radius)
ax.add_patch(circle)

# Add wedges
max_tag_secs = max(total_tag_secs)
for i in range(len(total_tag_secs)):
    start_angle = i * args.slice + 90
    end_angle = (i + 1) * args.slice + 90

    shade = total_tag_secs[i] / max_tag_secs
    color = (0, shade, shade)
    wedge = Wedge(center, radius, start_angle, end_angle, color=color)
    ax.add_patch(wedge)

# Set the limits and aspect ratio of the plot
ax.set_xlim(-radius * 1.2, radius * 1.2)
ax.set_ylim(-radius * 1.2, radius * 1.2)
ax.set_aspect("equal")

# Show the plot
plt.show()
