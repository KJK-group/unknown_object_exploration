#!/usr/bin/env python3
import sys
import airsim
import os


def usage():
    print("./airsim_generate_voxelgrid_octomap_of_environment.py <map file>")
    print("ref: https://microsoft.github.io/AirSim/voxel_grid/")


if len(sys.argv) == 1:
    usage()
    exit(1)


file = sys.argv[1]

client = airsim.VehicleClient()
center = airsim.Vector3r(0, 0, 0)

output_path = os.path.join(os.getcwd(), f"{file}.binvox")

x = 125
y = 125
z = 125
resolution = 0.5

client.simCreateVoxelGrid(center, x, y, z, resolution, output_path)
print(f"write voxelgrid to {output_path} with resolution {resolution}")

print(f"use: binvox2bt {file}.binvox to convert to octomap .bt format")
