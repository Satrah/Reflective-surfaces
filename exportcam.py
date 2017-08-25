#Script for blender. Writes camera parameters in a file
import bpy
import numpy as np
import math
import mathutils

cam_name = 'Camera.001'
file="results/exportcamvirt.txt"


#camera used
cam = 0
camera=bpy.data.cameras[cam_name]#=cam.data
aspect=camera.sensor_height/camera.sensor_width
cam=bpy.data.objects[cam_name]


cam.location
cam.matrix_world
camera.lens

fichier = open(file, "w")

for j in range(0,3):
    fichier.write(str(cam.location[j]))
    fichier.write(" ")
fichier.write('\n')

for i in range(0,3):
    for j in range(0,3):
        fichier.write(str(cam.matrix_world[i][j]))
        fichier.write(" ")
    fichier.write('\n')
        
fichier.write(str(camera.lens))
fichier.close()
