#Script for Blender. Write 2D-3D correspondence in a file

import bpy
import numpy as np
import math
import mathutils

cam_name = 'Camera.001'
img_name = 'IMG_8856.JPG'
frame_nb = 0
p3d_name = 'virt'
width=40
height=30
aspect=height/width
file="export2.txt"


def rotationMatrixToEulerAngles(R):
    x = math.atan2(R[2,1] , R[2,2])
    y = math.atan2(-R[2,0], math.sqrt(R[2,1]**2+R[2,2]**2))
    z = math.atan2(R[1,0],R[0,0])
    return (-x, -y, z+180*math.pi/180)

#camera used
cam = 0
camera=bpy.data.cameras[cam_name]#=cam.data
aspect=camera.sensor_height/camera.sensor_width

p3d = []
p2d = []
#get 3D points
for ob in bpy.data.objects:
    if ob.name[:len(p3d_name)]==p3d_name:
        print('empty {} at {}'.format(ob.name, ob.location))
        p3d.append(ob.location)
    if ob.name==cam_name:
        cam=ob


# get 2D points
track_data = bpy.data.movieclips[img_name].tracking
for t in track_data.tracks:
    for m in t.markers:
        if (m.frame==frame_nb):
            print('marker {} at {}'.format(t.name, m.co))
            p2d.append(mathutils.Vector([(2.0*m.co[0]-1.0),aspect*(2.0*m.co[1]-1.0)]))

p3d=np.array(p3d)
p2d=np.array(p2d)

if(2*p3d.size != 3*p2d.size):
    print("Put the same number of empty and markers")
    
n = int(p3d.size/3)

A=[]
#Compute A1i, A2i and A=[A11,A21,...,A1n,A2n]
for i in range(0,n):
    A1=np.array([p3d[i,0],p3d[i,1],p3d[i,2],1,0,0,0,0,-p3d[i,0]*p2d[i,0],-p3d[i,1]*p2d[i,0],-p3d[i,2]*p2d[i,0],-p2d[i,0]])
    A2=np.array([0,0,0,0,p3d[i,0],p3d[i,1],p3d[i,2],1,-p3d[i,0]*p2d[i,1],-p3d[i,1]*p2d[i,1],-p3d[i,2]*p2d[i,1],-p2d[i,1]])
    A.append(A1)
    A.append(A2)

A=np.array(A)

fichier = open(file, "w")
fichier.write(str(n))
fichier.write('\n')

for i in range(0,n):
    for j in range(0,3):
        fichier.write(str(p3d[i,j]))
        fichier.write(" ")
    fichier.write('\n')

for i in range(0,n):
    for j in range(0,2):
        fichier.write(str(p2d[i,j]))
        fichier.write(" ")
    fichier.write('\n')
        
fichier.close()
