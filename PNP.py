#Script for Blender. Localise a camera in a scene.

import bpy
import numpy as np
import math
import mathutils

#write here the name of the camera to localize
cam_name = 'Camera.001'
#write here the name of the image to localize
img_name = 'IMG_9110mirror.JPG'
frame_nb = 1
#all the empties you place should have a name starting by the following letters
p3d_name = 'virt'
width=40
height=30
aspect=height/width


def rotationMatrixToEulerAngles(R):
    x = math.atan2(R[2,1] , R[2,2])
    y = math.atan2(-R[2,0], math.sqrt(R[2,1]**2+R[2,2]**2))
    z = math.atan2(R[1,0],R[0,0])
    return (x, y, z)#+180*math.pi/180)

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
            #p2d.append(2.0*m.co-mathutils.Vector([1.0,1.0]))
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
print(A)
u,s,v=np.linalg.svd(A)

X=[]
for i in range(0,12):
    X.append(v[11,i])
    
X=np.array(X)

#estimated projection (normalized)
P=np.array([[X[0],X[1],X[2],X[3]],[X[4],X[5],X[6],X[7]],[X[8],X[9],X[10],X[11]]])
#K*R (normalised)
R1=np.array([[X[0],X[1],X[2]],[X[4],X[5],X[6]],[X[8],X[9],X[10]]])
#translation (normalised)
t=np.array([X[3],X[7],X[11]])

#RQ decomposition: R->K Q->Rot
q,r=np.linalg.qr(np.linalg.inv(R1))
q1=np.linalg.inv(q)
r1=np.linalg.inv(r)

#scale translation
for i in range(0,3):
    t[i]/=r1[i,i]

cam.location=-np.matrix(q)*np.transpose(np.matrix(t))
cam.rotation_euler=rotationMatrixToEulerAngles(q)

#estimate focal length
f=0
for i in range(0,n):
    a=np.matrix(q1)*np.transpose(np.matrix(p3d[i]))+np.transpose(np.matrix(t))
    b=p2d[i]
    f+=(32/2*b[0]/(a[0]/a[2]))+(32/2*b[1]/(a[1]/a[2]))
f/=(2*n)
if f < 0:
    f=-f

camera.lens=f
