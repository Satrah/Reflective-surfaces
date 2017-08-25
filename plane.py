#Script for Blender. Computer a symmetry plane between two cameras

import bpy
import numpy as np
import math
import mathutils

#write here the name of the two objects that have a symmetry plane
cam1_name = 'Camera'
cam2_name = 'Camera.001'
#camera1=bpy.data.cameras[cam1_name]
#camera2=bpy.data.cameras[cam2_name]
cam1=bpy.data.objects[cam1_name]
cam2=bpy.data.objects[cam2_name]

def rotationMatrixToEulerAngles(R):
    x = math.atan2(R[2,1] , R[2,2])
    y = math.atan2(-R[2,0], math.sqrt(R[2,1]**2+R[2,2]**2))
    z = math.atan2(R[1,0],R[0,0])
    return (-x, -y, z+180*math.pi/180)

loc = (cam1.location+cam2.location)/2
norm = (cam1.location-cam2.location).normalized()
x=(norm.cross([1,0,0])).normalized()
y=(norm.cross(x)).normalized()

R=np.array([x,y,norm])
rot=rotationMatrixToEulerAngles(R.transpose())

bpy.ops.mesh.primitive_plane_add(radius=8,location=loc, rotation=rot)
