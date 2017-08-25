This project contains tools for 3D reconstruction of specular surface

The python scripts are for Blender:

-PNP.py helps localizing an image in a 3D environment, thanks to correspondences given between the image and the 3D scene.
	2D points are markers placed on the image via the movie clip editor.
	3D points are empties placed on the mesh.

-plane.py compute a symmetry plane between two objects.

-export.py exports 2D and 3D correspondences to a file
-exportcam.py exports the camera paramters to a file

-calib.cpp takes in input the data obtained with export.py and computes the distortion of the image. it also undistort the image, and distort another image the same way, to check the result.

-reconstruct.cpp takes in input such a distortion, and the data of two cameras exported with exportcam.py, and reconstructs a point cloud corresponding to a mirror surface seen from camera 1 and which reflection make the distortion given in input. 
