segment the object
6DOF arm 0 Rgbd camera - tooltip 

fundametal point cloud processing 
- RGBD - 3D view of the scene
- identify major plane
- using major plane segment the object

emphaszing on collecting more data for grasping
- not many literature do this, they just assume

 there are different methods to do this
 to simplify
 - segment , move and 
 - libraries for normal vectors on the project
	 - using this can we design a grasp
 - find pair of points to check grasp
 - parallel jaw gripper
	 - use force balance formulation
- view sphere 

object - 
dataset YCB 

spawn 


PCL library
- filters
- segmentation
- octree or kdtree

![[Pasted image 20221004172246.png]]

- setting up environment - all 
- point cloud library learning - all
- filtering - 
- downsampling - 
- stitching the point cloud - do we really need it?  (bonus)
- segementation  - 
- get normals -
- spawn objects -
	- sphere on a table
	- cuboid on a table
	- glass vase on a table
- grasp calculation -
	find optimal grasp
	moving the camera 
	how do we assign tangential axis once we get the normal vectors
	probably do it in another node
	
- move camera optimally (optional)
- move robot to grasp (optional)

