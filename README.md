# 2D-3D-Image-Triangualtion
A program to track the joints of a human body in 3D space through two motion capture videos.

This program implements a forward (3D point to 2D point) and inverse (2D point to 3D ray) camera projection and performs triangulation from two cameras to do 3D reconstruction from pairs of matching 2D image points. The program handles 12 joints over ~30,000 frames of data being projected into 2 separate camera coordinate systems, making it over 700,000 joint projections into the camera view and approximately 350,000 reconstructions back into world coordinate system. The program also measures error between the reconstruction and the original input. 

Both quantitavie and qualitative results have been included. 
