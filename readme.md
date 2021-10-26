# 3D Object Tracking

Tracking a rigid object in 3D space and determining its 6DoF pose is an essential task in computer vision. Its application ranges from augmented reality to robotic perception. Given consecutive image frames and a 3D model of the object, the goal is to robustly estimate both the rotation and translation of a known object relative to the camera. While the problem has been thoroughly studied, many challenges such as partial occlusions, appearance changes, motion blur, background clutter, and real-time requirements still exist.

In this repository, we will continuously publish algorithms and code of our ongoing research on 3D object tracking. The folders for the different algorithms include everything necessary to reproduce results presented in our papers and to support full reusability in different projects and applications.

The algorithms corresponding to the following papers are included:
* [__RBGT__](https://github.com/DLR-RM/3DObjectTracking/tree/master/RBGT)
Stoiber M, Pfanne M, Strobl KH, Triebel R, Albu-Schäffer A (2020) A Sparse Gaussian Approach to Region-Based 6DoF Object Tracking. In: Asian Conference on Computer Vision
* [__SRT3D__](https://github.com/DLR-RM/3DObjectTracking/tree/master/SRT3D)
Stoiber M, Pfanne M, Strobl KH, Triebel R, Albu-Schäffer A (2021) SRT3D: A Sparse Region-Based 3D Object Tracking Approach for the Real World


