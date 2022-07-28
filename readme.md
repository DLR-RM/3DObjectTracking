# 3D Object Tracking

Tracking a rigid object in 3D space and determining its 6DoF pose is an essential task in computer vision. Its application ranges from augmented reality to robotic perception. Given consecutive image frames and a 3D model of the object, the goal is to robustly estimate both the rotation and translation of a known object relative to the camera. While the problem has been thoroughly studied, many challenges such as partial occlusions, appearance changes, motion blur, background clutter, and real-time requirements still exist.

In this repository, we will continuously publish algorithms and code of our ongoing research on 3D object tracking. The folders for the different algorithms include everything necessary to reproduce results presented in our papers. Note that the code for each new paper also includes an updated version of previous work. If you want to use our tracker in your own project or application, please use the code from the latest publication. Currently, the latest version of our code can be found in the folder [__ICG__](https://github.com/DLR-RM/3DObjectTracking/tree/master/ICG).

The algorithms corresponding to the following papers are included:
* [__RBGT__](https://github.com/DLR-RM/3DObjectTracking/tree/master/RBGT)
Stoiber M, Pfanne M, Strobl KH, Triebel R, Albu-Schäffer A (2020) A Sparse Gaussian Approach to Region-Based 6DoF Object Tracking. In: Asian Conference on Computer Vision
* [__SRT3D__](https://github.com/DLR-RM/3DObjectTracking/tree/master/SRT3D)
Stoiber M, Pfanne M, Strobl KH, Triebel R, Albu-Schäffer A (2022) SRT3D: A Sparse Region-Based 3D Object Tracking Approach for the Real World. In: International Journal of Computer Vision
* [__ICG__](https://github.com/DLR-RM/3DObjectTracking/tree/master/ICG)
Stoiber M, Sundermeyer M, Triebel R (2022) Iterative Corresponding Geometry: Fusing Region and Depth for Highly Efficient 3D Tracking of Textureless Objects. In: Conference on Computer Vision and Pattern Recognition
* [__Mb-ICG__](https://github.com/DLR-RM/3DObjectTracking/tree/master/Mb-ICG)
Stoiber M, Sundermeyer M, Boerdijk W, Triebel R (2022) A Multi-body Tracking Framework - From Rigid Objects to Kinematic Structures

