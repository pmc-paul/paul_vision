# paul_vision
package du module de vision

## installation

Installer le wrapper Realsense ROS pour la caméra:

https://github.com/IntelRealSense/realsense-ros

Pour les librairies Python
```
pip install -r requirements.txt
```

## Launch

Pour lancer la caméra
```
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true
```
