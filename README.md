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



## mesures de la caméra

range d'image couleur
distance en z 29 cm de l'étagère:
x: 34 cm
y: 21 cm

classification:
z minimum: 10 cm
z maximum: 27 cm

segmentation:
pour les petites cannes: minimum 75 cm, maximum 150 cm
pour les grosses bouteilles de vinaigrette hellmans: minimum 90 cm
