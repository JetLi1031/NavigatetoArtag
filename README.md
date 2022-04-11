
robot navigation to AR tag

# NavigatetoArtag
robot navigation toward charging dock 

Battery publisher with service client avaiable to initiate docking process
1) the robot navigate toward a given goal by calling sending action request
2) the pose of robot and ar_tag were recorded by multihreaded subsription client 
3) rotate until the transformd pubilshed between ar_tag and camera frame available
4) start aligniment by compensating heading error,yaw difference.
5) Stop progress and change state to charging condition
![alt text](https://github.com/JetLi1031/NavigatetoArtag/blob/main/output.gif "robot navigate output")

