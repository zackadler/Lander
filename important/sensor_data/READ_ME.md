We have the sensor data and the code to collect the data. Now what we need is to laern and understand the code that Josh made for the servos and the code made for propellors. 
Then integrate some sort of control to get something for hover.



Lidar and Optical Flow come from the same module hence they are coming from the same script/code.
IMU is seperate from the lidar/optical flow sensor. 

Lidar:
Distance q reflects how strong the laser’s returning signal is.
The data returned from the lidar sensor is this:
            "distance_m": self._distance_m,
            "rangefinder_quality": self._rangefinder_quality,

Optical Flow:
Flow q reflects how well the camera can pick out trackable detail on the surface below.
The data returned from the optical flow sensor is this:
            "motion_x": self._motion_x,
            "motion_y": self._motion_y,
            "flow_quality": self._flow_quality,

Each flow number is how fast the ground appears to slide past the camera along one axis, x first and then y. It’s a speed-like reading, not a distance. The sign is the direction. The size grows with speed and shrinks with height, because the ground appears to move less the higher you are.

Need to figure out the direction and compare it to the IMU. If we want to use this module, then we need to figure out how to reliably convert the data into lateral speed or something idk.

IMU: 
The data returned for the IMU is this:
            "accel_x": ax,
            "accel_y": ay,
            "accel_z": az,
            "roll": roll,
            "pitch": pitch,
