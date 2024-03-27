# ncat12_jackal

A repository for the NCAT12 Jackal project. The robot has a Kinova Gen 2 (Jaco 2) arm and Velodyne HDL-32e lidar.

## Environment Variables
Environment variables are set on start-up through the `/etc/ros/setup.bash`.

### Kinova Jaco2
The Kinova Jaco2 arm needs to be rotated by 90 degrees about the Z-axis. This is done by setting the following environment variable: 

```export JACKAL_KINOVA_RPY="0 0 1.5707" ```

### Velodyne HDL-32E
The Velodyne HDL-32E can be mounted on a tower. If so, reflect this change in the URDF by setting the following environment variable: 

```export JACKAL_LASER_3D_TOWER=1```


