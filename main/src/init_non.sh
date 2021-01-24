source /home/mman/.bashrc
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

export OPENNI2_INCLUDE=/home/mman/AXonOpenNI-Linux-Arm-2.3/Include
export OPENNI2_REDIST=/home/mman/AXonOpenNI-Linux-Arm-2.3/Redist

roslaunch lexiao_camera camera.launch &
rosrun lexiao_stereo stereo &
# sudo chmod 777 /dev/ttyUSB0
# sudo chmod 777 /dev/ttyTHS2
rosrun comm_tcp client_node $1 $2 &
rosrun serialPort serialPort_node &
rosrun gpsreader GPSPub.py &
rosrun CSMiddleWare SPMiddleWare &
rosrun CSMiddleWare TCPMiddleWare &
# rosrun detObs detObs.py &
sleep 20
rosrun main main
