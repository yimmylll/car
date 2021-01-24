roslaunch lexiao_camera camera.launch &
rosrun lexiao_stereo stereo &
# sudo chmod 777 /dev/ttyUSB0
# sudo chmod 777 /dev/ttyTHS2
rosrun comm_tcp client_node $1 $2 &
rosrun serialPort serialPort_node &
rosrun gpsreader GPSPub.py &
rosrun CSMiddleWare SPMiddleWare &
rosrun CSMiddleWare TCPMiddleWare &
rosrun detObs detObs.py &
