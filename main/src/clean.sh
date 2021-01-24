ps -ef | grep camera | grep -v grep | awk '{print $2}' | xargs kill -9
sleep 1
ps -ef | grep stereo | grep -v grep | awk '{print $2}' | xargs kill -9
sleep 1
ps -ef | grep gpsreader | grep -v grep | awk '{print $2}' | xargs kill -9
sleep 1
ps -ef | grep comm_tcp | grep -v grep | awk '{print $2}' | xargs kill -9
sleep 1
ps -ef | grep serialPort | grep -v grep | awk '{print $2}' | xargs kill -9
sleep 1
ps -ef | grep TCPMiddleWare | grep -v grep | awk '{print $2}' | xargs kill -9
sleep 1
ps -ef | grep SPMiddleWare | grep -v grep | awk '{print $2}' | xargs kill -9
sleep 1
ps -ef | grep detObs | grep -v grep | awk '{print $2}' | xargs kill -9
sleep 1
