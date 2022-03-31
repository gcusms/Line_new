#！/bin/bash
# sudo chmod 777 /dev/ros_tty
cd build
rm -f Robot_wolf
sudo make
while true
do
./Robot_wolf
done
