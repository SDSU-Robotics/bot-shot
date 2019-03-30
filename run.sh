sudo ifconfig can0 down
./canableStart.sh
sudo stty -F /dev/ttyUSB1 19200
bin/main
