sudo ifconfig can0 down
./canableStart.sh
sudo stty -F /dev/ttyUSB3 19200
bin/main
