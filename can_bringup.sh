sudo slcand -o -c -s0 /dev/ttyACM0 can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000