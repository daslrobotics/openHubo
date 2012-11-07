sudo rmmod vcan
sudo modprobe vcan
sudo ip link add type vcan
sudo ifconfig vcan0 up
sudo ip link add type vcan
sudo ifconfig vcan1 up
sudo ip link add type vcan
sudo ifconfig vcan2 up
sudo ip link add type vcan
sudo ifconfig vcan3 up
sudo ach -U hubo-ref
ach -C hubo-ref -m 10 -n 3000
sudo ach -U hubo-state
ach -C hubo-state -m 10 -n 3000
sudo ach -U hubo-init-cmd
ach -C hubo-init-cmd -m 10 -n 3000
sudo ach -U hubo-param
ach -C hubo-param -m 10 -n 3000
#TODO: make these install to /usr/local/lib?
sudo ~/hubo-ach/hubo-default
sudo ~/hubo-ach/hubo-main -v
