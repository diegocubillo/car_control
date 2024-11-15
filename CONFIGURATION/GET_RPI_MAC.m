clc
clear rpi
rpi = raspberrypi('192.168.0.108','pi','raspberry');
cmdout = rpi.system('cat /sys/class/net/wlan0/address');
disp(cmdout)