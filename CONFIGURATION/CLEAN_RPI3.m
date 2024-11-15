clear rpi;
rpi = raspberrypi(CAR_IP, 'pi', 'LabControl');
rpi.system(['rm /home/pi/TEST_MAPA* -f']);
rpi.system(['rm /home/pi/CAR_CONTROL_SYSTEM_* -f']);









 














rpi = raspberrypi(CAR_IP, 'pi', 'raspberry');
rpi.stopModel('CAR_CONTROL_SYSTEM')
pause(2)
rpi.system('sudo shutdown -h now');
clear rpi;
