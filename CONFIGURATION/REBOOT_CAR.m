rpi = raspberrypi(CAR_IP, 'pi', 'LabControl');
rpi.stopModel('CAR_CONTROL_SYSTEM')
pause(2)
rpi.system('sudo shutdown -r now');
clear rpi;