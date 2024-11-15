clear rpi;
rpi = raspberrypi(CAR_IP, 'pi', 'LabControl');
stopModel(rpi,'CAR_CONTROL_SYSTEM')
while isModelRunning(rpi,'CAR_CONTROL_SYSTEM')
end
rpi.system('sudo shutdown -h now');
clear rpi;