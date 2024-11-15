clear rpi;
rpi = raspberrypi(CAR_IP, 'pi', 'LabControl');
stopModel(rpi,'CAR_CONTROL_SYSTEM')
while isModelRunning(rpi,'CAR_CONTROL_SYSTEM')
end
runModel(rpi,'CAR_CONTROL_SYSTEM')
clear rpi;
count = 0;
while count<5
    pause(1)
    count = count+1;
    clc
    fprintf('TIMER = %d seconds',floor(count))
end