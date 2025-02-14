stopModel(rpi,'CAR_CONTROL_SYSTEM')
while isModelRunning(rpi,'CAR_CONTROL_SYSTEM')
end
getFile(rpi,'/home/pi/SCOPE_PC.mat')
% getFile(rpi,'/home/pi/SCOPE_HW.mat')
getFile(rpi,'/home/pi/SCOPE_COMP_FV.mat')
% getFile(rpi,'/home/pi/CPU_LOAD.mat')
getFile(rpi,'/home/pi/SCOPE_NAV.mat')
% Load .mat files
load SCOPE_PC.mat
% load SCOPE_HW.mat
load SCOPE_COMP_FV.mat
load SCOPE_NAV.mat
% Transform to structs
%------------------------------------------------------------------
% SCOPE_PC
STRUCT_PC = struct('time',SCOPE_PC(1,:)');
STRUCT_PC.signals(1).title = 'FORWARD VELOCITY (m/s)';
STRUCT_PC.signals(1).values = SCOPE_PC(2:3,:)';
STRUCT_PC.signals(2).title = 'YAW RATE (rad/s)';
STRUCT_PC.signals(2).values = SCOPE_PC(4:5,:)';
STRUCT_PC.signals(3).title = 'YAW ANGLE (deg)';
STRUCT_PC.signals(3).values = SCOPE_PC(6:7,:)';
STRUCT_PC.signals(4).title = 'WALL DIST (m)';
STRUCT_PC.signals(4).values = SCOPE_PC(8:9,:)';
STRUCT_PC.signals(5).title = 'MOTOR VOLTAGE (V)';
STRUCT_PC.signals(5).values = SCOPE_PC(10:14,:)';
STRUCT_PC.signals(6).title = 'MOTOR VOLTAGE (V)';
STRUCT_PC.signals(6).values = SCOPE_PC(15:18,:)';
STRUCT_PC.signals(7).title = 'POSITION XY (m)';
STRUCT_PC.signals(7).values = SCOPE_PC(19:22,:)';
STRUCT_PC.signals(8).title = 'VELOCITY XY (m)';
STRUCT_PC.signals(8).values = SCOPE_PC(23:26,:)';
STRUCT_PC.signals(9).title = 'MOTOR RATE (rad/s)';
STRUCT_PC.signals(9).values = SCOPE_PC(27:28,:)';
STRUCT_PC.signals(10).title = 'MOTOR RATE (rad/s)';
STRUCT_PC.signals(10).values = SCOPE_PC(29:30,:)';
STRUCT_PC.signals(11).title = 'PITCH RATE (rad/s)';
STRUCT_PC.signals(11).values = SCOPE_PC(31,:)';
STRUCT_PC.signals(12).title = 'PITCH ANGLE (deg)';
STRUCT_PC.signals(12).values = SCOPE_PC(32:33,:)';
SCOPE_PC = STRUCT_PC;
save SCOPE_PC SCOPE_PC
%------------------------------------------------------------------
% % SCOPE_HW
% STRUCT_HW = struct('time',SCOPE_HW(1,:)');
% STRUCT_HW.signals(1).title = 'IMU ACCEL (m/s^2)';
% STRUCT_HW.signals(1).values = SCOPE_HW(2:9,:)';
% STRUCT_HW.signals(2).title = 'IMU GYRO & EULER RATES (rad/s)';
% STRUCT_HW.signals(2).values = SCOPE_HW(10:21,:)';
% STRUCT_HW.signals(3).title = 'ENCODER FORWARD VELOCITY (m/s)';
% STRUCT_HW.signals(3).values = SCOPE_HW(22:23,:)';
% STRUCT_HW.signals(4).title = 'PITCH ANGLE (deg)';
% STRUCT_HW.signals(4).values = SCOPE_HW(24:26,:)';
% STRUCT_HW.signals(5).title = 'YAW ANGLE (deg)';
% STRUCT_HW.signals(5).values = SCOPE_HW(27:34,:)';
% STRUCT_HW.signals(6).title = 'IMU PITCH RATE (rad/s)';
% STRUCT_HW.signals(6).values = SCOPE_HW(35:36,:)';
% STRUCT_HW.signals(7).title = 'YAW RATE (rad/s)';
% STRUCT_HW.signals(7).values = SCOPE_HW(37:40,:)';
% STRUCT_HW.signals(8).title = ' WALL RANGE (m)';
% STRUCT_HW.signals(8).values = SCOPE_HW(41:45,:)';
% STRUCT_HW.signals(9).title = 'MOTOR PWM (%)';
% STRUCT_HW.signals(9).values = SCOPE_HW(46:47,:)';
% STRUCT_HW.signals(10).title = 'BATTERY & MOTOR VOLTAGES (V)';
% STRUCT_HW.signals(10).values = SCOPE_HW(48:56,:)';
% STRUCT_HW.signals(11).title = 'MOTOR CURRENTS (A)';
% STRUCT_HW.signals(11).values = SCOPE_HW(57:58,:)';
% STRUCT_HW.signals(12).title = 'ENCODER MOTOR RATES (rad/s)';
% STRUCT_HW.signals(12).values = SCOPE_HW(59:66,:)';
% SCOPE_HW = STRUCT_HW;
% save SCOPE_HW SCOPE_HW
%------------------------------------------------------------------
% SCOPE_COMP_FV
STRUCT_COMP_FV = struct('time',SCOPE_COMP_FV(1,:)');
STRUCT_COMP_FV.signals(1).title = 'FORWARD VELOCITY (m/s)';
STRUCT_COMP_FV.signals(1).values = SCOPE_COMP_FV(2:3,:)';
STRUCT_COMP_FV.signals(2).title = 'MOTOR VOLTAGE (V)';
STRUCT_COMP_FV.signals(2).values = SCOPE_COMP_FV(4,:)';
STRUCT_COMP_FV.signals(3).title = 'IAE';
STRUCT_COMP_FV.signals(3).values = SCOPE_COMP_FV(5,:)';
SCOPE_COMP_FV = STRUCT_COMP_FV;
save SCOPE_COMP_FV SCOPE_COMP_FV
%------------------------------------------------------------------
% SCOPE_NAV
STRUCT_NAV = struct('time',SCOPE_NAV(1,:)');
STRUCT_NAV.signals(1).title = 'POSITION (m)';
STRUCT_NAV.signals(1).values = SCOPE_NAV(2:5,:)';
STRUCT_NAV.signals(2).title = 'VELOCITY (m/s)';
STRUCT_NAV.signals(2).values = SCOPE_NAV(6:9,:)';
STRUCT_NAV.signals(3).title = 'VELOCITY MAGNITUDE (m/s)';
STRUCT_NAV.signals(3).values = SCOPE_NAV(10:11,:)';
STRUCT_NAV.signals(4).title = 'VELOCITY ANGLE (deg)';
STRUCT_NAV.signals(4).values = SCOPE_NAV(12:13,:)';
STRUCT_NAV.signals(5).title = 'MOTOR VOLTAGE (V)';
STRUCT_NAV.signals(5).values = SCOPE_NAV(14:15,:)';
STRUCT_NAV.signals(6).title = 'MOTOR VOLTAGE (V)';
STRUCT_NAV.signals(6).values = SCOPE_NAV(16:17,:)';
% 1.1 - POSITION X REF (m)
% 1.2 - POSITION Y REF (m)
% 1.3 - POSITION X CONTROL (m)
% 1.4 - POSITION Y CONTROL (m)
% 2.1 - VELOCITY X REF (m/s)
% 2.2 - VELOCITY Y REF (m/s)
% 2.3 - VELOCITY X CONTROL (m/s)
% 2.4 - VELOCITY Y CONTROL (m/s)
% 3.1 - VELOCITY MAGNITUDE REF (m/s)
% 3.2 - VELOCITY MAGNITUDE CONTROL (m/s)
% 4.1 - VELOCITY ANGLE REF (deg)
% 4.2 - VELOCITY ANGLE CONTROL (deg)
% 5.1 - COM MOTOR VOLT (V)
% 5.2 - DIF ROTOR VOLTAGE (V)
% 6.1 - LEFT MOTOR VOLT (V)
% 6.2 - RIGHT ROTOR VOLTAGE (V)