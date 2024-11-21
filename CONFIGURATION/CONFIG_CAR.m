%--------------------------------------------------------------
%% GENERAL CONFIGURATION
%--------------------------------------------------------------
% IP for SSH and TCP/IP communications
% CAR_IP = '127.0.0.1';
CAR_IP = '192.168.0.74';
% CAR_IP = '10.42.0.19';
% RUN_MODE DEFINITION
% / 0. REAL-TIME SIMULATION / 1. FAST SIMULATION / 2. TEST VERIFICATION 
% / 3. IMPLEMENTATION / 4. ALREADY DEPLOYED / 5. GAZEBO SIMULATION
RUN_MODE = 5;
% TEST FOR MODEL IDENTIFICATION
IDENT_TEST = false;
% VEHICLE MODE
% / 0. CAR / 1. SELF-BALANCING VEHICLE
VEHICLE_MODE = 0;
% Sampling time (s)
SAMPLING_TIME = 10e-3;
% The sampling time used in the control should be a multiple of the sampling time
CONTROL_SAMPLING_TIME = 1*SAMPLING_TIME; 
MCS_SAMPLING_TIME = 1*SAMPLING_TIME; % Not used

%--------------------------------------------------------------
%% STRUCT AND BUS DEFINITIONS
%--------------------------------------------------------------
clc
format short g
warning ('off','all');
%--------------------------------------------------------------
% MODEL
%--------------------------------------------------------------
cd ../SOFTWARE_COMPONENTS/MODEL
MODEL_INI = CONFIG_MODEL(SAMPLING_TIME,VEHICLE_MODE);
cd ../../CONFIGURATION
%--------------------------------------------------------------
% SENSORS AND ACTUATORS
%--------------------------------------------------------------
cd ../HARDWARE_COMPONENTS
% *********** MOTOR DRIVER MD25 CONFIGURATION ***********
MD25 = CONFIG_MD25();
% ************** IMU CONFIGURATION ****************
MPU60X0 = CONFIG_MPU60X0();
% ************** RANGE SENSOR CONFIGURATION *************
VL6180X = CONFIG_VL6180X();
% ************** RCRX iBUS CONFIGURATION ****************
RCRX = CONFIG_RCRX();
% *********  WIRELESS CHARGING: CURRENT SENSOR ***********
MCP3201 = CONFIG_MCP3201();
% *******************************************************
cd ../CONFIGURATION
%--------------------------------------------------------------
% EKF
%--------------------------------------------------------------
cd '../SOFTWARE_COMPONENTS/EKF'
EKF_IMU_INI = CONFIG_EKF_IMU(MODEL_INI);
EKF_WFL_INI = CONFIG_EKF_WFL(MODEL_INI);
EKF_NAV_INI = CONFIG_EKF_NAV(MODEL_INI);
cd ../../CONFIGURATION
%------------------------------------
%--------------------------------------------------------------
% COMMUNICATIONS
%--------------------------------------------------------------
% cd ../SOFTWARE_COMPONENTS/COMMUNICATION--------------------------
% CONTROL
%--------------------------------------------------------------
cd ../SOFTWARE_COMPONENTS/CONTROL
MODEL_INI.PARAM.CONTROL_SAMPLING_TIME = CONTROL_SAMPLING_TIME;
[CONTROL_INI,LIN_MODEL] = CONFIG_CONTROL(MODEL_INI);
% EKF
CONTROL_INI.EKF_IMU = EKF_IMU_INI;
CONTROL_INI.EKF_WFL = EKF_WFL_INI;
CONTROL_INI.EKF_NAV = EKF_NAV_INI;
% WALL FOLLOWER MODE: / 0. RANGE SENSOR / 1. LIDAR 2D 
% Option 1 only applies if EKF is enabled
CONTROL_INI.EKF_WFL.PARAM.WALL_FOLLOWER_MODE = CONTROL_INI.STATE.WALL_FOLLOWER_MODE;
% RC TRANSMITTER
CONTROL_INI.PARAM.RC_CH_DEF = RCRX.CH_DEF;
CONTROL_INI.PARAM.RC_CH_TRIM = RCRX.CH_TRIM;
CONTROL_INI.PARAM.RC_CALIB_PARAM = RCRX.CALIB_PARAM;
% CONTROL MODE
MODEL_INI.PARAM.CONTROL_MODE = CONTROL_INI.STATE.CONTROL_MODE;
MODEL_INI.PARAM.WALL_FOLLOWER_MODE = CONTROL_INI.STATE.WALL_FOLLOWER_MODE;
% NAVIGATION
CONTROL_INI = CONFIG_NAV(CONTROL_INI,LIN_MODEL);
CONTROL_INI.STATE.CURRENT_STATUS_RPI4 = uint8(0);
% ENCODER
CONTROL_INI.PARAM.ENC_SAMPLING_TIME = 0.1*SAMPLING_TIME; % Not used
cd ../../CONFIGURATION
%--------------------------------------------------------------
% COMMUNICATIONS
%--------------------------------------------------------------
cd ../SOFTWARE_COMPONENTS/COMMUNICATION
[MSG_LIST,MSG_INI] = CONFIG_MSG(CONTROL_INI);
cd ../../CONFIGURATION
%--------------------------------------------------------------
% BUS DEFINITIONS
%--------------------------------------------------------------
cd '../BUS_DEFINITIONS'
BusDefinition(CONTROL_INI,'CONTROL_Bus')
BusDefinition(MODEL_INI,'MODEL_Bus')
BusDefinition(MSG_INI,'MSG_Bus')
cd ../CONFIGURATION

%--------------------------------------------------------------
%% SIMULINK MODEL CONFIGURATION
%--------------------------------------------------------------
cd('../SIMULINK');
MODEL_SLX = 'CAR_CONTROL_SYSTEM';
PC_SLX = 'PC_CONTROL_STATION';
open(MODEL_SLX)
% RUN_MODE 
switch RUN_MODE
    %-------------------------------------
    case 0 % REAL-TIME SIMULATION
    %-------------------------------------
        set_param(MODEL_SLX,'FixedStep','MODEL_INI.PARAM.SIM_SAMPLING_TIME');
        set_param(MODEL_SLX,'StopTime','inf');
        set_param([MODEL_SLX '/HARDWARE'],'Commented','on');
        set_param([MODEL_SLX '/SIMULATION'],'Commented','off');
        set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
        set_param([MODEL_SLX '/Microseconds at Start'],'Commented','on');
        set_param([MODEL_SLX '/Microseconds at End'],'Commented','on');
        set_param([MODEL_SLX '/COMPUTATIONAL LOAD'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/TEST: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/EXTERNAL MODE: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/HARDWARE: SCOPES'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/COMMUNICATIONS'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/BLACKBOX'],'Commented','on');
        set_param([MODEL_SLX '/SIMULATION/Real-Time Synchronization'],'Commented','off');
        set_param([MODEL_SLX '/GAZEBO SIMULATION'],'Commented','on');
        set_param(MODEL_SLX,'SimulationMode','Normal');
        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(0);
        CONTROL_INI.STATE.CURRENT_STATUS_PC = uint8(1);
        % Communications mode
        CONTROL_INI.STATE.COMM_MODE = uint8(0);
        % Delay for control activation in the state machine
        CONTROL_INI.PARAM.CONTROL_ACT_DELAY = 0.5;
        % Closed-loop transfer-function filter activation
        for nn = 1:4
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF ' num2str(nn)],'Commented','through');
        end
        if CONTROL_INI.STATE.WALL_FOLLOWER_MODE
            MODEL_INI.STATE(6) = 0.25;
        else
            MODEL_INI.STATE(6) = 0.1;
        end
        clear nn       
    %-------------------------------------
    case 1 % FAST SIMULATION
    %-------------------------------------
        set_param(MODEL_SLX,'FixedStep','MODEL_INI.PARAM.SIM_SAMPLING_TIME');
        set_param(MODEL_SLX,'StopTime',num2str(MODEL_INI.PARAM.SIM_FINAL_TIME));
        set_param([MODEL_SLX '/HARDWARE'],'Commented','on');
        set_param([MODEL_SLX '/SIMULATION'],'Commented','off');
        set_param([MODEL_SLX '/SIMULATION/Real-Time Synchronization'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/TEST: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/HARDWARE: SCOPES'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/COMMUNICATIONS'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/EXTERNAL MODE: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/Microseconds at Start'],'Commented','on');
        set_param([MODEL_SLX '/Microseconds at End'],'Commented','on');
        set_param([MODEL_SLX '/COMPUTATIONAL LOAD'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/BLACKBOX'],'Commented','on');
        set_param([MODEL_SLX '/GAZEBO SIMULATION'],'Commented','on');
        set_param(MODEL_SLX,'SimulationMode','Normal');    
        % Communications mode
        CONTROL_INI.STATE.COMM_MODE = uint8(0);
        % Closed-loop transfer-function filter activation
        for nn = 1:4
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF ' num2str(nn)],'Commented','through');
        end
        clear nn
        %*************** CONTROL TYPE ****************
        % Delay in the state machine
        CONTROL_INI.PARAM.CONTROL_ACT_DELAY = 1;
        if CONTROL_INI.STATE.CONTROL_MODE == 1 % ATTITUDE CONTROL
            set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','on');
            set_param([MODEL_SLX '/MONITORING/FV COMPETITION: SCOPES'],'Commented','on');
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(4);
        elseif CONTROL_INI.STATE.CONTROL_MODE == 2 % VELOCITY
            set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','on');
            set_param([MODEL_SLX '/MONITORING/FV COMPETITION: SCOPES'],'Commented','on');
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(5);           
             if CONTROL_INI.STATE.FILTER_CHANGE
                MODEL_INI.PARAM.SIM_FINAL_TIME = 160;
            end
       elseif CONTROL_INI.STATE.CONTROL_MODE == 3 % NAVIGATION
            set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','on');
            set_param([MODEL_SLX '/MONITORING/FV COMPETITION: SCOPES'],'Commented','on');
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(6);
            MODEL_INI.PARAM.SIM_FINAL_TIME = 160;
        elseif CONTROL_INI.STATE.CONTROL_MODE == 4 % WALL FOLLOWER
            set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','on');
            set_param([MODEL_SLX '/MONITORING/FV COMPETITION: SCOPES'],'Commented','on');
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(7);
            if CONTROL_INI.STATE.FV_TARGET_TYPE==4
                MODEL_INI.PARAM.SIM_FINAL_TIME = 320;
            else
                MODEL_INI.PARAM.SIM_FINAL_TIME = 60;
            end
        elseif CONTROL_INI.STATE.CONTROL_MODE == 5 % WALL-FOLLOWER COMPETITION
            set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','on');
            CONTROL_INI.STATE.FV_TARGET_TYPE = uint8(0);
            CONTROL_INI.STATE.WD_TARGET_TYPE = uint8(0);
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(8);
       elseif CONTROL_INI.STATE.CONTROL_MODE == 6 % NAVIGATION COMPETITION
            set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/FV COMPETITION: SCOPES'],'Commented','on');
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(9);
            MODEL_INI.PARAM.SIM_FINAL_TIME = 160;
       elseif CONTROL_INI.STATE.CONTROL_MODE == 7 % FORWARD VELOCITY COMPETITION
            set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','on');
            set_param([MODEL_SLX '/MONITORING/FV COMPETITION: SCOPES'],'Commented','off');
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(10);
            if CONTROL_INI.STATE.FILTER_CHANGE
                MODEL_INI.PARAM.SIM_FINAL_TIME = 160;
            end
        elseif CONTROL_INI.STATE.CONTROL_MODE == 8 % SELF-BALANCE
            set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','on');
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','on');
            set_param([MODEL_SLX '/MONITORING/FV COMPETITION: SCOPES'],'Commented','on');
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(12);
            MODEL_INI.STATE(9) = CONTROL_INI.STATE.PA_INITIAL_VALUE;
            MODEL_INI.PARAM.SIM_FINAL_TIME = 60;
        else % OPEN LOOP
            set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','off');
            set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','on');
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(3);
        end
        set_param(MODEL_SLX,'StopTime',num2str(MODEL_INI.PARAM.SIM_FINAL_TIME));
        CONTROL_INI.INPUT.IMU_PITCH_ANG_INI = CONTROL_INI.PARAM.PA_INITIAL_VALUE;
        if CONTROL_INI.STATE.WALL_FOLLOWER_MODE
            MODEL_INI.STATE(6) = 0.25;
        else
            MODEL_INI.STATE(6) = 0.1;
        end
    %-------------------------------------
    case 2 % TEST VERIFICATION
    %-------------------------------------
        set_param(MODEL_SLX,'FixedStep','MODEL_INI.PARAM.SIM_SAMPLING_TIME');
        set_param(MODEL_SLX,'StopTime',num2str(SCOPE_DATA.time(end)));
        set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','on');     
        set_param([MODEL_SLX '/HARDWARE'],'Commented','on');
        set_param([MODEL_SLX '/SIMULATION'],'Commented','off');
        set_param([MODEL_SLX '/SIMULATION/Real-Time Synchronization'],'Commented','on');
        set_param([MODEL_SLX '/Microseconds at Start'],'Commented','on');
        set_param([MODEL_SLX '/Microseconds at End'],'Commented','on');
        set_param([MODEL_SLX '/COMPUTATIONAL LOAD'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/EXTERNAL MODE: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/COMMUNICATIONS'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/TEST: SCOPES'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/NAV COMPETITION: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/HARDWARE: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/BLACKBOX'],'Commented','on');
        set_param([MODEL_SLX '/GAZEBO SIMULATION'],'Commented','on');
        set_param(MODEL_SLX,'SimulationMode','Normal');
        % Communications mode
        CONTROL_INI.STATE.COMM_MODE = uint8(0);
        %*************** CONTROL TYPE ****************
        CONTROL_INI.PARAM.CONTROL_ACT_DELAY = 0;
        if CONTROL_INI.STATE.CONTROL_MODE == 1 % ATTITUDE CONTROL
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(4);
        elseif CONTROL_INI.STATE.CONTROL_MODE == 2 % VELOCITY
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(5);           
        elseif CONTROL_INI.STATE.CONTROL_MODE == 3 % NAVIGATION
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(6);
        elseif CONTROL_INI.STATE.CONTROL_MODE == 4 % WALL FOLLOWER
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(7);
        elseif CONTROL_INI.STATE.CONTROL_MODE == 5 % WALL-FOLLOWER COMPETITION
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(8);
        elseif CONTROL_INI.STATE.CONTROL_MODE == 7 % FORWARD VELOCITY COMPETITION
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(10);
        else % OPEN LOOP
            CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(3);
        end        
    %-------------------------------------
    case 3 % IMPLEMENTATION
    %-------------------------------------
        clear rpi
        rpi = raspberrypi(CAR_IP,'pi','LabControl');
        try
            rpi.stopModel(MODEL_SLX);
            rpi.system('rm -rf \MATLAB_ws/R2023b/CAR_CONTROL*')
            rpi.system('rm -rf \MATLAB_ws/R2023b/D')
            rpi.system('rm -rf \MATLAB_ws/R2023b/C')
            rpi.system('rm -rf \MATLAB_ws/R2023b/Users')
            aux = datestr(now,'mm dd yyyy HH:MM:SS');
            rpi.system(['sudo date -s "' aux(7:10) '-' aux(1:2) '-' aux(4:5) ' ' aux(12:end) '"'])
        catch
            disp('CONNECTION / REMOVE FAILED')
        end
        set_param(MODEL_SLX,'FixedStep','1e-3');
        set_param(MODEL_SLX,'StopTime','inf');
        set_param([MODEL_SLX '/CONTROL'],'Commented','off');
        set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
        set_param([MODEL_SLX '/HARDWARE'],'Commented','off');
        set_param([MODEL_SLX '/SIMULATION'],'Commented','on');
        set_param([MODEL_SLX '/Microseconds at Start'],'Commented','off');
        set_param([MODEL_SLX '/Microseconds at End'],'Commented','off');
        set_param([MODEL_SLX '/COMPUTATIONAL LOAD'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/EXTERNAL MODE: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/HARDWARE: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/COMMUNICATIONS'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/TEST: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/BLACKBOX'],'Commented','off');
        set_param([MODEL_SLX '/GAZEBO SIMULATION'],'Commented','on');
        % Wireless charging activation
        if CONTROL_INI.STATE.WIRELESS_CHARGING
            set_param([MODEL_SLX '/HARDWARE/SENSORS/CURRENT_ADC_MCP3201_SPI'],'Commented','off');
        else
            set_param([MODEL_SLX '/HARDWARE/SENSORS/CURRENT_ADC_MCP3201_SPI'],'Commented','on');           
        end
        % MCS enabled if needed
        if CONTROL_INI.STATE.NAV_MODE == 1 || CONTROL_INI.STATE.MCS_MODE == 1
            set_param([MODEL_SLX '/HARDWARE/COMMUNICATIONS/MCS_RX'],'Commented','off');
        else
            set_param([MODEL_SLX '/HARDWARE/COMMUNICATIONS/MCS_RX'],'Commented','on');
        end
        set_param(MODEL_SLX,'SimulationMode','External');
        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(0);
        CONTROL_INI.STATE.CURRENT_STATUS_PC = uint8(0);
        % CONTROL_INI.STATE.COMM_MODE = uint8(1);
        CONTROL_INI.PARAM.CONTROL_ACT_DELAY = 0.5;         
    %-------------------------------------
    case 4 % ALREADY DEPLOYED
    %-------------------------------------
        % SYSTEM STATUS
        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(0);
        CONTROL_INI.STATE.CURRENT_STATUS_PC = uint8(0);
        if VEHICLE_MODE == 1
            CONTROL_INI.PARAM.CONTROL_ACT_DELAY = 0;
        else
            CONTROL_INI.PARAM.CONTROL_ACT_DELAY = 0.5;
        end
        % Communications mode
        % CONTROL_INI.STATE.COMM_MODE = uint8(1);
        cd('../SIMULINK');
        % bdclose(MODEL_SLX);
        open(PC_SLX);
        set_param([PC_SLX '/HARDWARE/RT_MONITORING'],'Commented','off');
        set_param([PC_SLX '/HARDWARE/ROS2_MONITORING'],'Commented','on');
        for ii = 1:21
            set_param([PC_SLX '/HARDWARE/RT_MONITORING/TCP ' num2str(100+ii)],'Commented','on');
        end
        set_param([PC_SLX '/HARDWARE/RT_MONITORING/TCP ' CAR_IP(end-2:end)],'Commented','off');
        % set_param([PC_SLX '/HARDWARE/RT_MONITORING/TCP RSP'],'Commented','off');
        if CONTROL_INI.STATE.CONTROL_MODE == 5 % WALL-FOLLOWER COMPETITION
            CONTROL_INI.STATE.FV_TARGET_TYPE = uint8(0);
            CONTROL_INI.STATE.WD_TARGET_TYPE = uint8(0);
        end
    %-------------------------------------
    case 5 % GAZEBO SIMULATION
    %-------------------------------------

        set_param(MODEL_SLX,'FixedStep','1e-3');
        set_param(MODEL_SLX,'StopTime','inf');
        set_param([MODEL_SLX '/CONTROL'],'Commented','off');
        set_param([MODEL_SLX '/CONTROL/LOCAL TARGETS'],'Commented','off');
        set_param([MODEL_SLX '/HARDWARE'],'Commented','on');
        set_param([MODEL_SLX '/SIMULATION'],'Commented','off');
        set_param([MODEL_SLX '/Microseconds at Start'],'Commented','on');
        set_param([MODEL_SLX '/Microseconds at End'],'Commented','on');
        set_param([MODEL_SLX '/COMPUTATIONAL LOAD'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/TEST: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/HARDWARE: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/COMMUNICATIONS'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/EXTERNAL MODE: SCOPES'],'Commented','on');
        set_param([MODEL_SLX '/MONITORING/BLACKBOX'],'Commented','on');
        set_param([MODEL_SLX '/GAZEBO SIMULATION'],'Commented','off');
        set_param([MODEL_SLX '/SIMULATION'],'Commented','on');
        set_param(MODEL_SLX,'SimulationMode','External');
        set_param(MODEL_SLX,'HardwareBoard','Robot Operating System 2 (ROS 2)');
        set_param(MODEL_SLX,'TargetHWDeviceType','ARM Compatible->ARM 8');
        set_param(MODEL_SLX,'SupportVariableSizeSignals','on');
        set_param(MODEL_SLX,'CodeInterfacePackaging','Nonreusable function');
        set_param(MODEL_SLX,'MATLABDynamicMemAlloc','on');

        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(0);
        CONTROL_INI.STATE.CURRENT_STATUS_PC = uint8(0);
        CONTROL_INI.PARAM.CONTROL_ACT_DELAY = 0.5;
        % SYSTEM STATUS
        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(0);
        CONTROL_INI.STATE.CURRENT_STATUS_PC = uint8(0);
        if VEHICLE_MODE == 1
            CONTROL_INI.PARAM.CONTROL_ACT_DELAY = 0;
        else
            CONTROL_INI.PARAM.CONTROL_ACT_DELAY = 0.5;
        end
        % Communications mode
        % CONTROL_INI.STATE.COMM_MODE = uint8(1);
        cd('../SIMULINK');
        open(PC_SLX);
        set_param([PC_SLX '/HARDWARE/RT_MONITORING'],'Commented','on');
        set_param([PC_SLX '/HARDWARE/ROS2_MONITORING'],'Commented','off');
    otherwise
end

%--------------------------------------------------------------
%% SYSTEM IDENTIFICATION INPUT
%--------------------------------------------------------------
N = 512; % Period
B = 50; % Number of samples for which the value does not change
SYSID_TIME = (0:SAMPLING_TIME:(N-1)*SAMPLING_TIME)';
SYSID_INPUT = idinput([N,2],'prbs',[0 1/B],[-1 1]);
SYSID_INPUT = SYSID_INPUT - mean(SYSID_INPUT);
clear N B

%--------------------------------------------------------------
%% MODEL DEFINITION FOR CONTROL DESIGN
%--------------------------------------------------------------
% OPERATING_POINT
NUM_OP = CONTROL_INI.STATE.FORWARD_VEL_MAIN_OP;
% NUM_OP = 4;
% LAPLACE VARIABLE
s = tf('s');

[num_aux,den_aux] = tfdata(LIN_MODEL(NUM_OP).FORWARD_VEL_TF_MODEL);
FORWARD_VEL_TF = tf(num_aux{:},den_aux{:});
FORWARD_VEL_DELAY_TF = FORWARD_VEL_TF*exp(-MODEL_INI.PARAM.MOTOR_DELAY*s);
FORWARD_VEL_ZPK = zpk(FORWARD_VEL_TF);
FORWARD_VEL_ZPK.DisplayFormat = 'TimeConstant';

[num_aux,den_aux] = tfdata(LIN_MODEL(NUM_OP).YAW_RATE_TF_MODEL);
YAW_RATE_TF = tf(num_aux{:},den_aux{:});
YAW_RATE_DELAY_TF = YAW_RATE_TF*exp(-MODEL_INI.PARAM.MOTOR_DELAY*s);
YAW_RATE_ZPK = zpk(YAW_RATE_TF);
YAW_RATE_ZPK.DisplayFormat = 'TimeConstant';

[num_aux,den_aux] = tfdata(LIN_MODEL(NUM_OP).YAW_ANG_TF_MODEL);
YAW_ANG_TF = tf(num_aux{:},den_aux{:});
YAW_ANG_ZPK = zpk(YAW_ANG_TF);
YAW_ANG_ZPK.DisplayFormat = 'TimeConstant';

[num_aux,den_aux] = tfdata(LIN_MODEL(NUM_OP).WFL_SL_TF_MODEL);
WFL_SL_TF = tf(num_aux{:},den_aux{:});
WFL_SL_ZPK = zpk(WFL_SL_TF);
WFL_SL_ZPK.DisplayFormat = 'TimeConstant';

[num_aux,den_aux] = tfdata(LIN_MODEL(NUM_OP).WFL_CD_TF_MODEL);
WFL_CD_TF = minreal(tf(num_aux{:},den_aux{:}),1e-1);
WFL_CD_ZPK = minreal(zpk(WFL_CD_TF),1e-1);
WFL_CD_ZPK.DisplayFormat = 'TimeConstant';

if CONTROL_INI.STATE.VEHICLE_MODE == 1
    % Linear state-space model
    SBV_SS_MODEL = LIN_MODEL(1).PITCH_ANG_SS_MODEL;
    % Forward velocity transfer function
    [num_aux,den_aux] = tfdata(LIN_MODEL(1).PITCH_ANG_SS_MODEL(1,1));
    FV_SBV_TF = minreal(tf(num_aux{:},den_aux{:}),1e-1);
    FV_SBV_ZPK = minreal(zpk(FV_SBV_TF),1e-1);
    FV_SBV_ZPK.DisplayFormat = 'TimeConstant';
    % Pitch rate transfer function
    [num_aux,den_aux] = tfdata(LIN_MODEL(1).PITCH_ANG_SS_MODEL(2,1));
    PR_SBV_TF = minreal(tf(num_aux{:},den_aux{:}),1e-1);
    PR_SBV_ZPK = minreal(zpk(PR_SBV_TF),1e-1);
    PR_SBV_ZPK.DisplayFormat = 'TimeConstant';
    % Pitch angle transfer function
    [num_aux,den_aux] = tfdata(LIN_MODEL(1).PITCH_ANG_SS_MODEL(3,1));
    PA_SBV_TF = minreal(tf(num_aux{:},den_aux{:}),1e-1);
    PA_SBV_ZPK = minreal(zpk(PA_SBV_TF),1e-1);
    PA_SBV_ZPK.DisplayFormat = 'TimeConstant';
    % Wall follower: linearized state-space model
    WFL_SS_MODEL = LIN_MODEL(NUM_OP).WFL_SL_SS_MODEL(3,1);
end

%--------------------------------------------------------------
%% CONTROL FOR MODEL IDENTIFICATION TEST
%--------------------------------------------------------------
if IDENT_TEST && CONTROL_INI.STATE.CONTROL_MODE==2 && VEHICLE_MODE==0
    CONTROL_INI.PARAM.FORWARD_VEL_PID.K = 20;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.Ti = 1;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.Td = 0;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.N = 1;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.b = 1;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE = uint8(0);
    CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE = uint8(0);
    CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(0);
    CONTROL_INI.PARAM.FORWARD_VEL_PID.ANTIWINDUP = uint8(0);
    CONTROL_INI.PARAM.YAW_RATE_PID.K = 1.25;
    CONTROL_INI.PARAM.YAW_RATE_PID.Ti = 0;
    CONTROL_INI.PARAM.YAW_RATE_PID.Td = 0;
    CONTROL_INI.PARAM.YAW_RATE_PID.N = 1;
    CONTROL_INI.PARAM.YAW_RATE_PID.b = 1;
    CONTROL_INI.PARAM.YAW_RATE_PID.INT_DISC_TYPE = uint8(0);
    CONTROL_INI.PARAM.YAW_RATE_PID.DER_DISC_TYPE = uint8(0);
    CONTROL_INI.PARAM.YAW_RATE_PID.DER_INPUT = uint8(0);
    CONTROL_INI.PARAM.YAW_RATE_PID.ANTIWINDUP = uint8(0);
    CONTROL_INI.STATE.WFL_FEEDFORWARD = uint8(0);    
end
if IDENT_TEST && (CONTROL_INI.STATE.CONTROL_MODE==8 || ...
                  CONTROL_INI.STATE.CONTROL_MODE==2 || ...
                  CONTROL_INI.STATE.CONTROL_MODE==4) && ...
                  VEHICLE_MODE==1
    CONTROL_INI.PARAM.PITCH_ANG_SFC.K = [-18 4 25];
    % CONTROL_INI.PARAM.FORWARD_VEL_SFC.K = [-25 4 23];
    % CONTROL_INI.PARAM.FORWARD_VEL_SFC.Ki = 6;
end

clear MODEL_SLX

