function [CONTROL,LIN_MODEL] = CONFIG_CONTROL(MODEL)
%------------------------------------------------------------------------
%% CONTROL MODE
%-------------------------------------------------------------------------
% / 0. OPEN LOOP / 1. FORWARD VELOCITY AND YAW ANGLE / 
% / 2. FORWARD VELOCITY AND YAW RATE / 3. NAVIGATION / 
% / 4. WALL FOLLOWER / 5. WALL FOLLOWING COMPETITION /
% / 6. NAVIGATION COMPETITION / 7. FORWARD VELOCITY COMPETITION /
% / 8. SELF-BALANCE
CONTROL.STATE.CONTROL_MODE = uint8(2);

%------------------------------------------------------------------------
%% CONTROL TYPES
%-------------------------------------------------------------------------
% FORWARD VELOCITY CONTROL TYPE
% / 0. PID  / 1. STATE FEEDBACK CONTROL / 2, FIRST_ORDER DEAD BEAT 
% / 3. SECOND-ORDER DEAD BEAT        
CONTROL.STATE.FORWARD_VEL_CONTROL_TYPE = uint8(0);
%--------------------------------------------------------------
% YAW RATE CONTROL TYPE
% / 0. PID  / 1. STATE FEEDBACK CONTROL / 2, FIRST_ORDER DEAD BEAT 
% / 3. SECOND-ORDER DEAD BEAT        
CONTROL.STATE.YAW_RATE_CONTROL_TYPE = uint8(0);
%--------------------------------------------------------------
% YAW ANGLE CONTROL TYPE
% / 0. PID  / 1. STATE FEEDBACK CONTROL         
CONTROL.STATE.YAW_ANG_CONTROL_TYPE = uint8(0);
%--------------------------------------------------------------
% WALL FOLLOWER CONTROL TYPE
% / 0. SINGLE LOOP  / 1. CASCADE / 2. STATE FEEDBACK CONTROL         
CONTROL.STATE.WFL_CONTROL_TYPE = uint8(1);
%--------------------------------------------------------------
% PITCH ANGLE CONTROL TYPE
% / 0. PID  / 1. STATE FEEDBACK REGULATOR / 2. STATE FEEDBACK INTEGRAL CONTROL     
CONTROL.STATE.PITCH_ANG_CONTROL_TYPE = uint8(1);
%--------------------------------------------------------------
% NAVIGATION CONTROL TYPE
% / 0. ADAPTIVE LQR  / 1. NON-LINEAR MPC     
CONTROL.STATE.NAV_CONTROL_TYPE = uint8(0);
%--------------------------------------------------------------
% NAVIGATION IN CAR MODE (STATE FEEDBACK CONTROL IN FORWARD VELOCITY AND YAW RATE)
if (CONTROL.STATE.CONTROL_MODE==3 || CONTROL.STATE.CONTROL_MODE==6) && ...
                    MODEL.PARAM.VEHICLE_MODE==0
    CONTROL.STATE.FORWARD_VEL_CONTROL_TYPE = uint8(1);  
    CONTROL.STATE.YAW_RATE_CONTROL_TYPE = uint8(1);    
end

%--------------------------------------------------------------
%% CONTROL LIMITS
%--------------------------------------------------------------
% BATTERY
CONTROL.PARAM.BATTERY_VOLT = MODEL.PARAM.BATTERY_VOLT;
% MOTOR VOLTAGE LIMITS {'MOTOR VOLT LEFT','MOTOR VOLT RIGHT'}
BATTERY_VOLT = CONTROL.PARAM.BATTERY_VOLT;
% CONTROL.PARAM.MOTOR_VOLT_MAX = [BATTERY_VOLT BATTERY_VOLT]';
% CONTROL.PARAM.MOTOR_VOLT_MIN = [-BATTERY_VOLT -BATTERY_VOLT]';
CONTROL.PARAM.MOTOR_VOLT_MAX = [BATTERY_VOLT BATTERY_VOLT]';
CONTROL.PARAM.MOTOR_VOLT_MIN = [-BATTERY_VOLT -BATTERY_VOLT]';
% NAVIGATION CONTROL LIMITS {'FORWARD VEL REF','YAW RATE REF'}
CONTROL.PARAM.NAV_MV_MAX = [+0.5 +50*pi/180]';
CONTROL.PARAM.NAV_MV_MIN = [   0 -50*pi/180]';

%--------------------------------------------------------------
%% REFERENCE DEFINITION
%--------------------------------------------------------------
% FORWARD VELOCITY REFERENCE TYPE
% / 0. CONSTANT / 1. PULSE / 2. SQUARE / 3. PRBS / 4. RAMP
CONTROL.STATE.FV_TARGET_TYPE = uint8(0);
%--------------------------------------------------------------
% YAW RATE REFERENCE TYPE
% / 0. NONE / 1. PULSE / 2. SQUARE / 3. PRBS
CONTROL.STATE.YR_TARGET_TYPE = uint8(0);
%--------------------------------------------------------------
% YAW ANGLE REFERENCE TYPE
% / 0. NONE / 1. PULSE / 2. SQUARE / 3. PRBS
CONTROL.STATE.YA_TARGET_TYPE = uint8(0);
%--------------------------------------------------------------
% WALL DISTANCE REFERENCE TYPE
% / 0. CONSTANT = 0.1 m / 1. PULSE / 2. SQUARE / 3. PRBS / 4. CURVE
CONTROL.STATE.WD_TARGET_TYPE = uint8(0);
%--------------------------------------------------------------
% FORWARD VELOCITY CONSTANT REFERENCE VALUE (0-4)
CONTROL.PARAM.FV_TARGET_VALUE = uint8(1);
%--------------------------------------------------------------
% INITIAL PITCH ANGLE FOR SELF-BALANCING VEHICLE (rad) -> (pitch(0) = 11 deg)
CONTROL.PARAM.PA_INITIAL_VALUE = 5*pi/180;
%--------------------------------------------------------------
% WALL DISTANCE CONSTANT REFERENCE VALUE = 0.1 m
%--------------------------------------------------------------
% MOTOR VOLTAGE REFERENCE TYPE
% / 0. CONSTANT / 1. PULSE / 2. SQUARE / 3. PRBS
CONTROL.STATE.MV_TARGET_TYPE = uint8([2 0])';
%--------------------------------------------------------------
% NAVIGATION REFERENCE TYPE
% / 0. CONSTANT / 1. SQUARE / 2. SINUSOIDAL
CONTROL.STATE.NAV_TARGET_TYPE = uint8([0 0]);
% CONSTANT REFERENCE VALUE (INCREMENT OVER THE INITIAL POS_XY_REF)
CONTROL.PARAM.NAV_TARGET_VALUE = [1 0];
CONTROL.PARAM.NAV_STOP_RADIUS = 0.05;
CONTROL.PARAM.NAV_MAX_TARGET_RADIUS = 0.2;
% RELATIVE POSITION OF THE VEHICLE CENTER WITH RESPECT TO MOTOR AXIS CENTER
CONTROL.PARAM.NAV_CENTER_POS = [0.08775 -0.006356];
%--------------------------------------------------------------
% REFERENCE SOURCE
% / 0. LOCAL / 1. PC  / 2. RC JOYSTICK / 3. RC SWITCHES 
% 4. LOCAL MISSION / 5. EXTERNAL MISSION
CONTROL.STATE.MV_TARGET_SOURCE = uint8([0 0])';
CONTROL.STATE.FV_TARGET_SOURCE = uint8(1);
CONTROL.STATE.YR_TARGET_SOURCE = uint8(1);
CONTROL.STATE.YA_TARGET_SOURCE = uint8(0);
CONTROL.STATE.WD_TARGET_SOURCE = uint8(0);
CONTROL.STATE.NAV_TARGET_SOURCE = uint8(0);
%--------------------------------------------------------------
% MISSION WAYPOINTS [wait time(s) X(m) Y(m) YAW(deg)]
 MISSION_WAYPOINTS = [
     1    1     0     30
     1    0     1     180
     1    0     0    -60
     1    1     1     90
     1    0     1    120
];
% if any(MISSION_WAYPOINTS(1:end-1,1).*MISSION_WAYPOINTS(2:end,1))
%     disp('The first column of MISSION_WAYPOINTS can not have 2 consecutive non-zero values')
%     return
% end
CONTROL.PARAM.MISSION_WAYPOINTS = zeros(10,4);
CONTROL.PARAM.MISSION_NUM_WAYPOINTS = size(MISSION_WAYPOINTS,1);
if CONTROL.STATE.NAV_TARGET_SOURCE == uint8(4)
    CONTROL.PARAM.MISSION_WAYPOINTS(1:CONTROL.PARAM.MISSION_NUM_WAYPOINTS,:) = MISSION_WAYPOINTS;
end
CONTROL.PARAM.MISSION_WP_RADIUS = 0.05;
CONTROL.PARAM.MISSION_YA_ERROR = 2.5*pi/180;
CONTROL.PARAM.MISSION_COUNT = uint8(1);
%--------------------------------------------------------------
% RC RECEIVER (JOYSTICKS)
% Open-loop RC scale {'MOTOR VOLT LEFT','MOTOR VOLT RIGHT'}
CONTROL.PARAM.OPL_RC_SCALE = CONTROL.PARAM.MOTOR_VOLT_MAX;
% Velocity RC limits {'FORWARD VEL REF','YAW RATE REF'}
CONTROL.PARAM.VEL_RC_SCALE = [+0.5 +180*pi/180];
% WALL follower RC limits {'FORWARD VEL REF','WALL DIST REF'}
CONTROL.PARAM.WFL_RC_SCALE = [+0.5 +0.05];
% Navigation RC limits {'INC POS X REF','INC POS Y REF'}
CONTROL.PARAM.NAV_RC_SCALE = +1;
%--------------------------------------------------------------
% RC RECEIVER (SWITCHES)
% Open-loop switch values {'LEFT MOTOR VOLT','RIGHT MOTOR VOLT'}
CONTROL.PARAM.OPL_RC_SWITCH = [-BATTERY_VOLT/3 0 +BATTERY_VOLT/3 
                               -BATTERY_VOLT/3 0 +BATTERY_VOLT/3];
% Velocity switch values {'FORWARD VEL REF','YAW RATE REF'}
CONTROL.PARAM.VEL_RC_SWITCH = [      0  +0.2  +0.4
                                -180*pi/180 0 +180*pi/180];
% Wall follower switch values {'FORWARD VEL REF','WALL DIST REF'}
CONTROL.PARAM.WFL_RC_SWITCH = [0   0.25  0.5 
                              0.05  0.1 0.15]; 
% Navigation switch values {'POS X REF','POS Y REF'}
CONTROL.PARAM.NAV_RC_SWITCH = [1  0  -1 
                               1  0  -1]; 

%--------------------------------------------------------------
%% DESIGN METHOD FOR PID CONTROL
%--------------------------------------------------------------
% Only for FORWARD VELOCITY and YAW RATE
% / 0. FREQUENCY RESPONSE  / 1. TIME RESPONSE       
CONTROL.PARAM.PID_DESIGN_METHOD = uint8(0);
%--------------------------------------------------------------
% FREQUENCY RESPONSE DESIGN MODEL:
% / 0. ANALOG MODEL / 1. ANALOG MODIFIED MODEL
CONTROL.PARAM.PID_FR_DESIGN_MODEL = 0;
%--------------------------------------------------------------
% DISCRETIZATION METHOD (only for forward velocity PID): 
%  / 1. BACKWARD EULER  / 2. FORWARD EULER  / 3. TRAPEZOIDAL
CONTROL.PARAM.PID_FV_DISC_METHOD = 3;
%-------------------------------------------------------------
%% FEEDFORWARD (MOTOR VOLTAGE DROP IN DIFFERENTIAL MODE)
%-------------------------------------------------------------
% VOLTAGE DROP IN DIFFERENTIAL MOTOR VOLTAGE
% / 0. NOT ENABLED / 1. ENABLED
CONTROL.STATE.WFL_FEEDFORWARD = uint8(1);

%--------------------------------------------------------------
%% DELAY IN MOTOR VOLTAGE
%-------------------------------------------------------------
% MOTOR DELAY MODE 
% / 0. WITHOUT DELAY / 1. SIMULATED DELAY / 2. ACTUAL DELAY
CONTROL.STATE.MOTOR_DELAY_MODE = uint8(0);
%--------------------------------------------------------------
% SMITH PREDICTOR FOR FORWARD-VELOCITY AND YAW-RATE CONTROL
% / 0. DESIGN WITH DELAY / 1. SMITH PREDICTOR / 2. DESIGN WITHOUT DELAY
CONTROL.STATE.SP_MODE = uint8([0 0]);
%--------------------------------------------------------------
% MOTOR DELAY
CONTROL.PARAM.MOTOR_DELAY = MODEL.PARAM.MOTOR_DELAY;
%--------------------------------------------------------------
% DEVIATION IN ACTUAL MOTOR DELAY (multiples of +/- 10 ms)
CONTROL.PARAM.MOTOR_DELAY_ERR = 0;

%--------------------------------------------------------------
%% MEASUREMENT FILTERING AND STATE ESTIMATION
%-------------------------------------------------------------
% OBSERVER MODE
% / 0. FILTERED MEASUREMENT / 1. EKF 
CONTROL.STATE.OBSERVER_MODE = uint8(1);
%--------------------------------------------------------------
% ROTATION MEASUREMENT MODE (only for filtered measurement)
% / 0. IMU / 1. ENCODER
CONTROL.STATE.ROTATION_MSRT_MODE = uint8(1);
%--------------------------------------------------------------
% NAVIGATION MODE
% / 0. NOT AVAILABE / 1. MCS / 2. LIDAR / 3. OTHER
CONTROL.STATE.NAV_MODE = uint8(3);
% MCS MODE (AVAILABLE FOR COMPARISON)
% / 0. NOT AVAILABE / 1. AVAILABLE
CONTROL.STATE.MCS_MODE = uint8(0);
%--------------------------------------------------------------
% WALL-FOLLOWER MODE
% / 0. WALL-RANGE SENSOR / 1. LIDAR 2D 
% Option 1 only applies if EKF is enabled
CONTROL.STATE.WALL_FOLLOWER_MODE = uint8(0);
%--------------------------------------------------------------
% FILTER PARAMETERS
% External IMU low-pass filter (Hz)
CONTROL.PARAM.IMU_FILT_FREQ = 1/2/pi/0.04;
% Encoder low-pass filter (Hz)
CONTROL.PARAM.ENC_FILT_FREQ = 1/2/pi/0.04;
% Range low-pass filter (Hz)
CONTROL.PARAM.WFL_FILT_FREQ = 1/2/pi/0.04;
% SVF low-pass filter (Hz)
SVF_FILT_FREQ = 1/2/pi/0.04;
% EKF low-pass filter (Hz)
CONTROL.PARAM.EKF_FILT_FREQ = CONTROL.PARAM.ENC_FILT_FREQ;
% Complementary filter (discrete-time pole)
CONTROL.PARAM.IMU_CF_FREQ = -log(0.98)/2/pi/MODEL.PARAM.SAMPLING_TIME;
% MCS calibration
CONTROL.PARAM.MCS_ALFA_CALIB = 0.95;
CONTROL.PARAM.MCS_EARTH_POS_OFFS = zeros(3,1);
CONTROL.PARAM.MCS_EULER_ANG_OFFS = zeros(3,1);
CONTROL.PARAM.MCS_DELAY = 0.1;
% NAV calibration
CONTROL.PARAM.NAV_ALFA_CALIB = 0.95;
CONTROL.PARAM.NAV_EARTH_POS_OFFS = zeros(3,1);
CONTROL.PARAM.NAV_EULER_ANG_OFFS = zeros(3,1);

%--------------------------------------------------------------
% ENCODER FILTER: STATE VARIABLE FILTER     
clear SVF_IN SVF_OUT
% Natural frequency (rad/s)
SVF_IN.freq = 2*pi*CONTROL.PARAM.ENC_FILT_FREQ;
% Damping factor
SVF_IN.damp = 1;
% Sampling time
SVF_IN.ts = MODEL.PARAM.SAMPLING_TIME;
% Filter order
SVF_IN.order = 1;
% Filter design
SVF_OUT = DESIGN_SVF(SVF_IN);
ENC_FLT_SS_MODEL = SVF_OUT.F_ss;
%--------------------------------------------------------------
% IMU FILTER: STATE VARIABLE FILTER     
clear SVF_IN SVF_OUT
% Natural frequency (rad/s)
SVF_IN.freq = 2*pi*CONTROL.PARAM.IMU_FILT_FREQ;
% Damping factor
SVF_IN.damp = 1;
% Sampling time
SVF_IN.ts = MODEL.PARAM.SAMPLING_TIME;
% Filter order
SVF_IN.order = 1;
% Filter design
SVF_OUT = DESIGN_SVF(SVF_IN);
IMU_FLT_SS_MODEL = SVF_OUT.F_ss;
%--------------------------------------------------------------
% RANGE FILTER: STATE VARIABLE FILTER     
clear SVF_IN SVF_OUT
% Natural frequency (rad/s)
SVF_IN.freq = 2*pi*CONTROL.PARAM.WFL_FILT_FREQ;
% Damping factor
SVF_IN.damp = 1;
% Sampling time
SVF_IN.ts = MODEL.PARAM.SAMPLING_TIME;
% Filter order
SVF_IN.order = 1;
% Filter design
SVF_OUT = DESIGN_SVF(SVF_IN);
RNG_FLT_SS_MODEL = SVF_OUT.F_ss;
%--------------------------------------------------------------
% EKF FILTER: STATE VARIABLE FILTER     
clear SVF_IN SVF_OUT
% Natural frequency (rad/s)
SVF_IN.freq = 2*pi*CONTROL.PARAM.EKF_FILT_FREQ;
% Damping factor
SVF_IN.damp = 1;
% Sampling time
SVF_IN.ts = MODEL.PARAM.SAMPLING_TIME;
% Filter order
SVF_IN.order = 1;
% Filter design
SVF_OUT = DESIGN_SVF(SVF_IN);
EKF_FLT_SS_MODEL = SVF_OUT.F_ss;
%--------------------------------------------------------------
% SVF: STATE VARIABLE FILTER     
clear SVF_IN SVF_OUT
% Natural frequency (rad/s)
SVF_IN.freq = 2*pi*SVF_FILT_FREQ;
% Damping factor
SVF_IN.damp = 1;
% Sampling time
SVF_IN.ts = MODEL.PARAM.SAMPLING_TIME;
% Filter order
SVF_IN.order = 2;
% Filter design
SVF_OUT = DESIGN_SVF(SVF_IN);
% Output = [yf dyf]
CONTROL.PARAM.SVF_Ad = SVF_OUT.Ad;
CONTROL.PARAM.SVF_B1d = SVF_OUT.B1d;
CONTROL.PARAM.SVF_B2d = SVF_OUT.B2d;
CONTROL.PARAM.SVF_Cd = SVF_OUT.Cd;
CONTROL.PARAM.SVF_Dd = SVF_OUT.Dd;
%--------------------------------------------------------------
% ENCODER: FORWARD VELOCITY KALMAN FILTER
ts = MODEL.PARAM.SAMPLING_TIME;
Kv = MODEL.PARAM.VEL_GAIN; 
Tv = MODEL.PARAM.VEL_TIME_CONSTANT; 
Tf = 1/2/pi/CONTROL.PARAM.ENC_FILT_FREQ;
R = MODEL.PARAM.WHEEL_RADIUS;
% Velocity model
% dv/dt = -1/Tv*v + Kv/Tv*uc
% d(com_ang)/dt = v/R
A_v = [ -1/Tv -Kv/Tv 0; 0 0 0 ; 1/Tf 0 -1/Tf];
B_v = [ Kv/Tv ; 0 ; 0];
% Zero-order hold discretization
% x[k+1] = Ad*x[k] + Bd*u[k]
Ad_v = expm(A_v*ts);
% Ad = I + A*ts + (A*ts)^2/2 + ... + (A*ts)^k/k!                      
% Bd = A^-1*(Ad-I)*B = (I + A*ts/2 + ... + (A*ts)^(k-1)/k!)*B*ts
Bd_v = eye(3);
aux = eye(3);
for nn = 1:50
    aux = aux*A_v*ts/(nn+1);
    Bd_v = Bd_v + aux;
end
Bd_v = Bd_v*B_v*ts;
CONTROL.PARAM.ENC_FORWARD_VEL_matAd = Ad_v;
CONTROL.PARAM.ENC_FORWARD_VEL_matBd = Bd_v;
CONTROL.PARAM.ENC_FORWARD_VEL_matCd = [0 0 1; 1/Tf 0 -1/Tf];
CONTROL.PARAM.ENC_FORWARD_VEL_matQ = [1 0.00106147 0.00101901];
CONTROL.PARAM.ENC_FORWARD_VEL_matR = [3*0.0193655 103.434];
%--------------------------------------------------------------
% ENCODER: YAW RATE KALMAN FILTER
Kw = MODEL.PARAM.YAW_RATE_GAIN; 
Tw = MODEL.PARAM.YAW_RATE_TIME_CONSTANT; 
W = MODEL.PARAM.WHEEL_DISTANCE;
% Yaw rate model
% dw/dt = -1/Tw*w + Kw/Tw*ud
% d(dif_ang)/dt = w*W/R/2
A_w = [ -1/Tw -Kw/Tw 0 0; 0 0 0 0; 1/Tf 0 -1/Tf 0 ; 0 0 1 0];
B_w = [ Kw/Tw ; 0 ; 0 ; 0];
% Zero-order hold discretization
% x[k+1] = Ad*x[k] + Bd*u[k]
Ad_w = expm(A_w*ts);
% Ad = I + A*ts + (A*ts)^2/2 + ... + (A*ts)^k/k!                      
% Bd = A^-1*(Ad-I)*B = (I + A*ts/2 + ... + (A*ts)^(k-1)/k!)*B*ts
Bd_w = eye(4);
aux = eye(4);
for nn = 1:50
    aux = aux*A_w*ts/(nn+1);
    Bd_w = Bd_w + aux;
end
Bd_w = Bd_w*B_w*ts;
CONTROL.PARAM.ENC_YAW_RATE_matAd = Ad_w;
CONTROL.PARAM.ENC_YAW_RATE_matBd = Bd_w;
CONTROL.PARAM.ENC_YAW_RATE_matCd = [0 0 1 0; 0 0 1 0];
CONTROL.PARAM.ENC_YAW_RATE_matQ = [1 4.38945e-05 3.59744e-05 1e-3];
CONTROL.PARAM.ENC_YAW_RATE_matR = [88.9832/500 1682.83/500];
%--------------------------------------------------------------
%% OPERATING POINTS (FORWARD VELOCITY)
%--------------------------------------------------------------
% DEFINITION OF OPERATING POINTS (GAIN SCHEDULING)
FORWARD_VEL_OP = [0.2 0.3 0.4 0.5];
%--------------------------------------------------------------
% OPERATING POINT FOR CONTROL DESIGN (1-4)
CONTROL.STATE.FORWARD_VEL_MAIN_OP = uint8(2);
%--------------------------------------------------------------
% OPERATING POINT AND MODEL LINEARIZATION 
% COMPUTATION OF OPERATING POINTS AND LINEAR MODELS
MODEL.PARAM.WALL_FOLLOWER_MODE = double(CONTROL.STATE.WALL_FOLLOWER_MODE);
for nn = 1:length(FORWARD_VEL_OP)
    clc
    fprintf('OPERATING POINT COMPUTATION %d/%d \n',nn,length(FORWARD_VEL_OP));
    LIN_MODEL(nn) = LINEARIZE_MODEL(FORWARD_VEL_OP(nn),MODEL);
end
%--------------------------------------------------------------
% OPERATING POINTS
% Number of operating points
N = length(LIN_MODEL);
% Operating points
nu = length(LIN_MODEL(1).OP_INPUT);
nx = length(LIN_MODEL(1).OP_STATE);
ny = length(LIN_MODEL(1).OP_OUTPUT); 
CONTROL.PARAM.OP_INPUT = zeros(nu,N);
CONTROL.PARAM.OP_STATE = zeros(nx,N);
CONTROL.PARAM.OP_OUTPUT = zeros(ny,N);
%-------------------------------------------------------------
% STATE  = {'FORWARD VEL','YAW RATE','YAW_ANG','WALL DIST','PITCH RATE','PITCH ANG'}
% INPUT = {'MOTOR VOLT LEFT ','MOTOR VOLT RIGHT '}
% OUTPUT = {'FORWARD VEL','YAW RATE',YAW ANG','WALL DIST','PITCH RATE','PITCH ANG'}
%-------------------------------------------------------------
if CONTROL.STATE.CONTROL_MODE<8
    for ii = 1:N
        CONTROL.PARAM.OP_INPUT(:,ii) = LIN_MODEL(ii).OP_INPUT(:);
        CONTROL.PARAM.OP_STATE(:,ii) = LIN_MODEL(ii).OP_STATE(:);
        CONTROL.PARAM.OP_OUTPUT(:,ii) = LIN_MODEL(ii).OP_OUTPUT(:);
    end
end

%-------------------------------------------------------------
%% ADAPTIVE CONTROL
%-------------------------------------------------------------
% GAIN SCHEDULING (WALL FOLLOWER)
% / 0. NOT ENABLED / 1. ENABLED
CONTROL.STATE.GAIN_SCHEDULING = uint8(0);
%--------------------------------------------------------------
% FILTER TIME CONSTANT CHANGE
% / 0. NOT ENABLED / 1. ENABLED
CONTROL.STATE.FILTER_CHANGE = uint8(0);
%--------------------------------------------------------------
% SELF-TUNING FORWARD-VELOCITY CONTROL
% / 0. NOT ENABLED / 1. ENABLED
CONTROL.STATE.STR_MODE = uint8(0);
%--------------------------------------------------------------
% ESTIMATED PARAMETERS (STR)
CONTROL.PARAM.VEL_GAIN_EST = MODEL.PARAM.VEL_GAIN;
CONTROL.PARAM.VEL_TIME_CONSTANT_EST = MODEL.PARAM.VEL_TIME_CONSTANT; 
CONTROL.PARAM.FILT_TIME_CONSTANT_EST = 1/2/pi/CONTROL.PARAM.ENC_FILT_FREQ; 
CONTROL.PARAM.FILT_TIME_CONSTANT_ACT = 1/2/pi/CONTROL.PARAM.ENC_FILT_FREQ; 
%--------------------------------------------------------------
% ADAPTATION FLAG (STR)
CONTROL.PARAM.ADPT_FLAG = uint8(0);
%--------------------------------------------------------------
% COMPUTED CONTROL PARAMETERS (STR)
CONTROL.PARAM.FV_PI_STR_K = 1;
CONTROL.PARAM.FV_PI_STR_Ti = 1;
CONTROL.PARAM.FV_PI_STR_b = 1;

%--------------------------------------------------------------
%% COMMUNICATIONS
%--------------------------------------------------------------
% COMMUNICATIONS MODE
% / 0. NOT ENABLED / 1. ENABLED / 2. BLACKBOX (local monitoring)
% / 3. MATLAB
CONTROL.STATE.COMM_MODE = uint8(3);
%--------------------------------------------------------------
% COMMUNICATION SAMPLING TIME
CONTROL.PARAM.COMM_SAMPLING_TIME =  1*MODEL.PARAM.SAMPLING_TIME;
%--------------------------------------------------------------
% UART MODE
% / 0. NOT ENABLED / 1. ENABLED
CONTROL.STATE.UART_MODE = uint8(0);
%--------------------------------------------------------------
% BUFFER INITIALIZATION
CONTROL.PARAM.EMPTY_MSG = zeros(1400,1,'uint8');
%--------------------------------------------------------------
% MCS RX STATUS
CONTROL.STATE.MCS_RX_STATUS = uint8(255);
%--------------------------------------------------------------
% NAV RX STATUS
CONTROL.STATE.NAV_RX_STATUS = uint8(255);
% MCS RX STATUS
CONTROL.STATE.MCS_RX_STATUS = uint8(255);
%--------------------------------------------------------------
%% WIRELESS CHARGING
%--------------------------------------------------------------
% Wireless charging state
% / 0. Charging not enabled / 1. Charging enabled / 2. Charging required
% / 3. Charging initiated
CONTROL.STATE.WIRELESS_CHARGING = uint8(0);
% Battery voltage threshold to start wireless charging (V)
CONTROL.PARAM.WRL_CHRG_VOLT_THR = 0.85*BATTERY_VOLT;
% Charging current threshold to stop wireless charging (A)
CONTROL.PARAM.WRL_CHRG_CURRENT_THR = 0.05;

%--------------------------------------------------------------
%% GENERAL PARAMETERS
%--------------------------------------------------------------
% CONTROL SAMPLING TIME (s)
CONTROL.PARAM.SAMPLING_TIME = ts;
% Control period (s): must be a multiple of the sampling time
CONTROL.PARAM.CONTROL_SAMPLING_TIME = MODEL.PARAM.CONTROL_SAMPLING_TIME;
tc = CONTROL.PARAM.CONTROL_SAMPLING_TIME;
% VEHICLE MODE
% / 0. CAR / 1. SELF-BALANCING VEHICLE
CONTROL.STATE.VEHICLE_MODE = uint8(MODEL.PARAM.VEHICLE_MODE);
% GRAVITY
CONTROL.PARAM.GRAVITY = MODEL.PARAM.GRAVITY; % m/s^2
% FIRST-ORDER MODELS
CONTROL.PARAM.VEL_GAIN = MODEL.PARAM.VEL_GAIN; 
CONTROL.PARAM.VEL_TIME_CONSTANT = MODEL.PARAM.VEL_TIME_CONSTANT; 
CONTROL.PARAM.YAW_RATE_GAIN = MODEL.PARAM.YAW_RATE_GAIN; 
CONTROL.PARAM.YAW_RATE_TIME_CONSTANT = MODEL.PARAM.YAW_RATE_TIME_CONSTANT; 
% MOTOR DIFFERENTIAL VOLTAGE DROP
CONTROL.PARAM.MOTOR_VOLT_DROP_DIF = MODEL.PARAM.MOTOR_VOLT_DROP_DIF;
% IMU
CONTROL.PARAM.IMU_PITCH_OFFSET = MODEL.PARAM.IMU_PITCH_OFFSET;
% RANGE SENSOR
CONTROL.PARAM.RANGE_XA = MODEL.PARAM.RANGE_XA;
CONTROL.PARAM.RANGE_YA = MODEL.PARAM.RANGE_YA;
CONTROL.PARAM.RANGE_SENS_DIST = MODEL.PARAM.RANGE_SENS_DIST;
% ENCODER
CONTROL.PARAM.MOTOR_ENC_RES = MODEL.PARAM.MOTOR_ENC_RES;
% WHEELS
CONTROL.PARAM.WHEEL_GEAR_RATIO = MODEL.PARAM.WHEEL_GEAR_RATIO;
CONTROL.PARAM.MOTOR_GEAR_RATIO = MODEL.PARAM.MOTOR_GEAR_RATIO;
CONTROL.PARAM.WHEEL_DISTANCE = MODEL.PARAM.WHEEL_DISTANCE;
CONTROL.PARAM.WHEEL_RADIUS = MODEL.PARAM.WHEEL_RADIUS;
% MOTOR PWM & DEAD ZONE
CONTROL.PARAM.MOTOR_PWM_SLOPE = [-CONTROL.PARAM.MOTOR_VOLT_MIN(1)/127 ... 
                                  CONTROL.PARAM.MOTOR_VOLT_MAX(1)/128]'; % [- +]
CONTROL.PARAM.MOTOR_DEAD_ZONE = [-0.2119 0.3703]';
% LIDAR 2D MAP: DISTANCE (m) -> 3 deg resolution (0 - 357)
CONTROL.PARAM.LIDAR_2D_MAP = zeros(120,1);
%--------------------------------------------------------------
%% PITCH ANGLE: PID CONTROL
%--------------------------------------------------------------
% YAW ANGLE PID: LIN_MODEL UPDATE
% Control type
% / 1. Control P          /  2. Control PI 
% / 3. Control PD error   /  4. Control PD output
% / 5. Control PID error  /  6. Control PID output
PITCH_ANG_PID.control_type = 6;
% Natural frequency of the dominant poles (rad/s)
PITCH_ANG_PID.natural_freq = 4;
% Damping factor
PITCH_ANG_PID.damping_factor = 0.7;
% Third pole or real pole (rad/s)
PITCH_ANG_PID.real_pole = -40;
% Filtering factor
PITCH_ANG_PID.N = 10;
% Reference weight                     
PITCH_ANG_PID.b = 1; 
%--------------------------------------------------------------
% PITCH ANGLE PID: TRANSFER FUNCTIONS FOR DESIGN
%--------------------------------------------------------------
% PITCH ANGLE MODEL (MOTOR_VOLT_COM -> PITCH_ANG)
if CONTROL.STATE.VEHICLE_MODE == 1
    PITCH_ANG_SS_MODEL = LIN_MODEL(1).PITCH_ANG_SS_MODEL(3,1);
    PITCH_ANG_TF_MODEL = zpk(minreal(PITCH_ANG_SS_MODEL));
    PITCH_ANG_TF_MODEL.DisplayFormat = 'TimeConstant';
    % Transfer function: MOTOR_VOLT_COM -> PITCH_ANG
    % P(s) = Kp*s/(1+Tv*s)/(1-Tw^2*s^2);
    s = tf('s');
    poles = sort(pole(PITCH_ANG_TF_MODEL),'ascend');
    % Kp sign is changed before designing PID
    Kp = -dcgain(minreal(PITCH_ANG_TF_MODEL/s,1e-4));
    [~,ind] = sort(abs(poles),'descend');
    Tv = -1/poles(ind(1));
    Tw = 1/sqrt(abs(poles(ind(2))*poles(ind(3))));
    %--------------------------------------------------------------
    % PITCH ANGLE PID: DESIGN
    %--------------------------------------------------------------
    % Control PD
    % C(s) = K*(1+Td.s)
    % num{1+G(s)} = -Tw^2*Tv*s^3 - (Tw^2 - Kp*K*Td)*s^2 + (Tv + Kp*K)*s + 1
    % There is always a change of sign -> The control systen is always unstable
    %--------------------------------------------------------------
    % Control PID
    % C(s) = K*(1+1/Ti/s+Td*s) = K*(1 + Ti*s + Ti*Td*s^2)/Ti/s
    % num{1+G(s)} = -Tw^2*Tv*Ti*s^3 - (Tw^2 - K*Kp*Td)*Ti*s^2 +
    %             + (Tv + K*Kp)*Ti*s + Ti + K*Kp
    % num{1+G(s)} = s^3 + (1-K*Kp*Td/Tw^2)/Tv*s^2 +
    %             - (1+K*Kp/Tv)/Tw^2*s - (1+K*Kp/Ti)/Tw^2/Tv
    % Polynommial as a function of the desired poles
    % num{1+G(s)} = (s^2 + 2*seta*wn*s + wn^2)*(s-p) =
    %             = s^3 + (2*seta*wn - p)*s^2 + (wn^2 - 2*seta*wn*p)*s - wn^2*p =
    %             = s^3 + a2*s^2 + a1*s + a0
    % Although the closed-loop transfer function is stable, the system is
    % internally unstable. It can be shown by simulating the natural response
    % when the pitch angle is measured with a bias.
    % The common motor voltage and the forward velocity grow unlimited
    %--------------------------------------------------------------
    seta = PITCH_ANG_PID.damping_factor;
    wn = PITCH_ANG_PID.natural_freq;
    p = PITCH_ANG_PID.real_pole;
    a2 = 2*seta*wn-p;
    a1 = wn^2-2*seta*wn*p;
    a0 = -wn^2*p;
    % Equation system with unknown variables: K, Ti, Td
    % (1-K*Kp*Td/Tw^2)/Tv = a2
    % -(1+K*Kp/Tv)/Tw^2 = a1
    % -(1+K*Kp/Ti)/Tw^2/Tv = a0
    K = -(1+a1*Tw^2)*Tv/Kp;
    Ti = -K*Kp/(1+a0*Tv*Tw^2);
    Td = (1-a2*Tv)*Tw^2/K/Kp;
    N = PITCH_ANG_PID.N;
    b = PITCH_ANG_PID.b;
    % K sign is change after designing PID
    K = -K;
    % Control model: state space
    if PITCH_ANG_PID.control_type==5
        a = 1;
    else
        a = 0;
    end
    % dx1 = r - y
    % Td*s/(1+s*Td/N) = Td/(1/s + Td/N)
    % ud/s + Td/N*ud = Td*(a*r-y)
    % dx2 = ud
    % x2 + Td/N*dx2 = Td*a*r - Td*y
    % dx2 = -N/Td*x2 + a*N*r - N*y
    % u = K*(x1/Ti + dx2 + b*r - y) = K*(x1/Ti - N/Td*x2 + (a*N+b)*r - (N+1)*y)
    matAc = [0 0 ; 0 -N/Td];
    matBc = [1 -1; a*N -N];
    matCc = [K/Ti -K*N/Td];
    matDc = [K*(a*N+b)  -K*(N+1)];
    PITCH_ANG_PID.K = K;
    PITCH_ANG_PID.Ti = Ti;
    PITCH_ANG_PID.Td = Td;
    %--------------------------------------------------------------
    % PITCH ANGLE PID: PARAMETERS
    %--------------------------------------------------------------
    CONTROL.PARAM.PITCH_ANG_PID.K = PITCH_ANG_PID.K;
    CONTROL.PARAM.PITCH_ANG_PID.Ti = PITCH_ANG_PID.Ti;
    CONTROL.PARAM.PITCH_ANG_PID.Td = PITCH_ANG_PID.Td;
    CONTROL.PARAM.PITCH_ANG_PID.N = PITCH_ANG_PID.N;
    CONTROL.PARAM.PITCH_ANG_PID.b = PITCH_ANG_PID.b;
    CONTROL.PARAM.PITCH_ANG_PID.INT_DISC_TYPE = uint8(3);
    CONTROL.PARAM.PITCH_ANG_PID.DER_DISC_TYPE = uint8(4);
    CONTROL.PARAM.PITCH_ANG_PID.DER_INPUT = uint8(1);
    CONTROL.PARAM.PITCH_ANG_PID.ANTIWINDUP = uint8(1);
    %--------------------------------------------------------------
    % PITCH ANGLE PID: CLOSED-LOOP STATE-SPACE MODEL
    %--------------------------------------------------------------
    % Plant model: state space
    P_ss = minreal(LIN_MODEL(1).PITCH_ANG_SS_MODEL([3 1],1));
    % Control model: state space
    C_ss = minreal(ss(matAc,matBc,matCc,matDc));
    C_ss.StateName = {'STATE_1_C' 'STATE_2_C'};
    C_ss.InputName = {'PITCH_ANG_REF' 'PITCH_ANG'};
    C_ss.OutputName = {'VOLT_COM_C'};
    % Open-loop state space model
    % It is assumed that the first inputs are the manipulated variables (MV)
    G_ss = minreal(series(C_ss,P_ss,1,1));
    % Feedback
    F_ss = minreal(feedback(G_ss,1,2,1,1));
    % Extra input is deleted
    matA = F_ss.a;
    matB = F_ss.b;
    matC = F_ss.c;
    matD = F_ss.d;
    matB = matB(:,1);
    matC = matC;
    matD = matD(:,1);
    F_ss = minreal(ss(matA,matB,matC,matD));
    % Closed-loop state-space model
    PITCH_ANG_PID_SS_MODEL = minreal(F_ss);
    PITCH_ANG_PID_SS_MODEL.TimeUnit = 'seconds';
    PITCH_ANG_PID_SS_MODEL.InputName = {'PITCH_ANG_REF'};
    PITCH_ANG_PID_SS_MODEL.InputUnit = {'rad'};
    PITCH_ANG_PID_SS_MODEL.OutputName = {'PITCH_ANG','FORWARD_VEL'};
    PITCH_ANG_PID_SS_MODEL.OutputUnit = {'rad','m/s'};
    PITCH_ANG_PID_SS_MODEL.InputGroup.MV = [1];
    PITCH_ANG_PID_SS_MODEL.OutputGroup.MO = [1 2];
    %--------------------------------------------------------------
    % PITCH ANGLE PID: LIN_MODEL UPDATE
    %--------------------------------------------------------------
    for nn = 1:length(LIN_MODEL)
        LIN_MODEL(nn).PITCH_ANG_PID = PITCH_ANG_PID; % PID info
        LIN_MODEL(nn).PITCH_ANG_TF_MODEL = PITCH_ANG_TF_MODEL; % plant model
        LIN_MODEL(nn).PITCH_ANG_PID_SS_MODEL = PITCH_ANG_PID_SS_MODEL;  % closed-loop model
    end
else
    CONTROL.PARAM.PITCH_ANG_PID.K = 0;
    CONTROL.PARAM.PITCH_ANG_PID.Ti = 1;
    CONTROL.PARAM.PITCH_ANG_PID.Td = 0;
    CONTROL.PARAM.PITCH_ANG_PID.N = 1;
    CONTROL.PARAM.PITCH_ANG_PID.b = 1;
    CONTROL.PARAM.PITCH_ANG_PID.INT_DISC_TYPE = uint8(0);
    CONTROL.PARAM.PITCH_ANG_PID.DER_DISC_TYPE = uint8(0);
    CONTROL.PARAM.PITCH_ANG_PID.DER_INPUT = uint8(0);
    CONTROL.PARAM.PITCH_ANG_PID.ANTIWINDUP = uint8(0);
end
%--------------------------------------------------------------
%% FORWARD VELOCITY: CAR PID CONTROL
%--------------------------------------------------------------
% Control type
% / 1. Control P          /  2. Control PI 
% / 3. Control PD error   /  4. Control PD output
% / 5. Control PID error  /  6. Control PID output
FWD_VEL_PID.control_type = 2;
if CONTROL.PARAM.PID_DESIGN_METHOD == 0 % Frequency response
    % Phase margin (deg)
    FWD_VEL_PID.phase_margin_deg = 80;
    % Gain margin (dB)
    FWD_VEL_PID.gain_margin_dB = 0;
    % Design frequency (rad/s): k = w_d/w_d_P
    FWD_VEL_PID.k_wd_P = 0.8;
    % Reference weight
    FWD_VEL_PID.b = 1.2;
    % Lag phase of the integral action (deg)
    FWD_VEL_PID.lag_phase_deg = -5;
    % Filtering factor of the derivative action
    FWD_VEL_PID.f = 0.1;
else % Time response
    % Damping factor
    FWD_VEL_PID.damping_factor = 0.7;
    % Design frequency (rad/s): k = w_d/w_d_P
    FWD_VEL_PID.k_wd_P = 0.8;
    % Reference weight
    FWD_VEL_PID.b = 1;
    % delta = wn*Ti between 1 and 5. Faster and more noisy for 1
    FWD_VEL_PID.delta = 5;
    % Filtering factor of the derivative action
    FWD_VEL_PID.N = 5;    
end
%--------------------------------------------------------------
if CONTROL.STATE.VEHICLE_MODE == 0
    % FORWARD VELOCITY PID: INITIALIZATION
    clear PID_IN PID_OUT
    s = tf('s');
    PID_IN.control_type = FWD_VEL_PID.control_type;
    PID_IN.ts = tc;
    PID_IN.design_method = CONTROL.PARAM.PID_DESIGN_METHOD;
    PID_IN.decoupling_type = 0;
    PID_IN.search_wd_P = true;
    PID_IN.wd_ref = 1;
    PID_IN.D_lower = true;
    PID_IN.int_disc_type = CONTROL.PARAM.PID_FV_DISC_METHOD;
    PID_IN.der_disc_type = CONTROL.PARAM.PID_FV_DISC_METHOD;
    PID_IN.antiwindup = true;
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: TRANSFER FUNCTIONS FOR DESIGN
    FORWARD_VEL_SS_MODEL = ...
        LIN_MODEL(CONTROL.STATE.FORWARD_VEL_MAIN_OP).FORWARD_VEL_SS_MODEL;
    % Additional filter
    if CONTROL.STATE.OBSERVER_MODE
        % EKF
        FORWARD_VEL_SS_MODEL = series(FORWARD_VEL_SS_MODEL,EKF_FLT_SS_MODEL(1,1));
    else
        % ENCODER FILTER
        FORWARD_VEL_SS_MODEL = series(FORWARD_VEL_SS_MODEL,ENC_FLT_SS_MODEL(1,1));
    end
    FORWARD_VEL_TF_MODEL = zpk(minreal(FORWARD_VEL_SS_MODEL));
    FORWARD_VEL_TF_MODEL.DisplayFormat = 'TimeConstant';
    if CONTROL.STATE.SP_MODE(1)==0 && CONTROL.STATE.MOTOR_DELAY_MODE>0
        DELAY = CONTROL.PARAM.MOTOR_DELAY + 0.5*tc*(CONTROL.PARAM.PID_FR_DESIGN_MODEL==1);
        FORWARD_VEL_TF_MODEL = FORWARD_VEL_TF_MODEL*exp(-DELAY*s);
    else
        DELAY = 0.5*tc*(CONTROL.PARAM.PID_DESIGN_METHOD==0 && CONTROL.PARAM.PID_FR_DESIGN_MODEL==1);
        FORWARD_VEL_TF_MODEL = FORWARD_VEL_TF_MODEL*exp(-DELAY*s);
    end
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: DESIGN
    % Plant model
    PID_IN.P = minreal(FORWARD_VEL_TF_MODEL);
    PID_IN.P.InputGroup.MV = 1;
    PID_IN.P.OutputGroup.MO = 1;
    % Specifications
    if CONTROL.PARAM.PID_DESIGN_METHOD == 0 % Frequency response
        PID_IN.gain_margin_dB = FWD_VEL_PID.gain_margin_dB;
        PID_IN.phase_margin_deg = FWD_VEL_PID.phase_margin_deg;
        PID_IN.k_wd_P = FWD_VEL_PID.k_wd_P;
        PID_IN.b = FWD_VEL_PID.b;
        PID_IN.lag_phase_deg = FWD_VEL_PID.lag_phase_deg;
        PID_IN.f = FWD_VEL_PID.f;
    else % Time response
        PID_IN.damping_factor = FWD_VEL_PID.damping_factor;
        PID_IN.k_wd_P = FWD_VEL_PID.k_wd_P;
        PID_IN.delta = FWD_VEL_PID.delta;
        PID_IN.N = FWD_VEL_PID.N;
        PID_IN.b = FWD_VEL_PID.b;
    end
    PID_OUT = DESIGN_PID(PID_IN);
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: PARAMETERS
    PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
    CONTROL.PARAM.FORWARD_VEL_PID.K = PID_OUT.K;
    CONTROL.PARAM.FORWARD_VEL_PID.Ti = PID_OUT.Ti;
    CONTROL.PARAM.FORWARD_VEL_PID.Td = PID_OUT.Td;
    CONTROL.PARAM.FORWARD_VEL_PID.N = PID_OUT.N;
    CONTROL.PARAM.FORWARD_VEL_PID.b = PID_OUT.b;
    CONTROL.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
    CONTROL.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
    CONTROL.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(PID_OUT.der_input);
    CONTROL.PARAM.FORWARD_VEL_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: CLOSED-LOOP STATE-SPACE MODEL
    % Closed-loop state-space model
    FWD_VEL_PID_SS_MODEL = minreal(PID_OUT.F_ss);
    FWD_VEL_PID_SS_MODEL.TimeUnit = 'seconds';
    FWD_VEL_PID_SS_MODEL.InputName = {'FORWARD VEL REF'};
    FWD_VEL_PID_SS_MODEL.InputUnit = {'m/s'};
    FWD_VEL_PID_SS_MODEL.OutputName = {'FORWARD VEL'};
    FWD_VEL_PID_SS_MODEL.OutputUnit = {'m/s'};
    FWD_VEL_PID_SS_MODEL.InputGroup.MV = [1];
    FWD_VEL_PID_SS_MODEL.OutputGroup.MO = [1];
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: LIN_MODEL UPDATE
    for nn = 1:length(LIN_MODEL)
        LIN_MODEL(nn).FORWARD_VEL_PID = PID_OUT;
        LIN_MODEL(nn).FORWARD_VEL_TF_MODEL = FORWARD_VEL_TF_MODEL;
        LIN_MODEL(nn).FORWARD_VEL_PID_SS_MODEL = FWD_VEL_PID_SS_MODEL;
    end
end

%--------------------------------------------------------------
%% FORWARD VELOCITY: SBV PID CONTROL
%--------------------------------------------------------------
% Control type
% / 1. Control P          /  2. Control PI 
% / 3. Control PD error   /  4. Control PD output
% / 5. Control PID error  /  6. Control PID output
FWD_VEL_PID.control_type = 4;
% Gain margin (deg)
FWD_VEL_PID.gain_margin_dB = 12;
% Design frequency (rad/s): k = w_d/w_d_P                                          
FWD_VEL_PID.k_wd_P = 1.3;                                                                 
% Reference weight                     
FWD_VEL_PID.b = 1.00;
% Lag phase of the integral action (deg)                     
FWD_VEL_PID.lag_phase_deg = -5; 
% Filtering factor of the derivative action
FWD_VEL_PID.f = 0.1;   
%--------------------------------------------------------------
if CONTROL.STATE.VEHICLE_MODE == 1
    % FORWARD VELOCITY PID: INITIALIZATION
    clear PID_IN PID_OUT
    PID_IN.control_type = FWD_VEL_PID.control_type;
    PID_IN.ts = CONTROL.PARAM.SAMPLING_TIME;
    PID_IN.design_method = 0;
    PID_IN.decoupling_type = 0;
    PID_IN.phase_margin_deg = 0;
    PID_IN.search_wd_P = true;
    PID_IN.wd_ref = 1;
    PID_IN.D_lower = true;
    PID_IN.int_disc_type = 3;
    PID_IN.der_disc_type = 0;
    PID_IN.antiwindup = true;
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: TRANSFER FUNCTIONS FOR DESIGN
    %--------------------------------------------------------------
    % FORWARD VELOCITY MODEL (PITCH_ANG_REF -> FORWARD_VEL)
    FORWARD_VEL_SS_MODEL = minreal(PITCH_ANG_PID_SS_MODEL(2,1));
    % Additional filter
    if CONTROL.STATE.OBSERVER_MODE
        % EKF
        FORWARD_VEL_SS_MODEL = series(FORWARD_VEL_SS_MODEL,EKF_FLT_SS_MODEL(1,1));
    else
        % ENCODER FILTER
        FORWARD_VEL_SS_MODEL = series(FORWARD_VEL_SS_MODEL,ENC_FLT_SS_MODEL(1,1));
    end
    FORWARD_VEL_TF_MODEL = zpk(minreal(FORWARD_VEL_SS_MODEL));
    FORWARD_VEL_TF_MODEL.DisplayFormat = 'TimeConstant';
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: DESIGN
    %--------------------------------------------------------------
    % Plant model
    PID_IN.P = minreal(FORWARD_VEL_SS_MODEL);
    PID_IN.P.InputGroup.MV = 1;
    PID_IN.P.OutputGroup.MO = 1;
    % Specifications
    PID_IN.gain_margin_dB = FWD_VEL_PID.gain_margin_dB;
    PID_IN.k_wd_P = FWD_VEL_PID.k_wd_P;
    PID_IN.b = FWD_VEL_PID.b;
    PID_IN.lag_phase_deg = FWD_VEL_PID.lag_phase_deg;
    PID_IN.f = FWD_VEL_PID.f;
    %--------------------------------------------------------------
    PID_OUT = DESIGN_PID(PID_IN);
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: PARAMETERS
    %--------------------------------------------------------------
    try
        PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
        CONTROL.PARAM.FORWARD_VEL_PID.K = PID_OUT.K;
        CONTROL.PARAM.FORWARD_VEL_PID.Ti = PID_OUT.Ti;
        CONTROL.PARAM.FORWARD_VEL_PID.Td = PID_OUT.Td;
        CONTROL.PARAM.FORWARD_VEL_PID.N = PID_OUT.N;
        CONTROL.PARAM.FORWARD_VEL_PID.b = PID_OUT.b;
        CONTROL.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
        CONTROL.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
        CONTROL.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(PID_OUT.der_input);
        CONTROL.PARAM.FORWARD_VEL_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
    catch
        CONTROL.PARAM.FORWARD_VEL_PID.K = 1;
        CONTROL.PARAM.FORWARD_VEL_PID.Ti = 1;
        CONTROL.PARAM.FORWARD_VEL_PID.Td = 0;
        CONTROL.PARAM.FORWARD_VEL_PID.N = 1;
        CONTROL.PARAM.FORWARD_VEL_PID.b = 1;
        CONTROL.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE = uint8(3);
        CONTROL.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE = uint8(3);
        CONTROL.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(0);
        CONTROL.PARAM.FORWARD_VEL_PID.ANTIWINDUP = uint8(0);
    end
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: CLOSED-LOOP STATE-SPACE MODEL
    %--------------------------------------------------------------
    % Closed-loop state-space model
    try
        FWD_VEL_PID_SS_MODEL = minreal(PID_OUT.F_ss);
    catch
        FWD_VEL_PID_SS_MODEL = [];
    end
    FWD_VEL_PID_SS_MODEL.TimeUnit = 'seconds';
    FWD_VEL_PID_SS_MODEL.InputName = {'FORWARD VEL REF'};
    FWD_VEL_PID_SS_MODEL.InputUnit = {'m/s'};
    FWD_VEL_PID_SS_MODEL.OutputName = {'FORWARD VEL'};
    FWD_VEL_PID_SS_MODEL.OutputUnit = {'m/s'};
    FWD_VEL_PID_SS_MODEL.InputGroup.MV = [1];
    FWD_VEL_PID_SS_MODEL.OutputGroup.MO = [1];
    %--------------------------------------------------------------
    % FORWARD VELOCITY PID: LIN_MODEL UPDATE
    %--------------------------------------------------------------
    for nn = 1:length(LIN_MODEL)
        LIN_MODEL(nn).FORWARD_VEL_PID = PID_OUT;
        LIN_MODEL(nn).FORWARD_VEL_TF_MODEL = FORWARD_VEL_TF_MODEL;
        LIN_MODEL(nn).FORWARD_VEL_PID_SS_MODEL = FWD_VEL_PID_SS_MODEL;
    end
end
%--------------------------------------------------------------
%% YAW RATE: PID CONTROL
%--------------------------------------------------------------
% Control type
% / 1. Control P          /  2. Control PI 
% / 3. Control PD error   /  4. Control PD output
% / 5. Control PID error  /  6. Control PID output
YAW_RATE_PID.control_type = 2;
if CONTROL.PARAM.PID_DESIGN_METHOD == 0 % Frequency response
    % Phase margin (deg)
    YAW_RATE_PID.phase_margin_deg = 85;
    % Gain margin (dB)
    YAW_RATE_PID.gain_margin_dB = 0;
    % Design frequency (rad/s): k = w_d/w_d_P
    YAW_RATE_PID.k_wd_P = 0.8;
    % Reference weight
    YAW_RATE_PID.b = 1.3;
    % Lag phase of the integral action (deg)
    YAW_RATE_PID.lag_phase_deg = -10;
    % Filtering factor of the derivative action
    YAW_RATE_PID.f = 0.1;
else % Time response
    % Damping factor
    YAW_RATE_PID.damping_factor = 0.9;
    % Design frequency (rad/s): k = w_d/w_d_P
    YAW_RATE_PID.k_wd_P = 0.8;
    % Reference weight
    YAW_RATE_PID.b = 1.2;
    % delta = wn*Ti between 1 and 5. Faster and more noisy for 1
    YAW_RATE_PID.delta = 2.5;
    % Filtering factor of the derivative action
    YAW_RATE_PID.N = 5;    
end
%--------------------------------------------------------------
% YAW RATE PID: INITIALIZATION
clear PID_IN PID_OUT
PID_IN.control_type = YAW_RATE_PID.control_type;
PID_IN.ts = tc;
PID_IN.design_method = CONTROL.PARAM.PID_DESIGN_METHOD;
PID_IN.decoupling_type = 0;
PID_IN.search_wd_P = true;
PID_IN.wd_ref = 1;
PID_IN.D_lower = true;
PID_IN.int_disc_type = 3;
PID_IN.der_disc_type = 3; 
PID_IN.antiwindup = true;
%--------------------------------------------------------------
% YAW RATE PID: TRANSFER FUNCTIONS FOR DESIGN
% YAW MODEL (MOTOR_VOLT_DIF -> YAW_ANG)
YAW_RATE_SS_MODEL = LIN_MODEL(1).YAW_RATE_SS_MODEL;
% Additional filter
if CONTROL.STATE.ROTATION_MSRT_MODE
    % ENCODER
    ADD_FLT_SS_MODEL = ENC_FLT_SS_MODEL;
else
    % IMU
    ADD_FLT_SS_MODEL = IMU_FLT_SS_MODEL;
end
if CONTROL.STATE.OBSERVER_MODE
    % EKF
    YAW_RATE_SS_MODEL = series(YAW_RATE_SS_MODEL,EKF_FLT_SS_MODEL(1,1));    
else
    % FILTER
    YAW_RATE_SS_MODEL = series(YAW_RATE_SS_MODEL,ADD_FLT_SS_MODEL(1,1));
end
YAW_RATE_TF_MODEL = zpk(minreal(YAW_RATE_SS_MODEL));
YAW_RATE_TF_MODEL.DisplayFormat = 'TimeConstant';
if CONTROL.STATE.SP_MODE(2)==0 && CONTROL.STATE.MOTOR_DELAY_MODE>0 && CONTROL.STATE.VEHICLE_MODE==0
    DELAY = CONTROL.PARAM.MOTOR_DELAY + 0.5*tc*(CONTROL.PARAM.PID_FR_DESIGN_MODEL==1);
    YAW_RATE_TF_MODEL = YAW_RATE_TF_MODEL*exp(-DELAY*s);  
else
    DELAY = 0.5*tc*(CONTROL.PARAM.PID_DESIGN_METHOD==0 && CONTROL.PARAM.PID_FR_DESIGN_MODEL==1);
    YAW_RATE_TF_MODEL = YAW_RATE_TF_MODEL*exp(-DELAY*s);     
end
%--------------------------------------------------------------
% YAW RATE PID: DESIGN
% Plant model
PID_IN.P = YAW_RATE_TF_MODEL;
PID_IN.P.InputGroup.MV = 1;
PID_IN.P.OutputGroup.MO = 1;
% Specifications
if CONTROL.PARAM.PID_DESIGN_METHOD == 0 % Frequency response
    PID_IN.gain_margin_dB = YAW_RATE_PID.gain_margin_dB;
    PID_IN.phase_margin_deg = YAW_RATE_PID.phase_margin_deg;
    PID_IN.k_wd_P = YAW_RATE_PID.k_wd_P;
    PID_IN.b = YAW_RATE_PID.b;
    PID_IN.lag_phase_deg = YAW_RATE_PID.lag_phase_deg;
    PID_IN.f = YAW_RATE_PID.f;
else % Time response
    PID_IN.damping_factor = YAW_RATE_PID.damping_factor;
    PID_IN.k_wd_P = YAW_RATE_PID.k_wd_P;
    PID_IN.delta = YAW_RATE_PID.delta;
    PID_IN.N = YAW_RATE_PID.N;
    PID_IN.b = YAW_RATE_PID.b;
end
PID_OUT = DESIGN_PID(PID_IN);
%--------------------------------------------------------------
% YAW RATE PID: PARAMETERS
PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
CONTROL.PARAM.YAW_RATE_PID.K = PID_OUT.K;
CONTROL.PARAM.YAW_RATE_PID.Ti = PID_OUT.Ti;
CONTROL.PARAM.YAW_RATE_PID.Td = PID_OUT.Td;
CONTROL.PARAM.YAW_RATE_PID.N = PID_OUT.N;
CONTROL.PARAM.YAW_RATE_PID.b = PID_OUT.b;
CONTROL.PARAM.YAW_RATE_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
CONTROL.PARAM.YAW_RATE_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
CONTROL.PARAM.YAW_RATE_PID.DER_INPUT = uint8(PID_OUT.der_input);
CONTROL.PARAM.YAW_RATE_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
%--------------------------------------------------------------
% YAW RATE PID: CLOSED-LOOP STATE-SPACE MODEL
% Closed-loop state-space model
TimeUnit = 'seconds';
InputName = {'YAW RATE REF'};
InputUnit = {'rad/s'};
OutputName = {'YAW RATE'};
OutputUnit = {'rad/s'};
YAW_RATE_PID_SS_MODEL = minreal(PID_OUT.F_ss);
YAW_RATE_PID_SS_MODEL.TimeUnit = TimeUnit;
YAW_RATE_PID_SS_MODEL.InputName = InputName;
YAW_RATE_PID_SS_MODEL.InputUnit = InputUnit;
YAW_RATE_PID_SS_MODEL.OutputName = OutputName;
YAW_RATE_PID_SS_MODEL.OutputUnit = OutputUnit;
YAW_RATE_PID_SS_MODEL.InputGroup.MV = [1];
YAW_RATE_PID_SS_MODEL.OutputGroup.MO = [1];
%--------------------------------------------------------------
% YAW RATE PID: LIN_MODEL UPDATE
for nn = 1:length(LIN_MODEL)
    LIN_MODEL(nn).YAW_RATE_PID = PID_OUT;
    LIN_MODEL(nn).YAW_RATE_TF_MODEL = YAW_RATE_TF_MODEL;
    LIN_MODEL(nn).YAW_RATE_PID_SS_MODEL = YAW_RATE_PID_SS_MODEL;
end
%--------------------------------------------------------------
%% YAW ANGLE: PID CONTROL
%--------------------------------------------------------------
% Control type
% / 1. Control P          /  2. Control PI 
% / 3. Control PD error   /  4. Control PD output
% / 5. Control PID error  /  6. Control PID output
YAW_ANG_PID.control_type = 2;
% Phase margin (deg)
YAW_ANG_PID.phase_margin_deg = 60;
% Gain margin (dB)
YAW_ANG_PID.gain_margin_dB = 0;
% Design frequency (rad/s): k = w_d/w_d_P                                          
YAW_ANG_PID.k_wd_P = 0.9;                                                                 
% Reference weight                     
YAW_ANG_PID.b = 1;
% Lag phase of the integral action (deg)                     
YAW_ANG_PID.lag_phase_deg = -10; 
% Filtering factor of the derivative action
YAW_ANG_PID.f = 0.1;   
%--------------------------------------------------------------
% YAW ANGLE PID: INITIALIZATION
clear PID_IN PID_OUT
PID_IN.control_type = YAW_ANG_PID.control_type;
PID_IN.ts = tc;
PID_IN.design_method = 0;
PID_IN.decoupling_type = 0;
PID_IN.search_wd_P = true;
PID_IN.wd_ref = 1;
PID_IN.D_lower = true;
PID_IN.int_disc_type = 3;
PID_IN.der_disc_type = 3; 
PID_IN.antiwindup = true;
%--------------------------------------------------------------
% YAW ANGLE PID: TRANSFER FUNCTIONS FOR DESIGN
% YAW MODEL (MOTOR_VOLT_DIF -> YAW_ANG)
YAW_ANG_SS_MODEL = LIN_MODEL(1).YAW_ANG_SS_MODEL;
% Additional filter
if CONTROL.STATE.CONTROL_MODE==4 || CONTROL.STATE.CONTROL_MODE==5
    % RANGE
    ADD_FLT_SS_MODEL = RNG_FLT_SS_MODEL;
else
    if CONTROL.STATE.ROTATION_MSRT_MODE
        % ENCODER
        ADD_FLT_SS_MODEL = ENC_FLT_SS_MODEL;
    else
        % IMU
        ADD_FLT_SS_MODEL = IMU_FLT_SS_MODEL;
    end
end
if CONTROL.STATE.OBSERVER_MODE
    % EKF
    YAW_ANG_SS_MODEL = series(YAW_ANG_SS_MODEL,EKF_FLT_SS_MODEL(1,1));    
else
    % FILTER
    YAW_ANG_SS_MODEL = series(YAW_ANG_SS_MODEL,ADD_FLT_SS_MODEL(1,1));
end
YAW_ANG_TF_MODEL = zpk(minreal(YAW_ANG_SS_MODEL));
YAW_ANG_TF_MODEL.DisplayFormat = 'TimeConstant';
if CONTROL.STATE.MOTOR_DELAY_MODE>0
    DELAY = CONTROL.PARAM.MOTOR_DELAY;
    YAW_ANG_TF_MODEL = YAW_ANG_TF_MODEL*exp(-DELAY*s);  
end
%--------------------------------------------------------------
% YAW ANGLE PID: DESIGN
% Plant model
PID_IN.P = YAW_ANG_TF_MODEL;
PID_IN.P.InputGroup.MV = 1;
PID_IN.P.OutputGroup.MO = 1;
% Specifications
PID_IN.phase_margin_deg = YAW_ANG_PID.phase_margin_deg;
PID_IN.gain_margin_dB = YAW_ANG_PID.gain_margin_dB;
PID_IN.k_wd_P = YAW_ANG_PID.k_wd_P;
PID_IN.b = YAW_ANG_PID.b;
PID_IN.lag_phase_deg = YAW_ANG_PID.lag_phase_deg;
PID_IN.f = YAW_ANG_PID.f;
PID_OUT = DESIGN_PID(PID_IN);
%--------------------------------------------------------------
% YAW ANGLE PID: PARAMETERS
PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
CONTROL.PARAM.YAW_ANG_PID.K = PID_OUT.K;
CONTROL.PARAM.YAW_ANG_PID.Ti = PID_OUT.Ti;
CONTROL.PARAM.YAW_ANG_PID.Td = PID_OUT.Td;
CONTROL.PARAM.YAW_ANG_PID.N = PID_OUT.N;
CONTROL.PARAM.YAW_ANG_PID.b = PID_OUT.b;
CONTROL.PARAM.YAW_ANG_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
CONTROL.PARAM.YAW_ANG_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
CONTROL.PARAM.YAW_ANG_PID.DER_INPUT = uint8(PID_OUT.der_input);
CONTROL.PARAM.YAW_ANG_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
%--------------------------------------------------------------
% YAW ANGLE PID: CLOSED-LOOP STATE-SPACE MODEL
% Closed-loop state-space model
TimeUnit = 'seconds';
InputName = {'YAW ANG REF'};
InputUnit = {'rad'};
OutputName = {'YAW ANG'};
OutputUnit = {'rad'};
YAW_ANG_PID_SS_MODEL = minreal(PID_OUT.F_ss);
YAW_ANG_PID_SS_MODEL.TimeUnit = TimeUnit;
YAW_ANG_PID_SS_MODEL.InputName = InputName;
YAW_ANG_PID_SS_MODEL.InputUnit = InputUnit;
YAW_ANG_PID_SS_MODEL.OutputName = OutputName;
YAW_ANG_PID_SS_MODEL.OutputUnit = OutputUnit;
YAW_ANG_PID_SS_MODEL.InputGroup.MV = [1];
YAW_ANG_PID_SS_MODEL.OutputGroup.MO = [1];
%--------------------------------------------------------------
% YAW ANGLE PID: LIN_MODEL UPDATE
for nn = 1:length(LIN_MODEL)
    LIN_MODEL(nn).YAW_ANG_PID = PID_OUT;
    LIN_MODEL(nn).YAW_ANG_TF_MODEL = YAW_ANG_TF_MODEL;
    LIN_MODEL(nn).YAW_ANG_PID_SS_MODEL = YAW_ANG_PID_SS_MODEL;
end
clear PID_IN PID_OUT

%--------------------------------------------------------------
%% FORWARD VELOCITY: DEAD BEAT
%--------------------------------------------------------------
z = tf('z',tc);
% Plant state space model
FORWARD_VEL_SS_MODEL = ...
    LIN_MODEL(CONTROL.STATE.FORWARD_VEL_MAIN_OP).FORWARD_VEL_SS_MODEL;
% Additional encoder filter
FORWARD_VEL_SS_MODEL = series(FORWARD_VEL_SS_MODEL,ENC_FLT_SS_MODEL(1,1));
% Transfer function for dead beat
Pv = tf(minreal(FORWARD_VEL_SS_MODEL));
% Discretization
Pv_dis = c2d(Pv,tc,'zoh');
% ZPK format
zpk(Pv_dis);
%----------------------------------
% First-order dead beat
if CONTROL.STATE.FORWARD_VEL_CONTROL_TYPE == 2
    % Closed-loop transfer function
    Fv_dis=1/z;
end
%----------------------------------
% Second-order dead beat
if CONTROL.STATE.FORWARD_VEL_CONTROL_TYPE == 3
    % Plant zero
    cero=zero(Pv_dis);
    % Closed-loop transfer function
    Fv_dis=(z-cero)/z^2/(1-cero);
end
%-------------------------------
if CONTROL.STATE.FORWARD_VEL_CONTROL_TYPE >= 2
    % Open-loop transfer function
    Gv_dis=Fv_dis/(1-Fv_dis);
    % Control transfer function
    Cv_db=minreal(Gv_dis/Pv_dis);
    % Numerator and denominator
    [numDB,denDB] = tfdata(Cv_db,'v');
    CONTROL.PARAM.FORWARD_VEL_DB.NUM = numDB(1:3)/denDB(1);
    CONTROL.PARAM.FORWARD_VEL_DB.DEN = denDB(2:3)/denDB(1);    
else
    CONTROL.PARAM.FORWARD_VEL_DB.NUM = zeros(1,3);
    CONTROL.PARAM.FORWARD_VEL_DB.DEN = zeros(1,2);    
end

%--------------------------------------------------------------
%% YAW RATE: DEAD BEAT
%--------------------------------------------------------------
z = tf('z',tc);
% Plant state space model
YAW_RATE_SS_MODEL = LIN_MODEL(1).YAW_RATE_SS_MODEL;
% Additional encoder filter
YAW_RATE_SS_MODEL = series(YAW_RATE_SS_MODEL,ENC_FLT_SS_MODEL(1,1));
% Transfer function for dead beat
Pv = tf(minreal(YAW_RATE_SS_MODEL));
% Discretization
Pv_dis = c2d(Pv,tc,'zoh');
% ZPK format
zpk(Pv_dis);
%----------------------------------
% First-order dead beat
if CONTROL.STATE.YAW_RATE_CONTROL_TYPE == 2
    % Closed-loop transfer function
    Fv_dis=1/z;
end
%----------------------------------
% Second-order dead beat
if CONTROL.STATE.YAW_RATE_CONTROL_TYPE == 3
    % Plant zero
    cero=zero(Pv_dis);
    % Closed-loop transfer function
    Fv_dis=(z-cero)/z^2/(1-cero);
end
%-------------------------------
if CONTROL.STATE.YAW_RATE_CONTROL_TYPE >= 2
    % Open-loop transfer function
    Gv_dis=Fv_dis/(1-Fv_dis);
    % Control transfer function
    Cv_db=minreal(Gv_dis/Pv_dis);
    % Numerator and denominator
    [numDB,denDB] = tfdata(Cv_db,'v');
    CONTROL.PARAM.YAW_RATE_DB.NUM = numDB(1:3)/denDB(1);
    CONTROL.PARAM.YAW_RATE_DB.DEN = denDB(2:3)/denDB(1);    
else
    CONTROL.PARAM.YAW_RATE_DB.NUM = zeros(1,3);
    CONTROL.PARAM.YAW_RATE_DB.DEN = zeros(1,2);    
end

%--------------------------------------------------------------
%% WALL FOLLOWER: SINGLE-LOOP PID
%--------------------------------------------------------------
% Control type
% / 1. Control P          /  2. Control PI 
% / 3. Control PD error   /  4. Control PD output
% / 5. Control PID error  /  6. Control PID output
WFL_SL_PID.control_type = 4;
% Phase margin (deg)
WFL_SL_PID.phase_margin_deg = [60 20 20 20];
% Design frequency (rad/s): k = w_d/w_d_P                                          
WFL_SL_PID.k_wd_P = [1.5 1.5 1.5 1.5];                                                                 
% Reference weight                     
WFL_SL_PID.b = [1.00 1.00 1.00 1.00];
% Lag phase of the integral action (deg)                     
WFL_SL_PID.lag_phase_deg = [-5 -5 -5 -5]; 
% Filtering factor of the derivative action
WFL_SL_PID.f = [0.1 0.1 0.1 0.1];   
%--------------------------------------------------------------
% WALL FOLLOWER SL PID: INITIALIZATION
clear PID_IN PID_OUT
% Control parameters
N = length(LIN_MODEL);
CONTROL.PARAM.WFL_SL_PID.K = zeros(N,1);
CONTROL.PARAM.WFL_SL_PID.Ti = zeros(N,1);
CONTROL.PARAM.WFL_SL_PID.Td = zeros(N,1);
CONTROL.PARAM.WFL_SL_PID.N = zeros(N,1);
CONTROL.PARAM.WFL_SL_PID.b = zeros(N,1);
PID_IN.ts = tc;
PID_IN.control_type = WFL_SL_PID.control_type;
PID_IN.design_method = 0;
PID_IN.decoupling_type = 0;
PID_IN.gain_margin_dB = 0;
if WFL_SL_PID.control_type>2
    PID_IN.search_wd_P = false;
else
    PID_IN.search_wd_P = true;
end
PID_IN.wd_ref = 1;
PID_IN.D_lower = true;
PID_IN.int_disc_type = 3;
PID_IN.der_disc_type = 3; 
PID_IN.antiwindup = true;
%--------------------------------------------------------------
% WALL FOLLOWER SL PID: DESIGN FOR EACH OPERATING POINT
for ii = 1:N
    clc
    fprintf('WALL FOLLOWER PID DESIGN: OP %d/%d \n',ii,N);    
    % Plant model
    WFL_SL_SS_MODEL = LIN_MODEL(ii).WFL_SL_SS_MODEL;
    % Additional filter (RANGE)
    ADD_FLT_SS_MODEL = RNG_FLT_SS_MODEL;
    if CONTROL.STATE.OBSERVER_MODE
        % EKF
        WFL_SL_SS_MODEL = series(WFL_SL_SS_MODEL,EKF_FLT_SS_MODEL(1,1));
    else
        % FILTER
        WFL_SL_SS_MODEL = series(WFL_SL_SS_MODEL,ADD_FLT_SS_MODEL(1,1));
    end    
    PID_IN.P = WFL_SL_SS_MODEL(3,1);
    WFL_SL_TF_MODEL = zpk(minreal(WFL_SL_SS_MODEL(3,1)));
    WFL_SL_TF_MODEL.DisplayFormat = 'TimeConstant';
    LIN_MODEL(ii).WFL_SL_TF_MODEL = WFL_SL_TF_MODEL;
    PID_IN.P.InputGroup.MV = 1;
    PID_IN.P.OutputGroup.MO = 1;
    % Specifications
    PID_IN.phase_margin_deg = WFL_SL_PID.phase_margin_deg(ii);
    PID_IN.k_wd_P = WFL_SL_PID.k_wd_P(ii);
    PID_IN.b = WFL_SL_PID.b(ii);
    PID_IN.lag_phase_deg = WFL_SL_PID.lag_phase_deg(ii);
    PID_IN.f = WFL_SL_PID.f(ii);
    PID_OUT = DESIGN_PID(PID_IN);
    LIN_MODEL(ii).WFL_SL_PID = PID_OUT;
    try
        PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
        CONTROL.PARAM.WFL_SL_PID.K(ii,1) = PID_OUT.K;
        CONTROL.PARAM.WFL_SL_PID.Ti(ii,1) = PID_OUT.Ti;
        CONTROL.PARAM.WFL_SL_PID.Td(ii,1) = PID_OUT.Td;
        CONTROL.PARAM.WFL_SL_PID.N(ii,1) = PID_OUT.N;
        CONTROL.PARAM.WFL_SL_PID.b(ii,1) = PID_OUT.b;
        % CLOSED-LOOP MODEL
        WFL_SL_PID_SS_MODEL = minreal(PID_OUT.F_ss);
        LIN_MODEL(ii).WFL_SL_PID_SS_MODEL = WFL_SL_PID_SS_MODEL;
    catch
        CONTROL.PARAM.WFL_SL_PID.K(ii,1) = 0;
        CONTROL.PARAM.WFL_SL_PID.Ti(ii,1) = 1;
        CONTROL.PARAM.WFL_SL_PID.Td(ii,1) = 0;
        CONTROL.PARAM.WFL_SL_PID.N(ii,1) = 1;
        CONTROL.PARAM.WFL_SL_PID.b(ii,1) = 1;        
    end
end
CONTROL.PARAM.WFL_SL_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
CONTROL.PARAM.WFL_SL_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
CONTROL.PARAM.WFL_SL_PID.DER_INPUT = uint8(PID_OUT.der_input);
CONTROL.PARAM.WFL_SL_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
%--------------------------------------------------------------
%% WALL FOLLOWER: CASCADE CONTROL
%--------------------------------------------------------------
% Control type
% / 1. Control P          /  2. Control PI 
% / 3. Control PD error   /  4. Control PD output
% / 5. Control PID error  /  6. Control PID output
WFL_CD_PID.control_type = 6;
% Phase margin (deg)
WFL_CD_PID.phase_margin_deg = 50*[1 1 1 1];
% Design frequency (rad/s): k = w_d/w_d_P                                          
WFL_CD_PID.k_wd_P = 1.5*[1 1 1 1];                                                                 
% Reference weight                     
WFL_CD_PID.b = [1.00 1.00 1.00 1.00];
% Lag phase of the integral action (deg)                     
WFL_CD_PID.lag_phase_deg = [-5 -5 -5 -5]; 
% Filtering factor of the derivative action
WFL_CD_PID.f = [0.1 0.1 0.1 0.1];   
%-------------------------------------------------------------
% WALL FOLLOWER CD PID: MODEL FOR DESIGN
% P = Fry_yaw*vo*(xA/vo*s+1)/s = Fry*P1
% P1 = vo*(xA/vo*s+1)/s = xA + vo/s 
% dn = xA*yaw + z
% dX/dt = A*X + B*yaw_ref
% dz/dt = vo*yaw = vo*(C*X + D*yaw_ref)
% dn = z + xA*yaw = z + xA*(C*X + D*yaw_ref)
% State-space model 
matA_aux = YAW_ANG_PID_SS_MODEL.a;
matB_aux = YAW_ANG_PID_SS_MODEL.b;
matC_aux = YAW_ANG_PID_SS_MODEL.c;
matD_aux = YAW_ANG_PID_SS_MODEL.d;
nx = size(matB_aux,1);
xA = MODEL.PARAM.RANGE_XA;
for nn = 1:length(LIN_MODEL)
    vo = CONTROL.PARAM.OP_STATE(1,nn);
    matA = [matA_aux zeros(nx,1) ; vo*matC_aux 0];
    matB = [matB_aux ; vo*matD_aux];
    matC = [-xA*matC_aux -1];
    matD = -xA*matD_aux;
    AUX1_SS = ss(matA,matB,matC,matD);
    InputName = {'YAW ANG REF'};
    InputUnit = {'rad'};
    OutputName = {'WALL DIST'};
    OutputUnit = {'m'};
    % Model definition
    % Additional filter (RANGE)
    ADD_FLT_SS_MODEL = RNG_FLT_SS_MODEL;
    if CONTROL.STATE.OBSERVER_MODE
        % EKF
        AUX1_SS = series(AUX1_SS,EKF_FLT_SS_MODEL(1,1));
    else
        % FILTER
        AUX1_SS = series(AUX1_SS,ADD_FLT_SS_MODEL(1,1));
    end        
    WFL_CD_SS_MODEL = AUX1_SS;
    WFL_CD_SS_MODEL.TimeUnit = TimeUnit;
    WFL_CD_SS_MODEL.InputName = InputName;
    WFL_CD_SS_MODEL.InputUnit = InputUnit;
    WFL_CD_SS_MODEL.OutputName = OutputName;
    WFL_CD_SS_MODEL.OutputUnit = OutputUnit;
    WFL_CD_SS_MODEL.InputGroup.MV = 1;
    WFL_CD_SS_MODEL.OutputGroup.MO = 1;
    WFL_CD_TF_MODEL = zpk(minreal(WFL_CD_SS_MODEL));
    WFL_CD_TF_MODEL.DisplayFormat = 'TimeConstant';
    LIN_MODEL(nn).WFL_CD_SS_MODEL = WFL_CD_SS_MODEL;    
    LIN_MODEL(nn).WFL_CD_TF_MODEL = WFL_CD_TF_MODEL;
end
%--------------------------------------------------------------
% WALL FOLLOWER CD PID: INITIALIZATION
clear PID_IN PID_OUT
% Control parameters
N = length(LIN_MODEL);
CONTROL.PARAM.WFL_CD_PID.K = zeros(N,1);
CONTROL.PARAM.WFL_CD_PID.Ti = zeros(N,1);
CONTROL.PARAM.WFL_CD_PID.Td = zeros(N,1);
CONTROL.PARAM.WFL_CD_PID.N = zeros(N,1);
CONTROL.PARAM.WFL_CD_PID.b = zeros(N,1);
PID_IN.ts = tc;
PID_IN.control_type = WFL_CD_PID.control_type;
PID_IN.design_method = 0;
PID_IN.decoupling_type = 0;
PID_IN.gain_margin_dB = 0;
PID_IN.search_wd_P = true;
PID_IN.wd_ref = 5;
PID_IN.D_lower = true;
PID_IN.int_disc_type = 3;
PID_IN.der_disc_type = 3; 
PID_IN.antiwindup = true;
%--------------------------------------------------------------
% WALL FOLLOWER CD PID: DESIGN FOR EACH OPERATING POINT
for ii = 1:N
    clc
    fprintf('WALL FOLLOWER CASCADE DESIGN: OP %d/%d \n',ii,N); 
    % Plant model
    PID_IN.P = LIN_MODEL(ii).WFL_CD_SS_MODEL; 
    % Specifications
    PID_IN.phase_margin_deg = WFL_CD_PID.phase_margin_deg(ii);
    PID_IN.k_wd_P = WFL_CD_PID.k_wd_P(ii);
    PID_IN.b = WFL_CD_PID.b(ii);
    PID_IN.lag_phase_deg = WFL_CD_PID.lag_phase_deg(ii);
    PID_IN.f = WFL_CD_PID.f(ii);
    PID_OUT = DESIGN_PID(PID_IN);
    CONTROL.PARAM.WFL_CD_PID.INT_DISC_TYPE = uint8(PID_IN.int_disc_type);
    CONTROL.PARAM.WFL_CD_PID.DER_DISC_TYPE = uint8(PID_IN.der_disc_type);
    CONTROL.PARAM.WFL_CD_PID.DER_INPUT = uint8(0);
    CONTROL.PARAM.WFL_CD_PID.ANTIWINDUP = uint8(PID_IN.antiwindup);
    LIN_MODEL(ii).WFL_CD_PID = PID_OUT;
    try
        PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
        CONTROL.PARAM.WFL_CD_PID.K(ii,1) = PID_OUT.K;
        CONTROL.PARAM.WFL_CD_PID.Ti(ii,1) = PID_OUT.Ti;
        CONTROL.PARAM.WFL_CD_PID.Td(ii,1) = PID_OUT.Td;
        CONTROL.PARAM.WFL_CD_PID.N(ii,1) = PID_OUT.N;
        CONTROL.PARAM.WFL_CD_PID.b(ii,1) = PID_OUT.b;
    catch
        CONTROL.PARAM.WFL_CD_PID.K(ii,1) = 1;
        CONTROL.PARAM.WFL_CD_PID.Ti(ii,1) = 1;
        CONTROL.PARAM.WFL_CD_PID.Td(ii,1) = 0;
        CONTROL.PARAM.WFL_CD_PID.N(ii,1) = 1;
        CONTROL.PARAM.WFL_CD_PID.b(ii,1) = 1;
    end
    % CLOSED-LOOP MODEL
    try
        WFL_CD_PID_SS_MODEL = minreal(PID_OUT.F_ss);
        LIN_MODEL(ii).WFL_CD_PID_SS_MODEL = WFL_CD_PID_SS_MODEL;
        CONTROL.PARAM.WFL_CD_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
        CONTROL.PARAM.WFL_CD_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
        CONTROL.PARAM.WFL_CD_PID.DER_INPUT = uint8(PID_OUT.der_input);
        CONTROL.PARAM.WFL_CD_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
    catch
    end
end
%--------------------------------------------------------------
%% PITCH ANGLE: STATE FEEDBACK REGULATOR
%--------------------------------------------------------------
% Design method
% 1. Pole placement / 2. LQR
PITCH_ANG_SFC.design_method = 1;
if CONTROL.STATE.OBSERVER_MODE == 1 % EKF
    % Damping factor
    PITCH_ANG_SFC.damping_factor = 0.85;
    % Natural frequency factor: closed-loop wn / open-loop wn 
    PITCH_ANG_SFC.wn_factor = 0.55; 
    % Third pole module / closed-loop wn 
    PITCH_ANG_SFC.p_factor = 20;  
    % LQR state weighting matrix
    PITCH_ANG_SFC.matQ = [30 1.25 1];
    % LQR MV weighting matrix
    PITCH_ANG_SFC.matR = 0.3;
else % Complementary filter 
    % Damping factor
    PITCH_ANG_SFC.damping_factor = 0.85;
    % Natural frequency factor: closed-loop wn / open-loop wn 
    PITCH_ANG_SFC.wn_factor = 0.55; 
    % Third pole module / closed-loop wn
    PITCH_ANG_SFC.p_factor = 20; 
    % LQR state weighting matrix
    PITCH_ANG_SFC.matQ = [20 1.3 1];
    % LQR MV weighting matrix
    PITCH_ANG_SFC.matR = 0.75;
end
%--------------------------------------------------------------
% ATTITUDE SFR: INITIALIZATION
%--------------------------------------------------------------
if CONTROL.STATE.VEHICLE_MODE == 1
    clear SFC_IN SFC_OUT
    SFC_IN.control_type = 1;
    SFC_IN.ts = CONTROL.PARAM.SAMPLING_TIME;
    SFC_IN.nd = 0;
    SFC_IN.int_disc_type = 0;
    SFC_IN.antiwindup = false;
    %--------------------------------------------------------------
    % ATTITUDE SFR: PITCH CONTROL DESIGN
    %--------------------------------------------------------------
    % MV = {'MOTOR VOLT COM'}
    % STATE = {'FORWARD VEL','PITCH RATE','PITCH ANG'}
    %--------------------------------------------------------------
    if PITCH_ANG_SFC.design_method == 1 % Pole placement
        % Unstable pole frequency
        w_ol = max(eig(LIN_MODEL(1).PITCH_ANG_SS_MODEL.a));
        % Natural frequency and damping factor of the dominant closed-loop poles
        wn = PITCH_ANG_SFC.wn_factor*w_ol;
        seta = PITCH_ANG_SFC.damping_factor;
        % Closed loop poles in continuous time
        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -PITCH_ANG_SFC.p_factor].';
    else     % LQR
        SFC_IN.matQ = PITCH_ANG_SFC.matQ;
        SFC_IN.matR = PITCH_ANG_SFC.matR;
    end
    % Design method
    SFC_IN.design_method = PITCH_ANG_SFC.design_method;
    % Plant model
    SFC_IN.P = LIN_MODEL(1).PITCH_ANG_SS_MODEL(3,1);
    %--------------------------------------------------------------
    SFC_OUT = DESIGN_SFC(SFC_IN);
    %--------------------------------------------------------------
    CONTROL.PARAM.PITCH_ANG_SFC.K = SFC_OUT.K;
    for nn = 1:length(LIN_MODEL)
        LIN_MODEL(nn).PITCH_ANG_SFC = SFC_OUT;
    end
else
    CONTROL.PARAM.PITCH_ANG_SFC.K = zeros(1,3);
end
%-------------------------------------------------------------
%% VELOCITY: CAR STATE FEEDBACK CONTROL
%-------------------------------------------------------------
% NAVIGATION INNER-LOOP SFC: SPECIFICATIONS [FORWARD_VEL YAW_RATE]                    
% Design method
% 1. Pole placement / 2. LQR
VEL_SFC.design_method = [2 2];
% Closed-loop wn / Open-loop wn 
VEL_SFC.wn_factor = [0.5 1];
% Damping factor
VEL_SFC.damping_factor = [0.8 0.8];
% LQR state weighting matrix
VEL_SFC.forward_vel_matQ = [0 1];
VEL_SFC.yaw_rate_matQ = [0 1];
% LQR MV weighting matrix
% VEL_SFC.forward_vel_matR = 5e-05;
% VEL_SFC.yaw_rate_matR = 0.05;
VEL_SFC.forward_vel_matR = 2e-05;
VEL_SFC.yaw_rate_matR = 2e-3;
%--------------------------------------------------------------
% VELOCITY SFC: INITIALIZATION
if CONTROL.STATE.VEHICLE_MODE == 0
    clear SFC_IN SFC_OUT
    % Control type
    % 1. Regulator / 2. Setpoint gain / 3. Integral
    SFC_IN.control_type = 3;
    SFC_IN.ts = tc;
    SFC_IN.nd = 0;
    SFC_IN.int_disc_type = 3;
    SFC_IN.antiwindup = true;
    %--------------------------------------------------------------
    % FORWARD VELOCITY: SFC specifications
    % MV = {'MOTOR VOLT COM'}
    % STATE = {'FORWARD VEL'}
    if VEL_SFC.design_method(1) == 1 % Pole placement
        % Open-loop pole frequency
        w_ol = abs(eig(LIN_MODEL(1).FORWARD_VEL_SS_MODEL.a));
        % Natural frequency and damping factor of the dominant closed-loop poles
        wn = VEL_SFC.wn_factor(1)*w_ol;
        seta = VEL_SFC.damping_factor(1);
        % Closed loop poles in continuous time
        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j ].';
    else % LQR
        SFC_IN.matQ = VEL_SFC.forward_vel_matQ;
        SFC_IN.matR = VEL_SFC.forward_vel_matR;
    end
    % Design method
    SFC_IN.design_method = VEL_SFC.design_method(1);
    % Plant model
    SFC_IN.P = LIN_MODEL(1).FORWARD_VEL_SS_MODEL(1,1);
    SFC_OUT = DESIGN_SFC(SFC_IN);
    CONTROL.PARAM.FORWARD_VEL_SFC.K = [SFC_OUT.K 0 0];
    CONTROL.PARAM.FORWARD_VEL_SFC.Ki = SFC_OUT.Ki;
    CONTROL.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
    CONTROL.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
    for nn = 1:length(LIN_MODEL)
        LIN_MODEL(nn).FORWARD_VEL_SFC = SFC_OUT;
    end
    %--------------------------------------------------------------
    % YAW RATE: SFC specifications
    % MV = {'MOTOR VOLT DIF'}
    % STATE = {'YAW RATE'}
    if VEL_SFC.design_method(2) == 1  % Pole placement
        % Open-loop maximun pole frequency
        w_ol=max(abs(eig(LIN_MODEL(1).YAW_RATE_SS_MODEL.a)));
        % Natural frequency and damping factor of the dominant closed-loop poles
        wn = VEL_SFC.wn_factor(2)*w_ol;
        seta = VEL_SFC.damping_factor(2);
        % Closed loop poles in continuous time
        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j].';
    else % LQR
        SFC_IN.matQ = VEL_SFC.yaw_rate_matQ;
        SFC_IN.matR = VEL_SFC.yaw_rate_matR;
    end
    % Design method
    SFC_IN.design_method = VEL_SFC.design_method(2);
    % Plant model
    SFC_IN.P = LIN_MODEL(1).YAW_RATE_SS_MODEL;
    SFC_OUT = DESIGN_SFC(SFC_IN);
    CONTROL.PARAM.YAW_RATE_SFC.K = SFC_OUT.K;
    CONTROL.PARAM.YAW_RATE_SFC.Ki = SFC_OUT.Ki;
    CONTROL.PARAM.YAW_RATE_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
    CONTROL.PARAM.YAW_RATE_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
    for nn = 1:length(LIN_MODEL)
        LIN_MODEL(nn).YAW_RATE_SFC = SFC_OUT;
    end
end
%--------------------------------------------------------------
%% VELOCITY: SBV STATE FEEDBACK CONTROL
%--------------------------------------------------------------
% Design method
% 1. Pole placement / 2. LQR
VEL_SFC.design_method = [1 1];
if CONTROL.STATE.OBSERVER_MODE == 1 % EKF
    % Damping factor
    VEL_SFC.damping_factor = [0.6 0.7];
    % Natural frequency factor: closed-loop wn / open-loop wn
    VEL_SFC.wn_factor = [0.85 0.8];
    % Third pole module / closed-loop wn (only forward velocity)
    VEL_SFC.p3_factor = 0.1;
    % Fourth pole module / closed-loop wn (only forward velocity)
    VEL_SFC.p4_factor = 10;
    % LQR state weighting matrix
    VEL_SFC.forward_vel_matQ = [12 6 0.85 1];
    VEL_SFC.yaw_rate_matQ = [0.1 1];
    % LQR MV weighting matrix
    VEL_SFC.forward_vel_matR = 2.5;
    VEL_SFC.yaw_rate_matR = 0.1;
else % Complementary filter
    % Damping factor
    VEL_SFC.damping_factor = [0.6 0.7];
    % Natural frequency factor: closed-loop wn / open-loop wn
    VEL_SFC.wn_factor = [0.85 0.8];
    % Third pole module / closed-loop wn (only forward velocity)
    VEL_SFC.p3_factor = 0.1;
    % Fourth pole module / closed-loop wn (only forward velocity)
    VEL_SFC.p4_factor = 10;
    % LQR state weighting matrix
    VEL_SFC.forward_vel_matQ = [12 6 0.85 1];
    VEL_SFC.yaw_rate_matQ = [0.1 1];
    % LQR MV weighting matrix
    VEL_SFC.forward_vel_matR = 2.5;
    VEL_SFC.yaw_rate_matR = 0.1;
end
%--------------------------------------------------------------
% VELOCITY SFC: INITIALIZATION
%--------------------------------------------------------------
if CONTROL.STATE.VEHICLE_MODE == 1
    clear SFC_IN SFC_OUT
    % Control type
    % 1. Regulator / 2. Setpoint gain / 3. Integral
    SFC_IN.control_type = 3;
    SFC_IN.ts = CONTROL.PARAM.SAMPLING_TIME;
    SFC_IN.nd = 0;
    SFC_IN.int_disc_type = 1;
    SFC_IN.antiwindup = true;
    %--------------------------------------------------------------
    % FORWARD VELOCITY: SFC specifications
    %--------------------------------------------------------------
    % MV = {'MOTOR VOLT COM'}
    % STATE = {'FORWARD VEL','PITCH RATE','PITCH ANG'}
    %--------------------------------------------------------------
    if VEL_SFC.design_method(1) == 1 % Pole placement
        % Unstable pole frequency
        w_ol = max(eig(LIN_MODEL(1).PITCH_ANG_SS_MODEL.a));
        % Natural frequency and damping factor of the dominant closed-loop poles
        wn = VEL_SFC.wn_factor(1)*w_ol;
        seta = VEL_SFC.damping_factor(1);
        % Closed loop poles in continuous time
        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -VEL_SFC.p3_factor -VEL_SFC.p4_factor].';
    else % LQR
        SFC_IN.matQ = VEL_SFC.forward_vel_matQ;
        SFC_IN.matR = VEL_SFC.forward_vel_matR;
    end
    % Design method
    SFC_IN.design_method = VEL_SFC.design_method(1);
    % Plant model
    SFC_IN.P = LIN_MODEL(1).PITCH_ANG_SS_MODEL(1,1);
    %--------------------------------------------------------------
    SFC_OUT = DESIGN_SFC(SFC_IN);
    %--------------------------------------------------------------
    CONTROL.PARAM.FORWARD_VEL_SFC.K = SFC_OUT.K;
    CONTROL.PARAM.FORWARD_VEL_SFC.Ki = SFC_OUT.Ki;
    CONTROL.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
    CONTROL.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
    for nn = 1:length(LIN_MODEL)
        LIN_MODEL(nn).FORWARD_VEL_SFC = SFC_OUT;
    end
    if VEL_SFC.design_method(2) == 1  % Pole placement
        % Open-loop maximun pole frequency
        w_ol=max(abs(eig(LIN_MODEL(1).YAW_RATE_SS_MODEL.a)));
        % Natural frequency and damping factor of the dominant closed-loop poles
        wn = VEL_SFC.wn_factor(2)*w_ol;
        seta = VEL_SFC.damping_factor(2);
        % Closed loop poles in continuous time
        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j].';
    else % LQR
        SFC_IN.matQ = VEL_SFC.yaw_rate_matQ;
        SFC_IN.matR = VEL_SFC.yaw_rate_matR;
    end
    % Design method
    SFC_IN.design_method = VEL_SFC.design_method(2);
    % Plant model
    SFC_IN.P = LIN_MODEL(1).YAW_RATE_SS_MODEL;
    %--------------------------------------------------------------
    SFC_OUT = DESIGN_SFC(SFC_IN);
    %--------------------------------------------------------------
    CONTROL.PARAM.YAW_RATE_SFC.K = SFC_OUT.K;
    CONTROL.PARAM.YAW_RATE_SFC.Ki = SFC_OUT.Ki;
    CONTROL.PARAM.YAW_RATE_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
    CONTROL.PARAM.YAW_RATE_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
    for nn = 1:length(LIN_MODEL)
        LIN_MODEL(nn).YAW_RATE_SFC = SFC_OUT;
    end
end

%-------------------------------------------------------------
%% YAW ANGLE: STATE FEEDBACK CONTROL
%-------------------------------------------------------------
% YAW ANGLE SFC: SPECIFICATIONS                   
% Design method
% 1. Pole placement / 2. LQR
YAW_ANG_SFC.design_method = 2;
% Closed-loop wn / Open-loop wn 
YAW_ANG_SFC.wn_factor = 0.6;
% Damping factor
YAW_ANG_SFC.damping_factor = 0.5;
% Third pole module / closed-loop wn
YAW_ANG_SFC.p_factor = 1;
% LQR state weighting matrix
YAW_ANG_SFC.matQ = [0 0 1];
% LQR MV weighting matrix
YAW_ANG_SFC.matR = 1e-2;
%--------------------------------------------------------------
% YAW ANGLE SFC: INITIALIZATION
clear SFC_IN SFC_OUT
% Control type
% 1. Regulator / 2. Setpoint gain / 3. Integral
SFC_IN.control_type = 3;
SFC_IN.ts = tc;
SFC_IN.nd = 0;
SFC_IN.int_disc_type = 3;
SFC_IN.antiwindup = true;
%--------------------------------------------------------------
% YAW ANGLE: SFC specifications                     
% MV = {'MOTOR VOLT DIF'}
% STATE = {'YAW RATE','YAW ANG'}
if YAW_ANG_SFC.design_method == 1 % Pole placement    
    % Open-loop maximun pole frequency
    w_ol=max(abs(eig(LIN_MODEL(1).YAW_ANG_SS_MODEL.a))); 
    % Natural frequency and damping factor of the dominant closed-loop poles
    wn = YAW_ANG_SFC.wn_factor*w_ol;
    seta = YAW_ANG_SFC.damping_factor;
    % Closed loop poles in continuous time
    SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -YAW_ANG_SFC.p_factor].';
else   % LQR
    SFC_IN.matQ = YAW_ANG_SFC.matQ;
    SFC_IN.matR = YAW_ANG_SFC.matR;
end
% Design method
SFC_IN.design_method = YAW_ANG_SFC.design_method;
% Plant model
SFC_IN.P = LIN_MODEL(1).YAW_ANG_SS_MODEL;
SFC_OUT = DESIGN_SFC(SFC_IN);
CONTROL.PARAM.YAW_ANG_SFC.K = SFC_OUT.K;
CONTROL.PARAM.YAW_ANG_SFC.Ki = SFC_OUT.Ki;
CONTROL.PARAM.YAW_ANG_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
CONTROL.PARAM.YAW_ANG_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
for nn = 1:length(LIN_MODEL)
    LIN_MODEL(nn).YAW_ANG_SFC = SFC_OUT;
end

%-------------------------------------------------------------
%% WALL FOLLOWER: STATE FEEDBACK REGULATOR
%-------------------------------------------------------------
% WALL FOLLOWER SFR: SPECIFICATIONS                     
% Design method
% 1. Pole placement / 2. LQR
WFL_SFC.design_method = 1;
% Closed-loop wn (rad/s) 
WFL_SFC.natural_freq = [10 5 5 5];
% Damping factor
WFL_SFC.damping_factor = [0.99 0.99 0.99 0.99];
% Third pole module / closed-loop wn 
WFL_SFC.p_factor = [5 5 5 5];
% LQR state weighting matrix
WFL_SFC.matQ = [5e-4 5e-4 1 ; 5e-4 5e-4 1 ; 5e-4 5e-4 1 ; 5e-4 5e-4 1];
% LQR MV weighting matrix
WFL_SFC.matR = [0.5e-3 ; 0.5e-3 ; 0.5e-3 ; 0.5e-3];
%--------------------------------------------------------------
% WALL FOLLOWER SFR: INITIALIZATION
N = length(LIN_MODEL);
CONTROL.PARAM.WFL_SFC.K = zeros(N,3);
CONTROL.PARAM.WFL_SFC.Ki = zeros(N,1);
clear SFC_IN SFC_OUT
SFC_IN.control_type = 1;
SFC_IN.design_method = WFL_SFC.design_method;
SFC_IN.ts = tc;
SFC_IN.nd = 0;
SFC_IN.int_disc_type = 0;
SFC_IN.antiwindup = false;
for nn = 1:length(LIN_MODEL)
    if SFC_IN.design_method == 1 % Pole placement    
        % Natural frequency and damping factor of the dominant closed-loop poles
        wn = WFL_SFC.natural_freq(nn);
        seta = WFL_SFC.damping_factor(nn);
        % Closed loop poles in continuous time
        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j  -WFL_SFC.p_factor(nn)].';
    else  % LQR
        SFC_IN.matQ = WFL_SFC.matQ(nn,:); 
        SFC_IN.matR = WFL_SFC.matR(nn,:); 
    end   
    % Plant model
    SFC_IN.P = LIN_MODEL(nn).WFL_SL_SS_MODEL(3,1);   
    SFC_OUT = DESIGN_SFC(SFC_IN);
    CONTROL.PARAM.WFL_SFC.K(nn,:) = SFC_OUT.K;
    LIN_MODEL(nn).WFL_SFR = SFC_OUT;
end

%--------------------------------------------------------------
%% WALL FOLLOWING COMPETITION

%--------------------------------------------------------------
%% CONTROL TARGET STRUCT DEFINITION
%--------------------------------------------------------------
%--------------------------------------------------------------
% MOTOR VOLTAGE TARGET = {'MOTOR VOLT COM','MOTOR VOLT DIF'}
%--------------------------------------------------------------
CONTROL.TARGET.MOTOR_VOLT_CD_REF = zeros(2,1);
CONTROL.TARGET.MOTOR_VOLT_CD_REF_LC = zeros(2,1);
CONTROL.TARGET.MOTOR_VOLT_CD_REF_PC = zeros(2,1);
CONTROL.TARGET.MOTOR_VOLT_CD_REF_RC = zeros(2,1);
%--------------------------------------------------------------
% ATTITUDE TARGET = {'YAW ANG REF'}
%--------------------------------------------------------------
CONTROL.TARGET.PITCH_ANG_REF = 0;
CONTROL.TARGET.PITCH_ANG_REF_LC = 0;
CONTROL.TARGET.PITCH_ANG_REF_PC = 0;
CONTROL.TARGET.PITCH_ANG_REF_RC = 0;
CONTROL.TARGET.YAW_ANG_REF = 0;
CONTROL.TARGET.YAW_ANG_REF_LC = 0;
CONTROL.TARGET.YAW_ANG_REF_PC = 0;
CONTROL.TARGET.YAW_ANG_REF_RC = 0;
%--------------------------------------------------------------
% VELOCITY TARGETS = {'FORWARD VEL REF','YAW RATE REF'}
%--------------------------------------------------------------
CONTROL.TARGET.FORWARD_VEL_REF = 0;
CONTROL.TARGET.FORWARD_VEL_REF_LC = 0;
CONTROL.TARGET.FORWARD_VEL_REF_PC = 0;
CONTROL.TARGET.FORWARD_VEL_REF_RC = 0;
CONTROL.TARGET.YAW_RATE_REF = 0;
CONTROL.TARGET.YAW_RATE_REF_LC = 0;
CONTROL.TARGET.YAW_RATE_REF_PC = 0;
CONTROL.TARGET.YAW_RATE_REF_RC = 0;
%--------------------------------------------------------------
% WALL FOLLOWER TARGETS = {'FORWARD VEL REF','WALL DIST REF'}
%--------------------------------------------------------------
CONTROL.TARGET.WALL_DIST_REF = 0.1;
CONTROL.TARGET.WALL_DIST_REF_LC = 0.1;
CONTROL.TARGET.WALL_DIST_REF_PC = 0.1;
CONTROL.TARGET.WALL_DIST_REF_RC = 0.1;
%--------------------------------------------------------------
% NAVIGATION TARGETS = {'POS X REF','POS Y REF'}
%--------------------------------------------------------------
CONTROL.TARGET.NAV_REF = zeros(2,1);
CONTROL.TARGET.NAV_REF_LC = zeros(2,1);
CONTROL.TARGET.NAV_REF_PC = zeros(2,1);
CONTROL.TARGET.NAV_REF_RC = zeros(2,1);
CONTROL.TARGET.NAV_REF_PP = zeros(2,1);
CONTROL.TARGET.POS_XY_REF = zeros(2,1);
CONTROL.TARGET.VEL_XY_REF = zeros(2,1);
%--------------------------------------------------------------
% RC CHANNELS (pu): channel range [-1 1] / Throtle range (channel 4) [0 1] 
%--------------------------------------------------------------
CONTROL.TARGET.RC_CH_PU = zeros(14,1);
CONTROL.TARGET.RC_CH_RAW = zeros(14,1);

%--------------------------------------------------------------
%% CONTROL INPUT STRUCT DEFINITION
%--------------------------------------------------------------
% BATTERY 
CONTROL.INPUT.BATTERY_VOLT = MODEL.PARAM.BATTERY_VOLT; % V
CONTROL.INPUT.BATTERY_MV = 1000*MODEL.PARAM.BATTERY_VOLT; %mV
%--------------------------------------------------------------
% MOTORS
CONTROL.INPUT.MOTOR_CURRENT = zeros(2,1); % A
CONTROL.INPUT.MOTOR_RATE = zeros(2,1); % rad/s
CONTROL.INPUT.MOTOR_RATE_CD = zeros(2,1); % rad/s
%--------------------------------------------------------------
% IMU SENSOR          
CONTROL.INPUT.IMU_ACCEL_MEAN = zeros(3,1); % m/s^2                    
CONTROL.INPUT.IMU_ACCEL_MEAN_FILT = zeros(3,1); % m/s^2             
CONTROL.INPUT.IMU_GYRO_MEAN = zeros(3,1); % rad/s    
CONTROL.INPUT.IMU_GYRO_MEAN_FILT = zeros(3,1); % rad/s   
CONTROL.INPUT.IMU_EULER_RATE_CF = zeros(3,1); % rad/s
CONTROL.INPUT.IMU_EULER_RATE_KF = zeros(3,1); % rad/s
CONTROL.INPUT.IMU_EULER_ANG_CF = zeros(3,1); % rad
CONTROL.INPUT.IMU_EULER_ANG_KF = zeros(3,1); % rad
CONTROL.INPUT.IMU_PITCH_ANG_INI = 0; % rad
CONTROL.INPUT.IMU_FORWARD_ACCEL_CF = 0; % m/s^2
CONTROL.INPUT.IMU_FORWARD_ACCEL_KF = 0; % m/s^2
%--------------------------------------------------------------
% MOTOR ENCODER
CONTROL.INPUT.ENC_MOTOR_RATE_LPF = zeros(2,1); % rad/s
CONTROL.INPUT.ENC_MOTOR_RATE_KF = zeros(2,1); % rad/s
CONTROL.INPUT.ENC_MOTOR_RATE_CD_LPF = zeros(2,1); % rad/s
CONTROL.INPUT.ENC_MOTOR_RATE_CD_KF = zeros(2,1); % rad/s
CONTROL.INPUT.ENC_FORWARD_VEL_LPF = 0; % m/s
CONTROL.INPUT.ENC_FORWARD_VEL_KF = 0; % m/s
CONTROL.INPUT.ENC_FORWARD_ACCEL_LPF = 0; % m/s^2
CONTROL.INPUT.ENC_FORWARD_ACCEL_KF = 0; % m/s^2
CONTROL.INPUT.ENC_YAW_ANG = 0; % rad
CONTROL.INPUT.ENC_YAW_ANG_LPF = 0; % rad
CONTROL.INPUT.ENC_YAW_ANG_KF = 0; % rad
CONTROL.INPUT.ENC_YAW_RATE_LPF = 0; % rad/s
CONTROL.INPUT.ENC_YAW_RATE_KF = 0; % rad/s
% ENCODER ANGLES
CONTROL.INPUT.ENC_INC2_COM_ANG = 0; % deg
CONTROL.INPUT.ENC_INC2_DIF_ANG = 0; % deg
CONTROL.INPUT.ENC_INC_COM_ANG = 0;  % deg
CONTROL.INPUT.ENC_INC_DIF_ANG = 0;  % deg
CONTROL.INPUT.ENC_DIF_ANG = 0;      % deg
CONTROL.INPUT.ENC_MOTOR_ANG = zeros(2,1); % rad;
%--------------------------------------------------------------
% LIDAR 2D
% [ANGLE(deg) DISTANCE(m)]: 120 measurements available each sampling time
CONTROL.INPUT.LIDAR_2D = zeros(120,2);
CONTROL.INPUT.LIDAR_2D(:,1) = (0:3:357)';
%--------------------------------------------------------------
% WALL RANGE SENSOR
CONTROL.INPUT.RANGE_FRONT_DIST = 0.1; % m
CONTROL.INPUT.RANGE_REAR_DIST = 0.1; % m
CONTROL.INPUT.RANGE_WALL_ANG = 0; % rad
CONTROL.INPUT.RANGE_WALL_DIST = 0.1; % m
%--------------------------------------------------------------
CONTROL.INPUT.LIDAR_2D_WALL_ANG = 0; % rad
CONTROL.INPUT.LIDAR_2D_WALL_DIST = 0.25; % m
%--------------------------------------------------------------
% MOTION CAPTURE SYSTEM
CONTROL.INPUT.MCS_EULER_ANG = zeros(3,1); % rad
CONTROL.INPUT.MCS_EARTH_POS = zeros(3,1); % m
%--------------------------------------------------------------
% NAVIGATION
CONTROL.INPUT.NAV_EULER_ANG = zeros(3,1); % rad
CONTROL.INPUT.NAV_EULER_ANG_KF = zeros(3,1); % rad
CONTROL.INPUT.NAV_EARTH_POS = zeros(3,1); % m
CONTROL.INPUT.NAV_EARTH_POS_FILT = zeros(3,1); % m
CONTROL.INPUT.NAV_EARTH_POS_KF = zeros(3,1); % m
CONTROL.INPUT.NAV_EARTH_VEL_FILT = zeros(3,1); % m/s
CONTROL.INPUT.NAV_EARTH_ACCEL_FILT = zeros(3,1); % m/s^2
%--------------------------------------------------------------
% ATTITUDE 
%--------------------------------------------------------------
CONTROL.INPUT.PITCH_ANG = 0;
CONTROL.INPUT.YAW_ANG = 0;
%--------------------------------------------------------------
% VELOCITY 
%--------------------------------------------------------------
CONTROL.INPUT.FORWARD_VEL = 0;
CONTROL.INPUT.PITCH_RATE = 0;
CONTROL.INPUT.YAW_RATE = 0;
%--------------------------------------------------------------
% WALL FOLLOWER  
%--------------------------------------------------------------
CONTROL.INPUT.WALL_ANG_FILT = 0; % rad
CONTROL.INPUT.WALL_ANG_KF = 0; % rad
if CONTROL.STATE.WALL_FOLLOWER_MODE
    CONTROL.INPUT.WALL_DIST = 0.25;
    CONTROL.INPUT.WALL_DIST_FILT = 0.25;
    CONTROL.INPUT.WALL_DIST_KF = 0.25;
else
    CONTROL.INPUT.WALL_DIST = 0.1;
    CONTROL.INPUT.WALL_DIST_FILT = 0.1;
    CONTROL.INPUT.WALL_DIST_KF = 0.1;
end
%--------------------------------------------------------------
% NAVIGATION  
%--------------------------------------------------------------
CONTROL.INPUT.NAV_MO = zeros(2,1);
CONTROL.INPUT.EARTH_POS = zeros(3,1);
CONTROL.INPUT.EARTH_VEL = zeros(3,1);
% FORWARD-VELOCITY INTEGRAL ERROR
CONTROL.INPUT.FV_INT_ERROR = 0;
% YAW-RATE INTEGRAL ERROR
CONTROL.INPUT.YR_INT_ERROR = 0;
%--------------------------------------------------------------
% WIRELESS CHARGING
%--------------------------------------------------------------
CONTROL.INPUT.CHARGING_CURRENT = 0;
%--------------------------------------------------------------
% FULL STATE
%--------------------------------------------------------------
% STATE = {'FORWARD VEL','YAW RATE','YAW ANG','WALL DIST','POS X','POS Y'}
CONTROL.INPUT.STATE = zeros(6,1);
CONTROL.INPUT.STATE(4) = 0.1;

%--------------------------------------------------------------
%% CONTROL OUTPUT STRUCT DEFINITION
%--------------------------------------------------------------
% ACTUATORS 
CONTROL.OUTPUT.MOTOR_VOLT = zeros(2,1);
CONTROL.OUTPUT.MOTOR_PWM = zeros(2,1);
CONTROL.OUTPUT.MOTOR_VOLT_CD = zeros(2,1);
% Real motor voltages after applying the pwm rate limit
CONTROL.OUTPUT.REAL_MOTOR_VOLT = zeros(2,1);
CONTROL.OUTPUT.REAL_MOTOR_VOLT_CD = zeros(2,1);
% LEDs
CONTROL.OUTPUT.LEDS = zeros(3,1,'uint8');

%--------------------------------------------------------------
%% CONTROL STATE STRUCT DEFINITION
%--------------------------------------------------------------
%--------------------------------------------------------------
% INPUT 
%--------------------------------------------------------------
% BUTTONS
CONTROL.STATE.BUTTONS = zeros(3,1,'uint8');
CONTROL.STATE.PC_BUTTONS = zeros(3,1,'uint8');
CONTROL.STATE.NAV_BUTTONS = zeros(3,1,'uint8');
% MOTOR STATUS
CONTROL.STATE.MOTOR_STATUS = uint8(0);
%--------------------------------------------------------------
% STATE FLOW CONTROL 
%--------------------------------------------------------------
% / 0. BOOTING     / 1. SENSOR CALIBRATION / 2.  READY
% / 3. OPEN LOOP   / 4. ATTITUDE           / 5.  VELOCITY  
% / 6. NAVIGATION  / 7. WALL FOLLOWER      / 8.  NAVIGATION COMPETITION 
% / 9. WALL FOLLOWER COMPETITION           /10.  FORWARD VELOCITY COMPETITION
% /11. STOP        / 12. SELF-BALANCE      /13.  SBV CALIBRATION  
CONTROL.STATE.CURRENT_STATUS_SYS = uint8(0);
CONTROL.STATE.PREVIOUS_STATUS = uint8(0);
% PC STATE MACHINE
CONTROL.STATE.CURRENT_STATUS_PC = uint8(0);
% CONTROL STATUS
% / 0. INITIALIZATION  / 1. ESTIMATION  / 2. ACTIVE CONTROL
CONTROL.STATE.CONTROL_STATUS = uint8(0);
% MOTOR MODE:
% / 0. NOT ENABLED / 1. ENABLED 
CONTROL.STATE.MOTOR_MODE = uint8(0);
% LIDAR 2D MODE:
% / 0. NOT ENABLED / 1. ENABLED 
CONTROL.STATE.LIDAR_2D_MODE = uint8(0);
% TIMING
CONTROL.STATE.TIMER = 0;
CONTROL.STATE.COMM_TIMER = 0;
CONTROL.STATE.SAMPLING_COUNT = uint8(0);
CONTROL.PARAM.CONTROL_ACT_DELAY = 0; % DELAY IN CONTROL ACTIVATION
% WALL-FOLLOWER COMPETITION
CONTROL.STATE.MAX_FORWARD_VEL_REACHED = uint8(0);
% NAVIGATION COMPETITION
CONTROL.STATE.NAV_TARGET_REACHED = uint8(0);
CONTROL.STATE.COMP_STARTED = uint8(0);
CONTROL.STATE.COMP_FINISHED = uint8(0);
CONTROL.STATE.COMP_START_TIME = 0;
CONTROL.STATE.COMP_TIME = 0;
CONTROL.STATE.SAMPLING_COUNT = uint8(0);
CONTROL.STATE.MAX_VEL_MAG_REACHED = uint8(0);
CONTROL.STATE.NEXT_WP = uint8(1);
CONTROL.STATE.YAW_ANG_CONTROL = uint8(0);
% FORWARD VELOCITY COMPETITION
CONTROL.STATE.COMP_IAE = 0;
% COMMUNICATIONS STATUS
% / 0. WORKING / 1. FAILED
CONTROL.STATE.COMM_STATUS = uint8(0);
% NUMBER OF WRONG BYTES IN TCP/IP COMMUNICATION
CONTROL.PARAM.NUM_TX_WRONG_BYTES = 0;
CONTROL.PARAM.MAX_NUM_TX_WRONG_BYTES = 10;
%--------------------------------------------------------------
% OUTPUT 
%--------------------------------------------------------------
% CPU LOAD (%)
CONTROL.STATE.CPU_LOAD = uint8(0);
% MD25 INIT
CONTROL.STATE.MD25_INIT = uint8(0);

return
