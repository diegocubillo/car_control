function EKF = CONFIG_EKF_IMU(MODEL)

%-------------------------------------------------------------------------
%% EKF ESTIMATION MODE: 
%-------------------------------------------------------------------------
% / 0. NOT ENABLED / 1. MCS NOT AVAILABLE  / 2. AVAILABLE
EKF.ESTIMATION_MODE = uint8(0);

%-------------------------------------------------------------------------
%% EKF PARAMETERS
%-------------------------------------------------------------------------
% Sampling time (s)
EKF.PARAM.SAMPLING_TIME = MODEL.PARAM.SAMPLING_TIME;
% EKF input dimension
EKF.PARAM.INPUT_SIZE = uint8(3);
% EKF output dimension
EKF.PARAM.OBSRV_SIZE = uint8(7);
% EKF state dimension
EKF.PARAM.STATE_SIZE = uint8(6);
% Gravity (m/s^2)
EKF.PARAM.GRAVITY = MODEL.PARAM.GRAVITY;

%-------------------------------------------------------------------------
%% INITIALIZATION
%-------------------------------------------------------------------------
% EKF INPUT = GYRO 
EKF.INPUT = zeros(EKF.PARAM.INPUT_SIZE,1);
% EKF OBSRV = [ACCEL ENC_YAW_ANG MCS_EULER_ANG];
EKF.OBSRV = zeros(EKF.PARAM.OBSRV_SIZE,1);
% EKF STATE = [EULER_ANG ; GYRO_BIAS];
EKF.STATE = zeros(EKF.PARAM.STATE_SIZE,1);

%-------------------------------------------------------------------------
%% PROCESS AND OBSERVATION NOISE STANDARD DEVIATIONS
%-------------------------------------------------------------------------

%--------------------------------------------------------------
% IMU PROCESS NOISE STANDARD DEVIATIONS
%--------------------------------------------------------------
% Adaptive gain for accelerometer
EKF.PARAM.ACCEL_ADPTV_GAIN = 1e6;
% Euler angles (rad)
EULER_ANG_STD = 1;
% Gyro bias (rad/s)
GYRO_BIAS_STD = 2e-06;

%--------------------------------------------------------------
% IMU OBSERVATION NOISE STANDARD DEVIATIONS
%--------------------------------------------------------------
% IMU accelerometer (m/s^2)
IMU_ACCEL_STD = 10;
% Encoder yaw angle (rad)
ENC_YAW_ANG_STD = 30;
% MCS Euler angle (rad)
MCS_EULER_ANG_STD = 20;

%-------------------------------------------------------------------------
% VARIANCE VECTORS
%-------------------------------------------------------------------------
EKF.PROCESS_NOISE_VAR = [ones(1,3)*EULER_ANG_STD^2 ones(1,3)*GYRO_BIAS_STD^2]';
EKF.OBSRV_NOISE_VAR = [ones(1,3)*IMU_ACCEL_STD^2 ENC_YAW_ANG_STD^2 ...
                       ones(1,3)*MCS_EULER_ANG_STD^2 ]';    

%-------------------------------------------------------------------------
%% STATE COVARIANCE MATRIX
%-------------------------------------------------------------------------
P = 1*eye(EKF.PARAM.STATE_SIZE);
EKF.COV_MATRIX = P;

return
