function EKF = CONFIG_EKF_NAV(MODEL)

%-------------------------------------------------------------------------
%% EKF ESTIMATION MODE: 
%-------------------------------------------------------------------------
% / 0. NOT ENABLED / 1. NOT ENABLED
EKF.ESTIMATION_MODE = uint8(0);

%-------------------------------------------------------------------------
%% EKF PARAMETERS
%-------------------------------------------------------------------------
% Sampling time (s)
EKF.PARAM.SAMPLING_TIME = MODEL.PARAM.SAMPLING_TIME;
% EKF input dimension
EKF.PARAM.INPUT_SIZE = uint8(2);
% EKF output dimension
EKF.PARAM.OBSRV_SIZE = uint8(3);
% EKF state dimension
EKF.PARAM.STATE_SIZE = uint8(3);

%-------------------------------------------------------------------------
%% INITIALIZATION
%-------------------------------------------------------------------------
% EKF INPUT = [FORWARD_VEL ; YAW_RATE] 
EKF.INPUT = zeros(EKF.PARAM.INPUT_SIZE,1);
% EKF OBSRV = [MCS_YAW_ANG ; MCS_EARTH_POS_X ; MCS_EARTH_POS_Y]
EKF.OBSRV = zeros(EKF.PARAM.OBSRV_SIZE,1);
% EKF STATE = [YAW_ANG ; EARTH_POS_X ; EARTH_POS_Y]
EKF.STATE = zeros(EKF.PARAM.STATE_SIZE,1);

%-------------------------------------------------------------------------
%% PROCESS AND OBSERVATION NOISE STANDARD DEVIATIONS
%-------------------------------------------------------------------------

%--------------------------------------------------------------
% PROCESS NOISE STANDARD DEVIATIONS
%--------------------------------------------------------------
% Yaw angle (rad)
YAW_ANG_STD = 1;
% Position XY in Earth reference frame (m)
POS_XY_STD = 0.1;

%--------------------------------------------------------------
% OBSERVATION NOISE STANDARD DEVIATIONS
%--------------------------------------------------------------
% MCS Euler angle (rad)
MCS_EULER_ANG_STD = 0.01;
% MCS Earth position (m)
MCS_EARTH_POS_STD = 0.01;

%-------------------------------------------------------------------------
% VARIANCE VECTORS
%-------------------------------------------------------------------------
EKF.PROCESS_NOISE_VAR = [YAW_ANG_STD^2 ones(1,2)*POS_XY_STD^2]';
EKF.OBSRV_NOISE_VAR = [MCS_EULER_ANG_STD^2 ones(1,2)*MCS_EARTH_POS_STD^2]';    

%-------------------------------------------------------------------------
%% STATE COVARIANCE MATRIX
%-------------------------------------------------------------------------
P = 1*eye(EKF.PARAM.STATE_SIZE);
EKF.COV_MATRIX = P;

return
