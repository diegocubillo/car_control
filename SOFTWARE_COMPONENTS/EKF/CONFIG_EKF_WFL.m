function EKF = CONFIG_EKF_WFL(MODEL)

%-------------------------------------------------------------------------
%% EKF ESTIMATION MODE: 
%-------------------------------------------------------------------------
% / 0. NOT ENABLED / 1. ENABLED
EKF.ESTIMATION_MODE = uint8(0);

%-------------------------------------------------------------------------
%% EKF PARAMETERS
%-------------------------------------------------------------------------
% Sampling time (s)
EKF.PARAM.SAMPLING_TIME = MODEL.PARAM.SAMPLING_TIME;
% EKF input dimension
EKF.PARAM.INPUT_SIZE = uint8(2);
% EKF output dimension
EKF.PARAM.OBSRV_SIZE = uint8(2);
% EKF state dimension
EKF.PARAM.STATE_SIZE = uint8(3);
% Range sensor coordinates
% xA: midle-point range sensor distance in x-axis (m)
if MODEL.PARAM.VEHICLE_MODE==0
    EKF.PARAM.RANGE_XA = MODEL.PARAM.RANGE_XA;
else
    EKF.PARAM.RANGE_XA = MODEL.PARAM.RANGE_ZA;
end
% yA: midle-point range sensor distance in y-axis (m)
EKF.PARAM.RANGE_YA = MODEL.PARAM.RANGE_YA;
% Lidar sensor coordinates
% xA: midle-point lidar sensor distance in x-axis (m)
EKF.PARAM.LIDAR_2D_XA = MODEL.PARAM.LIDAR_2D_XA;
% yA: midle-point lidar sensor distance in y-axis (m)
EKF.PARAM.LIDAR_2D_YA = MODEL.PARAM.LIDAR_2D_YA;
% Angle offset for wall following (deg)
EKF.PARAM.LIDAR_2D_WFL_ANG_OFFS = 270;
% Angle sign for wall following (1 or -1)
EKF.PARAM.LIDAR_2D_WFL_ANG_SIGN = -1;
% WALL FOLLOWER MODE: / 0. RANGE SENSOR / 1. LIDAR 2D 
EKF.PARAM.WALL_FOLLOWER_MODE = uint8(0);

%-------------------------------------------------------------------------
%% INITIALIZATION
%-------------------------------------------------------------------------
% EKF INPUT = [FORWARD_VEL ; YAW_RATE] 
EKF.INPUT = zeros(EKF.PARAM.INPUT_SIZE,1);
% EKF OBSRV = [RANGE_YAW_ANG ; RANGE_WALL_DIST]
EKF.OBSRV = zeros(EKF.PARAM.OBSRV_SIZE,1);
% EKF STATE = [WALL_ANG ; WALL_DIST ; YAW_RATE_BIAS]
EKF.STATE = zeros(EKF.PARAM.STATE_SIZE,1);

%-------------------------------------------------------------------------
%% PROCESS AND OBSERVATION NOISE STANDARD DEVIATIONS
%-------------------------------------------------------------------------

%--------------------------------------------------------------
% WFL PROCESS NOISE STANDARD DEVIATIONS
%--------------------------------------------------------------
% Wall angle (rad)
WALL_ANG_STD = 1;
% Wall distance (m)
WALL_DIST_STD = 10;
% Yaw rate bias (rad/s)
YAW_RATE_BIAS_STD = 2.5e-4;

%--------------------------------------------------------------
% WFL OBSERVATION NOISE STANDARD DEVIATIONS
%--------------------------------------------------------------
% Yaw angle measurement (rad)
WALL_ANG_MEAS_STD = 0.2;
% Wall distance measurement (m)
WALL_DIST_MEAS_STD = 2;

%-------------------------------------------------------------------------
% VARIANCE VECTORS
%-------------------------------------------------------------------------
EKF.PROCESS_NOISE_VAR = [WALL_ANG_STD^2 WALL_DIST_STD^2 YAW_RATE_BIAS_STD^2]';
EKF.OBSRV_NOISE_VAR = [WALL_ANG_MEAS_STD^2 WALL_DIST_MEAS_STD^2]';    

%-------------------------------------------------------------------------
%% STATE COVARIANCE MATRIX
%-------------------------------------------------------------------------
P = 1*eye(EKF.PARAM.STATE_SIZE);
EKF.COV_MATRIX = P;

return
