function MODEL = CONFIG_MODEL(SAMPLING_TIME,VEHICLE_MODE)
%--------------------------------------------------------------
%% EMG30 MOTOR SPECIFICATION
%--------------------------------------------------------------
% Rated voltage		12v
% Rated torque		1.5kg.cm
% Rated speed 		170rpm
% Rated current		530mA
% No load speed		216rpm
% No load current	150mA
% Stall Current 	2.5A
% Rated output		4.22W
% Encoder counts per output shaft turn		360
%--------------------------------------------------------------
% Resistance = 12 V / 2.5 A = 4.8 ohm
% BEMF constant (no load) = (12 - 4.8*0.15)/(216*pi/30) = 4.9869e-01 V.s/rad
% BEMF constant (rated)   = (12 - 4.8*0.53)/(170*pi/30) = 5.3117e-01 V.s/rad
% Torque constant = (1.5*9.81/100)/0.53 = 2.7764e-01 N.m/A
% Efficiency (rated) = 4.22/((12-4.8*0.53)*0.53) ->  84.2%
% T*w/(e*i) = Kt*i*w/(Ke*w*i) = Kt/Ke = 8.4203e-01
% Torque constant = 5.3117e-01*8.4203e-01 = 4.4726e-01 N.m/A
% Rated motor torque = 4.4726e-01*0.53/9.81*100 = 2.4164 Kg.cm
% Rated net torque = 1.5Kg.cm
% Viscous friction (rated) = (2.4164-1.5)*9.81/100/(170*pi/3) = 5.0498e-04 N.m.s
%--------------------------------------------------------------
%% GENERAL PARAMETERS
%--------------------------------------------------------------
% Control sampling time (s)
MODEL.PARAM.SAMPLING_TIME = SAMPLING_TIME;
% Simulation sampling time (s)
MODEL.PARAM.SIM_SAMPLING_TIME = 1e-3;
% Simulation final time (s)
MODEL.PARAM.SIM_FINAL_TIME = 60;
% Vehicle mode: / 0. CAR / 1. SELF-BALANCING VEHICLE
MODEL.PARAM.VEHICLE_MODE = VEHICLE_MODE;
% Gravity in Madrid (m/s^2)
MODEL.PARAM.GRAVITY = 9.80208;
% Flag to enable noise in simulation
MODEL.PARAM.NOISE_FLAG = uint8(1);

%--------------------------------------------------------------
%% ESTIMATED PARAMETERS
%--------------------------------------------------------------

%--------------------------------------------------------------
% MOTOR PARAMETERS
%--------------------------------------------------------------
% Rated voltage (V)
MODEL.PARAM.BATTERY_VOLT = 12;
% Rotor resistance (ohm)
MODEL.PARAM.MOTOR_RESISTANCE = 7.101;
% Motor back-emf constant (V.s/rad)
MODEL.PARAM.MOTOR_BEMF_CONSTANT = 0.48572;
% Torque constant (N.m/A)
MODEL.PARAM.MOTOR_TORQUE_CONSTANT = 0.48572;
% Motor inertia (kg.m^2)
MODEL.PARAM.MOTOR_INERTIA = 0.000365238;
% Viscous friction constant (N.m.s/rad)
MODEL.PARAM.MOTOR_VISCOUS_FRICTION_COEFF = 0.000931;
% Static friction torque (N.m)
MODEL.PARAM.MOTOR_STATIC_TORQUE = 0.0745612;
% Motor converter voltage drop (V)
MODEL.PARAM.MOTOR_VOLT_DROP_COM = 0;
MODEL.PARAM.MOTOR_VOLT_DROP_DIF = -0.00423311;
% Motor converter delay (s)
MODEL.PARAM.MOTOR_DELAY = 0.05;
% Motor gear ratio
MODEL.PARAM.MOTOR_GEAR_RATIO = 1;

%--------------------------------------------------------------
% CAR PARAMETERS
%--------------------------------------------------------------
% Wheel mass (kg)
MODEL.PARAM.WHEEL_MASS = 0.147;
% Wheel radius (m)
MODEL.PARAM.WHEEL_RADIUS = 0.05;
% Wheel distance (m)
MODEL.PARAM.WHEEL_DISTANCE = 0.2037;
% Body mass (kg)
MODEL.PARAM.BODY_MASS = 2-2*MODEL.PARAM.WHEEL_MASS';
% Body inertia in X axis (kg.m^2)
MODEL.PARAM.BODY_INERTIA_X = 0.00983486;
% Body inertia in Y axis (kg.m^2)
MODEL.PARAM.BODY_INERTIA_Y = 0.0117646;
% Body inertia in Z axis (kg.m^2)
MODEL.PARAM.BODY_INERTIA_Z = 0.0298466;
% Body viscous friction coefficient in X axis (N.m.s/rad)
MODEL.PARAM.BODY_VISCOUS_FRICTION_COEFF_X = 0.17572;
% Body viscous friction coefficient in Z axis (N.m.s/rad)
MODEL.PARAM.BODY_VISCOUS_FRICTION_COEFF_Z = 0.0472748;
% Distance fron wheel axis to body COG [m]
MODEL.PARAM.BODY_DISTANCE = 0.065;
% Wheel gear ratio
MODEL.PARAM.WHEEL_GEAR_RATIO = 1;
% Pitch offset (rad)
MODEL.PARAM.IMU_PITCH_OFFSET = 0;

%--------------------------------------------------------------------------
%% TRANSFER FUNCTION COMPUTATION
%--------------------------------------------------------------------------
% FIRST-ORDER MODELS (FORWARD VELOCITY AND YAW RATE)
M = MODEL.PARAM.BODY_MASS;
m = MODEL.PARAM.WHEEL_MASS;
W = MODEL.PARAM.WHEEL_DISTANCE;
R = MODEL.PARAM.WHEEL_RADIUS;
Ke = MODEL.PARAM.MOTOR_BEMF_CONSTANT;
Kt = MODEL.PARAM.MOTOR_TORQUE_CONSTANT;
Rm = MODEL.PARAM.MOTOR_RESISTANCE;
Im = MODEL.PARAM.MOTOR_INERTIA;
Dm = MODEL.PARAM.MOTOR_VISCOUS_FRICTION_COEFF;
Ix = MODEL.PARAM.BODY_INERTIA_X;
Iy = MODEL.PARAM.BODY_INERTIA_Y;
Iz = MODEL.PARAM.BODY_INERTIA_Z;
Dx = MODEL.PARAM.BODY_VISCOUS_FRICTION_COEFF_X;
Dz = MODEL.PARAM.BODY_VISCOUS_FRICTION_COEFF_Z;
h = MODEL.PARAM.BODY_DISTANCE;
n1 = MODEL.PARAM.MOTOR_GEAR_RATIO;
n2 = MODEL.PARAM.WHEEL_GEAR_RATIO; 
g = MODEL.PARAM.GRAVITY;
if VEHICLE_MODE == 0 % CAR
    MODEL.PARAM.VEL_GAIN = 2*n2*Kt/R/Rm/(2*(n2/R)^2*(Dm+Kt*Ke/Rm));
    MODEL.PARAM.VEL_TIME_CONSTANT = (M+2*m+2*(n2/R)^2*Im)/(2*(n2/R)^2*(Dm+Kt*Ke/Rm));
    MODEL.PARAM.YAW_RATE_GAIN = n2*W*Kt/R/Rm/(Dz+(n2*W/R)^2/2*(Dm+Kt*Ke/Rm));
    MODEL.PARAM.YAW_RATE_TIME_CONSTANT = (Iz+(n2*W/R)^2/2*Im)/(Dz+(n2*W/R)^2/2*(Dm+Kt*Ke/Rm));
else % SELF-BALANCING VEHICLE
    gamma = (M+2*m+2*(n2/R)^2*Im)*Iy + (2*m+2*(n2/R)^2*Im)*M*h^2 - (M+2*m)*2*Im/n1^2;
    a11 = -2*(n2/R)^2*(Ke*Kt/Rm+Dm)/gamma*(Iy+M*h^2-2*Im/n1^2);
    a12 = -2*n2/n1/R*(Ke*Kt/Rm+Dm)/gamma*(Iy+M*h^2-2*Im/n1^2);
    a13 = M*g*h/gamma*(M*h-2*n2/n1/R*Im);
    a21 = -2*(n2/R)^2*(Ke*Kt/Rm+Dm)/gamma*(M*h+2*n2/n1/R*Im);
    a22 = -2*n2/n1/R*(Ke*Kt/Rm+Dm)/gamma*(M*h+2*n2/n1/R*Im);
    a23 = M*g*h/gamma*(M+2*m+2*(n2/R)^2*Im);
    b1 = 2*n2*Kt/R/Rm/gamma*(Iy+M*h^2-2*Im/n1^2);
    b2 = 2*n2*Kt/R/Rm/gamma*(M*h+2*n2/n1/R*Im);
    matA = [a11 a12 a13 ; a21 a22 a23 ; 0 1 0];
    matB = [b1 ; b2 ; 0];
    matC = eye(3);
    matD = zeros(3,1);
    model_ss = ss(matA,matB,matC,matD);
    model_tf = zpk(model_ss);
    model_tf.DisplayFormat = 'TimeConstant';
    s = tf('s');
    Fuv = tf([b1 -b1*a22+b2*a12 -b1*a23+b2*a13],[1 -a11-a22 a11*a22-a12*a21-a23 a11*a23-a13*a21]);
    Fuw = tf([b2  b1*a21-b2*a11 0],[1 -a11-a22 a11*a22-a12*a21-a23 a11*a23-a13*a21]);
    Futh = tf([b2  b1*a21-b2*a11],[1 -a11-a22 a11*a22-a12*a21-a23 a11*a23-a13*a21]);
    Fuv = zpk(Fuv);
    Fuv.DisplayFormat = 'TimeConstant';
    Fuw = zpk(Fuw);
    Fuw.DisplayFormat = 'TimeConstant';
    Futh = zpk(Futh);
    Futh.DisplayFormat = 'TimeConstant';
    MODEL.PARAM.VEL_GAIN = dcgain(Fuv);
    MODEL.PARAM.VEL_TIME_CONSTANT = min(abs(-1../pole(Fuv)));
    MODEL.PARAM.YAW_RATE_GAIN = n2*W*Kt/R/Rm/(Dx+(n2*W/R)^2/2*(Dm+Kt*Ke/Rm));
    MODEL.PARAM.YAW_RATE_TIME_CONSTANT = (Ix+(n2*W/R)^2/2*Im)/(Dx+(n2*W/R)^2/2*(Dm+Kt*Ke/Rm)); 
end

%--------------------------------------------------------------
%% IMU
%--------------------------------------------------------------
% GYROSCOPE 
% Gyro noise (standard deviation) -> rad/s
MODEL.PARAM.IMU_GYRO_NOISE = 0.75;
%--------------------------------------------------------------
% ACCELEROMETER
% Accelerometer noise (standard deviation) -> m/s^2
MODEL.PARAM.IMU_ACCEL_NOISE = 0.1;

%--------------------------------------------------------------
%% ENCODER 
%--------------------------------------------------------------
% Encoder resolution: Number of pulses / revolution of motor main shaft
MODEL.PARAM.MOTOR_ENC_RES = 360; 

%--------------------------------------------------------------
%% RANGE SENSOR
%--------------------------------------------------------------
MODEL.PARAM.WALL_FOLLOWER_MODE = 0;
% Distance between range sensors (m)
MODEL.PARAM.RANGE_SENS_DIST = 0.1225;
% Range distance noise (standard deviation) -> m
MODEL.PARAM.RANGE_DIST_NOISE = 0.5e-3;
% WALL FOLLOWER
MODEL.PARAM.RANGE_XA = 0.2075;
MODEL.PARAM.RANGE_YA = 0.127;
MODEL.PARAM.RANGE_ZA = 0;

%--------------------------------------------------------------
%% LIDAR 2D
%--------------------------------------------------------------
% Lidar sensor coordinates
% xA: midle-point lidar sensor distance in x-axis (m)
MODEL.PARAM.LIDAR_2D_XA = 0.105;
% yA: midle-point lidar sensor distance in y-axis (m)
MODEL.PARAM.LIDAR_2D_YA = -0.035;
% Lidar 2D distance noise (standard deviation) -> m
MODEL.PARAM.LIDAR_2D_DIST_NOISE = 0.5e-3;
% Lidar 2D angle noise (standard deviation) -> rad
MODEL.PARAM.LIDAR_2D_ANG_NOISE = 0.5*pi/180;

%--------------------------------------------------------------
%% MOTION CAPTURE SYSTEM 
%--------------------------------------------------------------
% Standard deviation for MCS Euler angles (rad)
MODEL.PARAM.MCS_EULER_ANG_NOISE = 0.5*pi/180;
% Standard deviation for MCS earth position XYZ (m)
MODEL.PARAM.MCS_EARTH_POS_NOISE = 0.01; 

%--------------------------------------------------------------
%% WALL FOLLOWER: CIRCUIT PARAMETERS
%--------------------------------------------------------------
MODEL.CIRCUIT.PERIMETER_POINTS = 1000;
MODEL.CIRCUIT.STRAIGHT_SIDE_LENGTH = 2;
MODEL.CIRCUIT.RAMP_LENGTH = 1.5;
MODEL.CIRCUIT.CURVE_RADIUS = 0.5;
MODEL.CIRCUIT.RAMP = 0;
MODEL.CIRCUIT.STATUS = 0;

%--------------------------------------------------------------
%% MODEL INPUT STRUCT DEFINITION
%--------------------------------------------------------------
MODEL.INPUT.MOTOR_VOLT = zeros(2,1);
MODEL.INPUT.WALL_RATE = 0;

%--------------------------------------------------------------
%% MODEL OUTPUT STRUCT DEFINITION
%--------------------------------------------------------------
MODEL.OUTPUT.EULER_ANG = zeros(3,1);
MODEL.OUTPUT.EULER_RATE = zeros(3,1);
MODEL.OUTPUT.EULER_RATE_DER = zeros(3,1);
MODEL.OUTPUT.matDCM_BE = eye(3);
MODEL.OUTPUT.matEULER = eye(3);
MODEL.OUTPUT.matEULER_DER = zeros(3);
MODEL.OUTPUT.BODY_RATE = zeros(3,1);
MODEL.OUTPUT.BODY_RATE_DER = zeros(3,1);
MODEL.OUTPUT.EARTH_VEL = zeros(3,1);
MODEL.OUTPUT.EARTH_POS = zeros(3,1);
MODEL.OUTPUT.EARTH_VEL_DER = zeros(3,1);
MODEL.OUTPUT.BODY_VEL_DER = zeros(3,1);
MODEL.OUTPUT.FORWARD_VEL = 0;
MODEL.OUTPUT.PITCH_RATE = 0;
MODEL.OUTPUT.PITCH_ANG = 0;
MODEL.OUTPUT.YAW_RATE = 0;
MODEL.OUTPUT.YAW_ANG = 0;
MODEL.OUTPUT.MOTOR_VOLT_CD = zeros(2,1);
MODEL.OUTPUT.MOTOR_RATE = zeros(2,1);
MODEL.OUTPUT.MOTOR_CURRENT = zeros(2,1);
MODEL.OUTPUT.WHEEL_RATE = zeros(2,1);
MODEL.OUTPUT.WALL_ANG = 0;
MODEL.OUTPUT.WALL_DIST = 0;

%--------------------------------------------------------------
%% MODEL STATE STRUCT DEFINITION 
%--------------------------------------------------------------
% STATE = {'FORWARD VEL','YAW RATE','YAW ANG','POS X','POS Y',...
%          'WALL DIST','WALL_ANG','PITCH RATE','PITCH ANG'}
MODEL.STATE = zeros(9,1);


return
