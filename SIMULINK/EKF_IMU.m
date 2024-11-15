function EKF = EKF_IMU(EKF_IN)

%---------------------------------------------------------
% MODEL: Process   
% x[k] = f(x[k-1],u[k]) + w[k]
% MODEL: Measurement 
% z[k] = h(x[k]) + v[k]
%---------------------------------------------------------
% EKF: Prediction stage
% x[k|k-1] = f(x[k-1|k-1],u[k])
% P[k|k-1] = F[k]*P[k-1|k-1]*F[k]' + Q[k]
% EKF: Update stage
% e = z[k] - h(x[k|k-1])
% S[k] = H[k]*P[k|k-1]*H[k]' + R[k]
% K[k] = P[k|k-1]*H[k]'*inv(S[k])
% x[k|k] = x[k|k-1] + K[k]*e[k]
% P[k|k] = (I - K[k]H[k])*P[k|k-1]
%---------------------------------------------------------
% DEFINITIONS:
% State covariance matrix
% P[k|k] = cov(x_true[k]-x[k|k])
% Process-noise covariance matrix
% Q[k] = cov(w[k]) = diag(var(w[k]))
% Measurement-noise covariance matrix
% R[k] = cov(v[k]) = diag(var(v[k]))
% State-trasition matrix
% F[k] = df(x,u)/dx for x = x[k-1|k-1|) and u = u[k]
% Observation matrix
% H[k] = dh(x)/dx for x = x[k|k-1|)
%---------------------------------------------------------
% Non-additive formulation
% x[k] = f(x[k-1],u[k],w[k])
% z[k] = h(x[k],v[k])
% P[k|k-1] = F[k]*P[k-1|k-1]*F[k]' + L[k]*Q[k]*L[k]'
% S[k] = H[k]*P[k|k-1]*H[k]' + M[k]*R[k]*M[k]'
% L[k] = df(x,u,w)/dw for x = x[k-1|k-1|) and u = u[k]
% M[k] = dh(x,v)/dx for x = x[k|k-1|)
%---------------------------------------------------------
% COPY BUS
%---------------------------------------------------------
EKF = EKF_IN;
EST_MODE = EKF_IN.ESTIMATION_MODE;
%---------------------------------------------------------
if EST_MODE == uint8(0)
    return
end
%% EKF INPUT BUS 
T = EKF_IN.PARAM.SAMPLING_TIME;
g = EKF_IN.PARAM.GRAVITY;
x = EKF_IN.STATE; % x[k-1|k-1]
u = EKF_IN.INPUT; % u[k]
z = EKF_IN.OBSRV; % z[k]
P = EKF_IN.COV_MATRIX; % P[k-1|k-1]
matQ = EKF.PROCESS_NOISE_VAR; % var(w[k])
matR = EKF.OBSRV_NOISE_VAR; % var(v[k])
% Adaptive output (observation) noise for accelerometers
ACCEL_ADPTV_GAIN = EKF_IN.PARAM.ACCEL_ADPTV_GAIN;
matR(1:3) = matR(1:3) + ACCEL_ADPTV_GAIN*abs(norm(z(1:2))); 
Nx = length(x);
Nz = length(z);

%% OBSERVATION ACTIVATION 
persistent initialize z_delayed
if isempty(initialize)
    z_delayed = zeros(size(z));
    initialize = 1;
end

% EKF ESTIMATION MODE: 
% / 0. NOT ENABLED / 1. MCS AVAILABLE  / 2. MCS NOT AVAILABLE
% EKF STATE = [EULER_ANG ; GYRO_BIAS];
% EKF OBSVR = [ACCEL ; ENC_YAW_ANG ; MCS_EULER_ANG];
switch EST_MODE
    case 0 % NOT ENABLED
        ind_x = logical([0 0 0 0 0 0]);
        ind_z = false(1,Nz);
    case 1 % MCS NOT AVAILABLE
        ind_x = logical([1 1 1 1 1 1]);
        ind_z = logical([1 1 1 1 0 0 0]);
    case 2 % MCS AVAILABLE
        ind_x = logical([1 1 1 1 1 1]);
        ind_z = logical([1 1 1 1 1 1 1]);
    otherwise % NOT ENABLED
        ind_x = logical([0 0 0 0 0 0]);
        ind_z = false(1,Nz);
end
% State-variable selection as function of EKF mode
x = x.*ind_x(:);
ind_z = ind_z & (abs(z-z_delayed)>0)';
z_delayed = z;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% PASTE SYMBOLIC EQUATIONS FROM HERE %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State variables
x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);
x5 = x(5);
x6 = x(6);
% Trigonometric functions
s_x1   = sin(x1);
s_x2   = sin(x2);
s_x3   = sin(x3);
c_x1   = cos(x1);
c_x2   = cos(x2);
c_x3   = cos(x3);
t_x2   = s_x2/c_x2;
% Input variables
u1 = u(1);
u2 = u(2);
u3 = u(3);
% State-noise covariance matrix
matQ1 = matQ(1);
matQ2 = matQ(2);
matQ3 = matQ(3);
matQ4 = matQ(4);
matQ5 = matQ(5);
matQ6 = matQ(6);
% State equations
f1 = x1 + T*(u1 - x4 + c_x1*t_x2*(u3 - x6) + s_x1*t_x2*(u2 - x5));
f2 = x2 + T*(c_x1*(u2 - x5) - s_x1*(u3 - x6));
f3 = x3 + T*((c_x1*(u3 - x6))/c_x2 + (s_x1*(u2 - x5))/c_x2);
f4 = x4;
f5 = x5;
f6 = x6;
f = [ f1 f2 f3 f4 f5 f6 ]';
% State-transition matrix
F1_1 = T*(c_x1*t_x2*(u2 - x5) - s_x1*t_x2*(u3 - x6)) + 1;
F1_2 = T*(c_x1*(t_x2^2 + 1)*(u3 - x6) + s_x1*(t_x2^2 + 1)*(u2 - x5));
F1_4 = -T;
F1_5 = -T*s_x1*t_x2;
F1_6 = -T*c_x1*t_x2;
F2_1 = -T*(c_x1*(u3 - x6) + s_x1*(u2 - x5));
F2_2 = 1;
F2_5 = -T*c_x1;
F2_6 = T*s_x1;
F3_1 = T*((c_x1*(u2 - x5))/c_x2 - (s_x1*(u3 - x6))/c_x2);
F3_2 = T*((c_x1*s_x2*(u3 - x6))/c_x2^2 + (s_x1*s_x2*(u2 - x5))/c_x2^2);
F3_3 = 1;
F3_5 = -(T*s_x1)/c_x2;
F3_6 = -(T*c_x1)/c_x2;
F4_4 = 1;
F5_5 = 1;
F6_6 = 1;
% Covariance-matrix prediction
Pk = zeros(Nx,Nx);
Pk(1,1) = matQ1 + F1_1*(F1_1*P(1,1) + F1_2*P(2,1) + F1_4*P(4,1) + F1_5*P(5,1) + F1_6*P(6,1)) + F1_2*(F1_1*P(2,1) + F1_2*P(2,2) + F1_4*P(4,2) + F1_5*P(5,2) + F1_6*P(6,2)) + F1_4*(F1_1*P(4,1) + F1_2*P(4,2) + F1_4*P(4,4) + F1_5*P(5,4) + F1_6*P(6,4)) + F1_5*(F1_1*P(5,1) + F1_2*P(5,2) + F1_4*P(5,4) + F1_5*P(5,5) + F1_6*P(6,5)) + F1_6*(F1_1*P(6,1) + F1_2*P(6,2) + F1_4*P(6,4) + F1_5*P(6,5) + F1_6*P(6,6));
Pk(1,2) = F2_1*(F1_1*P(1,1) + F1_2*P(2,1) + F1_4*P(4,1) + F1_5*P(5,1) + F1_6*P(6,1)) + F2_2*(F1_1*P(2,1) + F1_2*P(2,2) + F1_4*P(4,2) + F1_5*P(5,2) + F1_6*P(6,2)) + F2_5*(F1_1*P(5,1) + F1_2*P(5,2) + F1_4*P(5,4) + F1_5*P(5,5) + F1_6*P(6,5)) + F2_6*(F1_1*P(6,1) + F1_2*P(6,2) + F1_4*P(6,4) + F1_5*P(6,5) + F1_6*P(6,6));
Pk(1,3) = F3_1*(F1_1*P(1,1) + F1_2*P(2,1) + F1_4*P(4,1) + F1_5*P(5,1) + F1_6*P(6,1)) + F3_2*(F1_1*P(2,1) + F1_2*P(2,2) + F1_4*P(4,2) + F1_5*P(5,2) + F1_6*P(6,2)) + F3_3*(F1_1*P(3,1) + F1_2*P(3,2) + F1_4*P(4,3) + F1_5*P(5,3) + F1_6*P(6,3)) + F3_5*(F1_1*P(5,1) + F1_2*P(5,2) + F1_4*P(5,4) + F1_5*P(5,5) + F1_6*P(6,5)) + F3_6*(F1_1*P(6,1) + F1_2*P(6,2) + F1_4*P(6,4) + F1_5*P(6,5) + F1_6*P(6,6));
Pk(1,4) = F4_4*(F1_1*P(4,1) + F1_2*P(4,2) + F1_4*P(4,4) + F1_5*P(5,4) + F1_6*P(6,4));
Pk(1,5) = F5_5*(F1_1*P(5,1) + F1_2*P(5,2) + F1_4*P(5,4) + F1_5*P(5,5) + F1_6*P(6,5));
Pk(1,6) = F6_6*(F1_1*P(6,1) + F1_2*P(6,2) + F1_4*P(6,4) + F1_5*P(6,5) + F1_6*P(6,6));
Pk(2,1:1) = Pk(1:1,2);
Pk(2,2) = matQ2 + F2_1*(F2_1*P(1,1) + F2_2*P(2,1) + F2_5*P(5,1) + F2_6*P(6,1)) + F2_2*(F2_1*P(2,1) + F2_2*P(2,2) + F2_5*P(5,2) + F2_6*P(6,2)) + F2_5*(F2_1*P(5,1) + F2_2*P(5,2) + F2_5*P(5,5) + F2_6*P(6,5)) + F2_6*(F2_1*P(6,1) + F2_2*P(6,2) + F2_5*P(6,5) + F2_6*P(6,6));
Pk(2,3) = F3_1*(F2_1*P(1,1) + F2_2*P(2,1) + F2_5*P(5,1) + F2_6*P(6,1)) + F3_2*(F2_1*P(2,1) + F2_2*P(2,2) + F2_5*P(5,2) + F2_6*P(6,2)) + F3_3*(F2_1*P(3,1) + F2_2*P(3,2) + F2_5*P(5,3) + F2_6*P(6,3)) + F3_5*(F2_1*P(5,1) + F2_2*P(5,2) + F2_5*P(5,5) + F2_6*P(6,5)) + F3_6*(F2_1*P(6,1) + F2_2*P(6,2) + F2_5*P(6,5) + F2_6*P(6,6));
Pk(2,4) = F4_4*(F2_1*P(4,1) + F2_2*P(4,2) + F2_5*P(5,4) + F2_6*P(6,4));
Pk(2,5) = F5_5*(F2_1*P(5,1) + F2_2*P(5,2) + F2_5*P(5,5) + F2_6*P(6,5));
Pk(2,6) = F6_6*(F2_1*P(6,1) + F2_2*P(6,2) + F2_5*P(6,5) + F2_6*P(6,6));
Pk(3,1:2) = Pk(1:2,3);
Pk(3,3) = matQ3 + F3_1*(F3_1*P(1,1) + F3_2*P(2,1) + F3_3*P(3,1) + F3_5*P(5,1) + F3_6*P(6,1)) + F3_2*(F3_1*P(2,1) + F3_2*P(2,2) + F3_3*P(3,2) + F3_5*P(5,2) + F3_6*P(6,2)) + F3_3*(F3_1*P(3,1) + F3_2*P(3,2) + F3_3*P(3,3) + F3_5*P(5,3) + F3_6*P(6,3)) + F3_5*(F3_1*P(5,1) + F3_2*P(5,2) + F3_3*P(5,3) + F3_5*P(5,5) + F3_6*P(6,5)) + F3_6*(F3_1*P(6,1) + F3_2*P(6,2) + F3_3*P(6,3) + F3_5*P(6,5) + F3_6*P(6,6));
Pk(3,4) = F4_4*(F3_1*P(4,1) + F3_2*P(4,2) + F3_3*P(4,3) + F3_5*P(5,4) + F3_6*P(6,4));
Pk(3,5) = F5_5*(F3_1*P(5,1) + F3_2*P(5,2) + F3_3*P(5,3) + F3_5*P(5,5) + F3_6*P(6,5));
Pk(3,6) = F6_6*(F3_1*P(6,1) + F3_2*P(6,2) + F3_3*P(6,3) + F3_5*P(6,5) + F3_6*P(6,6));
Pk(4,1:3) = Pk(1:3,4);
Pk(4,4) = matQ4 + F4_4^2*P(4,4);
Pk(4,5) = F4_4*F5_5*P(5,4);
Pk(4,6) = F4_4*F6_6*P(6,4);
Pk(5,1:4) = Pk(1:4,5);
Pk(5,5) = matQ5 + F5_5^2*P(5,5);
Pk(5,6) = F5_5*F6_6*P(6,5);
Pk(6,1:5) = Pk(1:5,6);
Pk(6,6) = matQ6 + F6_6^2*P(6,6);
% Trigonometric functions for predicted state
s_f1   = sin(f1);
s_f2   = sin(f2);
s_f3   = sin(f3);
c_f1   = cos(f1);
c_f2   = cos(f2);
c_f3   = cos(f3);
% Output equations
H = zeros(Nz,Nx);
e = zeros(Nz,1);
if ind_z(1)
    h1 = g*s_f2;
    e(1) = z(1) - h1;
    H(1,2) = g*c_f2;
end
if ind_z(2)
    h2 = -g*c_f2*s_f1;
    e(2) = z(2) - h2;
    H(2,1) = -g*c_f1*c_f2;
    H(2,2) = g*s_f1*s_f2;
end
if ind_z(3)
    h3 = -g*c_f1*c_f2;
    e(3) = z(3) - h3;
    H(3,1) = g*c_f2*s_f1;
    H(3,2) = g*c_f1*s_f2;
end
if ind_z(4)
    h4 = f3;
    e(4) = z(4) - h4;
    H(4,3) = 1;
end
if ind_z(5)
    h5 = f1;
    e(5) = z(5) - h5;
    H(5,1) = 1;
end
if ind_z(6)
    h6 = f2;
    e(6) = z(6) - h6;
    H(6,2) = 1;
end
if ind_z(7)
    h7 = f3;
    e(7) = z(7) - h7;
    H(7,3) = 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% PASTE SYMBOLIC EQUATIONS UP TO HERE %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% State and covariance matrix update
% Near-optimal Kalman gain
H_aux = H(ind_z,ind_x);
P_aux = Pk(ind_x,ind_x);
PHT = P_aux*H_aux';
K = PHT/(H_aux*PHT + diag(matR(ind_z)));
% Updated state estimate    
x(ind_x) = f(ind_x) + K*e(ind_z);
% Updated covariance estimate
KH = K*H_aux;
P(ind_x,ind_x) = (eye(size(KH)) - KH)*P_aux;

%% EKF BUS UPDATE
EKF = EKF_IN;
EKF.STATE = x; % x[k|k]
EKF.COV_MATRIX = P; % P[k|k]

return




