function EKF = EKF_NAV(EKF_IN)

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
x = EKF_IN.STATE; % x[k-1|k-1]
u = EKF_IN.INPUT; % u[k]
z = EKF_IN.OBSRV; % z[k]
P = EKF_IN.COV_MATRIX; % P[k-1|k-1]
matQ = EKF.PROCESS_NOISE_VAR; % var(w[k])
matR = EKF.OBSRV_NOISE_VAR; % var(v[k])
Nx = length(x);
Nz = length(z);

%% OBSERVATION ACTIVATION 
persistent initialize z_delayed
if isempty(initialize)
    z_delayed = zeros(size(z));
    initialize = 1;
end

% EKF ESTIMATION MODE: 
% / 0. NOT ENABLED / 1. ENABLE
% EKF STATE = [YAW_ANG ; EARTH_POS_X ; EARTH_POS_Y]
% EKF OBSVR = [MCS_YAW_ANG ; MCS_EARTH_POS_X ; MCS_EARTH_POS_Y]
switch EST_MODE
    case 0 % NOT ENABLED
        ind_x = logical([0 0 0]);
        ind_z = false(1,Nz);
    case 1 % ENABLED
        ind_x = logical([1 1 1]);
        ind_z = logical([1 1 1]);
    otherwise % NOT ENABLED
        ind_x = logical([0 0 0]);
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
% Trigonometric functions
s_x1   = sin(x1);
c_x1   = cos(x1);
% Input variables
u1 = u(1);
u2 = u(2);
% State-noise covariance matrix
matQ1 = matQ(1);
matQ2 = matQ(2);
matQ3 = matQ(3);
% State equations
f1 = x1 + T*u2;
f2 = x2 + T*u1*c_x1;
f3 = x3 + T*u1*s_x1;
f = [ f1 f2 f3 ]';
% State-transition matrix
F1_1 = 1;
F2_1 = -T*u1*s_x1;
F2_2 = 1;
F3_1 = T*u1*c_x1;
F3_3 = 1;
% Covariance-matrix prediction
Pk = zeros(Nx,Nx);
Pk(1,1) = matQ1 + F1_1^2*P(1,1);
Pk(1,2) = F1_1*F2_1*P(1,1) + F1_1*F2_2*P(2,1);
Pk(1,3) = F1_1*F3_1*P(1,1) + F1_1*F3_3*P(3,1);
Pk(2,1:1) = Pk(1:1,2);
Pk(2,2) = matQ2 + F2_1*(F2_1*P(1,1) + F2_2*P(2,1)) + F2_2*(F2_1*P(2,1) + F2_2*P(2,2));
Pk(2,3) = F3_1*(F2_1*P(1,1) + F2_2*P(2,1)) + F3_3*(F2_1*P(3,1) + F2_2*P(3,2));
Pk(3,1:2) = Pk(1:2,3);
Pk(3,3) = matQ3 + F3_1*(F3_1*P(1,1) + F3_3*P(3,1)) + F3_3*(F3_1*P(3,1) + F3_3*P(3,3));
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
    h1 = f1;
    e(1) = z(1) - h1;
    H(1,1) = 1;
end
if ind_z(2)
    h2 = f2;
    e(2) = z(2) - h2;
    H(2,2) = 1;
end
if ind_z(3)
    h3 = f3;
    e(3) = z(3) - h3;
    H(3,3) = 1;
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




