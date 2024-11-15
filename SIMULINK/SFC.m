function OUTPUT = SFC(INPUT,PARAM)

%--------------------------------------------------------------
% INPUTS
%--------------------------------------------------------------
% Reference
r = INPUT.REF;
r_prv = INPUT.REF_PRV;
% Measured output
y = INPUT.MO;
y_prv = INPUT.MO_PRV;
% State
x = INPUT.STATE;
x_prv = INPUT.STATE_PRV;
% State used to compensate for input delays
% Manipulated variables
u_prv = INPUT.MV_PRV(:,1);
% State for delay compensation
xd = INPUT.MV_PRV(:,1:end-1)';
xd = xd(:);
xd_prv = INPUT.MV_PRV(:,2:end)';
xd_prv = xd_prv(:);
% Integral state
xi_prv = INPUT.STATE_INT;

%--------------------------------------------------------------
% PARAMETERS
%--------------------------------------------------------------
% Sampling time
ts = PARAM.ts;
% Control parameters: 
% x  -> plant state
% xd -> computation-delay state
% xi -> integral-control state
% u = Kr*r - K*x - Kd*xd - Ki*xi
Kr = PARAM.Kr;
K  = PARAM.K;
Kd = PARAM.Kd;
Ki = PARAM.Ki;
% Plant model matrices
% A = PARAM.matAd;
% B = PARAM.matBd;
% C = PARAM.matCd;
% Operating point
xo = PARAM.OP_STATE;
uo = PARAM.OP_INPUT;
yo = PARAM.OP_OUTPUT;
% Number of computational delays to be compensated
num_dlys = length(xd)/size(u_prv,1);
xdo = (uo*ones(1,num_dlys))';
xdo = xdo(:);
% Manipulated-variable maximum
u_max = PARAM.MV_MAX;
% Manipulated-variable minimum
u_min = PARAM.MV_MIN;
% Discretization type for integral term
% INT_DISC_TYPE: / 0. Without integration / 1. Backward Euler /
%                / 2. Fordward Euler      / 3. Trapezoidal
int_disc_type = PARAM.INT_DISC_TYPE;
flag_int = zeros(length(r),1,'uint8');
for ii=1:length(r)
    if ~any(abs(Ki(ii,:))>0)
        flag_int(ii) = uint8(0);
    else
        flag_int(ii) = uint8(1);
    end
end
alfa = ts*(int_disc_type==2) + ts/2*(int_disc_type==3);
beta = ts*(int_disc_type==1) + ts/2*(int_disc_type==3);
% Antiwindup for integral saturation (boolean): '1' if it is applied
% antiwindup = PARAM.ANTIWINDUP; 

%--------------------------------------------------------------
% STATE FEEDBACK CONTROL
%--------------------------------------------------------------
% State-feedback control is applied
xi = zeros(length(y),1);
u = zeros(length(y),1);
for ii=1:length(r)
    if flag_int(ii) % Integral control with incremental formulation
        inc_x = x - x_prv;
        inc_xd = xd - xd_prv;
        inc_xi = alfa*r + beta*r_prv - alfa*y - beta*y_prv;
        xi = xi_prv + inc_xi;
        inc_u = -Ki(ii,:)*inc_xi - K(ii,:)*inc_x - Kd(ii,:)*inc_xd;
        u(ii) = u_prv(ii) + inc_u;
    else % Regulator for an operating point
        u(ii) = -K(ii,:)*(x-xo) - Kd(ii,:)*(xd-xdo) + Kr(ii,:)*(r-yo) + uo(ii);
    end
end

%--------------------------------------------------------------
% MANIPULATED VARIABLES
%--------------------------------------------------------------
% Saturation
u_sat = u.*(u<u_max & u>u_min) + u_max.*(u>=u_max) + u_min.*(u<=u_min);

%% OUTPUTS
OUTPUT.MV = u_sat;
OUTPUT.STATE_INT = xi;
              
return
