function OUTPUT = PID_MV(INPUT,PARAM)

%--------------------------------------------------------------
% INPUTS
%--------------------------------------------------------------
% Reference
r = INPUT.REF;
r_prv = INPUT.REF_PRV;
% Measured output
y = INPUT.MO;
y_prv = INPUT.MO_PRV;
% Measured oupput derivative if available
dy = INPUT.MOD;
dy_prv = INPUT.MOD_PRV;
% State
x = INPUT.STATE;
x_prv = INPUT.STATE_PRV;
% Decoupling state
xdcp = INPUT.STATE_DCP;
% Derivative action
Ie_prv = INPUT.INT_PID_PRV;
% Derivative action
De_prv = INPUT.DER_PID_PRV;
% Manipulated variables
u_prv = INPUT.MV_PRV(:,1);
% Decoupled manipulated variables
uc_prv = INPUT.MV_PID_PRV;

%--------------------------------------------------------------
% PARAMETERS
%--------------------------------------------------------------
% Sampling time
ts = PARAM.ts;
% Control parameters: 2 dof controller
% Cr(s) = K*(b + Td*s/(1+Td/N*s) + 1/Ti/s)
% C(s)  = K*(1 + Td*s/(1+Td/N*s) + 1/Ti/s)
K = PARAM.K;
Ti = PARAM.Ti;
Td = PARAM.Td;
b = PARAM.b;
N = PARAM.N;
% Manipulated-variable maximum
u_max = PARAM.MV_MAX;
% Manipulated-variable minimum
u_min = PARAM.MV_MIN;
% Discretization type for integral term
% INT_DISC_TYPE: / 0. Without integration / 1. Backward Euler /
%                / 2. Fordward Euler      / 3. Trapezoidal
int_disc_type = PARAM.INT_DISC_TYPE.*uint8(abs(Ti)>0);
alfa_i = ts*(int_disc_type==2) + ts/2*(int_disc_type==3);
beta_i = ts*(int_disc_type==1) + ts/2*(int_disc_type==3);
% Discretization type for derivative term
% DER_DISC_TYPE: / 0. Without derivative  / 1. Backward Euler /
%                / 2. Fordward Euler      / 3. Trapezoidal    /
%                / 4. Measured derivative /
der_disc_type = PARAM.DER_DISC_TYPE.*uint8(abs(Td)>0);
alfa_d = ts*(der_disc_type==2) + ts/2*(der_disc_type==3);
beta_d = ts*(der_disc_type==1) + ts/2*(der_disc_type==3);
% Derivative input
% DER_INPUT:     / 0. Error / 1. Measurement /
a = PARAM.DER_INPUT==0;
% Antiwindup for integral saturation (boolean): '1' if it is applied
% antiwindup = PARAM.ANTIWINDUP; 
% Decoupling network for multivariable control
if length(r)>1
    Adcp = PARAM.DCP_A;
    Bdcp = PARAM.DCP_B;
    Cdcp = PARAM.DCP_C;
    Ddcp = PARAM.DCP_D;
end

%--------------------------------------------------------------
% PIDs
%--------------------------------------------------------------
% Number of measured outputs
nMO = length(y);
% Incremental formulation
inc_r = r - r_prv;
inc_y = y - y_prv;
% Integral
Ie = zeros(nMO,1);
% Derivative
De = zeros(nMO,1);
% Decoupled manipulated variables
uc = zeros(nMO,1);
% All decoupled PIDs are computed
for nn=1:nMO
    % Derivative
    if der_disc_type(nn)>0 && der_disc_type(nn)<4 % Discretixation
        De(nn) = (Td(nn)-beta_d(nn)*N(nn))/(Td(nn)+alfa_d(nn)*N(nn))*De_prv(nn) ...
                 + N(nn)*Td(nn)/(Td(nn)+alfa_d(nn)*N(nn))*(a(nn)*inc_r(nn)-inc_y(nn));
        inc_De = De(nn) - De_prv(nn);
    elseif der_disc_type(nn)>3 % Measured output derivative
        De(nn) = (Td(nn)-beta_d(nn)*N(nn))/(Td(nn)+alfa_d(nn)*N(nn))*De_prv(nn) ...
                  - N(nn)*Td(nn)*alfa_d(nn)/(Td(nn)+alfa_d(nn)*N(nn))*dy(nn) ...
                  - N(nn)*Td(nn)*beta_d(nn)/(Td(nn)+alfa_d(nn)*N(nn))*dy_prv(nn);
        inc_De = De(nn) - De_prv(nn);    
    else % Without derivative
        De(nn) = 0;
        inc_De = 0;
    end
    if int_disc_type(nn)>0 % With integral action
        % Integral: incremental formulation
        inc_Ie = alfa_i(nn)*(r(nn)-y(nn)) + beta_i(nn)*(r_prv(nn)-y_prv(nn));
        Ie(nn) = Ie_prv(nn) + inc_Ie;
        % Decoupled manipulated variables
        inc_uc = K(nn)*(b(nn)*inc_r(nn)-inc_y(nn) + 1/Ti(nn)*inc_Ie + inc_De);
        uc(nn) = uc_prv(nn) + inc_uc;
        % uc(nn) = K(nn)*(b(nn)*r(nn)-y(nn) + 1/Ti(nn)*Ie(nn) + De(nn));       
    else % Without integral action
        Ie(nn) = 0;
        % Decoupled manipulated variables
        uc(nn) = K(nn)*(r(nn)-y(nn) + De(nn));       
    end        
end

%--------------------------------------------------------------
% MANIPULATED VARIABLE COMPUTATION
%--------------------------------------------------------------
if int_disc_type(nn)>0 % With integral action
    % Incremental formulation
    inc_uc = uc - uc_prv;
    % Multivariable control
    if length(r)>1
        inc_x = x - x_prv;
        inc_u = Cdcp*xdcp + Ddcp*[inc_uc ; inc_x];
        % Decoupling state update
        xdcp = Adcp*xdcp + Bdcp*[inc_uc ; inc_x];
    % Monovariable control    
    else
        inc_u = inc_uc;        
    end
    u = u_prv + inc_u;
else % Without integral action
    % Multivariable control
    if length(r)>1
        u = Cdcp*xdcp + Ddcp*[uc ; x];
        % Decoupling state update
        xdcp = Adcp*xdcp + Bdcp*[uc ; x];
    % Monovariable control    
    else
        u = uc;
    end
end

%--------------------------------------------------------------
% MANIPULATED VARIABLE SATURATION
%--------------------------------------------------------------
% Saturation
u_sat = u.*(u<u_max & u>u_min) + u_max.*(u>=u_max) + u_min.*(u<=u_min);
 
%--------------------------------------------------------------
% OUTPUTS
%--------------------------------------------------------------
OUTPUT.MV = u_sat;
OUTPUT.INT = Ie;
OUTPUT.DER = De;
if length(r)>1    
    OUTPUT.MV_PID = uc;
    OUTPUT.STATE_DCP = xdcp;
end
                
return



