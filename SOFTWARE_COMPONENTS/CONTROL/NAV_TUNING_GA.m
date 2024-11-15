%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%  TUNING NAVIGATION CONTROL BASED ON GENETIC ALGORITHM  %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--------------------------------------------------------------
%% GENERAL CONFIGURATION
%--------------------------------------------------------------
clc
tic
clear theta thaux 
format compact
format short g
cd ../../CONFIGURATION
CONFIG_CAR 
MODEL_SLX = 'CAR_CONTROL_SYSTEM';
clc
% Simulation final time (s)
MODEL_INI.PARAM.SIM_FINAL_TIME = 10;
% Start time for cost function computation
startTime = 1;
% Flag to enable noise in simulation
MODEL_INI.PARAM.NOISE_FLAG = uint8(0);
set_param(MODEL_SLX,'StopTime',num2str(MODEL_INI.PARAM.SIM_FINAL_TIME));
% VEHICLE MODE: CAR
MODEL_INI.PARAM.VEHICLE_MODE = 0;
CONTROL_INI.STATE.VEHICLE_MODE = uint8(MODEL_INI.PARAM.VEHICLE_MODE);
% CONTROL MODE: NAVIGATION
CONTROL_INI.STATE.CONTROL_MODE = uint8(3);
% CONTROL TYPE: STATE-FEEDBACK CONTROL
CONTROL_INI.STATE.FORWARD_VEL_CONTROL_TYPE = uint8(1);
CONTROL_INI.STATE.YAW_RATE_CONTROL_TYPE = uint8(1);
% NAVIGATION CONTROL TYPE: ADAPTIVE LQR     
CONTROL_INI.STATE.NAV_CONTROL_TYPE = uint8(0);
% REFERENCE INITIALITATION
CONTROL_INI.STATE.FV_TARGET_TYPE = uint8(0);
CONTROL_INI.STATE.YR_TARGET_TYPE = uint8(0);
CONTROL_INI.STATE.YA_TARGET_TYPE = uint8(0);
CONTROL_INI.STATE.WD_TARGET_TYPE = uint8(0);
% REFERENCE SOURCE: LOCAL
CONTROL_INI.STATE.NAV_TARGET_SOURCE = uint8(0);
% NAVIGATION REFERENCE TYPE: SINUSOIDAL
CONTROL_INI.STATE.NAV_TARGET_TYPE = uint8([2 2]);
%  CONSTANT REFERENCE VALUE (INCREMENT OVER THE INITIAL POS_XY_REF)
CONTROL_INI.STATE.NAV_TARGET_VALUE = [-1 -1];
% OBSERVER MODE: EKF
CONTROL_INI.STATE.OBSERVER_MODE = uint8(1);
% NAVIGATION MODE: MCS
CONTROL_INI.STATE.NAV_MODE = uint8(1);

%--------------------------------------------------------------
%% PARAMETER STRING LIST
%--------------------------------------------------------------
PARAM_STRING = { ...
 %------------ 1 - 4 ---------------   
'VEL_SFC.forward_vel_matQ(1)'
'VEL_SFC.yaw_rate_matQ(1)'
'VEL_SFC.forward_vel_matR'
'VEL_SFC.yaw_rate_matR'
 %------------ 5 - 10 ---------------   
'NAV_SFC.matQ(1)'
'NAV_SFC.matQ(2)'
'NAV_SFC.matQ(3)'
'NAV_SFC.matQ(4)'
'NAV_SFC.matQ(5)'
'NAV_SFC.matQ(6)'
 %------------ 11 - 12 ---------------   
'NAV_SFC.matR(1)'
'NAV_SFC.matR(2)'
 %------------ 13 - 17 ---------------   
'PROCESS_NOISE_VAR(2)'
'PROCESS_NOISE_VAR(3)'
'OBSRV_NOISE_VAR(1)'
'OBSRV_NOISE_VAR(2)'
'OBSRV_NOISE_VAR(3)'
}; 
NUM_PARAM = length(PARAM_STRING);

%--------------------------------------------------------------
% INITIAL PARAMETER VALUES
% Forward velocity: LQR state and MV weighting matrices
VEL_SFC.forward_vel_matQ = [0 1];
VEL_SFC.forward_vel_matR = 1e-5;
% Yaw rate: LQR state and MV weighting matrices
VEL_SFC.yaw_rate_matQ = [0 1];
VEL_SFC.yaw_rate_matR = 5e-3;
% NavigationLQR state and MV weighting matrices
NAV_SFC.matQ = [0 0 0 0 0 0 1 1];
NAV_SFC.matR = [2.5 0.05];
% Navigation EKF
PROCESS_NOISE_VAR = [1 1e-1^2 1e-1^2];
OBSRV_NOISE_VAR = [1e-2^2 1e-4^2 1e-4^2];
%--------------------------------------------------------------
%% FIT MODE CONFIGURATION 
%--------------------------------------------------------------
PARAM_LIST = [3 4 11 12 13 15 16];

%--------------------------------------------------------------
%% INITIALIZATION FOR FORWARD VELOCITY CONTROL
%--------------------------------------------------------------
% Control type
% 1. Regulator / 2. Setpoint gain / 3. Integral
FV_SFC_IN.control_type = 3;
FV_SFC_IN.ts = SAMPLING_TIME;
FV_SFC_IN.nd = 0;
FV_SFC_IN.int_disc_type = 3;
FV_SFC_IN.antiwindup = true;
%--------------------------------------------------------------
% MV = {'MOTOR VOLT COM'}
% STATE = {'FORWARD VEL'}
% Design method
FV_SFC_IN.design_method = 2;
% Plant model
FV_SFC_IN.P = LIN_MODEL(1).FORWARD_VEL_SS_MODEL(1,1);

%--------------------------------------------------------------
%% INITIALIZATION FOR YAW RATE CONTROL
%--------------------------------------------------------------
% Control type
% 1. Regulator / 2. Setpoint gain / 3. Integral
YR_SFC_IN.control_type = 3;
YR_SFC_IN.ts = SAMPLING_TIME;
YR_SFC_IN.nd = 0;
YR_SFC_IN.int_disc_type = 3;
YR_SFC_IN.antiwindup = true;
%--------------------------------------------------------------
% MV = {'MOTOR VOLT COM'}
% STATE = {'FORWARD VEL'}
% Design method
YR_SFC_IN.design_method = 2;
% Plant model
YR_SFC_IN.P = LIN_MODEL(1).YAW_RATE_SS_MODEL;

%--------------------------------------------------------------
%% INITIALIZATION FOR NAVIGATION CONTROL
%--------------------------------------------------------------
% Control type
% 1. Regulator / 2. Setpoint gain / 3. Integral
NAV_IN.control_type = 3;
NAV_IN.ts = CONTROL_INI.PARAM.CONTROL_SAMPLING_TIME;
NAV_IN.nd = 0;
NAV_IN.int_disc_type = 3;
NAV_IN.antiwindup = true;
% Design method
NAV_IN.design_method = 2;
% Plant constant matrices
matCn = [zeros(2,4) eye(2)];
matDn = zeros(2);

%--------------------------------------------------------------
%% PARAMETER LIST INITIALIZATION
%--------------------------------------------------------------
th_ini = [];
for ii = 1:NUM_PARAM
    if min(abs(PARAM_LIST-ii))==0
        command = ['th_ini = [th_ini ; ' PARAM_STRING{ii} '];'];
        eval(command)
    end
end
th = ones(length(th_ini),1); 
theta=th;
Np=length(theta); % Number of parameters

%--------------------------------------------------------------
%% GENETIC ALGORITHM
%--------------------------------------------------------------
% ALGORITHM PARAMETERS
rng default % For reproducibility
options = optimoptions('ga','PlotFcn','gaplotbestf',...
                            'PopulationSize',40,...
                            'Display','iter',...
                            'MaxGenerations',50,...
                            'MaxTime',3600*5);
%    Default properties:
%              ConstraintTolerance: 0.001
%                      CreationFcn: []
%                     CrossoverFcn: []
%                CrossoverFraction: 0.8
%                          Display: 'final'
%                       EliteCount: '0.05*PopulationSize'
%                     FitnessLimit: -Inf
%                FitnessScalingFcn: @fitscalingrank
%                FunctionTolerance: 1e-06
%                        HybridFcn: []
%          InitialPopulationMatrix: []
%           InitialPopulationRange: []
%              InitialScoresMatrix: []
%                   MaxGenerations: '100*numberOfVariables'
%              MaxStallGenerations: 50
%                     MaxStallTime: Inf
%                          MaxTime: Inf
%                      MutationFcn: []
%     NonlinearConstraintAlgorithm: 'auglag'
%                        OutputFcn: []
%                          PlotFcn: []
%                   PopulationSize: '50 when numberOfVariables <= 5, else 200'
%                   PopulationType: 'doubleVector'
%                     SelectionFcn: []
%                      UseParallel: 0
%                    UseVectorized: 0
numVar = length(PARAM_LIST);
lb = 0.5*ones(1,numVar);
ub = 2*ones(1,numVar);
% Aineq*x <= bineq
Aineq = [];
bineq = [];
FitnessFunction = @(x) fitnessFcn(x,CONTROL_INI,MODEL_INI,PARAM_LIST,PARAM_STRING,...
                                  FV_SFC_IN,YR_SFC_IN,NAV_IN,startTime,th_ini,...
                                  VEL_SFC,NAV_SFC,LIN_MODEL,MODEL_SLX);
[theta,fval] = ga(FitnessFunction,numVar,Aineq,bineq,[],[],lb,ub,[],options);

[V,POS_X,POS_Y,t] = fitnessFcn(theta,CONTROL_INI,MODEL_INI,PARAM_LIST,PARAM_STRING,...
                                  FV_SFC_IN,YR_SFC_IN,NAV_IN,startTime,th_ini,...
                                  VEL_SFC,NAV_SFC,LIN_MODEL,MODEL_SLX);

% The parameters and the cost function are shown on the screen
fprintf('--------------------------------------------------------------\n')
disp('PARAMETERS')
% Final parameters
nn = 1;
for ii = 1:NUM_PARAM
    if min(abs(PARAM_LIST-ii))==0
        command = [PARAM_STRING{ii} ' = theta(nn)*th_ini(nn); nn=nn+1;'];
        eval(command)
    end
end
disp(theta(:)')
disp('COST FUNCTION')
disp(V)
% Graphical representation
clf
figure(1)
subplot(211)
plot([t(1) t(end)],[CONTROL_INI.STATE.NAV_TARGET_VALUE(1) CONTROL_INI.STATE.NAV_TARGET_VALUE(1)],'-r',t,POS_X,'-b')
xlabel('Time (s)')
ylabel('Position X')
legend('Reference','Control')
grid
subplot(212)
plot([t(1) t(end)],[CONTROL_INI.STATE.NAV_TARGET_VALUE(2) CONTROL_INI.STATE.NAV_TARGET_VALUE(2)],'-r',t,POS_Y,'-b')
xlabel('Time (s)')
ylabel('Position Y')
legend('Reference','Control')
grid

%--------------------------------------------------------------
%% SCREEN OUTPUT
%--------------------------------------------------------------
% FINAL PARAMETERS
disp(' ')
disp('---------- CONTROL PARAMETERS ------------')
disp(' ')
nn = 1;
for ii = 1:NUM_PARAM
    if min(abs(PARAM_LIST-ii))==0
        command = ['fprintf(''' PARAM_STRING{ii} ' = %g;\n'',theta(nn)*th_ini(nn));'];
        eval(command)
        nn=nn+1;
    end
end

toc

return

%--------------------------------------------------------------
%% FITNESS FUNCTION
%--------------------------------------------------------------
function [V,POS_X,POS_Y,t] = fitnessFcn(th,CONTROL_INI,MODEL_INI,PARAM_LIST,PARAM_STRING,...
                               FV_SFC_IN,YR_SFC_IN,NAV_IN,startTime,th_ini,...
                               VEL_SFC,NAV_SFC,LIN_MODEL,MODEL_SLX)
cd ../SOFTWARE_COMPONENTS/CONTROL
%--------------------------------------------------------------
% PARAMETERS
NUM_PARAM = length(PARAM_STRING);
nn = 1;
for ii = 1:NUM_PARAM
    if min(abs(PARAM_LIST-ii))==0
        command = [PARAM_STRING{ii} ' = th(nn)*th_ini(nn); nn=nn+1;'];
        eval(command)
    end
end
%--------------------------------------------------------------------------
% FORWARD-VELOCITY CONTROL DESIGN: LQR
FV_SFC_IN.matQ = VEL_SFC.forward_vel_matQ;
FV_SFC_IN.matR = VEL_SFC.forward_vel_matR;
% CONTROL DESIGN
FV_SFC_OUT = DESIGN_SFC(FV_SFC_IN);
% SFC CONTROL PARAMETERS
CONTROL_INI.PARAM.FORWARD_VEL_SFC.K = [FV_SFC_OUT.K 0 0];
CONTROL_INI.PARAM.FORWARD_VEL_SFC.Ki = FV_SFC_OUT.Ki;
CONTROL_INI.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = uint8(FV_SFC_OUT.int_disc_type);
CONTROL_INI.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = uint8(FV_SFC_OUT.antiwindup);
%--------------------------------------------------------------------------
% YAW-RATE CONTROL DESIGN: LQR
YR_SFC_IN.matQ = VEL_SFC.yaw_rate_matQ;
YR_SFC_IN.matR = VEL_SFC.yaw_rate_matR;
% CONTROL DESIGN
YR_SFC_OUT = DESIGN_SFC(YR_SFC_IN);
% SFC CONTROL PARAMETERS
CONTROL_INI.PARAM.YAW_RATE_SFC.K = YR_SFC_OUT.K;
CONTROL_INI.PARAM.YAW_RATE_SFC.Ki = YR_SFC_OUT.Ki;
CONTROL_INI.PARAM.YAW_RATE_SFC.INT_DISC_TYPE = uint8(YR_SFC_OUT.int_disc_type);
CONTROL_INI.PARAM.YAW_RATE_SFC.ANTIWINDUP = uint8(YR_SFC_OUT.antiwindup);
%--------------------------------------------------------------------------
% NAVIGATION CONTROL DESIGN: ADAPTIVE LQR
% NAVIGATION MODEL (position control in polar coordinates)
% r*exp(j*phi): vector from car position to target
% v*exp(j*shi); velocity vector
% d/dt(r*exp(j*phi)) = (dr/dt + j*r*d(phi)/dt)*exp(j*phi) = -v*exp(j*shi)
% cos(phi)*dr/dt - sin(phi)*r*d(phi)/dt = -v*cos(shi)
% sin(shi)*dr/dt + cos(phi)*r*d(phi)/dt = -v*sin(shi)
% [dr/dt ; d(phi)/dt] = -1/r*[r*cos(phi) r*sin(phi) ; -sin(phi) cos(phi)]*[v*cos(shi) ; v*sin(shi)]
% dr/dt = -v*(sin(phi)*sin(shi)+cos(phi)*cos(shi)) = -v*cos(phi-shi) = -v*cos(alpha)
% d(phi)/dt = v/r*(sin(phi)*cos(shi)-cos(phi)sin(shi)) = v/r*sin(phi-shi)
% d(alpha)/dt = v/r*sin(alpha) - w 
% Closed-loop state-space models for v and w 
% Control matrices
K_aux = zeros(2,2);
Ki_aux = zeros(2,2);
K_aux(1,1) = CONTROL_INI.PARAM.FORWARD_VEL_SFC.K(1);
K_aux(2,2) = CONTROL_INI.PARAM.YAW_RATE_SFC.K;
Ki_aux(1,1) =  CONTROL_INI.PARAM.FORWARD_VEL_SFC.Ki;
Ki_aux(2,2) =  CONTROL_INI.PARAM.YAW_RATE_SFC.Ki;
% Navigation model
P_ss = minreal(LIN_MODEL(1).NAV_SS_MODEL([1 2],1:2));
matA = P_ss.a;
matB = P_ss.b;
matC = P_ss.c;
% matD = P_ss.d;
% Closed-loop continuous-time state-space model
% X(t) = Y(t) = [v ; w]
% Xi(t) = [Iev ; Iew]
% U(t) = [uc ; ud]
% R(t) = [v_ref ; w_ref]
% U(t) = -K*X(t) - Ki*Xi(t)
% dX(t)/dt = matA*X(t) + matB*U(t)
% dXi(t)/dt = R(t) - Y(t) = R(t) - matC*X(t) - matD*U(t)
% Y(t) = matC*X(t) + matD*U(t)
% Xa(t) = [X(t) ; Xi(t)]
% dXa(t)/dt = [matA zeros(nx,ny); -matC  zeros(ny)]*Xa(t) +
%           + [matB ; zeros(ny,nu)]*U(t) + [zeros(nx,ny) ; eye(ny)]*R(t)
% Y(t) = [matC zeros(ny)]*Xa(t) + matD*U(t)
% U(t) = -[K Ki]*Xa(t)
nx = size(matB,1); % States
nu = size(matB,2); % Inputs
ny = size(matC,1); % Outputs
matA_ol = [matA zeros(nx,ny); -matC  zeros(ny)];
matB_ol = [matB ; zeros(ny,nu)];
matBr_ol = [zeros(nx,ny) ; eye(ny)];
% matC_ol = [matC zeros(ny)];
% matD_ol = matD;
% Closed-loop model: R(t) = [v_ref ; w_ref] -> Y(t) = [v ; w]
matA_cl = matA_ol - matB_ol*[K_aux Ki_aux]; 
matB_cl = matBr_ol;
% matC_cl = matC_ol - matD_ol*[K_aux Ki_aux];
% matD_cl = zeros(ny);
% Fry_ss = ss(matA_cl,matB_cl,matC_cl,matD_cl);
% Position model
% dr/dt = -v*cos(alpha)
% d(alpha)/dt = v/r*sin(alpha) - w 
% [dr ; d(alpha)/dt] = [-cos(alpha) 0 ; 1/r*sin(alpha) -1]*[v ; w]
%                
% Xv(t) = Xa(t) = [X(t) ; Xi(t)] = [v ; w ; Iev ; Iew] 
% Xp(t) = [r ; alpha]
% Xn(t) = [Xv ; Xp] = [v ; w ; Iev ; Iew ; r ; alpha]
% dXn = [dXv ; dXp] = [matA_cl zeros(4,2) ; ...
%                     [-cos(alpha) 0 ; 1/r*sin(alpha) -1] zeros(2,4)]*[Xv ; Xp] + ...
%                      [matB_cl ; zeros(2,2)]*[vref ; w_ref]
% Yn(t) = [r ; alpha]
% Linearization along the trajectory and LQR design
% d(inc_Xn) = [matA_cl zeros(4,2) ; ...
%                -cos(alpha)    0 0 0         0           -v*sin(alpha) ; ...
%              1/r*sin(alpha) -1 0 0 -v/r^2*sin(alpha)  v/r*cos(alpha)]*[inc_vref ; inc_w_ref]
calphao = linspace(0.05,0.95,5);
N_calphao = length(calphao);
vo = linspace(0.05,0.45,5);
N_vo = length(vo);
invro = linspace(4,100,5);
N_invro = length(invro);
CONTROL.PARAM.NAV_SFC.KPP = zeros(N_calphao*N_vo*N_invro,19);
CONTROL.PARAM.NAV_SFC.KPN = zeros(N_calphao*N_vo*N_invro,19);
CONTROL.PARAM.NAV_SFC.KNP = zeros(N_calphao*N_vo*N_invro,19);
CONTROL.PARAM.NAV_SFC.KNN = zeros(N_calphao*N_vo*N_invro,19);
% Control type
% 1. Regulator / 2. Setpoint gain / 3. Integral
NAV_IN.matQ = NAV_SFC.matQ;
NAV_IN.matR = NAV_SFC.matR;
% Design method
NAV_IN.design_method = 2;
% Plant constant matrices
matBn = [matB_cl ; zeros(2,2)];
matCn = [zeros(2,4) eye(2)];
matDn = zeros(2);
% Gain scheduling
nn = 0;
for ii = 1:N_calphao
    for jj = 1:N_vo
        for kk = 1:N_invro
            nn = nn + 1;
            %------------------------------------------------------------------
            % cos(alpha)>0 & sin(alpha)>0 
            % d(inc_Xn) = [matA_cl zeros(4,2) ; ...
            %               -cos(alpha)    0 0 0         0            v*sin(alpha) ; ...
            %              1/r*sin(alpha) -1 0 0 -v/r^2*sin(alpha)  v/r*cos(alpha)]*[inc_vref ; inc_w_ref]
            matAn = [matA_cl zeros(4,2) ; ...
                      -calphao(ii)            0 0 0                    0                       vo(jj)*sqrt(1-calphao(ii)^2) ; ...
             invro(kk)*sqrt(1-calphao(ii)^2) -1 0 0 -vo(jj)*invro(kk)^2*sqrt(1-calphao(ii)^2)  vo(jj)*invro(kk)*calphao(ii)];
            % LQR
            NAV_IN.P = ss(matAn,matBn,matCn,matDn);
            NAV_OUT = DESIGN_SFC(NAV_IN);
            CONTROL.PARAM.NAV_SFC.KPP(nn,:) = [calphao(ii) vo(jj) invro(kk) NAV_OUT.K(1,:) NAV_OUT.K(2,:) NAV_OUT.Ki(1,:) NAV_OUT.Ki(2,:)];
            %------------------------------------------------------------------
            % cos(alpha)>0 & sin(alpha)<0 
            matAn = [matA_cl zeros(4,2) ; ...
                      -calphao(ii)             0 0 0                    0                      -vo(jj)*sqrt(1-calphao(ii)^2) ; ...
             -invro(kk)*sqrt(1-calphao(ii)^2) -1 0 0  vo(jj)*invro(kk)^2*sqrt(1-calphao(ii)^2)  vo(jj)*invro(kk)*calphao(ii)];
            % LQR
            NAV_IN.P = ss(matAn,matBn,matCn,matDn);
            NAV_OUT = DESIGN_SFC(NAV_IN);
            CONTROL.PARAM.NAV_SFC.KPN(nn,:) = [calphao(ii) vo(jj) invro(kk) NAV_OUT.K(1,:) NAV_OUT.K(2,:) NAV_OUT.Ki(1,:) NAV_OUT.Ki(2,:)];
            %------------------------------------------------------------------
            % cos(alpha)<0 & sin(alpha)>0 
            matAn = [matA_cl zeros(4,2) ; ...
                      calphao(ii)            0 0 0                    0                        vo(jj)*sqrt(1-calphao(ii)^2) ; ...
             invro(kk)*sqrt(1-calphao(ii)^2) -1 0 0 -vo(jj)*invro(kk)^2*sqrt(1-calphao(ii)^2) -vo(jj)*invro(kk)*calphao(ii)];
            % LQR
            NAV_IN.P = ss(matAn,matBn,matCn,matDn);
            NAV_OUT = DESIGN_SFC(NAV_IN);
            CONTROL.PARAM.NAV_SFC.KNP(nn,:) = [calphao(ii) vo(jj) invro(kk) NAV_OUT.K(1,:) NAV_OUT.K(2,:) NAV_OUT.Ki(1,:) NAV_OUT.Ki(2,:)];
            %------------------------------------------------------------------
            % cos(alpha)<0 & sin(alpha)<0 
            matAn = [matA_cl zeros(4,2) ; ...
                       calphao(ii)            0 0 0                    0                       -vo(jj)*sqrt(1-calphao(ii)^2) ; ...
             -invro(kk)*sqrt(1-calphao(ii)^2) -1 0 0  vo(jj)*invro(kk)^2*sqrt(1-calphao(ii)^2) -vo(jj)*invro(kk)*calphao(ii)];
            % LQR
            NAV_IN.P = ss(matAn,matBn,matCn,matDn);
            NAV_OUT = DESIGN_SFC(NAV_IN);
            CONTROL.PARAM.NAV_SFC.KNN(nn,:) = [calphao(ii) vo(jj) invro(kk) NAV_OUT.K(1,:) NAV_OUT.K(2,:) NAV_OUT.Ki(1,:) NAV_OUT.Ki(2,:)];
            %------------------------------------------------------------------
        end
    end
end
CONTROL_INI.PARAM.NAV_SFC.INT_DISC_TYPE = uint8(NAV_OUT.int_disc_type);
CONTROL_INI.PARAM.NAV_SFC.ANTIWINDUP = uint8(NAV_OUT.antiwindup);
%--------------------------------------------------------------------------
% SIMULATION
cd('../../SIMULINK');
warning('off','all')
try
    hws = get_param(bdroot,'modelworkspace');
    hws.assignin('MODEL_INI',MODEL_INI);
    hws.assignin('CONTROL_INI',CONTROL_INI);
    sim(MODEL_SLX)
    flag_sim = 1;
catch
    flag_sim = 0;
end
if flag_sim==1
    POS_X = SCOPE_SIM.signals(7).values(:,5); 
    POS_Y = SCOPE_SIM.signals(7).values(:,6); 
    t = SCOPE_SIM.time;
    [~,ind1] = min(abs(t-startTime-1));
    flag_X = abs(POS_X(ind1:end)-CONTROL_INI.STATE.NAV_TARGET_VALUE(1))<=0.01;
    flag_Y = abs(POS_Y(ind1:end)-CONTROL_INI.STATE.NAV_TARGET_VALUE(2))<=0.01;
    [~,ind2] = max(flag_X & flag_Y);
    V = t(ind1+ind2)-startTime;
else  
    POS_X = [];
    POS_Y = [];
    t = [];
    V = 1e6;
end


end
