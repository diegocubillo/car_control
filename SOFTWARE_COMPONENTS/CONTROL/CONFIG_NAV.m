function CONTROL = CONFIG_NAV(CONTROL_IN,LIN_MODEL)

CONTROL = CONTROL_IN;

%-------------------------------------------------------------
%% NAVIGATION: NONLINEAR MODEL IN POLAR COORDINATES
%-------------------------------------------------------------
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
if CONTROL.STATE.CONTROL_MODE==3 || CONTROL.STATE.CONTROL_MODE==6
    if CONTROL.STATE.VEHICLE_MODE == 0
        K_aux = zeros(2,2);
        Ki_aux = zeros(2,2);
        K_aux(1,1) = CONTROL.PARAM.FORWARD_VEL_SFC.K(1);
        K_aux(2,2) = CONTROL.PARAM.YAW_RATE_SFC.K;
        Ki_aux(1,1) =  CONTROL.PARAM.FORWARD_VEL_SFC.Ki;
        Ki_aux(2,2) =  CONTROL.PARAM.YAW_RATE_SFC.Ki;
    else
        K_aux = zeros(2,5);
        Ki_aux = zeros(2,2);
        K_aux(1,1:3) = CONTROL.PARAM.FORWARD_VEL_SFC.K;
        K_aux(2,4) = CONTROL.PARAM.YAW_RATE_SFC.K;
        Ki_aux(1,1) =  CONTROL.PARAM.FORWARD_VEL_SFC.Ki;
        Ki_aux(2,2) =  CONTROL.PARAM.YAW_RATE_SFC.Ki;
    end
    % Navigation model
    P_ss = minreal(LIN_MODEL(1).NAV_SS_MODEL([1 2],1:2));
    matA = P_ss.a;
    matB = P_ss.b;
    matC = P_ss.c;
    matD = P_ss.d;
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
    matC_ol = [matC zeros(ny)];
    matD_ol = matD;
    % Closed-loop model: R(t) = [v_ref ; w_ref] -> Y(t) = [v ; w]
    matA_cl = matA_ol - matB_ol*[K_aux Ki_aux];
    matB_cl = matBr_ol;
    matC_cl = matC_ol - matD_ol*[K_aux Ki_aux];
    matD_cl = zeros(ny);
end
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

%-------------------------------------------------------------
%% NAVIGATION: ADAPTIVE LQR CONTROL (GAIN SCHEDULING)
%-------------------------------------------------------------
% Number of points for the adaptive control
N_calphao = 5;
N_vo = 5;
N_invro = 5;
calphao = linspace(0.05,0.95,N_calphao);
vo = linspace(0.05,CONTROL.PARAM.NAV_MV_MAX(1)-0.05,N_vo);
invro = linspace(1/CONTROL.PARAM.NAV_MAX_TARGET_RADIUS,1/CONTROL.PARAM.NAV_STOP_RADIUS,N_invro);
CONTROL.PARAM.NAV_SFC.COS_ANG = calphao;
CONTROL.PARAM.NAV_SFC.FV = vo;
CONTROL.PARAM.NAV_SFC.INV_DIST = invro;
if CONTROL.STATE.CONTROL_MODE==3 || CONTROL.STATE.CONTROL_MODE==6
    % LQR state weighting matrix
    % X = [v ; w ; Iev ; Iew ; r ; alpha ; r_err ; alpha_err]
    NAV_SFC.matQ = [10 1 0 0 25 1 1 1];
    % NAV_SFC.matR = [7 0.0325];
    NAV_SFC.matR = [5 0.5];
    % Linearization along the trajectory and LQR design
    % d(inc_Xn) = [matA_cl zeros(4,2) ; ...
    %                -cos(alpha)    0 0 0         0           -v*sin(alpha) ; ...
    %              1/r*sin(alpha) -1 0 0 -v/r^2*sin(alpha)  v/r*cos(alpha)]*[inc_vref ; inc_w_ref]
    CONTROL.PARAM.NAV_SFC.KPP = zeros(N_calphao*N_vo*N_invro,16);
    CONTROL.PARAM.NAV_SFC.KPN = zeros(N_calphao*N_vo*N_invro,16);
    CONTROL.PARAM.NAV_SFC.KNP = zeros(N_calphao*N_vo*N_invro,16);
    CONTROL.PARAM.NAV_SFC.KNN = zeros(N_calphao*N_vo*N_invro,16);
    % Control type
    % 1. Regulator / 2. Setpoint gain / 3. Integral
    SFC_IN.control_type = 3;
    SFC_IN.ts = CONTROL.PARAM.CONTROL_SAMPLING_TIME;
    SFC_IN.nd = 0;
    SFC_IN.int_disc_type = 3;
    SFC_IN.antiwindup = true;
    SFC_IN.matQ = NAV_SFC.matQ;
    SFC_IN.matR = NAV_SFC.matR;
    % Design method
    SFC_IN.design_method = 2;
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
                SFC_IN.P = ss(matAn,matBn,matCn,matDn);
                SFC_OUT = DESIGN_SFC(SFC_IN);
                CONTROL.PARAM.NAV_SFC.KPP(nn,:) = [SFC_OUT.K(1,:) SFC_OUT.K(2,:) SFC_OUT.Ki(1,:) SFC_OUT.Ki(2,:)];
                %------------------------------------------------------------------
                % cos(alpha)>0 & sin(alpha)<0
                matAn = [matA_cl zeros(4,2) ; ...
                    -calphao(ii)             0 0 0                    0                      -vo(jj)*sqrt(1-calphao(ii)^2) ; ...
                    -invro(kk)*sqrt(1-calphao(ii)^2) -1 0 0  vo(jj)*invro(kk)^2*sqrt(1-calphao(ii)^2)  vo(jj)*invro(kk)*calphao(ii)];
                % LQR
                SFC_IN.P = ss(matAn,matBn,matCn,matDn);
                SFC_OUT = DESIGN_SFC(SFC_IN);
                CONTROL.PARAM.NAV_SFC.KPN(nn,:) = [SFC_OUT.K(1,:) SFC_OUT.K(2,:) SFC_OUT.Ki(1,:) SFC_OUT.Ki(2,:)];
                %------------------------------------------------------------------
                % cos(alpha)<0 & sin(alpha)>0
                matAn = [matA_cl zeros(4,2) ; ...
                    calphao(ii)            0 0 0                    0                        vo(jj)*sqrt(1-calphao(ii)^2) ; ...
                    invro(kk)*sqrt(1-calphao(ii)^2) -1 0 0 -vo(jj)*invro(kk)^2*sqrt(1-calphao(ii)^2) -vo(jj)*invro(kk)*calphao(ii)];
                % LQR
                SFC_IN.P = ss(matAn,matBn,matCn,matDn);
                SFC_OUT = DESIGN_SFC(SFC_IN);
                CONTROL.PARAM.NAV_SFC.KNP(nn,:) = [SFC_OUT.K(1,:) SFC_OUT.K(2,:) SFC_OUT.Ki(1,:) SFC_OUT.Ki(2,:)];
                %------------------------------------------------------------------
                % cos(alpha)<0 & sin(alpha)<0
                matAn = [matA_cl zeros(4,2) ; ...
                    calphao(ii)            0 0 0                    0                       -vo(jj)*sqrt(1-calphao(ii)^2) ; ...
                    -invro(kk)*sqrt(1-calphao(ii)^2) -1 0 0  vo(jj)*invro(kk)^2*sqrt(1-calphao(ii)^2) -vo(jj)*invro(kk)*calphao(ii)];
                % LQR
                SFC_IN.P = ss(matAn,matBn,matCn,matDn);
                SFC_OUT = DESIGN_SFC(SFC_IN);
                CONTROL.PARAM.NAV_SFC.KNN(nn,:) = [SFC_OUT.K(1,:) SFC_OUT.K(2,:) SFC_OUT.Ki(1,:) SFC_OUT.Ki(2,:)];
                %------------------------------------------------------------------
            end
        end
    end
    CONTROL.PARAM.NAV_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
    CONTROL.PARAM.NAV_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
else
    CONTROL.PARAM.NAV_SFC.KPP = zeros(N_calphao*N_vo*N_invro,19);
    CONTROL.PARAM.NAV_SFC.KPN = zeros(N_calphao*N_vo*N_invro,19);
    CONTROL.PARAM.NAV_SFC.KNP = zeros(N_calphao*N_vo*N_invro,19);
    CONTROL.PARAM.NAV_SFC.KNN = zeros(N_calphao*N_vo*N_invro,19);
    CONTROL.PARAM.NAV_SFC.INT_DISC_TYPE = uint8(3);
    CONTROL.PARAM.NAV_SFC.ANTIWINDUP = uint8(1);
end

%--------------------------------------------------------------
%% NAVIGATION COMPETITION
%--------------------------------------------------------------
% List of waypoints coordinates in XY fixed-reference frame (m)
% Square_side (m)
L = 2;
% Points in the square (vertices and midpoints on each side)
SQ_POINTS = [ ...
 0       0   
 L/2     0
 L/2     L/2
 0       L/2
-L/2     L/2
-L/2     0
-L/2    -L/2
 0      -L/2
 L/2    -L/2
];
% Trajectory definition
% WP_SEQ = [1 2 3 4 5 1 9 8 7 6 1 1]; %trayectoria 1
%WP_SEQ = [1 6 4 2 8 5 3 9 1 7 1 1]; %trayectoria 2
WP_SEQ = [1 2 3 4 1 2 3 4 1 1 1 1]; % trayectoria 2
% Waypoints
if CONTROL.STATE.CONTROL_MODE==6
    MISSION_WAYPOINTS = zeros(length(WP_SEQ),4);
    for nn = 1:length(WP_SEQ)
        MISSION_WAYPOINTS(nn,2:3) = SQ_POINTS(WP_SEQ(nn),:);
    end
    CONTROL.TARGET.MISSION_NUM_WAYPOINTS = size(MISSION_WAYPOINTS,1);
    CONTROL.TARGET.MISSION_WAYPOINTS(1:CONTROL.TARGET.MISSION_NUM_WAYPOINTS,:) = MISSION_WAYPOINTS;
    CONTROL.TARGET.MISSION_WP_RADIUS = 0.02;
    CONTROL.TARGET.MISSION_YA_ERROR = pi;
    CONTROL.TARGET.MISSION_COUNT = uint8(1);
end


end

% close('all')
% 
% 
% %------------------------------------------------------------------------
% %% OCCUPANCY MAP
% %-------------------------------------------------------------------------
% % Create a occupancy map for the laboratory
% % Losetas: 51 x 51
% % Profundo: 9 losetas (20 medias losetas de 25.5 cm)
% % Ancho: 10 losetas + 40 cm (22 medias losetas de 25.5 cm)
% rows = 20;
% cols = 22;
% res = 1/0.255;
% occpProb = 0.1;
% map = zeros(rows,cols);
% % Edge definition
% map(1,:) = 1; map(end,:) = 1; map(:,1) = 1; map(:,end) = 1;
% % Add random obstacles
% map(2:end-1,2:end-1) = rand(rows-2,cols-2)>(1-occpProb);
% % Occupancy map
% map = occupancyMap(map,res);
% %    mapLayer Properties
% %         OccupiedThreshold: 0.64998
% %             FreeThreshold: 0.20002
% %     ProbabilitySaturation: [0.001 0.999]
% %                 LayerName: 'probabilityLayer'
% %                  DataType: 'double'
% %              DefaultValue: 0.50003
% %       GridLocationInWorld: [0 0]
% %         GridOriginInLocal: [0 0]
% %        LocalOriginInWorld: [0 0]
% %                Resolution: 3.9216
% %                  GridSize: [20 22]
% %              XLocalLimits: [0 5.61]
% %              YLocalLimits: [0 5.1]
% %              XWorldLimits: [0 5.61]
% %              YWorldLimits: [0 5.1]
% % Frame origin definition
% map.GridOriginInLocal = -[cols/2 rows/2]/res;
% map.GridLocationInWorld = [0 0];
% % Show map
% show(map)
% axis square
% hold on
% 
% %------------------------------------------------------------------------
% %% ROBOT
% %-------------------------------------------------------------------------
% % Create robot geometry
% robot = rigidBodyTree();
% % Collision box geometry
% collisionObj = collisionBox(0.25,0.29,0.1);
% addCollision(robot.Base,collisionObj);
% % Set the initial position of the robot
% initialPos = [map.LocalOriginInWorld 0 -pi/4];
% show(robot,'Position',initialPos,'Collisions','on');
% 
% %------------------------------------------------------------------------
% %% DEFINE TARGET REGION 
% %------------------------------------------------------------------------
% xTarget = 0.5*map.XWorldLimits(2)*rand(1) + 0.25*map.XWorldLimits(2);
% yTarget = 0.5*map.YWorldLimits(2)*rand(1) + 0.25*map.YWorldLimits(2);
% sideLength = 0.255;
% vertices = [xTarget-sideLength/2 yTarget+sideLength/2 ; ...
%             xTarget+sideLength/2 yTarget+sideLength/2 ; ...
%             xTarget+sideLength/2 yTarget-sideLength/2 ; ...
%             xTarget-sideLength/2 yTarget-sideLength/2];
% targetRegion = polyshape(vertices);
% plot(targetRegion,FaceColor='red',FaceAlpha=0.2);
% % Set the target position to the centroid of the target region
% % [xTarget,yTarget] = centroid(targetRegion);
% 
% %------------------------------------------------------------------------
% %% OCCUPANCY CHECK
% %------------------------------------------------------------------------
% % Create a Dubins state space
% ss = stateSpaceDubins;
% ss.StateBounds = [map.XWorldLimits ; map.YWorldLimits ; [-pi pi]];
% ss.MinTurningRadius = 0.01;
% % State validator using the created state space and the map
% sv = validatorOccupancyMap(ss,Map=map);
% sv.ValidationDistance = sqrt((collisionObj.X/2)^2 + (collisionObj.Y/2)^2);
% 
% %------------------------------------------------------------------------
% %% RRT PATH PLANNER
% %------------------------------------------------------------------------
% % Rapidly-exploring random tree path planner
% planner = plannerRRT(ss,sv);
% %                StateSpace: [1×1 stateSpaceDubins]
% %            StateValidator: [1×1 validatorOccupancyMap]
% %           MaxNumTreeNodes: 10000
% %             MaxIterations: 2500
% %     MaxConnectionDistance: 0.255
% %            GoalReachedFcn: @nav.algs.checkIfGoalIsReached
% %                  GoalBias: 0.05
% planner.MaxIterations = 2500;
% planner.MaxConnectionDistance = 0.255;
% % goalReachedRegionCheck = @(planner,currentPos,goalPos) isinterior(goalRegion,currentPos(1:2));
% % planner.GoalReachedFcn = goalReachedRegionCheck;
% % Plan the path
% % rng(10,'twister'); % for repeatable result
% [pthObj,solnInfo] = plan(planner,initialPos([1 2 4]),[xTarget yTarget 0]);
% % Visualize the path
% plot(pthObj.States(:,1),pthObj.States(:,2),'r-',LineWidth=1)
% hold off
% 
% return
% 
% 
% %-------------------------------------------------------------
% %% NAVIGATION: MODEL PREDICTIVE CONTROL
% %-------------------------------------------------------------
% % NAV_MPC_SS_MODEL = minreal(Fry_ss);
% % NAV_MPC_SS_MODEL.TimeUnit = 'seconds';
% % NAV_MPC_SS_MODEL.InputName = {'FORWARD VEL REF','YAW RATE REF'};
% % NAV_MPC_SS_MODEL.InputUnit = {'m/s','rad/s'};
% % NAV_MPC_SS_MODEL.OutputName = {'FORWARD VEL','YAW RATE'};
% % NAV_MPC_SS_MODEL.OutputUnit = {'m/s','rad/s'};
% % NAV_MPC_SS_MODEL.InputGroup.MV = [1 2];
% % NAV_MPC_SS_MODEL.InputGroup.MD = [];
% % NAV_MPC_SS_MODEL.InputGroup.UD = [];
% % NAV_MPC_SS_MODEL.OutputGroup.MO = [1 2];
% % NAV_MPC_SS_MODEL.OutputGroup.UO = [];
% % for ii = 1:length(LIN_MODEL)
% %     LIN_MODEL(ii).NAV_MPC_SS_MODEL = NAV_MPC_SS_MODEL;
% % end
% % %--------------------------------------------------------------
% % % NON-LINEAR MPC DEFINITION
% Model.StateFcn = @(X,U,matA_cl,matB_cl) stateFunction(X,U,matA_cl,matB_cl)
% Jacobian.StateFcn = @(X,U,matA_cl,matB_cl) stateJacobian(X,U,matA_cl,matB_cl)
% % NAV_MPC = nlmpc(nx+2,nx+2,nu);
% % % nlobj = nlmpc(nx,ny,'MV',mvIndex,'MD',mdIndex,'UD',udIndex)
% % NAV_MPC.Ts = 0.1;
% % NAV_MPC.PredictionHorizon = 10;
% % NAV_MPC.ControlHorizon = 3;
% % NAV_MPC.StateFcn = "NAV_SS_MODEL";
% % NAV_MPC.OutputFcn = "myOutputFunction";
% % NAV_MPC.IsContinuousTime = true;
% % NAV_MPC.NumberOfParameters = 0;
% % NAV_MPC.States.Min = [];
% % NAV_MPC.States.Max = [];
% % NAV_MPC.States.Name = {'',''};
% % NAV_MPC.States.Units = {'',''};
% % NAV_MPC.States.ScaleFactor = [];
% % NAV_MPC.OutputVariables.Min = [];
% % NAV_MPC.OutputVariables.Max = [];
% % NAV_MPC.OutputVariables.MinECR = [];
% % NAV_MPC.OutputVariables.MaxECR = [];
% % NAV_MPC.OutputVariables.Name = {'',''};
% % NAV_MPC.OutputVariables.Units = {'',''};
% % NAV_MPC.OutputVariables.ScaleFactor = [];
% % NAV_MPC.ManipulatedVariables.Min = [];
% % NAV_MPC.ManipulatedVariables.Max = [];
% % NAV_MPC.ManipulatedVariables.MinECR = [];
% % NAV_MPC.ManipulatedVariables.MaxECR = [];
% % NAV_MPC.ManipulatedVariables.RateMin = [];
% % NAV_MPC.ManipulatedVariables.RateMax = [];
% % NAV_MPC.ManipulatedVariables.RateMinECR = [];
% % NAV_MPC.ManipulatedVariables.RateMaxECR = [];
% % NAV_MPC.ManipulatedVariables.Name = {'',''};
% % NAV_MPC.ManipulatedVariables.Units = {'',''};
% % NAV_MPC.ManipulatedVariables.ScaleFactor = [];
% % NAV_MPC.MeasuredDisturbances.Name = {'',''};
% % NAV_MPC.MeasuredDisturbances.Units = {'',''};
% % NAV_MPC.MeasuredDisturbances.ScaleFactor = [];
% % NAV_MPC.Weights.ManipulateVariables = [];
% % NAV_MPC.Weights.ManipulateVariablesRate = [];
% % NAV_MPC.Weights.OutputVariables = [];
% % NAV_MPC.Weights.ECR = 1e5;
% % NAV_MPC.Optimization
% % NAV_MPC.Jacobian
% %--------------------------------------------------------------
% % NON-LINEAR MPC PARAMETERS
% CONTROL.PARAM.NAV_MPC.SAMPLING_TIME = 26*ts;
% CONTROL.PARAM.NAV_MPC.PRED_HRZ = 10;
% 
% % Ahead targets for MPC
% CONTROL.TARGET.NAV_REF_MPC = zeros(CONTROL.PARAM.NAV_MPC.PRED_HRZ,2);
% 
% %-------------------------------------------------------------------------
% function dX = stateFunction(X,U,matA_cl,matB_cl)
% %-------------------------------------------------------------------------
% % Position model
% % dShi = w
% % dx = vx = v*cos(Shi)
% % dy = vy = v*sin(Shi)
% % [dShi ; dx ; dy] = [0 1 ; cos(Shi) 0 ; sin(Shi) 0]*[v ; w]
% %
% % X_v = [v ; w ; Iev ; Iew]
% % X_p = [Shi ; x ; y]
% % X = [X_v ; X_p]
% X_v = X(1:4);
% X_p = X(5:7);
% v_ref = U(1);
% w_ref = U(2);
% Shi = X(5);
% dX = [matA_cl zeros(4,3) ; ...
%      [0 1 ; cos(Shi) 0 ; sin(Shi) 0] zeros(3,5)]*[X_v ; X_p] + ...
%      [matB_cl ; zeros(3,2)]*[v_ref ; w_ref];
% 
% end
% 
% %-------------------------------------------------------------------------
% function [matJa,matJbmv] = stateJacobian(X,U,matA_cl,matB_cl)
% %-------------------------------------------------------------------------
% % Position model
% % dShi = w
% % dx = vx = v*cos(Shi)
% % dy = vy = v*sin(Shi)
% % [dShi ; dx ; dy] = [0 1 ; cos(Shi) 0 ; sin(Shi) 0]*[v ; w]
% %
% % X_v = [v ; w ; Iev ; Iew]
% % X_p = [Shi ; x ; y]
% % X = [X_v ; X_p]
% v = X(1);
% Shi = X(5);
% % dX = [matA_cl zeros(4,3) ; ...
% %      [0 1 ; cos(Shi) 0 ; sin(Shi) 0] zeros(3,5)]*[X_v ; X_p] + ...
% %      [matB_cl ; zeros(3,2)]*[v_ref ; w_ref];
% matJa = [matA_cl zeros(4,3) ; ...
%         [0 1 ; cos(Shi) 0 ; sin(Shi) 0] zeros(3,2) [0 ; -sin(Shi)*v ; cos(Shi)*v] zeros(3,2)];
% matJbmv = [matB_cl ; zeros(3,2)];
% 
% 
% end
% 
