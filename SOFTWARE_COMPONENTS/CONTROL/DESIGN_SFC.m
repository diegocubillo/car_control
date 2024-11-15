function SFC_OUT = DESIGN_SFC(SFC_IN)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Struct SFC_IN: Mandatory fields in the input
% SFC_IN.ts -> Sampling time
% SFC_IN.P  -> Continuous-time plant model in state-space
% SFC_IN.nd -> Number of input delays
% SFC_IN.control_type  -> 1. Regulator / 2. Setpoint gain / 3. Integral
% SFC_IN.design_method -> 1. Pole placement / 2. LQR  
% SFC_IN.int_disc_type -> Discretization type for integral action (Integral control only)
%     / 0. Without integration / 1. Backward Euler
%     / 2. Fordward Euler      / 3. Trapezoidal
% SFC_IN.antiwindup -> Antiwindup activation (Integral control only)
% - True : Antiwindup
% - False: Antiwindup is not applied
% SFC_IN.matQ -> Diagonal of weighting matrix Q (LQR only)
% SFC_IN.matR -> Diagonal of weighting matrix R (LQR only)
% SFC_IN.poles -> List of continuous-time poles in closed loop (Pole placement only)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Struct SFC_OUT: Additional fields in the output
% SFC_OUT.wP     -> Natural frequency of the plant poles
% SFC_OUT.setaP  -> Damping of the plant poles
% SFC_OUT.Pd     -> Discrete-time plant model in state-space
% SFC_OUT.Pd_cd  -> Discrete-time plant model for control design
% SFC_OUT.matBrd -> Input matrix for output reference
% SFC_OUT.nx -> Number of states in the model for control design
% SFC_OUT.nu -> Number of inputs in the model for control design
% SFC_OUT.ny -> Number of outputs in the model for control design
% SFC_OUT.error_msg -> Error message
% SFC_OUT.int_disc_type -> Discretization type for integral action
% SFC_OUT.antiwindup -> Antiwindup activation
% SFC_OUT.matQ -> Diagonal of weighting matrix Q
% SFC_OUT.matR -> Diagonal of weighting matrix R 
% SFC_OUT.poles -> List of continuous-time poles in closed loop
% SFC_OUT.poles_d -> List of discrete-time poles in closed loop
% SFC_OUT.wnP     -> Natural frequency of the plant poles
% SFC_OUT.setaP   -> Damping of the plant poles
% SFC_OUT.wnCL    -> Natural frequency of the closed-loop poles
% SFC_OUT.setaCL  -> Damping of the closed-loop poles
% SFC_OUT.K  -> State gain matrix
% SFC_OUT.Ki -> Integral gain matrix
% SFC_OUT.Kd -> Delay gain matrix
% SFC_OUT.Kr -> Reference gain matrix
% SFC_OUT.Fryd_ss -> Discrete-time closed-loop state-space model
% SFC_OUT.Fry_ss  -> Continuous-time closed-loop state-space model
% SFC_OUT.Fry     -> Continuous-time closed-loop transfer matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SFC_OUT = SFC_IN;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          PLANT             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sampling time
ts = SFC_OUT.ts;
% State-space matrices in continuous time
matA = SFC_OUT.P.a;
matB = SFC_OUT.P.b;
matC = SFC_OUT.P.c;
matD = SFC_OUT.P.d; 
% Dimensions
nx = size(matB,1); % States
nu = size(matB,2); % Inputs
ny = size(matC,1); % Outputs
% Open-loop poles
[wnP,setaP]=damp(eig(matA));
SFC_OUT.wnP = wnP;
SFC_OUT.setaP = setaP;
%--------------------------------------------------------------
% Discretization
matAd = expm(matA*ts);
aux1 = eye(nx)*ts;
aux2 = aux1;
for nn = 1:10
    aux1 = aux1*matA*ts/(nn+1);
    aux2 = aux2 + aux1;
end
matBd = aux2*matB;
matCd = matC;
matDd = matD;
Pd = ss(matAd,matBd,matCd,matDd,ts);
SFC_OUT.Pd = Pd;
%--------------------------------------------------------------
% Input delays are added
nd = SFC_OUT.nd;
if nd>0
    for nn=1:nd
        matAd = [matAd matBd ; zeros(nu,nx) zeros(nu)];
        matBd = [zeros(nx,nu) ; eye(nu)];
        matCd = [matCd matDd];
        nx = size(matBd,1); % States
        nu = size(matBd,2); % Inputs
        ny = size(matCd,1); % Outputs
    end       
end
%--------------------------------------------------------------
% Plant for integral control
if SFC_OUT.control_type==3  
    % Integration method
    int_disc_type = SFC_OUT.int_disc_type;
    switch int_disc_type
        case 1    % Backward Euler: ts*z^-1/(1-z^-1) = ts*(z^-1/(1-z^-1))
            alfa = 0;
            beta = ts;
        case 2    % Fordward Euler: ts/(1-z^-1) = ts*z*(z^-1/(1-z^-1))
            alfa = ts;
            beta = 0;
        case 3    % Trapezoidal: 0.5*ts(1+z^-1)/(1-z^-1) = 0.5*ts(z+1)*(z-1/(1-z^-1))
            alfa = ts/2;
            beta = ts/2;
        otherwise % Error
            SFC_OUT.error_msg = 'This is not a valid integration method.';
            return
    end    
    matAd = [matAd zeros(nx,nu) ; -matCd*(beta*eye(nx)+alfa*matAd) eye(ny)];
    matBd = [matBd ; -alfa*matCd*matBd];
    matCd = [matCd zeros(ny)];
    nx = size(matBd,1); % States
    nu = size(matBd,2); % Inputs
    ny = size(matCd,1); % Outputs
    % Input matrix for reference: it is assumed that r[k+1] = r[k]
    matBrd = zeros(nx,ny);
    matBrd(end-ny+1:end,:) = (beta+alfa)*eye(ny);
    SFC_OUT.matBrd = matBrd;    
    antiwindup = SFC_OUT.antiwindup;     
else
    SFC_OUT.matBrd = zeros(nx,ny);
    SFC_OUT.int_dis_type = 0; % without integration
    antiwindup = false;  
    int_disc_type = 0;
end
%--------------------------------------------------------------
Pd_cd = ss(matAd,matBd,matCd,matDd,ts);
SFC_OUT.Pd_cd = Pd_cd;
SFC_OUT.nx = nx;
SFC_OUT.nu = nu;
SFC_OUT.ny = ny;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Control specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch SFC_OUT.design_method
    case 1 % Pole placement
        % Closed-loop poles
        poles = SFC_OUT.poles;
        poles_d = exp(poles*ts);
        [wnCL,setaCL] = damp(poles);
        SFC_OUT.poles_d = poles_d;
        SFC_OUT.wnCL = wnCL;
        SFC_OUT.setaCL = setaCL;
        % Gain matrix
        matK = place(matAd,matBd,poles_d);
        % Matrix Q
        SFC_OUT.matQ = 0;
        % Matrix R
        SFC_OUT.matR = 0;        
    case 2 % LQR
        % Matrix Q
        matQ = diag(SFC_OUT.matQ);
        % Matrix R
        matR = diag(SFC_OUT.matR);
        % Gain matrix
        [matK,~,poles_d] = dlqr(matAd,matBd,matQ,matR);
        % Closed-loop poles
        SFC_OUT.poles_d = poles_d;
        SFC_OUT.poles = log(poles_d)/ts;
        [wnCL,setaCL] = damp(SFC_OUT.poles);
        SFC_OUT.wnCL = wnCL;
        SFC_OUT.setaCL = setaCL;
    otherwise
        SFC_OUT.error_msg = 'This is not a valid design method.';
        return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. REGULATOR / 2. SETPOINT GAIN / 3. INTEGRAL CONTROL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch SFC_OUT.control_type

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% REGULATOR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 1 
        K = matK(:,1:end-nd*nu);
        Ki = zeros(nu,ny);
        nx_aux = size(matB,1);
        if nd>0
            Kd = matK(:,nx_aux+1:nx_aux+nd*nu);
        else
            Kd = zeros(nu,1);
        end
        Kr = zeros(ny);    
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SETPOINT GAIN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 2 
        K = matK(:,1:end-nd*nu);
        Ki = zeros(nu,ny);
        nx_aux = size(matB,1);
        if nd>0
            Kd = matK(:,nx_aux+1:nx_aux+nd*nu);
        else
            Kd = zeros(nu,1);
        end
        Fd0=(matCd-matDd*matK)/(eye(nx)-matAd+matBd*matK)*matBd+matDd;
        Kr = inv(Fd0);       
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INTEGRAL CONTROL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 3 
        K = matK(:,1:end-nd*nu-ny);
        Ki = matK(:,end-ny+1:end);
        nx_aux = size(matB,1);
        if nd>0
            Kd = matK(:,nx_aux+1:nx_aux+nd*nu);
        else
            Kd = zeros(nu,1);
        end
        Kr = zeros(ny);    
        
    otherwise
            SFC_OUT.error_msg = 'This is not a valid control type.';
            return
end

% Gain matrices
SFC_OUT.K = K;
SFC_OUT.Ki = Ki;
SFC_OUT.Kd = Kd;
SFC_OUT.Kr = Kr;
% Discretization type for integral term
%  / 0. Without integration / 1. Backward Euler /
%  / 2. Fordward Euler      / 3. Trapezoidal
SFC_OUT.int_disc_type = int_disc_type;
% Antiwindup for integral saturation (boolean): '1' if it is applied
SFC_OUT.antiwindup = antiwindup;
% Closed-loop discrete-time state-space model
% u[k] = -matK*x[k] + Kr*r[k]
% x[k+1] = matAd*x[k] + matBd*u[k]
% y[k] = matCd*x[k] + matDd*u[k]
% x[k+1] = (matAd-matBd*matK)*x[k] + matBd*Kr*r[k]
% y[k] = (matCd-matDd*matK)*x[k] + matDd*Kr*r[k]
matAd_cl = matAd-matBd*matK;
matBd_cl = matBd*Kr;
matCd_cl = matCd-matDd*matK;
matDd_cl = matDd*Kr;
Fryd_ss = ss(matAd_cl,matBd_cl,matCd_cl,matDd_cl,ts);
SFC_OUT.Fryd_ss = Fryd_ss;
% Closed-loop continuous-time state-space model (without delay)
% u(t) = -K*x(t) -Ki*xi(t) + Kr*r(t)
% dx(t)/dt = matA*x(t) + matB*u(t)
% dxi(t)/dt = r(t) - y(t) = r(t) - matC*x(t) - matD*u(t)
% y(t) = matC*x(t) + matD*u(t)
% % xa(t) = [x(t) ; xi(t)]
% dxa(t)/dt = [matA zeros(nx,ny); -matC  zeros(ny)]*xa(t) +
%           + [matB ; zeros(ny,nu)]*u(t) + [zeros(nx,ny) ; eye(ny)]*r(t)
% y(t) = [matC zeros(ny)]*xa(t) + matD*u(t)
% u(t) = -[K Ki]*x(t) + Kr*r(t)
nx = size(matB,1); % States
nu = size(matB,2); % Inputs
ny = size(matC,1); % Outputs
matA_ol = [matA zeros(nx,ny); -matC  zeros(ny)];
matB_ol = [matB ; zeros(ny,nu)];
matBr_ol = [zeros(nx,ny) ; eye(ny)];
matC_ol = [matC zeros(ny)];
matD_ol = matD;
matA_cl = matA_ol - matB_ol*[K Ki]; 
matB_cl = matB_ol*Kr + matBr_ol;
matC_cl = matC_ol - matD_ol*[K Ki];
matD_cl = zeros(ny);
Fry_ss = ss(matA_cl,matB_cl,matC_cl,matD_cl);
SFC_OUT.Fry_ss = Fry_ss;
% Closed-loop continuous-time transfer matrix (without delay)
Fry = minreal(zpk(Fry_ss));
Fry.DisplayFormat = 'TimeConstant';
SFC_OUT.Fry = Fry;

return


