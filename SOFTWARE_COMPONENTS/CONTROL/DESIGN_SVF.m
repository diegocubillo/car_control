function SVF_OUT = DESIGN_SVF(SVF_IN)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Struct SVF_IN: Mandatory fields in the input
% SVF_IN.ts -> Sampling time
% SVF_IN.order -> Filter order (1 or 2)
% SVF_IN.freq -> Frequency in rad/s
% SVF_IN.damp -> Dampimg
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Struct SVF_OUT: Additional fields in the output
% SVF_OUT.F_ss -> Continuous-time state-space filter
% Discrete-time state-space filter
% SVF_OUT.Ad
% SVF_OUT.B1d
% SVF_OUT.B2d
% SVF_OUT.Cd
% SVF_OUT.Dd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SVF_OUT = SVF_IN;

%--------------------------------------------------------------
% FILTER DESIGN        
%--------------------------------------------------------------
% Natural frequency (rad/s)
SVF_FREQ = SVF_OUT.freq;
% Damping factor
SVF_DAMP = SVF_OUT.damp;
% Sampling time
ts = SVF_OUT.ts;
% CONTINUOUS-TIME SVF MATRICES
switch SVF_OUT.order
    case 1
        % First-order filter
        % wn*s/(s+wn) -> filtered derivative
        % wn/(s+wn) -> filtered signal
        % Controlability canonical form
        matA = -SVF_FREQ;
        matB = 1;
        matC = [SVF_FREQ ; -SVF_FREQ^2];
        matD = [0 ; SVF_FREQ];
        OutputName = {'y','d1y'};
        OutputUnit = {'',''};
        StateName = {'x1'};
        StateUnit = {''};
    case 2
        % Second-order filter
        % wn^2*s^2/(s^2+2*seta*wn*s+wn^2) -> filtered second derivative
        % wn^2*s/(s^2+2*seta*wn*s+wn^2) -> filtered derivative
        % wn^2/(s^2+2*seta*wn*s+wn^2) -> filtered signal
        % Controlability canonical form
        a1 = 2*SVF_DAMP*SVF_FREQ;
        a0 = SVF_FREQ^2;
        b1 = a0;
        matA = [0 1 ; -a0 -a1];
        matB = [0 ; 1];
        matC = [b1 0 ; 0 b1 ; -a0*b1 -a1*b1];
        matD = [0 ; 0 ; b1];
        OutputName = {'y','d1y','d2y'};
        OutputUnit = {'','',''};
        StateName = {'y','d1y'};
        StateUnit = {'',''};
    otherwise
        disp('SVF FILTER ORDER MUST BE 1 OR 2')
        return
end

%--------------------------------------------------------------
% CONTINUOUS-TIME STATE SPACE MODEL
%--------------------------------------------------------------
SVF_OUT.F_ss = ss(matA,matB,matC,matD);
TimeUnit = 'seconds';
InputName = {'u'};
InputUnit = {''};
SVF_OUT.F_ss.TimeUnit = TimeUnit;
SVF_OUT.F_ss.InputName = InputName;
SVF_OUT.F_ss.InputUnit = InputUnit;
SVF_OUT.F_ss.StateName = StateName;
SVF_OUT.F_ss.StateUnit = StateUnit;
SVF_OUT.F_ss.OutputName = OutputName;
SVF_OUT.F_ss.OutputUnit = OutputUnit;

%--------------------------------------------------------------
% DISCRETIZATION
%--------------------------------------------------------------
% Triangular hold
% x[k] = Ad*x[k-1] + B1d*u[k-1] + B2d*u[k]
% Ad = expm(A*ts)
% B1d = A\(Ad+inv(A)/ts-inv(A)*Ad/ts)*B;
% B2d = A\(inv(A)*Ad/ts-eye(2)-inv(A)/ts)*B;
matAd = expm(matA*ts);
matB1d = matA\(matAd+inv(matA)/ts-matA\matAd/ts)*matB;
matB2d = matA\(matA\matAd/ts-eye(SVF_OUT.order)-inv(matA)/ts)*matB;
matCd = matC;
matDd = matD;
SVF_OUT.Ad = matAd;
SVF_OUT.B1d = matB1d;
SVF_OUT.B2d = matB2d;
SVF_OUT.Cd = matCd;
SVF_OUT.Dd = matDd;

%--------------------------------------------------------------
% INITIAL STATE
%--------------------------------------------------------------
% x0 = matAd*x0 + (matBd1+matBd2)*u0
% (I-matA)*x0 = (matBd1+matBd2)*u0
% x0 = (I-matA)\(matBd1+matBd2)*u0
% y0 = (matCd*((I-matA)\(matBd1+matBd2))+matDd)*u0 



return


