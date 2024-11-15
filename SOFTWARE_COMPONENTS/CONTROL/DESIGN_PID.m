function PID_OUT = DESIGN_PID(PID_IN)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Struct PID_IN: Mandatory fields in the input
% PID_IN.P -> Continuous-time plant model (MPC model)
% - PID_IN.P.InputName      -> Input names
% - PID_IN.P.InputUnit      -> Input units
% - PID_IN.P.InputGroup.MV  -> Manipulated variables
% - PID_IN.P.InputGroup.MD  -> Measured disturbances
% - PID_IN.P.InputGroup.UD  -> Unmeasured disturbances
% - PID_IN.P.OutputName     -> Output names
% - PID_IN.P.OutputUnit     -> Output units
% - PID_IN.P.OutputGroup.MO -> Measured outputs
% - PID_IN.P.OutputGroup.UO -> Measured disturbances
% - PID_IN.P.StateName      -> State names
% - PID_IN.P.StateUnit      -> State units
%   The numbers of manipulated variables and measured outputs must be the same
% PID_IN.ts -> Sampling time
% PID_IN.control_type -> Control type (row: 1 x MV)
%     / 1. Control P          /  2. Control PI 
%     / 3. Control PD error   /  4. Control PD output
%     / 5. Control PID error  /  6. Control PID output
% PID_IN.decoupling_type -> Decoupling type
%     / 0. Without decoupling             / 1. Feedforward static decoupling
%     / 2. Feedforward dynamic decoupling / 3. State feedback decoupling
% PID_IN.design_method -> Meethod for PID design.
%     / 0. Frequency response / 1. Time response
% PID_IN.damping_factor -> Damping factor for dominant poles (row: 1 x MV)
% PID_IN.phase_margin_deg -> Phase margin in degrees (row: 1 x MV)
% PID_IN.gain_margin_dB   -> Gain margin in dB (row: 1 x MV)
% PID_IN.search_wd_P -> boolean (row: 1 x MV)
% - True: Design frequency for control P must be searched
% - False: Design frequency is given
% PID_IN.wd_ref (row: 1 x MV)
% - if PID_IN.search_wd_P == true -> Initial estimate for frequency searching
% - if PID_IN.search_wd_P == false -> Reference frequency for design frequency computation 
% PID_IN.k_wd_P -> factor applied to obtain design frequency (row: 1 x MV)
% - PI: must be <= 1
% - PD and PID: must be >= 1
% PID_IN.delta -> % delta = wn*Ti between 1 and 5. Faster and more noisy for 1 (PID only)                  
% PID_IN.lag_phase_deg -> Lag phase for the integral action (row: 1 x MV) (PID only)
% PID_IN.f -> Filtering factor (row: 1 x MV) (PD & PID only)
% PID_IN.b -> Reference weighting factor (row: 1 x MV) (PI & PID only)
% PID_IN.D_lower = boolean (row: 1 x MV) (PD & PID only)
% - True : D lower and Kp higher are used
% - False: D higher and Kp lower are used
% PID_IN.int_disc_type -> Discretization type for integral action (row: 1 x MV) (PI & PID only)
%     / 0. Without integration / 1. Backward Euler
%     / 2. Fordward Euler      / 3. Trapezoidal
% PID_IN.der_disc_type -> Discretization type for derivative action (row: 1 x MV) (PD & PID only)
%     / 0. Without derivative  / 1. Backward Euler 
%     / 2. Fordward Euler      / 3. Trapezoidal    
%     / 4. Measured derivative 
% PID_IN.antiwindup -> Antiwindup activation (row: 1 x MV) (PI & PID only)
% - True : Antiwindup is applied
% - False: Antiwindup is not applied
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Struct PID_OUT: Additional fields in the output
% PID_OUT.P_ss  -> Plant model (state-space model)
% PID_OUT.P_tf  -> Plant model (transfer matrix) for the control design
% PID_OUT.wnP     -> Natural frequency of the plant poles
% PID_OUT.setaP   -> Damping of the plant poles
% PID_OUT.RGA -> RGA matrix without integrators 
% PID_OUT.RGA_out -> Dominant output for each input in RGA matrix
% PID_OUT.P_hat  -> Decoupled`plant model (transfer matrix) for the control design
% PID_OUT.D_tf  -> Decoupling network (transfer matrix)
% PID_OUT.D_ss  -> Decoupling network (state-space model)
% PID_OUT.error_msg -> Error message
% PID_OUT.wd_P -> wd_P applied in control design (row: 1 x MV)
% - if PID_IN.search_wd_P == true -> Control P frequency
% - if PID_IN.search_wd_P == false -> Reference frequency for the design frequency computation
% PID_OUT.exitflag -> Error code in design frequency searching for the control P (row: 1 x MV)
% PID_OUT.wd -> Design frequency (rad/s) (row: 1 x MV)
% PID_OUT.Ap -> Plant gain (row: 1 x MV)
% PID_OUT.Fp -> Plant phase (row: 1 x MV)
% PID_OUT.Ac -> Control gain (row: 1 x MV)
% PID_OUT.Fc -> Control phase (deg) (row: 1 x MV)
% Control parameters in parallel format (row: 1 x MV)
% PID_OUT.K
% PID_OUT.Ti
% PID_OUT.Td 
% PID_OUT.N 
% PID_OUT.b
% Configuration parameters
% PID_OUT.int_disc_type -> Discretization type for the integral action (row: 1 x MV)
% PID_OUT.der_disc_type -> Discretization type for the derivative action (row: 1 x MV)
% PID_OUT.der_input -> Derivative term connection (row: 1 x MV)
% PID_OUT.antiwindup -> Antiwindup activation (row: 1 x MV)
% Control transfer functions 
% PID_OUT.C    -> Applied to the measurement (row: 1 x MV)
% PID_OUT.Cr   -> Applied to the reference (row: 1 x MV)
% PID_OUT.G    -> Open-loop transfer function (row: 1 x MV)
% PID_OUT.Am   -> Gain margin (row: 1 x MV)
% PID_OUT.AmdB -> Gain margin (dB) (row: 1 x MV)
% PID_OUT.Fm   -> Phase margin (deg) (row: 1 x MV)
% PID_OUT.wu   -> Ultimate frequency (rad/s) (row: 1 x MV)
% PID_OUT.wo   -> Crossover frequency (rad/s) (row: 1 x MV)
% PID_OUT.S    -> Sensitivity transfer function (row: 1 x MV)
% PID_OUT.ws   -> Maximum sensitivity frequency (row: 1 x MV)
% PID_OUT.MsdB -> Maximum sensitivity (dB) (row: 1 x MV)
% PID_OUT.Fry  -> Closed-loop transfer function r(t) -> y(t) (row: 1 x MV)
% PID_OUT.Fru  -> Closed-loop transfer function r(t) -> u(t) (row: 1 x MV)
% PID_OUT.wr   -> Resonance frequency (rad/s) (row: 1 x MV)
% PID_OUT.MrdB -> Resonance peak (dB) (row: 1 x MV)
% PID_OUT.C_ss -> State-space control [r(t) y(t)]' -> u(t)
% PID_OUT.Cd_ss -> Discrete-time state-space control [r(t) y(t)]' -> u(t)
% PID_OUT.Fry_ss -> Closed-loop state-space model r(t) -> y(t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s=tf('s');
PID_OUT = PID_IN;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          PLANT             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P = PID_OUT.P;
% Transfer matrix for control design
MV = PID_IN.P.InputGroup.MV;
nMV = length(MV);
MO = PID_IN.P.OutputGroup.MO;
nMO = length(MO);
if nMV ~= nMO
    PID_OUT.error_msg = 'The numbers of manipulated variables and measured outputs must be the same.';
    return
end
P = P(MO,MV);
% State space model
P_ss = minreal(ss(P)); 
PID_OUT.P_ss = P_ss;
% Number of states
nx = size(P_ss.a,1); 
% Open-loop poles
[wnP,setaP]=damp(eig(PID_OUT.P_ss.a));
PID_OUT.wnP = wnP;
PID_OUT.setaP = setaP;
% Transfer matrix
P = minreal(zpk(P));
P.DisplayFormat = 'TimeConstant';
PID_OUT.P_tf = P;
% Number of integrators
ni = zeros(nMV);
int_off = tf(0,1)*ones(nMV);
int_on = tf(0,1)*ones(nMV);
for ii = 1:nMV
    for jj = 1:nMV
        ni(ii,jj) = sum(abs(P.p{ii,jj})<=1e-8);
        int_off(ii,jj) = s^ni(ii,jj);
        int_on(ii,jj) = 1/s^ni(ii,jj);        
    end
end
int_off = zpk(int_off);
int_off.DisplayFormat = 'TimeConstant';
int_on = zpk(int_on);
int_on.DisplayFormat = 'TimeConstant';
% Plant without integrators
P_woi = minreal(P.*int_off);
P0_woi = dcgain(P_woi);
% DC gain sign for the plant without integrators
P0_woi_sign = sign(P0_woi);
% RGA matrix without integrators
PID_OUT.RGA = P0_woi.*(inv(P0_woi))';
% Dominant output for each input in RGA matrix
[~,RGA_out] = max(abs(PID_OUT.RGA'));
PID_OUT.RGA_out = RGA_out;
% Decoupling
ts = PID_OUT.ts;
switch PID_OUT.decoupling_type
    case 0 % Without decoupling  
        % MO and MV are associated and sign changes are applied to avoid 
        % positive feedback if DC gain is negative        
        aux1 = diag(diag(P0_woi_sign(:,RGA_out)));
        aux1 = aux1 + (aux1==0);
        P_hat = aux1.*P(:,RGA_out);
        matA_dcp = [];
        matB_dcp = [];
        matC_dcp = [];
        % Sign change and ordering of MV
        aux2 = diag(diag(aux1));
        matD_dcp = aux2(RGA_out,:);
        D_ss = minreal(ss(matA_dcp,matB_dcp,matC_dcp,matD_dcp));
        D_tf = minreal(zpk(D_ss));
        D_tf.DisplayFormat = 'TimeConstant';
    case 1 % Feedforward static decoupling                
        P_hat = (P_woi/P0_woi).*int_on;
        matA_dcp = [];
        matB_dcp = [];
        matC_dcp = [];
        matD_dcp = inv(P0_woi);
        D_ss = minreal(ss(matA_dcp,matB_dcp,matC_dcp,matD_dcp));
        D_tf = minreal(zpk(D_ss));
        D_tf.DisplayFormat = 'TimeConstant';
    case 2 % Feedforward dynamic decoupling
        % Transmission zeros with positive real part
        ceros=tzero(P);
        aux_ceros=(real(ceros)>=1e-5);
        ceros_pos=ceros(aux_ceros);
        % Transfer function to compensate for unstable transmission zeros 
        if isempty(ceros_pos)
            Pzp=1;
        else
            Pzp=zpk(ceros_pos,[],1);
            Pzp=Pzp/dcgain(Pzp);
            Pzp.DisplayFormat = 'TimeConstant';
        end
        % Transfer matrix definition: diagonal matrix.
        % The transfer function un the position i is the sum of all the  
        % transfer functions in the row i of the plant transfer matrix
        P_hat = tf(0,1)*ones(nMV);
        for ii = 1:nMV
            aux1 = tf(0,1);
            for jj = 1:nMV
                aux1 = minreal(aux1 + P(ii,jj));
            end
            [~,den] = tfdata(aux1,'v');
            p = roots(den);
            [~,p_ord] = sort(real(p),'descend');
            num_hat = prod(-p(p_ord(1:2))+(p(p_ord(1:2))==0));
            den_hat = poly(p(p_ord(1:2)));
            P_hat(ii,ii) = minreal(tf(num_hat,den_hat)*Pzp);
            % P_hat(ii,ii) = minreal(tf(1,aux1.den)*Pzp);
        end
        P_hat = zpk(P_hat);
        P_hat.DisplayFormat = 'TimeConstant';      
        % Decoupling network
        D_tf = minreal(zpk(P\P_hat));
        D_tf.DisplayFormat = 'TimeConstant';
        D_ss = minreal(ss(D_tf));     
    case 3
        % dx/dt matA*x + matB*u
        % y = matC*x + matD*u  
        % C(0) = matC            D(0) = matD
        % dy/dt = matC*matA*x + matC*matB*u + matD*du/dt
        % C(1) = C(0)*matA       D(1) = C(0)*matB  
        % d2y/dt2 = matC*matA^2*x + matC*matA*matB*u + matC*matB*du/dt + matD*d2u/dt2
        % C(2) = C(1)*matA       D(2)=C(1)*matB
        % dny/dtn = matC*matA^n*x + matC*matA^(n-1)*B*u + D(n-1)*du/dt + D(n-2)*d2u/dt2 + ...
        % C(n) = C(n-1)*matA     D(n)=C(n-1)*matB
        % A(n)*dny/dtn + ... + A(1)*dy/dt + A(0)*y = uc
        % uc = (A(n)*C(n) +...+ A(1)*C(1) + A(0)*C(0))*x + (A(n)*D(n) +...+ A(1)*D(1) + A(0)*D(0))*u
        % u = (A(n)*D(n) +...+ A(1)*D(1) + A(0)*D(0))\(uc - (A(n)*C(n) +...+ A(1)*C(1) + A(0)*C(0))*x)
        % matUc = inv(A(n)*D(n) +...+ A(1)*D(1) + A(0)*D(0))
        % matX = -matUc*(A(n)*C(n) +...+ A(1)*C(1) + A(0)*C(0))
        % Derivative order computation for each output

        % A 3D matrix with polynommial coefficients is defined
        % This matrix is used to define decoupled transfer functions
        A_aux = zeros(nMV,nMV,nx+1);
        for ii = 1:nMV
            aux1 = tf(0,1);
            for jj = 1:nMV
                aux1 = minreal(aux1 + P(ii,jj));
            end
            [~,den] = tfdata(aux1,'v');
            p = roots(den);
            [~,p_ord] = sort(real(p),'descend');
            num_hat = prod(-p(p_ord(1:2))+(p(p_ord(1:2))==0));
            den_hat = poly(p(p_ord(1:2)));
            A_aux(ii,ii,end-2:end) = den_hat/num_hat;
        end          
        matA = P_ss.a;
        matB = P_ss.b;
        matC = P_ss.c;
        matD = P_ss.d;
        D_aux = matD;
        C_aux = matC;
        H1_aux = A_aux(:,:,end)*D_aux;
        H2_aux = A_aux(:,:,end)*C_aux;
        DY_ord = (sum(D_aux,2)==0);
        flag_aux = true;
        nn = 1;
        while flag_aux 
            D_aux = C_aux*matB;
            C_aux = C_aux*matA;
            H1_aux = H1_aux + A_aux(:,:,end-nn)*D_aux;
            H2_aux = H2_aux + A_aux(:,:,end-nn)*C_aux;
            DY_ord = DY_ord + (sum(D_aux,2)==0);
            flag_aux = (abs(sum(sum(D_aux,2)))==0);
            nn = nn + 1;
        end   
        P_hat = tf(0,1)*ones(nMV);
        for ii=1:nMV
            P_hat(ii,ii) = tf(1,squeeze(A_aux(ii,ii,end-DY_ord(ii):end))');
        end
        P_hat = zpk(P_hat);
        P_hat.DisplayFormat = 'TimeConstant';      
        matUc = inv(H1_aux);
        matX = -matUc*H2_aux;
        matA_dcp = [];
        matB_dcp = [];
        matC_dcp = [];
        matD_dcp = [matUc matX];
        D_ss = minreal(ss(matA_dcp,matB_dcp,matC_dcp,matD_dcp));
        D_tf = minreal(zpk(D_ss));
        D_tf.DisplayFormat = 'TimeConstant';
    otherwise
        PID_OUT.error_msg = 'This option for decoupling is not valid.';
        return        
end
PID_OUT.P_hat = P_hat;
PID_OUT.D_ss = D_ss;
PID_OUT.D_tf = D_tf;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Control specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for nn = 1:nMV
    % Plant
    P = P_hat(nn,nn);
    if PID_OUT.design_method == 0  % Frequency response design
        % Phase margin
        Fm = PID_OUT.phase_margin_deg(nn);
        % Gain margin dB
        AmdB = PID_OUT.gain_margin_dB(nn);
        Am = 10^(AmdB/20);
        % One of the stability margins must be 0
        if abs(Fm*AmdB)>0
            PID_OUT.error_msg = 'The stability margin not used must be 0.';
            return
        end
    else  % Time response design
        % Damping factor
        seta = PID_OUT.damping_factor(nn);
        [numP,denP] = tfdata(minreal(P,1e-4),'v');
        numP = numP(end-2:end)/denP(end-2);
        denP = denP(end-2:end)/denP(end-2);       
    end    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Frequency(rad/s) used as reference for control design
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    wd_ref = PID_OUT.wd_ref(nn);
    % Look for design frequency for control P
    if PID_OUT.design_method == 0   % Frequency response design    
        if PID_OUT.search_wd_P(nn)
            options=optimoptions('fsolve','Display','none','Diagnostics','off');
            [wd_P,~,exitflag]=fsolve(@(w) rem(-180+Fm-180/pi*angle(freqresp(P,w)),360),wd_ref,options);
        else % Frequency for control P is not searched
            wd_P = wd_ref;
            exitflag = 1 - (PID_OUT.control_type==1); % exitflag = 0 for control P
        end
    else  % Time respose design
        if PID_OUT.search_wd_P(nn)
            % a0 = denP(end) + k*numP(end);
            % a1 = denP(end-1) + k*numP(end-1);
            % wn^2 = a0
            % 2*seta*wn = a1
            % wn = a1/2/seta;
            % a1^2/4/seta^2 = a0
            options=optimoptions('fsolve','Display','none','Diagnostics','off');
            [Kp,fval,exitflag]=fsolve(@(k) ...
                (denP(end-1)+k*numP(end-1))^2/4/seta^2-denP(end)-k*numP(end),2,options);
            % [Kp,fval,exitflag]=fsolve(@(k) get_damp(k,P)-seta,8000);
            wd_P = (denP(end-1)+Kp*numP(end-1))/2/seta;
        else % Frequency for control P is not searched
            wd_P = wd_ref;
            Kp = 0;
            exitflag = 1 - (PID_OUT.control_type==1); % exitflag = 0 for control P
        end
    end
    PID_OUT.wd_P(nn) = wd_P;
    PID_OUT.exitflag(nn) = exitflag;
    % Exit condition is checked
    if wd_P<=0 || exitflag<1
        PID_OUT.error_msg{nn} = 'The PID can not be designed using the current specifications.';
        return
    end   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 1. CONTROL P        /  2. CONTROL PI /
    % 3. CONTROL PD_ERR   /  4. CONTROL PD_OUT
    % 5. CONTROL PID_ERR  /  6. CONTROL PID_OUT
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    switch PID_OUT.control_type(nn)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CONTROL P
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 1
            % Design frequency (rad/s)
            wd = PID_OUT.wd_P(nn);
            if PID_OUT.design_method == 0   % Frequency response design
                % Plant gain
                Ap = abs(freqresp(P,wd));
                % Control gain
                Ac = 1/Ap/Am;
                % Control phase (deg)
                Fic = 0;
                % Control gain
                Kp = 1/Ap/Am;
            end
            % Control parameters
            K = Kp;
            Ti = 0;
            Td = 0;
            N = 1;
            b = 1;
            % Configuration parameters
            int_disc_type = 0;
            der_disc_type = 0;
            der_input = boolean(0);
            antiwindup = boolean(0);
            % Control transfer functions: 2 degrees of freedom
            C  = K;
            Cr = C;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % CONTROL PI
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 2 % PI
            if PID_OUT.k_wd_P(nn) >= 1
                PID_OUT.error_msg{nn} = 'The factor for design frequency computation must be < 1.';
                return
            end
            % Design frequency (rad/s)
            wd = PID_OUT.k_wd_P(nn)*PID_OUT.wd_P(nn);
            if PID_OUT.design_method == 0   % Frequency response design
                % Plant gain
                Ap=abs(freqresp(P,wd));
                % Control gain
                Ac=1/Ap/Am;
                % Control phase (deg)
                Fic = -180+Fm-180/pi*angle(freqresp(P,wd));
                % Control parameters
                I = tand(90+Fic)/wd;
                Kp = Ac*I*wd/sqrt(1+(I*wd)^2);
                K = Kp;
                Ti = I;                
            else % Time respose design
                % a0 = K/Ti*numP(end);
                % a1 = denP(end) + K/Ti*numP(end-1) + K*numP(end);
                % a2 = denP(end-1) + K*numP(end-1)
                % (s^2+2*seta*wn*s+wn^2)*(s-p)
                % -p*wn^2 = a0
                % wn^2 - 2*seta*wn*p = a1
                % 2*seta*wn - p = a2;
                % x = [p K/Ti K]
                options=optimoptions('fsolve','Display','none','Diagnostics','off');
                [sol,fval,exitflag]=fsolve(@(x)  ...
                    [-x(1)*wd^2-x(2)*numP(end) ; ...
                    wd^2 - 2*seta*wd*x(1) - denP(end) - x(2)*numP(end-1) - x(3)*numP(end) ; ...
                    2*seta*wd - x(1) - denP(end-1) - x(3)*numP(end-1)],[-1 ; 1 ; 1],options);
                p = sol(1);
                % Control parameters
                K = sol(3);
                Ti = K/sol(2);
            end
            Td = 0;
            N = 1;
            b = PID_OUT.b(nn);
            % Configuration parameters
            int_disc_type = PID_OUT.int_disc_type(nn);
            der_disc_type = 0;
            der_input = boolean(0);
            antiwindup = PID_OUT.antiwindup(nn);
            % Control transfer functions: 2 degrees of freedom
            C  = K*(1 + 1/Ti/s);
            Cr = K*(b + 1/Ti/s);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % CONTROL PD
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case {3,4} % PD
            if PID_OUT.k_wd_P(nn) < 1
                PID_OUT.error_msg{nn} = 'The factor for design frequency computation must be >=1.';
                return
            end
            % Design frequency (rad/s)
            wd = PID_OUT.k_wd_P(nn)*PID_OUT.wd_P(nn);
            if PID_OUT.design_method == 0   % Frequency response design
                % Plant gain
                Ap=abs(freqresp(P,wd));
                % Control gain
                Ac=1/Ap/Am;
                % Control phase (deg)
                % Maximum lead phase: Fic_max=asin(1-f)/(1+f)
                Fic = -180+Fm-180/pi*angle(freqresp(P,wd));
                % Control parameters
                f = PID_OUT.f(nn);
                if PID_OUT.D_lower(nn)
                    D = ((1/f-1)/2/tand(Fic) - sqrt(((1/f-1)/2/tand(Fic))^2-1/f))/wd;
                else
                    % Solution with higher D y lower Kp
                    D = ((1/f-1)/2/tand(Fic) + sqrt(((1/f-1)/2/tand(Fic))^2-1/f))/wd;
                end
                if abs(imag(D))>0
                    PID_OUT.error_msg{nn} = 'The parameter D is complex.';
                    return
                end
                Kp = Ac*sqrt(1+(D*f*wd)^2)/sqrt(1+(D*wd)^2);
                K = Kp;
                Td = (1-f)*D;
                N = 1/f-1;
            else % Time respose design
                % a0 = (denP(end) + K*numP(end))/(1 + K*Td*numP(end-1));
                % a1 = (denP(end-1) + K*numP(end-1) + K*Td*numP(end))/(1 + K*Td*numP(end-1));
                % s^2+2*seta*wn*s+wn^2
                % wn^2 = a0
                % 2*seta*wn = a1
                % x = [K K*Td]
                options=optimoptions('fsolve','Display','none','Diagnostics','off');
                [sol,fval,exitflag]=fsolve(@(x)  ...
                    [wd^2-(denP(end) + x(1)*numP(end))/(1 + x(2)*numP(end-1)) ; ...
                    2*seta*wd - (denP(end-1) + x(1)*numP(end-1) + x(2)*numP(end))/(1 + x(2)*numP(end-1))],[1 ; 1],options);
                K = sol(1);
                Td = sol(2)/K;
                N = PID_OUT.N(nn);
            end
            Ti = 0;
            b = 1;
            % Configuration parameters
            int_disc_type = 0;
            der_disc_type = PID_OUT.der_disc_type(nn);
            der_input = (PID_OUT.control_type(nn)==4);
            antiwindup = boolean(0);
            % Control transfer functions: 2 degrees of freedom
            C = K*(1+Td*s/(1+Td/N*s));
            if PID_OUT.control_type == 3 % Connected to error
                Cr = C;
            else % Connected to output
                Cr = K*(1+Td*s/(1+Td/N*s));
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % CONTROL PID
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case {5,6} % PID
            % Design frequency (rad/s)
            wd = PID_OUT.k_wd_P(nn)*PID_OUT.wd_P(nn);
            if PID_OUT.design_method == 0   % Frequency response design
                % Plant gain
                Ap=abs(freqresp(P,wd));
                % Control gain
                Ac=1/Ap/Am;
                % Control phase (deg)
                % Maximum lead phase: Fic_max=asin(1-f)/(1+f)
                Fic=-180+Fm-180/pi*angle(freqresp(P,wd));
                % Control lag phase (deg)
                Fir = PID_OUT.lag_phase_deg(nn);
                % Control lead phase (deg)
                Fia = Fic - Fir;
                % Control parameters
                I = tand(90+Fir)/wd;
                f = PID_OUT.f(nn);
                if PID_OUT.D_lower(nn)
                    D = ((1/f-1)/2/tand(Fia) - sqrt(((1/f-1)/2/tand(Fia))^2-1/f))/wd;
                else
                    % Solution with higher D y lower Kp
                    D = ((1/f-1)/2/tand(Fia) + sqrt(((1/f-1)/2/tand(Fia))^2-1/f))/wd;
                end
                if abs(imag(D))>0
                    PID_OUT.error_msg{nn} = 'The parameter D is complex.';
                    return
                end
                Kp = Ac*sqrt(1+(D*f*wd)^2)/sqrt(1+(D*wd)^2)*I*wd/sqrt(1+(I*wd)^2);
                nu = 1+(1-f)*D/I;
                K = nu*Kp;
                Ti = nu*I;
                Td = (1/nu-f)*D;
                N = 1/nu/f-1;
            else % Time respose design
                % delta = wn*Ti between 1 and 5. Faster and more noisy for 1
                delta = PID_OUT.delta(nn);
                Ti = delta/wd;
                % a0 = K*numP(end)/(Ti + K*Td*Ti*numP(end-1));
                % a1 = (Ti*denP(end) + K*numP(end-1) + K*Ti*numP(end))/(Ti + K*Td*Ti*numP(end-1));
                % a2 = (Ti*denP(end-1) + K*Ti*numP(end-1) + K*Td*Ti*numP(end))/(Ti + K*Td*Ti*numP(end-1))
                % (s^2+2*seta*wn*s+wn^2)*(s-p)
                % -p*wn^2 = a0
                % wn^2 - 2*seta*wn*p = a1
                % 2*seta*wn - p = a2;
                % x = [p K K*Td]
                options=optimoptions('fsolve','Display','none','Diagnostics','off');
                [sol,fval,exitflag]=fsolve(@(x)  ...
                    [-x(1)*wd^2-x(2)*numP(end)/(Ti + x(3)*Ti*numP(end-1)) ; ...
                    wd^2 - 2*seta*wd*x(1) - (Ti*denP(end) + x(2)*numP(end-1) + x(2)*Ti*numP(end))/(Ti + x(3)*Ti*numP(end-1)) ; ...
                    2*seta*wd - x(1) - (Ti*denP(end-1) + x(2)*Ti*numP(end-1)+ x(3)*Ti*numP(end))/(Ti + x(3)*Ti*numP(end-1))],[-1 ; 1 ; 1],options);
                % Control parameters
                K = sol(2);
                Td = sol(3)/K;
                N = PID_OUT.N(nn);               
            end
            b = PID_OUT.b(nn);
            % Configuration parameters
            int_disc_type = PID_OUT.int_disc_type(nn);
            der_disc_type = PID_OUT.der_disc_type(nn);
            der_input = (PID_OUT.control_type(nn)==6);
            antiwindup = PID_OUT.antiwindup(nn);
            % Control transfer functions: 2 degrees of freedom
            C = K*(1+Td*s/(1+Td/N*s)+1/Ti/s);
            if PID_OUT.control_type(nn) == 5 % Connected to error
                Cr = C;
            else % Connected to output
                Cr = K*(b+1/Ti/s);
            end
            
        otherwise
            PID_OUT.error_msg{nn} = 'This is not a valid control type.';
            return
    end
    
    if PID_OUT.design_method == 0   % Frequency response design
        PID_OUT.Ac(nn) = Ac;
        PID_OUT.Ac(nn) = Fic;
        PID_OUT.Ap(nn) = Ap;
        PID_OUT.Fp(nn) = -180+Fm-Fic;
    end
    % Design frequency (rad/s)
    PID_OUT.wd(nn) = wd;
    PID_OUT.K(nn) = K;
    PID_OUT.Ti(nn) = Ti;
    PID_OUT.Td(nn) = Td;
    PID_OUT.N(nn) = N;
    PID_OUT.b(nn) = b;
    C = minreal(zpk(C));
    C.DisplayFormat = 'TimeConstant';
    PID_OUT.C(nn) = C;
    Cr = minreal(zpk(Cr));
    Cr.DisplayFormat = 'TimeConstant';
    PID_OUT.Cr(nn) = Cr;
    % Discretization type for integral term
    %  / 0. Without integration / 1. Backward Euler /
    %  / 2. Fordward Euler      / 3. Trapezoidal
    PID_OUT.int_disc_type(nn) = int_disc_type;
    % Discretization type for derivative term
    %  / 0. Without derivative  / 1. Backward Euler /
    %  / 2. Fordward Euler      / 3. Trapezoidal    /
    %  / 4. Measured derivative /
    PID_OUT.der_disc_type(nn) = der_disc_type;
    % Derivative input
    %  / 0. Error / 1. Measurement /
    PID_OUT.der_input(nn) = der_input;
    % Antiwindup for integral saturation (boolean): '1' if it is applied
    PID_OUT.antiwindup(nn) = antiwindup;
    % Open-loop transfer function
    G = minreal(zpk(C*P));
    G.DisplayFormat = 'TimeConstant';
    PID_OUT.G(nn) = G;
    % Stability margins
    [Am,Fm,wu,wo] = margin(G);
    PID_OUT.Am(nn) = Am;
    PID_OUT.AmdB(nn) = 20*log10(Am);
    PID_OUT.Fm(nn) = Fm;
    PID_OUT.wu(nn) = wu;
    PID_OUT.wo(nn) = wo;
    try
        % Sensitivity transfer function
        S = minreal(zpk(1/(1+G)));
        S.DisplayFormat = 'TimeConstant';
        PID_OUT.S(nn) = S;
        % Maximum sensitivity frequency (rad/s)
        PID_OUT.ws(nn) = fminsearch(@(w) -abs(freqresp(S,w)),1);
        % Maximum sensitivity (dB)
        PID_OUT.MsdB(nn) = 20*log10(abs(freqresp(S,PID_OUT.ws(nn))));
        % Closed-loop transfer functions
        Fry = minreal(zpk(Cr*P/(1+G)));
        Fry.DisplayFormat = 'TimeConstant';
        Fru = minreal(zpk(Cr/(1+G)));
        Fru.DisplayFormat = 'TimeConstant';
        PID_OUT.Fry(nn) = Fry;
        PID_OUT.Fru(nn) = Fru;
    catch
    end    
end
%-----------------------------------------------------------
%% Control state-space model
%-----------------------------------------------------------
% Inputs: [REF MO PLANT_STATE MD]
% Outputs: MV
% PID state-space
% Xc = [xi ; xd]      Uc = [r ; y]      Yc = u
% 1/(Td/N + 1/s) -> r-y = Td/N*dxd/dt + xd
% dxi/dt = r-y
% dxd/dt = -N/Td*xd + N/Td*(a*r-y)
% u = K*(b*r-y + xi/Ti + Td*dxd/dt) = 
%   = K*(b*r-y + xi/Ti + -N*xd + N*(a*r-y)) =
%   = K*((b+a*N)*r-(1+N)*y + xi/Ti + -N*xd)
C_ss = ss([],[],[],[]);
for nn = 1:nMV
    K = PID_OUT.K(nn);
    Ti = PID_OUT.Ti(nn);
    Td = PID_OUT.Td(nn);
    N = PID_OUT.N(nn);
    b = PID_OUT.b(nn);  
    switch PID_OUT.control_type(nn)
        case 1 % P
            matAc = [];
            matBc = [];
            matCc = [];
            matDc = [K -K];
            C_aux1 = ss(matAc,matBc,matCc,matDc);
        case 2 % PI
            matAc = 0;
            matBc = [1 -1];
            matCc = K/Ti;
            matDc = [K*b -K];
            C_aux1 = ss(matAc,matBc,matCc,matDc);
        case {3,4}
            if PID_OUT.control_type(nn)==3
                a = 1;
            else
                a = 0;
            end
            matAc = -N/Td;
            matBc = [a*N/Td -N/Td];
            matCc = -K*N;
            matDc = [K*(1+a*N) -K*(1+N)];
            C_aux1 = ss(matAc,matBc,matCc,matDc);
        case {5,6}
            if PID_OUT.control_type(nn)==5
                a = 1;
            else
                a = 0;
            end
            matAc = [0 0 ; 0 -N/Td];
            matBc = [1 -1 ; a*N/Td -N/Td];
            matCc = [K/Ti -K*N];
            matDc = [K*(b+a*N)  -K*(1+N)];
            C_aux1 = ss(matAc,matBc,matCc,matDc);
        otherwise
    end
    % Append control state-space models
    C_ss = append(C_ss,C_aux1);
    % Reorder inputs [REF MO]
    C_ss = C_ss(:,[1:nn-1 2*nn-1 nn:2*nn-2 2*nn]);
end
% Decoupling network is added
C_ss = series(C_ss,D_ss,1:nMV,1:nMV);
if PID_OUT.decoupling_type == 3
    % Plant states are added as inputs
    matA = C_ss.a;
    matB = C_ss.b;
    matC = C_ss.c;
    matD = C_ss.d;
    nx = size(matB,1);
    nu = size(matB,2);
    nxd = size(D_ss.d,2)-nMV;
    matB = [matB zeros(nx,nxd)];
    matD = [matD D_ss.d(:,nMV+1:end)];
    C_ss = ss(matA,matB,matC,matD);   
end
PID_OUT.C_ss = C_ss;
% Discretization
PID_OUT.Cd_ss = c2d(PID_OUT.C_ss,PID_OUT.ts,'tustin');
%-----------------------------------------------------------
%% Closed-loop state-space model
%-----------------------------------------------------------
P_ss = ss(PID_OUT.P);
P_ss = P_ss(MO,:);
if PID_OUT.decoupling_type == 3
    % Plant states are added as outputs
    matA = P_ss.a;
    matB = P_ss.b;
    matC = P_ss.c;
    matD = P_ss.d;
    nx = size(matC,2);
    nu = size(matD,2);
    matC = [matC ; eye(nx)];
    matD = [matD ; zeros(nx,nu)];
    P_ss = ss(matA,matB,matC,matD);   
end
% Open-loop state space model 
% It is assumed that the first inputs are the manipulated variables (MV)
G_ss = series(C_ss,P_ss,1:nMV,1:nMV);
% Disturbances are added to open-loop model
matBp = P_ss.b;
matDp = P_ss.d;
nxp = size(matBp,1);
nup = size(matBp,2);
nd = nup - nMV;
matAg = G_ss.a;
matBg = G_ss.b;
matCg = G_ss.c;
matDg = G_ss.d;
nxg = size(matBg,1);
matBg = [matBg [matBp(:,nMV+1:end) ; zeros(nxg-nxp,nd)]];
matDg = [matDg matDp(:,nMV+1:end)];
G_ss = ss(matAg,matBg,matCg,matDg);
% Feedback
if PID_OUT.decoupling_type == 3
    nx = size(PID_OUT.P_ss.a,1);
    F_ss = feedback(G_ss,-eye(nMO+nx),nMV+1:2*nMV+nx,1:(nMO+nx));
    % Extra inputs and outputs are deleted
    matA = F_ss.a;
    matB = F_ss.b;
    matC = F_ss.c;
    matD = F_ss.d;
    matB = matB(:,[1:nMV (2*nMV+nx+1):(2*nMV+nx+nd)]);
    matC = matC(1:nMV,:);
    matD = matD(1:nMV,[1:nMV (2*nMV+nx+1):(2*nMV+nx+nd)]);
    F_ss = ss(matA,matB,matC,matD);
else
    F_ss = feedback(G_ss,-eye(nMO),nMO+1:2*nMO,1:nMO);
    % Extra inputs and outputs are deleted
    matA = F_ss.a;
    matB = F_ss.b;
    matC = F_ss.c;
    matD = F_ss.d;
    matB = matB(:,[1:nMV (2*nMV+1):(2*nMV+nd)]);
    matC = matC(1:nMV,:);
    matD = matD(1:nMV,[1:nMV (2*nMV+1):(2*nMV+nd)]);
    F_ss = ss(matA,matB,matC,matD);
end
PID_OUT.F_ss = F_ss;

return
