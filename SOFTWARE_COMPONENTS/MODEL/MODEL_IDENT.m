%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%  LEAST-SQUARES MODEL IDENTIFICATION ALGORITHM  %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------
%% FIT MODE
%--------------------------------------------------------------
% 1  : MOTOR RATE LEFT
% 2  : MOTOR RATE RIGHT
% 3  : MOTOR RATE COM
% 4  : MOTOR RATE DIF
% 5  : YAW RATE
% 6  : FORWARD VELOCITY
% 7  : PITCH RATE
% 8  : PITCH ANGLE
% 9  : WALL DISTANCE & YAW RATE
% 10  : FORWARD VELOCITY AND YAW RATE
FIT_MODE = 5;
%--------------------------------------------------------------
%% GENERAL CONFIGURATION
%--------------------------------------------------------------
clc
tic
clear theta thaux dgn J
format compact
format short e
cd ../../CONFIGURATION
%--------------------------------------------------------------
% SET RUN_MODE = 2 (TEST VERIFICATION) IN CONFIG_CAR
%--------------------------------------------------------------
CONFIG_CAR 
if RUN_MODE~=2
    disp('RUN_MODE must be = 2 in CONFIG_CAR.m')
    return
end
MODEL_SLX = 'CAR_CONTROL_SYSTEM';
clc
% Start time for cost function computation
startTime = 2;
% endTime = SCOPE_TEST.time(end);
endTime = SCOPE_DATA.time(end);
% Flag to enable noise in simulation
MODEL_INI.PARAM.NOISE_FLAG = uint8(0);
% SCALING FACTORS FOR FITTED OUTPUTS
factor = ones(2,1);

%--------------------------------------------------------------
%% OUTPUT STRING LIST
%--------------------------------------------------------------
OUT_STRING = { ...
'MOTOR RATE LEFT (rad/s)'
'MOTOR RATE RIGHT (rad/s)'
'MOTOR RATE COM (rad/s)'
'MOTOR RATE DIF (rad/s)'
'YAW RATE (rad/s)'
'FORWARD VELOCITY (m/s)'
'WALL DISTANCE (m)'
'PITCH RATE (rad/s)'
'PITCH ANGLE (deg)'
};

%--------------------------------------------------------------
%% PARAMETER STRING LIST
%--------------------------------------------------------------
PARAM_STRING = { ...
'MODEL_INI.PARAM.MOTOR_RESISTANCE'
'MODEL_INI.PARAM.MOTOR_BEMF_CONSTANT'
'MODEL_INI.PARAM.MOTOR_TORQUE_CONSTANT'
'MODEL_INI.PARAM.MOTOR_INERTIA'
'MODEL_INI.PARAM.MOTOR_VISCOUS_FRICTION_COEFF'
'MODEL_INI.PARAM.MOTOR_STATIC_TORQUE'
'MODEL_INI.PARAM.MOTOR_VOLT_DROP_COM'
'MODEL_INI.PARAM.MOTOR_VOLT_DROP_DIF'
'MODEL_INI.PARAM.BODY_INERTIA_Z'
'MODEL_INI.PARAM.BODY_VISCOUS_FRICTION_COEFF_Z'
'MODEL_INI.PARAM.BODY_INERTIA_Y'
'MODEL_INI.PARAM.BODY_DISTANCE'
'CONTROL_INI.PARAM.MOTOR_DEAD_ZONE(1)'
'CONTROL_INI.PARAM.MOTOR_DEAD_ZONE(2)'
}; 

NUM_PARAM = length(PARAM_STRING);

%--------------------------------------------------------------
%% FIT MODE CONFIGURATION 
%--------------------------------------------------------------
switch(FIT_MODE)
    %-------------------------------------
    case 1 % MOTOR RATE LEFT
    %-------------------------------------    
        PARAM_LIST = [1 2 4 5 6 7]; 
        % Fitted outputs in SCOPE_TEST
        test_fit = [9 3];
        sim_fit = [9 1];  
        % Labels in plot
        OUT_STRING_POS = 1;
    %-------------------------------------    
    case 2 % MOTOR RATE RIGHT
    %-------------------------------------    
        PARAM_LIST = [1 2 4 5 6 7]; 
        % Fitted outputs in SCOPE_TEST
        test_fit = [9 4];
        sim_fit = [9 2];  
        % Labels in plot
        OUT_STRING_POS = 2;
    %-------------------------------------
    case 3 % MOTOR RATE COM
    %-------------------------------------    
        PARAM_LIST = [4 5 6]; 
        % Fitted outputs in SCOPE_TEST
        test_fit = [10 3];
        sim_fit = [10 1];  
        % Labels in plot
        OUT_STRING_POS = 3;
    %-------------------------------------    
    case 4 % MOTOR RATE DIF
    %-------------------------------------    
        PARAM_LIST = [4 6]; 
        % Fitted outputs in SCOPE_TEST
        test_fit = [10 4];
        sim_fit = [10 2];  
        % Labels in plot
        OUT_STRING_POS = 4;
    %-------------------------------------    
    case 5 % YAW RATE
    %-------------------------------------    
        % PARAM_LIST = [1 2 4 5 6]; 
        PARAM_LIST = [8 9 10];
        % Fitted outputs in SCOPE_TEST
        test_fit = [2 3];
        sim_fit = [2 2];  
        % Labels in plot
        OUT_STRING_POS = 5;
    %-------------------------------------    
    case 6 % FORWARD VELOCITY
    %------------------------------------- 
        % PARAM_LIST = [1 2 3 4 5 6 7]; 
        % PARAM_LIST = [1 2 4 5 6];
        PARAM_LIST = [2 4 6];
        % Fitted outputs in SCOPE_TEST
        test_fit = [1 3];
        sim_fit = [1 2];  
        % Labels in plot
        OUT_STRING_POS = 6;
    %-------------------------------------    
    case 7 % PITCH RATE
    %------------------------------------- 
        PARAM_LIST = [11 12];
        % Fitted outputs in SCOPE_TEST
        test_fit = [11 2];
        sim_fit = [11 1];  
        % Labels in plot
        OUT_STRING_POS = 8;
    %-------------------------------------    
    case 8 % PITCH ANGLE
    %------------------------------------- 
        PARAM_LIST = 11;
        % Fitted outputs in SCOPE_TEST
        test_fit = [12 3];
        sim_fit = [12 2];  
        % Labels in plot
        OUT_STRING_POS = 9;
    %-------------------------------------    
    case 9 % WALL DISTANCE & YAW RATE
    %-------------------------------------    
        PARAM_LIST = [9 10 11 12]; 
        % Fitted outputs in SCOPE_TEST
        test_fit = [4 3 ; 4 4];
        sim_fit = [4 1 ; 4 2];  
        % Labels in plot
        OUT_STRING_POS = [8 5];   
    %-------------------------------------    
    case 10 % FORWARD VELOCITY & YAW RATE
    %-------------------------------------    
        PARAM_LIST = [2 3 4 6 8 9 10]; 
        % Fitted outputs in SCOPE_TEST
        test_fit = [1 3 ; 2 3];
        sim_fit = [1 2 ; 2 2];  
        % Labels in plot
        OUT_STRING_POS = [6 5];   
    %-------------------------------------    
    otherwise
    %-------------------------------------
        disp('FIT MODE IS NOT VALID')
        return
end


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
%% CONTROL FOR MODEL IDENTIFICATION TEST
%--------------------------------------------------------------
if CONTROL_INI.STATE.CONTROL_MODE==2
    CONTROL_INI.PARAM.FORWARD_VEL_PID.K = 20;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.Ti = 1;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.Td = 0;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.N = 1;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.b = 1;
    CONTROL_INI.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE = uint8(0);
    CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE = uint8(0);
    CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(0);
    CONTROL_INI.PARAM.FORWARD_VEL_PID.ANTIWINDUP = uint8(0);
    CONTROL_INI.PARAM.YAW_RATE_PID.K = 1.25;
    CONTROL_INI.PARAM.YAW_RATE_PID.Ti = 0;
    CONTROL_INI.PARAM.YAW_RATE_PID.Td = 0;
    CONTROL_INI.PARAM.YAW_RATE_PID.N = 1;
    CONTROL_INI.PARAM.YAW_RATE_PID.b = 1;
    CONTROL_INI.PARAM.YAW_RATE_PID.INT_DISC_TYPE = uint8(0);
    CONTROL_INI.PARAM.YAW_RATE_PID.DER_DISC_TYPE = uint8(0);
    CONTROL_INI.PARAM.YAW_RATE_PID.DER_INPUT = uint8(0);
    CONTROL_INI.PARAM.YAW_RATE_PID.ANTIWINDUP = uint8(0);
    CONTROL_INI.STATE.WFL_FEEDFORWARD = uint8(0);    
end
if CONTROL_INI.STATE.CONTROL_MODE==8
    CONTROL_INI.PARAM.PITCH_ANG_SFC.K = [-15 1.8 18];
end

%--------------------------------------------------------------
%% LIST OF POSITIVE PARAMETERS
%--------------------------------------------------------------
PARAM_POS = (PARAM_LIST~=8);

%--------------------------------------------------------------
%% GAUSS-NEWTON 
%--------------------------------------------------------------
% ALGORITHM PARAMETERS
ts=SAMPLING_TIME;                           % Sampling time
tfin = SCOPE_DATA.time(end);                % Final time for simulation
Nd=tfin/ts;                                 % Number of data
tol1=1;                                     % Tolerance in % for the cost function 'V'
tol2=1;                                     % Tolerance in % for parameter vector 'theta'
V=1;                                        % Initial value for the cost function
Vaux=0.01;                                  % Initial value for the cost function update
dgn=ones(1,Np);                             % Gauss-Newton search direction (parameter vector increment) 
niter=0;                                    % Number of iterations
mu=1;                                       % Modulation of Gauss_Newton search direction
%--------------------------------------------------------------
% The algorithm runs while the cost function change in % is greater than
% 'tol1' or the maximum parameter increment in % is higher than 'tol2'.
% The maximum number of iterations may also be used as termination condition 
%--------------------------------------------------------------
while (100*(V-Vaux)/Vaux>tol1 || 100*max(abs(mu*dgn./theta))>tol2) && niter<10
	
	niter=niter+1;	% The iteration counter is incremented

    %--------------------------------------------------------------
	th=theta;
    nn = 1;
    for ii = 1:NUM_PARAM
        if min(abs(PARAM_LIST-ii))==0
            command = [PARAM_STRING{ii} ' = th(nn)*th_ini(nn); nn=nn+1;'];
            eval(command)
        end
    end
    MODEL_INI.PARAM.MOTOR_TORQUE_CONSTANT = MODEL_INI.PARAM.MOTOR_BEMF_CONSTANT;
    % Simulation	
    warning('off','all')
    try
        sim(MODEL_SLX)
        flag_sim = 1;
    catch
        flag_sim = 0;
    end
    ym = [];
    ys = [];
    for ii = 1:size(sim_fit,1)
        factor(ii) = sqrt(mean(SCOPE_TEST.signals(test_fit(ii,1)).values(:,test_fit(ii,2)).^2));
        ym = [ym SCOPE_TEST.signals(test_fit(ii,1)).values(:,test_fit(ii,2))/factor(ii)]; % TEST
        
        if flag_sim==1
            ys = [ys SCOPE_TEST.signals(sim_fit(ii,1)).values(:,sim_fit(ii,2))/factor(ii)]; % SIM
        else
            ys = [ys zeros(length(SCOPE_TEST.time),1)];
        end
    end
    % Error computation (column vector)    
    ym = ym(SCOPE_TEST.time>startTime & SCOPE_TEST.time<=endTime,:);
    ys = ys(SCOPE_TEST.time>startTime & SCOPE_TEST.time<=endTime,:);
    ym = ym(:);
    ys = ys(:);
    error=ys-ym;
	% Cost function: mean squared error	
	V=sqrt(sum((error).^2)/Nd);    		
	% Jacobian matrix calculated by finite difference method
    for i=1:Np
        thaux=theta;
        % Increment for output derivatives with respect to parameters
        h=0.1*abs(theta(i));
        if abs(theta(i))<10*sqrt(eps)
            h=0.1;
        end
        thaux(i)=theta(i)+h;
        % Parameter update
        th=thaux;
        nn = 1;
        for ii = 1:NUM_PARAM
            if min(abs(PARAM_LIST-ii))==0
                command = [PARAM_STRING{ii} ' = th(nn)*th_ini(nn); nn=nn+1;'];
                eval(command)
            end
        end
        MODEL_INI.PARAM.MOTOR_TORQUE_CONSTANT = MODEL_INI.PARAM.MOTOR_BEMF_CONSTANT;
        % Simulation
        warning('off','all')
        try
            sim(MODEL_SLX)
            flag_sim=1;
        catch
            flag_sim=0;
        end
        yaux = [];
        for ii = 1:size(sim_fit,1)
            if flag_sim==1
                yaux = [yaux SCOPE_TEST.signals(sim_fit(ii,1)).values(:,sim_fit(ii,2))/factor(ii)]; % OUTPUT
            else
                yaux = [yaux zeros(length(SCOPE_TEST.time),1)];
            end
        end
        yaux = yaux(SCOPE_TEST.time>startTime & SCOPE_TEST.time<=endTime,:);
        yaux = yaux(:);
        % Jacobian matrix              
        J(:,i)=(yaux-ys)/h;
    end    
    dgn=(J\(ym-ys)); % Gauss-Newton search direction
    
    % SEARCH OF 'MU' (tunning of Gauss-Newton search-direction step to
    % ensure the cost function reduction)
    mu=1;  % mu initialization
    Vaux=V+10; % Initialization of the cost function update
    % 'mu' is divided by 2 while the cost function is not reduced
    thaux=theta;
    % Stability flag
    flag_sim = 0;
    while Vaux > V || flag_sim == 0 || any(thaux(PARAM_POS)<0) 
        mu=mu/2; % Step reduction
        % Parameter update        
        thaux=theta+mu*dgn(:); 
        th=thaux;
        nn = 1;
        for ii = 1:NUM_PARAM
            if min(abs(PARAM_LIST-ii))==0
                command = [PARAM_STRING{ii} ' = th(nn)*th_ini(nn); nn=nn+1;'];
                eval(command)
            end
        end
        MODEL_INI.PARAM.MOTOR_TORQUE_CONSTANT = MODEL_INI.PARAM.MOTOR_BEMF_CONSTANT;
        % Simulation
        warning('off','all')
        try
            sim(MODEL_SLX)
            flag_sim = 1;
        catch
            flag_sim = 0;
        end
        yaux = [];
        for ii = 1:size(sim_fit,1)
            if flag_sim==1
                yaux = [yaux SCOPE_TEST.signals(sim_fit(ii,1)).values(:,sim_fit(ii,2))/factor(ii)]; % OUTPUT
            else
                yaux = [yaux zeros(length(SCOPE_TEST.time),1)];
            end
        end
        yaux = yaux(SCOPE_TEST.time>startTime & SCOPE_TEST.time<=endTime,:);
        yaux = yaux(:);
        % Error computation (column vector)                
        error=yaux-ym;
        % Cost function update
        Vaux=sqrt(sum((yaux-ym).^2)/Nd);
    end
        
	% The parameters and the cost function are shown on the screen
	theta=thaux;
    fprintf('-------- ITERATION NUMBER: %d --------\n',niter)    
	disp('PARAMETERS')    
    disp(theta(:)')
    disp('COST FUNCTION')
	disp(Vaux)
    % Graphical representation 
    clf
    figure(1)
    time = SCOPE_TEST.time(SCOPE_TEST.time>startTime & SCOPE_TEST.time<=endTime);
    N_plot = length(time);
    if FIT_MODE<9 
        plot_y_label = OUT_STRING(OUT_STRING_POS);    
        plot(time,ym*factor(1),'-r',time,yaux*factor(1),'-b',time,(ym-yaux)*factor(1),'-g')
        xlabel('Time (s)')
        ylabel(plot_y_label)
        legend('Real value','Simulated value','Error')
        grid   
    else
        plot_y_label = OUT_STRING(OUT_STRING_POS); 
        subplot(211)
        plot(time,ym(1:N_plot)*factor(1),'-r',time,yaux(1:N_plot)*factor(1),'-b',time,(ym(1:N_plot)-yaux(1:N_plot))*factor(1),'-g')
        xlabel('Time (s)')
        ylabel(plot_y_label{1})
        legend('Real value','Simulated value','Error')
        grid
        subplot(212)
        plot(time,ym(N_plot+1:end)*factor(2),'-r',time,yaux(N_plot+1:end)*factor(2),'-b',time,(ym(N_plot+1:end)-yaux(N_plot+1:end))*factor(2),'-g')
        xlabel('Time (s)')
        ylabel(plot_y_label{2})
        legend('Real value','Simulated value','Error')
        grid
    end      
end

%--------------------------------------------------------------
%% SCREEN OUTPUT
%--------------------------------------------------------------
disp(' ')
disp('---------- FINAL PARAMETERS ------------')
disp(' ')
fprintf('%%--------------------------------------------------------------\n')
fprintf('%% MOTOR PARAMETERS\n')
fprintf('%%--------------------------------------------------------------\n')
fprintf('%% Rated voltage (V)\n')
fprintf('MODEL.PARAM.BATTERY_VOLT = 12;\n')
fprintf('%% Rotor resistance (ohm)\n')
fprintf('MODEL.PARAM.MOTOR_RESISTANCE = %g;\n',...
         MODEL_INI.PARAM.MOTOR_RESISTANCE);
fprintf('%% Motor back-emf constant (V.s/rad)\n')
fprintf('MODEL.PARAM.MOTOR_BEMF_CONSTANT = %g;\n',...
         MODEL_INI.PARAM.MOTOR_BEMF_CONSTANT);
fprintf('%% Torque constant (N.m/A)\n')
fprintf('MODEL.PARAM.MOTOR_TORQUE_CONSTANT = %g;\n',...
         MODEL_INI.PARAM.MOTOR_TORQUE_CONSTANT);
fprintf('%% Motor inertia (kg.m^2)\n')
fprintf('MODEL.PARAM.MOTOR_INERTIA = %g;\n',...
         MODEL_INI.PARAM.MOTOR_INERTIA);
fprintf('%% Viscous friction constant (N.m.s/rad)\n')
fprintf('MODEL.PARAM.MOTOR_VISCOUS_FRICTION_COEFF = %g;\n',...
         MODEL_INI.PARAM.MOTOR_VISCOUS_FRICTION_COEFF);
fprintf('%% Static friction torque (N.m)\n')
fprintf('MODEL.PARAM.MOTOR_STATIC_TORQUE = %g;\n',...
         MODEL_INI.PARAM.MOTOR_STATIC_TORQUE);
fprintf('%% Motor converter voltage drop (V)\n')
fprintf('MODEL.PARAM.MOTOR_VOLT_DROP_COM = %g;\n',...
         MODEL_INI.PARAM.MOTOR_VOLT_DROP_COM);
fprintf('MODEL.PARAM.MOTOR_VOLT_DROP_DIF = %g;\n',...
         MODEL_INI.PARAM.MOTOR_VOLT_DROP_DIF);
fprintf('%% Motor converter delay (s)\n')
fprintf('MODEL.PARAM.MOTOR_DELAY = %g;\n',MODEL_INI.PARAM.MOTOR_DELAY)
fprintf('%% Motor gear ratio\n')
fprintf('MODEL.PARAM.MOTOR_GEAR_RATIO = %g;\n\n',...
         MODEL_INI.PARAM.MOTOR_GEAR_RATIO);
fprintf('%%--------------------------------------------------------------\n')
fprintf('%% CAR PARAMETERS\n')
fprintf('%%--------------------------------------------------------------\n')
fprintf('%% Wheel mass (kg)\n')
fprintf('MODEL.PARAM.WHEEL_MASS = %g;\n',...
         MODEL_INI.PARAM.WHEEL_MASS);
fprintf('%% Wheel radius (m)\n')
fprintf('MODEL.PARAM.WHEEL_RADIUS = %g;\n',...
         MODEL_INI.PARAM.WHEEL_RADIUS);
fprintf('%% Wheel distance (m)\n')
fprintf('MODEL.PARAM.WHEEL_DISTANCE = %g;\n',...
         MODEL_INI.PARAM.WHEEL_DISTANCE);
fprintf('%% Body mass (kg)\n')
fprintf('MODEL.PARAM.BODY_MASS = %g-2*MODEL.PARAM.WHEEL_MASS'';\n',...
         MODEL_INI.PARAM.BODY_MASS+2*MODEL_INI.PARAM.WHEEL_MASS);
fprintf('%% Body inertia in X axis (kg.m^2)\n')
fprintf('MODEL.PARAM.BODY_INERTIA_X = %g;\n',...
         MODEL_INI.PARAM.BODY_INERTIA_X);
fprintf('%% Body inertia in Y axis (kg.m^2)\n')
fprintf('MODEL.PARAM.BODY_INERTIA_Y = %g;\n',...
         MODEL_INI.PARAM.BODY_INERTIA_Y);
fprintf('%% Body inertia in Z axis (kg.m^2)\n')
fprintf('MODEL.PARAM.BODY_INERTIA_Z = %g;\n',...
         MODEL_INI.PARAM.BODY_INERTIA_Z);
fprintf('%% Body viscous friction coefficient in X axis (N.m.s/rad)\n')
fprintf('MODEL.PARAM.BODY_VISCOUS_FRICTION_COEFF_X = %g;\n',...
         MODEL_INI.PARAM.BODY_VISCOUS_FRICTION_COEFF_X);
fprintf('%% Body viscous friction coefficient in Z axis (N.m.s/rad)\n')
fprintf('MODEL.PARAM.BODY_VISCOUS_FRICTION_COEFF_Z = %g;\n',...
         MODEL_INI.PARAM.BODY_VISCOUS_FRICTION_COEFF_Z);
fprintf('%% Distance fron wheel axis to body COG [m]\n')
fprintf('MODEL.PARAM.BODY_DISTANCE = %g;\n',...
         MODEL_INI.PARAM.BODY_DISTANCE);
fprintf('%% Wheel gear ratio\n')
fprintf('MODEL.PARAM.WHEEL_GEAR_RATIO = 1;\n');
fprintf('%% Pitch offset (rad)\n')
fprintf('MODEL.PARAM.IMU_PITCH_OFFSET = %g;\n\n\n',...
         MODEL_INI.PARAM.IMU_PITCH_OFFSET);

toc

% Covariance matrix for parameter vector (Sensitivities)
% P=Vaux^2*inv(J'*J); 
% disp('Confidence interval in %')
% disp(100*sqrt(diag(P)')./theta)



