%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%  LEAST-SQUARES CONTROL TUNING ALGORITHM  %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------
%% FIT MODE
%--------------------------------------------------------------
% 1  : FORWARD VELOCITY
% 2  : YAW RATE
% 3  : YAW ANGLE
% 4  : WALL DISTANCE
% 5  : PITCH ANGLE 
FIT_MODE = 5;

%--------------------------------------------------------------
%% FIT OUTPUT
%--------------------------------------------------------------
% 2  : CONTROL OUTPUT
% 3  : MODEL OUTPUT
FIT_OUTPUT = 3;

%--------------------------------------------------------------
%% REFERENCE MODELS & COMMAND FILTER
%--------------------------------------------------------------
%  FORWARD VELOCITY / YAW RATE / YAW ANGLE / WALL DISTANCE
% Closed-loop transfer-function gain
CLTF_GAIN = [1 1 1 1];
% Closed-loop transfer-function natural frequency (rad/s)
CLTF_FREQ = [10 10 5 2.5];
% Closed-loop transfer-function damping
CLTF_DAMP = [1 1 1 1];
% Command digital lowpass filter: order and cut-off frequency (pu wrt Nyquist frequency)
[numF,denF] = butter(4,0.01);

%--------------------------------------------------------------
%% OUTPUT STRING LIST
%--------------------------------------------------------------
OUT_STRING = { ...
'FORWARD VELOCITY (m/s)'
'YAW RATE (rad/s)'
'YAW ANGLE (deg)'
'WALL DISTANCE (m)'
'PITCH ANGLE (deg)'
};

%--------------------------------------------------------------
%% PARAMETER STRING LIST
%--------------------------------------------------------------
PARAM_STRING = { ...
 %------------ 1 - 9 ---------------   
'FWD_VEL_PID.phase_margin_deg'
'FWD_VEL_PID.gain_margin_dB'
'FWD_VEL_PID.k_wd_P'
'FWD_VEL_PID.lag_phase_deg'
'FWD_VEL_PID.f'
'FWD_VEL_PID.b'
'FWD_VEL_PID.damping_factor'
'FWD_VEL_PID.delta'
'FWD_VEL_PID.N'
 %------------ 10 - 18 ----------   
'YAW_RATE_PID.phase_margin_deg'
'YAW_RATE_PID.gain_margin_dB'
'YAW_RATE_PID.k_wd_P'
'YAW_RATE_PID.lag_phase_deg'
'YAW_RATE_PID.f'
'YAW_RATE_PID.b'
'YAW_RATE_PID.damping_factor'
'YAW_RATE_PID.delta'
'YAW_RATE_PID.N'
 %------------ 19 - 24 ----------   
'YAW_ANG_PID.phase_margin_deg'
'YAW_ANG_PID.gain_margin_dB'
'YAW_ANG_PID.k_wd_P'
'YAW_ANG_PID.lag_phase_deg'
'YAW_ANG_PID.f'
'YAW_ANG_PID.b'
 %------------ 25 - 29 ----------   
'WFL_SL_PID.phase_margin_deg'
'WFL_SL_PID.k_wd_P'
'WFL_SL_PID.lag_phase_deg'
'WFL_SL_PID.f'
'WFL_SL_PID.b'
 %------------ 30 - 34 ----------   
'WFL_CD_PID.phase_margin_deg'
'WFL_CD_PID.k_wd_P'
'WFL_CD_PID.lag_phase_deg'
'WFL_CD_PID.f'
'WFL_CD_PID.b'
 %------------ 35 - 46 ----------   
'VEL_SFC.damping_factor(1)'
'VEL_SFC.wn_factor(1)'
'VEL_SFC.p_factor'
'VEL_SFC.damping_factor(2)'
'VEL_SFC.wn_factor(2)'
'VEL_SFC.forward_vel_matQ(1)'
'VEL_SFC.forward_vel_matQ(2)'
'VEL_SFC.forward_vel_matQ(3)'
'VEL_SFC.forward_vel_matR'
'VEL_SFC.yaw_rate_matQ(1)'
'VEL_SFC.yaw_rate_matQ(2)'
'VEL_SFC.yaw_rate_matR'
 %------------ 47 - 53 ----------   
'YAW_ANG_SFC.wn_factor'
'YAW_ANG_SFC.damping_factor'
'YAW_ANG_SFC.p_factor'
'YAW_ANG_SFC.matQ(1)'
'YAW_ANG_SFC.matQ(2)'
'YAW_ANG_SFC.matQ(3)'
'YAW_ANG_SFC.matR'
 %------------ 54 - 60 ----------   
'WFL_SFC.natural_freq'
'WFL_SFC.damping_factor'
'WFL_SFC.p_factor'
'WFL_SFC.matQ(1)'
'WFL_SFC.matQ(2)'
'WFL_SFC.matQ(3)'
'WFL_SFC.matR'
 %------------ 61 - 67 ----------   
'PITCH_ANG_SFC.damping_factor'
'PITCH_ANG_SFC.wn_factor'
'PITCH_ANG_SFC.p_factor'
'PITCH_ANG_SFC.matQ(1)'
'PITCH_ANG_SFC.matQ(2)'
'PITCH_ANG_SFC.matR'
'IMU_CF_FREQ'
}; 

NUM_PARAM = length(PARAM_STRING);

%--------------------------------------------------------------
%% GENERAL CONFIGURATION
%--------------------------------------------------------------
clc
clear theta thaux dgn J
format compact
format short e
cd ../../CONFIGURATION 
%--------------------------------------------------------------
% SET RUN_MODE = 1 (FAST SIMULATION) IN CONFIG_CAR
% SET CONTROL.STATE.GAIN_SCHEDULING = false IN CONFIG_CONTROL
%--------------------------------------------------------------
CONFIG_CAR
MODEL_SLX = 'CAR_CONTROL_SYSTEM';
if CONTROL_INI.STATE.GAIN_SCHEDULING
    disp('Please, disable adaptive control')
    return
end
ts = SAMPLING_TIME;
% Simulation final time (s)
if FIT_MODE == 1 % FORWARD VELOCITY 
    MODEL_INI.PARAM.SIM_FINAL_TIME = 10;
    % Start time for cost function computation
    startTime = 2;
elseif FIT_MODE == 2 % YAW RATE
    MODEL_INI.PARAM.SIM_FINAL_TIME = 10;
    % Start time for cost function computation
    startTime = 2;
elseif FIT_MODE == 3 % YAW ANGLE
    MODEL_INI.PARAM.SIM_FINAL_TIME = 10;
    % Start time for cost function computation
    startTime = 2;
elseif FIT_MODE == 4 % WALL FOLLOWER
    MODEL_INI.PARAM.SIM_FINAL_TIME = 25;
    % Start time for cost function computation
    startTime = 4;
elseif FIT_MODE == 5 % PITCH ANGLE
    MODEL_INI.PARAM.SIM_FINAL_TIME = 20;
    % Start time for cost function computation
    startTime = 0;
else
    disp('FIT_MODE must be between 1 and 5')
    return    
end    
set_param(MODEL_SLX,'StopTime',num2str(MODEL_INI.PARAM.SIM_FINAL_TIME));
% OPERATING POINT
NUM_OP = CONTROL_INI.STATE.FORWARD_VEL_MAIN_OP;
% REFERENCE INITIALITATION
CONTROL_INI.STATE.FV_TARGET_TYPE = uint8(0);
CONTROL_INI.STATE.YR_TARGET_TYPE = uint8(0);
CONTROL_INI.STATE.YA_TARGET_TYPE = uint8(0);
CONTROL_INI.STATE.WD_TARGET_TYPE = uint8(0);

%--------------------------------------------------------------
%% INITIAL PARAMETER VALUES
%--------------------------------------------------------------
FWD_VEL_PID.phase_margin_deg = 70;
FWD_VEL_PID.gain_margin_dB = 10;
FWD_VEL_PID.k_wd_P = 1.3;
FWD_VEL_PID.lag_phase_deg = -10;
FWD_VEL_PID.f = 0.1;
FWD_VEL_PID.b = 1;
FWD_VEL_PID.damping_factor = 0.7;
FWD_VEL_PID.delta = 5;
FWD_VEL_PID.N = 5;
YAW_RATE_PID.phase_margin_deg = 70;
YAW_RATE_PID.gain_margin_dB = 10;
YAW_RATE_PID.k_wd_P = 1.3;
YAW_RATE_PID.lag_phase_deg = -10;
YAW_RATE_PID.f = 0.1;
YAW_RATE_PID.b = 1;
YAW_RATE_PID.damping_factor = 0.7;
YAW_RATE_PID.delta = 2.5;
YAW_RATE_PID.N = 5;
YAW_ANG_PID.phase_margin_deg = 60;
YAW_ANG_PID.gain_margin_dB = 10;
YAW_ANG_PID.k_wd_P = 1;
YAW_ANG_PID.lag_phase_deg = -10;
YAW_ANG_PID.f = 0.1;
YAW_ANG_PID.b = 1;
WFL_SL_PID.phase_margin_deg = 60;
WFL_SL_PID.k_wd_P = 1.5;
WFL_SL_PID.lag_phase_deg = -10;
WFL_SL_PID.f = 0.1;
WFL_SL_PID.b = 1;
WFL_CD_PID.phase_margin_deg = 60;
WFL_CD_PID.k_wd_P = 1.3;
WFL_CD_PID.lag_phase_deg = -10;
WFL_CD_PID.f = 0.2;
WFL_CD_PID.b = 1;
VEL_SFC.damping_factor = [0.85 0.8];
VEL_SFC.wn_factor = [0.42 1];
VEL_SFC.p_factor = 15;
VEL_SFC.forward_vel_matQ = [12.1356 5.86616 0.870693 1];
VEL_SFC.forward_vel_matR = 2.4527;
VEL_SFC.yaw_rate_matQ = [1e-3 1];
VEL_SFC.yaw_rate_matR = 0.05;
YAW_ANG_SFC.wn_factor = 0.6;
YAW_ANG_SFC.damping_factor = 0.5;
YAW_ANG_SFC.p_factor = 1;
YAW_ANG_SFC.matQ = [5e-4 5e-4 1];
YAW_ANG_SFC.matR = 0.5e-3;
WFL_SFC.natural_freq = 2;
WFL_SFC.damping_factor = 0.99;
WFL_SFC.p_factor = 4;
WFL_SFC.matQ = [5e-4 5e-4 1];
WFL_SFC.matR = 0.5e-3;
PITCH_ANG_SFC.damping_factor = 1.13;
PITCH_ANG_SFC.wn_factor = 0.4;
PITCH_ANG_SFC.p_factor = 50;
PITCH_ANG_SFC.matQ(1) = 28.3257;
PITCH_ANG_SFC.matQ(2) = 1.24197;
PITCH_ANG_SFC.matQ(3) = 1;
PITCH_ANG_SFC.matR = 0.328471;
IMU_CF_FREQ = -log(0.98)/2/pi/SAMPLING_TIME;

%--------------------------------------------------------------
% FILTERS
%--------------------------------------------------------------
cd ../SOFTWARE_COMPONENTS/CONTROL
clear SVF_IN SVF_OUT
% Natural frequency (rad/s)
SVF_IN.freq = 2*pi*CONTROL_INI.PARAM.ENC_FILT_FREQ;
% Damping factor
SVF_IN.damp = 1;
% Sampling time
SVF_IN.ts = MODEL_INI.PARAM.SAMPLING_TIME;
% Filter order
SVF_IN.order = 1;
% Filter design
SVF_OUT = DESIGN_SVF(SVF_IN);
ENC_FLT_SS_MODEL = SVF_OUT.F_ss;
%--------------------------------------------------------------
% IMU FILTER: STATE VARIABLE FILTER     
clear SVF_IN SVF_OUT
% Natural frequency (rad/s)
SVF_IN.freq = 2*pi*CONTROL_INI.PARAM.IMU_FILT_FREQ;
% Damping factor
SVF_IN.damp = 1;
% Sampling time
SVF_IN.ts = MODEL_INI.PARAM.SAMPLING_TIME;
% Filter order
SVF_IN.order = 1;
% Filter design
SVF_OUT = DESIGN_SVF(SVF_IN);
IMU_FLT_SS_MODEL = SVF_OUT.F_ss;
%--------------------------------------------------------------
% RANGE FILTER: STATE VARIABLE FILTER     
clear SVF_IN SVF_OUT
% Natural frequency (rad/s)
SVF_IN.freq = 2*pi*CONTROL_INI.PARAM.WFL_FILT_FREQ;
% Damping factor
SVF_IN.damp = 1;
% Sampling time
SVF_IN.ts = MODEL_INI.PARAM.SAMPLING_TIME;
% Filter order
SVF_IN.order = 1;
% Filter design
SVF_OUT = DESIGN_SVF(SVF_IN);
RNG_FLT_SS_MODEL = SVF_OUT.F_ss;
%--------------------------------------------------------------
% EKF FILTER: STATE VARIABLE FILTER     
clear SVF_IN SVF_OUT
% Natural frequency (rad/s)
SVF_IN.freq = 2*pi*CONTROL_INI.PARAM.EKF_FILT_FREQ;
% Damping factor
SVF_IN.damp = 1;
% Sampling time
SVF_IN.ts = MODEL_INI.PARAM.SAMPLING_TIME;
% Filter order
SVF_IN.order = 1;
% Filter design
SVF_OUT = DESIGN_SVF(SVF_IN);
EKF_FLT_SS_MODEL = SVF_OUT.F_ss;

%--------------------------------------------------------------
%% FIT MODE CONFIGURATION 
%--------------------------------------------------------------
switch(FIT_MODE)
    %-------------------------------------
    case 1 % FORWARD VELOCITY
    %-------------------------------------
        % / 0. PID  / 1. STATE FEEDBACK CONTROL
        CONTROL_TYPE = 0;
        % / 1. P / 2. PI / 3. PD ERR / 4. PD OUT / 5. PID ERR / 6. PID OUT
        PID_CONTROL_TYPE = 6;
        % / 0. FREQUENCY RESPONSE  / 1. TIME RESPONSE
        PID_DESIGN_METHOD = 1;
        % / 0. Phase margin / 1. Gain margin
        STAB_MARGIN = 0;
        % / 1. Pole placement / 2. LQR
        SFC_DESIGN_METHOD = 1;
        % Parameter definition
        if CONTROL_TYPE == 0 % PID
            switch PID_CONTROL_TYPE
                case 1 % P
                    if PID_DESIGN_METHOD % Time response
                        PARAM_LIST = [7];
                    else % Frequency response
                        PARAM_LIST = [];
                    end
                case 2 % PI
                    if PID_DESIGN_METHOD % Time response
                        PARAM_LIST = [7 3 6];
                    else % Frequency response
                        PARAM_LIST = [3 6];
                    end
                    if FWD_VEL_PID.k_wd_P>1
                        disp('Initial value of FWD_VEL_PID.k_wd_P must be < 1')
                        return
                    end
                case {3,4} % PD
                    if PID_DESIGN_METHOD % Time response
                        PARAM_LIST = [7 3 9];
                    else % Frequency response
                        PARAM_LIST = [3 5];
                    end
                    if FWD_VEL_PID.k_wd_P<=1
                        disp('Initial value of FWD_VEL_PID.k_wd_P must be > 1')
                        return
                    end
                case {5,6} % PID
                    if PID_DESIGN_METHOD % Time response
                        PARAM_LIST = [7 3 8 9 6];
                    else % Frequency response
                        PARAM_LIST = [3 4 5 6];
                    end
                    if FWD_VEL_PID.k_wd_P<=1
                        disp('Initial value of FWD_VEL_PID.k_wd_P must be > 1')
                        return
                    end
                otherwise
                    disp('Invalid PID control type')
                    return
            end
            if PID_DESIGN_METHOD == 0
                if STAB_MARGIN==0 % Phase margin
                    PARAM_LIST = [1 PARAM_LIST];
                    FWD_VEL_PID.gain_margin_dB = 0;
                else % Gain margin
                    PARAM_LIST = [2 PARAM_LIST];
                    FWD_VEL_PID.phase_margin_deg = 0;
                end
            end
            if (PID_CONTROL_TYPE==4) || (PID_CONTROL_TYPE==6)
                CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(1);
            else
                CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(0);
            end
        else % STATE FEEDBACK CONTROL
            if SFC_DESIGN_METHOD==1 % Pole placement
                PARAM_LIST = [35 36];
            elseif SFC_DESIGN_METHOD==2% LQR
                PARAM_LIST = [39 41];
            else
                disp('Invalid SFC design method')
                return
            end
        end
        % Fitted outputs in SCOPE_SIM
        ref_fit = [1 1];
        out_fit = [1 FIT_OUTPUT];
        % Command signals in SCOPE_SIM
        cmd_fit = [6 1];
        % Command noise standard deviation threshold (V)
        CMD_NOISE_THR = 0.5;
        % Labels in plot
        OUT_STRING_POS = 1;
        % Closed-loop model reference activation
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 1'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 2'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 3'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 4'],'Commented','through');
        % Control configuration
        CONTROL_MODE = uint8(2);
        CONTROL_INI.STATE.CONTROL_MODE = uint8(CONTROL_MODE);
        CONTROL_INI.STATE.FORWARD_VEL_CONTROL_TYPE = uint8(CONTROL_TYPE);
        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(5);
        % Reference
        CONTROL_INI.STATE.FV_TARGET_TYPE = uint8(2);
    %-------------------------------------
    case 2 % YAW RATE
    %-------------------------------------
        % / 0. PID  / 1. STATE FEEDBACK CONTROL         
        CONTROL_TYPE = 0; 
        % / 1. P / 2. PI / 3. PD err / 4. PD out / 5. PID err / 6. PID out
        PID_CONTROL_TYPE = 6;
        % / 0. FREQUENCY RESPONSE  / 1. TIME RESPONSE       
        PID_DESIGN_METHOD = 0;
        % / 0. Phase margin / 1. Gain margin
        STAB_MARGIN = 0;
        % / 1. Pole placement / 2. LQR
        SFC_DESIGN_METHOD = 1;
        % Parameter definition
        if CONTROL_TYPE == 0 % PID
            switch PID_CONTROL_TYPE
                case 1 % P
                    if PID_DESIGN_METHOD % Time response
                        PARAM_LIST = [16];
                    else % Frequency response
                        PARAM_LIST = [];
                    end
                case 2 % PI
                    if PID_DESIGN_METHOD % Time response
                        PARAM_LIST = [16 12 15];
                    else % Frequency response
                        PARAM_LIST = [12 15];
                    end
                    if YAW_RATE_PID.k_wd_P>1
                        disp('Initial value of YAW_RATE_PID.k_wd_P must be < 1')
                        return
                    end
                case {3,4} % PD
                    if PID_DESIGN_METHOD % Time response
                        PARAM_LIST = [16 12 18];
                    else % Frequency response
                        PARAM_LIST = [12 14];
                    end
                    if YAW_RATE_PID.k_wd_P<=1
                        disp('Initial value of YAW_RATE_PID.k_wd_P must be > 1')
                        return
                    end
                case {5,6} % PID
                    if PID_DESIGN_METHOD % Time response
                        PARAM_LIST = [16 12 17 18 15];
                    else % Frequency response
                        PARAM_LIST = [12 13 14 15];
                    end
                    if YAW_RATE_PID.k_wd_P<=1
                        disp('Initial value of YAW_RATE_PID.k_wd_P must be > 1')
                        return
                    end
                otherwise
                    disp('Invalid PID control type')
                    return
            end
            if PID_DESIGN_METHOD == 0
                if STAB_MARGIN==0 % Phase margin
                    PARAM_LIST = [10 PARAM_LIST];
                    YAW_RATE_PID.gain_margin_dB = 0;
                else % Gain margin
                    PARAM_LIST = [11 PARAM_LIST];
                    YAW_RATE_PID.phase_margin_deg = 0;
                end
            end
            if (PID_CONTROL_TYPE==4) || (PID_CONTROL_TYPE==6)
                CONTROL_INI.PARAM.YAW_RATE_PID.DER_INPUT = uint8(1);
            else
                CONTROL_INI.PARAM.YAW_RATE_PID.DER_INPUT = uint8(0);
            end
        else % STATE FEEDBACK CONTROL
            if SFC_DESIGN_METHOD==1 % Pole placement
                PARAM_LIST = [37 38];
            elseif SFC_DESIGN_METHOD==2 % LQR
                PARAM_LIST = [42 44];
            else
                disp('Invalid SFC design method')
                return
            end
        end
        % Fitted outputs in SCOPE_SIM
        ref_fit = [2 1];
        out_fit = [2 FIT_OUTPUT];
        % Command signals in SCOPE_SIM
        cmd_fit = [6 2];
        % Command noise standard deviation threshold (V)
        CMD_NOISE_THR = 0.5;
        % Labels in plot
        OUT_STRING_POS = 2;
        % Closed-loop model reference activation
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 1'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 2'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 3'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 4'],'Commented','through');
        % Control configuration
        CONTROL_MODE = uint8(2);
        CONTROL_INI.STATE.CONTROL_MODE = uint8(CONTROL_MODE);
        CONTROL_INI.STATE.YAW_RATE_CONTROL_TYPE = uint8(CONTROL_TYPE); 
        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(5);
        % Reference
        CONTROL_INI.STATE.YR_TARGET_TYPE = uint8(2);
    %-------------------------------------
    case 3 % YAW ANGLE
    %-------------------------------------
        % / 0. PID  / 1. STATE FEEDBACK CONTROL
        CONTROL_TYPE = 0;
        % / 1. P / 2. PI / 3. PD err / 4. PD out / 5. PID err / 6. PID out
        PID_CONTROL_TYPE = 6;
        % / 0. Phase margin / 1. Gain margin
        STAB_MARGIN = 0;
        % / 1. Pole placement / 2. LQR
        SFC_DESIGN_METHOD = 1;
        % Parameter definition
        if CONTROL_TYPE == 0 % PID
            switch PID_CONTROL_TYPE
                case 1 % P
                    PARAM_LIST = [];
                case 2 % PI
                    PARAM_LIST = [21 24];
                    if YAW_ANG_PID.k_wd_P>1
                        disp('Initial value of YAW_RATE_PID.k_wd_P must be < 1')
                        return
                    end
                case {3,4} % PD
                    PARAM_LIST = [21 23];
                    if YAW_ANG_PID.k_wd_P<=1
                        disp('Initial value of YAW_RATE_PID.k_wd_P must be > 1')
                        return
                    end
                case {5,6} % PID
                    PARAM_LIST = [21 22 23 24];
                    if YAW_RATE_PID.k_wd_P<=1
                        disp('Initial value of YAW_RATE_PID.k_wd_P must be > 1')
                        return
                    end
                otherwise
                    disp('Invalid PID control type')
                    return
            end
            if STAB_MARGIN==0 % Phase margin
                PARAM_LIST = [19 PARAM_LIST];
                YAW_ANG_PID.gain_margin_dB = 0;
            else % Gain margin
                PARAM_LIST = [20 PARAM_LIST];
                YAW_ANG_PID.phase_margin_deg = 0;
            end
            if (PID_CONTROL_TYPE==4) || (PID_CONTROL_TYPE==6)
                CONTROL_INI.PARAM.YAW_ANG_PID.DER_INPUT = uint8(1);
            else
                CONTROL_INI.PARAM.YAW_ANG_PID.DER_INPUT = uint8(0);
            end
        else % STATE FEEDBACK CONTROL
            if SFC_DESIGN_METHOD==1 % Pole placement
                PARAM_LIST = [45 46 47];
            elseif SFC_DESIGN_METHOD==2 % LQR
                PARAM_LIST = [48 49 51];
            else
                disp('Invalid SFC design method')
                return
            end
        end
        % Fitted outputs in SCOPE_SIM
        ref_fit = [3 1];
        out_fit = [3 FIT_OUTPUT];
        % Command signals in SCOPE_SIM
        cmd_fit = [6 2];
        % Command noise standard deviation threshold (V)
        CMD_NOISE_THR = 0.5;
        % Labels in plot
        OUT_STRING_POS = 3;
        % Closed-loop model reference activation
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 1'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 2'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 3'],'Commented','off');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 4'],'Commented','through');
        % Control configuration
        CONTROL_MODE = uint8(1);
        CONTROL_INI.STATE.CONTROL_MODE = uint8(CONTROL_MODE);
        CONTROL_INI.STATE.YAW_ANG_CONTROL_TYPE = uint8(CONTROL_TYPE);
        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(4);
        % Reference
        CONTROL_INI.STATE.YA_TARGET_TYPE = uint8(2);
    %-------------------------------------
    case 4 % WALL DISTANCE
    %-------------------------------------
        % / 0. SINGLE LOOP  / 1. CASCADE / 2. STATE FEEDBACK CONTROL         
        CONTROL_TYPE = 2;
        % / 1. P / 2. PI / 3. PD err / 4. PD out / 5. PID err / 6. PID out
        PID_CONTROL_TYPE = 4;
        % / 1. Pole placement / 2. LQR
        SFC_DESIGN_METHOD = 2;
        % Parameter definition
        if CONTROL_TYPE < 2 % PID
            switch PID_CONTROL_TYPE
                case 1 % P
                    if CONTROL_TYPE == 0 % SINGLE LOOP
                        PARAM_LIST = [25];
                    else % CASCADE
                        PARAM_LIST = [30];
                    end
                case 2 % PI
                    if CONTROL_TYPE == 0 % SINGLE LOOP
                        PARAM_LIST = [25 26 29];
                        if WFL_SL_PID.k_wd_P>1
                            disp('Initial value of WFL_SL_PID.k_wd_P must be < 1')
                            return
                        end
                    else % CASCADE
                        PARAM_LIST = [30 31 34];
                        if WFL_CD_PID.k_wd_P>1
                            disp('Initial value of WFL_CD_PID.k_wd_P must be < 1')
                            return
                        end
                    end
                case {3,4} % PD
                    if CONTROL_TYPE == 0 % SINGLE LOOP
                        PARAM_LIST = [25 26 28];
                    else % CASCADE
                        PARAM_LIST = [30 31 33];
                        if WFL_CD_PID.k_wd_P<=1
                            disp('Initial value of WFL_CD_PID.k_wd_P must be > 1')
                            return
                        end
                    end
                case {5,6} % PID
                    if CONTROL_TYPE == 0 % SINGLE LOOP
                        PARAM_LIST = [25 26 27 28 29];
                    else % CASCADE
                        PARAM_LIST = [30 31 32 33 34];
                        if WFL_CD_PID.k_wd_P<=1
                            disp('Initial value of WFL_CD_PID.k_wd_P must be > 1')
                            return
                        end
                    end
                otherwise
                    disp('Invalid PID control type')
                    return
            end
            if (PID_CONTROL_TYPE==4) || (PID_CONTROL_TYPE==6)
                CONTROL_INI.PARAM.WFL_SL_PID.DER_INPUT = uint8(1);
                CONTROL_INI.PARAM.WFL_CD_PID.DER_INPUT = uint8(1);
            else
                CONTROL_INI.PARAM.WFL_SL_PID.DER_INPUT = uint8(0);
                CONTROL_INI.PARAM.WFL_CD_PID.DER_INPUT = uint8(0);
            end
        else % STATE FEEDBACK CONTROL
            if SFC_DESIGN_METHOD==1 % Pole placement
                PARAM_LIST = [52 53 54];
            elseif SFC_DESIGN_METHOD==2 % LQR
                PARAM_LIST = [55 56 58];
            else
                disp('Invalid SFC design method')
                return
            end
        end
        % Fitted outputs in SCOPE_SIM
        ref_fit = [4 1];
        out_fit = [4 FIT_OUTPUT];
        if CONTROL_TYPE == 1 % CASCADE
            % Command signals in SCOPE_SIM
            cmd_fit = [3 1];
            % Command noise standard deviation threshold (V)
            CMD_NOISE_THR = 0.5;
        else
            % Command signals in SCOPE_SIM
            cmd_fit = [6 2];
            % Command noise standard deviation threshold (V)
            CMD_NOISE_THR = 0.5;
        end
        % Labels in plot
        OUT_STRING_POS = 4;
        % Closed-loop model reference activation
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 1'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 2'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 3'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 4'],'Commented','off');
        % Control configuration
        CONTROL_MODE = uint8(4);
        CONTROL_INI.STATE.CONTROL_MODE = uint8(CONTROL_MODE);
        MODEL_INI.PARAM.CONTROL_MODE = uint8(CONTROL_MODE);
        CONTROL_INI.STATE.WFL_CONTROL_TYPE = uint8(CONTROL_TYPE);
        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(7);
        % Reference
        CONTROL_INI.STATE.WD_TARGET_TYPE = uint8(2);
    %-------------------------------------
    case 5 % PITCH ANGLE
    %-------------------------------------
        % / 0. PID  / 1. STATE FEEDBACK REGULATOR / 2. STATE FEEDBACK INTEGRAL CONTROL
        CONTROL_TYPE = 2;
        % / 1. Pole placement / 2. LQR
        SFC_DESIGN_METHOD = 1;
        % / 0. COMPLEMENTARY FILTER / 1. EKF
        OBSERVER_MODE = 0;
        % Parameter definition
        if SFC_DESIGN_METHOD==1 % Pole placement
            if CONTROL_TYPE == 1
                PARAM_LIST = [61 62 63];
            elseif CONTROL_TYPE == 2
                PARAM_LIST = [35 36 37];
            else
                disp('Invalid pitch angle control type')
                return
            end
        elseif SFC_DESIGN_METHOD==2 % LQR
            if CONTROL_TYPE == 1
                PARAM_LIST = [64 65 66];
            elseif CONTROL_TYPE == 2
                PARAM_LIST = [40 41 42 43];
            else
                disp('Invalid pitch angle control type')
                return
            end
        else
            disp('Invalid SFC design method')
            return
        end
        % if OBSERVER_MODE==0
        %    PARAM_LIST = [PARAM_LIST 66];
        % end
        % Fitted outputs in SCOPE_SIM
        % ref_fit = [12 1];
        % out_fit = [12 FIT_OUTPUT];
        ref_fit = [1 1];
        out_fit = [1 FIT_OUTPUT];
        % Command signals in SCOPE_SIM
        cmd_fit = [6 1];
        % Command noise standard deviation threshold (V)
        CMD_NOISE_THR = 0.5;
        % Labels in plot
        OUT_STRING_POS = 5;
        % Closed-loop model reference activation
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 1'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 2'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 3'],'Commented','through');
        set_param([MODEL_SLX '/MONITORING/SIMULATION: SCOPES/Second-order LPF 4'],'Commented','through');
        % Control configuration
        CONTROL_MODE = uint8(8);
        CONTROL_INI.STATE.VEHICLE_MODE = uint8(1);
        CONTROL_INI.STATE.CONTROL_MODE = uint8(CONTROL_MODE);
        CONTROL_INI.STATE.PITCH_ANG_CONTROL_TYPE = uint8(CONTROL_TYPE);
        CONTROL_INI.STATE.CURRENT_STATUS_SYS = uint8(12);

    %-------------------------------------
    otherwise
    %-------------------------------------
        disp('FIT MODE IS NOT VALID')
        return
end


%--------------------------------------------------------------
%% INITIALIZATION FOR FORWARD VELOCITY CONTROL
%--------------------------------------------------------------
if FIT_MODE==1
    if CONTROL_TYPE==0 % PID
        % Struct PID_IN: Mandatory fields in the input
        clear PID_IN PID_OUT
        PID_IN.control_type = PID_CONTROL_TYPE;
        PID_IN.ts = SAMPLING_TIME;
        PID_IN.design_method = PID_DESIGN_METHOD;
        PID_IN.decoupling_type = 0;
        PID_IN.search_wd_P = true;
        PID_IN.wd_ref = 1;
        PID_IN.D_lower = true;
        PID_IN.int_disc_type = 3;
        PID_IN.der_disc_type = 3;
        PID_IN.antiwindup = true;
        %------------------------------------
        FORWARD_VEL_SS_MODEL = ...
            LIN_MODEL(CONTROL_INI.STATE.FORWARD_VEL_MAIN_OP).FORWARD_VEL_SS_MODEL;
        % Additional filter
        if CONTROL_INI.STATE.OBSERVER_MODE
            % EKF
            FORWARD_VEL_SS_MODEL = series(FORWARD_VEL_SS_MODEL,EKF_FLT_SS_MODEL(1,1));
        else
            % ENCODER FILTER
            FORWARD_VEL_SS_MODEL = series(FORWARD_VEL_SS_MODEL,ENC_FLT_SS_MODEL(1,1));
        end
        FORWARD_VEL_TF_MODEL = zpk(minreal(FORWARD_VEL_SS_MODEL));
        FORWARD_VEL_TF_MODEL.DisplayFormat = 'TimeConstant';
        if CONTROL_INI.STATE.SP_MODE(1)==0 && CONTROL_INI.STATE.MOTOR_DELAY_MODE>0
            DELAY = CONTROL_INI.PARAM.MOTOR_DELAY + 0.5*PID_IN.ts;
            FORWARD_VEL_TF_MODEL = FORWARD_VEL_TF_MODEL*exp(-DELAY*s);
        else
            DELAY = 0.5*PID_IN.ts*(CONTROL_INI.PARAM.PID_DESIGN_METHOD==0);
            FORWARD_VEL_TF_MODEL = FORWARD_VEL_TF_MODEL*exp(-DELAY*s);
        end
        %--------------------------------------------------------------
        % FORWARD VELOCITY PID: DESIGN
        % Plant model
        PID_IN.P = minreal(FORWARD_VEL_TF_MODEL);
        PID_IN.P.InputGroup.MV = 1;
        PID_IN.P.OutputGroup.MO = 1;
    else % STATE-FEEDBACK CONTROL
        clear SFC_IN SFC_OUT
        % Control type
        % 1. Regulator / 2. Setpoint gain / 3. Integral
        SFC_IN.control_type = 3;
        SFC_IN.ts = SAMPLING_TIME;
        SFC_IN.nd = 0;
        SFC_IN.int_disc_type = 3;
        SFC_IN.antiwindup = true;
        %--------------------------------------------------------------
        % MV = {'MOTOR VOLT COM'}
        % STATE = {'FORWARD VEL'}
        % Design method
        SFC_IN.design_method = SFC_DESIGN_METHOD;
        % Plant model
        SFC_IN.P = LIN_MODEL(1).FORWARD_VEL_SS_MODEL(1,1);
    end
end

%--------------------------------------------------------------
%% INITIALIZATION FOR YAW RATE CONTROL
%--------------------------------------------------------------
if FIT_MODE==2
    if CONTROL_TYPE==0 % PID
        % Struct PID_IN: Mandatory fields in the input
        clear PID_IN PID_OUT
        PID_IN.control_type = PID_CONTROL_TYPE;
        PID_IN.ts = SAMPLING_TIME;
        PID_IN.design_method = PID_DESIGN_METHOD;
        PID_IN.decoupling_type = 0;
        PID_IN.search_wd_P = true;
        PID_IN.wd_ref = 1;
        PID_IN.D_lower = true;
        PID_IN.int_disc_type = 3;
        PID_IN.der_disc_type = 3;
        PID_IN.antiwindup = true;
        %------------------------------------
        YAW_RATE_SS_MODEL = LIN_MODEL(1).YAW_RATE_SS_MODEL;
        % Additional filter
        if CONTROL_INI.STATE.ROTATION_MSRT_MODE
            % ENCODER
            ADD_FLT_SS_MODEL = ENC_FLT_SS_MODEL;
        else
            % IMU
            ADD_FLT_SS_MODEL = IMU_FLT_SS_MODEL;
        end
        if CONTROL_INI.STATE.OBSERVER_MODE
            % EKF
            YAW_RATE_SS_MODEL = series(YAW_RATE_SS_MODEL,EKF_FLT_SS_MODEL(1,1));
        else
            % FILTER
            YAW_RATE_SS_MODEL = series(YAW_RATE_SS_MODEL,ADD_FLT_SS_MODEL(1,1));
        end
        YAW_RATE_TF_MODEL = zpk(minreal(YAW_RATE_SS_MODEL));
        YAW_RATE_TF_MODEL.DisplayFormat = 'TimeConstant';
        if CONTROL_INI.STATE.SP_MODE(2)==0 && CONTROL_INI.STATE.MOTOR_DELAY_MODE>0
            DELAY = CONTROL_INI.PARAM.MOTOR_DELAY + 0.5*ts;
            YAW_RATE_TF_MODEL = YAW_RATE_TF_MODEL*exp(-DELAY*s);
        else
            DELAY = 0.5*ts*(CONTROL_INI.PARAM.PID_DESIGN_METHOD==0);
            YAW_RATE_TF_MODEL = YAW_RATE_TF_MODEL*exp(-DELAY*s);
        end
        %--------------------------------------------------------------
        % YAW RATE PID: DESIGN
        % Plant model
        PID_IN.P = YAW_RATE_TF_MODEL;
        PID_IN.P.InputGroup.MV = 1;
        PID_IN.P.OutputGroup.MO = 1;
    else % STATE-FEEDBACK CONTROL
        clear SFC_IN SFC_OUT
        % Control type
        % 1. Regulator / 2. Setpoint gain / 3. Integral
        SFC_IN.control_type = 3;
        SFC_IN.ts = SAMPLING_TIME;
        SFC_IN.nd = 0;
        SFC_IN.int_disc_type = 3;
        SFC_IN.antiwindup = true;
        %--------------------------------------------------------------
        % MV = {'MOTOR VOLT DIF'}
        % STATE = {'YAW RATE'}
        % Design method
        SFC_IN.design_method = SFC_DESIGN_METHOD;
        % Plant model
        SFC_IN.P = LIN_MODEL(1).YAW_RATE_SS_MODEL(1,1);
    end
end

%--------------------------------------------------------------
%% INITIALIZATION FOR YAW ANGLE CONTROL
%--------------------------------------------------------------
if FIT_MODE==3
    if CONTROL_TYPE==0 % PID
        % Struct PID_IN: Mandatory fields in the input
        clear PID_IN PID_OUT
        PID_IN.control_type = PID_CONTROL_TYPE;
        PID_IN.ts = SAMPLING_TIME;
        PID_IN.design_method = 0;
        PID_IN.decoupling_type = 0;
        PID_IN.search_wd_P = true;
        PID_IN.wd_ref = 1;
        PID_IN.D_lower = true;
        PID_IN.int_disc_type = 3;
        PID_IN.der_disc_type = 3;
        PID_IN.antiwindup = true;
        %------------------------------------
        YAW_ANG_SS_MODEL = LIN_MODEL(1).YAW_ANG_SS_MODEL;
        % Additional filter
        if CONTROL_INI.STATE.ROTATION_MSRT_MODE
            % ENCODER
            ADD_FLT_SS_MODEL = ENC_FLT_SS_MODEL;
        else
            % IMU
            ADD_FLT_SS_MODEL = IMU_FLT_SS_MODEL;
        end
        if CONTROL_INI.STATE.OBSERVER_MODE
            % EKF
            YAW_ANG_SS_MODEL = series(YAW_ANG_SS_MODEL,EKF_FLT_SS_MODEL(1,1));
        else
            % FILTER
            YAW_ANG_SS_MODEL = series(YAW_ANG_SS_MODEL,ADD_FLT_SS_MODEL(1,1));
        end
        YAW_ANG_TF_MODEL = zpk(minreal(YAW_ANG_SS_MODEL));
        YAW_ANG_TF_MODEL.DisplayFormat = 'TimeConstant';
        if CONTROL_INI.STATE.MOTOR_DELAY_MODE>0
            DELAY = CONTROL_INI.PARAM.MOTOR_DELAY;
            YAW_ANG_TF_MODEL = YAW_ANG_TF_MODEL*exp(-DELAY*s);
        end
        %--------------------------------------------------------------
        PID_IN.P = YAW_ANG_TF_MODEL;
        PID_IN.P.InputGroup.MV = 1;
        PID_IN.P.OutputGroup.MO = 1;
    else % STATE-FEEDBACK CONTROL
        clear SFC_IN SFC_OUT
        % Control type
        % 1. Regulator / 2. Setpoint gain / 3. Integral
        SFC_IN.control_type = 3;
        SFC_IN.ts = SAMPLING_TIME;
        SFC_IN.nd = 0;
        SFC_IN.int_disc_type = 3;
        SFC_IN.antiwindup = true;
        %--------------------------------------------------------------
        % MV = {'MOTOR VOLT DIF'}
        % STATE = {'YAW RATE','YAW ANG'}
        % Design method
        SFC_IN.design_method = SFC_DESIGN_METHOD;
        % Plant model
        SFC_IN.P = LIN_MODEL(1).YAW_ANG_SS_MODEL;
    end
end

%--------------------------------------------------------------
%% INITIALIZATION FOR WALL DISTANCE CONTROL
%--------------------------------------------------------------
if FIT_MODE==4
    if CONTROL_TYPE==0 % SINGLE-LOOP PID
        % Struct PID_IN: Mandatory fields in the input
        clear PID_IN PID_OUT
        PID_IN.ts = SAMPLING_TIME;
        PID_IN.control_type = PID_CONTROL_TYPE;
        PID_IN.design_method = 0;
        PID_IN.decoupling_type = 0;
        PID_IN.gain_margin_dB = 0;
        if PID_CONTROL_TYPE>2
            PID_IN.search_wd_P = false;
        else
            PID_IN.search_wd_P = true;
        end
        PID_IN.wd_ref = 1;
        PID_IN.D_lower = true;
        PID_IN.int_disc_type = 3;
        PID_IN.der_disc_type = 3;
        PID_IN.antiwindup = true;
        %------------------------------------
        % Plant model
        WFL_SL_SS_MODEL = LIN_MODEL(NUM_OP).WFL_SL_SS_MODEL;
        PID_IN.P = WFL_SL_SS_MODEL(3,1);
        WFL_SL_TF_MODEL = zpk(minreal(WFL_SL_SS_MODEL(3,1)));
        WFL_SL_TF_MODEL.DisplayFormat = 'TimeConstant';
        LIN_MODEL(NUM_OP).WFL_SL_TF_MODEL = WFL_SL_TF_MODEL;
        PID_IN.P.InputGroup.MV = 1;
        PID_IN.P.OutputGroup.MO = 1;
    elseif CONTROL_TYPE==1 % CASCADE PID
        % Struct PID_IN: Mandatory fields in the input
        clear PID_IN PID_OUT
        % Control parameters
        PID_IN.ts = SAMPLING_TIME;
        PID_IN.control_type = PID_CONTROL_TYPE;
        PID_IN.design_method = 0;
        PID_IN.decoupling_type = 0;
        PID_IN.gain_margin_dB = 0;
        PID_IN.search_wd_P = true;
        PID_IN.wd_ref = 5;
        PID_IN.D_lower = true;
        PID_IN.int_disc_type = 3;
        PID_IN.der_disc_type = 3;
        PID_IN.antiwindup = true;
        %--------------------------------------------------------------
        % WALL FOLLOWER CD PID: DESIGN FOR SELECTED OPERATING POINT
        % Plant model
        PID_IN.P = LIN_MODEL(NUM_OP).WFL_CD_SS_MODEL;
    else % STATE-FEEDBACK CONTROL
        clear SFC_IN SFC_OUT
        SFC_IN.control_type = 1;
        SFC_IN.design_method = SFC_DESIGN_METHOD;
        SFC_IN.ts = SAMPLING_TIME;
        SFC_IN.nd = 0;
        SFC_IN.int_disc_type = 0;
        SFC_IN.antiwindup = false;
        %--------------------------------------------------------------
        % Plant model
        SFC_IN.P = LIN_MODEL(NUM_OP).WFL_SL_SS_MODEL(3,1);
    end
end

%--------------------------------------------------------------
%% INITIALIZATION FOR PITCH ANGLE CONTROL
%--------------------------------------------------------------
if FIT_MODE==5
    % STATE-FEEDBACK CONTROL
    clear SFC_IN SFC_OUT
    SFC_IN.design_method = SFC_DESIGN_METHOD;
    SFC_IN.ts = SAMPLING_TIME;
    SFC_IN.nd = 0;
    if CONTROL_TYPE==1
        SFC_IN.control_type = 1;        
        SFC_IN.int_disc_type = 0;
        SFC_IN.antiwindup = false;
        % Plant model
        SFC_IN.P = LIN_MODEL(1).PITCH_ANG_SS_MODEL(3,1);
    else
        SFC_IN.control_type = 3;
        SFC_IN.int_disc_type = 1;
        SFC_IN.antiwindup = true;
        % Plant model
        SFC_IN.P = LIN_MODEL(1).PITCH_ANG_SS_MODEL(1,1);
    end
end

cd ../../SIMULINK

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
% SCALING FACTORS FOR FITTED OUTPUTS
factor = ones(2,1);

%--------------------------------------------------------------
%% GAUSS-NEWTON ALGORITHM
%--------------------------------------------------------------
% ALGORITHM PARAMETERS
ts=SAMPLING_TIME;                           % Sampling time
tfin = MODEL_INI.PARAM.SIM_FINAL_TIME;      % Final time for simulation
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Parameter update
	th=theta;
    nn = 1;
    for ii = 1:NUM_PARAM
        if min(abs(PARAM_LIST-ii))==0
            command = [PARAM_STRING{ii} ' = th(nn)*th_ini(nn); nn=nn+1;'];
            eval(command)
        end
    end
    % Simulation    
    warning('off','all')
    try
        cd ../SOFTWARE_COMPONENTS/CONTROL
        % FORWARD VELOCITY CONTROL
        if FIT_MODE == 1
            if CONTROL_TYPE == 0 % PID
                % Specifications
                if PID_DESIGN_METHOD == 0 % Frequency response
                    PID_IN.gain_margin_dB = FWD_VEL_PID.gain_margin_dB;
                    PID_IN.phase_margin_deg = FWD_VEL_PID.phase_margin_deg;
                    PID_IN.k_wd_P = FWD_VEL_PID.k_wd_P;
                    PID_IN.b = FWD_VEL_PID.b;
                    PID_IN.lag_phase_deg = FWD_VEL_PID.lag_phase_deg;
                    PID_IN.f = FWD_VEL_PID.f;
                else % Time response
                    PID_IN.damping_factor = FWD_VEL_PID.damping_factor;
                    PID_IN.k_wd_P = FWD_VEL_PID.k_wd_P;
                    PID_IN.delta = FWD_VEL_PID.delta;
                    PID_IN.N = FWD_VEL_PID.N;
                    PID_IN.b = FWD_VEL_PID.b;
                end
                % CONTROL DESIGN
                PID_OUT = DESIGN_PID(PID_IN);
                % PID CONTROL PARAMETERS
                CONTROL_INI.PARAM.FORWARD_VEL_PID.K(NUM_OP,:) = PID_OUT.K;
                CONTROL_INI.PARAM.FORWARD_VEL_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                CONTROL_INI.PARAM.FORWARD_VEL_PID.Td(NUM_OP,:) = PID_OUT.Td;
                CONTROL_INI.PARAM.FORWARD_VEL_PID.N(NUM_OP,:) = PID_OUT.N;
                CONTROL_INI.PARAM.FORWARD_VEL_PID.b(NUM_OP,:) = PID_OUT.b;
                CONTROL_INI.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(PID_OUT.der_input);
                CONTROL_INI.PARAM.FORWARD_VEL_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);                
            else % STATE FEEDBACK CONTROL
                % Specifications
                if SFC_DESIGN_METHOD==1 % Pole placement
                    % Open-loop pole frequency
                    w_ol = abs(eig(LIN_MODEL(1).FORWARD_VEL_SS_MODEL.a));
                    % Natural frequency and damping factor of the dominant closed-loop poles
                    wn = VEL_SFC.wn_factor(1)*w_ol;
                    seta = VEL_SFC.damping_factor(1);
                    % Closed loop poles in continuous time
                    SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j ].';
                else % LQR
                    SFC_IN.matQ = VEL_SFC.forward_vel_matQ;
                    SFC_IN.matR = VEL_SFC.forward_vel_matR;
                end
                % CONTROL DESIGN
                SFC_OUT = DESIGN_SFC(SFC_IN);
                % SFC CONTROL PARAMETERS
                CONTROL_INI.PARAM.FORWARD_VEL_SFC.K = SFC_OUT.K;
                CONTROL_INI.PARAM.FORWARD_VEL_SFC.Ki = SFC_OUT.Ki;
                CONTROL_INI.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                CONTROL_INI.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
            end
        end
        % YAW RATE CONTROL
        if FIT_MODE == 2
            if CONTROL_TYPE==0 % PID
                % Specifications
                if PID_DESIGN_METHOD == 0 % Frequency response
                    PID_IN.gain_margin_dB = YAW_RATE_PID.gain_margin_dB;
                    PID_IN.phase_margin_deg = YAW_RATE_PID.phase_margin_deg;
                    PID_IN.k_wd_P = YAW_RATE_PID.k_wd_P;
                    PID_IN.b = YAW_RATE_PID.b;
                    PID_IN.lag_phase_deg = YAW_RATE_PID.lag_phase_deg;
                    PID_IN.f = YAW_RATE_PID.f;
                else % Time response
                    PID_IN.damping_factor = YAW_RATE_PID.damping_factor;
                    PID_IN.k_wd_P = YAW_RATE_PID.k_wd_P;
                    PID_IN.delta = YAW_RATE_PID.delta;
                    PID_IN.N = YAW_RATE_PID.N;
                    PID_IN.b = YAW_RATE_PID.b;
                end
                % CONTROL DESIGN
                PID_OUT = DESIGN_PID(PID_IN);
                % PID CONTROL PARAMETERS
                CONTROL_INI.PARAM.YAW_RATE_PID.K(NUM_OP,:) = PID_OUT.K;
                CONTROL_INI.PARAM.YAW_RATE_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                CONTROL_INI.PARAM.YAW_RATE_PID.Td(NUM_OP,:) = PID_OUT.Td;
                CONTROL_INI.PARAM.YAW_RATE_PID.N(NUM_OP,:) = PID_OUT.N;
                CONTROL_INI.PARAM.YAW_RATE_PID.b(NUM_OP,:) = PID_OUT.b;
                CONTROL_INI.PARAM.YAW_RATE_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                CONTROL_INI.PARAM.YAW_RATE_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                CONTROL_INI.PARAM.YAW_RATE_PID.DER_INPUT = uint8(PID_OUT.der_input);
                CONTROL_INI.PARAM.YAW_RATE_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
            else % STATE FEEDBACK CONTROL
                % Specifications
                if SFC_DESIGN_METHOD == 1 % Pole placement
                    % Open-loop pole frequency
                    w_ol = abs(eig(LIN_MODEL(1).YAW_RATE_SS_MODEL.a));
                    % Natural frequency and damping factor of the dominant closed-loop poles
                    wn = VEL_SFC.wn_factor(2)*w_ol;
                    seta = VEL_SFC.damping_factor(2);
                    % Closed loop poles in continuous time
                    SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j ].';
                else % LQR
                    SFC_IN.matQ = VEL_SFC.yaw_rate_matQ;
                    SFC_IN.matR = VEL_SFC.yaw_rate_matR;
                end
                % CONTROL DESIGN
                SFC_OUT = DESIGN_SFC(SFC_IN);
                % SFC CONTROL PARAMETERS
                CONTROL_INI.PARAM.YAW_RATE_SFC.K = SFC_OUT.K;
                CONTROL_INI.PARAM.YAW_RATE_SFC.Ki = SFC_OUT.Ki;
                CONTROL_INI.PARAM.YAW_RATE_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                CONTROL_INI.PARAM.YAW_RATE_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
            end
        end
        % YAW ANGLE CONTROL
        if FIT_MODE == 3
            if CONTROL_TYPE==0 % PID
                % Specifications
                PID_IN.gain_margin_dB = YAW_ANG_PID.gain_margin_dB;
                PID_IN.phase_margin_deg = YAW_ANG_PID.phase_margin_deg;
                PID_IN.k_wd_P = YAW_ANG_PID.k_wd_P;
                PID_IN.b = YAW_ANG_PID.b;
                PID_IN.lag_phase_deg = YAW_ANG_PID.lag_phase_deg;
                PID_IN.f = YAW_ANG_PID.f;
                % CONTROL DESIGN
                PID_OUT = DESIGN_PID(PID_IN);
                % PID CONTROL PARAMETERS
                CONTROL_INI.PARAM.YAW_ANG_PID.K(NUM_OP,:) = PID_OUT.K;
                CONTROL_INI.PARAM.YAW_ANG_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                CONTROL_INI.PARAM.YAW_ANG_PID.Td(NUM_OP,:) = PID_OUT.Td;
                CONTROL_INI.PARAM.YAW_ANG_PID.N(NUM_OP,:) = PID_OUT.N;
                CONTROL_INI.PARAM.YAW_ANG_PID.b(NUM_OP,:) = PID_OUT.b;
                CONTROL_INI.PARAM.YAW_ANG_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                CONTROL_INI.PARAM.YAW_ANG_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                CONTROL_INI.PARAM.YAW_ANG_PID.DER_INPUT = uint8(PID_OUT.der_input);
                CONTROL_INI.PARAM.YAW_ANG_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
            else % STATE FEEDBACK CONTROL
                % Specifications
                if SFC_DESIGN_METHOD == 1 % Pole placement
                    % Open-loop maximun pole frequency
                    w_ol=max(abs(eig(LIN_MODEL(1).YAW_ANG_SS_MODEL.a)));
                    % Natural frequency and damping factor of the dominant closed-loop poles
                    wn = YAW_ANG_SFC.wn_factor*w_ol;
                    seta = YAW_ANG_SFC.damping_factor;
                    % Closed loop poles in continuous time
                    SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -YAW_ANG_SFC.p_factor].';
                else % LQR
                    SFC_IN.matQ = YAW_ANG_SFC.matQ;
                    SFC_IN.matR = YAW_ANG_SFC.matR;
                end
                % CONTROL DESIGN
                SFC_OUT = DESIGN_SFC(SFC_IN);
                % SFC CONTROL PARAMETERS
                CONTROL_INI.PARAM.YAW_ANG_SFC.K = SFC_OUT.K;
                CONTROL_INI.PARAM.YAW_ANG_SFC.Ki = SFC_OUT.Ki;
                CONTROL_INI.PARAM.YAW_ANG_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                CONTROL_INI.PARAM.YAW_ANG_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
            end
        end
        % WALL DISTANCE CONTROL
        if FIT_MODE == 4
            if CONTROL_TYPE==0 % SINGLE-LOOP PID
                % Specifications
                PID_IN.phase_margin_deg = WFL_SL_PID.phase_margin_deg;
                PID_IN.k_wd_P = WFL_SL_PID.k_wd_P;
                PID_IN.b = WFL_SL_PID.b;
                PID_IN.lag_phase_deg = WFL_SL_PID.lag_phase_deg;
                PID_IN.f = WFL_SL_PID.f;
                % CONTROL DESIGN
                PID_OUT = DESIGN_PID(PID_IN);
                % PID CONTROL PARAMETERS
                PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
                CONTROL_INI.PARAM.WFL_SL_PID.K(NUM_OP,:) = PID_OUT.K;
                CONTROL_INI.PARAM.WFL_SL_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                CONTROL_INI.PARAM.WFL_SL_PID.Td(NUM_OP,:) = PID_OUT.Td;
                CONTROL_INI.PARAM.WFL_SL_PID.N(NUM_OP,:) = PID_OUT.N;
                CONTROL_INI.PARAM.WFL_SL_PID.b(NUM_OP,:) = PID_OUT.b;
                CONTROL_INI.PARAM.WFL_SL_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                CONTROL_INI.PARAM.WFL_SL_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                CONTROL_INI.PARAM.WFL_SL_PID.DER_INPUT = uint8(PID_OUT.der_input);
                CONTROL_INI.PARAM.WFL_SL_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
            elseif CONTROL_TYPE==1 % CASCADE PID
                % Specifications
                PID_IN.phase_margin_deg = WFL_CD_PID.phase_margin_deg;
                PID_IN.k_wd_P = WFL_CD_PID.k_wd_P;
                PID_IN.b = WFL_CD_PID.b;
                PID_IN.lag_phase_deg = WFL_CD_PID.lag_phase_deg;
                PID_IN.f = WFL_CD_PID.f;
                % CONTROL DESIGN
                PID_OUT = DESIGN_PID(PID_IN);
                % PID CONTROL PARAMETERS
                PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
                CONTROL_INI.PARAM.WFL_CD_PID.K(NUM_OP,:) = PID_OUT.K;
                CONTROL_INI.PARAM.WFL_CD_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                CONTROL_INI.PARAM.WFL_CD_PID.Td(NUM_OP,:) = PID_OUT.Td;
                CONTROL_INI.PARAM.WFL_CD_PID.N(NUM_OP,:) = PID_OUT.N;
                CONTROL_INI.PARAM.WFL_CD_PID.b(NUM_OP,:) = PID_OUT.b;
                CONTROL_INI.PARAM.WFL_CD_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                CONTROL_INI.PARAM.WFL_CD_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                CONTROL_INI.PARAM.WFL_CD_PID.DER_INPUT = uint8(PID_OUT.der_input);
                CONTROL_INI.PARAM.WFL_CD_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
            else % STATE FEEDBACK CONTROL
                % Specifications
                if SFC_DESIGN_METHOD == 1 % Pole placement
                    % Natural frequency and damping factor of the dominant closed-loop poles
                    wn = WFL_SFC.natural_freq;
                    seta = WFL_SFC.damping_factor;
                    % Closed loop poles in continuous time
                    SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j  -WFL_SFC.p_factor].';
                else  % LQR
                    SFC_IN.matQ = WFL_SFC.matQ;
                    SFC_IN.matR = WFL_SFC.matR;
                end
                % CONTROL DESIGN
                SFC_OUT = DESIGN_SFC(SFC_IN);
                % SFC CONTROL PARAMETERS
                CONTROL_INI.PARAM.WFL_SFR.K(NUM_OP,:) = SFC_OUT.K;
            end
        end
        % PITCH ANGLE CONTROL
        if FIT_MODE == 5
            % STATE FEEDBACK CONTROL
            if CONTROL_TYPE == 1
                % Specifications
                if SFC_DESIGN_METHOD == 1 % Pole placement
                    % Open-loop maximun pole frequency
                    w_ol=max(eig(LIN_MODEL(1).PITCH_ANG_SS_MODEL.a));
                    % Natural frequency and damping factor of the dominant closed-loop poles
                    wn = PITCH_ANG_SFC.wn_factor*w_ol;
                    seta = PITCH_ANG_SFC.damping_factor;
                    % Closed loop poles in continuous time
                    SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -PITCH_ANG_SFC.p_factor].';
                else % LQR
                    SFC_IN.matQ = PITCH_ANG_SFC.matQ;
                    SFC_IN.matR = PITCH_ANG_SFC.matR;
                end
                % CONTROL DESIGN
                SFC_OUT = DESIGN_SFC(SFC_IN);
                % SFC CONTROL PARAMETERS
                CONTROL_INI.PARAM.PITCH_ANG_SFC.K = SFC_OUT.K;
            else
                if SFC_DESIGN_METHOD == 1 % Pole placement
                    % Unstable pole frequency
                    w_ol = max(eig(LIN_MODEL(1).PITCH_ANG_SS_MODEL.a));
                    % Natural frequency and damping factor of the dominant closed-loop poles
                    wn = VEL_SFC.wn_factor(1)*w_ol;
                    seta = VEL_SFC.damping_factor(1);
                    % Closed loop poles in continuous time
                    SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -VEL_SFC.p_factor -10].';
                else % LQR
                    SFC_IN.matQ = VEL_SFC.forward_vel_matQ;
                    SFC_IN.matR = VEL_SFC.forward_vel_matR;
                end
                % CONTROL DESIGN
                SFC_OUT = DESIGN_SFC(SFC_IN);
                % SFC CONTROL PARAMETERS
                CONTROL_INI.PARAM.FORWARD_VEL_SFC.K = SFC_OUT.K;
                CONTROL_INI.PARAM.FORWARD_VEL_SFC.Ki = SFC_OUT.Ki;
                CONTROL_INI.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                CONTROL_INI.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
            end
        end
        cd ../../SIMULINK
        sim(MODEL_SLX)
        flag_sim = 1;
    catch
        CurrentFolder = pwd;
        if strcmp(CurrentFolder(end-6:end),'CONTROL')
            cd ../../SIMULINK
        end
        flag_sim = 0;
    end
    ym = [];
    ys = [];
    flag_time = SCOPE_SIM.time>startTime;
    for ii = 1:size(out_fit,1)
        if size(out_fit,1)>1
            factor(ii) = sqrt(mean(SCOPE_SIM.signals(out_fit(ii,1)).values(:,out_fit(ii,2)).^2));
        end
        aux = SCOPE_SIM.signals(ref_fit(ii,1)).values(:,ref_fit(ii,2));
        aux = aux(flag_time);
        NN = length(aux);
        ym = [ym ; aux/factor(ii)]; % TARGET
        if flag_sim==1
            aux = SCOPE_SIM.signals(out_fit(ii,1)).values(:,out_fit(ii,2));
            aux = aux(flag_time);
            ys = [ys ; aux/factor(ii)]; % OUTPUT
        else
            ys = [ys ; zeros(NN,1)];
        end
    end
    % Define command noise threshold
    if niter==1
        % Command
        cmd_raw = double([SCOPE_SIM.signals(cmd_fit(1,1)).values(:,cmd_fit(1,2))]);
        % Command filtering
        cmd_filt = filtfilt(numF,denF,cmd_raw);
        % Command noise
        cmd_noise = (cmd_raw-cmd_filt);
        % Command noise standard deviation
        cmd_noise_std = std(cmd_noise); 
        % command noise threshold
        CMD_NOISE_THR = max([CMD_NOISE_THR 1.1*cmd_noise_std(:)']);
    end
    % Error computation (column vector)
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
        % Simulation
        warning('off','all')
        try
            cd ../SOFTWARE_COMPONENTS/CONTROL
            % FORWARD VELOCITY CONTROL
            if FIT_MODE == 1
                if CONTROL_TYPE == 0 % PID
                    % Specifications
                    if PID_DESIGN_METHOD == 0 % Frequency response
                        PID_IN.gain_margin_dB = FWD_VEL_PID.gain_margin_dB;
                        PID_IN.phase_margin_deg = FWD_VEL_PID.phase_margin_deg;
                        PID_IN.k_wd_P = FWD_VEL_PID.k_wd_P;
                        PID_IN.b = FWD_VEL_PID.b;
                        PID_IN.lag_phase_deg = FWD_VEL_PID.lag_phase_deg;
                        PID_IN.f = FWD_VEL_PID.f;
                    else % Time response
                        PID_IN.damping_factor = FWD_VEL_PID.damping_factor;
                        PID_IN.k_wd_P = FWD_VEL_PID.k_wd_P;
                        PID_IN.delta = FWD_VEL_PID.delta;
                        PID_IN.N = FWD_VEL_PID.N;
                        PID_IN.b = FWD_VEL_PID.b;
                    end
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                else % STATE FEEDBACK CONTROL
                    % Specifications
                    if SFC_DESIGN_METHOD==1 % Pole placement
                        % Open-loop pole frequency
                        w_ol = abs(eig(LIN_MODEL(1).FORWARD_VEL_SS_MODEL.a));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = VEL_SFC.wn_factor(1)*w_ol;
                        seta = VEL_SFC.damping_factor(1);
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j ].';
                    else % LQR
                        SFC_IN.matQ = VEL_SFC.forward_vel_matQ;
                        SFC_IN.matR = VEL_SFC.forward_vel_matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.K = SFC_OUT.K;
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.Ki = SFC_OUT.Ki;
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
                end
            end
            % YAW RATE CONTROL
            if FIT_MODE == 2
                if CONTROL_TYPE==0 % PID
                    % Specifications
                    if PID_DESIGN_METHOD == 0 % Frequency response
                        PID_IN.gain_margin_dB = YAW_RATE_PID.gain_margin_dB;
                        PID_IN.phase_margin_deg = YAW_RATE_PID.phase_margin_deg;
                        PID_IN.k_wd_P = YAW_RATE_PID.k_wd_P;
                        PID_IN.b = YAW_RATE_PID.b;
                        PID_IN.lag_phase_deg = YAW_RATE_PID.lag_phase_deg;
                        PID_IN.f = YAW_RATE_PID.f;
                    else % Time response
                        PID_IN.damping_factor = YAW_RATE_PID.damping_factor;
                        PID_IN.k_wd_P = YAW_RATE_PID.k_wd_P;
                        PID_IN.delta = YAW_RATE_PID.delta;
                        PID_IN.N = YAW_RATE_PID.N;
                        PID_IN.b = YAW_RATE_PID.b;
                    end
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    CONTROL_INI.PARAM.YAW_RATE_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.YAW_RATE_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.YAW_RATE_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.YAW_RATE_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.YAW_RATE_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.YAW_RATE_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.YAW_RATE_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.YAW_RATE_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.YAW_RATE_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                else % STATE FEEDBACK CONTROL
                    % Specifications
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Open-loop pole frequency
                        w_ol = abs(eig(LIN_MODEL(1).YAW_RATE_SS_MODEL.a));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = VEL_SFC.wn_factor(2)*w_ol;
                        seta = VEL_SFC.damping_factor(2);
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j ].';
                    else % LQR
                        SFC_IN.matQ = VEL_SFC.yaw_rate_matQ;
                        SFC_IN.matR = VEL_SFC.yaw_rate_matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.YAW_RATE_SFC.K = SFC_OUT.K;
                    CONTROL_INI.PARAM.YAW_RATE_SFC.Ki = SFC_OUT.Ki;
                    CONTROL_INI.PARAM.YAW_RATE_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                    CONTROL_INI.PARAM.YAW_RATE_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
                end
            end
            % YAW ANGLE CONTROL
            if FIT_MODE == 3
                if CONTROL_TYPE==0 % PID
                    % Specifications
                    PID_IN.gain_margin_dB = YAW_ANG_PID.gain_margin_dB;
                    PID_IN.phase_margin_deg = YAW_ANG_PID.phase_margin_deg;
                    PID_IN.k_wd_P = YAW_ANG_PID.k_wd_P;
                    PID_IN.b = YAW_ANG_PID.b;
                    PID_IN.lag_phase_deg = YAW_ANG_PID.lag_phase_deg;
                    PID_IN.f = YAW_ANG_PID.f;
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    CONTROL_INI.PARAM.YAW_ANG_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.YAW_ANG_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.YAW_ANG_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.YAW_ANG_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.YAW_ANG_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.YAW_ANG_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.YAW_ANG_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.YAW_ANG_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.YAW_ANG_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                else % STATE FEEDBACK CONTROL
                    % Specifications
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Open-loop maximun pole frequency
                        w_ol=max(abs(eig(LIN_MODEL(1).YAW_ANG_SS_MODEL.a)));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = YAW_ANG_SFC.wn_factor*w_ol;
                        seta = YAW_ANG_SFC.damping_factor;
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -YAW_ANG_SFC.p_factor].';
                    else % LQR
                        SFC_IN.matQ = YAW_ANG_SFC.matQ;
                        SFC_IN.matR = YAW_ANG_SFC.matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.YAW_ANG_SFC.K = SFC_OUT.K;
                    CONTROL_INI.PARAM.YAW_ANG_SFC.Ki = SFC_OUT.Ki;
                    CONTROL_INI.PARAM.YAW_ANG_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                    CONTROL_INI.PARAM.YAW_ANG_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
                end
            end
            % WALL DISTANCE CONTROL
            if FIT_MODE == 4
                if CONTROL_TYPE==0 % SINGLE-LOOP PID
                    % Specifications
                    PID_IN.phase_margin_deg = WFL_SL_PID.phase_margin_deg;
                    PID_IN.k_wd_P = WFL_SL_PID.k_wd_P;
                    PID_IN.b = WFL_SL_PID.b;
                    PID_IN.lag_phase_deg = WFL_SL_PID.lag_phase_deg;
                    PID_IN.f = WFL_SL_PID.f;
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
                    CONTROL_INI.PARAM.WFL_SL_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.WFL_SL_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.WFL_SL_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.WFL_SL_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.WFL_SL_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.WFL_SL_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.WFL_SL_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.WFL_SL_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.WFL_SL_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                elseif CONTROL_TYPE==1 % CASCADE PID
                    % Specifications
                    PID_IN.phase_margin_deg = WFL_CD_PID.phase_margin_deg;
                    PID_IN.k_wd_P = WFL_CD_PID.k_wd_P;
                    PID_IN.b = WFL_CD_PID.b;
                    PID_IN.lag_phase_deg = WFL_CD_PID.lag_phase_deg;
                    PID_IN.f = WFL_CD_PID.f;
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
                    CONTROL_INI.PARAM.WFL_CD_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.WFL_CD_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.WFL_CD_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.WFL_CD_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.WFL_CD_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.WFL_CD_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.WFL_CD_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.WFL_CD_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.WFL_CD_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                else % STATE FEEDBACK CONTROL
                    % Specifications
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = WFL_SFC.natural_freq;
                        seta = WFL_SFC.damping_factor;
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j  -WFL_SFC.p_factor].';
                    else  % LQR
                        SFC_IN.matQ = WFL_SFC.matQ;
                        SFC_IN.matR = WFL_SFC.matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.WFL_SFR.K(NUM_OP,:) = SFC_OUT.K;
                end
            end
            % PITCH ANGLE CONTROL
            if FIT_MODE == 5
                % STATE FEEDBACK CONTROL
                if CONTROL_TYPE == 1
                    % Specifications
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Open-loop maximun pole frequency
                        w_ol=max(eig(LIN_MODEL(1).PITCH_ANG_SS_MODEL.a));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = PITCH_ANG_SFC.wn_factor*w_ol;
                        seta = PITCH_ANG_SFC.damping_factor;
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -PITCH_ANG_SFC.p_factor].';
                    else % LQR
                        SFC_IN.matQ = PITCH_ANG_SFC.matQ;
                        SFC_IN.matR = PITCH_ANG_SFC.matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.PITCH_ANG_SFC.K = SFC_OUT.K;
                else
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Unstable pole frequency
                        w_ol = max(eig(LIN_MODEL(1).PITCH_ANG_SS_MODEL.a));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = VEL_SFC.wn_factor(1)*w_ol;
                        seta = VEL_SFC.damping_factor(1);
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -VEL_SFC.p_factor -10].';
                    else % LQR
                        SFC_IN.matQ = VEL_SFC.forward_vel_matQ;
                        SFC_IN.matR = VEL_SFC.forward_vel_matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.K = SFC_OUT.K;
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.Ki = SFC_OUT.Ki;
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
                end
            end
            cd ../../SIMULINK
            sim(MODEL_SLX)
            flag_sim = 1;
        catch
            CurrentFolder = pwd;
            if strcmp(CurrentFolder(end-6:end),'CONTROL')
                cd ../../SIMULINK
            end
            flag_sim = 0;
        end
        yaux = [];
        for ii = 1:size(out_fit,1)
            if flag_sim==1
                aux = SCOPE_SIM.signals(out_fit(ii,1)).values(:,out_fit(ii,2));
                aux = aux(flag_time);
                yaux = [yaux ; aux/factor(ii)]; % OUTPUT                
                % Command
                cmd_raw = double([SCOPE_SIM.signals(cmd_fit(1,1)).values(:,cmd_fit(1,2))]);
                % Command filtering
                cmd_filt = filtfilt(numF,denF,cmd_raw);
                % Command noise
                cmd_noise = (cmd_raw-cmd_filt);
                % Command noise standard deviation
                cmd_noise_std = std(cmd_noise);
            else
                yaux = [yaux ; zeros(NN,1)];
                cmd_noise_std = 0;
            end
        end
        % Jacobian matrix
        J(:,i)=(yaux-ys)/h;
    end
    dgn=(J\(ym-ys)); % Gauss-Newton search direction
    
    % SEARCH OF 'MU' (tunning of Gauss-Newton search-direction step to
    % ensure the cost function reduction)
    mu=1;  % mu initialization
    Vaux=1.05*V; % Initialization of the cost function update
    % 'mu' is divided by 2 while the cost function is not reduced
    thaux=theta;
    % Stability flag
    flag_sim = 0;
    cmd_noise_std = 0;
    while Vaux > V || any(thaux<0) || flag_sim == 0 || max(cmd_noise_std)>CMD_NOISE_THR
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
        % Simulation
        warning('off','all')       
        try
            cd ../SOFTWARE_COMPONENTS/CONTROL
            % FORWARD VELOCITY CONTROL
            if FIT_MODE == 1
                if CONTROL_TYPE == 0 % PID
                    % Specifications
                    if PID_DESIGN_METHOD == 0 % Frequency response
                        PID_IN.gain_margin_dB = FWD_VEL_PID.gain_margin_dB;
                        PID_IN.phase_margin_deg = FWD_VEL_PID.phase_margin_deg;
                        PID_IN.k_wd_P = FWD_VEL_PID.k_wd_P;
                        PID_IN.b = FWD_VEL_PID.b;
                        PID_IN.lag_phase_deg = FWD_VEL_PID.lag_phase_deg;
                        PID_IN.f = FWD_VEL_PID.f;
                    else % Time response
                        PID_IN.damping_factor = FWD_VEL_PID.damping_factor;
                        PID_IN.k_wd_P = FWD_VEL_PID.k_wd_P;
                        PID_IN.delta = FWD_VEL_PID.delta;
                        PID_IN.N = FWD_VEL_PID.N;
                        PID_IN.b = FWD_VEL_PID.b;
                    end
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.FORWARD_VEL_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                else % STATE FEEDBACK CONTROL
                    % Specifications
                    if SFC_DESIGN_METHOD==1 % Pole placement
                        % Open-loop pole frequency
                        w_ol = abs(eig(LIN_MODEL(1).FORWARD_VEL_SS_MODEL.a));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = VEL_SFC.wn_factor(1)*w_ol;
                        seta = VEL_SFC.damping_factor(1);
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j ].';
                    else % LQR
                        SFC_IN.matQ = VEL_SFC.forward_vel_matQ;
                        SFC_IN.matR = VEL_SFC.forward_vel_matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.K = SFC_OUT.K;
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.Ki = SFC_OUT.Ki;
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
                end
            end
            % YAW RATE CONTROL
            if FIT_MODE == 2
                if CONTROL_TYPE==0 % PID
                    % Specifications
                    if PID_DESIGN_METHOD == 0 % Frequency response
                        PID_IN.gain_margin_dB = YAW_RATE_PID.gain_margin_dB;
                        PID_IN.phase_margin_deg = YAW_RATE_PID.phase_margin_deg;
                        PID_IN.k_wd_P = YAW_RATE_PID.k_wd_P;
                        PID_IN.b = YAW_RATE_PID.b;
                        PID_IN.lag_phase_deg = YAW_RATE_PID.lag_phase_deg;
                        PID_IN.f = YAW_RATE_PID.f;
                    else % Time response
                        PID_IN.damping_factor = YAW_RATE_PID.damping_factor;
                        PID_IN.k_wd_P = YAW_RATE_PID.k_wd_P;
                        PID_IN.delta = YAW_RATE_PID.delta;
                        PID_IN.N = YAW_RATE_PID.N;
                        PID_IN.b = YAW_RATE_PID.b;
                    end
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    CONTROL_INI.PARAM.YAW_RATE_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.YAW_RATE_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.YAW_RATE_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.YAW_RATE_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.YAW_RATE_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.YAW_RATE_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.YAW_RATE_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.YAW_RATE_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.YAW_RATE_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                else % STATE FEEDBACK CONTROL
                    % Specifications
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Open-loop pole frequency
                        w_ol = abs(eig(LIN_MODEL(1).YAW_RATE_SS_MODEL.a));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = VEL_SFC.wn_factor(2)*w_ol;
                        seta = VEL_SFC.damping_factor(2);
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j ].';
                    else % LQR
                        SFC_IN.matQ = VEL_SFC.yaw_rate_matQ;
                        SFC_IN.matR = VEL_SFC.yaw_rate_matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.YAW_RATE_SFC.K = SFC_OUT.K;
                    CONTROL_INI.PARAM.YAW_RATE_SFC.Ki = SFC_OUT.Ki;
                    CONTROL_INI.PARAM.YAW_RATE_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                    CONTROL_INI.PARAM.YAW_RATE_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
                end
            end
            % YAW ANGLE CONTROL
            if FIT_MODE == 3
                if CONTROL_TYPE==0 % PID
                    % Specifications
                    PID_IN.gain_margin_dB = YAW_ANG_PID.gain_margin_dB;
                    PID_IN.phase_margin_deg = YAW_ANG_PID.phase_margin_deg;
                    PID_IN.k_wd_P = YAW_ANG_PID.k_wd_P;
                    PID_IN.b = YAW_ANG_PID.b;
                    PID_IN.lag_phase_deg = YAW_ANG_PID.lag_phase_deg;
                    PID_IN.f = YAW_ANG_PID.f;
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    CONTROL_INI.PARAM.YAW_ANG_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.YAW_ANG_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.YAW_ANG_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.YAW_ANG_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.YAW_ANG_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.YAW_ANG_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.YAW_ANG_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.YAW_ANG_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.YAW_ANG_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                else % STATE FEEDBACK CONTROL
                    % Specifications
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Open-loop maximun pole frequency
                        w_ol=max(abs(eig(LIN_MODEL(1).YAW_ANG_SS_MODEL.a)));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = YAW_ANG_SFC.wn_factor*w_ol;
                        seta = YAW_ANG_SFC.damping_factor;
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -YAW_ANG_SFC.p_factor].';
                    else % LQR
                        SFC_IN.matQ = YAW_ANG_SFC.matQ;
                        SFC_IN.matR = YAW_ANG_SFC.matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.YAW_ANG_SFC.K = SFC_OUT.K;
                    CONTROL_INI.PARAM.YAW_ANG_SFC.Ki = SFC_OUT.Ki;
                    CONTROL_INI.PARAM.YAW_ANG_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                    CONTROL_INI.PARAM.YAW_ANG_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
                end
            end
            % WALL DISTANCE CONTROL
            if FIT_MODE == 4
                if CONTROL_TYPE==0 % SINGLE-LOOP PID
                    % Specifications
                    PID_IN.phase_margin_deg = WFL_SL_PID.phase_margin_deg;
                    PID_IN.k_wd_P = WFL_SL_PID.k_wd_P;
                    PID_IN.b = WFL_SL_PID.b;
                    PID_IN.lag_phase_deg = WFL_SL_PID.lag_phase_deg;
                    PID_IN.f = WFL_SL_PID.f;
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
                    CONTROL_INI.PARAM.WFL_SL_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.WFL_SL_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.WFL_SL_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.WFL_SL_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.WFL_SL_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.WFL_SL_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.WFL_SL_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.WFL_SL_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.WFL_SL_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                elseif CONTROL_TYPE==1 % CASCADE PID
                    % Specifications
                    PID_IN.phase_margin_deg = WFL_CD_PID.phase_margin_deg;
                    PID_IN.k_wd_P = WFL_CD_PID.k_wd_P;
                    PID_IN.b = WFL_CD_PID.b;
                    PID_IN.lag_phase_deg = WFL_CD_PID.lag_phase_deg;
                    PID_IN.f = WFL_CD_PID.f;
                    % CONTROL DESIGN
                    PID_OUT = DESIGN_PID(PID_IN);
                    % PID CONTROL PARAMETERS
                    PID_OUT.K = PID_OUT.K*PID_OUT.D_ss.d;
                    CONTROL_INI.PARAM.WFL_CD_PID.K(NUM_OP,:) = PID_OUT.K;
                    CONTROL_INI.PARAM.WFL_CD_PID.Ti(NUM_OP,:) = PID_OUT.Ti;
                    CONTROL_INI.PARAM.WFL_CD_PID.Td(NUM_OP,:) = PID_OUT.Td;
                    CONTROL_INI.PARAM.WFL_CD_PID.N(NUM_OP,:) = PID_OUT.N;
                    CONTROL_INI.PARAM.WFL_CD_PID.b(NUM_OP,:) = PID_OUT.b;
                    CONTROL_INI.PARAM.WFL_CD_PID.INT_DISC_TYPE = uint8(PID_OUT.int_disc_type);
                    CONTROL_INI.PARAM.WFL_CD_PID.DER_DISC_TYPE = uint8(PID_OUT.der_disc_type);
                    CONTROL_INI.PARAM.WFL_CD_PID.DER_INPUT = uint8(PID_OUT.der_input);
                    CONTROL_INI.PARAM.WFL_CD_PID.ANTIWINDUP = uint8(PID_OUT.antiwindup);
                else % STATE FEEDBACK CONTROL
                    % Specifications
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = WFL_SFC.natural_freq;
                        seta = WFL_SFC.damping_factor;
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j  -WFL_SFC.p_factor].';
                    else  % LQR
                        SFC_IN.matQ = WFL_SFC.matQ;
                        SFC_IN.matR = WFL_SFC.matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.WFL_SFR.K(NUM_OP,:) = SFC_OUT.K;
                end
            end
            % PITCH ANGLE CONTROL
            if FIT_MODE == 5
                % STATE FEEDBACK CONTROL
                if CONTROL_TYPE == 1
                    % Specifications
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Open-loop maximun pole frequency
                        w_ol=max(eig(LIN_MODEL(1).PITCH_ANG_SS_MODEL.a));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = PITCH_ANG_SFC.wn_factor*w_ol;
                        seta = PITCH_ANG_SFC.damping_factor;
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -PITCH_ANG_SFC.p_factor].';
                    else % LQR
                        SFC_IN.matQ = PITCH_ANG_SFC.matQ;
                        SFC_IN.matR = PITCH_ANG_SFC.matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.PITCH_ANG_SFC.K = SFC_OUT.K;
                else
                    if SFC_DESIGN_METHOD == 1 % Pole placement
                        % Unstable pole frequency
                        w_ol = max(eig(LIN_MODEL(1).PITCH_ANG_SS_MODEL.a));
                        % Natural frequency and damping factor of the dominant closed-loop poles
                        wn = VEL_SFC.wn_factor(1)*w_ol;
                        seta = VEL_SFC.damping_factor(1);
                        % Closed loop poles in continuous time
                        SFC_IN.poles = wn*[-seta+sqrt(1-seta^2)*1j -seta-sqrt(1-seta^2)*1j -VEL_SFC.p_factor -10].';
                    else % LQR
                        SFC_IN.matQ = VEL_SFC.forward_vel_matQ;
                        SFC_IN.matR = VEL_SFC.forward_vel_matR;
                    end
                    % CONTROL DESIGN
                    SFC_OUT = DESIGN_SFC(SFC_IN);
                    % SFC CONTROL PARAMETERS
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.K = SFC_OUT.K;
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.Ki = SFC_OUT.Ki;
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = uint8(SFC_OUT.int_disc_type);
                    CONTROL_INI.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = uint8(SFC_OUT.antiwindup);
                end
            end
            cd ../../SIMULINK
            sim(MODEL_SLX)
            flag_sim = 1;
        catch
            CurrentFolder = pwd;
            if strcmp(CurrentFolder(end-6:end),'CONTROL')
                cd ../../SIMULINK
            end
            flag_sim = 0;
        end
        yaux = [];
        for ii = 1:size(out_fit,1)
            if flag_sim==1
                aux = SCOPE_SIM.signals(out_fit(ii,1)).values(:,out_fit(ii,2));
                aux = aux(flag_time);
                yaux = [yaux ; aux/factor(ii)]; % OUTPUT                
                % Command
                cmd_raw = double([SCOPE_SIM.signals(cmd_fit(1,1)).values(:,cmd_fit(1,2))]);
                % Command filtering
                cmd_filt = filtfilt(numF,denF,cmd_raw);
                % Command noise
                cmd_noise = (cmd_raw-cmd_filt);
                % Command noise standard deviation
                cmd_noise_std = std(cmd_noise);
            else
                yaux = [yaux ; zeros(NN,1)];
                cmd_noise_std = 0;
            end
        end
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
    time = SCOPE_SIM.time(flag_time);
    N_plot = length(time);
    if FIT_MODE<6 
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
% FINAL PARAMETERS
disp(' ')
disp('---------- CONTROL PARAMETERS ------------')
disp(' ')
nn = 1;
for ii = 1:NUM_PARAM
    if min(abs(PARAM_LIST-ii))==0
        command = ['fprintf(''' PARAM_STRING{ii} ' = %g;\n'',th(nn)*th_ini(nn));'];
        eval(command)
        nn=nn+1;
    end
end

% Covariance matrix for parameter vector (Sensitivities)
% P=Vaux^2*inv(J'*J);
% disp('Confidence interval in %')
% disp(100*sqrt(diag(P)')./theta)