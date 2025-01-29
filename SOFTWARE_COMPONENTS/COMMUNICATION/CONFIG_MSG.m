function [MSG_LIST,MSG] = CONFIG_MSG(CONTROL)
%--------------------------------------------------------------
UART_MODE = CONTROL.STATE.UART_MODE; 
%--------------------------------------------------------------
% MESSAGE LIST
% The maximum length of a message is 500 bytes
%--------------------------------------------------------------
nn = 0;
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%% SYSTEM & PC MONITORIZATION %%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 1 - RPI TO PC (1 x UINT8 = 1 BYTE)
MSG_LIST(nn).TYPE = '.STATE.';
MSG_LIST(nn).LABELS = {'CURRENT_STATUS_SYS'};
%--------------------------------------------------------------
nn = nn+1; % MSG 2 - PC TO RPI (4 x UINT8 = 4 BYTES)
MSG_LIST(nn).TYPE = '.STATE.';
MSG_LIST(nn).LABELS = {'CURRENT_STATUS_PC','PC_BUTTONS'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%% CONTROL PARAMETERS & CONFIGURATION %%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 3 - PC TO RPI (27 x UINT8 + 2 x SINGLE = 35 BYTES)
MSG_LIST(nn).TYPE = '.STATE.';
MSG_LIST(nn).LABELS = {'CONTROL_MODE','VEHICLE_MODE','COMM_MODE',...
                       'FORWARD_VEL_CONTROL_TYPE','YAW_RATE_CONTROL_TYPE',...
                       'YAW_ANG_CONTROL_TYPE','WFL_CONTROL_TYPE',...
                       'PITCH_ANG_CONTROL_TYPE','WFL_FEEDFORWARD',...
                       'GAIN_SCHEDULING','SP_MODE','MOTOR_DELAY_MODE',...
                       'OBSERVER_MODE','FORWARD_VEL_MAIN_OP',...
                       'ROTATION_MSRT_MODE','MCS_MODE','NAV_MODE',...
                       'MV_TARGET_SOURCE','YA_TARGET_SOURCE'...
                       'FV_TARGET_SOURCE','FV_TARGET_VALUE',...
                       'YR_TARGET_SOURCE','WD_TARGET_SOURCE',...
                       'MV_TARGET_TYPE','YA_TARGET_TYPE',...
                       'YR_TARGET_TYPE','FV_TARGET_TYPE','WD_TARGET_TYPE',...
                       'FILTER_CHANGE','STR_MODE','DELAY','WALL_FOLLOWER_MODE'....
                       'PA_INITIAL_VALUE'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OPERATING POINT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 4 - PC TO RPI (56 x SINGLE = 224 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'OP_INPUT','OP_STATE','OP_OUTPUT'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RECEIVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 5 - PC TO RPI (24 x SINGLE = 96 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'OPL_RC_SCALE','VEL_RC_SCALE','WFL_RC_SCALE',...
                       'OPL_RC_SWITCH','VEL_RC_SWITCH','WFL_RC_SWITCH'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% YAW ANGLE PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 6 - PC TO RPI (4 x UINT8 + 5 x SINGLE = 24 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.YAW_ANG_PID.';
MSG_LIST(nn).LABELS = {'K','Ti','Td','N','b','INT_DISC_TYPE',...
                       'DER_DISC_TYPE','DER_INPUT','ANTIWINDUP'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%% FORWARD VEL PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 7 - PC TO RPI (4 x UINT8 + 5 x SINGLE = 24 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.FORWARD_VEL_PID.';
MSG_LIST(nn).LABELS = {'K','Ti','Td','N','b','INT_DISC_TYPE',...
                       'DER_DISC_TYPE','DER_INPUT','ANTIWINDUP'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% YAW RATE PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 8 - PC TO RPI (4 x UINT8 + 5 x SINGLE = 24 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.YAW_RATE_PID.';
MSG_LIST(nn).LABELS = {'K','Ti','Td','N','b','INT_DISC_TYPE',...
                       'DER_DISC_TYPE','DER_INPUT','ANTIWINDUP'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%% FORWARD VEL SFC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 9 - PC TO RPI (2 x UINT8 + 4 x SINGLE = 18 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.FORWARD_VEL_SFC.';
MSG_LIST(nn).LABELS = {'K','Ki','INT_DISC_TYPE','ANTIWINDUP'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%% YAW RATE SFC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 10 - PC TO RPI (2 x UINT8 + 2 x SINGLE = 10 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.YAW_RATE_SFC.';
MSG_LIST(nn).LABELS = {'K','Ki','INT_DISC_TYPE','ANTIWINDUP'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%% YAW ANG SFC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 11 - PC TO RPI (2 x UINT8 + 3 x SINGLE = 14 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.YAW_ANG_SFC.';
MSG_LIST(nn).LABELS = {'K','Ki','INT_DISC_TYPE','ANTIWINDUP'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%% WALL FOLLOWER SFC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 12 - PC TO RPI (12 x SINGLE = 48 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.WFL_SFC.';
MSG_LIST(nn).LABELS = {'K'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%% WALL FOLLOWER SL PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 13 - PC TO RPI (4 x UINT8 + 20 x SINGLE = 84 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.WFL_SL_PID.';
MSG_LIST(nn).LABELS = {'K','Ti','Td','N','b','INT_DISC_TYPE',...
                       'DER_DISC_TYPE','DER_INPUT','ANTIWINDUP'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%% WALL FOLLOWER CD PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 14 - PC TO RPI (4 x UINT8 + 20 x SINGLE = 84 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.WFL_CD_PID.';
MSG_LIST(nn).LABELS = {'K','Ti','Td','N','b','INT_DISC_TYPE',...
                       'DER_DISC_TYPE','DER_INPUT','ANTIWINDUP'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%% NAVIGATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 15 - PC TO RPI (1 x SINGLE = 4 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.NAV_MPC.';
MSG_LIST(nn).LABELS = {'PRED_HRZ'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%% FILTERS PARAM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 16 - PC TO RPI (55 x SINGLE = 220 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'CONTROL_SAMPLING_TIME',...
                       'IMU_FILT_FREQ','ENC_FILT_FREQ','EKF_FILT_FREQ',...
                       'WFL_FILT_FREQ','IMU_CF_FREQ',...
                       'VEL_GAIN','VEL_TIME_CONSTANT',...
                       'YAW_RATE_GAIN','YAW_RATE_TIME_CONSTANT',...
                       'MOTOR_DELAY','MOTOR_DELAY_ERR','MOTOR_VOLT_DROP_DIF',...
                       'MOTOR_VOLT_MAX','MOTOR_VOLT_MIN','MOTOR_DEAD_ZONE','MOTOR_PWM_SLOPE',...
                       'VEL_MAG_MAX','MCS_ALFA_CALIB','NAV_ALFA_CALIB'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%% HW MONITORIZATION 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 17 - RPI TO PC (47 x SINGLE = 188 BYTES)
MSG_LIST(nn).TYPE = '.INPUT.';
MSG_LIST(nn).LABELS = {'BATTERY_VOLT','CHARGING_CURRENT',...
                       'IMU_ACCEL_MEAN','IMU_ACCEL_MEAN_FILT','IMU_GYRO_MEAN','IMU_GYRO_MEAN_FILT',...
                       'IMU_EULER_RATE_CF','IMU_EULER_RATE_KF',...
                       'IMU_EULER_ANG_CF','IMU_EULER_ANG_KF','IMU_PITCH_ANG_INI',...
                       'IMU_FORWARD_ACCEL_CF','IMU_FORWARD_ACCEL_KF',...
                       'ENC_MOTOR_RATE_LPF','ENC_MOTOR_RATE_KF',...
                       'ENC_MOTOR_RATE_CD_LPF','ENC_MOTOR_RATE_CD_KF',...
                       'ENC_FORWARD_ACCEL_LPF','ENC_FORWARD_ACCEL_KF',...
                       'ENC_FORWARD_VEL_LPF','ENC_FORWARD_VEL_KF',...     
                       'ENC_YAW_RATE_LPF','ENC_YAW_RATE_KF',...
                       'ENC_YAW_ANG','ENC_YAW_ANG_LPF','ENC_YAW_ANG_KF'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 18 - RPI TO PC (6 x SINGLE = 24 BYTES)
MSG_LIST(nn).TYPE = '.OUTPUT.';
MSG_LIST(nn).LABELS = {'MOTOR_PWM','MOTOR_VOLT','MOTOR_VOLT_CD'};
%-------------------------------------------------------------- 
nn = nn+1; % MSG 19 - RPI TO PC (6 x UINT8 + 2 x SINGLE = 14 BYTES)
MSG_LIST(nn).TYPE = '.STATE.';
MSG_LIST(nn).LABELS = {'BUTTONS','MOTOR_MODE','CPU_LOAD','MCS_RX_STATUS',...
                       'NAV_RX_STATUS','COMP_TIME','COMP_IAE'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 20 - RPI TO PC (9 x SINGLE = 36 BYTES)
MSG_LIST(nn).TYPE = '.TARGET.';
MSG_LIST(nn).LABELS = {'YAW_RATE_REF','YAW_ANG_REF','PITCH_ANG_REF','FORWARD_VEL_REF',...
                       'WALL_DIST_REF','POS_XY_REF','VEL_XY_REF'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 21 - RPI TO PC (1 x UINT8 + 10 x SINGLE = 41 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'VEL_GAIN_EST','VEL_TIME_CONSTANT_EST','FILT_TIME_CONSTANT_EST',...
                       'FV_PI_STR_K','FV_PI_STR_Ti','FV_PI_STR_b','ADPT_FLAG',...
                       'FORWARD_VEL_PID.K','FORWARD_VEL_PID.Ti','FORWARD_VEL_PID.b',...
                       'FILT_TIME_CONSTANT_ACT'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 22 - PC TO RPI (7 x SINGLE = 28 BYTES)
MSG_LIST(nn).TYPE = '.EKF_WFL.';
MSG_LIST(nn).LABELS = {'PARAM.RANGE_XA','PARAM.RANGE_YA',...
                       'PARAM.LIDAR_2D_XA','PARAM.LIDAR_2D_YA',...
                       'PARAM.LIDAR_2D_WFL_ANG_OFFS','PARAM.LIDAR_2D_WFL_ANG_SIGN',...
                       'PROCESS_NOISE_VAR','OBSRV_NOISE_VAR'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 23 - PC TO RPI (57 x SINGLE = 228 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'ENC_FORWARD_VEL_matAd','ENC_FORWARD_VEL_matBd',...
                       'ENC_FORWARD_VEL_matCd','ENC_FORWARD_VEL_matQ','ENC_FORWARD_VEL_matR',...
                       'ENC_YAW_RATE_matAd','ENC_YAW_RATE_matBd',...
                       'ENC_YAW_RATE_matCd','ENC_YAW_RATE_matQ','ENC_YAW_RATE_matR'};
%--------------------------------------------------------------
nn = nn+1; % MSG 24 - PC TO RPI (17 x SINGLE = 68 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'SVF_Ad','SVF_B1d','SVF_B2d','SVF_Cd','SVF_Dd'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%% HW MONITORIZATION 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 25 - RPI TO PC (45 x SINGLE = 180 BYTES)
MSG_LIST(nn).TYPE = '.INPUT.';
MSG_LIST(nn).LABELS = {'RANGE_FRONT_DIST','RANGE_REAR_DIST',...
                       'RANGE_WALL_DIST','LIDAR_2D_WALL_DIST','WALL_DIST_FILT','WALL_DIST_KF',...
                       'RANGE_WALL_ANG','LIDAR_2D_WALL_ANG','WALL_ANG_FILT','WALL_ANG_KF',...
                       'MCS_EULER_ANG','MCS_EARTH_POS','NAV_EULER_ANG','NAV_EARTH_POS',...
                       'NAV_EARTH_POS_FILT','NAV_EARTH_VEL_FILT',...
                       'MOTOR_RATE','MOTOR_RATE_CD',...
                       'WALL_DIST','FORWARD_VEL','YAW_RATE','PITCH_RATE',...
                       'YAW_ANG','PITCH_ANG','EARTH_POS','EARTH_VEL'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 26 - PC TO RPI (6 x SINGLE = 24 BYTES)
MSG_LIST(nn).TYPE = '.EKF_NAV.';
MSG_LIST(nn).LABELS = {'PROCESS_NOISE_VAR','OBSRV_NOISE_VAR'};
%--------------------------------------------------------------
nn = nn+1; % MSG 27 - PC TO RPI (14 x SINGLE = 56 BYTES)
MSG_LIST(nn).TYPE = '.EKF_IMU.';
MSG_LIST(nn).LABELS = {'PARAM.ACCEL_ADPTV_GAIN','PROCESS_NOISE_VAR','OBSRV_NOISE_VAR'};
%--------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%% PITCH ANGLE SFC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nn = nn+1; % MSG 28 - PC TO RPI (12 x SINGLE = 48 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.PITCH_ANG_SFC.';
MSG_LIST(nn).LABELS = {'K'};
%--------------------------------------------------------------
nn = nn+1; % MSG 29 - PC TO RPI ()
MSG_LIST(nn).TYPE = '.TARGET.';
MSG_LIST(nn).LABELS = {'MISSION_NUM_WAYPOINTS','MISSION_WAYPOINTS',...
                       'MISSION_WP_RADIUS','MISSION_YA_ERROR'};
%--------------------------------------------------------------
nn = nn+1; % MSG 30 - PC TO RPI (10 x SINGLE = 40 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'FORWARD_VEL_DB.NUM','FORWARD_VEL_DB.DEN',...
                       'YAW_RATE_DB.NUM','YAW_RATE_DB.DEN'};
%--------------------------------------------------------------
NUM_MSG = nn;
%--------------------------------------------------------------
% MESSAGE IDs LIST
%--------------------------------------------------------------
if UART_MODE == 1
    MSG.MSG_ID_LIST = [uint8([3 4 0 0 0 0 0 0]'),...
                       uint8([5 6 7 8 9 10 11 0]'),...
                       uint8([12 13 14 15 0 0 0 0]'),...                                              
                       uint8([16 22 0 0 0 0 0]'),...
                       uint8([23 24 0 0 0 0 0 0]'),...
                       uint8([26 27 28 30 0 0 0 0]'),...
                       uint8([29 0 0 0 0 0 0 0]')];
else
    MSG.MSG_ID_LIST = [uint8([2 3 4 0 0 0 0 0]'),...
                       uint8([2 5 6 7 8 9 10 11]'),...
                       uint8([2 12 13 14 15 0 0 0]'),...
                       uint8([2 16 22 0 0 0 0 0]'),...
                       uint8([2 23 24 0 0 0 0 0]'),...
                       uint8([2 26 27 28 30 0 0 0]'),...
                       uint8([2 29 0 0 0 0 0 0]')];
end
%--------------------------------------------------------------
% MESSAGE CODER
%--------------------------------------------------------------
for nn=1:NUM_MSG
    NUM_LABELS = length(MSG_LIST(nn).LABELS);  
    command = sprintf('DATA = [];\n');
    MSG_LIST(nn).CODER = [];
    MSG_LIST(nn).CODER = [MSG_LIST(nn).CODER command];
    for ii=1:NUM_LABELS
        FIELD = ['CONTROL' MSG_LIST(nn).TYPE char(MSG_LIST(nn).LABELS(ii))];
        command = ['class_field = class(' FIELD ');'];
        eval(command)
        % Number of dimensions
        command = ['num_dim = ndims(' FIELD ');'];
        eval(command)
        % Dimension sizes
        nd = zeros(1,num_dim);
        for hh = 1:num_dim
            command = ['nd(hh) = size(' FIELD ',hh);'];
            eval(command);
        end        
        if strcmp(class_field,'double')
            if prod(nd)==1
                command = sprintf('DATA = [ DATA ; typecast(single(%s(:)),''uint8'')''];\n',FIELD);
            else
                command = sprintf('DATA = [ DATA ; typecast(single(%s(:)),''uint8'')];\n',FIELD);
            end
        else
            command = sprintf('DATA = [ DATA ; %s(:)];\n',FIELD);
        end
        MSG_LIST(nn).CODER = [MSG_LIST(nn).CODER command];
    end
end  

%--------------------------------------------------------------
% MESSAGE DECODER
%--------------------------------------------------------------
for nn=1:NUM_MSG
    NUM_LABELS = length(MSG_LIST(nn).LABELS);  
    MSG_LIST(nn).DECODER = [];
    % DATA counter
    jj = 1;        
    for ii=1:NUM_LABELS
        FIELD = ['CONTROL' MSG_LIST(nn).TYPE char(MSG_LIST(nn).LABELS(ii))];
        command = ['class_field = class(' FIELD ');'];
        eval(command)
        % Number of dimensions
        command = ['num_dim = ndims(' FIELD ');'];
        eval(command)
        % Dimension sizes
        nd = zeros(1,num_dim);
        for hh = 1:num_dim
            command = ['nd(hh) = size(' FIELD ',hh);'];
            eval(command);
        end
        % Data selection and reshape
        switch num_dim 
            case 2
                if strcmp(class_field,'double')
                    command = sprintf('%s = reshape(double(typecast(DATA(DataOffset+%d:DataOffset+%d),''single'')),%d,%d);\n',FIELD,jj,jj+4*prod(nd)-1,nd(1),nd(2));
                    jj = jj + 4*prod(nd);
                else
                    command = sprintf('%s = reshape(DATA(DataOffset+%d:DataOffset+%d),%d,%d);\n',FIELD,jj,jj+prod(nd)-1,nd(1),nd(2));
                    jj = jj + prod(nd);
                end
            case 3
                command = sprintf('%s = reshape(double(typecast(DATA(DataOffset+%d:DataOffset+%d),''single'')),%d,%d,%d);\n',FIELD,jj,jj+4*prod(nd)-1,nd(1),nd(2),nd(3));
                jj = jj + 4*prod(nd);
        end
        
        MSG_LIST(nn).DECODER = [MSG_LIST(nn).DECODER command];
    end
end  

%--------------------------------------------------------------
% MESSAGE STRUCT DEFINITION
%--------------------------------------------------------------
MSG.RX_BUFFER = zeros(500,1,'uint8');
MSG.TX_BUFFER = zeros(500,1,'uint8');
MSG.TX_ID = zeros(8,1,'uint8');

%--------------------------------------------------------------
% MESSAGE VERIFICATION
%--------------------------------------------------------------
% for nn=1:NUM_MSG
%     eval(MSG_LIST(nn).CODER)
%     eval(MSG_LIST(nn).DECODER)
% end

%--------------------------------------------------------------
% FULL CODER GENERATION
%--------------------------------------------------------------
CoderFile = 'MSG_CODER.m';
CoderFunction = [];
command = sprintf('function MSG = MSG_CODER(MSG_IN,CONTROL)\n');
CoderFunction = [CoderFunction command];
command = sprintf('%%--------------------------------------------------------------\n');
CoderFunction = [CoderFunction command];
command = sprintf('MSG = MSG_IN;\n');
CoderFunction = [CoderFunction command];
command = sprintf('%%--------------------------------------------------------------\n');
CoderFunction = [CoderFunction command];
% Header (2*num_msg + 1)
% [num_msg / msg_id(1) / msg_len(1) / ... / msg_id(num_msg) / msg_len(num_msg) /]
command = sprintf('TX_ID = sort(MSG.TX_ID(MSG.TX_ID>0));\n');
CoderFunction = [CoderFunction command];
command = sprintf('NUM_MSG = uint8(length(TX_ID));\n');
CoderFunction = [CoderFunction command];
command = sprintf('HEADER = zeros(2*NUM_MSG+1,1,''uint8'');\n');
CoderFunction = [CoderFunction command];
command = sprintf('HEADER(1) = NUM_MSG;\n');
CoderFunction = [CoderFunction command];
command = sprintf('for nn = 1:NUM_MSG\n');
CoderFunction = [CoderFunction command];
command = sprintf('HEADER(2*(nn-1)+2) = uint8(TX_ID(nn));\n');
CoderFunction = [CoderFunction command];
command = sprintf('end\n');
CoderFunction = [CoderFunction command];
command = sprintf('BUFFER = [];\n');
CoderFunction = [CoderFunction command];
command = sprintf('nn = 1;\n');
CoderFunction = [CoderFunction command];
for nn=1:NUM_MSG
    command = sprintf('%%--------------------------------------------------------------\n');
    CoderFunction = [CoderFunction command];
    command = sprintf('if ~all(TX_ID ~= %d)\n',nn);
    CoderFunction = [CoderFunction command];
    CoderFunction = [CoderFunction MSG_LIST(nn).CODER];     
    command = sprintf('BUFFER = [BUFFER ; DATA];\n',nn);
    CoderFunction = [CoderFunction command];
    command = sprintf('HEADER(2*(nn-1)+3) = uint8(length(DATA));\n');
    CoderFunction = [CoderFunction command];
    command = sprintf('nn = nn + 1;\n');
    CoderFunction = [CoderFunction command];
    command = sprintf('end\n');
    CoderFunction = [CoderFunction command]; 
end
command = sprintf('BUFFER = [HEADER ; BUFFER];\n');
CoderFunction = [CoderFunction command];
command = sprintf('BUFFER_LEN = length(BUFFER);\n');
CoderFunction = [CoderFunction command];
command = sprintf('MSG.TX_BUFFER = zeros(500,1,''uint8'');\n');
CoderFunction = [CoderFunction command];
command = sprintf('MSG.TX_BUFFER(1:BUFFER_LEN) = BUFFER;\n');
CoderFunction = [CoderFunction command];
% Delete coder file
% Write output file
fid = fopen(CoderFile,'wt');
fprintf(fid,'%s\n',CoderFunction);
fclose(fid);
% CoderScript = matlab.desktop.editor.openDocument(fullfile(pwd,CoderFile));
% CoderScript.smartIndentContents;
% CoderScript.save;
% CoderScript.close;

%--------------------------------------------------------------
% FULL DECODER GENERATION
%--------------------------------------------------------------
DecoderFile = 'MSG_DECODER.m';
DecoderFunction = [];
command = sprintf('function [MSG,CONTROL] = MSG_DECODER(MSG_IN,CONTROL_IN)\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('%%--------------------------------------------------------------\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('MSG = MSG_IN;\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('CONTROL = CONTROL_IN;\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('%%--------------------------------------------------------------\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('BUFFER = MSG.RX_BUFFER;\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('NUM_MSG = double(BUFFER(1));\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('HEADER = BUFFER(1:2*NUM_MSG+1);\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('RX_ID = sort(HEADER(2:2:end));\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('MSG_LEN = double(HEADER(3:2:end));\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('DATA_LEN = sum(MSG_LEN);\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('DATA = BUFFER(2*NUM_MSG+2:2*NUM_MSG+2+DATA_LEN-1);\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('nn = uint8(0);\n');
DecoderFunction = [DecoderFunction command];
command = sprintf('DataOffset = 0;\n');
DecoderFunction = [DecoderFunction command];
for nn=1:NUM_MSG
    command = sprintf('%%--------------------------------------------------------------\n');
    DecoderFunction = [DecoderFunction command];
    command = sprintf('if ~all(RX_ID ~= %d)\n',nn);
    DecoderFunction = [DecoderFunction command];
    DecoderFunction = [DecoderFunction MSG_LIST(nn).DECODER];
    command = sprintf('nn = nn + 1;\n');
    DecoderFunction = [DecoderFunction command];
    command = sprintf('DataOffset = DataOffset + MSG_LEN(nn);\n');
    DecoderFunction = [DecoderFunction command];      
    command = sprintf('end\n');
    DecoderFunction = [DecoderFunction command]; 
end
% Write output file
fid = fopen(DecoderFile,'Wt');
fprintf(fid,'%s\n',DecoderFunction);
fclose(fid);
% DecoderScript = matlab.desktop.editor.openDocument(fullfile(pwd,DecoderFile));
% DecoderScript.smartIndentContents;
% DecoderScript.save;
% DecoderScript.close;

return