function [MSG_LIST,MSG] = CONFIG_MSG(CONTROL)
%--------------------------------------------------------------
UART_MODE = CONTROL.STATE.UART_MODE; 
%--------------------------------------------------------------
% MESSAGE LIST
% The maximum length of a message is 1400 bytes
%--------------------------------------------------------------
nn = 0;
%--------------------------------------------------------------
nn = nn+1; % MSG 1 - RPI TO PC (1 x UINT8 = 1 BYTE)
MSG_LIST(nn).TYPE = '.STATE.';
MSG_LIST(nn).LABELS = {'CURRENT_STATUS_SYS'};
%--------------------------------------------------------------
nn = nn+1; % MSG 2 - PC TO RPI (4 x UINT8 = 4 BYTES)
MSG_LIST(nn).TYPE = '.STATE.';
MSG_LIST(nn).LABELS = {'CURRENT_STATUS_PC','PC_BUTTONS'};
%--------------------------------------------------------------
nn = nn+1; % MSG 3 - PC TO RPI (37 x UINT8 = 37 BYTES)
MSG_LIST(nn).TYPE = '.STATE.';
MSG_LIST(nn).LABELS = {'CONTROL_MODE','VEHICLE_MODE','COMM_MODE',...
                       'FORWARD_VEL_CONTROL_TYPE','YAW_RATE_CONTROL_TYPE',...
                       'YAW_ANG_CONTROL_TYPE','WFL_CONTROL_TYPE','NAV_CONTROL_TYPE',...
                       'PITCH_ANG_CONTROL_TYPE','WFL_FEEDFORWARD',...
                       'GAIN_SCHEDULING','MOTOR_DELAY_MODE',...
                       'OBSERVER_MODE','FORWARD_VEL_MAIN_OP',...
                       'ROTATION_MSRT_MODE','MCS_MODE','NAV_MODE',...
                       'YA_TARGET_SOURCE','FV_TARGET_SOURCE','YA_TARGET_TYPE',...
                       'YR_TARGET_SOURCE','WD_TARGET_SOURCE','NAV_TARGET_SOURCE',...
                       'YR_TARGET_TYPE','FV_TARGET_TYPE','WD_TARGET_TYPE',...
                       'FILTER_CHANGE','STR_MODE','WALL_FOLLOWER_MODE',....
                       'SP_MODE','MV_TARGET_SOURCE','MV_TARGET_TYPE','NAV_TARGET_TYPE'};
%--------------------------------------------------------------
nn = nn+1; % MSG 4 - PC TO RPI (56 x SINGLE = 224 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'OP_INPUT','OP_STATE','OP_OUTPUT'};
%--------------------------------------------------------------
nn = nn+1; % MSG 5 - PC TO RPI (60 x SINGLE = 240 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'OPL_RC_SCALE','VEL_RC_SCALE','WFL_RC_SCALE','NAV_RC_SCALE',...
                       'OPL_RC_SWITCH','VEL_RC_SWITCH','WFL_RC_SWITCH','NAV_RC_SWITCH',...
                       'FORWARD_VEL_DB.NUM','FORWARD_VEL_DB.DEN',...
                       'YAW_RATE_DB.NUM','YAW_RATE_DB.DEN',...
                       'NAV_SFC.INT_DISC_TYPE','NAV_SFC.ANTIWINDUP',...
                       'NAV_STOP_RADIUS','NAV_MAX_TARGET_RADIUS',...
                       'NAV_SFC.COS_ANG','NAV_SFC.FV','NAV_SFC.INV_DIST'};
%--------------------------------------------------------------
nn = nn+1; % MSG 6 - PC TO RPI (20 x UINT8 + 55 x SINGLE = 240 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'FORWARD_VEL_PID.K','FORWARD_VEL_PID.Ti','FORWARD_VEL_PID.Td',...
                       'FORWARD_VEL_PID.N','FORWARD_VEL_PID.b',...
                       'FORWARD_VEL_PID.INT_DISC_TYPE','FORWARD_VEL_PID.DER_DISC_TYPE',...
                       'FORWARD_VEL_PID.DER_INPUT','FORWARD_VEL_PID.ANTIWINDUP',...
                       'YAW_RATE_PID.K','YAW_RATE_PID.Ti','YAW_RATE_PID.Td',...
                       'YAW_RATE_PID.N','YAW_RATE_PID.b',...
                       'YAW_RATE_PID.INT_DISC_TYPE','YAW_RATE_PID.DER_DISC_TYPE',...
                       'YAW_RATE_PID.DER_INPUT','YAW_RATE_PID.ANTIWINDUP',...
                       'YAW_ANG_PID.K','YAW_ANG_PID.Ti','YAW_ANG_PID.Td',...
                       'YAW_ANG_PID.N','YAW_ANG_PID.b',...
                       'YAW_ANG_PID.INT_DISC_TYPE','YAW_ANG_PID.DER_DISC_TYPE',...
                       'YAW_ANG_PID.DER_INPUT','YAW_ANG_PID.ANTIWINDUP',...
                       'WFL_SL_PID.K','WFL_SL_PID.Ti','WFL_SL_PID.Td',...
                       'WFL_SL_PID.N','WFL_SL_PID.b',...
                       'WFL_SL_PID.INT_DISC_TYPE','WFL_SL_PID.DER_DISC_TYPE',...
                       'WFL_SL_PID.DER_INPUT','WFL_SL_PID.ANTIWINDUP',...
                       'WFL_CD_PID.K','WFL_CD_PID.Ti','WFL_CD_PID.Td',...
                       'WFL_CD_PID.N','WFL_CD_PID.b',...
                       'WFL_CD_PID.INT_DISC_TYPE','WFL_CD_PID.DER_DISC_TYPE',...
                       'WFL_CD_PID.DER_INPUT','WFL_CD_PID.ANTIWINDUP'};
%--------------------------------------------------------------
nn = nn+1; % MSG 7 - PC TO RPI (6 x UINT8 + 56 x SINGLE = 230 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'FORWARD_VEL_SFC.K','FORWARD_VEL_SFC.Ki',...
                       'FORWARD_VEL_SFC.INT_DISC_TYPE','FORWARD_VEL_SFC.ANTIWINDUP',...
                       'YAW_RATE_SFC.K','YAW_RATE_SFC.Ki',...
                       'YAW_RATE_SFC.INT_DISC_TYPE','YAW_RATE_SFC.ANTIWINDUP',...
                       'YAW_ANG_SFC.K','YAW_ANG_SFC.Ki',...
                       'YAW_ANG_SFC.INT_DISC_TYPE','YAW_ANG_SFC.ANTIWINDUP',...
                       'WFL_SFC.K','PITCH_ANG_SFC.K',...
                       'CONTROL_SAMPLING_TIME','IMU_FILT_FREQ','ENC_FILT_FREQ','EKF_FILT_FREQ',...
                       'WFL_FILT_FREQ','IMU_CF_FREQ','VEL_GAIN','VEL_TIME_CONSTANT',...
                       'YAW_RATE_GAIN','YAW_RATE_TIME_CONSTANT',...
                       'MOTOR_DELAY','MOTOR_DELAY_ERR','MOTOR_VOLT_DROP_DIF',...
                       'MCS_ALFA_CALIB','NAV_ALFA_CALIB',...
                       'MOTOR_VOLT_MAX','MOTOR_VOLT_MIN','MOTOR_DEAD_ZONE','MOTOR_PWM_SLOPE'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 8 - PC TO RPI (57 x SINGLE = 228 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'ENC_FORWARD_VEL_matAd','ENC_FORWARD_VEL_matBd',...
                       'ENC_FORWARD_VEL_matCd','ENC_FORWARD_VEL_matQ','ENC_FORWARD_VEL_matR',...
                       'ENC_YAW_RATE_matAd','ENC_YAW_RATE_matBd',...
                       'ENC_YAW_RATE_matCd','ENC_YAW_RATE_matQ','ENC_YAW_RATE_matR'};
%--------------------------------------------------------------
nn = nn+1; % MSG 9 - PC TO RPI (43 x SINGLE = 172 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'MISSION_NUM_WAYPOINTS','MISSION_WAYPOINTS',...
                       'MISSION_WP_RADIUS','MISSION_YA_ERROR'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 10 - PC TO RPI (53 x SINGLE = 212 BYTES)
MSG_LIST(nn).TYPE = '.';
MSG_LIST(nn).LABELS = {'EKF_WFL.PARAM.RANGE_XA','EKF_WFL.PARAM.RANGE_YA',...
                       'EKF_WFL.PARAM.LIDAR_2D_XA','EKF_WFL.PARAM.LIDAR_2D_YA',...
                       'EKF_WFL.PARAM.LIDAR_2D_WFL_ANG_OFFS','EKF_WFL.PARAM.LIDAR_2D_WFL_ANG_SIGN',...
                       'EKF_WFL.PROCESS_NOISE_VAR','EKF_WFL.OBSRV_NOISE_VAR',...
                       'EKF_NAV.PROCESS_NOISE_VAR','EKF_NAV.OBSRV_NOISE_VAR',...
                       'EKF_IMU.PARAM.ACCEL_ADPTV_GAIN','EKF_IMU.PROCESS_NOISE_VAR','EKF_IMU.OBSRV_NOISE_VAR',...
                       'PARAM.SVF_Ad','PARAM.SVF_B1d','PARAM.SVF_B2d','PARAM.SVF_Cd','PARAM.SVF_Dd',...
                       'PARAM.CONTROL_ACT_DELAY','PARAM.PA_INITIAL_VALUE','PARAM.FV_TARGET_VALUE','PARAM.NAV_TARGET_VALUE'};
%--------------------------------------------------------------
% MSG 11-42 - PC TO RPI (63 x SINGLE = 252 BYTES / 62 x SINGLE = 248 BYTES)
for ii = 1:16
    nn = nn+1;
    MSG_LIST(nn).TYPE = '.PARAM.NAV_SFC.';
    MSG_LIST(nn).LABELS = {['KPP(1:63,' num2str(ii) ')']};
    nn = nn+1;
    MSG_LIST(nn).TYPE = '.PARAM.NAV_SFC.';
    MSG_LIST(nn).LABELS = {['KPP(64:125,' num2str(ii) ')']};
end
%--------------------------------------------------------------
% MSG 43-74 - PC TO RPI (63 x SINGLE = 252 BYTES / 62 x SINGLE = 248 BYTES)
for ii = 1:16
    nn = nn+1;
    MSG_LIST(nn).TYPE = '.PARAM.NAV_SFC.';
    MSG_LIST(nn).LABELS = {['KPN(1:63,' num2str(ii) ')']};
    nn = nn+1;
    MSG_LIST(nn).TYPE = '.PARAM.NAV_SFC.';
    MSG_LIST(nn).LABELS = {['KPN(64:125,' num2str(ii) ')']};
end
%--------------------------------------------------------------
% MSG 75-106 - PC TO RPI (63 x SINGLE = 252 BYTES / 62 x SINGLE = 248 BYTES)
for ii = 1:16
    nn = nn+1;
    MSG_LIST(nn).TYPE = '.PARAM.NAV_SFC.';
    MSG_LIST(nn).LABELS = {['KNP(1:63,' num2str(ii) ')']};
    nn = nn+1;
    MSG_LIST(nn).TYPE = '.PARAM.NAV_SFC.';
    MSG_LIST(nn).LABELS = {['KNP(64:125,' num2str(ii) ')']};
end
%--------------------------------------------------------------
% MSG 107-138 - PC TO RPI (63 x SINGLE = 252 BYTES / 62 x SINGLE = 248 BYTES)
for ii = 1:16
    nn = nn+1;
    MSG_LIST(nn).TYPE = '.PARAM.NAV_SFC.';
    MSG_LIST(nn).LABELS = {['KNN(1:63,' num2str(ii) ')']};
    nn = nn+1;
    MSG_LIST(nn).TYPE = '.PARAM.NAV_SFC.';
    MSG_LIST(nn).LABELS = {['KNN(64:125,' num2str(ii) ')']};
end
%--------------------------------------------------------------
nn = nn+1; % MSG 139 - RPI TO PC (46 x SINGLE = 184 BYTES)
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
nn = nn+1; % MSG 140 - RPI TO PC (6 x SINGLE = 24 BYTES)
MSG_LIST(nn).TYPE = '.OUTPUT.';
MSG_LIST(nn).LABELS = {'MOTOR_PWM','MOTOR_VOLT','MOTOR_VOLT_CD'};
%-------------------------------------------------------------- 
nn = nn+1; % MSG 141 - RPI TO PC (7 x UINT8 + 2 x SINGLE = 15 BYTES)
MSG_LIST(nn).TYPE = '.STATE.';
MSG_LIST(nn).LABELS = {'BUTTONS','MOTOR_MODE','CPU_LOAD','MCS_RX_STATUS',...
                       'NAV_RX_STATUS','COMP_TIME','COMP_IAE'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 142 - RPI TO PC (9 x SINGLE = 36 BYTES)
MSG_LIST(nn).TYPE = '.TARGET.';
MSG_LIST(nn).LABELS = {'YAW_RATE_REF','YAW_ANG_REF','PITCH_ANG_REF','FORWARD_VEL_REF',...
                       'WALL_DIST_REF','POS_XY_REF','VEL_XY_REF'};
%--------------------------------------------------------------                   
nn = nn+1; % MSG 143 - RPI TO PC (1 x UINT8 + 10 x SINGLE = 41 BYTES)
MSG_LIST(nn).TYPE = '.PARAM.';
MSG_LIST(nn).LABELS = {'VEL_GAIN_EST','VEL_TIME_CONSTANT_EST','FILT_TIME_CONSTANT_EST',...
                       'FV_PI_STR_K','FV_PI_STR_Ti','FV_PI_STR_b','ADPT_FLAG',...
                       'FORWARD_VEL_PID.K','FORWARD_VEL_PID.Ti','FORWARD_VEL_PID.b',...
                       'FILT_TIME_CONSTANT_ACT'};
%--------------------------------------------------------------
nn = nn+1; % MSG 144 - RPI TO PC (44 x SINGLE = 176 BYTES)
MSG_LIST(nn).TYPE = '.INPUT.';
MSG_LIST(nn).LABELS = {'RANGE_FRONT_DIST','RANGE_REAR_DIST',...
                       'RANGE_WALL_DIST','LIDAR_2D_WALL_DIST','WALL_DIST_FILT','WALL_DIST_KF',...
                       'RANGE_WALL_ANG','LIDAR_2D_WALL_ANG','WALL_ANG_FILT','WALL_ANG_KF',...
                       'MCS_EULER_ANG','MCS_EARTH_POS','NAV_EULER_ANG','NAV_EARTH_POS',...
                       'NAV_EARTH_POS_FILT','NAV_EARTH_VEL_FILT','MOTOR_RATE','MOTOR_RATE_CD',...
                       'WALL_DIST','FORWARD_VEL','YAW_RATE','PITCH_RATE',...
                       'YAW_ANG','PITCH_ANG','EARTH_POS','EARTH_VEL'};

%--------------------------------------------------------------
NUM_MSG = nn;
%--------------------------------------------------------------
% MESSAGE IDs LIST
%--------------------------------------------------------------
if UART_MODE == 1
    MSG.MSG_ID_LIST = [uint8([  3  4  5  6  7  8  9  0]'),...
                       uint8([ 10 11 12 13 14  0  0  0]')];
    for ii=15:5:134
        MSG.MSG_ID_LIST = [MSG.MSG_ID_LIST uint8([ii ii+1 ii+2 ii+3 ii+4 0 0 0]')];
    end
    MSG.MSG_ID_LIST = [MSG.MSG_ID_LIST uint8([135 136 137 138 0 0 0 0]')];
else
    MSG.MSG_ID_LIST = [uint8([2  3  4  5  6  7  8  9]'),...
                       uint8([2 10 11 12 13 14  0  0]')];
    for ii=15:5:134
        MSG.MSG_ID_LIST = [MSG.MSG_ID_LIST uint8([2 ii ii+1 ii+2 ii+3 ii+4 0 0]')];
    end
    MSG.MSG_ID_LIST = [MSG.MSG_ID_LIST uint8([2 135 136 137 138 0 0 0]')];
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
                if FIELD(end)==')'
                    command = sprintf('DATA = [ DATA ; typecast(single(%s),''uint8'')''];\n',FIELD);
                else
                    command = sprintf('DATA = [ DATA ; typecast(single(%s(:)),''uint8'')''];\n',FIELD);
                end
            else
                if FIELD(end)==')'
                    command = sprintf('DATA = [ DATA ; typecast(single(%s),''uint8'')];\n',FIELD);
                else
                    command = sprintf('DATA = [ DATA ; typecast(single(%s(:)),''uint8'')];\n',FIELD);
                end
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
MSG.RX_BUFFER = zeros(1400,1,'uint8');
MSG.TX_BUFFER = zeros(1400,1,'uint8');
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
command = sprintf('TX_ID = MSG.TX_ID(MSG.TX_ID>0);\n');
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
command = sprintf('MSG.TX_BUFFER = zeros(1400,1,''uint8'');\n');
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
command = sprintf('RX_ID = HEADER(2:2:end);\n');
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