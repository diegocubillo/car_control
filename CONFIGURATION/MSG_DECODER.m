function [MSG,CONTROL] = MSG_DECODER(MSG_IN,CONTROL_IN)
%--------------------------------------------------------------
MSG = MSG_IN;
CONTROL = CONTROL_IN;
%--------------------------------------------------------------
BUFFER = MSG.RX_BUFFER;
NUM_MSG = double(BUFFER(1));
HEADER = BUFFER(1:2*NUM_MSG+1);
RX_ID = HEADER(2:2:end);
MSG_LEN = double(HEADER(3:2:end));
DATA_LEN = sum(MSG_LEN);
DATA = BUFFER(2*NUM_MSG+2:2*NUM_MSG+2+DATA_LEN-1);
nn = uint8(0);
DataOffset = 0;
%--------------------------------------------------------------
if ~all(RX_ID ~= 1)
CONTROL.STATE.CURRENT_STATUS_SYS = reshape(DATA(DataOffset+1:DataOffset+1),1,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 2)
CONTROL.STATE.CURRENT_STATUS_PC = reshape(DATA(DataOffset+1:DataOffset+1),1,1);
CONTROL.STATE.PC_BUTTONS = reshape(DATA(DataOffset+2:DataOffset+4),3,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 3)
CONTROL.STATE.CONTROL_MODE = reshape(DATA(DataOffset+1:DataOffset+1),1,1);
CONTROL.STATE.VEHICLE_MODE = reshape(DATA(DataOffset+2:DataOffset+2),1,1);
CONTROL.STATE.COMM_MODE = reshape(DATA(DataOffset+3:DataOffset+3),1,1);
CONTROL.STATE.FORWARD_VEL_CONTROL_TYPE = reshape(DATA(DataOffset+4:DataOffset+4),1,1);
CONTROL.STATE.YAW_RATE_CONTROL_TYPE = reshape(DATA(DataOffset+5:DataOffset+5),1,1);
CONTROL.STATE.YAW_ANG_CONTROL_TYPE = reshape(DATA(DataOffset+6:DataOffset+6),1,1);
CONTROL.STATE.WFL_CONTROL_TYPE = reshape(DATA(DataOffset+7:DataOffset+7),1,1);
CONTROL.STATE.NAV_CONTROL_TYPE = reshape(DATA(DataOffset+8:DataOffset+8),1,1);
CONTROL.STATE.PITCH_ANG_CONTROL_TYPE = reshape(DATA(DataOffset+9:DataOffset+9),1,1);
CONTROL.STATE.WFL_FEEDFORWARD = reshape(DATA(DataOffset+10:DataOffset+10),1,1);
CONTROL.STATE.GAIN_SCHEDULING = reshape(DATA(DataOffset+11:DataOffset+11),1,1);
CONTROL.STATE.MOTOR_DELAY_MODE = reshape(DATA(DataOffset+12:DataOffset+12),1,1);
CONTROL.STATE.OBSERVER_MODE = reshape(DATA(DataOffset+13:DataOffset+13),1,1);
CONTROL.STATE.FORWARD_VEL_MAIN_OP = reshape(DATA(DataOffset+14:DataOffset+14),1,1);
CONTROL.STATE.ROTATION_MSRT_MODE = reshape(DATA(DataOffset+15:DataOffset+15),1,1);
CONTROL.STATE.MCS_MODE = reshape(DATA(DataOffset+16:DataOffset+16),1,1);
CONTROL.STATE.NAV_MODE = reshape(DATA(DataOffset+17:DataOffset+17),1,1);
CONTROL.STATE.YA_TARGET_SOURCE = reshape(DATA(DataOffset+18:DataOffset+18),1,1);
CONTROL.STATE.FV_TARGET_SOURCE = reshape(DATA(DataOffset+19:DataOffset+19),1,1);
CONTROL.STATE.YA_TARGET_TYPE = reshape(DATA(DataOffset+20:DataOffset+20),1,1);
CONTROL.STATE.YR_TARGET_SOURCE = reshape(DATA(DataOffset+21:DataOffset+21),1,1);
CONTROL.STATE.WD_TARGET_SOURCE = reshape(DATA(DataOffset+22:DataOffset+22),1,1);
CONTROL.STATE.NAV_TARGET_SOURCE = reshape(DATA(DataOffset+23:DataOffset+23),1,1);
CONTROL.STATE.YR_TARGET_TYPE = reshape(DATA(DataOffset+24:DataOffset+24),1,1);
CONTROL.STATE.FV_TARGET_TYPE = reshape(DATA(DataOffset+25:DataOffset+25),1,1);
CONTROL.STATE.WD_TARGET_TYPE = reshape(DATA(DataOffset+26:DataOffset+26),1,1);
CONTROL.STATE.FILTER_CHANGE = reshape(DATA(DataOffset+27:DataOffset+27),1,1);
CONTROL.STATE.STR_MODE = reshape(DATA(DataOffset+28:DataOffset+28),1,1);
CONTROL.STATE.WALL_FOLLOWER_MODE = reshape(DATA(DataOffset+29:DataOffset+29),1,1);
CONTROL.STATE.SP_MODE = reshape(DATA(DataOffset+30:DataOffset+31),1,2);
CONTROL.STATE.MV_TARGET_SOURCE = reshape(DATA(DataOffset+32:DataOffset+33),2,1);
CONTROL.STATE.MV_TARGET_TYPE = reshape(DATA(DataOffset+34:DataOffset+35),2,1);
CONTROL.STATE.NAV_TARGET_TYPE = reshape(DATA(DataOffset+36:DataOffset+37),1,2);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 4)
CONTROL.PARAM.OP_INPUT = reshape(double(typecast(DATA(DataOffset+1:DataOffset+32),'single')),2,4);
CONTROL.PARAM.OP_STATE = reshape(double(typecast(DATA(DataOffset+33:DataOffset+128),'single')),6,4);
CONTROL.PARAM.OP_OUTPUT = reshape(double(typecast(DATA(DataOffset+129:DataOffset+224),'single')),6,4);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 5)
CONTROL.PARAM.OPL_RC_SCALE = reshape(double(typecast(DATA(DataOffset+1:DataOffset+8),'single')),2,1);
CONTROL.PARAM.VEL_RC_SCALE = reshape(double(typecast(DATA(DataOffset+9:DataOffset+16),'single')),1,2);
CONTROL.PARAM.WFL_RC_SCALE = reshape(double(typecast(DATA(DataOffset+17:DataOffset+24),'single')),1,2);
CONTROL.PARAM.NAV_RC_SCALE = reshape(double(typecast(DATA(DataOffset+25:DataOffset+28),'single')),1,1);
CONTROL.PARAM.OPL_RC_SWITCH = reshape(double(typecast(DATA(DataOffset+29:DataOffset+52),'single')),2,3);
CONTROL.PARAM.VEL_RC_SWITCH = reshape(double(typecast(DATA(DataOffset+53:DataOffset+76),'single')),2,3);
CONTROL.PARAM.WFL_RC_SWITCH = reshape(double(typecast(DATA(DataOffset+77:DataOffset+100),'single')),2,3);
CONTROL.PARAM.NAV_RC_SWITCH = reshape(double(typecast(DATA(DataOffset+101:DataOffset+124),'single')),2,3);
CONTROL.PARAM.FORWARD_VEL_DB.NUM = reshape(double(typecast(DATA(DataOffset+125:DataOffset+136),'single')),1,3);
CONTROL.PARAM.FORWARD_VEL_DB.DEN = reshape(double(typecast(DATA(DataOffset+137:DataOffset+144),'single')),1,2);
CONTROL.PARAM.YAW_RATE_DB.NUM = reshape(double(typecast(DATA(DataOffset+145:DataOffset+156),'single')),1,3);
CONTROL.PARAM.YAW_RATE_DB.DEN = reshape(double(typecast(DATA(DataOffset+157:DataOffset+164),'single')),1,2);
CONTROL.PARAM.NAV_SFC.INT_DISC_TYPE = reshape(DATA(DataOffset+165:DataOffset+165),1,1);
CONTROL.PARAM.NAV_SFC.ANTIWINDUP = reshape(DATA(DataOffset+166:DataOffset+166),1,1);
CONTROL.PARAM.NAV_STOP_RADIUS = reshape(double(typecast(DATA(DataOffset+167:DataOffset+170),'single')),1,1);
CONTROL.PARAM.NAV_MAX_TARGET_RADIUS = reshape(double(typecast(DATA(DataOffset+171:DataOffset+174),'single')),1,1);
CONTROL.PARAM.NAV_SFC.COS_ANG = reshape(double(typecast(DATA(DataOffset+175:DataOffset+194),'single')),1,5);
CONTROL.PARAM.NAV_SFC.FV = reshape(double(typecast(DATA(DataOffset+195:DataOffset+214),'single')),1,5);
CONTROL.PARAM.NAV_SFC.INV_DIST = reshape(double(typecast(DATA(DataOffset+215:DataOffset+234),'single')),1,5);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 6)
CONTROL.PARAM.FORWARD_VEL_PID.K = reshape(double(typecast(DATA(DataOffset+1:DataOffset+4),'single')),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.Ti = reshape(double(typecast(DATA(DataOffset+5:DataOffset+8),'single')),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.Td = reshape(double(typecast(DATA(DataOffset+9:DataOffset+12),'single')),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.N = reshape(double(typecast(DATA(DataOffset+13:DataOffset+16),'single')),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.b = reshape(double(typecast(DATA(DataOffset+17:DataOffset+20),'single')),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE = reshape(DATA(DataOffset+21:DataOffset+21),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE = reshape(DATA(DataOffset+22:DataOffset+22),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.DER_INPUT = reshape(DATA(DataOffset+23:DataOffset+23),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.ANTIWINDUP = reshape(DATA(DataOffset+24:DataOffset+24),1,1);
CONTROL.PARAM.YAW_RATE_PID.K = reshape(double(typecast(DATA(DataOffset+25:DataOffset+28),'single')),1,1);
CONTROL.PARAM.YAW_RATE_PID.Ti = reshape(double(typecast(DATA(DataOffset+29:DataOffset+32),'single')),1,1);
CONTROL.PARAM.YAW_RATE_PID.Td = reshape(double(typecast(DATA(DataOffset+33:DataOffset+36),'single')),1,1);
CONTROL.PARAM.YAW_RATE_PID.N = reshape(double(typecast(DATA(DataOffset+37:DataOffset+40),'single')),1,1);
CONTROL.PARAM.YAW_RATE_PID.b = reshape(double(typecast(DATA(DataOffset+41:DataOffset+44),'single')),1,1);
CONTROL.PARAM.YAW_RATE_PID.INT_DISC_TYPE = reshape(DATA(DataOffset+45:DataOffset+45),1,1);
CONTROL.PARAM.YAW_RATE_PID.DER_DISC_TYPE = reshape(DATA(DataOffset+46:DataOffset+46),1,1);
CONTROL.PARAM.YAW_RATE_PID.DER_INPUT = reshape(DATA(DataOffset+47:DataOffset+47),1,1);
CONTROL.PARAM.YAW_RATE_PID.ANTIWINDUP = reshape(DATA(DataOffset+48:DataOffset+48),1,1);
CONTROL.PARAM.YAW_ANG_PID.K = reshape(double(typecast(DATA(DataOffset+49:DataOffset+52),'single')),1,1);
CONTROL.PARAM.YAW_ANG_PID.Ti = reshape(double(typecast(DATA(DataOffset+53:DataOffset+56),'single')),1,1);
CONTROL.PARAM.YAW_ANG_PID.Td = reshape(double(typecast(DATA(DataOffset+57:DataOffset+60),'single')),1,1);
CONTROL.PARAM.YAW_ANG_PID.N = reshape(double(typecast(DATA(DataOffset+61:DataOffset+64),'single')),1,1);
CONTROL.PARAM.YAW_ANG_PID.b = reshape(double(typecast(DATA(DataOffset+65:DataOffset+68),'single')),1,1);
CONTROL.PARAM.YAW_ANG_PID.INT_DISC_TYPE = reshape(DATA(DataOffset+69:DataOffset+69),1,1);
CONTROL.PARAM.YAW_ANG_PID.DER_DISC_TYPE = reshape(DATA(DataOffset+70:DataOffset+70),1,1);
CONTROL.PARAM.YAW_ANG_PID.DER_INPUT = reshape(DATA(DataOffset+71:DataOffset+71),1,1);
CONTROL.PARAM.YAW_ANG_PID.ANTIWINDUP = reshape(DATA(DataOffset+72:DataOffset+72),1,1);
CONTROL.PARAM.WFL_SL_PID.K = reshape(double(typecast(DATA(DataOffset+73:DataOffset+88),'single')),4,1);
CONTROL.PARAM.WFL_SL_PID.Ti = reshape(double(typecast(DATA(DataOffset+89:DataOffset+104),'single')),4,1);
CONTROL.PARAM.WFL_SL_PID.Td = reshape(double(typecast(DATA(DataOffset+105:DataOffset+120),'single')),4,1);
CONTROL.PARAM.WFL_SL_PID.N = reshape(double(typecast(DATA(DataOffset+121:DataOffset+136),'single')),4,1);
CONTROL.PARAM.WFL_SL_PID.b = reshape(double(typecast(DATA(DataOffset+137:DataOffset+152),'single')),4,1);
CONTROL.PARAM.WFL_SL_PID.INT_DISC_TYPE = reshape(DATA(DataOffset+153:DataOffset+153),1,1);
CONTROL.PARAM.WFL_SL_PID.DER_DISC_TYPE = reshape(DATA(DataOffset+154:DataOffset+154),1,1);
CONTROL.PARAM.WFL_SL_PID.DER_INPUT = reshape(DATA(DataOffset+155:DataOffset+155),1,1);
CONTROL.PARAM.WFL_SL_PID.ANTIWINDUP = reshape(DATA(DataOffset+156:DataOffset+156),1,1);
CONTROL.PARAM.WFL_CD_PID.K = reshape(double(typecast(DATA(DataOffset+157:DataOffset+172),'single')),4,1);
CONTROL.PARAM.WFL_CD_PID.Ti = reshape(double(typecast(DATA(DataOffset+173:DataOffset+188),'single')),4,1);
CONTROL.PARAM.WFL_CD_PID.Td = reshape(double(typecast(DATA(DataOffset+189:DataOffset+204),'single')),4,1);
CONTROL.PARAM.WFL_CD_PID.N = reshape(double(typecast(DATA(DataOffset+205:DataOffset+220),'single')),4,1);
CONTROL.PARAM.WFL_CD_PID.b = reshape(double(typecast(DATA(DataOffset+221:DataOffset+236),'single')),4,1);
CONTROL.PARAM.WFL_CD_PID.INT_DISC_TYPE = reshape(DATA(DataOffset+237:DataOffset+237),1,1);
CONTROL.PARAM.WFL_CD_PID.DER_DISC_TYPE = reshape(DATA(DataOffset+238:DataOffset+238),1,1);
CONTROL.PARAM.WFL_CD_PID.DER_INPUT = reshape(DATA(DataOffset+239:DataOffset+239),1,1);
CONTROL.PARAM.WFL_CD_PID.ANTIWINDUP = reshape(DATA(DataOffset+240:DataOffset+240),1,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 7)
CONTROL.PARAM.FORWARD_VEL_SFC.K = reshape(double(typecast(DATA(DataOffset+1:DataOffset+12),'single')),1,3);
CONTROL.PARAM.FORWARD_VEL_SFC.Ki = reshape(double(typecast(DATA(DataOffset+13:DataOffset+16),'single')),1,1);
CONTROL.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE = reshape(DATA(DataOffset+17:DataOffset+17),1,1);
CONTROL.PARAM.FORWARD_VEL_SFC.ANTIWINDUP = reshape(DATA(DataOffset+18:DataOffset+18),1,1);
CONTROL.PARAM.YAW_RATE_SFC.K = reshape(double(typecast(DATA(DataOffset+19:DataOffset+22),'single')),1,1);
CONTROL.PARAM.YAW_RATE_SFC.Ki = reshape(double(typecast(DATA(DataOffset+23:DataOffset+26),'single')),1,1);
CONTROL.PARAM.YAW_RATE_SFC.INT_DISC_TYPE = reshape(DATA(DataOffset+27:DataOffset+27),1,1);
CONTROL.PARAM.YAW_RATE_SFC.ANTIWINDUP = reshape(DATA(DataOffset+28:DataOffset+28),1,1);
CONTROL.PARAM.YAW_ANG_SFC.K = reshape(double(typecast(DATA(DataOffset+29:DataOffset+36),'single')),1,2);
CONTROL.PARAM.YAW_ANG_SFC.Ki = reshape(double(typecast(DATA(DataOffset+37:DataOffset+40),'single')),1,1);
CONTROL.PARAM.YAW_ANG_SFC.INT_DISC_TYPE = reshape(DATA(DataOffset+41:DataOffset+41),1,1);
CONTROL.PARAM.YAW_ANG_SFC.ANTIWINDUP = reshape(DATA(DataOffset+42:DataOffset+42),1,1);
CONTROL.PARAM.WFL_SFC.K = reshape(double(typecast(DATA(DataOffset+43:DataOffset+90),'single')),4,3);
CONTROL.PARAM.PITCH_ANG_SFC.K = reshape(double(typecast(DATA(DataOffset+91:DataOffset+102),'single')),1,3);
CONTROL.PARAM.CONTROL_SAMPLING_TIME = reshape(double(typecast(DATA(DataOffset+103:DataOffset+106),'single')),1,1);
CONTROL.PARAM.IMU_FILT_FREQ = reshape(double(typecast(DATA(DataOffset+107:DataOffset+110),'single')),1,1);
CONTROL.PARAM.ENC_FILT_FREQ = reshape(double(typecast(DATA(DataOffset+111:DataOffset+114),'single')),1,1);
CONTROL.PARAM.EKF_FILT_FREQ = reshape(double(typecast(DATA(DataOffset+115:DataOffset+118),'single')),1,1);
CONTROL.PARAM.WFL_FILT_FREQ = reshape(double(typecast(DATA(DataOffset+119:DataOffset+122),'single')),1,1);
CONTROL.PARAM.IMU_CF_FREQ = reshape(double(typecast(DATA(DataOffset+123:DataOffset+126),'single')),1,1);
CONTROL.PARAM.VEL_GAIN = reshape(double(typecast(DATA(DataOffset+127:DataOffset+130),'single')),1,1);
CONTROL.PARAM.VEL_TIME_CONSTANT = reshape(double(typecast(DATA(DataOffset+131:DataOffset+134),'single')),1,1);
CONTROL.PARAM.YAW_RATE_GAIN = reshape(double(typecast(DATA(DataOffset+135:DataOffset+138),'single')),1,1);
CONTROL.PARAM.YAW_RATE_TIME_CONSTANT = reshape(double(typecast(DATA(DataOffset+139:DataOffset+142),'single')),1,1);
CONTROL.PARAM.MOTOR_DELAY = reshape(double(typecast(DATA(DataOffset+143:DataOffset+146),'single')),1,1);
CONTROL.PARAM.MOTOR_DELAY_ERR = reshape(double(typecast(DATA(DataOffset+147:DataOffset+150),'single')),1,1);
CONTROL.PARAM.MOTOR_VOLT_DROP_DIF = reshape(double(typecast(DATA(DataOffset+151:DataOffset+154),'single')),1,1);
CONTROL.PARAM.MCS_ALFA_CALIB = reshape(double(typecast(DATA(DataOffset+155:DataOffset+158),'single')),1,1);
CONTROL.PARAM.NAV_ALFA_CALIB = reshape(double(typecast(DATA(DataOffset+159:DataOffset+162),'single')),1,1);
CONTROL.PARAM.MOTOR_VOLT_MAX = reshape(double(typecast(DATA(DataOffset+163:DataOffset+170),'single')),2,1);
CONTROL.PARAM.MOTOR_VOLT_MIN = reshape(double(typecast(DATA(DataOffset+171:DataOffset+178),'single')),2,1);
CONTROL.PARAM.MOTOR_DEAD_ZONE = reshape(double(typecast(DATA(DataOffset+179:DataOffset+186),'single')),2,1);
CONTROL.PARAM.MOTOR_PWM_SLOPE = reshape(double(typecast(DATA(DataOffset+187:DataOffset+194),'single')),2,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 8)
CONTROL.PARAM.ENC_FORWARD_VEL_matAd = reshape(double(typecast(DATA(DataOffset+1:DataOffset+36),'single')),3,3);
CONTROL.PARAM.ENC_FORWARD_VEL_matBd = reshape(double(typecast(DATA(DataOffset+37:DataOffset+48),'single')),3,1);
CONTROL.PARAM.ENC_FORWARD_VEL_matCd = reshape(double(typecast(DATA(DataOffset+49:DataOffset+72),'single')),2,3);
CONTROL.PARAM.ENC_FORWARD_VEL_matQ = reshape(double(typecast(DATA(DataOffset+73:DataOffset+84),'single')),1,3);
CONTROL.PARAM.ENC_FORWARD_VEL_matR = reshape(double(typecast(DATA(DataOffset+85:DataOffset+92),'single')),1,2);
CONTROL.PARAM.ENC_YAW_RATE_matAd = reshape(double(typecast(DATA(DataOffset+93:DataOffset+156),'single')),4,4);
CONTROL.PARAM.ENC_YAW_RATE_matBd = reshape(double(typecast(DATA(DataOffset+157:DataOffset+172),'single')),4,1);
CONTROL.PARAM.ENC_YAW_RATE_matCd = reshape(double(typecast(DATA(DataOffset+173:DataOffset+204),'single')),2,4);
CONTROL.PARAM.ENC_YAW_RATE_matQ = reshape(double(typecast(DATA(DataOffset+205:DataOffset+220),'single')),1,4);
CONTROL.PARAM.ENC_YAW_RATE_matR = reshape(double(typecast(DATA(DataOffset+221:DataOffset+228),'single')),1,2);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 9)
CONTROL.PARAM.MISSION_NUM_WAYPOINTS = reshape(double(typecast(DATA(DataOffset+1:DataOffset+4),'single')),1,1);
CONTROL.PARAM.MISSION_WAYPOINTS = reshape(double(typecast(DATA(DataOffset+5:DataOffset+164),'single')),10,4);
CONTROL.PARAM.MISSION_WP_RADIUS = reshape(double(typecast(DATA(DataOffset+165:DataOffset+168),'single')),1,1);
CONTROL.PARAM.MISSION_YA_ERROR = reshape(double(typecast(DATA(DataOffset+169:DataOffset+172),'single')),1,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 10)
CONTROL.EKF_WFL.PARAM.RANGE_XA = reshape(double(typecast(DATA(DataOffset+1:DataOffset+4),'single')),1,1);
CONTROL.EKF_WFL.PARAM.RANGE_YA = reshape(double(typecast(DATA(DataOffset+5:DataOffset+8),'single')),1,1);
CONTROL.EKF_WFL.PARAM.LIDAR_2D_XA = reshape(double(typecast(DATA(DataOffset+9:DataOffset+12),'single')),1,1);
CONTROL.EKF_WFL.PARAM.LIDAR_2D_YA = reshape(double(typecast(DATA(DataOffset+13:DataOffset+16),'single')),1,1);
CONTROL.EKF_WFL.PARAM.LIDAR_2D_WFL_ANG_OFFS = reshape(double(typecast(DATA(DataOffset+17:DataOffset+20),'single')),1,1);
CONTROL.EKF_WFL.PARAM.LIDAR_2D_WFL_ANG_SIGN = reshape(double(typecast(DATA(DataOffset+21:DataOffset+24),'single')),1,1);
CONTROL.EKF_WFL.PROCESS_NOISE_VAR = reshape(double(typecast(DATA(DataOffset+25:DataOffset+36),'single')),3,1);
CONTROL.EKF_WFL.OBSRV_NOISE_VAR = reshape(double(typecast(DATA(DataOffset+37:DataOffset+44),'single')),2,1);
CONTROL.EKF_NAV.PROCESS_NOISE_VAR = reshape(double(typecast(DATA(DataOffset+45:DataOffset+56),'single')),3,1);
CONTROL.EKF_NAV.OBSRV_NOISE_VAR = reshape(double(typecast(DATA(DataOffset+57:DataOffset+68),'single')),3,1);
CONTROL.EKF_IMU.PARAM.ACCEL_ADPTV_GAIN = reshape(double(typecast(DATA(DataOffset+69:DataOffset+72),'single')),1,1);
CONTROL.EKF_IMU.PROCESS_NOISE_VAR = reshape(double(typecast(DATA(DataOffset+73:DataOffset+96),'single')),6,1);
CONTROL.EKF_IMU.OBSRV_NOISE_VAR = reshape(double(typecast(DATA(DataOffset+97:DataOffset+124),'single')),7,1);
CONTROL.PARAM.SVF_Ad = reshape(double(typecast(DATA(DataOffset+125:DataOffset+140),'single')),2,2);
CONTROL.PARAM.SVF_B1d = reshape(double(typecast(DATA(DataOffset+141:DataOffset+148),'single')),2,1);
CONTROL.PARAM.SVF_B2d = reshape(double(typecast(DATA(DataOffset+149:DataOffset+156),'single')),2,1);
CONTROL.PARAM.SVF_Cd = reshape(double(typecast(DATA(DataOffset+157:DataOffset+180),'single')),3,2);
CONTROL.PARAM.SVF_Dd = reshape(double(typecast(DATA(DataOffset+181:DataOffset+192),'single')),3,1);
CONTROL.PARAM.CONTROL_ACT_DELAY = reshape(double(typecast(DATA(DataOffset+193:DataOffset+196),'single')),1,1);
CONTROL.PARAM.PA_INITIAL_VALUE = reshape(double(typecast(DATA(DataOffset+197:DataOffset+200),'single')),1,1);
CONTROL.PARAM.FV_TARGET_VALUE = reshape(DATA(DataOffset+201:DataOffset+201),1,1);
CONTROL.PARAM.NAV_TARGET_VALUE = reshape(double(typecast(DATA(DataOffset+202:DataOffset+209),'single')),1,2);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 11)
CONTROL.PARAM.NAV_SFC.KPP(1:63,1) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 12)
CONTROL.PARAM.NAV_SFC.KPP(64:125,1) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 13)
CONTROL.PARAM.NAV_SFC.KPP(1:63,2) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 14)
CONTROL.PARAM.NAV_SFC.KPP(64:125,2) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 15)
CONTROL.PARAM.NAV_SFC.KPP(1:63,3) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 16)
CONTROL.PARAM.NAV_SFC.KPP(64:125,3) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 17)
CONTROL.PARAM.NAV_SFC.KPP(1:63,4) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 18)
CONTROL.PARAM.NAV_SFC.KPP(64:125,4) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 19)
CONTROL.PARAM.NAV_SFC.KPP(1:63,5) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 20)
CONTROL.PARAM.NAV_SFC.KPP(64:125,5) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 21)
CONTROL.PARAM.NAV_SFC.KPP(1:63,6) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 22)
CONTROL.PARAM.NAV_SFC.KPP(64:125,6) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 23)
CONTROL.PARAM.NAV_SFC.KPP(1:63,7) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 24)
CONTROL.PARAM.NAV_SFC.KPP(64:125,7) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 25)
CONTROL.PARAM.NAV_SFC.KPP(1:63,8) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 26)
CONTROL.PARAM.NAV_SFC.KPP(64:125,8) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 27)
CONTROL.PARAM.NAV_SFC.KPP(1:63,9) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 28)
CONTROL.PARAM.NAV_SFC.KPP(64:125,9) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 29)
CONTROL.PARAM.NAV_SFC.KPP(1:63,10) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 30)
CONTROL.PARAM.NAV_SFC.KPP(64:125,10) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 31)
CONTROL.PARAM.NAV_SFC.KPP(1:63,11) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 32)
CONTROL.PARAM.NAV_SFC.KPP(64:125,11) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 33)
CONTROL.PARAM.NAV_SFC.KPP(1:63,12) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 34)
CONTROL.PARAM.NAV_SFC.KPP(64:125,12) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 35)
CONTROL.PARAM.NAV_SFC.KPP(1:63,13) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 36)
CONTROL.PARAM.NAV_SFC.KPP(64:125,13) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 37)
CONTROL.PARAM.NAV_SFC.KPP(1:63,14) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 38)
CONTROL.PARAM.NAV_SFC.KPP(64:125,14) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 39)
CONTROL.PARAM.NAV_SFC.KPP(1:63,15) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 40)
CONTROL.PARAM.NAV_SFC.KPP(64:125,15) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 41)
CONTROL.PARAM.NAV_SFC.KPP(1:63,16) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 42)
CONTROL.PARAM.NAV_SFC.KPP(64:125,16) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 43)
CONTROL.PARAM.NAV_SFC.KPN(1:63,1) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 44)
CONTROL.PARAM.NAV_SFC.KPN(64:125,1) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 45)
CONTROL.PARAM.NAV_SFC.KPN(1:63,2) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 46)
CONTROL.PARAM.NAV_SFC.KPN(64:125,2) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 47)
CONTROL.PARAM.NAV_SFC.KPN(1:63,3) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 48)
CONTROL.PARAM.NAV_SFC.KPN(64:125,3) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 49)
CONTROL.PARAM.NAV_SFC.KPN(1:63,4) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 50)
CONTROL.PARAM.NAV_SFC.KPN(64:125,4) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 51)
CONTROL.PARAM.NAV_SFC.KPN(1:63,5) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 52)
CONTROL.PARAM.NAV_SFC.KPN(64:125,5) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 53)
CONTROL.PARAM.NAV_SFC.KPN(1:63,6) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 54)
CONTROL.PARAM.NAV_SFC.KPN(64:125,6) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 55)
CONTROL.PARAM.NAV_SFC.KPN(1:63,7) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 56)
CONTROL.PARAM.NAV_SFC.KPN(64:125,7) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 57)
CONTROL.PARAM.NAV_SFC.KPN(1:63,8) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 58)
CONTROL.PARAM.NAV_SFC.KPN(64:125,8) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 59)
CONTROL.PARAM.NAV_SFC.KPN(1:63,9) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 60)
CONTROL.PARAM.NAV_SFC.KPN(64:125,9) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 61)
CONTROL.PARAM.NAV_SFC.KPN(1:63,10) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 62)
CONTROL.PARAM.NAV_SFC.KPN(64:125,10) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 63)
CONTROL.PARAM.NAV_SFC.KPN(1:63,11) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 64)
CONTROL.PARAM.NAV_SFC.KPN(64:125,11) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 65)
CONTROL.PARAM.NAV_SFC.KPN(1:63,12) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 66)
CONTROL.PARAM.NAV_SFC.KPN(64:125,12) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 67)
CONTROL.PARAM.NAV_SFC.KPN(1:63,13) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 68)
CONTROL.PARAM.NAV_SFC.KPN(64:125,13) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 69)
CONTROL.PARAM.NAV_SFC.KPN(1:63,14) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 70)
CONTROL.PARAM.NAV_SFC.KPN(64:125,14) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 71)
CONTROL.PARAM.NAV_SFC.KPN(1:63,15) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 72)
CONTROL.PARAM.NAV_SFC.KPN(64:125,15) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 73)
CONTROL.PARAM.NAV_SFC.KPN(1:63,16) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 74)
CONTROL.PARAM.NAV_SFC.KPN(64:125,16) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 75)
CONTROL.PARAM.NAV_SFC.KNP(1:63,1) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 76)
CONTROL.PARAM.NAV_SFC.KNP(64:125,1) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 77)
CONTROL.PARAM.NAV_SFC.KNP(1:63,2) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 78)
CONTROL.PARAM.NAV_SFC.KNP(64:125,2) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 79)
CONTROL.PARAM.NAV_SFC.KNP(1:63,3) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 80)
CONTROL.PARAM.NAV_SFC.KNP(64:125,3) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 81)
CONTROL.PARAM.NAV_SFC.KNP(1:63,4) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 82)
CONTROL.PARAM.NAV_SFC.KNP(64:125,4) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 83)
CONTROL.PARAM.NAV_SFC.KNP(1:63,5) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 84)
CONTROL.PARAM.NAV_SFC.KNP(64:125,5) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 85)
CONTROL.PARAM.NAV_SFC.KNP(1:63,6) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 86)
CONTROL.PARAM.NAV_SFC.KNP(64:125,6) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 87)
CONTROL.PARAM.NAV_SFC.KNP(1:63,7) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 88)
CONTROL.PARAM.NAV_SFC.KNP(64:125,7) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 89)
CONTROL.PARAM.NAV_SFC.KNP(1:63,8) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 90)
CONTROL.PARAM.NAV_SFC.KNP(64:125,8) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 91)
CONTROL.PARAM.NAV_SFC.KNP(1:63,9) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 92)
CONTROL.PARAM.NAV_SFC.KNP(64:125,9) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 93)
CONTROL.PARAM.NAV_SFC.KNP(1:63,10) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 94)
CONTROL.PARAM.NAV_SFC.KNP(64:125,10) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 95)
CONTROL.PARAM.NAV_SFC.KNP(1:63,11) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 96)
CONTROL.PARAM.NAV_SFC.KNP(64:125,11) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 97)
CONTROL.PARAM.NAV_SFC.KNP(1:63,12) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 98)
CONTROL.PARAM.NAV_SFC.KNP(64:125,12) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 99)
CONTROL.PARAM.NAV_SFC.KNP(1:63,13) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 100)
CONTROL.PARAM.NAV_SFC.KNP(64:125,13) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 101)
CONTROL.PARAM.NAV_SFC.KNP(1:63,14) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 102)
CONTROL.PARAM.NAV_SFC.KNP(64:125,14) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 103)
CONTROL.PARAM.NAV_SFC.KNP(1:63,15) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 104)
CONTROL.PARAM.NAV_SFC.KNP(64:125,15) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 105)
CONTROL.PARAM.NAV_SFC.KNP(1:63,16) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 106)
CONTROL.PARAM.NAV_SFC.KNP(64:125,16) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 107)
CONTROL.PARAM.NAV_SFC.KNN(1:63,1) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 108)
CONTROL.PARAM.NAV_SFC.KNN(64:125,1) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 109)
CONTROL.PARAM.NAV_SFC.KNN(1:63,2) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 110)
CONTROL.PARAM.NAV_SFC.KNN(64:125,2) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 111)
CONTROL.PARAM.NAV_SFC.KNN(1:63,3) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 112)
CONTROL.PARAM.NAV_SFC.KNN(64:125,3) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 113)
CONTROL.PARAM.NAV_SFC.KNN(1:63,4) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 114)
CONTROL.PARAM.NAV_SFC.KNN(64:125,4) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 115)
CONTROL.PARAM.NAV_SFC.KNN(1:63,5) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 116)
CONTROL.PARAM.NAV_SFC.KNN(64:125,5) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 117)
CONTROL.PARAM.NAV_SFC.KNN(1:63,6) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 118)
CONTROL.PARAM.NAV_SFC.KNN(64:125,6) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 119)
CONTROL.PARAM.NAV_SFC.KNN(1:63,7) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 120)
CONTROL.PARAM.NAV_SFC.KNN(64:125,7) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 121)
CONTROL.PARAM.NAV_SFC.KNN(1:63,8) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 122)
CONTROL.PARAM.NAV_SFC.KNN(64:125,8) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 123)
CONTROL.PARAM.NAV_SFC.KNN(1:63,9) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 124)
CONTROL.PARAM.NAV_SFC.KNN(64:125,9) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 125)
CONTROL.PARAM.NAV_SFC.KNN(1:63,10) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 126)
CONTROL.PARAM.NAV_SFC.KNN(64:125,10) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 127)
CONTROL.PARAM.NAV_SFC.KNN(1:63,11) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 128)
CONTROL.PARAM.NAV_SFC.KNN(64:125,11) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 129)
CONTROL.PARAM.NAV_SFC.KNN(1:63,12) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 130)
CONTROL.PARAM.NAV_SFC.KNN(64:125,12) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 131)
CONTROL.PARAM.NAV_SFC.KNN(1:63,13) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 132)
CONTROL.PARAM.NAV_SFC.KNN(64:125,13) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 133)
CONTROL.PARAM.NAV_SFC.KNN(1:63,14) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 134)
CONTROL.PARAM.NAV_SFC.KNN(64:125,14) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 135)
CONTROL.PARAM.NAV_SFC.KNN(1:63,15) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 136)
CONTROL.PARAM.NAV_SFC.KNN(64:125,15) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 137)
CONTROL.PARAM.NAV_SFC.KNN(1:63,16) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+252),'single')),63,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 138)
CONTROL.PARAM.NAV_SFC.KNN(64:125,16) = reshape(double(typecast(DATA(DataOffset+1:DataOffset+248),'single')),62,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 139)
CONTROL.INPUT.BATTERY_VOLT = reshape(double(typecast(DATA(DataOffset+1:DataOffset+4),'single')),1,1);
CONTROL.INPUT.CHARGING_CURRENT = reshape(double(typecast(DATA(DataOffset+5:DataOffset+8),'single')),1,1);
CONTROL.INPUT.IMU_ACCEL_MEAN = reshape(double(typecast(DATA(DataOffset+9:DataOffset+20),'single')),3,1);
CONTROL.INPUT.IMU_ACCEL_MEAN_FILT = reshape(double(typecast(DATA(DataOffset+21:DataOffset+32),'single')),3,1);
CONTROL.INPUT.IMU_GYRO_MEAN = reshape(double(typecast(DATA(DataOffset+33:DataOffset+44),'single')),3,1);
CONTROL.INPUT.IMU_GYRO_MEAN_FILT = reshape(double(typecast(DATA(DataOffset+45:DataOffset+56),'single')),3,1);
CONTROL.INPUT.IMU_EULER_RATE_CF = reshape(double(typecast(DATA(DataOffset+57:DataOffset+68),'single')),3,1);
CONTROL.INPUT.IMU_EULER_RATE_KF = reshape(double(typecast(DATA(DataOffset+69:DataOffset+80),'single')),3,1);
CONTROL.INPUT.IMU_EULER_ANG_CF = reshape(double(typecast(DATA(DataOffset+81:DataOffset+92),'single')),3,1);
CONTROL.INPUT.IMU_EULER_ANG_KF = reshape(double(typecast(DATA(DataOffset+93:DataOffset+104),'single')),3,1);
CONTROL.INPUT.IMU_PITCH_ANG_INI = reshape(double(typecast(DATA(DataOffset+105:DataOffset+108),'single')),1,1);
CONTROL.INPUT.IMU_FORWARD_ACCEL_CF = reshape(double(typecast(DATA(DataOffset+109:DataOffset+112),'single')),1,1);
CONTROL.INPUT.IMU_FORWARD_ACCEL_KF = reshape(double(typecast(DATA(DataOffset+113:DataOffset+116),'single')),1,1);
CONTROL.INPUT.ENC_MOTOR_RATE_LPF = reshape(double(typecast(DATA(DataOffset+117:DataOffset+124),'single')),2,1);
CONTROL.INPUT.ENC_MOTOR_RATE_KF = reshape(double(typecast(DATA(DataOffset+125:DataOffset+132),'single')),2,1);
CONTROL.INPUT.ENC_MOTOR_RATE_CD_LPF = reshape(double(typecast(DATA(DataOffset+133:DataOffset+140),'single')),2,1);
CONTROL.INPUT.ENC_MOTOR_RATE_CD_KF = reshape(double(typecast(DATA(DataOffset+141:DataOffset+148),'single')),2,1);
CONTROL.INPUT.ENC_FORWARD_ACCEL_LPF = reshape(double(typecast(DATA(DataOffset+149:DataOffset+152),'single')),1,1);
CONTROL.INPUT.ENC_FORWARD_ACCEL_KF = reshape(double(typecast(DATA(DataOffset+153:DataOffset+156),'single')),1,1);
CONTROL.INPUT.ENC_FORWARD_VEL_LPF = reshape(double(typecast(DATA(DataOffset+157:DataOffset+160),'single')),1,1);
CONTROL.INPUT.ENC_FORWARD_VEL_KF = reshape(double(typecast(DATA(DataOffset+161:DataOffset+164),'single')),1,1);
CONTROL.INPUT.ENC_YAW_RATE_LPF = reshape(double(typecast(DATA(DataOffset+165:DataOffset+168),'single')),1,1);
CONTROL.INPUT.ENC_YAW_RATE_KF = reshape(double(typecast(DATA(DataOffset+169:DataOffset+172),'single')),1,1);
CONTROL.INPUT.ENC_YAW_ANG = reshape(double(typecast(DATA(DataOffset+173:DataOffset+176),'single')),1,1);
CONTROL.INPUT.ENC_YAW_ANG_LPF = reshape(double(typecast(DATA(DataOffset+177:DataOffset+180),'single')),1,1);
CONTROL.INPUT.ENC_YAW_ANG_KF = reshape(double(typecast(DATA(DataOffset+181:DataOffset+184),'single')),1,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 140)
CONTROL.OUTPUT.MOTOR_PWM = reshape(double(typecast(DATA(DataOffset+1:DataOffset+8),'single')),2,1);
CONTROL.OUTPUT.MOTOR_VOLT = reshape(double(typecast(DATA(DataOffset+9:DataOffset+16),'single')),2,1);
CONTROL.OUTPUT.MOTOR_VOLT_CD = reshape(double(typecast(DATA(DataOffset+17:DataOffset+24),'single')),2,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 141)
CONTROL.STATE.BUTTONS = reshape(DATA(DataOffset+1:DataOffset+3),3,1);
CONTROL.STATE.MOTOR_MODE = reshape(DATA(DataOffset+4:DataOffset+4),1,1);
CONTROL.STATE.CPU_LOAD = reshape(DATA(DataOffset+5:DataOffset+5),1,1);
CONTROL.STATE.MCS_RX_STATUS = reshape(DATA(DataOffset+6:DataOffset+6),1,1);
CONTROL.STATE.NAV_RX_STATUS = reshape(DATA(DataOffset+7:DataOffset+7),1,1);
CONTROL.STATE.COMP_TIME = reshape(double(typecast(DATA(DataOffset+8:DataOffset+11),'single')),1,1);
CONTROL.STATE.COMP_IAE = reshape(double(typecast(DATA(DataOffset+12:DataOffset+15),'single')),1,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 142)
CONTROL.TARGET.YAW_RATE_REF = reshape(double(typecast(DATA(DataOffset+1:DataOffset+4),'single')),1,1);
CONTROL.TARGET.YAW_ANG_REF = reshape(double(typecast(DATA(DataOffset+5:DataOffset+8),'single')),1,1);
CONTROL.TARGET.PITCH_ANG_REF = reshape(double(typecast(DATA(DataOffset+9:DataOffset+12),'single')),1,1);
CONTROL.TARGET.FORWARD_VEL_REF = reshape(double(typecast(DATA(DataOffset+13:DataOffset+16),'single')),1,1);
CONTROL.TARGET.WALL_DIST_REF = reshape(double(typecast(DATA(DataOffset+17:DataOffset+20),'single')),1,1);
CONTROL.TARGET.POS_XY_REF = reshape(double(typecast(DATA(DataOffset+21:DataOffset+28),'single')),2,1);
CONTROL.TARGET.VEL_XY_REF = reshape(double(typecast(DATA(DataOffset+29:DataOffset+36),'single')),2,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 143)
CONTROL.PARAM.VEL_GAIN_EST = reshape(double(typecast(DATA(DataOffset+1:DataOffset+4),'single')),1,1);
CONTROL.PARAM.VEL_TIME_CONSTANT_EST = reshape(double(typecast(DATA(DataOffset+5:DataOffset+8),'single')),1,1);
CONTROL.PARAM.FILT_TIME_CONSTANT_EST = reshape(double(typecast(DATA(DataOffset+9:DataOffset+12),'single')),1,1);
CONTROL.PARAM.FV_PI_STR_K = reshape(double(typecast(DATA(DataOffset+13:DataOffset+16),'single')),1,1);
CONTROL.PARAM.FV_PI_STR_Ti = reshape(double(typecast(DATA(DataOffset+17:DataOffset+20),'single')),1,1);
CONTROL.PARAM.FV_PI_STR_b = reshape(double(typecast(DATA(DataOffset+21:DataOffset+24),'single')),1,1);
CONTROL.PARAM.ADPT_FLAG = reshape(DATA(DataOffset+25:DataOffset+25),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.K = reshape(double(typecast(DATA(DataOffset+26:DataOffset+29),'single')),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.Ti = reshape(double(typecast(DATA(DataOffset+30:DataOffset+33),'single')),1,1);
CONTROL.PARAM.FORWARD_VEL_PID.b = reshape(double(typecast(DATA(DataOffset+34:DataOffset+37),'single')),1,1);
CONTROL.PARAM.FILT_TIME_CONSTANT_ACT = reshape(double(typecast(DATA(DataOffset+38:DataOffset+41),'single')),1,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end
%--------------------------------------------------------------
if ~all(RX_ID ~= 144)
CONTROL.INPUT.RANGE_FRONT_DIST = reshape(double(typecast(DATA(DataOffset+1:DataOffset+4),'single')),1,1);
CONTROL.INPUT.RANGE_REAR_DIST = reshape(double(typecast(DATA(DataOffset+5:DataOffset+8),'single')),1,1);
CONTROL.INPUT.RANGE_WALL_DIST = reshape(double(typecast(DATA(DataOffset+9:DataOffset+12),'single')),1,1);
CONTROL.INPUT.LIDAR_2D_WALL_DIST = reshape(double(typecast(DATA(DataOffset+13:DataOffset+16),'single')),1,1);
CONTROL.INPUT.WALL_DIST_FILT = reshape(double(typecast(DATA(DataOffset+17:DataOffset+20),'single')),1,1);
CONTROL.INPUT.WALL_DIST_KF = reshape(double(typecast(DATA(DataOffset+21:DataOffset+24),'single')),1,1);
CONTROL.INPUT.RANGE_WALL_ANG = reshape(double(typecast(DATA(DataOffset+25:DataOffset+28),'single')),1,1);
CONTROL.INPUT.LIDAR_2D_WALL_ANG = reshape(double(typecast(DATA(DataOffset+29:DataOffset+32),'single')),1,1);
CONTROL.INPUT.WALL_ANG_FILT = reshape(double(typecast(DATA(DataOffset+33:DataOffset+36),'single')),1,1);
CONTROL.INPUT.WALL_ANG_KF = reshape(double(typecast(DATA(DataOffset+37:DataOffset+40),'single')),1,1);
CONTROL.INPUT.MCS_EULER_ANG = reshape(double(typecast(DATA(DataOffset+41:DataOffset+52),'single')),3,1);
CONTROL.INPUT.MCS_EARTH_POS = reshape(double(typecast(DATA(DataOffset+53:DataOffset+64),'single')),3,1);
CONTROL.INPUT.NAV_EULER_ANG = reshape(double(typecast(DATA(DataOffset+65:DataOffset+76),'single')),3,1);
CONTROL.INPUT.NAV_EARTH_POS = reshape(double(typecast(DATA(DataOffset+77:DataOffset+88),'single')),3,1);
CONTROL.INPUT.NAV_EARTH_POS_FILT = reshape(double(typecast(DATA(DataOffset+89:DataOffset+100),'single')),3,1);
CONTROL.INPUT.NAV_EARTH_VEL_FILT = reshape(double(typecast(DATA(DataOffset+101:DataOffset+112),'single')),3,1);
CONTROL.INPUT.MOTOR_RATE = reshape(double(typecast(DATA(DataOffset+113:DataOffset+120),'single')),2,1);
CONTROL.INPUT.MOTOR_RATE_CD = reshape(double(typecast(DATA(DataOffset+121:DataOffset+128),'single')),2,1);
CONTROL.INPUT.WALL_DIST = reshape(double(typecast(DATA(DataOffset+129:DataOffset+132),'single')),1,1);
CONTROL.INPUT.FORWARD_VEL = reshape(double(typecast(DATA(DataOffset+133:DataOffset+136),'single')),1,1);
CONTROL.INPUT.YAW_RATE = reshape(double(typecast(DATA(DataOffset+137:DataOffset+140),'single')),1,1);
CONTROL.INPUT.PITCH_RATE = reshape(double(typecast(DATA(DataOffset+141:DataOffset+144),'single')),1,1);
CONTROL.INPUT.YAW_ANG = reshape(double(typecast(DATA(DataOffset+145:DataOffset+148),'single')),1,1);
CONTROL.INPUT.PITCH_ANG = reshape(double(typecast(DATA(DataOffset+149:DataOffset+152),'single')),1,1);
CONTROL.INPUT.EARTH_POS = reshape(double(typecast(DATA(DataOffset+153:DataOffset+164),'single')),3,1);
CONTROL.INPUT.EARTH_VEL = reshape(double(typecast(DATA(DataOffset+165:DataOffset+176),'single')),3,1);
nn = nn + 1;
DataOffset = DataOffset + MSG_LEN(nn);
end

