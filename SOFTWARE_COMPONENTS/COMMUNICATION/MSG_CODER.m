function MSG = MSG_CODER(MSG_IN,CONTROL)
%--------------------------------------------------------------
MSG = MSG_IN;
%--------------------------------------------------------------
TX_ID = sort(MSG.TX_ID(MSG.TX_ID>0));
NUM_MSG = uint8(length(TX_ID));
HEADER = zeros(2*NUM_MSG+1,1,'uint8');
HEADER(1) = NUM_MSG;
for nn = 1:NUM_MSG
HEADER(2*(nn-1)+2) = uint8(TX_ID(nn));
end
BUFFER = [];
nn = 1;
%--------------------------------------------------------------
if ~all(TX_ID ~= 1)
DATA = [];
DATA = [ DATA ; CONTROL.STATE.CURRENT_STATUS_SYS(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 2)
DATA = [];
DATA = [ DATA ; CONTROL.STATE.CURRENT_STATUS_PC(:)];
DATA = [ DATA ; CONTROL.STATE.PC_BUTTONS(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 3)
DATA = [];
DATA = [ DATA ; CONTROL.STATE.CONTROL_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.VEHICLE_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.COMM_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.FORWARD_VEL_CONTROL_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.YAW_RATE_CONTROL_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.YAW_ANG_CONTROL_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.WFL_CONTROL_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.PITCH_ANG_CONTROL_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.WFL_FEEDFORWARD(:)];
DATA = [ DATA ; CONTROL.STATE.GAIN_SCHEDULING(:)];
DATA = [ DATA ; CONTROL.STATE.SP_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.MOTOR_DELAY_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.OBSERVER_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.FORWARD_VEL_MAIN_OP(:)];
DATA = [ DATA ; CONTROL.STATE.ROTATION_MSRT_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.MCS_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.NAV_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.MV_TARGET_SOURCE(:)];
DATA = [ DATA ; CONTROL.STATE.YA_TARGET_SOURCE(:)];
DATA = [ DATA ; CONTROL.STATE.FV_TARGET_SOURCE(:)];
DATA = [ DATA ; CONTROL.STATE.FV_TARGET_VALUE(:)];
DATA = [ DATA ; CONTROL.STATE.YR_TARGET_SOURCE(:)];
DATA = [ DATA ; CONTROL.STATE.WD_TARGET_SOURCE(:)];
DATA = [ DATA ; CONTROL.STATE.MV_TARGET_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.YA_TARGET_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.YR_TARGET_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.FV_TARGET_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.WD_TARGET_TYPE(:)];
DATA = [ DATA ; CONTROL.STATE.FILTER_CHANGE(:)];
DATA = [ DATA ; CONTROL.STATE.STR_MODE(:)];
DATA = [ DATA ; typecast(single(CONTROL.STATE.DELAY(:)),'uint8')'];
DATA = [ DATA ; CONTROL.STATE.WALL_FOLLOWER_MODE(:)];
DATA = [ DATA ; typecast(single(CONTROL.STATE.PA_INITIAL_VALUE(:)),'uint8')'];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 4)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.OP_INPUT(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.OP_STATE(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.OP_OUTPUT(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 5)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.OPL_RC_SCALE(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.VEL_RC_SCALE(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_RC_SCALE(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.OPL_RC_SWITCH(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.VEL_RC_SWITCH(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_RC_SWITCH(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 6)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_ANG_PID.K(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_ANG_PID.Ti(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_ANG_PID.Td(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_ANG_PID.N(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_ANG_PID.b(:)),'uint8')'];
DATA = [ DATA ; CONTROL.PARAM.YAW_ANG_PID.INT_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.YAW_ANG_PID.DER_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.YAW_ANG_PID.DER_INPUT(:)];
DATA = [ DATA ; CONTROL.PARAM.YAW_ANG_PID.ANTIWINDUP(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 7)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_PID.K(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_PID.Ti(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_PID.Td(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_PID.N(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_PID.b(:)),'uint8')'];
DATA = [ DATA ; CONTROL.PARAM.FORWARD_VEL_PID.INT_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.FORWARD_VEL_PID.DER_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.FORWARD_VEL_PID.DER_INPUT(:)];
DATA = [ DATA ; CONTROL.PARAM.FORWARD_VEL_PID.ANTIWINDUP(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 8)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_PID.K(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_PID.Ti(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_PID.Td(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_PID.N(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_PID.b(:)),'uint8')'];
DATA = [ DATA ; CONTROL.PARAM.YAW_RATE_PID.INT_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.YAW_RATE_PID.DER_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.YAW_RATE_PID.DER_INPUT(:)];
DATA = [ DATA ; CONTROL.PARAM.YAW_RATE_PID.ANTIWINDUP(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 9)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_SFC.K(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_SFC.Ki(:)),'uint8')'];
DATA = [ DATA ; CONTROL.PARAM.FORWARD_VEL_SFC.INT_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.FORWARD_VEL_SFC.ANTIWINDUP(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 10)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_SFC.K(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_SFC.Ki(:)),'uint8')'];
DATA = [ DATA ; CONTROL.PARAM.YAW_RATE_SFC.INT_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.YAW_RATE_SFC.ANTIWINDUP(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 11)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_ANG_SFC.K(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_ANG_SFC.Ki(:)),'uint8')'];
DATA = [ DATA ; CONTROL.PARAM.YAW_ANG_SFC.INT_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.YAW_ANG_SFC.ANTIWINDUP(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 12)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_SFC.K(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 13)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_SL_PID.K(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_SL_PID.Ti(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_SL_PID.Td(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_SL_PID.N(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_SL_PID.b(:)),'uint8')];
DATA = [ DATA ; CONTROL.PARAM.WFL_SL_PID.INT_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.WFL_SL_PID.DER_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.WFL_SL_PID.DER_INPUT(:)];
DATA = [ DATA ; CONTROL.PARAM.WFL_SL_PID.ANTIWINDUP(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 14)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_CD_PID.K(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_CD_PID.Ti(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_CD_PID.Td(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_CD_PID.N(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_CD_PID.b(:)),'uint8')];
DATA = [ DATA ; CONTROL.PARAM.WFL_CD_PID.INT_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.WFL_CD_PID.DER_DISC_TYPE(:)];
DATA = [ DATA ; CONTROL.PARAM.WFL_CD_PID.DER_INPUT(:)];
DATA = [ DATA ; CONTROL.PARAM.WFL_CD_PID.ANTIWINDUP(:)];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 15)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.NAV_MPC.PRED_HRZ(:)),'uint8')'];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 16)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.CONTROL_SAMPLING_TIME(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.IMU_FILT_FREQ(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_FILT_FREQ(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.EKF_FILT_FREQ(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.WFL_FILT_FREQ(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.IMU_CF_FREQ(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.VEL_GAIN(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.VEL_TIME_CONSTANT(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_GAIN(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_TIME_CONSTANT(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.MOTOR_DELAY(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.MOTOR_DELAY_ERR(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.MOTOR_VOLT_DROP_DIF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.MOTOR_VOLT_MAX(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.MOTOR_VOLT_MIN(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.MOTOR_DEAD_ZONE(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.MOTOR_PWM_SLOPE(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.VEL_MAG_MAX(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.MCS_ALFA_CALIB(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.NAV_ALFA_CALIB(:)),'uint8')'];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 17)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.BATTERY_VOLT(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.CHARGING_CURRENT(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_ACCEL_MEAN(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_ACCEL_MEAN_FILT(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_GYRO_MEAN(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_GYRO_MEAN_FILT(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_EULER_RATE_CF(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_EULER_RATE_KF(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_EULER_ANG_CF(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_EULER_ANG_KF(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_PITCH_ANG_INI(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_FORWARD_ACCEL_CF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.IMU_FORWARD_ACCEL_KF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_MOTOR_RATE_LPF(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_MOTOR_RATE_KF(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_MOTOR_RATE_CD_LPF(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_MOTOR_RATE_CD_KF(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_FORWARD_ACCEL_LPF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_FORWARD_ACCEL_KF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_FORWARD_VEL_LPF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_FORWARD_VEL_KF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_YAW_RATE_LPF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_YAW_RATE_KF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_YAW_ANG(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_YAW_ANG_LPF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.ENC_YAW_ANG_KF(:)),'uint8')'];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 18)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.OUTPUT.MOTOR_PWM(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.OUTPUT.MOTOR_VOLT(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.OUTPUT.MOTOR_VOLT_CD(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 19)
DATA = [];
DATA = [ DATA ; CONTROL.STATE.BUTTONS(:)];
DATA = [ DATA ; CONTROL.STATE.MOTOR_MODE(:)];
DATA = [ DATA ; CONTROL.STATE.CPU_LOAD(:)];
DATA = [ DATA ; CONTROL.STATE.MCS_RX_STATUS(:)];
DATA = [ DATA ; CONTROL.STATE.NAV_RX_STATUS(:)];
DATA = [ DATA ; typecast(single(CONTROL.STATE.COMP_TIME(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.STATE.COMP_IAE(:)),'uint8')'];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 20)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.YAW_RATE_REF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.YAW_ANG_REF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.PITCH_ANG_REF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.FORWARD_VEL_REF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.WALL_DIST_REF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.POS_XY_REF(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.VEL_XY_REF(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 21)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.VEL_GAIN_EST(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.VEL_TIME_CONSTANT_EST(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FILT_TIME_CONSTANT_EST(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FV_PI_STR_K(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FV_PI_STR_Ti(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FV_PI_STR_b(:)),'uint8')'];
DATA = [ DATA ; CONTROL.PARAM.ADPT_FLAG(:)];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_PID.K(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_PID.Ti(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_PID.b(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FILT_TIME_CONSTANT_ACT(:)),'uint8')'];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 22)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.EKF_WFL.PARAM.RANGE_XA(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.EKF_WFL.PARAM.RANGE_YA(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.EKF_WFL.PARAM.LIDAR_2D_XA(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.EKF_WFL.PARAM.LIDAR_2D_YA(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.EKF_WFL.PARAM.LIDAR_2D_WFL_ANG_OFFS(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.EKF_WFL.PARAM.LIDAR_2D_WFL_ANG_SIGN(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.EKF_WFL.PROCESS_NOISE_VAR(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.EKF_WFL.OBSRV_NOISE_VAR(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 23)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_FORWARD_VEL_matAd(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_FORWARD_VEL_matBd(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_FORWARD_VEL_matCd(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_FORWARD_VEL_matQ(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_FORWARD_VEL_matR(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_YAW_RATE_matAd(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_YAW_RATE_matBd(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_YAW_RATE_matCd(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_YAW_RATE_matQ(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.ENC_YAW_RATE_matR(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 24)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.SVF_Ad(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.SVF_B1d(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.SVF_B2d(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.SVF_Cd(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.SVF_Dd(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 25)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.RANGE_FRONT_DIST(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.RANGE_REAR_DIST(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.RANGE_WALL_DIST(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.LIDAR_2D_WALL_DIST(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.WALL_DIST_FILT(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.WALL_DIST_KF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.RANGE_WALL_ANG(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.LIDAR_2D_WALL_ANG(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.WALL_ANG_FILT(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.WALL_ANG_KF(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.MCS_EULER_ANG(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.MCS_EARTH_POS(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.NAV_EULER_ANG(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.NAV_EARTH_POS(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.NAV_EARTH_POS_FILT(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.NAV_EARTH_VEL_FILT(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.MOTOR_RATE(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.MOTOR_RATE_CD(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.WALL_DIST(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.FORWARD_VEL(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.YAW_RATE(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.PITCH_RATE(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.YAW_ANG(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.PITCH_ANG(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.EARTH_POS(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.INPUT.EARTH_VEL(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 26)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.EKF_NAV.PROCESS_NOISE_VAR(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.EKF_NAV.OBSRV_NOISE_VAR(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 27)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.EKF_IMU.PARAM.ACCEL_ADPTV_GAIN(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.EKF_IMU.PROCESS_NOISE_VAR(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.EKF_IMU.OBSRV_NOISE_VAR(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 28)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.PITCH_ANG_SFC.K(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 29)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.MISSION_NUM_WAYPOINTS(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.MISSION_WAYPOINTS(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.MISSION_WP_RADIUS(:)),'uint8')'];
DATA = [ DATA ; typecast(single(CONTROL.TARGET.MISSION_YA_ERROR(:)),'uint8')'];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
%--------------------------------------------------------------
if ~all(TX_ID ~= 30)
DATA = [];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_DB.NUM(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.FORWARD_VEL_DB.DEN(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_DB.NUM(:)),'uint8')];
DATA = [ DATA ; typecast(single(CONTROL.PARAM.YAW_RATE_DB.DEN(:)),'uint8')];
BUFFER = [BUFFER ; DATA];
HEADER(2*(nn-1)+3) = uint8(length(DATA));
nn = nn + 1;
end
BUFFER = [HEADER ; BUFFER];
BUFFER_LEN = length(BUFFER);
MSG.TX_BUFFER = zeros(500,1,'uint8');
MSG.TX_BUFFER(1:BUFFER_LEN) = BUFFER;

