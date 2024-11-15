function LIN_MODEL = LINEARIZE_MODEL(FORWARD_VEL_OP,MODEL)
%-------------------------------------------------------------
% STATE  = {'FORWARD VEL','YAW RATE','YAW_ANG','WALL DIST','PITCH RATE','PITCH ANG'}
% INPUT = {'MOTOR VOLT LEFT ','MOTOR VOLT RIGHT '}
% OUTPUT = {'FORWARD VEL','YAW RATE',YAW ANG','WALL DIST','PITCH RATE','PITCH ANG'}
%-------------------------------------------------------------
% Simulink model
SLX_MODEL = 'CAR_MODEL';
%-------------------------------------------------------------
% Initial values
Xini = [FORWARD_VEL_OP ; 0 ;  0 ; 0.1 ; 0 ; 0];
IX = [1:6];
Uini = FORWARD_VEL_OP/MODEL.PARAM.VEL_GAIN*ones(2,1);
IU = [];
Yini = Xini;
IY = [1:6];
%-------------------------------------------------------------
% Temporal definition of Model_Bus
cd '../../BUS_DEFINITIONS'
BusDefinition(MODEL,'MODEL_Bus')
% Operating point
cd('../SIMULINK')
[X0,U0,Y0,DX0]=trim(SLX_MODEL,Xini,Uini,Yini,IX,IU,IY);
%-------------------------------------------------------------
% MODEL_OP struct definition
LIN_MODEL.OP_INPUT = U0;
LIN_MODEL.OP_STATE = X0;
LIN_MODEL.OP_OUTPUT = Y0;
LIN_MODEL.OP_STATE_DER = DX0;
%-------------------------------------------------------------
% Linearization
[matA_full,matB_full,matC_full,matD_full]=linmod(SLX_MODEL,X0,U0);
% Decoupled model
matB_full_dcp = matB_full(:,1:2)*[1 1 ; 1 -1];
matD_full_dcp = matD_full(:,1:2)*[1 1 ; 1 -1];
%-------------------------------------------------------------
% NAVIGATION MODEL 
%-------------------------------------------------------------
% State-space model 
AUX1_SS = ss(matA_full(1:3,1:3),matB_full_dcp(1:3,:),matC_full(1:3,1:3),matD_full_dcp(1:3,:));
TimeUnit = 'seconds';
InputName = {'MOTOR VOLT COM','MOTOR VOLT DIF','MOTOR DIST COM','MOTOR DIST DIF'};
InputUnit = {'V','V','V','V'};
OutputName = {'FORWARD VEL','YAW RATE','YAW ANG'};
OutputUnit = {'m/s','rad/s','rad'};
StateName = {'FORWARD VEL','YAW RATE','YAW ANG'};
StateUnit = {'m/s','rad/s','rad'};
% Continuous-time state-space model
matA = AUX1_SS.a;
matB = [AUX1_SS.b AUX1_SS.b];
matC = AUX1_SS.c;
matD = [AUX1_SS.d AUX1_SS.d];
% Model definition
NAV_SS_MODEL = ss(matA,matB,matC,matD);
NAV_SS_MODEL.TimeUnit = TimeUnit;
NAV_SS_MODEL.InputName = InputName;
NAV_SS_MODEL.InputUnit = InputUnit;
NAV_SS_MODEL.StateName = StateName;
NAV_SS_MODEL.StateUnit = StateUnit;
NAV_SS_MODEL.OutputName = OutputName;
NAV_SS_MODEL.OutputUnit = OutputUnit;
NAV_SS_MODEL.InputGroup.MV = [1 2];
NAV_SS_MODEL.InputGroup.MD = [];
NAV_SS_MODEL.InputGroup.UD = [3 4];
NAV_SS_MODEL.OutputGroup.MO = 1:3;
NAV_SS_MODEL.OutputGroup.UO = [];
LIN_MODEL.NAV_SS_MODEL = NAV_SS_MODEL;
%-------------------------------------------------------------
% FORWARD VELOCITY MODEL
%-------------------------------------------------------------
% State-space model 
matA_fv = matA_full(1,1);
matB_fv = matB_full_dcp(1,1);
matC_fv = matC_full(1,1);
matD_fv = matD_full_dcp(1,1);
AUX1_SS = ss(matA_fv,matB_fv,matC_fv,matD_fv);
InputName = {'MOTOR VOLT COM'};
InputUnit = {'V'};
OutputName = {'FORWARD VEL'};
OutputUnit = {'m/s'};
StateName = {'FORWARD VEL'};
StateUnit = {'m/s'};
% Model definition
FORWARD_VEL_SS_MODEL = AUX1_SS;
FORWARD_VEL_SS_MODEL.TimeUnit = TimeUnit;
FORWARD_VEL_SS_MODEL.InputName = InputName;
FORWARD_VEL_SS_MODEL.InputUnit = InputUnit;
FORWARD_VEL_SS_MODEL.StateName = StateName;
FORWARD_VEL_SS_MODEL.StateUnit = StateUnit;
FORWARD_VEL_SS_MODEL.OutputName = OutputName;
FORWARD_VEL_SS_MODEL.OutputUnit = OutputUnit;
FORWARD_VEL_SS_MODEL.InputGroup.MV = [1];
FORWARD_VEL_SS_MODEL.InputGroup.MD = [];
FORWARD_VEL_SS_MODEL.InputGroup.UD = [];
FORWARD_VEL_SS_MODEL.OutputGroup.MO = 1;
FORWARD_VEL_SS_MODEL.OutputGroup.UO = [];
LIN_MODEL.FORWARD_VEL_SS_MODEL = FORWARD_VEL_SS_MODEL;
%-------------------------------------------------------------
% YAW RATE MODEL
%-------------------------------------------------------------
% State-space model 
matA_yaw_rate = matA_full(2,2);
matB_yaw_rate = matB_full_dcp(2,2);
matC_yaw_rate = matC_full(2,2);
matD_yaw_rate = matD_full_dcp(2,2);
AUX1_SS = ss(matA_yaw_rate,matB_yaw_rate,matC_yaw_rate,matD_yaw_rate);
InputName = {'MOTOR VOLT DIF'};
InputUnit = {'V'};
OutputName = {'YAW RATE'};
OutputUnit = {'rad/s'};
StateName = {'YAW RATE'};
StateUnit = {'rad/s'};
% Model definition
YAW_RATE_SS_MODEL = AUX1_SS;
YAW_RATE_SS_MODEL.TimeUnit = TimeUnit;
YAW_RATE_SS_MODEL.InputName = InputName;
YAW_RATE_SS_MODEL.InputUnit = InputUnit;
YAW_RATE_SS_MODEL.StateName = StateName;
YAW_RATE_SS_MODEL.StateUnit = StateUnit;
YAW_RATE_SS_MODEL.OutputName = OutputName;
YAW_RATE_SS_MODEL.OutputUnit = OutputUnit;
YAW_RATE_SS_MODEL.InputGroup.MV = [1];
YAW_RATE_SS_MODEL.InputGroup.MD = [];
YAW_RATE_SS_MODEL.InputGroup.UD = [];
YAW_RATE_SS_MODEL.OutputGroup.MO = 1;
YAW_RATE_SS_MODEL.OutputGroup.UO = [];
LIN_MODEL.YAW_RATE_SS_MODEL = YAW_RATE_SS_MODEL;
%-------------------------------------------------------------
% YAW ANGLE MODEL
%-------------------------------------------------------------
% State-space model 
matA_yaw_ang = matA_full(2:3,2:3);
matB_yaw_ang = matB_full_dcp(2:3,2);
matC_yaw_ang = matC_full(3,2:3);
matD_yaw_ang = matD_full_dcp(3,2);
AUX1_SS = ss(matA_yaw_ang,matB_yaw_ang,matC_yaw_ang,matD_yaw_ang);
InputName = {'MOTOR VOLT DIF'};
InputUnit = {'V'};
OutputName = {'YAW ANG'};
OutputUnit = {'rad'};
StateName = {'YAW RATE','YAW ANG'};
StateUnit = {'rad/s','rad'};
% Model definition
YAW_ANG_SS_MODEL = AUX1_SS;
YAW_ANG_SS_MODEL.TimeUnit = TimeUnit;
YAW_ANG_SS_MODEL.InputName = InputName;
YAW_ANG_SS_MODEL.InputUnit = InputUnit;
YAW_ANG_SS_MODEL.StateName = StateName;
YAW_ANG_SS_MODEL.StateUnit = StateUnit;
YAW_ANG_SS_MODEL.OutputName = OutputName;
YAW_ANG_SS_MODEL.OutputUnit = OutputUnit;
YAW_ANG_SS_MODEL.InputGroup.MV = [1];
YAW_ANG_SS_MODEL.InputGroup.MD = [];
YAW_ANG_SS_MODEL.InputGroup.UD = [];
YAW_ANG_SS_MODEL.OutputGroup.MO = 1;
YAW_ANG_SS_MODEL.OutputGroup.UO = [];
LIN_MODEL.YAW_ANG_SS_MODEL = YAW_ANG_SS_MODEL;
%-------------------------------------------------------------
% WALL FOLLOWER MODEL
%-------------------------------------------------------------
% State-space model 
matA_wfl = matA_full(2:4,2:4);
matB_wfl = matB_full_dcp(2:4,2);
matC_wfl = matC_full(2:4,2:4);
matD_wfl = matD_full_dcp(2:4,2);
AUX1_SS = ss(matA_wfl,matB_wfl,matC_wfl,matD_wfl);
InputName = {'MOTOR VOLT DIF'};
InputUnit = {'V'};
OutputName = {'YAW RATE','YAW ANG','WALL DIST'};
OutputUnit = {'rad/s','rad','m'};
StateName = {'YAW RATE','YAW ANG','WALL DIST'};
StateUnit = {'rad/s','rad','m'};
% Model definition
WFL_SL_SS_MODEL = AUX1_SS;
WFL_SS_MODEL.TimeUnit = TimeUnit;
WFL_SL_SS_MODEL.InputName = InputName;
WFL_SS_MODEL.InputUnit = InputUnit;
WFL_SL_SS_MODEL.StateName = StateName;
WFL_SL_SS_MODEL.StateUnit = StateUnit;
WFL_SS_MODEL.OutputName = OutputName;
WFL_SL_SS_MODEL.OutputUnit = OutputUnit;
WFL_SL_SS_MODEL.InputGroup.MV = [1];
WFL_SL_SS_MODEL.InputGroup.MD = [];
WFL_SL_SS_MODEL.InputGroup.UD = [];
WFL_SL_SS_MODEL.OutputGroup.MO = 1:3;
WFL_SL_SS_MODEL.OutputGroup.UO = [];
LIN_MODEL.WFL_SL_SS_MODEL = WFL_SL_SS_MODEL;
%-------------------------------------------------------------
% ATTITUDE MODEL 
%-------------------------------------------------------------
% State-space model 
AUX1_SS = ss(matA_full([1 2 3 5 6],[1 2 3 5 6]),matB_full_dcp([1 2 3 5 6],:),matC_full([1 2 3 5 6],[1 2 3 5 6]),matD_full_dcp([1 2 3 5 6],:));
TimeUnit = 'seconds';
InputName = {'MOTOR VOLT COM','MOTOR VOLT DIF','MOTOR DIST COM','MOTOR DIST DIF'};
InputUnit = {'V','V','V','V'};
OutputName = {'FORWARD VEL','YAW RATE','YAW ANG','PITCH RATE','PITCH ANG'};
OutputUnit = {'m/s','rad/s','rad','rad/s','rad'};
StateName = {'FORWARD VEL','YAW RATE','YAW ANG','PITCH RATE','PITCH ANG'};
StateUnit = {'m/s','rad/s','rad','rad/s','rad'};
% Continuous-time state-space model
matA = AUX1_SS.a;
matB = [AUX1_SS.b AUX1_SS.b];
matC = AUX1_SS.c;
matD = [AUX1_SS.d AUX1_SS.d];
% Model definition
ATT_SS_MODEL = ss(matA,matB,matC,matD);
ATT_SS_MODEL.TimeUnit = TimeUnit;
ATT_SS_MODEL.InputName = InputName;
ATT_SS_MODEL.InputUnit = InputUnit;
ATT_SS_MODEL.StateName = StateName;
ATT_SS_MODEL.StateUnit = StateUnit;
ATT_SS_MODEL.OutputName = OutputName;
ATT_SS_MODEL.OutputUnit = OutputUnit;
ATT_SS_MODEL.InputGroup.MV = [1 2];
ATT_SS_MODEL.InputGroup.MD = [];
ATT_SS_MODEL.InputGroup.UD = [3 4];
ATT_SS_MODEL.OutputGroup.MO = 1:5;
ATT_SS_MODEL.OutputGroup.UO = [];
LIN_MODEL.ATT_SS_MODEL = ATT_SS_MODEL;
%-------------------------------------------------------------
% PITCH MODEL
%-------------------------------------------------------------
% State-space model 
matA_pitch = matA_full([1 5 6],[1 5 6]);
matB_pitch = matB_full_dcp([1 5 6],1);
matC_pitch = matC_full([1 5 6],[1 5 6]);
matD_pitch = matD_full_dcp([1 5 6],1);
AUX1_SS = ss(matA_pitch,matB_pitch,matC_pitch,matD_pitch);
InputName = {'MOTOR VOLT COM'};
InputUnit = {'V'};
OutputName = {'FORWARD VEL','PITCH RATE','PITCH ANG'};
OutputUnit = {'m/s','rad/s','rad'};
StateName = {'FORWARD VEL','PITCH RATE','PITCH ANG'};
StateUnit = {'m/s','rad/s','rad'};
% Model definition
PITCH_ANG_SS_MODEL = AUX1_SS;
PITCH_ANG_SS_MODEL.TimeUnit = TimeUnit;
PITCH_ANG_SS_MODEL.InputName = InputName;
PITCH_ANG_SS_MODEL.InputUnit = InputUnit;
PITCH_ANG_SS_MODEL.StateName = StateName;
PITCH_ANG_SS_MODEL.StateUnit = StateUnit;
PITCH_ANG_SS_MODEL.OutputName = OutputName;
PITCH_ANG_SS_MODEL.OutputUnit = OutputUnit;
PITCH_ANG_SS_MODEL.InputGroup.MV = [1];
PITCH_ANG_SS_MODEL.InputGroup.MD = [];
PITCH_ANG_SS_MODEL.InputGroup.UD = [];
PITCH_ANG_SS_MODEL.OutputGroup.MO = 1:3;
PITCH_ANG_SS_MODEL.OutputGroup.UO = [];
LIN_MODEL.PITCH_ANG_SS_MODEL = PITCH_ANG_SS_MODEL;
%-------------------------------------------------------------
% Change working folder
cd '../SOFTWARE_COMPONENTS/CONTROL'

return