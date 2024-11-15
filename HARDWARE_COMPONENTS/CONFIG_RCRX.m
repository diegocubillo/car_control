function RCRX = CONFIG_RCRX()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% RCRX PARAMETER STRUCT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Minimum value for channel range
CH_MIN = [1000 1000 1000 1000 ...
          1000 1000 1000 1000 ...
          1000 1000 1000 1000 ...
          1000 1000];
% CH_MIN = [1154 1094 1193 1146 ...
%           1000 1000 1000 1000 ...
%           1000 1000 1000 1000 ...
%           1000 1000];
% Maximum for channel range
CH_MAX = [2000 2000 2000 2000 ...
          2000 2000 2000 2000 ...
          2000 2000 2000 2000 ...
          2000 2000];
% CH_MAX = [1866 1894 1882 1866 ...
%           2000 2000 2000 2000 ...
%           2000 2000 2000 2000 ...
%           2000 2000];
% Trim or central point
RCRX.CH_TRIM = [1500 1500 1500 1500 ...
                1500 1500 1500 1500 ...
                1500 1500 1500 1500 ...
                1500 1500];
% RCRX.CH_TRIM = [1504 1492 1500 1500 ...
%                 1500 1500 1500 1500 ...
%                 1500 1500 1500 1500 ...
%                 1500 1500];
% Channel definition : {'COM MOTOR VOLT','DIFF MOTOR VOLT'}
%                      {'FORWARD VEL REF','YAW RATE REF'}
%                      {'FORWARD VEL REF','YAW ANGLE REF'}
%                      {'FORWARD VEL REF','WALL DIST REF'}
RCRX.CH_DEF = [2 1];
% FIT ENDPOINTS BY 2 STRAIGHT LINES 
% y = a1^u b1
% -1 = a1*CH_MIN + b1
%  0 = a1*CH_TRIM + b1
%  1 = a2*CH_MAX + b2
%  0 = a2*CH_TRIM + b2
NUM_CH = length(CH_MIN);
RCRX.CALIB_PARAM = zeros(NUM_CH,4);
for nn = 1:NUM_CH
    matA = [CH_MIN(nn)  1 0 0
            RCRX.CH_TRIM(nn) 1 0 0
            0 0 CH_MAX(nn) 1 
            0 0 RCRX.CH_TRIM(nn) 1];
    matB = [ -1 ; 0 ; 1 ; 0 ];
    RCRX.CALIB_PARAM(nn,:) = (matA\matB)';
end
% -------------------------------------------------------------------------
% figure(1)
% THROTTLE = linspace(0,1,100)';
% u_aux = [0.25 0.75]';
% y_aux = [u_aux(1)+RCRX_INI.MOT_THST_EXPO u_aux(2)-RCRX_INI.MOT_THST_EXPO]';
% A_aux = [u_aux.^3-u_aux.^2 u_aux-u_aux.^2];
% th_aux = A_aux\(y_aux-u_aux.^2);
% THROTTLE_EXPO = 2*([THROTTLE.^3 THROTTLE.^2 THROTTLE])*[th_aux(1) 1-th_aux(1)-th_aux(2) th_aux(2)]';
% plot(THROTTLE,THROTTLE_EXPO,'-',u_aux,2*y_aux,'ro')
% grid
% xlabel('THROTTLE (pu)')
% ylabel('MODIFIED THROTTLE (pu)')

