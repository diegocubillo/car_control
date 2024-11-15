%--------------------------------------------------------------
% FLAG FOR SAVING FILE .mat
%--------------------------------------------------------------
SAVE_DATA_FLAG = false;
FILE_NAME = 'PRUEBA';
COMMENT = '';
%--------------------------------------------------------------
% FLAG FOR SELECTING TIME INTERVAL
%--------------------------------------------------------------
SELECT_TIME_FLAG = false;

%--------------------------------------------------------------
%% SCOPE DATA DEFINITION
%--------------------------------------------------------------
SCOPE_DATA = SCOPE_HW;
% Signals and values to be plotted
N = length(SCOPE_DATA.signals);
signals = [
    1 1
    1 2
    1 3
    1 4
    1 5
    1 6
    1 7
    1 8
    2 1
    2 2
    2 3
    2 4
    2 5
    2 6
    2 7
    2 8
    2 9
    2 10
    2 11
    2 12
    3 1
    3 2
    4 1
    4 2
    4 3
    5 1
    5 2
    5 3
    5 4
    5 5
    5 6
    5 7
    5 8
    6 1
    6 2
    7 1
    7 2
    7 3
    7 4
    8 1
    8 2
    8 3
    8 4
    8 5
    9 1
    9 2
    10 1 
    10 2
    10 3
    10 4
    10 5
    10 6
    10 7
    10 8
    10 9
    11 1
    11 2
    12 1
    12 2
    12 3
    12 4
    12 5
    12 6
    12 7
    12 8
    ];

%--------------------------------------------------------------
% SIGNALS IN SCOPE_SIM (structure with time)
%--------------------------------------------------------------
% 1.1 - IMU ACCEL X (m/s^2)
% 1.2 - IMU ACCEL Y (m/s^2)
% 1.3 - IMU ACCEL Z (m/s^2)
% 1.4 - IMU ACCEL X LPF (m/s^2)
% 1.5 - IMU ACCEL Y LPF (m/s^2)
% 1.6 - IMU ACCEL Z LPF (m/s^2)
% 1.7 - IMU FORWARD ACCEL CF (m/s^2)
% 1.8 - IMU FORWARD ACCEL EKF (m/s^2)
% 2.1 - IMU GYRO X (rad/s)
% 2.2 - IMU GYRO Y (rad/s)
% 2.3 - IMU GYRO Z (rad/s)
% 2.4 - IMU GYRO X LPF (rad/s)
% 2.5 - IMU GYRO Y LPF (rad/s)
% 2.6 - IMU GYRO Z LPF (rad/s)
% 2.7 - IMU ROLL RATE (rad/s)
% 2.8 - IMU PITCH RATE (rad/s)
% 2.9 - IMU YAW RATE (rad/s)
% 2.10 - IMU ROLL RATE LPF (rad/s)
% 2.11 - IMU PITCH RATE LPF (rad/s)
% 2.12 - IMU YAW RATE LPF (rad/s)
% 3.1 - FORWARD VEL LPF (m/s)
% 3.2 - FORWARD VEL KF (m/s)
% 4.1 - IMU PITCH ANGLE CF (deg)
% 4.2 - IMU PITCH ANGLE EKF (deg)
% 4.3 - MCS PITCH ANGLE (deg)
% 5.1 - ENC YAW ANGLE LPF (deg)
% 5.2 - ENC YAW ANGLE KF (deg)
% 5.3 - IMU YAW ANGLE CF (deg)
% 5.4 - IMU YAW ANGLE EKF (deg)
% 5.5 - MCS YAW ANGLE (deg)
% 5.6 - RANGE YAW ANGLE (deg)
% 5.7 - RANGE YAW ANGLE LPF (deg)
% 5.8 - RANGE YAW ANGLE EPF (deg)
% 6.1 - IMU PITCH RATE CF (rad/s)
% 6.2 - IMU PITCH RATE EKF (rad/s)
% 7.1 - ENC YAW RATE LPF (rad/s)
% 7.2 - ENC YAW RATE KF (rad/s)
% 7.3 - IMU YAW RATE CF (rad/s)
% 7.4 - IMU YAW RATE EKF (rad/s)
% 8.1 - RANGE WALL DISTANCE (m)
% 8.2 - RANGE FRONT DISTANCE(m)
% 8.3 - RANGE REAR DISTANCE (m)
% 8.4 - RANGE WALL DISTANCE LPF (m)
% 8.5 - RANGE WALL DISTANCE EKF (m)
% 9.1 - LEFT MOTOR PWM (%)
% 9.2 - RIGHT MOTOR PWM (%)
% 10.1 - BATTERY VOLTAGE (V)
% 10.2 - CONTROL LEFT MOTOR VOLT (V)
% 10.3 - CONTROL RIGHT MOTOR VOLT (V)
% 10.4 - REAL LEFT MOTOR VOLT (V)
% 10.5 - REAL RIGHT MOTOR VOLT (V)
% 10.6 - CONTROL COM MOTOR VOLT (V)
% 10.7 - CONTROL DIF MOTOR VOLT (V)
% 10.8 - REAL COM MOTOR VOLT (V)
% 10.9 - REAL DIF MOTOR VOLT (V)
% 11.1 - LEFT MOTOR CURRENT (A)
% 11.2 - RIGHT MOTOR CURRENT (A)
% 12.1 - ENCODER LEFT MOTOR RATE LPF (rad/s)
% 12.2 - ENCODER RIGHT MOTOR RATE LPF (rad/s)
% 12.3 - ENCODER LEFT MOTOR RATE KF (rad/s)
% 12.4 - ENCODER RIGHT MOTOR RATE KF (rad/s)
% 12.5 - ENCODER COM MOTOR RATE LPF (rad/s)
% 12.6 - ENCODER DIF MOTOR RATE LPF (rad/s)
% 12.7 - ENCODER COM MOTOR RATE KF (rad/s)
% 12.8 - ENCODER DIF MOTOR RATE KF (rad/s)

LEGEND = { ...
{'X','Y','Z','LPF X','LPF Y','LPF Z','CF FW','EKF PW'}
{'X','Y','Z','LPF X','LPF Y','LPF Z',...
'CF ROLL','CF PITCH','CF YAW',...
'EKF ROLL','EKF PITCH','EKF YAW'}
{'LPF', 'KF'}
{'IMU CF','IMU EKF','MCS'}
{'ENC LPF','ENC KF','IMU CF','IMU EKF','MCS','RANGE','RANGE LPF','RANGE EKF'}
{'CF','EKF'}
{'ENC LPF','ENC KF','IMU CF','IMU EKF'}
{'DIST', 'FRONT','REAR','DIST LPF','DIST EKF'}
{'LEFT', 'RIGHT'}
{'BATTERY','CONTROL LEFT','CONTROL RIGHT','REAL LEFT','REAL RIGHT',...
 'CONTROL COM','CONTROL DIF','REAL COM','REAL DIF'}
{'LEFT','RIGHT'}
{'LEFT LPF','RIGHT LPF','LEFT KF','RIGHT KF',...
 'COM LPF','DIF LPF','COM KF','DIF KF'}
};

%--------------------------------------------------------------
% SELECT TIME INTERVAL
%--------------------------------------------------------------
% Close all the figures
close('all')
if SELECT_TIME_FLAG
    % Select time interval in samples
    plot([SCOPE_DATA.signals(4).values(:,1), SCOPE_DATA.signals(10).values(:,2:3)])
    xlabel('Samples')
    ylabel('deg & V')
    title('PITCH ANGLE & MOTOR VOLTAGES')
    grid
    disp('ZOOM TO SELECT THE DATA INTERVAL')
    disp('FINALLY, HIT ANY KEY')
    pause
    k = round(ginput(2));
else
    k = [1 ; length(SCOPE_DATA.time)];
end

%--------------------------------------------------------------
% PLOT SIGNALS
%--------------------------------------------------------------
% Select signals
signal_sel = unique(signals(:,1));
Nfig = length(signal_sel);
% Subplot distribution
if Nfig<=3
    sp1 = Nfig;
    sp2 = 1;
elseif Nfig<=6
    sp1 = round((Nfig+0.5)/2);
    sp2 = 2;
elseif Nfig<=9
    sp1 = 3;
    sp2 = 3;
else
    sp1 = 4;
    sp2 = 3;
end
% New structure with time
SCOPE_DATA.time = double(SCOPE_DATA.time(k(1):k(2))-SCOPE_DATA.time(k(1)));
for nn=1:N
    SCOPE_DATA.signals(nn).values = double(SCOPE_DATA.signals(nn).values(k(1):k(2),:));    
end
figure('units','normalized','outerposition',[0 0 1 1])
sizeTitle = 8;
sizeLabel = 8;
sizeAxis = 8;
axis_sp = zeros(N,1);
for ii = 1:Nfig
    nn = signal_sel(ii);
    % Select values for each signal
    value_sel = signals((signals(:,1)==nn),2)';
    SCOPE_DATA.signals(nn).values = double(SCOPE_DATA.signals(nn).values(:,value_sel));
    subplot(sp1,sp2,ii)
    plot(SCOPE_DATA.time,SCOPE_DATA.signals(nn).values,'LineWidth',2);
    grid
    SCOPE_DATA.signals(nn).label = {'Time (s)',char(SCOPE_DATA.signals(nn).title)};
    xlabel(SCOPE_DATA.signals(nn).label(1),'fontsize',sizeLabel,'fontweight','b')
    ylabel(SCOPE_DATA.signals(nn).label(2),'fontsize',sizeLabel,'fontweight','b')
    SCOPE_DATA.signals(nn).legend = LEGEND{nn};
    title(SCOPE_DATA.signals(nn).title,'fontsize',sizeTitle,'fontweight','b')
    legend(SCOPE_DATA.signals(nn).legend(value_sel),'Location','best')
    set(gca,'fontsize',sizeAxis,'fontweight','b')
    xlim([0 SCOPE_DATA.time(end)])
    axis_sp(nn)=gca;    
end
linkaxes(axis_sp,'x')
subplot

%--------------------------------------------------------------
% SAVE FILE .mat
%--------------------------------------------------------------
if SAVE_DATA_FLAG
    command = ['save ' FILE_NAME ' COMMENT SCOPE_DATA'];
    eval(command)
end
clear command COMMENT FILE_NAME

return
