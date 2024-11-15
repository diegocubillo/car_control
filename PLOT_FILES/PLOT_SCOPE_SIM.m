%--------------------------------------------------------------
% FLAG FOR SAVING FILE .mat
%--------------------------------------------------------------
SAVE_DATA_FLAG = false;
FILE_NAME = 'ControlP_MAM';
COMMENT = '';
%--------------------------------------------------------------
% FLAG FOR SELECTING TIME INTERVAL
%--------------------------------------------------------------
SELECT_TIME_FLAG = false;

%--------------------------------------------------------------
%% SCOPE DATA DEFINITION
%--------------------------------------------------------------
SCOPE_DATA = SCOPE_SIM;
% Signals and values to be plotted
N = length(SCOPE_DATA.signals);
signals = [
    1 1
    1 2
    1 3
    2 1
    2 2
    2 3
    2 4
    3 1
    3 2
    3 3
    3 4
    4 1
    4 2
    4 3
    5 1
    5 2
    5 3
    5 4
    5 5
    6 1
    6 2
    6 3
    6 4
    7 1
    7 2
    7 3
    7 4
    7 5
    7 6
    8 1
    8 2
    8 3
    8 4
    8 5
    8 6
    9 1
    9 2
    10 1 
    10 2
    11 1
    11 2
    12 1
    12 2
    12 3
    ];

%--------------------------------------------------------------
% SIGNALS IN SCOPE_SIM (structure with time)
%--------------------------------------------------------------
% 1.1 - FORWARD VELOCITY REF (m/s)
% 1.2 - FORWARD VELOCITY CONTROL (m/s)
% 1.3 - FORWARD VELOCITY SIM (m/s)
% 2.1 - YAW RATE REF (rad/s)
% 2.2 - YAW RATE CONTROL (rad/s)
% 2.3 - YAW RATE SIM (rad/s)
% 2.4 - WALL RATE (rad/s)
% 3.1 - YAW ANGLE REF (deg)
% 3.2 - YAW ANGLE CONTROL (deg)
% 3.3 - YAW ANGLE SIM (deg)
% 3.4 - WALL ANGLE (deg)
% 4.1 - WALL DIST REF (m)
% 4.2 - WALL DIST CONTROL (m)
% 4.3 - WALL DIST SIM (m)
% 5.1 - LEFT MOTOR VOLTAGE (V)
% 5.2 - RIGHT CONTROL MOTOR VOLTAGE (V)
% 5.2 - RIGHT CONTROL MOTOR VOLTAGE (V)
% 5.3 - BATTERY VOLTAGE (V)
% 5.4 - LEFT REAL MOTOR VOLTAGE (V)
% 5.5 - RIGHT REAL MOTOR VOLTAGE (V)
% 6.1 - MOTOR CONTROL COM VOLTAGE (V)
% 6.2 - MOTOR CONTROL DIF VOLTAGE (V)
% 6.3 - MOTOR REAL COM VOLTAGE (V)
% 6.4 - MOTOR REAL DIF VOLTAGE (V)
% 7.1 - POSITION X REF (m)
% 7.2 - POSITION Y REF (m)
% 7.3 - POSITION X CONTROL (m)
% 7.4 - POSITION Y CONTROL (m)
% 7.5 - POSITION X SIM (m)
% 7.6 - POSITION Y SIM (m)
% 8.1 - VELOCITY X REF (m)
% 8.2 - VELOCITY Y REF (m)
% 8.3 - VELOCITY Y CONTROL (m)
% 8.4 - VELOCITY X CONTROL (m)
% 8.5 - VELOCITY X SIM (m)
% 8.6 - VELOCITY Y SIM (m)
% 9.1 - LEFT MOTOR RATE (rad/s)
% 9.2 - RIGHT MOTOR RATE (rad/s)
% 10.1 - MOTOR COM RATE (rad/s)
% 10.2 - MOTOR DIF RATE (rad/s)
% 11.1 - PITCH RATE CONTROL (rad/s)
% 11.2 - PITCH RATE SIM (rad/s)
% 12.1 - PITCH ANGLE REF (deg)
% 12.2 - PITCH ANGLE CONTROL (deg)
% 12.3 - PITCH ANGLE SIM (deg)

LEGEND = { ...
{'REF','CONTROL', 'SIM'}
{'REF','CONTROL', 'SIM','WALL RATE'}
{'REF','CONTROL', 'SIM','WALL ANG'}
{'REF','CONTROL', 'SIM'}
{'CONTROL LEFT ','CONTROL RIGHT ','BATTERY','SIM LEFT ','SIM RIGHT'}
{'CONTROL COM','CONTROL DIF','SIM COM','SIM DIF'}
{'REF X','REF Y','CONTROL X','CONTROL Y','SIM X','SIM Y'}
{'REF X','REF Y','CONTROL X','CONTROL Y','SIM X','SIM Y'}
{'LEFT','RIGHT'}
{'COM','DIF'}
{'CONTROL','SIM'}
{'REF','CONTROL','SIM'}
};

%--------------------------------------------------------------
% SELECT TIME INTERVAL
%--------------------------------------------------------------
% Close all the figures
close('all')
if SELECT_TIME_FLAG
    % Select time interval in samples
    plot([SCOPE_DATA.signals(12).values(:,2:3),SCOPE_DATA.signals(10).values(:,1:2)])
    xlabel('Samples')
    ylabel('deg & V')
    title('PITCH ANGLE & MOTOR RATE')
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
