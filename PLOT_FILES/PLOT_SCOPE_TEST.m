%--------------------------------------------------------------
% FLAG FOR SAVING FILE .mat
%--------------------------------------------------------------
SAVE_DATA_FLAG = false;
FILE_NAME = 'comp_P_30ms_Fim80';
COMMENT = '';
%--------------------------------------------------------------
% FLAG FOR SELECTING TIME INTERVAL
%--------------------------------------------------------------
SELECT_TIME_FLAG = false;

%--------------------------------------------------------------
%% SCOPE DATA DEFINITION
%--------------------------------------------------------------
SCOPE_DATA = SCOPE_TEST;
% Signals and values to be plotted
N = length(SCOPE_DATA.signals);
signals = [
    1 1
    1 2
    1 3
%     2 1
%     2 2
%     2 3
%     3 1
%     3 2
%     3 3
%     4 1
%     4 2
%     4 3
    5 1
    5 2
    5 3
    5 4
    6 1
    6 2
    6 3
    6 4
%     7 1
%     7 2
%     7 3
%     7 4
%     7 5
%     7 6
%     8 1
%     8 2
%     8 3
%     8 4
%     8 5
%     8 6
%     9 1
%     9 2
%     9 3
%     9 4
%     10 1 
%     10 2
%     10 3
%     10 4
%     11 1
%     11 2
%     12 1
%     12 2
%     12 3
    ];

%--------------------------------------------------------------
% SIGNALS IN SCOPE_SIM (structure with time)
%--------------------------------------------------------------
% 1.1 - FORWARD VELOCITY REF (m/s)
% 1.2 - FORWARD VELOCITY SIM (m/s)
% 1.3 - FORWARD VELOCITY TEST(m/s)
% 2.1 - YAW RATE REF (rad/s)
% 2.2 - YAW RATE SIM (rad/s)
% 2.3 - YAW RATE TEST (rad/s)
% 3.1 - YAW ANGLE REF (deg)
% 3.2 - YAW ANGLE SIM (deg)
% 3.3 - YAW ANGLE TEST (deg)
% 4.1 - WALL DIST REF (m)
% 4.2 - WALL DIST SIM (m)
% 4.3 - WALL DIST TEST (m)
% 5.1 - LEFT MOTOR VOLT SIM (V)
% 5.2 - RIGHT MOTOR VOLT SIM (V)
% 5.3 - LEFT MOTOR VOLT TEST (V)
% 5.4 - RIGHT MOTOR VOLT TEST (V)
% 6.1 - MOTOR COM VOLT SIM (V)
% 6.2 - MOTOR DIF VOLT SIM (V)
% 6.3 - MOTOR COM VOLT TEST (V)
% 6.4 - MOTOR DIF VOLT TEST (V)
% 7.1 - POSITION X REF (m)
% 7.2 - POSITION Y REF (m)
% 7.3 - POSITION X SIM (m)
% 7.4 - POSITION Y SIM (m)
% 7.5 - POSITION X TEST (m)
% 7.6 - POSITION Y REF (m)
% 8.1 - VELOCITY X REF (m/s)
% 8.2 - VELOCITY Y REF (m/s)
% 8.3 - VELOCITY X SIM (m/s)
% 8.4 - VELOCITY Y SIM (m/s)
% 8.5 - VELOCITY X TEST (m/s)
% 8.6 - VELOCITY Y TEST (m/s)
% 9.1 - LEFT MOTOR RATE SIM (rad/s)
% 9.2 - RIGHT MOTOR RATE SIM (rad/s)
% 9.3 - LEFT MOTOR RATE TEST (rad/s)
% 9.4 - RIGHT MOTOR RATE TEST (rad/s)
% 10.1 - MOTOR COM RATE SIM (rad/s)
% 10.2 - MOTOR DIF RATE SIM (rad/s)
% 10.3 - MOTOR COM RATE TEST (rad/s)
% 10.4 - MOTOR DIF RATE TEST (rad/s)
% 11.1 - PITCH RATE SIM (rad/s)
% 11.1 - PITCH RATE TEST (rad/s)
% 12.1 - PITCH ANGLE REF (deg)
% 12.2 - PITCH ANGLE SIM (deg)
% 12.3 - PITCH ANGLE TEST (deg)

LEGEND = { ...
{'REF','SIM','TEST'}
{'REF','SIM','TEST'}
{'REF','SIM','TEST'}
{'REF','SIM','TEST'}
{'SIM LEFT','SIM RIGHT','TEST LEFT','TEST RIGHT'}
{'SIM COM','SIM DIF','TEST COM','TEST DIF'}
{'REF X','REF Y','SIM X','SIM Y','TEST X','TEST Y'}
{'REF X','REF Y','SIM X','SIM Y','TEST X','TEST Y'}
{'SIM LEFT','SIM RIGHT','TEST LEFT','TEST RIGHT'}
{'SIM COM','SIM DIF','TEST COM','TEST DIF'}
{'SIM','TEST'}
{'REF','SIM','TEST'}
};

%--------------------------------------------------------------
% SELECT TIME INTERVAL
%--------------------------------------------------------------
% Close all the figures
close('all')
if SELECT_TIME_FLAG
    % Select time interval in samples
    plot([SCOPE_DATA.signals(5).values(:,1:2),SCOPE_DATA.signals(10).values(:,1:2)])
    xlabel('Samples')
    ylabel('deg & V')
    title('MOTOR VOLTAGE & MOTOR RATE')
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
