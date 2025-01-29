%--------------------------------------------------------------
% FLAG FOR SAVING FILE .mat
%--------------------------------------------------------------
SAVE_DATA_FLAG = false;
FILE_NAME = 'File Name';
COMMENT = '';
%--------------------------------------------------------------
% FLAG FOR SELECTING TIME INTERVAL
%--------------------------------------------------------------
SELECT_TIME_FLAG = false;    

%--------------------------------------------------------------
%% SCOPE DATA DEFINITION
%--------------------------------------------------------------
load SCOPE_COMP_FV
SCOPE_DATA = SCOPE_COMP_FV;
% Filtering
z = tf('z',1);
alfa = 0.995;
filtTF = (1-alfa)/(1 - alfa*z^-1);
filt_IAE = lsim(filtTF,SCOPE_DATA.signals(3).values);
SCOPE_DATA.signals(3).values = [SCOPE_DATA.signals(3).values filt_IAE];
% Signals and values to be plotted
N = length(SCOPE_DATA.signals);
signals = [
    1 1
    1 2
    2 1
    3 1
    3 2
    ];

%--------------------------------------------------------------
% SIGNALS IN SCOPE_PC (structure with time)
%--------------------------------------------------------------
% 1.1 - FORWARD VELOCITY REF (m/s)
% 1.2 - FORWARD VELOCITY CONTROL (m/s)
% 2.1 - COM MOTOR VOLTAGE (V)
% 3.1 - FORWARD VELOCITY IAE
% 3.2 - FILTERED IAE

LEGEND = { ...
{'REF','CONTROL'}
{'COM VOLTAGE'}
{'IAE','FILTERED IAE'}
};

%--------------------------------------------------------------
% SELECT TIME INTERVAL
%--------------------------------------------------------------
% Close all the figures
close('all')
if SELECT_TIME_FLAG
    % Select time interval in samples
    plot(SCOPE_DATA.signals(1).values(:,1:2))
    xlabel('Samples')
    ylabel('rad/s')
    title('MOTOR RATE')
    grid
    disp('ZOOM TO SELECT THE DATA INTERVAL')
    disp('FINALLY, HIT ANY KEY')
    pause
    k = round(ginput(2));
    k = k(:,1);
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
    if nn==3
       text(2,3000,[SCOPE_DATA.signals(nn).title ' = ' num2str(round(filt_IAE(end)))],'fontsize',30,'fontweight','b')
    end
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
