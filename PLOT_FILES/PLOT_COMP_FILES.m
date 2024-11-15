%--------------------------------------------------------------
% DEFINE FILES (same reference)
%--------------------------------------------------------------
FILES = {
     'ensayo_PD_07_15_OUT_coche_20'
     'ensayo_PI_07_09_10_coche_20'
    };

PLOT_LEGEND = {
    'CONTROL PD'
    'CONTROL PI'
    };
%--------------------------------------------------------------
% REFERENCE TYPE
%--------------------------------------------------------------
% REF_TYPE:
% 1. FORWARD VELOCITY
% 2. YAW RATE
% 3. YAW ANGLE
% 4. WALL DISTANCE
REF_TYPE = 1;

% Signals and values to be plotted (one subplot for each one)
signals = [
    1 2
    2 2
    3 2
    4 2
    5 1
    5 2
    6 1
    6 2
    ];

%--------------------------------------------------------------
% SIGNALS IN SCOPE_DATA (structure with time)
%--------------------------------------------------------------
% 1.1 - FORWARD VELOCITY REF (m/s)
% 1.2 - FORWARD VELOCITY (m/s)
% 2.1 - YAW RATE REF (rad/s)
% 2.2 - YAW RATE (rad/s)
% 3.1 - YAW ANGLE REF (deg)
% 3.2 - YAW ANGLE (deg)
% 4.1 - WALL DIST REF (m)
% 4.2 - WALL DIST (m)
% 5.1 - LEFT MOTOR VOLT (V)
% 5.2 - RIGHT MOTOR VOLT (V)
% 5.3 - BATTERY VOLT (V)
% 6.1 - COM MOTOR VOLT (V)
% 6.2 - DIF MOTOR VOLT (V)

FIG_YLABEL = { ...
{'','FORWARD VELOCITY (m/s)'}
{'','YAW RATE (rad/s)'}
{'','YAW ANGLE (deg)'}
{'','WALL DIST (m)'}
{'LEFT MOTOR VOLT (V)','RIGHT MOTOR VOLT (V)'}
{'COM MOTOR VOLT (V)','DIF MOTOR VOLT (V)'}
};

%--------------------------------------------------------------
%% LOAD FILES (same reference)
%--------------------------------------------------------------
Nf = length(FILES);
for nn = 1:Nf
    command = ['load ' FILES{nn}];
    eval(command)
    command = ['SCOPE_DATA_' num2str(nn) ' = SCOPE_DATA;'];
    eval(command)
end
clear SCOPE_DATA

%--------------------------------------------------------------
%% FIND TIME SYNC FOR THE REFERENCE
%--------------------------------------------------------------
Ns = length(SCOPE_DATA_1.signals);
aux = [];
switch REF_TYPE 
    case 1
        scope_ref = 1;
        signal_ref = 1;
    case 2
        scope_ref = 2;
        signal_ref = 1;  
    case 3
        scope_ref = 3;
        signal_ref = 1;        
    case 4
        scope_ref = 4;
        signal_ref = 1;
    otherwise
        disp( 'The specified reference type is not valid')
        return
end
for nn = 1:Nf      
    command = ['ref_' num2str(nn) '= SCOPE_DATA_' num2str(nn) '.signals(scope_ref).values(:,signal_ref);'];
    eval(command)
    command = ['N' num2str(nn) ' = length(ref_' num2str(nn) ');'];
    eval(command)
    command = ['aux = [aux N' num2str(nn) '];'];
    eval(command)    
end
N = min(aux);
for nn = 1:Nf
    command = ['ref_' num2str(nn) ' = ref_' num2str(nn) '(1:N);'];
    eval(command)
end
k_start = ones(Nf,1);
NN = floor(N/4);
n_ref = 1;
for nn = 2:Nf
    command = ['ref_r = ref_' num2str(n_ref) ';'];
    eval(command)    
    command = ['ref_n = ref_' num2str(nn) ';'];
    eval(command)
    drn = NaN(NN,1);
    dnr = NaN(NN,1);
    for ii=1:NN
        drn(ii) = sum(abs(ref_r(1:NN)-ref_n(ii:ii+NN-1)));
        dnr(ii) = sum(abs(ref_n(1:NN)-ref_r(ii:ii+NN-1)));
    end
    [drn_min,krn_min] = min(drn);
    [dnr_min,knr_min] = min(dnr);
    [d_min,aux1] = min([drn_min dnr_min]);
    if aux1==1
        k_start(nn) = krn_min;
    else
        n_ref = nn;
        k_start(1:nn-1) = k_start(1:nn-1) + knr_min - 1;
    end
end
k_end = N - (max(k_start)-k_start);
for nn = 1:Nf
    command = ['SCOPE_DATA_' num2str(nn) '.time = SCOPE_DATA_' num2str(nn) '.time(k_start(nn):k_end(nn))-SCOPE_DATA_' num2str(nn)  '.time(k_start(nn));'];
    eval(command)
    for ii=1:Ns
        command = ['SCOPE_DATA_' num2str(nn) '.signals(ii).values = SCOPE_DATA_' num2str(nn) '.signals(ii).values(k_start(nn):k_end(nn),:);'];
        eval(command)
    end
end

% Close all the figures
close('all')
warning('off')

%--------------------------------------------------------------
% PLOT SIGNALS
%--------------------------------------------------------------
% Select signals
Nfig = size(signals(:,1),1);
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
figure('units','normalized','outerposition',[0 0 1 1])
sizeTitle = 8;
sizeLabel = 8;
sizeAxis = 8;
axis_sp = zeros(N,1);
for ii = 1:Nfig
    signal_sel = signals(ii,1);
    % Select value for each signal
    value_sel = signals(ii,2);
    subplot(sp1,sp2,ii)
    command = ['plot(SCOPE_DATA_1.time,[SCOPE_DATA_1.signals(signal_sel).values(:,value_sel)'];
    for jj = 2:Nf
        command = [command ' SCOPE_DATA_' num2str(jj) '.signals(signal_sel).values(:,value_sel)'];
    end
    if signal_sel==scope_ref && (value_sel-1)==signal_ref
        command = [command ' SCOPE_DATA_1.signals(signal_sel).values(:,signal_ref)],''LineWidth'',2);'];
        PLOT_LEGEND_EXT = [PLOT_LEGEND ; 'REF'];
    elseif signal_sel==5 
        command = [command ' SCOPE_DATA_1.signals(signal_sel).values(:,3)],''LineWidth'',2);'];
        PLOT_LEGEND_EXT = [PLOT_LEGEND ; 'BATTERY'];        
    else
        command = [command '],''LineWidth'',2);'];
        PLOT_LEGEND_EXT = PLOT_LEGEND;
    end  
    eval(command)
    ylabel(FIG_YLABEL{signal_sel}{value_sel},'fontsize',sizeLabel,'fontweight','b')
    grid
    xlabel('Time (s)','fontsize',sizeLabel,'fontweight','b');
    legend(PLOT_LEGEND_EXT,'Location','best')
    set(gca,'fontsize',sizeAxis,'fontweight','b')
    xlim([0 SCOPE_DATA_1.time(end)])
    axis_sp(ii)=gca;    
end
linkaxes(axis_sp,'x')
subplot

return
