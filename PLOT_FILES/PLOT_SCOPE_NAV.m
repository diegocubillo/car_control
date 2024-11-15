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
SCOPE_DATA = SCOPE_COMP_NAV;
% Signals and values to be plotted
N = length(SCOPE_DATA.signals);
signals = [
    1 1
    1 2
    1 3
    1 4
    2 1
    2 2
    2 3
    2 4
    3 1
    3 2
    4 1
    4 2
    5 1
    5 2
    6 1
    6 2
    ];


%--------------------------------------------------------------
% SIGNALS IN SCOPE_SIM (structure with time)
%--------------------------------------------------------------
% 1.1 - POSITION X REF (m)
% 1.2 - POSITION Y REF (m)
% 1.3 - POSITION X CONTROL (m)
% 1.4 - POSITION Y CONTROL (m)
% 2.1 - VELOCITY X REF (m/s)
% 2.2 - VELOCITY Y REF (m/s)
% 2.3 - VELOCITY X CONTROL (m/s)
% 2.4 - VELOCITY Y CONTROL (m/s)
% 3.1 - VELOCITY MAGNITUDE REF (m/s)
% 3.2 - VELOCITY MAGNITUDE CONTROL (m/s)
% 4.1 - VELOCITY ANGLE REF (deg)
% 4.2 - VELOCITY ANGLE CONTROL (deg)
% 5.1 - COM MOTOR VOLT (V)
% 5.2 - DIF ROTOR VOLTAGE (V)
% 6.1 - LEFT MOTOR VOLT (V)
% 6.2 - RIGHT ROTOR VOLTAGE (V)
LEGEND = { ...
{'REF X','REF Y','X','Y'}
{'REF X','REF Y','X','Y'}
{'REF','MAG'}
{'REF','ANG'}
{'COM','DIF'}
{'LEFT','RIGHT'}
};

%--------------------------------------------------------------
% SELECT TIME INTERVAL
%--------------------------------------------------------------
% Close all the figures
close('all')
if SELECT_TIME_FLAG
    % Select time interval in samples
    plot([SCOPE_DATA.signals(5).values(:,1:2), SCOPE_DATA.signals(3).values(:,2)])
    xlabel('Samples')
    ylabel('V')
    title('COM & DIF MOTOR VOLTAGES & VELOCITY MAGNITUDE')
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

%--------------------------------------------------------------
% 2D PLOT
%--------------------------------------------------------------
L = 2;
AUX_X0 = 0;
AUX_X1 = L/2;
AUX_X2 = -L/2;
AUX_Y0 = 0;
AUX_Y1 = L/2;
AUX_Y2 = -L/2;
SQUARE_X_DATA = [AUX_X1 AUX_X2 AUX_X2 AUX_X1 AUX_X1];
SQUARE_Y_DATA = [AUX_Y1 AUX_Y1 AUX_Y2 AUX_Y2 AUX_Y1];

alfa = linspace(-pi,pi, 1000);
R = 0.05;

r0 = AUX_X0 + 1j*AUX_Y0 + R*exp(-1j*alfa);
x0 = real(r0);
y0 = imag(r0);

r1 = AUX_X1 + 1j*AUX_Y0 + R*exp(-1j*alfa);
x1 = real(r1);
y1 = imag(r1);

r2 = AUX_X1 + 1j*AUX_Y1 + R*exp(-1j*alfa);
x2 = real(r2);
y2 = imag(r2);

r3 = AUX_X0 + 1j*AUX_Y1 + R*exp(-1j*alfa);
x3 = real(r3);
y3 = imag(r3);

r4 = AUX_X2 + 1j*AUX_Y1 + R*exp(-1j*alfa);
x4 = real(r4);
y4 = imag(r4);

r5 = AUX_X1 + 1j*AUX_Y2 + R*exp(-1j*alfa);
x5 = real(r5);
y5 = imag(r5);

r6 = AUX_X0 + 1j*AUX_Y2 + R*exp(-1j*alfa);
x6 = real(r6);
y6 = imag(r6);

r7 = AUX_X2 + 1j*AUX_Y2 + R*exp(-1j*alfa);
x7 = real(r7);
y7 = imag(r7);

r8 = AUX_X2 + 1j*AUX_Y0 + R*exp(-1j*alfa);
x8 = real(r8);
y8 = imag(r8);

x = [x0, x1, x2, x3, x4, x5, x6, x7, x8];
y = [y0, y1, y2, y3, y4, y5, y6, y7, y8];

i = 2;
k = 1;
X_LEG = struct('WP1', {},'WP2', {},'WP3', {},'WP4', {},'WP5', {},'WP6', {},'WP7', {},'WP8', {},'WP9', {},'WP10', {}, 'WP11', {});
Y_LEG = struct('WP1', {},'WP2', {},'WP3', {},'WP4', {},'WP5', {},'WP6', {},'WP7', {},'WP8', {},'WP9', {},'WP10', {}, 'WP11', {});


while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP1 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP1 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end
k = 1;
i = i + 1;
while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP2 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP2 = SCOPE_DATA.signals(1).values(i,4);
            i = i +1;
            k = k + 1;
end
k = 1;
i = i + 1;
while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP3 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP3 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end
k = 1;
i = i + 1;
while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP4 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP4 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end
k = 1;
i = i + 1;
while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP5 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP5 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end
k = 1;
i = i + 1;
while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP6 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP6 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end
k = 1;
i = i + 1;
while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP7 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP7 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end
k = 1;
i = i + 1;
while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP8 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP8 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end
k = 1;
i = i + 1;
while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP9 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP9 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end
k = 1;
i = i + 1;
while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP10 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP10 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end
k = 1;
i = i + 1;

while (i <= length(SCOPE_DATA.time)) && (SCOPE_DATA.signals(1).values(i,1) == SCOPE_DATA.signals(1).values(i-1,1)) && (SCOPE_DATA.signals(1).values(i,2) == SCOPE_DATA.signals(1).values(i-1,2))
            X_LEG(k).WP11 = SCOPE_DATA.signals(1).values(i,3);
            Y_LEG(k).WP11 = SCOPE_DATA.signals(1).values(i,4);
            i = i + 1;
            k = k + 1;
end

figure(3) 
plot([X_LEG.WP1], [Y_LEG.WP1], 'LineWidth',2)
hold on
plot([X_LEG.WP2], [Y_LEG.WP2], 'LineWidth',2)
hold on
plot([X_LEG.WP3], [Y_LEG.WP3], 'LineWidth',2)
hold on
plot([X_LEG.WP4], [Y_LEG.WP4], 'LineWidth',2)
hold on
plot([X_LEG.WP5], [Y_LEG.WP5], 'LineWidth',2)
hold on
plot([X_LEG.WP6], [Y_LEG.WP6], 'LineWidth',2)
hold on
plot([X_LEG.WP7], [Y_LEG.WP7], 'LineWidth',2)
hold on
plot([X_LEG.WP8], [Y_LEG.WP8], 'LineWidth',2)
hold on
plot([X_LEG.WP9], [Y_LEG.WP9], 'LineWidth',2)
hold on
plot([X_LEG.WP10], [Y_LEG.WP10], 'LineWidth',2)
hold on
plot([X_LEG.WP11], [Y_LEG.WP11], 'LineWidth',2)
hold on
plot(SQUARE_X_DATA, SQUARE_Y_DATA, 'black')
hold on
i = 1;
j = 1000;
for aux = 1:9
    j = i*1000;
    j_ant = (i-1)*1000 + 1;
    plot(x(j_ant:j), y(j_ant:j), 'magenta', 'LineWidth',2)
    hold on
    i = i + 1;
end

axis(L/2*[-1.5 1.5 -1.5 1.5])
axis equal
return
