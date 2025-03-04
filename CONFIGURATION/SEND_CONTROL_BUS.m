
% Initialization of variables
PC_MSG_BUFFER = uint8(1); % Message buffer index
COMM_FAILED = uint8(0); % Communication failure flag
nn = 0; % Retry counter
MSG_PC = MSG_INI;
CONTROL_PC = CONTROL_INI;
COMM_SAMPLING_TIME = CONTROL_PC.PARAM.COMM_SAMPLING_TIME;

if RUN_MODE == 5
    topics = ros2("topic", "list");
    target_topic = "/model/kitt/car_to_pc";
    if any(strcmp(topics, target_topic))
        disp("Car control system node is active, starting...");

        ros2_node = ros2node('matlab_bus_node');
        pub = ros2publisher(ros2_node,"model/kitt/pc_to_car","std_msgs/UInt8MultiArray","Reliability","besteffort","Depth",1);
        sub = ros2subscriber(ros2_node,"model/kitt/car_to_pc","std_msgs/UInt8MultiArray","Reliability","besteffort","Depth",1);
    else
        disp("Node is not active, aborting execution.")
        return
    end
else
    % clear rpi;
    rpi = raspberrypi(CAR_IP, 'pi', 'LabControl');
    % deleteFile(rpi,'/home/pi/SCOPE_PC.mat')
    stopModel(rpi,'CAR_CONTROL_SYSTEM')
    while isModelRunning(rpi,'CAR_CONTROL_SYSTEM')
        stopModel(rpi,'CAR_CONTROL_SYSTEM')
    end
    runModel(rpi,'CAR_CONTROL_SYSTEM')
    while isModelRunning(rpi,'CAR_CONTROL_SYSTEM')==false
        runModel(rpi,'CAR_CONTROL_SYSTEM')
    end
    % clear rpi;
    
    
    % Communication setup
    SEND_PORT = 27001; % Sending port
    RECEIVE_PORT = 27000; % Receiving port
    pause(2)
    tcpSend = tcpclient(CAR_IP, SEND_PORT, 'Timeout', 10);
    tcpReceive = tcpclient(CAR_IP, RECEIVE_PORT, 'Timeout', 10);
end

% Main communication loop
disp('Starting communication...');
while true

    % Encode the message to send using MSG_CODER
    MSG_PC.TX_ID = MSG_PC.MSG_ID_LIST(:,PC_MSG_BUFFER);
    MSG_PC = MSG_CODER(MSG_PC,CONTROL_PC); % Encode the MSG structure into TX_BUFFER

    % Send the encoded data
    if MSG_PC.TX_BUFFER(1, 1) > 0
        if RUN_MODE == 5
            msg = ros2message(pub);
            msg.data = MSG_PC.TX_BUFFER;
            send(pub,msg)
            disp('Ros2 message sent.');
        else
            write(tcpSend, MSG_PC.TX_BUFFER, 'uint8');
        end
        disp(['Message ' num2str(PC_MSG_BUFFER) ' sent to Raspberry Pi.']);
    end

    % Pause for processing
    pause(COMM_SAMPLING_TIME);

    % Receive and decode the message
    if MSG_PC.TX_BUFFER(1, 1) > 0
        if RUN_MODE == 5
            try
                msg2 = receive(sub,0.1);
                MSG_PC.RX_BUFFER = msg2.data;
                [MSG_PC, CONTROL_PC] = MSG_DECODER(MSG_PC, CONTROL_PC); % Decode received data into MSG
                disp(['Message ' num2str(PC_MSG_BUFFER) ' received and processed.']);
            catch
            end
        elseif tcpReceive.NumBytesAvailable > 0
            MSG_PC.RX_BUFFER = read(tcpReceive, length(MSG_PC.TX_BUFFER), 'uint8')';
            [MSG_PC, CONTROL_PC] = MSG_DECODER(MSG_PC, CONTROL_PC); % Decode received data into MSG
            disp(['Message ' num2str(PC_MSG_BUFFER) ' received and processed.']);
        end
    end

    % Compare TX and RX data
    BUFFER_RX = MSG_PC.RX_BUFFER;
    NUM_MSG_RX = BUFFER_RX(1);
    HEADER_RX = BUFFER_RX(1:2*NUM_MSG_RX+1);
    MSG_LEN_RX = HEADER_RX(3:2:end);
    if isempty(MSG_LEN_RX), MSG_LEN_RX=0; end
    ind_ini = 2*double(NUM_MSG_RX)+2+double(MSG_LEN_RX(1))+1;
    ind_end = ind_ini + sum(MSG_LEN_RX(2:end)) - 1;
    DATA_RX = BUFFER_RX(ind_ini:ind_end);
    BUFFER_TX = MSG_PC.TX_BUFFER;
    NUM_MSG_TX = BUFFER_TX(1);
    HEADER_TX = BUFFER_TX(1:2*NUM_MSG_TX+1);
    MSG_LEN_TX = HEADER_TX(3:2:end);
    if isempty(MSG_LEN_TX), MSG_LEN_TX=0; end
    ind_ini = 2*double(NUM_MSG_TX)+2+double(MSG_LEN_TX(1))+1;
    ind_end = ind_ini + sum(MSG_LEN_TX(2:end)) - 1;
    DATA_TX = BUFFER_TX(ind_ini:ind_end);
    if PC_MSG_BUFFER <= size(MSG_PC.MSG_ID_LIST, 2)
        if isequal(DATA_RX, DATA_TX) && ~isempty(DATA_TX)
            disp('Message matching.')
            COMM_FAILED = uint8(0);
            nn = 0;
            if PC_MSG_BUFFER == size(MSG_PC.MSG_ID_LIST, 2)
                CONTROL_PC.STATE.CURRENT_STATUS_PC = uint8(1); % WAITING FOR RPI SET UP
                PC_MSG_BUFFER = uint8(1);
                % Encode the message to send using MSG_CODER
                MSG_PC.TX_ID = uint8([2 0 0 0 0 0 0 0]');
                MSG_PC = MSG_CODER(MSG_PC,CONTROL_PC); % Encode the MSG structure into TX_BUFFER
                if RUN_MODE == 5
                    msg = ros2message(pub);
                    msg.data = MSG_PC.TX_BUFFER;
                    send(pub,msg);
                else
                    write(tcpSend, MSG_PC.TX_BUFFER, 'uint8');
                end
                disp('All messages sent and confirmed. Communication complete.');
                break;
            end
            PC_MSG_BUFFER = PC_MSG_BUFFER + 1;
        elseif nn >= 400 && (~isequal(DATA_RX,DATA_TX))
            disp('Communication failed. Stopping...');
            COMM_FAILED = uint8(1);
            CONTROL_PC.STATE.CURRENT_STATUS_PC = uint8(3); % STOP
            break;
        else
            nn = nn + 1;
        end
    end
end

% Cleanup
clear tcpSend tcpReceive pub sub ros2_node;
disp('Connections closed. Communication finished.');