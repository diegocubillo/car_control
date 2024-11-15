function MD25 = CONFIG_MD25()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% MD25 PARAMETER STRUCT (Devantech)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Maximum clock frequency = 100 kHz

%--------------------------------------------------------------
% I2C ADDRESS
%--------------------------------------------------------------
MD25.ADDRESS = bin2dec('1011000');
MD25.ADDRESS_WRITE = hex2dec('B0');
MD25.ADDRESS_READ = hex2dec('B1');

%--------------------------------------------------------------
% REGISTER MAP
%--------------------------------------------------------------
% Motor1 speed (mode 0,1) or speed (mode 2,3) (R/W)
MD25.SPEED1 = hex2dec('00');
MD25.SPEED1_BYTE = hex2dec('00');
% Motor2 speed (mode 0,1) or turn (mode 2,3) (R/W)
MD25.SPEED2 = hex2dec('01');
MD25.SPEED2_BYTE = hex2dec('00');
% Encoder 1 position, 1st byte (highest), capture count when read (Read only)
MD25.ENC1A = hex2dec('02');
% Encoder 1 position, 2nd byte (Read only)
MD25.ENC1B = hex2dec('03');
% Encoder 1 position, 3rd byte (Read only)
MD25.ENC1C = hex2dec('04');
% Encoder 1 position, 4th byte (Read only)
MD25.ENC1D = hex2dec('05');
% Encoder 2 position, 1st byte (highest), capture count when read (Read only)
MD25.ENC2A = hex2dec('06');
% Encoder 2 position, 2nd byte (Read only)
MD25.ENC2B = hex2dec('07');
% Encoder 2 position, 3rd byte (Read only)
MD25.ENC2C = hex2dec('08');
% Encoder 2 position, 4th byte (Read only)
MD25.ENC2D = hex2dec('09');
% The supply battery voltage x10 (Read only)
MD25.BATTERY_VOLTS = hex2dec('0A');
% The current through motor 1 x10 (Read only)
MD25.MOTOR1_CURRENT = hex2dec('0B');
% The current through motor 2 x10 (Read only)
MD25.MOTOR2_CURRENT = hex2dec('0C');
% Software Revision Number (Read only)
MD25.SOFTWARE_REV = hex2dec('0D');
% Optional Acceleration register (R/W)
% Step from current speed to maximum speed in fordward or reversed
% direction. Default value is equal to 5 -> 1.25 s from maximum speed in
% fordward direction to maximum speed in reversed direction. Maximum value
% is equal to 10 -> 0.65 s from maximum speed in
% fordward direction to maximum speed in reversed direction. Each step is
% applied each 25 ms.
MD25.ACCEL_RATE = hex2dec('0E');
MD25.ACCEL_RATE_BYTE = hex2dec('05');
% Mode of operation (R/W)
% The mode register selects which mode of operation and I2C data input type
% the user requires. The options being:
% 0: (Default Setting) If a value of 0 is written to the mode register then
%    the meaning of the speed registers is literal speeds in the range of 0
%    (Full Reverse) 128 (Stop) 255 (Full Forward).
% 1: Mode 1 is similar to Mode 0, except that the speed registers are
%    interpreted as signed values. The meaning of the speed registers is
%    literal speeds in the range of -128 (Full Reverse) 0 (Stop) 127
%    (Full Forward).
% 2: Writing a value of 2 to the mode register will make speed1 control
%    both motors speed, and speed2 becomes the turn value.
%    Data is in the range of 0 (Full Reverse) 128 (Stop) 255 (Full Forward).
% 3: Mode 3 is similar to Mode 2, except that the speed registers are
%    interpreted as signed values. Data is in the range of -128
%    (Full Reverse) 0 (Stop) 127 (Full Forward).
MD25.MODE = hex2dec('0F');
MD25.MODE_BYTE = hex2dec('01');
% Command register
% 0x20 -> Resets the encoder registers to zero
% 0x30 -> Disables automatic speed regulation
% 0x31 -> Enables automatic speed regulation (default)
% 0x32 -> Disables 2 second timeout of motors (Version 2 onwards only)
% 0x33 -> Enables 2 second timeout of motors when no I2C comms (default)
%         (Version 2 onwards only)
% 0xA0 -> 1st in sequence to change I2C address
% 0xAA -> 2nd in sequence to change I2C address
% 0xA5 -> 3rd in sequence to change I2C address
MD25.COMMAND = hex2dec('10');

%--------------------------------------------------------------
% OTHER PARAMETERS
%--------------------------------------------------------------
% Battery voltage (V)
MD25.BATTERY_VOLT = 12;
% Battery voltage non-linear filter threshold  (V)
MD25.THR_NLFILT = 2.5;
% Battery voltage linear filter pole
MD25.ALFA_FILT = 0.995;
% Maximum and minimum PWM
MD25.PWM_MAX = 128;
MD25.PWM_MIN = -127;





