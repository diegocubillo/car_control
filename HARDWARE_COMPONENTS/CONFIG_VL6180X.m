function VL6180X = CONFIG_VL6180X()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% CONFIG_VL6180X  %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters for VL6180 from ST
% Maximum clock frequency = 400 kHz

%% I2C ADDRESS
VL6180X.ADDRESS = hex2dec('29'); %41
VL6180X.ADDRESS_WRITE = hex2dec('52');
VL6180X.ADDRESS_READ = hex2dec('53'); 
VL6180X.FRONT_ADDRESS = hex2dec('65'); %101
VL6180X.REAR_ADDRESS = hex2dec('67'); %103

%%%%%%%%%%%%%%%%%%%%%%%%%%
% VL6180X Register Map
%%%%%%%%%%%%%%%%%%%%%%%%%%
% IDENTIFICATION
VL6180X.IDENTIFICATION_MODEL_ID              = hex2dec('000');
VL6180X.IDENTIFICATION_MODEL_REV_MAJOR       = hex2dec('001');
VL6180X.IDENTIFICATION_MODEL_REV_MINOR       = hex2dec('002');
VL6180X.IDENTIFICATION_MODULE_REV_MAJOR      = hex2dec('003');
VL6180X.IDENTIFICATION_MODULE_REV_MINOR      = hex2dec('004');
VL6180X.IDENTIFICATION_DATE_HI               = hex2dec('006');
VL6180X.IDENTIFICATION_DATE_LO               = hex2dec('007');
VL6180X.IDENTIFICATION_TIME                  = hex2dec('008'); % 16-bit
% SYSTEM
VL6180X.SYSTEM_MODE_GPIO0                    = hex2dec('010');
VL6180X.SYSTEM_MODE_GPIO1                    = hex2dec('011');
VL6180X.SYSTEM_HISTORY_CTRL                  = hex2dec('012');
VL6180X.SYSTEM_INTERRUPT_CONFIG_GPIO         = hex2dec('014');
VL6180X.SYSTEM_INTERRUPT_CLEAR               = hex2dec('015');
VL6180X.SYSTEM_FRESH_OUT_OF_RESET            = hex2dec('016');
VL6180X.SYSTEM_GROUPED_PARAMETER_HOLD        = hex2dec('017');
% SYSRANGE
VL6180X.SYSRANGE_START                       = hex2dec('018');
VL6180X.SYSRANGE_THRESH_HIGH                 = hex2dec('019');
VL6180X.SYSRANGE_THRESH_LOW                  = hex2dec('01A');
VL6180X.SYSRANGE_INTERMEASUREMENT_PERIOD     = hex2dec('01B');
VL6180X.SYSRANGE_MAX_CONVERGENCE_TIME        = hex2dec('01C');
VL6180X.SYSRANGE_CROSSTALK_COMPENSATION_RATE = hex2dec('01E'); % 16-bit
VL6180X.SYSRANGE_CROSSTALK_VALID_HEIGHT      = hex2dec('021');
VL6180X.SYSRANGE_EARLY_CONVERGENCE_ESTIMATE  = hex2dec('022'); % 16-bit
VL6180X.SYSRANGE_PART_TO_PART_RANGE_OFFSET   = hex2dec('024');
VL6180X.SYSRANGE_RANGE_IGNORE_VALID_HEIGHT   = hex2dec('025');
VL6180X.SYSRANGE_RANGE_IGNORE_THRESHOLD      = hex2dec('026'); % 16-bit
VL6180X.SYSRANGE_MAX_AMBIENT_LEVEL_MULT      = hex2dec('02C');
VL6180X.SYSRANGE_RANGE_CHECK_ENABLES         = hex2dec('02D');
VL6180X.SYSRANGE_VHV_RECALIBRATE             = hex2dec('02E');
VL6180X.SYSRANGE_VHV_REPEAT_RATE             = hex2dec('031');
% SYSALS
VL6180X.SYSALS_START                         = hex2dec('038');
VL6180X.SYSALS_THRESH_HIGH                   = hex2dec('03A');
VL6180X.SYSALS_THRESH_LOW                    = hex2dec('03C');
VL6180X.SYSALS_INTERMEASUREMENT_PERIOD       = hex2dec('03E');
VL6180X.SYSALS_ANALOGUE_GAIN                 = hex2dec('03F');
VL6180X.SYSALS_INTEGRATION_PERIOD            = hex2dec('040');
% RESULT
VL6180X.RESULT_RANGE_STATUS                  = hex2dec('04D');
VL6180X.RESULT_ALS_STATUS                    = hex2dec('04E');
VL6180X.RESULT_INTERRUPT_STATUS_GPIO         = hex2dec('04F');
VL6180X.RESULT_ALS_VAL                       = hex2dec('050'); % 16-bit
VL6180X.RESULT_HISTORY_BUFFER_0              = hex2dec('052'); % 16-bit
VL6180X.RESULT_HISTORY_BUFFER_1              = hex2dec('054'); % 16-bit
VL6180X.RESULT_HISTORY_BUFFER_2              = hex2dec('056'); % 16-bit
VL6180X.RESULT_HISTORY_BUFFER_3              = hex2dec('058'); % 16-bit
VL6180X.RESULT_HISTORY_BUFFER_4              = hex2dec('05A'); % 16-bit
VL6180X.RESULT_HISTORY_BUFFER_5              = hex2dec('05C'); % 16-bit
VL6180X.RESULT_HISTORY_BUFFER_6              = hex2dec('05E'); % 16-bit
VL6180X.RESULT_HISTORY_BUFFER_7              = hex2dec('060'); % 16-bit
VL6180X.RESULT_RANGE_VAL                     = hex2dec('062');
VL6180X.RESULT_RANGE_RAW                     = hex2dec('064');
VL6180X.RESULT_RANGE_RETURN_RATE             = hex2dec('066'); % 16-bit
VL6180X.RESULT_RANGE_REFERENCE_RATE          = hex2dec('068'); % 16-bit
VL6180X.RESULT_RANGE_RETURN_SIGNAL_COUNT     = hex2dec('06C'); % 32-bit
VL6180X.RESULT_RANGE_REFERENCE_SIGNAL_COUNT  = hex2dec('070'); % 32-bit
VL6180X.RESULT_RANGE_RETURN_AMB_COUNT        = hex2dec('074'); % 32-bit
VL6180X.RESULT_RANGE_REFERENCE_AMB_COUNT     = hex2dec('078'); % 32-bit
VL6180X.RESULT_RANGE_RETURN_CONV_TIME        = hex2dec('07C'); % 32-bit
VL6180X.RESULT_RANGE_REFERENCE_CONV_TIME     = hex2dec('080'); % 32-bit
% RANGE SCALER VALUES
VL6180X.RANGE_SCALER_VALUES = uint16([0 253 127 84]);% 
% RANGE
VL6180X.RANGE_SCALER                          = hex2dec('096'); % 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h
% VARIOUS
VL6180X.READOUT_AVERAGING_SAMPLE_PERIOD      = hex2dec('10A');
VL6180X.FIRMWARE_BOOTUP                      = hex2dec('119');
VL6180X.FIRMWARE_RESULT_SCALER               = hex2dec('120');
VL6180X.I2C_SLAVE_DEVICE_ADDRESS             = hex2dec('212');
VL6180X.INTERLEAVED_MODE_ENABLE              = hex2dec('2A3');


%% OTHER PARAMETERS
% Default value of SYSRANGE_CROSSTALK_VALID_HEIGHT
VL6180X.DEFAULT_CROSSTALK_VALID_HEIGHT = uint8(20); 
% Scaling (between 1 and 3)
VL6180X.RANGE_SCALING = 1;
% Period for continuous range measurement (ms) (10 ms resolution)
% The period must be greater than the time it takes to perform a
% measurement. See section 2.4.4 ("Continuous mode limits") in the datasheet
% for details.
VL6180X.RANGE_PERIOD = uint16(10);
VL6180X.SYSRANGE_INTERMEASUREMENT_PERIOD_BYTE = uint8(int16(VL6180X.RANGE_PERIOD/10) - int16(1));
% RANGING_MODE: 0 - SINGLE SHOT  / 1 - CONTINUOUS
VL6180X.RANGING_MODE = uint8(1);
% Period for continuous ambient light measurement (ms) (10 ms resolution)
% The period must be greater than the time it takes to perform a
% measurement. See section 2.4.4 ("Continuous mode limits") in the datasheet
% for details.
VL6180X.ALS_PERIOD = uint16(500);
VL6180X.SYSALS_INTERMEASUREMENT_PERIOD_BYTE = uint8(int16(VL6180X.ALS_PERIOD/10) - int16(1));
% Period for interleaved ambient light and range measurement (ms) (10 ms resolution)
% (10 ms resolution; defaults to 500 ms if not specified). In this mode, each
% ambient light measurement is immediately followed by a range measurement.
% The datasheet recommends using this mode instead of running "range and ALS
% continuous modes simultaneously (i.e. asynchronously)".
% The period must be greater than the time it takes to perform both
% measurements. See section 2.4.4 ("Continuous mode limits") in the datasheet
% for details.
VL6180X.INTERLEAVED_MODE = false;
if VL6180X.INTERLEAVED_MODE
    VL6180X.INTERLEAVED_PERIOD = uint16(500);
    VL6180X.SYSALS_INTERMEASUREMENT_PERIOD_BYTE = uint8(int16(VL6180X.ALS_PERIOD/10) - int16(1));
end



