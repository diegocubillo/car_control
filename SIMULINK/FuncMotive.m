function FuncMotive(block)
% Level-2 MATLAB file S-Function for unit delay demo.
%   Copyright 1990-2009 The MathWorks, Inc.
%   $Revision: 1.1.6.3 $
 
  setup(block);
 
%endfunction
 
function setup(block)
 
  block.NumDialogPrms  = 1;
 
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;
 
  %% Setup functional port properties to dynamically
  %% inherited.
  %block.SetPreCompInpPortInfoToInherited;
  %block.SetPreCompOutPortInfoToInherited;
 
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
 
  block.OutputPort(1).Dimensions       = 7;  % (6  Before) one Object: (x,y,z, yaw, pitch, roll)
 
  %% Set block sample time to [0.01 0]
  block.SampleTimes = [0.01 0];
 %%%%%%%%%%%%%%%%%%%%%%%%%% Sampling time: 10 ms %%%%%%%%%%%%%%%%%%%%%%%%%%%
 
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';
 
  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions); 
  block.RegBlockMethod('Outputs',                 @Output); 
  block.RegBlockMethod('Update',                  @Update);
  block.RegBlockMethod('Terminate',               @Terminate);
 
 
%endfunction
 
function DoPostPropSetup(block)
 
  %% Setup Dwork
  block.NumDworks = 1;
  block.Dwork(1).Name = 'x0';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
 
%endfunction
 
function InitConditions(block)
 
  %% Initialize Dwork
  block.Dwork(1).Data = block.DialogPrm(1).Data;
 
 disp('NatNet Sample Begin')
    global theClient;
    global frameRate;
    lastFrameTime = -1.0;
    lastFrameID = -1.0;
    usePollingLoop = false;         % approach 1 : poll for mocap data in a tight loop using GetLastFrameOfData
    usePollingTimer = false;        % approach 2 : poll using a Matlab timer callback ( better for UI based apps )
    useFrameReadyEvent = true;      % approach 3 : use event callback from NatNet (no polling)
  %  useUI = true;
 
  %  persistent arr;
    % Open figure
  %  if(useUI)
  %      hFigure = figure('Name','OptiTrack NatNet Matlab Sample','NumberTitle','off');
  %  end
 
   % try
        % Add NatNet .NET assembly so that Matlab can access its methods, delegates, etc.
        % Note : The NatNetML.DLL assembly depends on NatNet.dll, so make sure they
        % are both in the same folder and/or path if you move them.
        display('[NatNet] Creating Client.')
        % TODO : update the path to your NatNetML.DLL file here :
	%%%%%%%%%%%%%%%%% CHANGED HERE %%%%%%%%%%%%%%%%%%%
        %dllPath = fullfile('c:','NatNetSDK2.5','Samples','bin','NatNetML.dll');
        %dllPath = fullfile('C:\PFM\CRAZYYAW\NATNETSDK\NatNetSDK\lib\x64','NatNetML.dll');
%         dllPath = fullfile('C:\Users\stic\Desktop\CAR_PROJECT\MOTIVE\NatNetSDK\lib\x64','NatNetML.dll');
        dllPath = fullfile('D:\CAR_PROJECT\NatNetSDK\NatNetSDK\lib\x64','NatNetML.dll');
        % dllPath = '..\NatNEtSDK\lib\x64\NatNetML.dll';
        %dllPath = fullfile('c:','NaturalPoint Trackd Module','naturalpointtracker.dll');
        assemblyInfo = NET.addAssembly(dllPath);
 
        % Create an instance of a NatNet client
        theClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast
        version = theClient.NatNetVersion();
        fprintf( '[NatNet] Client Version : %d.%d.%d.%d\n', version(1), version(2), version(3), version(4) );
 
        % Connect to an OptiTrack server (e.g. Motive)
        display('[NatNet] Connecting to OptiTrack Server.')
        hst = java.net.InetAddress.getLocalHost;
        
        
        %%%%%%%%%%%%%%%%% CHANGED HERE %%%%%%%%%%%%%%%%%%%
        %HostIP = char(hst.getHostAddress);
        localhost = char('127.0.0.1');
%         HostIP = char('192.168.1.140');% Motive Computer IP
%         ClientIP = char('192.168.1.102'); % GCS Computer IP
        HostIP = localhost;
        ClientIP = localhost;
        flg = theClient.Initialize(ClientIP,HostIP); % Flg = returnCode: 0 = Success
        if (flg == 0)
            display('[NatNet] Initialization Succeeded')
        else
            display('[NatNet] Initialization Failed')
        end
       
        % print out a list of the active tracking Models in Motive
        GetDataDescriptions(theClient)
       
        % Test - send command/request to Motive
        [byteArray, retCode] = theClient.SendMessageAndWait('FrameRate');
        if(retCode ==0)
            byteArray = uint8(byteArray);
            frameRate = typecast(byteArray,'single');
        end
    %catch
 
%endfunction
 
    function Output(block)
       
        %block.OutputPort(1).Data = block.Dwork(1).Data;
        global theClient;
        java.lang.Thread.sleep(5);
        data = theClient.GetLastFrameOfData();
        D=ProcessFrame(data);
        %block.OutputPort(1).Data=double([D.x;D.y;D.z]);
        block.OutputPort(1).Data=double([D.x;D.y;D.z;D.qX;D.qY;D.qZ;D.qW]);
        %frameTime = data.fLatency;
        %frameID = data.iFrame;
        %                     if(frameTime ~= lastFrameTime)
        %                         fprintf('FrameTime: %0.3f\tFrameID: %5d\n',frameTime, frameID);
        %                         lastFrameTime = frameTime;
        %                         lastFrameID = frameID;
        %                     else
        %                         display('Duplicate frame');
        %                     end
       
 
%endfunction
 
        function Update(block)
           
            block.Dwork(1).Data = block.InputPort(1).Data;
 
 function Terminate(block)
      global theClient;
      theClient.Uninitialize();
     display('NatNet Sample End')
   
 
%endfunction
 
      function [D] = ProcessFrame( frameOfData )
         
          rigidBodyData = frameOfData.RigidBodies(1);
         
          % Position
         
          % Test : Marker Y Position Data
          % angleY = data.LabeledMarkers(1).y;
         if ~isempty(rigidBodyData)
          % Test : Rigid Body Y Position Data
             D.x= rigidBodyData.x; %*1000
             D.y= rigidBodyData.y;
             D.z= rigidBodyData.z;
             D.qX=rigidBodyData.qx;
             D.qY=rigidBodyData.qy;
             D.qZ=rigidBodyData.qz;
             D.qW=rigidBodyData.qw;
         else
             D.x= 0; %*1000
             D.y= 0;
             D.z= 0;
             D.qX=0;
             D.qY=0;
             D.qZ=0;
             D.qW=0;
         end

          % Test : Rigid Body 'Yaw'
          % Note : Motive display euler's is X (Pitch), Y (Yaw), Z (Roll), Right-Handed (RHS), Relative Axes
          % so we decode eulers heres to match that.
         
          % Angles
%             q = quaternion( rigidBodyData.qx, rigidBodyData.qy, rigidBodyData.qz, rigidBodyData.qw );
%             qRot = quaternion( 0, 0, 0, 1);     % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
%             q = mtimes(q, qRot);
%             angles = EulerAngles(q,'XZY');
%             D.angleX = -angles(1) * 180.0 / pi;   % must invert due to 180 flip above
%             D.angleY = angles(2) * 180.0 / pi;
%             D.angleZ = -angles(3) * 180.0 / pi;   % must invert due to 180 flip above
%             p=angle2quat(angles(1),angles(2),angles(3),'ZYX');
%             [D.angleX,D.angleY,D.angleZ]=quat2angle(p,'XYX');
          %endfunction