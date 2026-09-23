%debug Lawnmower for UT project
% June 2026 - Jens Schenebele

clear
% force to load a data base
db = canDatabase('GSL_DBC_v0.dbc');
pause(0.5);  % give driver time to settle
canCh=canChannel('PEAK-System','PCAN_USBBUS1');
pause(0.1);  % give driver time to settle
disp(canCh.BusSpeed)
% configBusSpeed(canCh, 250000);
% disp(canCh.BusSpeed)
pause(0.5);  % give driver time to settle
canCh.Database = db; % manually assigning databse without CanExplorere

% check what messages are in the file
specificDetails = messageInfo(db, 'RPDO1');
allDetails = messageInfo(db);

%% attempting to send
msg_RPDO1 = canMessage(db, 'RPDO1');
msg_RPDO1.Signals.Request_for_Mains_State=8;

msg_RPDO2 = canMessage(db, 'RPDO2');

msg_RPDO3 = canMessage(db, 'RPDO3');

msg_RPDO4 = canMessage(db, 'RPDO4');
msg_RPDO4.Signals.Batt_Dischg_Curr_Limit=20; % just enter in amps here % 30Amp = 1E; 20A=14; 10Amp=A 
msg_RPDO4.Signals.Batt_Chg_Curr_Limit=0; 

%%
start(canCh);
% continued transmission for certain messages
transmitPeriodic(canCh, msg_RPDO1, "On", 0.01);
pause(0.001)
transmitPeriodic(canCh, msg_RPDO2, "On", 0.01);
pause(0.001)
transmitPeriodic(canCh, msg_RPDO3, "On", 0.01);
pause(0.001)
transmitPeriodic(canCh, msg_RPDO4, "On", 0.01);
%%
% T = 0.01;
% while true
%     t0 = tic;
%     msgs=[msg_RPDO1 msg_RPDO2 msg_RPDO3 msg_RPDO4];
%     transmit(canCh, msgs, "On", 0.01);
%     % coarse wait
%     while toc(t0) < (T - 0.001)
%         pause(0.001);
%     end
%     % fine wait (tight)
%     while toc(t0) < T
%     end
% end

%%
msg_RPDO2.Signals.Motor1_ControlModeReq=1; % 1=speed mode
msg_RPDO2.Signals.Motor1_PWMOutEnableReq=1; % enable inverter
msg_RPDO2.Signals.Motor1_RefSpeedTorque_EnableReq=1; %enable comand

msg_RPDO2.Signals.Motor1_SpeedRef_Lim=200; % this shows up in Tau as Speed limit as rpm
msg_RPDO2.Signals.Motor1_TorqueRef_Lim=50; % this shows up in Tau as %

msg_RPDO2.Signals.Motor1_SpeedLimitReq=0; 
msg_RPDO2.Signals.Motor1_EmergencyStopReq=0;
msg_RPDO2.Signals.Motor1_SafetStopReq=0;

%%
msg_RPDO2.Signals.Motor1_ControlModeReq=0; % 1=speed mode
msg_RPDO2.Signals.Motor1_PWMOutEnableReq=0; % enable inverter
msg_RPDO2.Signals.Motor1_RefSpeedTorque_EnableReq=0; %enable comand

msg_RPDO2.Signals.Motor1_SpeedRef_Lim=0; % this shows up in Tau as Speed limit as rpm
msg_RPDO2.Signals.Motor1_TorqueRef_Lim=0; % this shows up in Tau as %

msg_RPDO2.Signals.Motor1_SpeedLimitReq=0; 
msg_RPDO2.Signals.Motor1_EmergencyStopReq=0;
msg_RPDO2.Signals.Motor1_SafetStopReq=0

%%
transmitPeriodic(canCh, msg_RPDO1, "off");
transmitPeriodic(canCh, msg_RPDO2, "off");
transmitPeriodic(canCh, msg_RPDO3, "off");
transmitPeriodic(canCh, msg_RPDO4, "off");
% stop(canCh)


%% start and log data
% start(canCh);
pause(.5); % capture the heartbeat 
% read everything that arrived
% disp("recording data")
% rxData = receive(canCh,Inf); 
% pause(10)
for k = 1:20
    rxData = receive(canCh,Inf, OutputFormat="timetable");

    if ~isempty(rxData)
        disp(rxData(:,{'ID','Extended','Data','Length'}));
    end
    pause(0.2)
end
% stop(canCh);
if ~isempty(rxData)
    % newDat=timetable2table(canSignalTimetable(rxData))
else
    disp("no mes recived")
end % if rxData
%
tempDat_struct=canSignalTimetable(rxData);
lastDat=timetable2table(tempDat_struct.TPDO1)
return
%% sequencing RPDO2
msg_RPDO2 = canMessage(db, 'RPDO2');
msg_RPDO2.Signals.Motor1_PWMOutEnableReq=1;
% --- PHASE 1: TRANSMIT REQUEST ---
disp('Phase 1: Sending mode switch request...');
transmit(canCh, msg_RPDO2);
%% --- PHASE 2: WAIT FOR CONFIRMATION ---
disp('Waiting for node confirmation message...');
%
% Configuration variables for the safety timeout loop
confirmationReceived = false;
timeoutDuration = 10.0; % Max seconds to wait before giving up
startTime = tic;       % Start a high-resolution stop watch

while toc(startTime) < timeoutDuration
    % Read any messages currently waiting in the hardware buffer.
    % Passing '1' means read 1 message at a time.
    % Passing a time duration (e.g., 0.1 seconds) blocks the loop efficiently.
    rxData = receive(canCh, 1, OutputFormat="timetable"); 

    % If the buffer was empty during that 100ms window, loop again
    if isempty(rxData)
        continue; 
    end

    % Check if the received message matches your expected confirmation message.
    % Replace 'TPDO1_StatusMessage' with the exact name from your DBC file.
    if strcmp(rxData.Name, 'TPDO4')

        % Check the specific signal value that indicates success.
        % Replace 'Mains_State_Feedback' and '1' with your actual DBC signal/value.
        if rxData.('Signals'){1,1}.Motor1_PWM_Output == 1
            disp('Confirmation received! Node successfully toggled to new mode.');
            confirmationReceived = true;
            break; % Exit the while loop immediately
        end
    end
end
%%
% --- PHASE 3: CONDITIONAL NEXT STEP ---
if confirmationReceived
    disp('Phase 3: Initiating next phase of the sequence...');

    % Configure and transmit your next phase message
    msg_RPDO2.Signals.Motor1_RefSpeedTorque_EnableReq=1
    transmit(canCh, msg_RPDO2);

else
    % Safety catch: Do not proceed if the node ignored your request or timed out
    error('Sequence Aborted: Timeout reached without receiving node confirmation.');
end

%% post process
msgtimetable = canMessageTimetable(rxData);
sigTimetable= canSignalTimetable(msgtimetable);
%% --- cleanup ---
stop(canCh);
delete(canCh);
clear canCh;

