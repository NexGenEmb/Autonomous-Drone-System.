%% Xbox Controller → Bluetooth (HC-05) Bridge (adaptive axes)
clear; clc;

% === USER SETTINGS ===
btPort = "COM9";     % <- your HC-05 port
btBaud = 9600;
loopHz = 50;
RC_MIN = 1000; RC_MID = 1500; RC_MAX = 2000;

% === BLUETOOTH SERIAL ===
bt = serialport(btPort, btBaud);
configureTerminator(bt, "LF");
disp("✅ Bluetooth connected on " + btPort);

% === CONTROLLER ===
id = 1;   % first joystick
joy = vrjoystick(id);
try
    info(joy);  %#ok<*NOPTS>  % harmless warning about deprecation
catch
    % ignore; not needed for functionality
end

% Probe current axes/buttons to learn layout
[axes0, buttons0, ~] = read(joy);
nAx = numel(axes0);
fprintf("Detected %d axes\n", nAx);
fprintf("Axes snapshot: "); fprintf("%.2f ", axes0); fprintf("\n");

% ---- Axis mapping (adaptive) ----
% We'll support two common layouts:
%  (A) 6 axes: [LX, LY, LT, RX, RY, RT]  -> use RT (axis 6) for throttle
%  (B) 5 axes: [LX, LY, RX, RY, LT/RT combined] -> use combined (axis 5), RT positive
%
% Roll  = left stick X
% Pitch = left stick Y
% Yaw   = right stick X
%
% If your device uses a different order, just tweak indices below.

axis_Roll  = 1;   % LX
axis_Pitch = 2;   % LY
axis_Yaw   = min(4, nAx);  % RX (clamped in case nAx<4)

useSeparateTriggers = (nAx >= 6);
if useSeparateTriggers
    axis_RT = 6;   % RT on axis 6 in many XInput stacks
else
    axis_Comb = 5; % combined LT/RT on axis 5: RT≈+1, LT≈-1
end

% Buttons for arming (adjust if needed)
btn_Arm    = 1;  % A button
btn_Disarm = 2;  % B button

% === Loop timing ===
loopPeriod = 1/loopHz;
arm = false;
tPrev = tic;

disp("🎮 Control loop starting... press Ctrl+C to stop.");

while true
    [ax, btn, ~] = read(joy);

    % Safe-guard if driver drops axes momentarily
    if numel(ax) ~= nAx
        nAx = numel(ax);   % update
        useSeparateTriggers = (nAx >= 6);
        axis_Yaw = min(4, nAx);
    end

    % Sticks (invert signs if you prefer opposite)
    roll  = -ax(axis_Roll);     % right = +
    pitch =  ax(axis_Pitch);    % forward = +
    yaw   = -ax(axis_Yaw);      % right = +

    % Throttle
    if useSeparateTriggers
        % RT in [-1..+1] -> map to 0..1
        thrRaw = (ax(axis_RT) + 1) * 0.5;
    else
        % Combined LT/RT on one axis: assume RT drives positive
        comb = ax(axis_Comb);   % -1..+1 (LT..RT)
        thrRaw = max(0, comb);  % ignore LT; RT gives 0..+1
    end
    % Map to RC uS
    R = round(RC_MID + roll  * 500);
    P = round(RC_MID + pitch * 500);
    Y = round(RC_MID + yaw   * 500);
    T = round(RC_MIN + thrRaw * (RC_MAX - RC_MIN));

    % Arm / Disarm
    if btn(btn_Arm),   arm = true;  end
    if btn(btn_Disarm),arm = false; end
    A1 = 2000 * arm + 1000 * (~arm);  % AUX1

    % Send to Arduino over BT
    line = sprintf("R%d P%d Y%d T%d A1%d\n", R, P, Y, T, A1);
    write(bt, line, "string");

    % Optional debug
    % fprintf("axes: "); fprintf("%.2f ", ax); fprintf(" | R=%d P=%d Y=%d T=%d ARM=%d\n", R,P,Y,T,arm);

    % Keep loop rate
    elapsed = toc(tPrev);
    pause(max(0, loopPeriod - elapsed));
    tPrev = tic;
end
