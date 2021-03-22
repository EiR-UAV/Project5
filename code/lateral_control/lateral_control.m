close all;
clear all;
clc;

%% Constants
g = 9.81; 
m = 0.5; % uav mass

dt = 50e-03; % 50ms simulation step size

k_xp = -2;
k_yp = -2;
k_zp = -2;
k_xd = -2;
k_yd = -2;
k_zd = -2;
traj_track = false;

%% Connect to CS
disp('Program started');
sim=remApi('remoteApi'); 
sim.simxFinish(-1); 
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
else
    disp('Failed connecting to remote API server');
end


%% Setup simulations

% Start in synchronous mode
disp('Starting simulation...');
simxSynchronous(sim, clientID, 1);
simxStartSimulation(sim, clientID, remApi.simx_opmode_blocking);

simxSetIntegerSignal(sim, clientID, 'use_embedded_attitude_control', 1, ...
    remApi.simx_opmode_blocking); % i want to use embedded controller
[~, uav] = simxGetObjectHandle(sim, clientID, 'AscTed_Hummingbird_dynamic', ...
    remApi.simx_opmode_blocking); % get uav handle

%% Bootstrap signals
simxGetObjectPosition(sim, clientID, uav, -1, remApi.simx_opmode_streaming);
simxGetObjectOrientation(sim, clientID, uav, -1, remApi.simx_opmode_streaming);
simxGetObjectVelocity(sim, clientID, uav, remApi.simx_opmode_streaming);

simxGetFloatSignal(sim, clientID, 'Reference/position/x', remApi.simx_opmode_streaming);
simxGetFloatSignal(sim, clientID, 'Reference/position/y', remApi.simx_opmode_streaming);
simxGetFloatSignal(sim, clientID, 'Reference/position/z', remApi.simx_opmode_streaming);

simxGetFloatSignal(sim, clientID, 'Reference/velocity/x', remApi.simx_opmode_streaming);
simxGetFloatSignal(sim, clientID, 'Reference/velocity/y', remApi.simx_opmode_streaming);
simxGetFloatSignal(sim, clientID, 'Reference/velocity/z', remApi.simx_opmode_streaming);

simxGetFloatSignal(sim, clientID, 'Reference/acceleration/x', remApi.simx_opmode_streaming);
simxGetFloatSignal(sim, clientID, 'Reference/acceleration/y', remApi.simx_opmode_streaming);
simxGetFloatSignal(sim, clientID, 'Reference/acceleration/z', remApi.simx_opmode_streaming);

simxGetFloatSignal(sim, clientID, 'simulationTime', remApi.simx_opmode_streaming);

p_all = [];
v_all = [];
phi_all = [];
theta_all = [];
psi_all = []

oldCommandTime = 0.0;
lastCommandTime = 0.0;
iterations = 0;
min_iterations = 0;

%% Simulation cycle
disp('Simulation started');
while true
    % Check if simulation ended
    [~, info] = simxGetInMessageInfo(sim, clientID, remApi.simx_headeroffset_server_state);
    if info == 0
        disp('Simulation ended');
        break 
    end
    
    iterations = iterations + 1;
    if iterations < min_iterations
       continue; 
    end
    
    % Adjust simulation stepsize if not default 50ms
    lastCommandTime = simxGetLastCmdTime(sim, clientID);
    if iterations > min_iterations + 1
       dt = (lastCommandTime - oldCommandTime) * 1e-03;
    end
    oldCommandTime = lastCommandTime;

    %% Get state
    [~, p] = simxGetObjectPosition(sim, clientID, uav, -1, remApi.simx_opmode_buffer);
%   [~, ang] = simxGetObjectOrientation(sim, clientID, uav, -1, remApi.simx_opmode_buffer);
    [~, quat] = simxGetObjectQuaternion(sim, clientID, uav, -1, remApi.simx_opmode_buffer);
    [~, v, ang_v] = simxGetObjectVelocity(sim, clientID, uav, remApi.simx_opmode_buffer);

    %% Convert from quaternion ro RPY
    qw = quat(1); 
    qx = quat(2); 
    qy = quat(3); 
    qz = quat(4);
    
    % roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz);
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);

    % pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1)
        pitch = sign(sinp)* (pi/2); % use 90 degrees if out of range
    else
        pitch = asin(sinp);
    end

    % yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy);
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = atan2(siny_cosp, cosy_cosp);
    
    phi = roll; 
    theta = pitch; 
    psi = yaw;
    
    %% Get reference
    [~, px] = simxGetFloatSignal(sim, clientID, 'Reference/position/x', remApi.simx_opmode_buffer);
    [~, py] = simxGetFloatSignal(sim, clientID, 'Reference/position/y', remApi.simx_opmode_buffer);
    [~, pz] = simxGetFloatSignal(sim, clientID, 'Reference/position/z', remApi.simx_opmode_buffer);
    
    [~, vx] = simxGetFloatSignal(sim, clientID, 'Reference/velocity/x', remApi.simx_opmode_buffer);
    [~, vy] = simxGetFloatSignal(sim, clientID, 'Reference/velocity/y', remApi.simx_opmode_buffer);
    [~, vz] = simxGetFloatSignal(sim, clientID, 'Reference/velocity/z', remApi.simx_opmode_buffer);
    
    [~, ax] = simxGetFloatSignal(sim, clientID, 'Reference/acceleration/x', remApi.simx_opmode_buffer);
    [~, ay] = simxGetFloatSignal(sim, clientID, 'Reference/acceleration/y', remApi.simx_opmode_buffer);
    [~, az] = simxGetFloatSignal(sim, clientID, 'Reference/acceleration/z', remApi.simx_opmode_buffer);
    
    p_des = [px, py, pz]';
    v_des = [vx, vy, vz]';
    a_des = [ax, ay, az]';

    %% Gains (trajectory tracking)
    if ~traj_track && any(a_des)
       traj_track = true; 
       disp('Trajectory tracking');
    end
    
    %% Compute control
    
    % Compute position errors
    e_x = p_des(1) - p(1);
    e_y = p_des(2) - p(2);
    e_z = p_des(3) - p(3);

    e_vx = v_des(1) - v(1);
    e_vy = v_des(2) - v(2);
    e_vz = v_des(3) - v(3);
    
    % thrust control input
    T = (m / (cos(phi)*cos(theta)))*( g - a_des(3) - k_zp*e_z - k_zd*e_vz);
    
    U_x = k_xp*e_x + k_xd*e_vx + a_des(1);
    U_y = k_yp*e_y + k_yd*e_vy + a_des(2);
    
    T_x = (m/T)*U_x;
    T_y = (m/T)*U_y;
    
    phi_d = asin(T_y*cos(psi) - T_x*sin(psi));
    theta_d = -asin((T_x*cos(psi) + T_y*sin(psi))/cos(phi_d));
    %% For analysis
    [~, TSim] = simxGetFloatSignal(sim, clientID, 'simulationTime', remApi.simx_opmode_buffer);
    disp(TSim)
    if (TSim>=4)
    p_all = [p_all;p];
    v_all = [v_all;v];
    phi_all = [phi_all;phi_d];
    theta_all = [theta_all;theta_d];
    psi_all = [psi_all;psi];
    end
    %% Send controls to CS
    thrust = T;
    roll_des = phi_d;
    pitch_des = theta_d;
    
    simxPauseCommunication(sim, clientID, 1);
    
    simxSetFloatSignal(sim, clientID, 'ControlInputs/Thrust', thrust, remApi.simx_opmode_oneshot);
    simxSetFloatSignal(sim, clientID, 'ControlInputs/roll_des', roll_des, remApi.simx_opmode_oneshot);
    simxSetFloatSignal(sim, clientID, 'ControlInputs/pitch_des', pitch_des, remApi.simx_opmode_oneshot);
    
    simxPauseCommunication(sim, clientID, 0);
    
    %% Perform simulation step
    simxSynchronousTrigger(sim, clientID);
end
disp('Closing');
disp(p);
disp(v);
disp(p_des);
disp(v_des);
disp(a_des);
%% Disconnect from CS
sim.simxGetPingTime(clientID);
sim.simxFinish(clientID);
sim.delete();

disp('Connection to remote API server closed');