close all;
clear all;
clc;

%% Constants
g = 9.81; 
m = 0.5; % uav mass

dt = 50e-03; % 50ms simulation step size

%% Gains (setpoint regulation)
cx = 0.905; cxd = 1.905; lx = 0.001;
cy = 0.905; cyd = 1.905; ly = 0.001;
cz = 1.255; czd = 1.625; lz = 0.001;

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
disp("Starting simulation...");
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

% Error integrals
chi_x = 0.0;
chi_y = 0.0;
chi_z = 0.0;

oldCommandTime = 0.0;
lastCommandTime = 0.0;
iterations = 0;
min_iterations = 0;

%% Simulation cycle
disp("Simulation started");
while true
    % Check if simulation ended
    [~, info] = simxGetInMessageInfo(sim, clientID, remApi.simx_headeroffset_server_state);

    if info == 0
        disp("Simulation ended");
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
%     [~, ang] = simxGetObjectOrientation(sim, clientID, uav, -1, remApi.simx_opmode_buffer);
    [~, quat] = simxGetObjectQuaternion(sim, clientID, uav, -1, remApi.simx_opmode_buffer);
    [~, v, ang_v] = simxGetObjectVelocity(sim, clientID, uav, remApi.simx_opmode_buffer);
    
    %% Convert from quaternion ro RPY
    qw = quat(1); qx = quat(2); qy = quat(3); qz = quat(4);
    
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
    
    phi = roll; theta = pitch; psi = yaw;
    
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
       disp("Trajectory tracking");
    end
    
    if traj_track
        cx = 4.060; cxd = 1.015; lx = 0.01;
        cy = 4.060; cyd = 1.015; ly = 0.01;
        cz = 4.060; czd = 1.015; lz = 0.01; 
    end
    
    %% Compute control
    
    % Compute errors
    e_x = p_des(1) - p(1);
    e_y = p_des(2) - p(2);
    e_z = p_des(3) - p(3);
    
    % Update integrals
    chi_x = chi_x + e_x * dt;
    chi_y = chi_y + e_y * dt;
    chi_z = chi_z + e_z * dt;
    
    % Altitude Controller
    % altitude speed tracking error
    e_z_d = cz*e_z + lz*chi_z + v_des(3) - v(3); 
    
    % thrust control input
    T = (m / (cos(phi)*cos(theta))) * ...
        ( g + (1 - cz^2 + lz)*e_z + (cz + czd)*e_z_d - cz*lz*chi_z );
    
    % Position Controller
    % speed tracking errors
    e_x_d = cx*e_x + lx*chi_x + v_des(1) - v(1); 
    e_y_d = cy*e_y + ly*chi_y + v_des(2) - v(2); 
    
    % desired roll and pitch angles
    theta_d = (m/T) * ...
        ( (1 - cx^2 + lx)*e_x + (cx + cxd)*e_x_d - cx*lx*chi_x );
    phi_d = -(m/T) * ...
        ( (1 - cy^2 + ly)*e_y + (cy + cyd)*e_y_d - cy*ly*chi_y );
    
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

%% Disconnect from CS
sim.simxGetPingTime(clientID);
sim.simxFinish(clientID);
sim.delete();

disp('Connection to remote API server closed');