function robotic_arm_mouse_follow_3link()
%% 3-Link Planar Robotic Arm Mouse-Follow with Trajectories, Angles, and Velocities
clc; clear; close all;

%% Link lengths
L1 = 3; L2 = 2; L3 = 1.5;

%% Time setup
Ts = 0.05; % sample time
maxTime = 30; % seconds
numSteps = maxTime/Ts;

%% Figure setup
fig = figure('Name','3-Link Robotic Arm Mouse Follow','NumberTitle','off',...
    'Color','w','WindowButtonMotionFcn',@mouseMove);
axis equal
axis([-sum([L1,L2,L3])-1 sum([L1,L2,L3])+1 -sum([L1,L2,L3])-1 sum([L1,L2,L3])+1])
grid on
hold on
title('Move the mouse to control the 3-link arm')
xlabel('X'); ylabel('Y');

% Arm plot
jointLine = plot([0 0 0 0],[0 0 0 0],'o-','LineWidth',3,'MarkerSize',6,'MarkerFaceColor','r');
endEffector = plot(0,0,'bo','MarkerSize',8,'MarkerFaceColor','b');

% Trajectory plots
hTarget = plot(0,0,'rx--','LineWidth',1.5,'DisplayName','Target Trajectory');
hActual = plot(0,0,'g.-','LineWidth',1.5,'DisplayName','End-Effector Trajectory');
legend('Location','northwest');

%% Initialize logs
target_traj = [];
actual_traj = [];
jointAngles = []; % [theta1; theta2; theta3]
time_log = [];
endEffectorVel = []; % [vx; vy]
prevX = 0; prevY = 0;

%% Nested function: Mouse callback
function mouseMove(~,~)
    cp = get(gca,'CurrentPoint');
    x = cp(1,1);
    y = cp(1,2);

    %% Inverse Kinematics (3-link planar)
    r = sqrt(x^2 + y^2);
    phi = atan2(y,x);

    D = (r^2 - L1^2 - L2^2)/(2*L1*L2);
    if abs(D) > 1
        return; % target unreachable
    end
    theta2 = atan2(sqrt(1-D^2), D);
    theta1 = phi - atan2(L2*sin(theta2), L1+L2*cos(theta2));
    theta3 = 0; % third link aligned

    %% Forward Kinematics
    x1 = L1*cos(theta1);
    y1 = L1*sin(theta1);
    x2 = x1 + L2*cos(theta1+theta2);
    y2 = y1 + L2*sin(theta1+theta2);
    x3 = x2 + L3*cos(theta1+theta2+theta3);
    y3 = y2 + L3*sin(theta1+theta2+theta3);

    % Update arm plot
    set(jointLine,'XData',[0 x1 x2 x3],'YData',[0 y1 y2 y3])
    set(endEffector,'XData',x3,'YData',y3)

    % Log trajectories
    target_traj = [target_traj [x;y]];
    actual_traj = [actual_traj [x3;y3]];
    jointAngles = [jointAngles [theta1;theta2;theta3]];
    
    % Compute velocities
    if isempty(prevX)
        vx = 0; vy = 0;
    else
        vx = (x3-prevX)/Ts;
        vy = (y3-prevY)/Ts;
    end
    prevX = x3; prevY = y3;
    endEffectorVel = [endEffectorVel [vx; vy]];
    time_log = [time_log now*0]; % placeholder, will replace with proper time after simulation

    % Update trajectory plots
    set(hTarget,'XData',target_traj(1,:),'YData',target_traj(2,:))
    set(hActual,'XData',actual_traj(1,:),'YData',actual_traj(2,:))

    drawnow
end

%% Wait for user to finish (close figure)
uiwait(fig);

%% After figure closed, plot joint angles and velocities
t = 0:Ts:(size(jointAngles,2)-1)*Ts;

figure('Name','Joint Angles and Velocities','NumberTitle','off','Color','w');

subplot(3,1,1)
plot(t, rad2deg(jointAngles(1,:)),'r', t, rad2deg(jointAngles(2,:)),'g', t, rad2deg(jointAngles(3,:)),'b','LineWidth',1.5)
xlabel('Time [s]'); ylabel('Angle [deg]')
legend('\theta_1','\theta_2','\theta_3'); grid on; title('Joint Angles vs Time')

subplot(3,1,2)
plot(t, endEffectorVel(1,:),'b','LineWidth',1.5)
xlabel('Time [s]'); ylabel('Vx [m/s]')
title('End-Effector Linear Velocity X'); grid on

subplot(3,1,3)
plot(t, endEffectorVel(2,:),'m','LineWidth',1.5)
xlabel('Time [s]'); ylabel('Vy [m/s]')
title('End-Effector Linear Velocity Y'); grid on

end
