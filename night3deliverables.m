%% Bridge of Doom

% @author: Thomas Jagielski
% Date: April 11, 2019
close all
clear all
% Initialize the ros connection
rosshutdown
rosinit('10.0.75.2',11311, 'NodeHost','10.0.75.1')
% Load the collected data from the encoder
encoder_data = load('eCurve.mat');

%% Deliverable 1
syms u;
a = 0.4;
l = 0.4;
sym_r = [-2.*a*((l-cos(u))*cos(u)+(1-l));2.*a*(l-cos(u))*sin(u);0];% define the position vector
sym_V = diff(sym_r); %Find the linear speed
sym_That = diff(sym_r)/norm(diff(sym_r));% define the unit tangent
sym_Nhat = diff(sym_That)/norm(diff(sym_That));% define the unit normal
sym_Bhat = cross(sym_That, sym_Nhat);%binormal matrix
u = linspace(0,2*pi,100); % define a set of evenly spaced points between 0 and 2*pi
r = double(subs(sym_r,u));
That = double(subs(sym_That,u));
Nhat = double(subs(sym_Nhat,u));
Bhat = double(subs(sym_Bhat,u));
figure()
hold on;
for n = 1:length(u)% loop through each of the points
    plot3(r(1,:),r(2,:),r(3,:)), axis([-1 1 -1 1 -2 2]),hold on % plot the entire curve
    quiver3(r(1,n),r(2,n),r(3,n),That(1,n),That(2,n),That(3,n),'r') % plot the unit tangent
    quiver3(r(1,n),r(2,n),r(3,n),Nhat(1,n),Nhat(2,n),Nhat(3,n),'b') % plot the unit normal
    quiver3(r(1,n),r(2,n),r(3,n),Bhat(1,n),Bhat(2,n),Bhat(3,n),'g'), hold off % plot the unit binormal
    drawnow % force the graphic to update as it goes
end
title('Path of Robot Using Parametric Curve')
xlabel('x - axis')
ylabel('y - axis')
zlabel('z - axis')
legend('Path','That','Nhat','Bhat')
hold off;

%% Deliverable 2
syms t;
a = 0.4;
l = 0.4;
alpha = 1/8;
sym_r = [-2.*a*((l-cos(alpha*t))*cos(alpha*t)+(1-l));2.*a*(l-cos(alpha*t))*sin(alpha*t);0];% define the position vector
sym_V = diff(sym_r); %Find the linear speed
sym_That = diff(sym_r)/norm(diff(sym_r));% define the unit tangent
sym_Nhat = diff(sym_That)/norm(diff(sym_That));% define the unit normal
sym_Bhat = cross(sym_That, sym_Nhat);%binormal matrix
sym_omega = cross(sym_That,diff(sym_That));
t = linspace(0,30,100); % define a set of evenly spaced points between 0 and 2*pi
% Substitute in the values of t into the symbolic expressions
r = double(subs(sym_r,t));
That = double(subs(sym_That,t));
V = double(subs(sym_V,t));
Nhat = double(subs(sym_Nhat,t));
Bhat = double(subs(sym_Bhat,t));
omega = double(subs(sym_omega,t));

d = 0.24; % Distance between NEATO's wheels (m)

V_l = vecnorm(V) - ((omega(3,:) * d) / 2); %Find the left wheel velocity
V_r = vecnorm(V) + ((omega(3,:) * d) / 2); %Find the right wheel velocity

% Find Wheel Velocity
diffEncoder = diff(encoder_data.dataset(:,1:3));
V_l_encoder = diffEncoder(:,2)./diffEncoder(:,1);
V_r_encoder = diffEncoder(:,3)./diffEncoder(:,1);
time_encoder = encoder_data.dataset(:,1);

% Initialize the data to after start
V_l_encoder = V_l_encoder(19:end-4);
V_r_encoder = V_r_encoder(19:end-4);
time_encoder = time_encoder(19:end-5)-time_encoder(19);

figure()
hold on;
plot(t,V_l,'b-')
plot(t,V_r,'r-')
plot(time_encoder,V_l_encoder,'b--')
plot(time_encoder,V_r_encoder,'r--')
xlabel('Time [s]')
ylabel('Wheel Velocity [m/s]')
title('Left and Right Wheel Velocities')
legend('V_l (Theoretical)','V_r (Theoretical)','V_l (encoder)','V_r (encoder)')
hold off;

%% Deliverable 3
% Find the linear speed
encoder_linear_speed = (V_l_encoder + V_r_encoder)/2;

% Find angular velocity
d = 0.24;%wheel distance [m] - changes for every robot
omega_encoder = (V_r_encoder - V_l_encoder)./d;

figure()
hold on
plot(t,vecnorm(V),'-')
plot(time_encoder, encoder_linear_speed,'--')
title('Robot Linear Speed')
xlabel('Time [s]')
ylabel('Linear Speed [m/s]')
legend('Theoretical','Measured')

figure()
hold on;
plot(t,omega(3,:),'-')
plot(time_encoder,omega_encoder,'--')
title('Robot Angular Speed')
xlabel('Time [s]')
ylabel('Angular Speed [rad/s]')
legend('Theoretical','Measured')
hold off;

%% Deliverable 4
% Program to traverse the 'e' shaped bridge of doom

%eCurve()

function eCurve()
% eCurve is a function to drive the path of the 'e' shaped curve for the
% bridge of doom challenge

d = 0.24;% Distance between the wheels [m] - changes for every robot

%Publish the left and right wheel velocities
pubvel = rospublisher('/raw_vel')

%Here we are creating a ROS message
message = rosmessage(pubvel); 

%start a timer
tic 

% Initialize the wheel velocity functions
syms t;
a = 0.4;
l = 0.4;
alpha = 1/8;
sym_r = [-2.*a*((l-cos(alpha*t))*cos(alpha*t)+(1-l));2.*a*(l-cos(alpha*t))*sin(alpha*t);0];% define the position vector
sym_V = diff(sym_r); %Find the linear velocity symbolically
sym_That = diff(sym_r)/norm(diff(sym_r));%Find the unit tangent symbolically
sym_omega = cross(sym_That,diff(sym_That));%Find the angular velocity symbolically

while 1
    % Pause for encoder collection and ensure the toc will never read 0.0
    pause(0.01)
    % Collect the current time since start and set it equal to 't'
    t = toc;
    if t < 27.5
        % Substitute in the current time for 't' in the 'V' expression
        V = double(subs(sym_V,t));
        % Substitute in the current time for 't' in the angular velocity
        % expression
        omega = double(subs(sym_omega,t));

        V_l = vecnorm(V) - ((omega(3,:) * d) / 2); %Find the left wheel velocity
        V_r = vecnorm(V) + ((omega(3,:) * d) / 2); %Find the right wheel velocity

        message.Data = [V_l,V_r]; % set wheel velocities to the computed V_l and V_r velocities computed for
        % a specific time
        send(pubvel, message); % send new wheel velocities
    else 
        % If the time is greater than 27.5 seconds, publish [0.0,0.0] as
        % the wheel speed
        message.Data = [0,0]; % set wheel velocities to zero if we have reached the desire distance    
        send(pubvel, message); % send new wheel velocities
        break
    end 
end
end