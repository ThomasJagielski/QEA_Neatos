%% Bridge of Doom

% @author: Thomas Jagielski
% Date: April 14, 2019
close all
clear all
% Load the collected data from the encoder
encoder_data = load('eCurve.mat');

%% Encoder Data
% Find Wheel Velocity
diffEncoder = diff(encoder_data.dataset(:,1:3));
V_l_encoder = diffEncoder(:,2)./diffEncoder(:,1);
V_r_encoder = diffEncoder(:,3)./diffEncoder(:,1);
time_encoder = encoder_data.dataset(:,1);

% Initialize the data to after start
V_l_encoder = V_l_encoder(19:end-4);
V_r_encoder = V_r_encoder(19:end-4);
time_encoder = time_encoder(19:end-4)-time_encoder(19);

% Find the linear speed
encoder_linear_speed = (V_l_encoder + V_r_encoder)/2;

% Find angular velocity
d = 0.24;%wheel distance [m] - changes for every robot
omega_encoder = (V_r_encoder - V_l_encoder)./d;

%% Find the Position
pos_x = 0;
pos_y = 0;
theta = -pi/2;

exp_pos = zeros(length(V_l_encoder),2);
theta_mat = zeros(length(V_l_encoder),1);

dt = diff(time_encoder);

for p=1:length(V_l_encoder)
    if p > 1
        exp_pos(p,1) = exp_pos(p-1,1)+(encoder_linear_speed(p,:)*dt(p,:)*cos(theta));%the x-coordinate
        exp_pos(p,2) = exp_pos(p-1,2)+(encoder_linear_speed(p,:)*dt(p,:)*sin(theta));%the y-coordinate
        theta = theta + omega_encoder(p,:)*dt(p,:);%the heading
        theta_mat(p) = theta; %save the angles for various time steps
    else
        exp_pos(p,1) = pos_x + (encoder_linear_speed(p,:)*dt(p,:)*cos(theta));%the x-coordinate
        exp_pos(p,2) = pos_y + (encoder_linear_speed(p,:)*dt(p,:)*sin(theta));%the y-coordinate
        theta = theta + omega_encoder(p,:)*dt(p,:);%the heading
        theta_mat(p) = theta; %save the angles for various time steps
    end
end

%% Deliverable 6
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
h1 = plot3(r(1,:),r(2,:),r(3,:)), axis([-1.25 1.25 -1.25 1.7 -2 2]);

% Plot Tangent Vectors
for k = linspace(1,length(u),6)% loop through each of the points
    n = round(k);% round to the nearest integer
    h2 = quiver3(r(1,n),r(2,n),r(3,n),That(1,n),That(2,n),That(3,n),'r'); % plot the unit tangent vectors for theoretical path
end

h3 = plot(exp_pos(:,1),exp_pos(:,2),'m --');

for l = linspace(1,length(theta_mat),6)
    h4 = quiver(exp_pos(l,1),exp_pos(l,2),cos(theta_mat(l)),sin(theta_mat(l)),'g --');% plot the unit tangent vectors for experiemental path
end

title('Path of Robot Using Parametric Curve')
xlabel('X Position [m]')
ylabel('Y Position [m]')
legend([h1,h2,h3,h4],'Theoretical Path','Theoretical Path Tangent Vectors','Experimental Path','Theoretical Path Tangent Vectors')
hold off;

%% Devliverable 7
syms t;
a = 0.4;
l = 0.4;
alpha = 1/8;
sym_r = [-2.*a*((l-cos(alpha*t))*cos(alpha*t)+(1-l));2.*a*(l-cos(alpha*t))*sin(alpha*t);0];% define the position vector
% substitute in the time steps
r_error = double(subs(sym_r,time_encoder'));

distance_xy = r_error(1:2,1:end-1)' - exp_pos;% find the difference in x and y coordinates

distance = zeros(length(distance_xy),1);% use the distance formula to find the expected vs theoretical position at a given time

for k=1:length(distance_xy)
    distance(k) = sqrt(distance_xy(k,1)^2 + distance_xy(k,2)^2); 
end

figure()
plot(time_encoder(1:end-1,:),distance)
xlabel('Time [s]')
ylabel('Error [m]')
title('Error with Respect to Time')
mean_error = mean(distance) % in meters