%% Bridge of Doom

% @author: Thomas Jagielski
% Date: April 11, 2019
close all
clear all
% Initialize the ros connection
rosshutdown
rosinit('10.0.75.2',11311, 'NodeHost','10.0.75.1')

%% Bridge of Doom Function

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