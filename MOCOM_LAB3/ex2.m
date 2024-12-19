%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
clc;
clear;
close("all");
addpath('include');
model = load("panda.mat"); % don't worry about the warnings

% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;

% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';

% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

% Initial transformation from <base> to <e-e>
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7'); %useful for save initial end-effector orientation w.r.t robot base
% END-EFFECTOR Goal definition 
bOge = [0.6; 0.4; 0.4]; %position of the goal in space 
eRge = bTe(1:3,1:3) * Rot(-(pi/4)); % Calculating the rotation matrix using function ROt and angle theta -45°
bTge(1:4,1:4) = [eRge bOge;0 0 0 1] %building transformation matrix by sticking rotation matrix and postion bOge.


% TOOL Goal definition
bOgt = [0.6; 0.4; 0.4];
eTt = [eye(3) [0,0,0.2]'; 0 0 0 1]; %building the transformation matrix of the tool
bTt = bTe * eTt %finding the transformation matrix from base to tool by multiplying two matrices 
tRg = bTt(1:3,1:3) * Rot(-(pi/4));
bTgt(1:4,1:4) = [tRg bOgt;0 0 0 1]

% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
v = zeros(6,1);
error_linear = zeros(3,1);
error_angular = zeros(3,1); 
% Start the inverse kinematic control 
tool = true; % change to true for using the tool 
q = q_init;

for i = t
    %% Compute the cartesian error to reach the goal
    if tool == true %compute the error between the tool frame and goal frame
        
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bTt = bTe * eTt;
        % Computing jacobian of end effector w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT

        % rigid body trasformation matrix from e-e frame to rigid-tool frame proected on base
        rbm_ee = [eye(3) zeros(3);-Skew(bTe(1:3,1:3) * eTt(1:3,4)) eye(3) ];

        % Computing the jacobian matrix 
        bJt = rbm_ee * bJe;  

        % compute the error committed angular and linear.
        rho = ComputeInverseAngleAxis(bTt(1:3, 1:3),tRg);
        error_angular = rho;
        error_linear = bTgt(1:3,4) - bTt(1:3,4);

         
        
    else % compute the neerror between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        rho = ComputeInverseAngleAxis(bTe(1:3, 1:3),eRge);
        error_angular = rho;
        error_linear = bTge(1:3,4) - bTe(1:3,4);
    end
    


%% Compute the reference velocities
    a_v = angular_gain * error_angular; %angular velocity is multp of angular error and the gain
    l_v = linear_gain * error_linear; %linear velocity is multp of linear error and the gain.
    v = [a_v; l_v];
   
    %% Compute desired joint velocities
    if tool == true
        q_dot = pinv(bJt) * v
    else
        q_dot = pinv(bJe) * v;
    end
%% Simulate the robot - implement the function KinematicSimulation()

    q = KinematicSimulation(q(1:7), q_dot, ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    show(model.franka,[q',0,0],'visuals','on');%switch visuals to off for seeing only the frames
    hold on
    if tool == true
        plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
        plot3(bOgt(1),bOgt(2),bOgt(3),'ro','LineWidth',5);
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOge(1),bOge(2),bOge(3),'ro','LineWidth',5);
    end
    drawnow
    if(norm(v) < 0.01)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
   if i < t_end
        % function that clean the window.
        cla();
    end
end
hold off

%% Q3.6
% new TOOL goal definition
bTg = [0.9986 -0.0412 -0.0335 0.6;
       0.0329 -0.0163  0.9993 0.4; 
      -0.0417 -0.9990 -0.0149 0.4; 
       0       0       0      1];
bOgt = bTg(1:3,4); %postion of the goal


q = q_init; % inizialzing q as the previous one.
eTt = [eye(3) [0,0,0.2]'; 0 0 0 1]; %transformation matrix for the tool, translation with distance 0.2

figure
for i = t
    %  Computing transformation matrix from base to end effector in this
    %  case tool = true.
    bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
    bTt = bTe * eTt;
    % Computing end effector jacobian w.r.t. base
    tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
    bJe = tmp(1:6,1:7); %DO NOT EDIT 
    % rigid body trasformation matrix from e-e frame to rigid-tool frame proected on base
    rbm_ee = [         eye(3)                        zeros(3);
                   -Skew(bTe(1:3,1:3) * eTt(1:3,4))        eye(3)      ];
    % Jacobian matrix from base to rigid-tool
    bJt = rbm_ee * bJe;  
    % compute the error committed at the current step
    rho = ComputeInverseAngleAxis(bTt(1:3, 1:3),tRg);
        error_angular = rho;
        error_linear = bTgt(1:3,4) - bTt(1:3,4);
  
    %% Compute the reference velocities
    a_l = angular_gain * error_angular; %amgular velocity
    l_v = linear_gain * error_linear; %linear velocity
    v = [a_l; l_v]; %
   
    %% Compute desired joint velocities 
    q_dot = pinv(bJt) * v;
    
    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot, ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    show(model.franka,[q',0,0],'visuals','on');%switch visuals to off for seeing only the frames
    hold on
    plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
    plot3(bOgt(1),bOgt(2),bOgt(3),'ro','LineWidth',5);
    drawnow
    if(norm(v) < 0.09) %making norm bigger to avoid errors.
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
    if i < t_end
        % function that clean the window.
        cla();
    end
end
hold off




