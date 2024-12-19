%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');


%% 1.
% You will need to define all the model matrices, and fill the so called iTj matrices inside BuildTree() function 
% Be careful to define the z-axis coinciding with the joint rotation axis,
% and such that the positive rotation is the same as showed in the CAD model you received.
geom_model = BuildTree();

% Useful initizializations
numberOfLinks = 7 ;                     % number of manipulator's links.
linkType = [0 0 0 0 0 0 0];                  % boolean that specifies two possible link types: Rotational, Prismatic.
bri= zeros(3,numberOfLinks);        % Basic vector of i-th link w.r.t. base
bTi = zeros(4,4,numberOfLinks);     % Trasformation matrix i-th link w.r.t. base

linkNumber_i = randi([1 7]); %initialize i
linkNumber_j = randi([1 7]); %initialize j
iTj = zeros(4,4,1);
% Initial joint configuration 
q = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];

% Q1.1 and Q1.2
biTei = GetDirectGeometry(q, geom_model, linkType)


%Q1.3

for i = 1:numberOfLinks
    if (i==7)
        res(:,:,i) = GetTransformationWrtBase(biTei, i) %we used if just to reveal final results not to get through every steps
        
    else
        res(:,:,i) = GetTransformationWrtBase(biTei, i); %hide steps
        
    end
end

s = 1;
for s = s:1
frm(:,:,s) = GetFrameWrtFrame(s,biTei) % user has to insert i and j, and the function will calculate the H.
end

for i = 1:numberOfLinks
    if (i == 7)
        bri(:,i) = GetBasicVectorWrtBase(biTei, i) %show only final results
    else 
        bri(:,i) = GetBasicVectorWrtBase(biTei, i); %hide the steps.
        
    end
end


% Q1.4
% Hint: use plot3() and line() matlab functions. 
q = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];
qf = [2,2,2,2,2,2,2];
numberOfSteps = 100;
linkType = [0 0 0 0 0 0 0];
biTei = GetDirectGeometry(q, geom_model, linkType); %calculate get direct geometry. 
k = (qf - q)/numberOfSteps; %calculate the difrrence over steps to use it as counter
for j = 1:numberOfSteps %Move function
    biTei = GetDirectGeometry(k, biTei, linkType);
    for i = 1:7
        res(:,:,i) = GetTransformationWrtBase(biTei, i); %extract GetTransofrmationwithBase
        bri(:,i) = GetBasicVectorWrtBase(biTei, i); %extract GetTransonVectorWthBase
    end
plot3(bri(1,:),bri(2,:),bri(3,:),'ro')
hold on;
grid on;
line(bri(1,:),bri(2,:),bri(3,:));
hold off;
pause(0.1);
end 
hold off

%Q1.4.2
q1 = [1.3, 0, 1.3, 1.7, 1.3, 0.8, 1.3];
qf1 = [2, 0, 1.3, 1.7, 1.3, 0.8, 1.3];
numberOfSteps = 100;
linkType = [0 0 0 0 0 0 0];
biTei1 = GetDirectGeometry(q1, geom_model, linkType);
h = (qf1 - q1)/numberOfSteps;
for j = 1:numberOfSteps
    biTei1 = GetDirectGeometry(h, biTei1, linkType);
    for i = 1:7
        res1(:,:,i) = GetTransformationWrtBase(biTei1, i);
        bri1(:,i) = GetBasicVectorWrtBase(biTei1, i);
    end 
plot3(bri1(1,:),bri1(2,:),bri1(3,:),'ro')
hold on;
grid on;
line(bri1(1,:),bri1(2,:),bri1(3,:));
hold off;
pause(0.1);
end 
hold off

%Q1.4.3
q2 = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
qf2 = [2, 2, 2, 2, 2, 2, 2];
numberOfSteps = 100;
linkType = [0 0 0 0 0 0 0];
biTei2 = GetDirectGeometry(q2, geom_model, linkType);
g = (qf2 - q2)/numberOfSteps;
for j = 1:numberOfSteps
    biTei2 = GetDirectGeometry(g, biTei2, linkType);
    for i = 1:7
        res2(:,:,i) = GetTransformationWrtBase(biTei2, i);
        bri2(:,i) = GetBasicVectorWrtBase(biTei2, i);
    end 
plot3(bri2(1,:),bri2(2,:),bri2(3,:),'ro')
hold on;
grid on;
line(bri2(1,:),bri2(2,:),bri2(3,:));
hold off;
pause(0.1);
end 
hold off

%Q1.5.1
q1 = [0, 0, 0, 0, 0, 0, 0]; % we initialize q1 for base configuration
qf1 = [pi/4, 0, 0, 0, 0, 0, 0]; %change q1 for 45°
numberOfSteps = 100;
linkType = [0 0 0 0 0 0 0];
biTei1 = GetDirectGeometry(q1, geom_model, linkType);
h = (qf1 - q1)/numberOfSteps;
for j = 1:numberOfSteps
    biTei1 = GetDirectGeometry(h, biTei1, linkType);
    for i = 1:7
        res1(:,:,i) = GetTransformationWrtBase(biTei1, i);
        bri1(:,i) = GetBasicVectorWrtBase(biTei1, i);
    end 
plot3(bri1(1,:),bri1(2,:),bri1(3,:),'ro')
hold on;
grid on;
line(bri1(1,:),bri1(2,:),bri1(3,:));
hold off;
pause(0.1);
end 
hold off
%Q1.5.2
q1 = [0, 0, 0, 0, 0, 0, 0];
qf1 = [pi/4, 0, 0, 0, 0, pi/2, 0]; %change qf6 for 90°
numberOfSteps = 100;
linkType = [0 0 0 0 0 0 0];
biTei1 = GetDirectGeometry(q1, geom_model, linkType);
h = (qf1 - q1)/numberOfSteps;
for j = 1:numberOfSteps
    biTei1 = GetDirectGeometry(h, biTei1, linkType);
    for i = 1:7
        res1(:,:,i) = GetTransformationWrtBase(biTei1, i);
        bri1(:,i) = GetBasicVectorWrtBase(biTei1, i);
    end 
plot3(bri1(1,:),bri1(2,:),bri1(3,:),'ro')
hold on;
grid on;
line(bri1(1,:),bri1(2,:),bri1(3,:));
hold off;
pause(0.1);
end 
hold off


%Q1.5.3
q1 = [0, 0, 0, 0, 0, 0, 0];
qf1 = [pi/4, 0, 0,-pi/4, 0, pi/2, 0]; %change af14 for -45°
numberOfSteps = 100;
linkType = [0 0 0 0 0 0 0];
biTei1 = GetDirectGeometry(q1, geom_model, linkType);
h = (qf1 - q1)/numberOfSteps;
for j = 1:numberOfSteps
    biTei1 = GetDirectGeometry(h, biTei1, linkType);
    for i = 1:7
        res1(:,:,i) = GetTransformationWrtBase(biTei1, i);
        bri1(:,i) = GetBasicVectorWrtBase(biTei1, i);
    end 
plot3(bri1(1,:),bri1(2,:),bri1(3,:),'ro')
hold on;
grid on;
line(bri1(1,:),bri1(2,:),bri1(3,:));
hold off;
pause(0.1);
end 
hold off



