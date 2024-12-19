%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix
numberOfLinks = 7;
addpath('include');
% The same model of assignment 2
geom_model = BuildTree();
 % number of manipulator's links.
linkType = zeros(numberOfLinks,1); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base
jointType = zeros(7,1);


% Initial joint configuration 
q_1 = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];
q_2 = [1.3, 0.4, 0.1, 0, 0.5, 1.1, 0];
q_3 = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
q_4 = [2, 2, 2, 2, 2, 2, 2];

% Compute direct geometry
biTei_1 = GetDirectGeometry(q_1, geom_model, linkType)
biTei_2 = GetDirectGeometry(q_2, geom_model, linkType)
biTei_3 = GetDirectGeometry(q_3, geom_model, linkType)
biTei_4 = GetDirectGeometry(q_4, geom_model, linkType)
% Compute the transformation w.r.t. the base

for i = 1:7
    if (i==7)
        bTe_1(:,:,i) = GetTransformationWrtBase(biTei_1, i) %we used if just to reveal final results not to get through every steps
        bTe_2(:,:,i) = GetTransformationWrtBase(biTei_2, i)
        bTe_3(:,:,i) = GetTransformationWrtBase(biTei_3, i)
        bTe_4(:,:,i) = GetTransformationWrtBase(biTei_4, i)
    else
        bTe_1(:,:,i) = GetTransformationWrtBase(biTei_1, i); %hide steps
        bTe_2(:,:,i) = GetTransformationWrtBase(biTei_2, i);
        bTe_3(:,:,i) = GetTransformationWrtBase(biTei_3, i);
        bTe_4(:,:,i) = GetTransformationWrtBase(biTei_4, i);
    end
end
% computing end effector jacobian

J1 = GetJacobian(bTe_1,biTei_1)
J2 = GetJacobian(bTe_2,biTei_2)
J3 = GetJacobian(bTe_3,biTei_3)
J4 = GetJacobian(bTe_4,biTei_4)


