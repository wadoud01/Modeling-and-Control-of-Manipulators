function J = GetJacobian(bTe,biTei)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix


for i = 1:7
    if (i==7)
        bTe(:,:,i) = GetTransformationWrtBase(biTei, i); %we used if just to reveal final results not to get through every steps
    else
        bTe(:,:,i) = GetTransformationWrtBase(biTei, i); %hide steps
    end
for s = 1:7
    if (s==7)
         Ja_1(1:3,s) = bTe(1:3,3,s)
         t_1(1:3,s) = bTe(1:3,4,s)
    else
         Ja_1(1:3,s) = bTe(1:3,3,s);
         t_1(1:3,s) = bTe(1:3,4,s);
    end

end

for s = 1:1:7
    J1(:,s) = [Ja_1(1:3,s); cross(Ja_1(1:3,s),(t_1(1:3,7) - t_1(1:3,s)))];
end
J = J1;
end