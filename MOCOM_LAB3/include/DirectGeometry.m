function biTei = DirectGeometry(qi, iTj, linkType)
% DirectGeometry Function 
% inputs: 
% q : current link position;
% biTri is the constant transformation between the base of the link <i>
% and its end-effector; 
% jointType:0 for revolute, 1 for prismatic

% output :
% biTei : transformation between the base of the joint <i> and its end-effector taking 
% into account the actual rotation/traslation of t  he joint
%K = [0 0 1];
V = iTj(1:3,4);
linkType = 0;
if linkType == 0 % rotational
    Rz = [cos(qi) -sin(qi) 0 ; sin(qi) cos(qi) 0  ; 0 0 1 ]; 
    P = iTj(1:3,1:3);
    R = P*Rz;
    biTei = [R V; 0 0 0 1];%transformation matrix

elseif linkType == 1 % prismatic
    biTei = [iTj(:,1) iTj(:,2) iTj(:,3) (iTj(:,4) + K*qi)]; %transformation matrix

end
