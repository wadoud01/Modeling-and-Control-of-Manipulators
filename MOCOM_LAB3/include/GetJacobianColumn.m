function [h] = GetJacobianColumn(bTei, bTe, jointType)
    %Versor of translation or rotation
    ki = bTei( (1 : 3) , 3);
    rei = bTe(1:3, 4) - bTei(1:3, 4);
    
    if(jointType == 1)%Prismatic
       %Jai is a null vector
       Jai = [0; 0; 0];
       %The traslation of the end effector is along ki
       Jli = ki;
    end
    
    if(jointType == 0)%Revolute
       %The rotation is around ki
       Jai = ki;
       %The transl. of the e.e. is along a versor given by the cross product of ki and rei 
       Jli = cross(ki, rei);
    end
    
    h = [Jai; Jli];
end