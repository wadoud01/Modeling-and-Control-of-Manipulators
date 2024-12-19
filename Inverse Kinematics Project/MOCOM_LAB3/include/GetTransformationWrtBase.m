function [bTi] = GetTransformationWrtBase(biTei, linkNumber)
%%% GetTransformatioWrtBase function
% inputs :
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current q.
% The size of biTri is equal to (4,4,numberOfLinks)
% linkNumber for which computing the transformation matrix
% outputs
% bTi : transformation matrix from the manipulator base to the ith joint in
% the configuration identified by biTei.


bTi = biTei(:,:,1);%initialize bTi

if (linkNumber == 1)
    bTi = biTei(:,:,1);
else
    for s = 2:linkNumber
        bTi_s = biTei(:,:,s);
        bTi = bTi*bTi_s;
    end

end
