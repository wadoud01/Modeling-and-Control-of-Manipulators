function [iTj]=GetFrameWrtFrame(s,biTei)
%%% GetFrameWrtFrame function 
% inputs : 
% linkNumber_i :number of ith link 
% linkNumber_j: number of jth link 
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current q.
% The size of biTri is equal to (4,4,numberOfLinks)
% outputs:
% iTj : transformationMatrix in between link i and link j for the
% configuration described in biTei.


i = input('Input value of i :') %ask user for start i
j = input('Input value of j :') %ask user for last joint j
s = i + 1
d = biTei(:,:,i);
iTj = biTei(:,:,s);
for s = (s+1):j
    iTj = iTj * biTei(:,:,s) ; %calculate from frame to frame
end
