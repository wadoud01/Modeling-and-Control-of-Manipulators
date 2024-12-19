function [matrix] = Skew(vector)
%%% Skew function
% Function to create a a skew symmetric matrix 
% from a vector [a,b,c]

% Input
% vector: 3*1 vector

% Output
% matrix: 3*3 skrew symmetrix matrix

    matrix = [   0         -vector(3,1)   vector(2,1)
               vector(3,1)     0         -vector(1,1)
              -vector(2,1)  vector(1,1)      0        ];
end
