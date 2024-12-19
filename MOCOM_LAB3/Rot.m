function Rz = Rot(q_i)
    Rz = [cos(q_i) -sin(q_i) 0;
          sin(q_i) cos(q_i) 0;
          0 0 1 ];
end