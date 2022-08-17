%this fct allows to calculate M forward kinematic matrix
% and J the jacobian of the forward kinematic matrix needed for force calculations

syms o1 o2 o3 l1 l2 l3
%omega/alpha/a/d
matrix = [
    o1  0 l1 0;
    -pi/2  -pi/2 0 0;
    o2 0 l2 0;
    o3 0 l3 0];
M = DH_HTM(matrix,'r')

J = jacobian([M(1,4),M(2,4),M(3,4)],[o1,o2,o3])




