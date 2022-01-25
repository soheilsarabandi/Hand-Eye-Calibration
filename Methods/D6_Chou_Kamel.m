function T = D6_Chou_Kamel(A, B)
%
% This function solves the problem AX=XB based on a quaternion 
% representation of rotations described in: 
% 
% J. C. Chou and M. Kamel, "Finding the position and orientation of
% a sensor on a robot manipulator using quaternions," The International
% Jou rnal of Robotics Research, vol. 10, no. 3, pp. 240-254, 1991.
%

  n = length(A);
  AA = zeros(4*n,4);
  
  % Compute rotation R
  
  for i = 1:n
    a = Mat2Quat(A{i}(1:3,1:3));
    b = Mat2Quat(B{i}(1:3,1:3));
    AA(4*i-3:4*i,:) = [a(1), -a(2:4)'; a(2:4), a(1)*eye(3)+skew(a(2:4))] -...
                      [b(1), -b(2:4)'; b(2:4), b(1)*eye(3)-skew(b(2:4))];
  end
  
  [~,~,V] = svd(AA);
  V = V(:,4);
  R = Quat2Mat(V);
  
  % Compute translation t and put everything together to form T
  
  C = zeros(3*n,3);
  d = zeros(3*n,1);
  
  for i = 1:n
    C(3*i-2:3*i,:) = eye(3) - A{i}(1:3,1:3);
    d(3*i-2:3*i,:) = A{i}(1:3,4) - R*B{i}(1:3,4);
  end
  
  t = C\d;
  T = [R t;0 0 0 1];
    
end
