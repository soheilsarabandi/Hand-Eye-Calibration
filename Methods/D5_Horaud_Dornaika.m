function T = D5_Horaud_Dornaika(A, B)
%
% This function solves the problem AX=XB based on the quaternion representation
% described in:
%
% R. Horaud and F. Dornaika, "Hand-eye calibration," The International
% Journal of Robotics Research, vol. 14, no. 3, pp. 195-210, 1995.
%

  n = length(A);
  AA = zeros(4,4);
  
  %Calculate Rotation R
  for i = 1:n
    a = AxisRotNorm(A{i});
    b = AxisRotNorm(B{i});
    AA = AA + ...
      [0,         -a(1)+b(1), -a(2)+b(2), -a(3)+b(3);...
       a(1)-b(1),  0,         -a(3)-b(3),  a(2)+b(2);...
       a(2)-b(2),  a(3)+b(3),  0,         -a(1)-b(1);...
       a(3)-b(3), -a(2)-b(2),  a(1)+b(1),  0         ]'*...
      [0,         -a(1)+b(1), -a(2)+b(2), -a(3)+b(3);...
       a(1)-b(1),  0,         -a(3)-b(3),  a(2)+b(2);...
       a(2)-b(2),  a(3)+b(3),  0,         -a(1)-b(1);...
       a(3)-b(3), -a(2)-b(2),  a(1)+b(1),  0         ];
  end
  
  [u,~]=eig(AA);
  R = Quat2Mat(u(:,1));
  
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
