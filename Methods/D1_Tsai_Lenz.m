function T = D1_Tsai_Lenz(A, B)
%
% This function solves the problem AX=XB based on the angle-axis 
% representation according to the algorithm presented in: 
% 
% R. Y. Tsai, R. K. Lenz, "A new technique for fully autonomous and 
% efficient 3D robotics hand/eye calibration," IEEE Transactions on
% Robotics and Automation, vol. 5, no. 3, pp. 345-358, 1989.
%

  n = length(A);
  
  S = zeros(3*n, 3);
  v = zeros(3*n, 1);
  
  % Calculate Rotation R
  
  for i = 1:n
    a = AxisRotNorm(A{i});
    b = AxisRotNorm(B{i});
    S(3*i-2:3*i,:) = skew(a+b);
    v(3*i-2:3*i,:) = a-b;
  end
  
  x = S\v;
  theta = 2*atan(norm(x));
  x = x/norm(x);
  R = (eye(3)*cos(theta) + sin(theta)*skew(x) + (1-cos(theta))*x*x')';
  
  % Compute translation t and put everything together to form T
  
  C = zeros(3*n,3);
  d = zeros(3*n,1);
  
  for i = 1:n
    C(3*i-2:3*i,:) = eye(3) - A{i}(1:3,1:3);
    d(3*i-2:3*i,:) = A{i}(1:3,4) - R*B{i}(1:3,4);
  end
  
  t = C\d;
  T = [R t; 0 0 0 1];
 
end
