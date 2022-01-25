function T = D2_Shiu_Ahmad(A, B)
%
% This function Solves the problem AX=XB based 
% on representing rotation by axis angle described in: 
% 
% Y. C. Shiu and S. Ahmad, "Calibration of wrist-mounted robotic sensors
% by solving homogeneous transform equations of the form ax= xb." IEEE
% Transactions on robotics and automation, vol. 5, no. 1, pp. 16�29, 1989.
%
% Y. Shiu and S. Ahmad, "Finding the mounting position of a sensor
% by solving a homogeneous transform equation of the form ax= xb,"
% in Proceedings. 1987 IEEE International Conference on Robotics and
% Automation, vol. 4, 1987, pp. 1666-1671.
%
  n = length(A);
  
  AA = zeros(9*(n-1),2*n);
  bb = zeros(9*(n-1),1);
  
  %Calculate Rotation R
  for i = 1:n
    a1 = AxisRotNorm(A{i});
    b1 = AxisRotNorm(B{i});
    v = cross(b1,a1);
    w = atan2(norm(v),b1'*a1);
    v = v/norm(v);
    XP = (eye(3)*cos(w) + sin(w)*skew(v) + (1-cos(w))*v*v');
    [Ai,bi] = ShiuAhmadMatrix(a1,XP);
    if i == 1
      AA(:,1:2) = repmat(-Ai,n-1,1);
      bb(:,1) = repmat(-bi,n-1,1);
    else
      AA(9*(i-2)+1:9*(i-1),2*i-1:2*i) = Ai;
      bb(9*(i-2)+1:9*(i-1),1) = bb(9*(i-2)+1:9*(i-1),1) + bi;
    end
  end
  
  beta = AA\bb;
  theta = atan2(beta(2*n), beta(2*n-1));
  RA = (eye(3)*cos(theta) + sin(theta)*skew(a1) + (1-cos(theta))*(a1*a1'));
  R = RA*XP;
  
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

function [A, b] =  ShiuAhmadMatrix(ka1, X)

  A = zeros(9,2);
  b = zeros(9,1);
  
  A(1,1) = X(1,1)-ka1(1)*X(:,1)'*ka1;
  A(2,1) = X(1,2)-ka1(1)*X(:,2)'*ka1;
  A(3,1) = X(1,3)-ka1(1)*X(:,3)'*ka1;
  
  A(4,1) = X(2,1)-ka1(2)*X(:,1)'*ka1;
  A(5,1) = X(2,2)-ka1(2)*X(:,2)'*ka1;
  A(6,1) = X(2,3)-ka1(2)*X(:,3)'*ka1;
  
  A(7,1) = X(3,1)-ka1(3)*X(:,1)'*ka1;
  A(8,1) = X(3,2)-ka1(3)*X(:,2)'*ka1;
  A(9,1) = X(3,3)-ka1(3)*X(:,3)'*ka1;
  
  n = cross(X(:,1),ka1);
  o = cross(X(:,2),ka1);
  a = cross(X(:,3),ka1);
  
  A(1,2) = -n(1);
  A(2,2) = -o(1);
  A(3,2) = -a(1);
  
  A(4,2) = -n(2);
  A(5,2) = -o(2);
  A(6,2) = -a(2);
  
  A(7,2) = -n(3);
  A(8,2) = -o(3);
  A(9,2) = -a(3);
  
  n = X(:,1);
  o = X(:,2);
  a = X(:,3);
  
  b(1) = -ka1(1)*n'*ka1;
  b(2) = -ka1(1)*o'*ka1;
  b(3) = -ka1(1)*a'*ka1;
  
  b(4) = -ka1(2)*n'*ka1;
  b(5) = -ka1(2)*o'*ka1;
  b(6) = -ka1(2)*a'*ka1;
  
  b(7) = -ka1(3)*n'*ka1;
  b(8) = -ka1(3)*o'*ka1;
  b(9) = -ka1(3)*a'*ka1;
  
end
