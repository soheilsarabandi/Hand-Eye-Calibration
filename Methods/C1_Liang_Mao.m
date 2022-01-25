function T = C1_Liang_Mao(A, B)
%
% This function solves the problem AX=XB based by expressing the problems 
% in terms of Kronecker products as described in: 
% 
% R.-h. Liang and J.-f. Mao, "Hand-eye calibration with a new linear
% decomposition algorithm," Journal of Zhejiang University-SCIENCE A,
% vol. 9, no. 10, pp. 1363-1368, 2008.
%

  n = length(A);

  % Compute rotation R
  
  AA = zeros(9*n,9);
  % b = zeros(9*n,1);  <- not used!!
  
  for i = 1:n
    Ra = A{i}(1:3,1:3);
    Rb = B{i}(1:3,1:3);
    AA(9*i-8:9*i,:) = kron(Ra, eye(3)) + kron(-eye(3), Rb');
  end
  
  [~,~,V] = svd(AA);
  X = V(:, end);
  R = reshape(X(1:9), 3, 3)';
  dR=det(R);
  R = sign(dR)/abs(dR)^(1/3)*R;
  
  [U,~,V] = svd(R);
  
  if det(R)<0
    R = U*diag([1 1 -1])*V';
  else
    R = U*V';
  end
  
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
