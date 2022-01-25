function T = D7_Ours(AA, BB)
%
% The method we introduce in the paper.

  n = length(AA);

  % Calculate Rotation R
  A = zeros(3,n);
  B = zeros(3,n);
  
  for i = 1:n
    A(:,i) = AxisRot(AA{i});
    B(:,i) = AxisRot(BB{i});
  end

  R = A/B;

  % Iterative orthonormalization method
  I=eye(3);
  S=R'*R; R=R*((3*I)+S)/(I+(3*S));
  S=R'*R; R=R*((3*I)+S)/(I+(3*S));

  % Compute translation t and put everything together to form T
  C = zeros(3*n,3);
  d = zeros(3*n,1);
  
  for i = 1:n
    C(3*i-2:3*i,:) = eye(3) - AA{i}(1:3,1:3);
    d(3*i-2:3*i,:) = AA{i}(1:3,4) - R*BB{i}(1:3,4);
  end
  
  t = C\d;
  T = [R,t; 0 0 0 1];
end

