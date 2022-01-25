function T = D7_OursCross(AA, BB)
%
% The method we introduce in the paper using cross products

  n = length(AA);

  % The cross products to consider
  c=nchoosek(1:n,2);
  
  m=size(c,1);

  % Calculate Rotation R
  A = zeros(3,n+m);
  B = zeros(3,n+m);
  
  for i = 1:n
    A(:,i) = AxisRot(AA{i});
    B(:,i) = AxisRot(BB{i});
  end
  
  for i=1:m
    A(:,n+i)=cross(A(:,c(i,1)),A(:,c(i,2)));
    B(:,n+i)=cross(B(:,c(i,1)),B(:,c(i,2)));
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

