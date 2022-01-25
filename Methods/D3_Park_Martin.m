function T = D3_Park_Martin(A,B)
%
% This function solves the problem AX=XB based on the algorithm presented in: 
%
% F. C. Park and B. J. Martin, "Robot sensor calibration: solving ax= xb on
% the euclidean group," IEEE Transactions on Robotics and Automation,
% vol. 10, no. 5, pp. 717-721, 1994.
%
  n = length(A);
  
  % Calculate Rotation R
  if n==2
    A1=zeros(3,3);
    A1(:,1)=AxisRotNorm(A{1});
    A1(:,2)=AxisRotNorm(A{2});
    A1(:,3)=cross(A1(:,1),A1(:,2));
    
    B1=zeros(3,3);
    B1(:,1)=AxisRotNorm(B{1});
    B1(:,2)=AxisRotNorm(B{2});
    B1(:,3)=cross(B1(:,1),B1(:,2));
    
    R=A1/B1;
  else
    M = zeros(3,3);
    A1 = zeros(3,n);
    B1 = zeros(3,n);
    
    for i = 1:n
      A1(:,i) = AxisRotNorm(A{i});
      B1(:,i) = AxisRotNorm(B{i});
      
      M  = M + B1(:,i)*A1(:,i)';
    end
    
    [U, V] = eig(M'*M);
    V = diag(diag(V).^(-1/2));
    R = U*V*U'*M';
  end
  
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
