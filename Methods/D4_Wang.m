function T = D4_Wang(A,B)
%
% This function Solves the problem AX=XB based on the algorithm described in:
% 
% C.-C. Wang, "Extrinsic calibration of a vision sensor mounted on a
% robot," IEEE Transactions on Robotics and Automation, vol. 8, no. 2, pp.
% 161-175, 1992.
%

  n = length(A);

  K = zeros(3,n-1);
  theta = zeros(1,n-1);
  
  % Compute rotation R
  
  for i = 1:n-1
    a1 = AxisRotNorm(A{i});
    b1 = AxisRotNorm(B{i});
    a2 = AxisRotNorm(A{i+1});
    b2 = AxisRotNorm(B{i+1});
    if norm(a1-b1)~=0 && norm(a2-b2)~=0 && norm(cross((a1-b1),(a2-b2)))~=0
      %disp('case 1');
      r = cross((a1-b1),(a2-b2));
      r = r/norm(r);
      theta(i) = sign((a1-b1)'*cross(r,a1+b1))*atan(norm(a1-b1)/norm(cross(r,a1+b1)))+...
        sign((a2-b2)'*cross(r,a2+b2))*atan(norm(a2-b2)/norm(cross(r,a2+b2)));
      K(:,i) = r;
    elseif norm(a1-b1)~=0 && norm(a2-b2)~=0 && norm(cross((a1-b1),(a2-b2)))==0
       %disp('case 2');
      r = -cross(cross(a1,a2),cross(b1,b2));
      r = r/norm(r);
      theta(i) = acos(cross(a1,a2)'*cross(b1,b2));
      K(:,i) = r;
    elseif norm(a1-b1)==0 && norm(a2-b2)~=0
      %disp('case 3');
      r = (a1+b1)/2;
      r = r/norm(r);
      theta(i) = 2*sign((a2-b2)'*cross(r,a2+b2))*atan(norm(a2-b2)/norm(cross(r,a2+b2)));
      K(:,i) = r;
    elseif norm(a1-b1)~=0 && norm(a2-b2)==0
      %disp('case 4');
      r = (a2+b2)/2;
      r = r/norm(r);
      theta(i) = 2*sign((a1-b1)'*cross(r,a1+b1))*atan(norm(a1-b1)/norm(cross(r,a1+b1)));
      K(:,i) = r;
    elseif norm(a1-b1)==0 && norm(a2-b2)==0
      %disp('case 5');
      r = [1;0;0];
      theta(i) = 0;
      K(:,i) = r;
    end
  end
  
  K = [K;theta];
  r=mean(K,2);
  sc = norm(r(1:3));
  r = r/sc;
  R = eye(3)*cos(r(4)) + (1-cos(r(4)))*r(1:3)*r(1:3)' + skew(r(1:3))*sin(r(4));
  
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
