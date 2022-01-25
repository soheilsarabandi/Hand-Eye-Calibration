function T = A2_Daniilidis(A,B)
%
% This function solves the problem AX=XB based on the dual quaternion-based 
% method described in: 
%
% K. Daniilidis, "Hand-eye calibration using dual quaternions," The International
% Journal of Robotics Research, vol. 18, no. 3, pp. 286-298, 1999.
%
% K. Daniilidis and E. Bayro-Corrochano, "The dual quaternion approach
% to hand-eye calibration," in proceedings of 13th International Conference
% on Pattern Recognition, vol. 1, 1996, pp. 318-322.
%
  
  n = length(A);
  
  T = zeros(6*n,8);
  
  % Compute rotation R and translation t
  
  for i = 1:n
    a = Hom2DQuat(A{i});
    b = Hom2DQuat(B{i});
    T(6*i-5:6*i,:) = ...
      [a(2:4,1)-b(2:4,1), skew(a(2:4,1)+b(2:4,1)), zeros(3,4);...
      a(2:4,2)-b(2:4,2),  skew(a(2:4,2)+b(2:4,2)), a(2:4,1)-b(2:4,1), skew(a(2:4,1)+b(2:4,1))];
  end
  
  [~,S,V] = svd(T);
  eig = diag(S);
  [~,index] = min(eig(eig~=0));
  
  u1 = V(1:4,index-1);
  v1 = V(5:8,index-1);
  u2 = V(1:4,index);
  v2 = V(5:8,index);
  
  a = u1'*v1;
  b = (u1'*v2+u2'*v1);
  c = u2'*v2 ;
  r = roots([a b c]);
  
  [val,in] = max(r.^2*(u1'*u1) + 2*r*(u1'*u2) + u2'*u2);
  s = r(in);
  lambda2 = sign(s)*sqrt(1/abs(val));
  lambda1 = s*lambda2;
  q = lambda1*V(:,index-1) + lambda2*V(:,index);
  
  % Put everything together to form T
  
  T = DQuat2Hom([q(1:4), q(5:8)]);
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dq = Hom2DQuat(H)
%
% This function returns the dual quaternion corresponding to a
% homogeneous matrix H
%

  q = Mat2Quat(H(1:3,1:3));
  qprime = .5*Qmult(q,[0;H(1:3,4)]);
  dq = [q qprime];

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function H = DQuat2Hom(dq)
%
% This function returns the homogeneous matrix H corresponding to the
% dual quaternion dq
%

  q = dq(:,1);
  qe = dq(:,2);
  
  %q = q/norm(q);
  R = Quat2Mat(q);
  q(2:4) = -q(2:4);
  
  t = 2*Qmult(q,qe);
  
  H = [R t(2:4,1);0 0 0 1];

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Q3 = Qmult(Q2 ,Q1)

% This function returns multiplication of two quaternions Q1 & Q2

  Q3 = [Q1(1)*Q2(1) - Q1(2)*Q2(2) - Q1(3)*Q2(3) - Q1(4)*Q2(4);...
        Q1(2)*Q2(1) + Q1(1)*Q2(2) - Q1(4)*Q2(3) + Q1(3)*Q2(4);...
        Q1(3)*Q2(1) + Q1(4)*Q2(2) + Q1(1)*Q2(3) - Q1(2)*Q2(4);...
        Q1(4)*Q2(1) - Q1(3)*Q2(2) + Q1(2)*Q2(3) + Q1(1)*Q2(4)];

end
