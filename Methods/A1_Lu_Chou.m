function T = A1_Lu_Chou(A, B)
%
% This function solves the problem AX=XB based on the eight-space quaternion 
% representation described in: 
%
% Y.-C. Lu and J. C. Chou, "Eight-space quaternion approach for robotic
% hand-eye calibration," in 1995 IEEE International Conference on Systems,
% Man and Cybernetics. Intelligent Systems for the 21st Century,
% vol. 4, 1995, pp. 3316-3321.
%
  n = length(A);
  
  % Compute rotation R
  
  N = zeros(8,8);
  
  for i = 1:n
    Ra = A{i}(1:3,1:3);
    Rb = B{i}(1:3,1:3);
    qa = Mat2Quat(Ra);
    qb = Mat2Quat(Rb);
    ta = [0; A{i}(1:3,4)];
    tb = [0; B{i}(1:3,4)];
    Eb = [qb(1), -qb(2:4)'; qb(2:4), qb(1)*eye(3)-skew(qb(2:4))];
    Ea = [qa(1), -qa(2:4)'; qa(2:4), qa(1)*eye(3)+skew(qa(2:4))];
    Tn = [tb(1), -tb(2:4)'; tb(2:4), tb(1)*eye(3)-skew(tb(2:4))];
    Tp = [ta(1), -ta(2:4)'; ta(2:4), ta(1)*eye(3)+skew(ta(2:4))];
    C = [Eb*(Tp-Tn),  Ea-Eb; Ea-Eb, zeros(4,4)];
    N = (C'*C)+N;
  end
  
  [~,S,V] = svd(N);
  eig = diag(S);
  [~,index] = min(eig(eig~=0));
  V = V(:,index);
  V = V/norm(V(1:4));
  R = Quat2Mat(V(1:4));
  
  % Compute translation t and put everything together to form T
  
  t = [-V(2:4), V(1)*eye(3) + skew(V(2:4))]*V(5:8);
  
  T = [R t; 0 0 0 1];
  
end