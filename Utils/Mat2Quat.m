function q = Mat2Quat(R)
%
% This function returns the set of Euler parameters corresponding to a
% rotation matrix.
%

 G =[ R(1,1)+R(2,2)+R(3,3),-R(2,3)+R(3,2),       -R(3,1)+R(1,3),       -R(1,2)+R(2,1);
     -R(2,3)+R(3,2),        R(1,1)-R(2,2)-R(3,3), R(1,2)+R(2,1),        R(1,3)+R(3,1);
     -R(3,1)+R(1,3),        R(1,2)+R(2,1),        R(2,2)-R(1,1)-R(3,3), R(2,3)+R(3,2);
     -R(1,2)+R(2,1),        R(1,3)+R(3,1),        R(2,3)+R(3,2),        R(3,3)-R(2,2)-R(1, 1)];
  [V, D] = eig(G);

  [~, index] = max(diag(D));
  q = V(:,index)/norm(V(:,index));

  if q(1)<0
    q=-q;
  end

end

