function M = RotPerturbation(bound)
%
% This function returns a random homogenous transformation matrix whose
% rotational part (Rot(X),Rot(Y),Rot(Z))are random angles uniformly 
% distributed in the interval [-bound, bound].
%

  ang = 2*bound.*rand(3,1) - bound;

  R = rotx(ang(1))*roty(ang(2))*rotz(ang(3));
  
  M = [R, zeros(3,1); 0 0 0 1];

end