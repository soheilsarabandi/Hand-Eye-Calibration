function M = TransPerturbation(bound)
%
% This function returns a homogenous transformation matrix whose 
% translational parts are random numbers uniformly distributed in the 
% interval [-bound, bound].
%

  M=[eye(3), 2*bound.*rand(3,1) - bound; 0, 0, 0, 1];

end