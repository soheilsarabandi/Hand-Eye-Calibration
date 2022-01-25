function axis = AxisRot(R)
%
% This function returns the axis of rotation corresponding to a rotation
% matrix without normalization. The result is not valid for rotations equal 
% to pi radians about arbitrary axes. In these cases, it returns (0, 0, 0).
% For null rotations, it also returns (0, 0, 0)
%

  axis = [(R(3,2) - R(2,3)); (R(1,3) - R(3,1));  (R(2,1) - R(1,2))];

end