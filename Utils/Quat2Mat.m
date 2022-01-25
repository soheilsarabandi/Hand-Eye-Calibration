function R = Quat2Mat(e)
  % This function returns the rotation matrix corresponding to a set of Euler
  % parameters.

  R=[e(1)^2+e(2)^2-e(3)^2-e(4)^2         2*(e(2)*e(3)-e(1)*e(4))         2*(e(2)*e(4)+e(1)*e(3))
         2*(e(2)*e(3)+e(1)*e(4))     e(1)^2-e(2)^2+e(3)^2-e(4)^2         2*(e(3)*e(4)-e(1)*e(2))
         2*(e(2)*e(4)-e(1)*e(3))         2*(e(3)*e(4)+e(1)*e(2))     e(1)^2-e(2)^2-e(3)^2+e(4)^2];

  % If e is a unit quaternion, the following instruction can be removed
  R = (1/(e(1)^2 + e(2)^2 + e(3)^2+ e(4)^2)).*R;
end

