 
function rpy = tr2rpy(m)
	
  eps=1e-6;
  
  rpy = zeros(1,3);

  % XYZ order
  if abs(m(3,3)) < eps && abs(m(2,3)) < eps
    % singularity
    rpy(1) = 0;  % roll is zero
    rpy(2) = atan2(m(1,3), m(3,3));  % pitch
    rpy(3) = atan2(m(2,1), m(2,2));  % yaw is sum of roll+yaw
  else
    rpy(1) = atan2(-m(2,3), m(3,3));        % roll
    % compute sin/cos of roll angle
    sr = sin(rpy(1));
    cr = cos(rpy(1));
    rpy(2) = atan2(m(1,3), cr * m(3,3) - sr * m(2,3));  % pitch
    rpy(3) = atan2(-m(1,2), m(1,1));        % yaw
  end
   
end