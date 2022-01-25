function axis = AxisRotNorm(R)
%
% This function returns the unit axis of rotation corresponding to a 
% rotation matrix. For null rotations, it returns (NaN, NaN, NaN).
%

  radical = 1 + R(1,1) + R(2,2) + R(3,3);

  if radical == 0
    a=zeros(3,1);
    a(1) = sqrt((1+R(1,1))/2);
    a(2) = sqrt((1+R(2,2))/2);
    a(3) = sqrt((1+R(3,3))/2);
    if R(1,2)<0 && R(2,3)<0
      a(2)= -a(2);
    end
    if R(2,3)<0 && R(3,1)<0
      a(3)= -a(3);
    end
    if R(3,1)<0 && R(1,2)<0
      a(1)= -a(1);
    end
  else
    a = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
  end
  
  axis = a/norm(a);

end

