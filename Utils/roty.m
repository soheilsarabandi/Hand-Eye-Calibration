function R = roty(t)

  ct = cos(t);
  st = sin(t);
  
  R = [ct,  0,   st; 0,   1,   0; -st,  0,   ct];

end

