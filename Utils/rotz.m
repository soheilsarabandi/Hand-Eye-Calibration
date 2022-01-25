function R = rotz(t)

  ct = cos(t);
  st = sin(t);
  
  R = [ct, -st, 0; st, ct,  0; 0, 0, 1];

end