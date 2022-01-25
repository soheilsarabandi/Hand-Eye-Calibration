function R = rotx(t)

  ct = cos(t);
  st = sin(t);
  
  R = [1, 0, 0; 0, ct, -st; 0, st, ct];
  
end
    
