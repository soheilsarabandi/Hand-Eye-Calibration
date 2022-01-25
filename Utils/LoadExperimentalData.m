function [A,B]=LoadExperimentalData(folder)

  % The end-effector robot poses are stored in a CSV file
  % (one pose per row)
  
  % Size of patter
  %   Data1 -> 24 mm
  %   Data2 -> 22 mm
  T=readtable(fullfile(folder,'EndEffector.txt'));
  
  m = size(T,1);
  K=cell(1,m);
  for i=1:m
    K{i}=reshape(table2array(T(i,:)),4,4);     
    %  convert translations to mm (Franka operates in meters but the
    % camera is calibrated in mm)
    K{i}(1:3,4)=K{i}(1:3,4)*1000;
  end
    
  A=cell(1,m-1);
  for i=1:m-1 
    A{i}=inv(K{i+1})*K{1};
  end
  
  % The parameters resulting from the camera callibration
  load(fullfile(folder,'CameraParams.mat'),'cameraParams');
  
  [~,~,n] =size(cameraParams.RotationMatrices);
  
  if (n~=m)
    error('Different number of camera and robot poses');
  end
  
  C=cell(1,n);
  for i=1:n
    R=cameraParams.RotationMatrices(:,:,i)';
    t=cameraParams.TranslationVectors(i,:)'; % camera is directly calibrated in mm
    C{i}=[R t;0 0 0 1];
  end

  B=cell(1,n-1);
  for i=1:n-1
    B{i}=C{i+1}*inv(C{1});
  end
       
end