function Table=Experiment(folder,varargin)
  % Table1=Experiment('RealDataSet');
  %    Hand-eye calibration using all the available measuraments.
  %    
  %    Input:
  %         folder: The folder including the data set. The dataset is
  %                 defined with two files
  %                 EndEffector.txt  -> the poses of the robot's end-effector
  %                 CameraParams.mat -> the result of the camera calibration using Matlab
  %        testAll: [optional] true if we have to test all the implemented methods.
  %                 If not given, false is assumed.
     
  if isempty(varargin)
    testAll=false;
  else
    testAll=varargin{1};
  end
  
  % The user can select to test all the implemented methods or only
  % the more stable ones
  if testAll
    method={@A1_Lu_Chou,...
            @A2_Daniilidis,...
            @B1_Wu_Sun_Wang_Liu_Modified,...
            @C1_Liang_Mao,...
            @D1_Tsai_Lenz,...
            @D2_Shiu_Ahmad,...
            @D3_Park_Martin,...
            @D4_Wang,...
            @D5_Horaud_Dornaika,...
            @D6_Chou_Kamel,...
            @D7_Ours...
            };
  else
    % Select the methods to be evalutated
    method={@A2_Daniilidis,...
            @B1_Wu_Sun_Wang_Liu_Modified,...
            @C1_Liang_Mao,...
            @D6_Chou_Kamel,...
            @D7_OursCross,...
            @D7_Ours...
            };
  end

  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % Nothing need to be adjusted behond this point  
  
  nMethods=length(method);
  
  [A,B]=LoadExperimentalData(folder);
 
  n=length(A);
    
  E_rot=zeros(nMethods,1);
  E_orth=zeros(nMethods,1);
  E_trans=zeros(nMethods,1);
  
  eulang=zeros(nMethods,3);
  trans=zeros(nMethods,3);
  
  Rerror=zeros(1,n);
  Terror=zeros(1,n);
  
  for k=1:nMethods
    X1=method{k}(A,B);

    for i=1:n
      E=A{i}*X1-X1*B{i};
      Rerror(i)=norm(E(1:3,1:3),'fro');
      Terror(i)=norm(E(1:3,4),'fro');
    end
    
    E_rot(k)=mean(Rerror);
    E_orth(k)=abs(det(X1(1:3,1:3))-1);
    E_trans(k)=mean(Terror);
    
    %%% Computation of the calibration results in roll, pitch and yaw angles %%
    eulang(k,:)=rad2deg(tr2rpy(X1));
    trans(k,:)=X1(1:3,4)';
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%    Results          %%%%%%%%%%%%%%%%%%%%%%%%%%
  
  methodName=cell(nMethods,1);
  for k=1:nMethods
    s=char(method{k});
    methodName{k}=s(1:2);
    if contains(s,'Cross')
      methodName{k}=[methodName{k} 'x'];
    end
  end
  
  Table=table(methodName,E_rot,E_orth,E_trans);
  Table.methodName=categorical(Table.methodName);
  disp(Table);
  
end
