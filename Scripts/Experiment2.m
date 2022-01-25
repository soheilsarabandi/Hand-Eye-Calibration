function Table=Experiment2(folder,varargin)
  % Table=Experiment2('RealDataSet');
  %    Hand-eye calibration using only two measuraments at a time.
  %    
  %    Input:
  %         folder: The folder including the data set. The dataset is
  %                 defined with two files
  %                 EndEffector.txt  -> the poses of the robot's end-effector
  %                 CameraParams.mat -> the result of the camera calibration using Matlab
  %  nMeasurements: [optional] Number of measurements to use. Optional. The default is 2.
  %        testAll: [optional] true if we have to test all the implemented methods.
  %                 If not given, false is assumed.
  %
  
  % Number of measuraments to be used in each experiment
  if isempty(varargin)
    nMeasurements=2; 
    testAll=false;
  else
    nMeasurements=varargin{1};
    if length(varargin)>1
      testAll=varargin{2};
    else
      testAll=false;
    end
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
            @D7_OursCross...
            };
  else
    % Select the methods to be evalutated
    method={@A2_Daniilidis,...
            @B1_Wu_Sun_Wang_Liu_Modified,...
            @C1_Liang_Mao,...
            @D6_Chou_Kamel,...
            @D7_OursCross...
            };
  end
     
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % Nothing need to be adjusted behond this point
  
  [A,B]=LoadExperimentalData(folder); 
  
  n=length(A);
  
  % Correct the number of elements in the subsets, if necessary
  if nMeasurements>n
    nMeasurements=n;
  else
    if nMeasurements<2
      nMeasurements=2;
    end
  end
  fprintf('Using combinations of %u measurements (out of %u)\n',nMeasurements,n);
  
  % For more than 2 measurements we can use our method without cross
  % products (although the cross products offer a better result while
  % the number of measurements is low, e.g., lower than 5).
  if nMeasurements>2
    method{end+1}=@D7_Ours;
  end
  
  nMethods=length(method);
  
  % All combinations of 'nMeasurements' elements
  subsetA=nchoosek(A,nMeasurements);
  subsetB=nchoosek(B,nMeasurements);
   
  nCombinations=size(subsetA,1); % == size(subsetB,1)
  
  Rerror=zeros(nMethods,nCombinations);
  Oerror=zeros(nMethods,nCombinations);
  Terror=zeros(nMethods,nCombinations);
  
  Re=zeros(1,nMeasurements);
  Te=zeros(1,nMeasurements);
  
  for c=1:nCombinations    
    % The sub-sets with the poses used in the current combination
    
    for k=1:nMethods
      X1=method{k}(subsetA(c,:),subsetB(c,:));
      
      for i=1:nMeasurements
        E=subsetA{c,i}*X1-X1*subsetB{c,i};
        Re(i)=norm(E(1:3,1:3),'fro');
        Te(i)=norm(E(1:3,4),'fro');
      end
      
      Rerror(k,c)=mean(Re);
      Oerror(k,c)=abs(det(X1(1:3,1:3))-1);
      Terror(k,c)=mean(Te);
    end    
  end
  
  E_rot=mean(Rerror,2);
  E_orth=mean(Oerror,2);
  E_trans=mean(Terror,2);
  
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
  fprintf('Average over %u combinations\n',nCombinations);
  disp(Table);
  
end