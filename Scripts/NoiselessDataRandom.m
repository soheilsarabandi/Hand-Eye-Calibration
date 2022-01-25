function Table=NoiselessDataRandom(ne,varargin)
  % Table=NoiselessDataRandom(ne);
  %
  % Where 'ne' is the type of experiment
  %   1: Table II
  %   2: Table III-left (last B=eye)
  %   3: Table III-right (last B=rot(pi))
  %   4: Table IV-left (R=eye)
  %   5: Table IV-right (R=rot(pi))
  %
  % use 'meanError=false, e.g., NoiselessData(ne,false), to obtain the 
  % MaxError instead of the mean error. The default is to compute the mean.
  %
  % Inputs:
  %   meanError: true to compute mean errors instead of maximum errors.
  %              The default is true.
  %
  % Outputs:
  %       Table: The table with numerical values.
 
  if isempty(varargin)
    meanError=true;
  else
    meanError=varargin{1};
  end
  
  % The methods to be evalutated
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
  
  % for full reproducibility we can fix the random seed 
  rng(1115856);
  
  % In some experiments we manually set Bn (it is not random)
  haveBn=false;
  switch ne
    case 2
      Bn=[eye(3), 10*rand(3,1)-[5; 5; 5]; 0 0 0 1];
      haveBn=true;
    case 3
      Bn=[rotx(pi), 10*rand(3,1)-[5; 5; 5]; 0 0 0 1];
      haveBn=true;
  end
  
  m=1000;                 % number of repetitions
  n=10;                   % number of measurements
  
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % Nothing need to be adjusted behond this points
  
  % Avoid warnings for clarity (they are informative, though!)
  warning('off','all');
  
  nMethods=length(method);
  
  A=cell(m,n);
  B=cell(m,n);
  X=cell(1,m);
            
  % Rotation error
  Rerror=zeros(m,nMethods);
  % Orthogonality error
  Oerror=zeros(m,nMethods);
  % Translation error
  Terror=zeros(m,nMethods);
  
  % We generate all the data befofe hand
  % In this way we can apply different methods to the same data
  % This is used to gather average execution times
  for j=1:m   % <- For all repetitions
    
    % The rotation (depending on the experiment)
    switch ne
      case {1,2,3}
        X{j}=RandomTransf();
      case 4
        % Table IV-left
        X{j}=[eye(3), 10*rand(3,1) - [5; 5; 5]; 0 0 0 1];
      case 5
        % Table IV-right
        X{j}=[roty(pi), 10*rand(3,1) - [5; 5; 5]; 0 0 0 1];
    end
    
    for i=1:n % <- Fix 'n' random B transforms
      if i==n && haveBn
        B{j,i}=Bn;
      else
        B{j,i}=RandomTransf();
      end
      A{j,i}=X{j}*B{j,i}*X{j}^-1;
    end 
  end
  
  % Now we solve for each method
  Average_Time_ms=zeros(nMethods,1);
  for k=1:nMethods
    
    e=false(1,m);
    X1=cell(1,m);
    
    % We execute the method 'k' over all the j-th data
    % We isolate the execution from the statistics computation
    % to get more accurate execution times
    tic;
    
    for j=1:m
      try
        X1{j}=method{k}(A(j,:),B(j,:));
      catch 
        e(j)=true;
      end
    end
    
    Average_Time_ms(k)=toc*1e3/m;
    
    % Now we compute statistics about the execution
    for j=1:m
      if e(j) || max(isnan(X1{j}(:)))
        Rerror(j,k)=NaN;
        Oerror(j,k)=NaN;
        Terror(j,k)=NaN;
      else
        Rerror(j,k)=norm(X{j}(1:3,1:3)-X1{j}(1:3,1:3),'fro');
        Oerror(j,k)=abs(det(X1{j}(1:3,1:3))-1);
        Terror(j,k)=norm(X{j}(1:3,4)-X1{j}(1:3,4),'fro');
      end
    end
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%    Results          %%%%%%%%%%%%%%%%%%%%%%%%%%%%
  methodName=cell(nMethods,1);
  for k=1:nMethods
    s=char(method{k});
    methodName{k}=s(1:2);
  end
  
  %%%%%%%%%%%%%%%%%%%%    Compute Mean/Max Error      %%%%%%%%%%%%%%%%%%%%%%%%%
  if meanError
    Mean_Rotation_Error=mean(Rerror,1)';
    Mean_Orthogonality_Error=mean(Oerror,1)';
    Mean_Translation_Error=mean(Terror,1)';
    
    Table=table(methodName, Mean_Rotation_Error,Mean_Orthogonality_Error, Mean_Translation_Error,Average_Time_ms);
  else
    Max_Rotation_Error=max(Rerror)';
    Max_Orthogonality_Error=max(Oerror)';
    Max_Translation_Error=max(Terror)';
    
    Table=table(methodName, Max_Rotation_Error,Max_Orthogonality_Error, Max_Translation_Error,Average_Time_ms);
  end
  
  Table.methodName=categorical(Table.methodName);
  disp(Table);
  
end 
