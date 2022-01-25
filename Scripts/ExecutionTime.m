function AverageTime=ExecutionTime(varargin)
  % [MeanRerror,MeanOerror,MeanTerror]=ExecutionTime;
  %   Execution time in function of the number of measuraments.
  %
  %   Generates Fig. 5 
  %
  % Inputs:
  %        testAll: [optional] true if we have to test all the implemented methods.
  %                 If not given, false is assumed.
  
  % for full reproducibility we can fix the random seed
  rng(1115856);
  
  if isempty(varargin)
    testAll=false;
  else
    testAll=varargin{1};
  end
  
  % Select the methods to be evalutated
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
            @D7_OursCross,...
            @D7_Ours...
            };
    color={'k:','k-','r-','g-','b:','m-','y-','r:','g:','b-','c--','c-'};
  else
    method={@A2_Daniilidis,...
            @B1_Wu_Sun_Wang_Liu_Modified,...
            @C1_Liang_Mao,...
            @D6_Chou_Kamel,...
            @D7_OursCross,...
            @D7_Ours...
            };
    color={'k-','r-','g-','b-','c--','c-'};
  end
          
  % Generate an arbitrary homogeneous transformation
  R=rotx(pi/3)*roty(pi/6)*rotz(pi/4);
  t=[10;5;4];
  
  % Variable number of measuraments depending on the problem
  n=[10 25 50 75 100 150 200 300 400 500];
  
  m=1000;        % number of repetitions
  sigmaR=0.025;  % noise level in rotation
  sigmaT=0.5;    % noise leven in translation
  
  fs=18; % Font size for plots
  lw=1.5; % Line width of the plots
  
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % Nothing need to be adjusted behond this point 
  
  % Avoid warnings for clarity (they are informative, though!)
  %warning('off','all');
  
  % The number of different number of measuraments to consider
  nn=length(n);
  
  X= [R t; 0 0 0 1];
     
  nMethods=length(method);
  
  % Generate all noisy matrices before hand
  B=cell(nn,m);
  A=cell(nn,m);
  
  % For the different number of measuraments, we generate a 'm' random 
  % problems each with 'n(i) random transforms
  for i=1:nn
    for j=1:m
      B{i,j}=cell(1,n(i));
      A{i,j}=cell(1,n(i));
      for k=1:n(i)
        B{i,j}{k}=RandomTransf();
        NoiseR=RotPerturbation(sigmaR);
        NoiseT=TransPerturbation(sigmaT);
        A{i,j}{k}=X*B{i,j}{k}*X^-1*NoiseT*NoiseR;
      end
    end
  end
  
  AverageTime=zeros(nMethods,nn);
  
  % for different number of measuraments
  for i=1:nn
    
    % Compute how long it takes (in average) for each method to solve
    % a problem with n(i) measuraments.
    % To get the average we solve 'm' problems
    fprintf('Num. measuraments: %u of %u\n',n(i),max(n));
    
    for k=1:nMethods
      tic;
      for j=1:m
        % We do not really care about the output. Just care about
        % execution time
        e=false; % Error in the method
        try
          X1=method{k}(A{i,j},B{i,j});
        catch
          e=true;
        end
      end
      AverageTime(k,i)=toc*1e3/m; % Average time in ms
    end

  end  
      
  methodName=cell(1,nMethods);
  for k=1:nMethods
    i=char(method{k});
    methodName{k}=i(1:2);
    if contains(i,'Cross')
      methodName{k}=[methodName{k} '$_\times$'];
    end
  end
  
  figure(1);
  for k=1:nMethods
    semilogy(n,AverageTime(k,:),color{k},'LineWidth',lw);
    if k==1
      xlabel('Num. Measuraments ($n$)','Interpreter','latex','FontSize',fs);
      ylabel('Average time (ms)','Interpreter','latex','FontSize',fs);
      hold on;
    end
  end
  h=gca;
  h.FontSize=fs;
  legend(methodName,'Location','northwest','Interpreter','latex','Orientation','horizontal','FontSize',fs);
  
end
        
        
