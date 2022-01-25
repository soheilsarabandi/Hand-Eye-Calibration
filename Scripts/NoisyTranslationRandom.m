function [MRerror,MOerror,MTerror]=NoisyTranslationRandom(varargin)
  % [MeanRerror,MeanOerror,MeanTerror]=NoisyTranslationRandom;
  %   Generates Fig. 3, left-column
  %
  % Optional parameters:
  %       testAll: true to test all the implemented methods.
  %     meanError: true to display mean errors. If false max errors are
  %                displayed.
  %     errorBars: true to add error bars with 95% confidence interval to
  %                the plots. If error bars are shown, the y-scale is
  %                linear (to avoid when error bars reach the negative).
  
  % for full reproducibility we can fix the random seed
  rng(1115856);
 
  if ~isempty(varargin)
    testAll=varargin{1};
  else
    testAll=false;
  end
  
  if length(varargin)>1
    meanError=varargin{2}; % if meanError=false -> maxError is computed
  else
    meanError=true;
  end
  
  if length(varargin)>2
    errorBars=varargin{3};
  else
    errorBars=false;
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
            @D7_OursCross...
            };
    color={'k:','k','r','g','r:','g:','b:','m','m:','b','c'};
  else
    method={@A2_Daniilidis,...
            @B1_Wu_Sun_Wang_Liu_Modified,...
            @C1_Liang_Mao,...
            @D6_Chou_Kamel,...
            @D7_Ours...
            };
    color={'k','r','g','b','c'};
  end
  
  n=15;        % number of measurements (default 15)
  m=1000;      % number of repetitions (default 1000)
  samples_error=50; % number of samples in the error (default 50)
  
  maxTransError=1; % maximum translation error
  
  fs=18; % Font size for plots
  lw=1.5; % Line width of the plots
  
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
  % Nothing need to be adjusted behond this point 
  
  nMethods=length(method);
  
  errorbound=linspace(0, maxTransError, samples_error);
  
  % Mean error
  MRerror=zeros(samples_error,nMethods);
  MOerror=zeros(samples_error,nMethods);
  MTerror=zeros(samples_error,nMethods);
  
  if errorBars
    % Standar deviation of the error
    SRerror=zeros(samples_error,nMethods);
    SOerror=zeros(samples_error,nMethods);
    STerror=zeros(samples_error,nMethods);
  end
  
  A=cell(1,n);
  B=cell(1,n);
  
  for s=1:samples_error    % level of error
    
    fprintf('Noise level %d of %d\n',s,samples_error);
    
    % Rotation error
    Rerror=zeros(m,nMethods);
    % Orthogonality error
    Oerror=zeros(m,nMethods);
    % Translation error
    Terror=zeros(m,nMethods);
    
    for j=1:m
      X=RandomTransf();
      for i=1:n
        B{i}=RandomTransf();
        A{i}=X*B{i}*X^-1*TransPerturbation(errorbound(s));
      end
      
      for k=1:nMethods
        e=false; % Error in the method
        try
          X1=method{k}(A,B);
        catch
          e=true;
        end
        if e || max(isnan(X1(:)))
          Rerror(j,k)=NaN;
          Oerror(j,k)=NaN;
          Terror(j,k)=NaN;
        else
          Rerror(j,k)=norm(X(1:3,1:3)-X1(1:3,1:3),'fro');
          Oerror(j,k)=abs(det(X1(1:3,1:3))-1);
          Terror(j,k)=norm(X(1:3,4)-X1(1:3,4),'fro');
        end
      end
    end
    
    %%%%%%%%%%%%%%%%%    Compute Rotation Error     %%%%%%%%%%%%%%%%%%%
    
    if meanError
      MRerror(s,:)=mean(Rerror,1);
      MOerror(s,:)=mean(Oerror,1);
      MTerror(s,:)=mean(Terror,1);
    else
      MRerror(s,:)=max(Rerror);
      MOerror(s,:)=max(Oerror);
      MTerror(s,:)=max(Terror);
    end
    if errorBars
      SRerror(s,:)=std(Rerror);
      SOerror(s,:)=std(Oerror);
      STerror(s,:)=std(Terror);
    end
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%    Results      %%%%%%%%%%%%%%%%%%%%%%%%%
  methodName=cell(1,nMethods);
  for k=1:nMethods
    s=char(method{k});
    methodName{k}=s(1:2);
  end
  
  figure(1); % Mean Distance Error (Rotation)
  if errorBars
    f=tinv(1-0.05,m-1); % Factor over std for the 90 confidence interval (5% at each tail)
    h=axes();
    for k=1:nMethods
      errorbar(errorbound',MRerror(:,k),f*SRerror(:,k),color{k},'LineWidth',lw);
      hold on;
    end
    h.YScale='log';
  else
    for k=1:nMethods
      semilogy(errorbound,MRerror(:,k),color{k},'LineWidth',lw);
      hold on;
    end
  end
  h=gca;
  h.FontSize=fs;
  if meanError
    ylabel('Mean $E_R$','Interpreter','latex','FontSize',fs);
  else
    ylabel('Max $E_R$','Interpreter','latex','FontSize',fs);
  end
  xlabel('Level of noise ($\sigma_{\rm trans}$)','Interpreter','latex','FontSize',fs);
  grid on;
  legend(methodName,'Location','northwest','Orientation','horizontal','Interpreter','latex','FontSize',fs);
  
  
  figure(2) ; % Mean Orthogonality Error
  if errorBars
    h=axes();
    for k=1:nMethods
      errorbar(errorbound',MOerror(:,k),f*SOerror(:,k),color{k},'LineWidth',lw);
      hold on;
    end
    h.YScale='log';
  else
    for k=1:nMethods
      semilogy(errorbound,MOerror(:,k),color{k},'LineWidth',lw);
      hold on;
    end
  end
  h=gca;
  h.FontSize=fs;
  if meanError
    ylabel('Mean $E_O$','Interpreter','latex','FontSize',fs);
  else
    ylabel('Max $E_O$','Interpreter','latex','FontSize',fs);
  end
  xlabel('Level of noise ($\sigma_{\rm trans}$)','Interpreter','latex','FontSize',fs);
  grid on;
  legend(methodName,'Location','northwest','Orientation','horizontal','Interpreter','latex','FontSize',fs);
  
  figure(3) ; % Mean Translation Error
  if errorBars
    h=axes();
    for k=1:nMethods
      errorbar(errorbound',MTerror(:,k),f*STerror(:,k),color{k},'LineWidth',lw);
      hold on;
    end
    h.YScale='log';
  else
    for k=1:nMethods
      semilogy(errorbound,MTerror(:,k),color{k},'LineWidth',lw);
      hold on;
    end
  end
  h=gca;
  h.FontSize=fs;
  if meanError
    ylabel('Mean $E_T$','Interpreter','latex','FontSize',fs);
  else
    ylabel('Max $E_T$','Interpreter','latex','FontSize',fs);
  end
  xlabel('Level of noise ($\sigma_{\rm trans}$)','Interpreter','latex','FontSize',fs);
  grid on;
  legend(methodName,'Location','northwest','Orientation','horizontal','Interpreter','latex','FontSize',fs);

end