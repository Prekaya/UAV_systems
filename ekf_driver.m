% ekf_driver.m
%
% Driver for Extended Kalman Filter simulation
%   Truth:
%     x:     True state vector, length n (nx1 column vector)
%     ymeas: Measurement vector, length m (mx1 column vector)
%   EKF Estimate:
%     xhat: Estimated state vector, nx1
%     P: Error covariance matrix, nxn
%     Q: Process noise covariance matrix, nxn
%     R: Measurement noise covariance matrix, mxm
%
% Code structure:
%    Initialize parameters, truth, EKF, logging, etc.
%    Time loop
%       Propagate Truth
%       Acquire measurement (if applicable)
%       Perform EKF
%       Log data
%    End time loop
%    Make plots
%
% Developed for JHU EP 525.661, UAV Systems & Control
%   

% Set simulation parameters
dt = .01;   % Sim time step, s
tMax = 20;  % Max time, s
g = 9.81;

% Define x state names (succinctly for plot labels)
xNames={'z, m', 'zdot, m/s'};
%xNames{end+1}='zdot, m/s';

% Set true initial state vector (nx1 if multidimensional)
x = [0; 85+1*randn];
len = length(x);
sigma_trueProcessNoise = [0; 0]; % nx1 (units of states)

% Set measurement parameters:
sigma_trueMeasNoise = [1.5;1.5]; % mx1 (units of measurements)
dtMeas=1; % measurement time interval, s
tNextMeas=dtMeas; % specifies next measurement time

% Initialize EKF (tuning parameters, initial estimate & uncertainty)
%  Note: EKF designer may or may not accurately know the true statistics.
sigma_ekfInitUncertainty = [0.5; 0.5]; % nx1 (units of states)
sigma_ekfProcessNoise =[0.001; 0.001]; % nx1 (units of states)
sigma_ekfMeasNoise = sigma_trueMeasNoise;
P = diag(sigma_ekfInitUncertainty.^2); % nxn (Initial P)
Q = diag(sigma_ekfProcessNoise.^2); % nxn
R = diag(sigma_ekfMeasNoise.^2); % mxm
xhat = [0; 85]; % nx1 (Initial xhat)

% For retaining history
tHistory = []; % Time vector
xHistory = []; % True state vectors (n columns)
yHistory = []; % True measurement vectors (m columns)
xhatHistory = []; % Estimated measurement vectors (n columns)
pHistory = []; % P diagonal elements (n columns)

% Time Loop
t=0;
while t<tMax

    %============================
    % True state propagation
    
    % Define state derivatives, xdot = f(x,...) + process noise
    fTrue = [x(2); -g];
    xdot = fTrue + sigma_trueProcessNoise/sqrt(dt).*randn(size(sigma_trueProcessNoise));

    % Propagate states and time
    x = x + xdot*dt;        % simple Euler integration
    t = t + dt;                 

    % Stopping criteria (e.g. if altitude<0 )
    if x(1)<=0
        break;
    end
    
    %============================
    % Periodically perform actual noisy measurement
    %   ymeas: actual measurement (mx1 if multidimensional)
    if t>=tNextMeas
        hTrue = x; % Perfect measurement (prior to adding noise)
        ymeas = hTrue + sigma_trueMeasNoise.*randn(size(sigma_trueMeasNoise));
        meas_available=1;
        tNextMeas = t+dtMeas-dt/2; % Determine next meas. time (dt/2 accounts for rounding errors)
    else
        % Set to nan vector if no measurement available
        ymeas=nan*ones(size(sigma_trueMeasNoise));    
        meas_available=0;
    end

    %==================
    % EKF processing
    
    % Propagation of previous state estimate to current time
    N=10; % Number of sub-steps for propagation each sample period
    for counter=1:N % Prediction step (N sub-steps)
        % Define state derivatives model f(), where xhatDot = f(xhat,...)
        f = [xhat(2); -g]; % nx1 (function of xhat, not x)
        
        % Define A, Linearization (Jacobian) of f(xhat,...) wrt xhat  (nxn matrix)
        A = [0 1; 0 0]; % nxn (function of xhat, not x)
        
        % State estimate propagate to sub-step N
        xhat = xhat + (dt/N)*f; % function of "dt", not "dtMeas"
        
        % Covariance matrix propagated to sub-step N
        P = P + (dt/N)*(A*P + P*A' + Q); % function of "dt", not "dtMeas"
        P = real(.5*P + .5*P'); % Make sure P stays real and symmetric
    end
    
    % State estimate correction at measurement
    if meas_available
        % Note: ymeas is the mx1 vector of actual measurements
        
        % Mathematical model of measurements based on xhat: yhat=h(xhat,...)
        h = xhat; % mx1 (function of xhat, not x) 
        
        % Define C: Linearization (Jacobian) of h(xhat,...) wrt xhat (mxn matrix)
        C = [1 0; 0 1]; % mxn (function of xhat, not x)
        
        % Kalman Gain matrix, nxm
        %  L: weightings to map measurement residuals into state estimates
        L = (P*C')/(C*P*C' + R);
        
        % States updated with measurement information
        xhat = xhat + L*(ymeas - h); % function of xhat, L, ymeas and h
        
        % Covariance matrix updated with measurement information
        I = eye(length(xhat)); % nxn identity matrix
        P = (I - L*C)*P; % function of I, L, C & P
        P = real(.5*P + .5*P'); % Make sure P stays real and symmetric        
    end

    %====================
    % Retain history
    tHistory(end+1,:) = t';    
    xHistory(end+1,:) = x';       
    yHistory(end+1,:) = ymeas';
    xhatHistory(end+1,:) = xhat';
    pHistory(end+1,:) = diag(P)';

end

% Plotting (ugly, but hopefully general enough to work for most cases without modification)
if 1
    for n=1:length(x)
        if length(xNames)<n, xNames{n}=sprintf('State %d',n); end
        subplot(2,length(x)+1,n); 
        try, for m=1:size(C,1), if C(m,n)==1, plot(tHistory,yHistory(:,m),'gd'); hold on; end; end; end
        hPlot=plot(tHistory,xHistory(:,n),'b',tHistory,xhatHistory(:,n),'r',tHistory,xhatHistory(:,n)+sqrt(pHistory(:,n)),'k',tHistory,xhatHistory(:,n)-sqrt(pHistory(:,n)),'k');
        set(hPlot(1),'linewidth',2);
        hold off
        grid on; xlabel('T, s'); ylabel(xNames{n});
        xlim([0 max(tHistory)])
        subplot(2,length(x)+1,length(x)+1+n); 
        plot(tHistory,xHistory(:,n)-xhatHistory(:,n),'r',tHistory,sqrt(pHistory(:,n)),'k',tHistory,-sqrt(pHistory(:,n)),'k')
        grid on; xlabel('T, s'); ylabel(['Error: ' xNames{n}]);
        xlim([0 max(tHistory)])
    end
    subplot(2,length(x)+1,length(x)+1); 
    plot(tHistory,yHistory(:,1),'gd'); for m=2:size(ymeas,1), hold on; plot(tHistory,yHistory(:,m),'d','color',[0 .25+.75*rand 0]); hold off; end
    grid on; ylabel('Measurements'); xlabel('T, s')
    xlim([0 max(tHistory)])
    subplot(2,length(x)+1,2*(length(x)+1))
    cla; axis([-1 1 -1 1]); 
    hL=text(0,0,{'==== Legend ====','Blue: Truth','Red: Estimate or Estimate Error','Black: 1-sigma error bounds','Green Diamonds: Measurements'}); 
    set(hL,'horizontalAlignment','center','verticalAlignment','middle','fontsize',8);
    axis off
end
