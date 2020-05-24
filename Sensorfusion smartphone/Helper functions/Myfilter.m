function [xhat, meas] = filterTemplate(calAcc, calGyr, calMag)
% FILTERTEMPLATE  Filter template
%
% This is a template function for how to collect and filter data
% sent from a smartphone live.  Calibration data for the
% accelerometer, gyroscope and magnetometer assumed available as
% structs with fields m (mean) and R (variance).
%
% The function returns xhat as an array of structs comprising t
% (timestamp), x (state), and P (state covariance) for each
% timestamp, and meas an array of structs comprising t (timestamp),
% acc (accelerometer measurements), gyr (gyroscope measurements),
% mag (magnetometer measurements), and orint (orientation quaternions
% from the phone).  Measurements not availabe are marked with NaNs.
%
% As you implement your own orientation estimate, it will be
% visualized in a simple illustration.  If the orientation estimate
% is checked in the Sensor Fusion app, it will be displayed in a
% separate view.
%
% Note that it is not necessary to provide inputs (calAcc, calGyr, calMag).

  %% Setup necessary infrastructure
  import('com.liu.sensordata.*');  % Used to receive data.

  %% Filter settings
  t0 = [];  % Initial time (initialize on first data received)
  nx = 4;   % Assuming that you use q as state variable.
  % Add your filter settings here.
  Rw = [6.49387879914775e-07,-1.42534248411078e-08,4.97196329600802e-08;
        -1.42534248411078e-08,1.01729591677740e-06,2.63720827134863e-08;
        4.97196329600802e-08,2.63720827134863e-08,6.71290025517459e-07];
    
  Ra = [0.000136329334910749,-1.77530195827294e-07,3.74440800946149e-07;
        -1.77530195827294e-07,0.000144713181506337,-1.26164948448555e-05;
        3.74440800946149e-07,-1.26164948448555e-05,0.000202076528637777];
    
  g0 = [0.115501569664997;-0.296649960504062;9.86741791140054];
  
  accOut = 0;
  magOut = 0;
  
  m0 = [-14.5161348443092;-6.57631458403429;-93.6254289214554];
  
  Rm = [0.118502459469225,-0.0109745354499427,-0.0972133850892457;
        -0.0109745354499427,0.114499214536508,-0.0630803937921023;
        -0.0972133850892457,-0.0630803937921023,1.81876242147819];
    
  gyr_bool = false;
  acc_bool = false;
  mag_bool = false;
  
  % Current filter state.
  x = [1; 0; 0 ;0];
  P = eye(nx, nx);

  % Saved filter states.
  xhat = struct('t', zeros(1, 0),...
                'x', zeros(nx, 0),...
                'P', zeros(nx, nx, 0));

  meas = struct('t', zeros(1, 0),...
                'acc', zeros(3, 0),...
                'gyr', zeros(3, 0),...
                'mag', zeros(3, 0),...
                'orient', zeros(4, 0));
  try
    %% Create data link
    server = StreamSensorDataReader(3400);
    % Makes sure to resources are returned.
    sentinel = onCleanup(@() server.stop());

    server.start();  % Start data reception.

    % Used for visualization.
    figure(1);
    subplot(1, 2, 1);
    ownView = OrientationView('Own filter', gca);  % Used for visualization.
    googleView = [];
    counter = 0;  % Used to throttle the displayed frame rate.

    %% Filter loop
    while server.status()  % Repeat while data is available
      % Get the next measurement set, assume all measurements
      % within the next 5 ms are concurrent (suitable for sampling
      % in 100Hz).
      data = server.getNext(5);

      if isnan(data(1))  % No new data received
        continue;        % Skips the rest of the look
      end
      t = data(1)/1000;  % Extract current time

      if isempty(t0)  % Initialize t0
        t0 = t;
        xhat.t(end+1) = 0;
      end

      acc = data(1, 2:4)';
      if ~any(isnan(acc))  % Acc measurements are available.
        % Do something
        acc_bool = true;
      end
      gyr = data(1, 5:7)';
      if ~any(isnan(gyr))  % Gyro measurements are available.
        % Do something
        gyr_bool = true;
      end

      mag = data(1, 8:10)';
      if ~any(isnan(mag))  % Mag measurements are available.
        % Do something
        mag_bool = true;
      end
      
      if gyr_bool == true
          dt = t - t0 - xhat.t(end);
          
          [x, P] = tu_qw(x, P, gyr, dt, Rw);       % Time update of EKF
          [x, P] = mu_normalizeQ(x, P);
          gyr_bool = false;
      end
      
      if acc_bool == true 
          if abs(9.81 - norm(acc)) <= 1e-1
              [x, P] = mu_acc(x, P, acc, Ra, g0);  
              [x, P] = mu_normalizeQ(x, P);
              accOut = 0;
          else
              accOut = 1;
          end
          ownView.setAccDist(accOut)
          acc_bool = false;
      end
      
      if mag_bool == true
          L = 
          if abs(L - norm(mag)) < = 1e-1
            [x, P] = mu_m(x, P, mag, m0,Rm);
            [x, P] = mu_normalizeQ(x, P);
            magOut = 0;
          else
              magOut = 1;
          end
          ownView.setMagDist(magOut);
            mag_bool = false;
      end
      
      orientation = data(1, 18:21)';  % Google's orientation estimate.

      % Visualize result
      if rem(counter, 10) == 0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        if ~any(isnan(orientation))
          if isempty(googleView)
            subplot(1, 2, 2);
            % Used for visualization.
            googleView = OrientationView('Google filter', gca);
          end
          setOrientation(googleView, orientation);
          title(googleView, 'GOOGLE', 'FontSize', 16);
        end
      end
      counter = counter + 1;

      % Save estimates
      xhat.x(:, end+1) = x;
      xhat.P(:, :, end+1) = P;
      xhat.t(end+1) = t - t0;

      meas.t(end+1) = t - t0;
      meas.acc(:, end+1) = acc;
      meas.gyr(:, end+1) = gyr;
      meas.mag(:, end+1) = mag;
      meas.orient(:, end+1) = orientation;
    end
  catch e
      rethrow(e)
%     fprintf(['Unsuccessful connecting to client!\n' ...
%       'Make sure to start streaming from the phone *after*'...
%              'running this function!']);
  end
end
