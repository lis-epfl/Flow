function calibration = ofvd(varargin)

% OFVD(gyro, of) calibrates the viewing direction of the optic-flow sensors
% using gyroscope and optic-flow output while only rotations are applied
% 
% Time series of a calibration run have to be provided. 
% 
% Note: delay between optic-flow and gyroscope values should be accounted for
%
% Please refer to paper by A. Briod, J.-c. Zufferey, and D. Floreano, 
%   “Automatically calibrating the viewing direction of optic-flow sensors” 
%   in International Conference on Robotics and Automation, 2012.
% Plase, cite the paper if you use the method
%
% mandatory arguments:
%   gyro(Nx3): 3-axis gyroscope data in rad/s
%   of(Nx2)  : 2d optic-flow vector measured by sensor
%
% optional arguments:
%   weight(Nx1): weight to give to each measurement
%   show_plots : boolean to indicate if the plots of the calibration
%                procedure should be shown or not
%   handle_plot3D : id of the figure on which to plot the 3D axes of the sensor
%   time(Nx1)  : time vector used for plots, if provided
%
%       Adrien Briod (adrien.briod@gmail.com) - (c) EPFL - 2012
% 
%  Uses the following function found on 'File Exchange':
%   - process_arguments.m by Alan Robinson
%     http://www.mathworks.com/matlabcentral/fileexchange/25881-process-named-arguments
%   - area2.m by John A. Bockstege
%     http://www.mathworks.com/matlabcentral/fileexchange/13188-shade-area-between-two-curves

    show_plots = 1;

    process_arguments(...
        {
            'gyro'
            'of'
        },{
            'weight'
            'show_plots'
            'handle_plot3D'
            'time'
        },{
            'weight'
            'show_plots'
            'handle_plot3D'
            'time'
        }, varargin);     % very important, keep varargin{1} = this when using this function in class methods

    L = size(of,1); % length of time series
    
    % if weight isn't provided, all measurements get the same weight
    if ~exist('weight')
        weight = ones(L,1);
    end
    
    % if time isn't provided, uses following vector
    if ~exist('time')
        time = 0:(L-1);
    end
    
    
    % CALIBRATION PX -> obtain r2 %%%%%%%%%%%%%%%%%%%%%%%%%%
    ofx = of(:,1);
    [xr2,xstd2] = iwls(gyro,-ofx,weight);
    
    r2   = xr2(:,end);      % second row of rotation matrix is the final LS estimate
    
    calibration.ofxDRO  = ofx - gyro*(-r2);     % derotation
    calibration.r2      = r2;
    calibration.norm2   = norm(r2);
    calibration.std2    = xstd2(:,end);    % final standard deviation of rotation matrix elements
    
    % CALIBRATION PY -> obtain r1  %%%%%%%%%%%%%%%%%%%%%%%%%%
    ofy = of(:,2);
    [xr1,xstd1] = iwls(gyro,ofy,weight);
    
    r1 = xr1(:,end);      % first row of rotation matrix is the final LS estimate
    
    calibration.ofyDRO  = ofy - gyro*r1;        % derotation
    calibration.r1      = r1;
    calibration.norm1   = norm(r1);
    calibration.std1    = xstd1(:,end);         % final standard deviation of rotation matrix elements
    
    % RECONSTRUCT APPROXIMATE R
    a1 = r1/norm(r1);
    a2 = r2/norm(r2);
    a3 = cross(a1, a2);
    A = [a1'; a2'; a3'];
    
    % RECONSTRUCT R with polar decomposition 
    calibration.R = A*(A'*A)^(-1/2);
    
    
    if show_plots
        %% plot estimation procedure of r coefficients %%%%%%%
        figure

        subplot(511)
        hold on
        plot(time,ofx,'Color',[0.75 0   0.75])
        plot(time,ofy,'Color',[1    0.7 0.4 ])
        title('Optic-flow measurements of one sensor');
        legend('p_x','p_y');
        axis([0 time(end) -5 5])
        ylabel('rad/s');

        subplot(512)
        hold on
        plot(time,gyro)
        title('Angular speed measured by gyroscopes');
        legend('w_1','w_2','w_3');
        ylabel('rad/s');
        axis([0 time(end) -5 5])

        subplot(513)
        hold on
        plot(time,weight)
        area2(time,ones(1,length(time))*-200,ones(1,length(time))*-200 + (weight'<=0)*400, [.5 .5 .5], 'none', .3)
        title('Weight of optic-flow sensor');
        legend('weight','discarded');
        ylabel('weight (normalized)');
        axis([0 time(end) 0 max(weight)])

        subplot(514)
        hold on
        plot(time,xr1)
        area2(time,xr1(1,:) + xstd1(1,:),xr1(1,:) - xstd1(1,:), [0 0 1], 'none', .3)
        area2(time,xr1(2,:) + xstd1(2,:),xr1(2,:) - xstd1(2,:), [.15 .8 .2], 'none', .3)
        area2(time,xr1(3,:) + xstd1(3,:),xr1(3,:) - xstd1(3,:), [1 0 0], 'none', .3)
        title('Estimation (± standard deviation) of 1st row of R_s');
        legend('r_{11}','r_{12}','r_{13}');
        axis([0 time(end) -1.5 1.5])

        subplot(515)
        hold on
        plot(time,xr2)
        area2(time,xr2(1,:) + xstd2(1,:),xr2(1,:) - xstd2(1,:), [0 0 1], 'none', .3)
        area2(time,xr2(2,:) + xstd2(2,:),xr2(2,:) - xstd2(2,:), [.15 .8 .2], 'none', .3)
        area2(time,xr2(3,:) + xstd2(3,:),xr2(3,:) - xstd2(3,:), [1 0 0], 'none', .3)
        title('Estimation (± standard deviation) of 2nd row of R_s');
        legend('r_{21}','r_{22}','r_{23}');
        axis([0 time(end) -1.5 1.5])
        xlabel('t (s)');


        %% plot de-rotated optic-flow with r and q estimation    %%%%%%%%%%%%%%%%%%%
        figure
        subplot(211)
        plot(time,ofx)
        hold on
        plot(time,calibration.ofxDRO,'r','LineWidth',2)
        legend('p_x','de-rotated p_x');
        ylabel('ang. speed (rad/s)');
        title('Comparison between normal optic-flow and de-rotated optic-flow');

        subplot(212)
        plot(time,ofy)
        hold on
        plot(time,calibration.ofyDRO,'r','LineWidth',2)
        legend('p_y','de-rotated p_y');
        xlabel('t (s)');
        ylabel('ang. speed (rad/s)');

    end
    
    %% 3D plot (in NED coordinates)
    if exist('handle_plot3D') || show_plots
        if ~exist('handle_plot3D')
            handle_plot3D = figure;
        else
            figure(handle_plot3D);
        end
        hold on
        set(gca,'DataAspectRatio',[1 1 1]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        view(3)
        
        R_s = inv(calibration.R);

        e_s_x = R_s(:,1)';
        e_s_y = R_s(:,2)';
        e_s_z = R_s(:,3)';
        
        % plot gyroscope frame
        plot3([0 0],[0 1],[0 0],'r','LineWidth',3);
        plot3([0 1],[0 0],[0 0],'b','LineWidth',3);
        plot3([0 0],[0 0],[0 -1],'Color',[.2 .7 .2],'LineWidth',3);

        % plot dashed line going from origin to sensor
        origin = e_s_z*1.5;
        plot3([0; origin(2)],[0; origin(1)],[0; -origin(3)],'k:');

        % plot sensor axes
        v = [origin; origin + e_s_x];
        plot3(v(:,2),v(:,1),-v(:,3),'r','LineWidth',1);
        v = [origin; origin + e_s_y];
        plot3(v(:,2),v(:,1),-v(:,3),'b','LineWidth',1);
        v = [origin; origin + e_s_z];
        plot3(v(:,2),v(:,1),-v(:,3),'Color',[.2 .8 .2],'LineWidth',1);

    end
end