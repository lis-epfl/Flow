function calibration = grm(varargin)

% OFVD(rg, cg) calibrates the viewing direction of the optic-flow sensor
% using robot gyroscope and camera gyroscope output while only rotations are applied
% 
% Time series of a calibration run have to be provided. 
% 
% Note: delay between the two gyroscopes values should be accounted for
%
%
% mandatory arguments:
%   rg(Nx3): 3-axis robot gyroscope data in rad/s
%   cg(Nx3): 3-axis camera gyroscope date in rad/s
%
% optional arguments:
%   weight(Nx1): weight to give to each measurement
%   show_plots : boolean to indicate if the plots of the calibration
%                procedure should be shown or not
%   handle_plot3D : id of the figure on which to plot the 3D axes of the sensor
%   time(Nx1)  : time vector used for plots, if provided
    
    %% Main function
    show_plots = 1;

    process_arguments(...
        {
            'rg'
            'cg'
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

    L = size(cg,1); % length of time series
    
    % if weight isn't provided, all measurements get the same weight
    if ~exist('weight')
        weight = ones(L,1);
    end
    
    % if time isn't provided, uses following vector
    if ~exist('time')
        time = 0:(L-1);
    end
     
    % CALIBRATION WX -> obtain r1 %%%%%%%%%%%%%%%%%%%%%%%%%%
    cgx = cg(:,1);
    [xr1,xstd1] = iwls(rg,-cgx,weight);
    
    r1   = xr1(:,end);      % second row of rotation matrix is the final LS estimate
    
    calibration.r1      = r1;
    calibration.norm1   = norm(r1);
    calibration.std1    = xstd1(:,end);    % final standard deviation of rotation matrix elements
    
    % CALIBRATION WY -> obtain r2  %%%%%%%%%%%%%%%%%%%%%%%%%%
    cgy = cg(:,2);
    [xr2,xstd2] = iwls(rg,cgy,weight);
    
    r2 = xr2(:,end);      % first row of rotation matrix is the final LS estimate
    
    calibration.r2      = r2;
    calibration.norm2   = norm(r2);
    calibration.std2    = xstd2(:,end);         % final standard deviation of rotation matrix elements
    
    % CALIBRATION WZ -> obtain r3  %%%%%%%%%%%%%%%%%%%%%%%%%%
    cgz = cg(:,3);
    [xr3,xstd3] = iwls(rg,cgz,weight);
    
    r3 = xr3(:,end);      % first row of rotation matrix is the final LS estimate
    
    calibration.r3      = r3;
    calibration.norm3   = norm(r3);
    calibration.std3    = xstd3(:,end);         % final standard deviation of rotation matrix elements
    
    % RECONSTRUCT APPROXIMATE R
    a1 = r1/norm(r1);
    a2 = r2/norm(r2);
    %a3 = cross(a1, a2); % First method: Ignore the third axis
    a3 = r3/norm(r3);   % Second method: Use third axis
    A = [a1'; a2'; a3'];
    
    % RECONSTRUCT R with polar decomposition 
    calibration.R = A*(A'*A)^(-1/2);
    
    %% Plot estimation procedure of r coefficients
    if show_plots
        figure

        subplot(611)
        hold on
        plot(time,cg)
        title('Angular speed measure by camera gyroscopes');
        legend('w_{cx}', 'w_{cy}', 'w_{cz}');
        ylabel('rad/s');
        axis([0 time(end) -5 5])

        subplot(612)
        hold on
        plot(time,rg)
        title('Angular speed measured by robot gyroscopes');
        legend('w_{rx}','w_{ry}','w_{rz}');
        ylabel('rad/s');
        axis([0 time(end) -5 5])

        subplot(613)
        hold on
        plot(time,weight)
        area2(time,ones(1,length(time))*-200,ones(1,length(time))*-200 + (weight'<=0)*400, [.5 .5 .5], 'none', .3)
        title('Weight of optic-flow sensor');
        legend('weight','discarded');
        ylabel('weight (normalized)');
        axis([0 time(end) 0 max(weight)])

        subplot(614)
        hold on
        plot(time,xr1)
        area2(time,xr1(1,:) + xstd1(1,:),xr1(1,:) - xstd1(1,:), [0 0 1], 'none', .3)
        area2(time,xr1(2,:) + xstd1(2,:),xr1(2,:) - xstd1(2,:), [.15 .8 .2], 'none', .3)
        area2(time,xr1(3,:) + xstd1(3,:),xr1(3,:) - xstd1(3,:), [1 0 0], 'none', .3)
        title('Estimation (± standard deviation) of 1st row of R_s');
        legend('r_{11}','r_{12}','r_{13}');
        axis([0 time(end) -1.5 1.5])

        subplot(615)
        hold on
        plot(time,xr2)
        area2(time,xr2(1,:) + xstd2(1,:),xr2(1,:) - xstd2(1,:), [0 0 1], 'none', .3)
        area2(time,xr2(2,:) + xstd2(2,:),xr2(2,:) - xstd2(2,:), [.15 .8 .2], 'none', .3)
        area2(time,xr2(3,:) + xstd2(3,:),xr2(3,:) - xstd2(3,:), [1 0 0], 'none', .3)
        title('Estimation (± standard deviation) of 2nd row of R_s');
        legend('r_{21}','r_{22}','r_{23}');
        axis([0 time(end) -1.5 1.5])
        xlabel('t (s)');
        
        subplot(616)
        hold on
        plot(time,xr3)
        area2(time,xr3(1,:) + xstd3(1,:),xr3(1,:) - xstd3(1,:), [0 0 1], 'none', .3)
        area2(time,xr3(2,:) + xstd3(2,:),xr3(2,:) - xstd3(2,:), [.15 .8 .2], 'none', .3)
        area2(time,xr3(3,:) + xstd3(3,:),xr3(3,:) - xstd3(3,:), [1 0 0], 'none', .3)
        title('Estimation (± standard deviation) of 3rd row of R_s');
        legend('r_{31}','r_{32}','r_{33}');
        axis([0 time(end) -1.5 1.5])
        xlabel('t (s)');
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