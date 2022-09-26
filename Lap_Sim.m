% Kieran Cosgrove stolen from Jonathan Vogel
% UT Austin FSAE Electric

clear
close all
clc

% The purpose of this code is to evaluate the points-scoring capacity of a
% virtual vehicle around the 2019 FSAE Michigan Dynamic Event Tracks

%% Section 0: Name all symbolic variables
% Don't touch this. This is just naming a bunch of variables and making
% them global so that all the other functions can access them
global r_max accel grip deccel lateral cornering gear shift_points...
    top_speed r_min path_boundaries tire_radius shift_time...
    powertrainpackage track_width path_boundaries_ax

tw = 48/12;
r_max = 150;
top_speed = 70;
VMAX = top_speed;

% make the rest of your functions for the GGV diagram
% braking as a function of speed
deccel = 1;
velocity = 15:5:130;
% lateral g's as a function of velocity
lateral = 1;
velocity_y = [10, 15, 20];
lateralg = [1, 1, 1];
radii = velocity_y.^2./lateralg/32.2;
% max velocity as a function of instantaneous turn radius
cornering = csaps(radii,velocity_y);
%% Section 7: Load Endurance Track Coordinates
disp('Loading Endurance Track Coordinates')
[data text] = xlsread('Endurance_Coordinates_1.xlsx','Scaled');

% the coordinates are now contained within 'data'. This is a 5 column
% matrix that contains a set of defined 'gates' that the car must mavigate
% through
% Column 1: Gate #
% Column 2: Outside boundary, x coordinate
% Column 3: Outside boundary, y coordinate
% Column 4: Inside boundary, x coordinate
% Column 5: Inside boundary, y coordinate

% sort the data into "inside" and "outside" cones
outside = data(:,2:3);
inside = data(:,4:5);
t = [1:length(outside)];
% define the minimum turn radius of the car
r_min = 4.5*3.28;
r_min = r_min-tw/2;
pp_out = spline(t,outside');
pp_in = spline(t,inside');

for i = 1:1:length(outside)
    % isolate individual gates
    gate_in = inside(i,:);
    gate_out = outside(i,:);
    % create the line that connects the two cones together
    x1 = gate_in(1);
    x2 = gate_out(1);
    y1 = gate_in(2);
    y2 = gate_out(2);
    % polynomial expression for the line:
    coeff = polyfit([x1, x2], [y1, y2], 1);
    % adjust the width of the gate for the width of the car:
    gate_width = sqrt((x2-x1)^2+(y2-y1)^2);
    path_width = gate_width-tw;
    x_fs = tw/(2*gate_width);
    % update the gate boundaries based on said new width
    x_bound = [min(x1,x2)+x_fs*abs(x2-x1),max(x1,x2)-x_fs*abs(x2-x1)];
    path_boundaries(i,:) = [coeff x_bound];
end
%% Seciton 8: Load Endurance Racing Line
disp('Loading Endurance Racing Line')
xx = load('endurance_racing_line.mat');
xx = xx.endurance_racing_line;
%% Section 9: Optimize Endurance Racing Line
% The pre-loaded racing line should work for most applications; however,
% if you have the need to re-evaluate or generate a new optimized racing
% line, simply un-comment the code below:


% disp('Optimizing Endurance Racing Line')
% A = eye(length(xx));
% b = ones(length(xx),1);
% lb = zeros(1,length(xx));
% ub = ones(1,length(xx));
% options = optimoptions('fmincon',...
%     'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-12);
% options = optimoptions(options,'MaxIter', 10000, 'MaxFunEvals', 1000000,'ConstraintTolerance',1e-12,'DiffMaxChange',.1);
% 
% x = fmincon(@lap_time,xx,[],[],[],[],lb,ub,@track_curvature,options);
% xx = x;
% x(end+1) = x(1);
%x(end+1) = x(2);
%% Section 10: Generate Final Endurance Trajectory
x = xx;
% Plot finished line
x(end+1) = x(1);
x(end+1) = x(2);
for i = 1:1:length(x)
    % for each gate, find the position defined between the cones
    coeff = path_boundaries(i,1:2);
    x2 = max(path_boundaries(i,3:4));
    x1 = min(path_boundaries(i,3:4));
    position = x(i);
    % place the car within via linear interpolation
    x3 = x1+position*(x2-x1);
    y3 = polyval(coeff,x3);
    %plot(x3,y3,'og')
    % the actual car's trajectory defined in x-y coordinates:
    path_points(i,:) = [x3 y3];
end

x = linspace(1,t(end-1),1000);
ppv = pchip(t,path_points');
vehicle_path = ppval(ppv,x);
vehicle_path_EN = vehicle_path;
Length = arclength(vehicle_path(1,:),vehicle_path(2,:));
%% Section 11: Simulate Endurance Lap
disp('Plotting Vehicle Trajectory')
[laptime time_elapsed velocity acceleration lateral_accel gear_counter path_length weights distance] = lap_information(xx);
%% Section 12: Load Autocross Track Coordinates
disp('Loading Autocross Track Coordinates')
[data text] = xlsread('Autocross_Coordinates_2.xlsx','Scaled');
outside = data(:,2:3);
inside = data(:,4:5);
t = [1:length(outside)];
r_min = 4.5*3.28;
r_min = r_min-tw/2;
pp_out = spline(t,outside');
pp_in = spline(t,inside');

%plot(outside(:,1),outside(:,2),'ok')
%hold on
%plot(inside(:,1),inside(:,2),'ok')
clear path_boundaries
for i = 1:1:length(outside)
    gate_in = inside(i,:);
    gate_out = outside(i,:);
    %plot([gate_in(1) gate_out(1)],[gate_in(2) gate_out(2)],'-k')
    x1 = gate_in(1);
    x2 = gate_out(1);
    y1 = gate_in(2);
    y2 = gate_out(2);
    coeff = polyfit([x1, x2], [y1, y2], 1);
    gate_width = sqrt((x2-x1)^2+(y2-y1)^2);
    path_width = gate_width-tw;
    x_fs = tw/(2*gate_width);
    x_bound = [min(x1,x2)+x_fs*abs(x2-x1),max(x1,x2)-x_fs*abs(x2-x1)];
    path_boundaries_ax(i,:) = [coeff x_bound];
    %text(round(x1),round(y1),num2str(i))
end


%save('path_boundaries.mat','path_boundaries');
%% Section 13: Load Autocross Racing Line
disp('Loading Autocross Racing Line')
xx = load('autocross_racing_line.mat');
xx = xx.autocross_racing_line;
%% Section 14: Optimize Autocross Racing Line
% Same applies here, optimizing the line is optional but if you want,
% simply un-comment the lines of code below:


% disp('Optimizing Racing Line')
% A = eye(length(xx));
% b = ones(length(xx),1);
% lb = zeros(1,length(xx));
% ub = ones(1,length(xx));
% options = optimoptions('fmincon',...
%     'Algorithm','sqp','Display','iter','ConstraintTolerance',1e-12);
% options = optimoptions(options,'MaxIter', 10000, 'MaxFunEvals', 1000000,'ConstraintTolerance',1e-12,'DiffMaxChange',.1);
% 
% x = fmincon(@lap_time_sprint,xx,[],[],[],[],lb,ub,@track_curvature_sprint,options);
% xx_auto = x;
% % x(end+1) = x(1);
% % x(end+1) = x(2);
%% Section 15: Generate Final Autocross Trajectory
xx_auto = xx;
x = xx_auto;
%Plot finished line

for i = 1:1:length(x)
    coeff = path_boundaries_ax(i,1:2);
    x2 = max(path_boundaries_ax(i,3:4));
    x1 = min(path_boundaries_ax(i,3:4));
    position = x(i);
    x3 = x1+position*(x2-x1);
    y3 = polyval(coeff,x3);
    %plot(x3,y3,'og')
    path_points_ax(i,:) = [x3 y3];
end
x = linspace(1,t(end),1000);
ppv = pchip(t,path_points_ax');
vehicle_path = ppval(ppv,x);
vehicle_path_AX = vehicle_path;
Length = arclength(vehicle_path(1,:),vehicle_path(2,:));
%% Section 16: Simulate Autocross Lap
disp('Plotting Vehicle Trajectory')
[laptime_ax time_elapsed_ax velocity_ax, acceleration_ax lateral_accel_ax gear_counter_ax path_length_ax weights_ax distance_ax] = lap_information_sprint(xx_auto);
%% Section 17: Calculate Dynamic Event Points
disp('Calculating Points at Competition')
% calculate endurance score
Tmin = 115.249;
Tmax = Tmin*1.45;
Endurance_Score =250*((Tmax/(laptime+13))-1)/(Tmax/Tmin-1)+25;

% Calculate autocross score
Tmin = 48.799;
Tmax = Tmin*1.45;
Autocross_Score =118.5*((Tmax/(laptime_ax))-1)/(Tmax/Tmin-1)+6.5;

% Skidpad Analysis
% define skidpad turn radius (ft)
path_radius = 25+tw/2+.5;
% determine speed possible to take:
speed = fnval(cornering,path_radius);
% calculate skidpad time
skidpad_time = path_radius*2*pi/speed;
% calculate score based on 2019 times
Tmin_skid = 4.865;
Tmax_skid = Tmin_skid*1.45;
Skidpad_Score = 71.5*((Tmax_skid/skidpad_time)^2-1)/((Tmax_skid/Tmin_skid)^2-1) + 3.5;

% Acceleration Analysis
% start at speed 0, gear 1, etc
count = 0;
v = 0;
vel = v;
gears = find((shift_points-vel)>0);
gear = gears(1)-1;
newgear = gear;
time_shifting = 0;
interval = 1;
% accel track is 247 feet, so I am defining 247 segments of 1 foot:
segment = 1:1:247;
t_accel_elapsed = 0;
clear dt_f v_f
% little quickie accel sim:
for i = 1:1:length(segment)
    d = 1;

    %gear = newgear;
    % find what gear you are in
    gears = find((shift_points-vel)>0);
    newgear = gears(1)-1;
    
    % compare to previous iteration, to detect an upshift
    if newgear > gear
        shifting = 1;
    else
        shifting = 0;
    end

    vmax = VMAX;
    % determine instantaneous acceleration capacity
    AX = fnval(accel,vel);
    dd = d/interval;
    for j = 1:1:interval
        count = count+1;
        if vel < vmax
            % if you are not shifting, and top speed has not been achieved
            % then you keep accelerating at maximum capacity possible
            ax_f(count) = AX;
            tt = roots([0.5*32.2*ax_f(count) vel -dd]);
            dt_f(count) = max(tt);
            dv = 32.2*ax_f(count)*dt_f(count);
            dvmax = vmax-vel;
            dv_f(count) = min(dv,dvmax);
            v_f(count) = vel+dv_f(count); 
            vel = v_f(count);
        else
            % if you are not shifting but you are at top speed, then just
            % hold top speed
            vel = vmax;
            dt_f(count) = dd/vel;
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
        end
        if time_shifting > shift_time
            shifting = 0;
            time_shifting = 0;
            gear = newgear;
        end
    end
    t_accel_elapsed = t_accel_elapsed+dt_f(count);
    t_accel(i) = t_accel_elapsed;
end
accel_time = sum(dt_f(2:end))+.1;
% calculate accel score:
Tmin_accel = 4.109;
Tmax_accel = Tmin_accel*1.5;
Accel_Score = 95.5*((Tmax_accel/accel_time)-1)/((Tmax_accel/Tmin_accel)-1) + 4.5;

Total_Points = Accel_Score+Skidpad_Score+Autocross_Score+Endurance_Score;

results = [time_elapsed' velocity'];
%xlswrite('logged_data.xlsx',results,'sim_data') 
%% Section 18: Generate Load Cases
disp('Generating Load Cases')
% find all three worst case acceleration cases:
AX_min = min(acceleration);
AX_max = max(acceleration);
AY_max = max(lateral_accel);
% then find where they took place
VX_min = velocity(find(acceleration == AX_min));
VX_max = velocity(find(acceleration == AX_max));
VY_max = velocity(find(lateral_accel == AY_max));
VY_max = max(VY_max);
frontF = zeros(3,3);
rearF = zeros(3,3);
% then calculate loads based on those speeds and accelerations: 
% see documentation spreadsheet for translation
frontF(3,:) = [WF/2 + Cl*VX_max^2*CoP/2 - WF*AX_max*cg/l/2 , WF/2 + Cl*VX_min^2*CoP/2 - WF*AX_min*cg/l/2, WF/2 + Cl*VY_max^2*CoP/2 + WF*AY_max*cg/tw/2];
rearF(3,:) = [WR/2 + Cl*VX_max^2*(1-CoP)/2 + WR*AX_max*cg/l/2 , WR/2 + Cl*VX_min^2*(1-CoP)/2 + WR*AX_min*cg/l/2, WR/2 + Cl*VY_max^2*(1-CoP)/2 + WR*AY_max*cg/tw/2];
frontF(2,:) = [0 0 (WF/2+WF*AY_max*cg/tw/2)*AY_max];
rearF(2,:) = [0 0 (WR/2+WR*AY_max*cg/tw/2)*AY_max];
frontF(1,:) = [0 -(WF/2 -WF*AX_min*cg/l/2)*AX_min 0];
rearF(1,:) = [W*AX_max/2 -(WR/2 +WR*AX_min*cg/l/2)*AX_min 0];
%% Section 19: Plot Results
disp('Plotting Results')
% This is just to make some pretty pictures, feel free to comment this out
figure
plot(distance,velocity,'k')
title('Endurance Simulation Velocity Trace')
xlabel('Distance Travelled (d) [ft]')
ylabel('Velocity (V) [ft/s]')
figure
plot(distance,acceleration,distance,lateral_accel)
title('Endurance Simulation Acceleration Traces')
xlabel('Distance Travelled (d) [ft]')
ylabel('Acceleration [g]')
legend('Longitudinal','Lateral')
figure
plot(distance_ax,velocity_ax,'k')
title('Autocross Simulation Velocity Trace')
xlabel('Distance Travelled (d) [ft]')
ylabel('Velocity (V) [ft/s]')
figure
plot(distance_ax,acceleration_ax,distance_ax,lateral_accel_ax)
title('Autocross Simulation Acceleration Traces')
xlabel('Distance Travelled (d) [ft]')
ylabel('Acceleration [g]')
legend('Longitudinal','Lateral')
disp('Analysis Complete')