m = 1615;
Jphi = 2800;
V = 5;
Ka = 150000;
Kp = 350000;
a = 1.59;
b = 1.06;
lra = 1.1;
lrp = 0.7;
[A,B,C,D] = bicycle(m,Jphi,V,Ka,Kp,a,b);
% [A2,B2,C2,D2] = bi
bike_sys = ss(A,B,C,D);
% load MPC_bike.mat

%% Simulating system with control input from F458....mat

load F458_Italia_vdd_double_lane_change.mat driver_demands_steering time Vehicle_States_lateral_disp Vehicle_States_lateral_vel_wrt_road Vehicle_States_yaw_angle ...
    Vehicle_States_yaw_angular_vel_wrt_road
u = 0.083*[driver_demands_steering(601:801) zeros(201,1)];
t = time(601:801);
x0 = [Vehicle_States_lateral_disp(601) Vehicle_States_lateral_vel_wrt_road(601) Vehicle_States_yaw_angle(601) ...
    Vehicle_States_yaw_angular_vel_wrt_road(601)];
t_xlog = linspace(6,8,133334);

y = lsim(bike_sys,u,t,x0);
figure(1)
plot(t,u(:,1))
figure(2)
hold on
plot(t,y(:,1))
plot(t_xlog,xLog(:,2))
hold off
figure(3)
hold on
plot(t,y(:,2))
plot(t_xlog,xLog(:,5))
hold off