%% Camera parameters and initial state
%camera position and parameters
xc1 = [0,5,3];
xc2 = [10,5,3];

a = 36/1000; %36mm
b = 27/1000; %27mm
lmbda = 100/1000; %50mm

dt = 0.02;
t = 0:dt:10;
nt = length(t);
b1 = 100;
b2 = 100;

A = [0,0;0,0];
B = [b1,0; 0,b2];
K = eye(2)*0.5;


x01 = [1*pi,0.3*pi];
x02 = [0*pi,0.3*pi];
x1 = zeros(2,nt);
x2 = zeros(2,nt);
x1(:,1) = x01';
x2(:,1) = x02';

F1 = find_fov(x01,xc1,a,b,lmbda);
F2 = find_fov(x02,xc2,a,b,lmbda);
figure(1)
clf
hold on
plot_FOV(F1,0,0,'green')
plot_FOV(F2,0,0,'yellow')
hold off

%% default FOV motion:
v = VideoWriter('default_FOV_motion','MPEG-4');
v.Quality = 100;
v.FrameRate = 25;
open(v);
for i = 1:nt
   
    t_span = [t(i) t(i) + dt];
    F1 = find_fov(x1(:,i),xc1,a,b,lmbda);
    F2 = find_fov(x2(:,i),xc2,a,b,lmbda);
    figure(1)
    clf
    hold on
    plot_FOV(F1,0,t(i),'green')
    plot_FOV(F2,0,t(i),'yellow')
    frame = getframe(gcf);
    writeVideo(v,frame);

    if(i==1)
        x_next1 = x1(:,i) + dt*phidot_default1(x1(:,i),x1(:,i));
        x_next2 = x2(:,i) + dt*phidot_default2(x2(:,i),x2(:,i));
        
    else
        x_next1 = x1(:,i) + dt*phidot_default1(x1(:,i),x1(:,i-1));
        x_next2 = x2(:,i) + dt*phidot_default2(x2(:,i),x2(:,i-1));
    end
    x1(:,i+1) = x_next1;
    x2(:,i+1) = x_next2;
   
    
end
close(v);

%% Initialise target:
xt0 = [5,5,0,0];
u_target = zeros(2,nt);
target(1) = define_moving_target(xt0,u_target,0.5,0.5,0.5,dt,t,2);

%% Show target motion
v = VideoWriter('target_motion','MPEG-4');
v.Quality = 100;
v.FrameRate = 25;
open(v);
for i = 1:nt
    figure(2)
    clf
    plot_moving_cube(target,i,t(i));
    frame = getframe(gcf);
    writeVideo(v,frame);
end
close(v);

%% Simulation:
v = VideoWriter('multiple_cameras_2','MPEG-4');
v.Quality = 100;
v.FrameRate = 25;
open(v);
for i = 1:nt
    figure(3)
    clf
    hold on
    t_span = [t(i) t(i) + dt];
    F1 = find_fov(x1(:,i),xc1,a,b,lmbda);
    F2 = find_fov(x2(:,i),xc2,a,b,lmbda);
    
    detection1 = check_intersection_t(F1,target,i);
    detection2 = check_intersection_t(F2,target,i);
    
    xt = target.location(i,1);
    yt = target.location(i,2);
    
    %For FOV 1
    if(detection1>1)
        plot_FOV(F1,1,t(i),'green');

        d = sqrt((xc1(1) - xt)^2 + (xc1(2) - yt)^2);
        phi_ref = atan(d/xc1(3));
        psi_ref = 3*pi/2 + atan((yt-xc1(2))/(xt - xc1(1)));
        x_ref = [psi_ref;phi_ref];

        state_error = x1(:,i) - x_ref;
        
        x_next1 = x1(:,i) + dt*phidot_track(b1,b2,K,state_error);
        x_next1(2) = min(x_next1(2),0.3*pi);
        x1(:,i+1) = x_next1;
    
    else
        plot_FOV(F1,0,t(i),'green');
        if(i==1)
            x_next1 = x1(:,i) + dt*phidot_default1(x1(:,i),x1(:,i));
        else
            x_next1 = x1(:,i) + dt*phidot_default1(x1(:,i),x1(:,i-1));
        end
        x1(:,i+1) = x_next1;    
    end
    
    %For FOV 2
    if(detection2>1)
        plot_FOV(F2,1,t(i),'yellow');
     
        d = sqrt((xc2(1) - xt)^2 + (xc2(2) - yt)^2);
        phi_ref = atan(d/xc2(3));
        psi_ref = atan(-(xt-xc2(1))/(yt - xc2(2)));
        if(psi_ref < 0)
            psi_ref = psi_ref +pi;
        end
        x_ref = [psi_ref;phi_ref];
        state_error = x2(:,i) - x_ref;
        
        x_next2 = x2(:,i) + dt*phidot_track(b1,b2,K,state_error);
        x_next2(2) = min(x_next2(2),0.3*pi);
        x2(:,i+1) = x_next2;
    
    else
        plot_FOV(F2,0,t(i),'yellow');
        if(i==1)
            x_next2 = x2(:,i) + dt*phidot_default2(x2(:,i),x2(:,i));
        else
            x_next2 = x2(:,i) + dt*phidot_default2(x2(:,i),x2(:,i-1));
        end
        x2(:,i+1) = x_next2;    
    end
   
    plot_moving_cube(target,i)
    plot_plane();
    
    hold off
    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v)



%% Functions
function F = find_fov(x,xc,a,b,lmbda)
%Returns 4 vertices of the FOV on ground plane and the camera position as a
% 3x5 matrix

% INPUTS: x = state vector
%         xc = camera position
%         a,b = size of image sensor (in meters)
%         lmbda = focal length of camera

% OUTPUTS: F = 3x5 matrix of vertices forming camera FOV in 3D

    %FOV vertices in camera frame
    q1 = [a/2 b/2 lmbda];
    q2 = [a/2 -b/2 lmbda];
    q3 = [-a/2 b/2 lmbda];
    q4 = [-a/2 -b/2 lmbda];

    Q = [q1' q2' q3' q4'];

    psi = x(1);
    phi = x(2);

    %Rotation matrices
    H_psi = [cos(psi),-sin(psi),0;
    sin(psi),cos(psi),0;
    0, 0, 1];
    
    H_phi = [1, 0, 0;
    0, cos(phi), -sin(phi);
    0, sin(phi), cos(phi)];

    r1 = -xc(3)/(0.5*b*sin(phi) + lmbda*cos(phi));
    r2 = -xc(3)/(-0.5*b*sin(phi) + lmbda*cos(phi));
    r3 = r1;
    r4 = r2;
    
    Q_I_temp = H_psi*H_phi*Q;
    
    Q_I = [r1*Q_I_temp(:,1),r2*Q_I_temp(:,2),r3*Q_I_temp(:,3),r4*Q_I_temp(:,4)];
    
    X_L = repmat(xc,4,1)' + Q_I;
   
    F = [xc', X_L];

end

function plot_FOV(F,detection,t,colour)

    if(detection)
        clr = 'red';
    else
        clr = colour;
    end
    
    FV.Vertices = F';
    FV.Faces = [1 2 3; 1 2 4; 1 3 5; 1 5 4];
    FV.FaceColor = clr;
    patch(FV,'FaceAlpha',0.5)
    view (3)
    xlim([0 10]);
    ylim([0 10]);
    zlim([0 5]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
    grid on;  
    title(append('t = ',num2str(t)));
    
end

function target = define_moving_target(x0,u,l,b,h,dt,t,mode)

    nt = length(t);

    if (mode == 0)
        A = [1,0,dt,0;0,1,0,dt;0,0,1,0;0,0,0,1];
        B = [0,0; 0,0; 0.1,0;0,0.1];
        C = [1,0,0,0;0,1,0,0];
        D = [0,0;0,0];


        sys_target = ss(A,B,C,D,dt);
        y = lsim(sys_target,u,t,x0);
    end
    if (mode == 1)
        y =  [ones(nt,1)*x0(1),x0(2) + 3*sin(pi*t)'];    
    end
    if(mode == 2)
        y = [x0(1) + 4*cos(pi*t)',x0(2) + 3*sin(pi*t)'];
    end
    if(mode == 3)
        y = [x0(1) + 4.5*sin(pi/2*t)',x0(2) + 4.5*sin(pi*t)'];
    end
    
    target.location = y;
    target.length = l;
    target.breadth = b;
    target.height = h;
end

function plot_moving_cube(targets,t_index)

    N = length(targets);
    hold on
    for i = 1:N
        target = targets(i);
        xt = target.location(t_index,:);
        l = target.length;
        b = target.breadth;
        h = target.height;

        x = xt(1);
%         x = 4 + 2*cos(5/2/pi*t);
        y = xt(2);
%         y = 4 + 2*sin(5/2/pi*t);

        V = [x-l/2, y+b/2, h;x+l/2, y+b/2, h;x+l/2, y+b/2, 0;x-l/2, y+b/2, 0;
            x-l/2, y-b/2, 0;x-l/2, y-b/2, h;x+l/2, y-b/2, h;x+l/2, y-b/2, 0];
        FV.Vertices = V;
        FV.Faces = [1 2 3 4;5 6 7 8;1 4 5 6;2 3 8 7; 3 4 5 8; 1 2 7 6];
        FV.FaceColor = 'blue';
        patch(FV,'FaceAlpha', 0.9);
        view(3)
    end
    hold off
    xlim([0 10]);
    ylim([0 10]);
    zlim([0 5]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
    grid on;  
end

function result = check_intersection_t(F,targets,t_index)
    
    N = length(targets);
    result = 0;
    for i = 1:N
        target = targets(i);
        xt = target.location(t_index,1);
%         xt = 4 + 2*cos(5/2/pi*t);
        yt = target.location(t_index,2);
%         yt = 4 + 2*sin(5/2/pi*t);
        l = target.length;
        b = target.breadth;
        h = target.height;

        shp = alphaShape(F');

        x_range = linspace((xt-l/2),(xt+l/2),5);
        y_range = linspace((yt-b/2),(yt+b/2),5);
        z_range = linspace(0,h,5);
        [X,Y,Z] = meshgrid(x_range,y_range,z_range);

        result = result + sum(inShape(shp,X,Y,Z),'all');
    end

end

function xdot = phidot_track(b1,b2,K,e)
    
    xdot = [-b1*K(1,1)*e(1);-b2*K(2,2)*e(2)];
  
end

function xdot = phidot_default1(x_curr,x_prev)
   
    psi_prev = x_prev(1);
    psi_curr = x_curr(1);
    
    omega = 3*pi/2;
    
    if (psi_curr > 1.95*pi)
        psi_dot = -omega;
    elseif (psi_curr < 1.05*pi)
        psi_dot = omega;
    elseif (psi_curr > psi_prev)
        psi_dot = omega;
    else
        psi_dot = -omega;
    end
    
    phi_dot = 0;
    
    xdot = [psi_dot;phi_dot];
   
end

function xdot = phidot_default2(x_curr,x_prev)
   
    psi_prev = x_prev(1);
    psi_curr = x_curr(1);
    
    omega = 3*pi/2;
    
    if (psi_curr < 0.05*pi)
        psi_dot = omega;
    elseif (psi_curr > 0.95*pi)
        psi_dot = -omega;
    elseif (psi_curr > psi_prev)
        psi_dot = omega;
    else
        psi_dot = -omega;
    end
    
    phi_dot = 0;
    
    xdot = [psi_dot;phi_dot];
   
end

function plot_plane()
    FV.Vertices = [5,0,10;5,0,0;5,10,0;5,10,10];
    FV.Faces = [1,2,3,4];
    FV.FaceColor = 'black';
    patch(FV,'FaceAlpha',0.5)
    view (3)
end




