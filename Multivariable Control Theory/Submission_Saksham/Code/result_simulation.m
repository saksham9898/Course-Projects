% clf
% figure(1)
% xlim([0,3])
% ylim([-0.5,2.5])
% c = [0 0.4470 0.7410];
% hold on
% roadPoints = [0,3,3,0;0,0,2,2];
% cgray = [0.5774,0.5774,0.5774];
% patch(roadPoints(1,:),roadPoints(2,:),cgray)
% plot(linspace(0,5,10),2*ones(10),'color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
% plot(linspace(0,5,10),0*ones(10),'color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
% plot(linspace(0,5,10),1*ones(10),'--w','LineWidth',3)
l = 0.25;
b = 0.05;
x0 = 0.5;
% y = 0.5;
% psi = pi;
% points = Rot(psi)*[l, -l, -l, +l;
%             b, b, -b, -b] + [x0;y];
% patch(points(1,:),points(2,:),c)

%% Simulation
V = 5; %m/s
figure(2)
clf
pointsR = [l, -l, -l, +l;
            b, b, -b, -b];
psi = data_obstacles{2}.Values.Data;
y = data_obstacles{1}.Values.Data;
v = VideoWriter('sim_video','MPEG-4');
v.Quality = 100;
v.FrameRate = 40;
open(v);
t = data_obstacles{1}.Values.Time; %get from sim data
N = length(t);
% x = 5*t;
for i = 1:N
    clf
    xlim([0,3])
    ylim([-0.5,2.5])
    c = [0 0.4470 0.7410];
    hold on
    plot_road()
    pointsG = Rot(psi(i))*pointsR + [x0;y(i)];
    patch(pointsG(1,:),pointsG(2,:),c)
    plot_barrier(10-5*t(i),0.5);
    plot_barrier(20-5*t(i),1.5);
    plot_barrier(35-5*t(i),0.5);
    plot_barrier(35-5*t(i),1.5);
    
    title(append('t = ',num2str(round(t(i),1))))
    frame = getframe(gcf);
    writeVideo(v,frame);
    
end
close(v);
    
function R = Rot(psi)
R = [cos(psi),-sin(psi);sin(psi),cos(psi)];
end
function plot_barrier(xc,yc)
l = 0.5;
b = 0.25;
c2 = [0.8500 0.3250 0.0980];
point = [l, -l, -l, +l;
            b, b, -b, -b] + [xc;yc];
patch(point(1,:),point(2,:),c2);
end

function plot_road()
roadPoints = [0,3,3,0;0,0,2,2];
cgray = [0.5774,0.5774,0.5774];
patch(roadPoints(1,:),roadPoints(2,:),cgray)
plot(linspace(0,5,10),2*ones(10),'color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
plot(linspace(0,5,10),0*ones(10),'color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
plot(linspace(0,5,10),1*ones(10),'--w','LineWidth',3)
end

