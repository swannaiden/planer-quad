clear
syms xdot ydot phidot y x phi p
k1 = 1;
k2 = 1;
m = 1;
g = 9.81;
Ixx = 1;

f = [xdot, ydot, phidot, (m*g-k1*ydot)*-sin(phi)/m, (m*g-k1*ydot)*cos(phi)/m-g,  k2*phi/Ixx];
state = {x, y, phi, xdot, ydot, phidot};

A = jacobian(f,[state{:}]);

L(phi,ydot) = norm( [
[0, 0,                         0, 1,         0, 0]
[0, 0,                         0, 0,         1, 0]
[0, 0,                         0, 0,         0, 1]
[0, 0, cos(phi)*(ydot - 981/100), 0,  sin(phi), 0]
[0, 0, sin(phi)*(ydot - 981/100), 0, -cos(phi), 0]
[0, 0,                         1, 0,         0, 0]]);

% [X,Y] = meshgrid(-pi:pi, -10:.9:10);
% Z = double(L(X, Y));
% 
% surf(X,Y,Z)
% xlabel('\phi')
% ylabel('ydot')
% zlabel('|J|')
% title('Norm of Jacobian')

%% find the lyapunov function 

clear
syms xdot ydot phidot y x phi p
k1 = 1;
k2 = 1;
m = 1;
g = 9.81;
Ixx = 1;

f = [ydot, -k2*phi, (-k1*ydot)*cos(phi)];
state = {y, phi, ydot};

%%

A = jacobian(f,[state{:}]);

A = subs(A, [phi,ydot], [0,0]);
% Q = -2.*diag([.5, .5, .5, .5, .5, .5]);
P = -1.*diag([0,0,0,-1,-1,-1]);

Q = A'*P+P'*A



double(eig(Q))



%% LQR control

clear
syms xdot ydot phidot y x phi p
k1 = 1;
k2 = 1;
m = 1;
g = 9.81;
Ixx = 1;

f = [xdot, ydot, phidot, 0, -g, 0];
state = {x, y, phi, xdot, ydot, phidot};

A = jacobian(f,[state{:}]);
A = [[0, 0, 0, 1, 0, 0],
[0, 0, 0, 0, 1, 0],
[0, 0, 0, 0, 0, 1],
[0, 0, -g, 0, 0, 0],
[0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0]];
B = [0,0;0,0;0,0;0,0;1/m,0;0,1/Ixx];

Q = 100.*diag([.1,.1,.1,.1,.1,.1]);
R = 1.*diag([.1,.1]);
N = [0,0;0,0;0,0;0,0;0,0;0,0];

[K,S,e] = lqr(A, B, Q, R);

%% try to solve for lyupnov equation
Q = 10.*diag([.1,.1,.1,.1,.1,.1]);
X = lyap((A-B*K), Q);

%% now caculate radius of convergence
clearvars g
count = 1;
for i = 0:.001:1
    g(count) = norm((A-B*K)*[0,0,i,0,0,0]'-qdynamics([0,0,i,0,0,0]', K*[0,0,i,0,0,0]'));
    normx(count) = norm([0,0,i,0,0,0]');
    count = count+1;
end
plot(normx, g)

%% simulate the quadrotor
clearvars -except K
x0 = [.09,0,0,0,0,0]';

dt = .05;
time = 10;
x(:,1) = x0;
t = 0:dt:time-dt;

for i = 2:(time/dt)
%     u = controller(x(:,i-1),5);
%     [ubar, lambda(i),h(i)] = barrier(x(:,i-1),u);
    x(:,i) = x(:,i-1)+qdynamics(x(:,i-1), -K*x(:,i-1))*dt;
end

%% animate trajectory
% record video
myVideo = VideoWriter('hoverController2'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

figure(2)
for i = 1:length(x)
    plot(x(1,1:i), x(2,1:i))
    hold on
%     plot(10*ones(1,20),linspace(-1,1,20))
    hold off
    theta = 1*x(3,i);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    x1 = R*[0;1];
%     y1 = R*[1;0];
    hold on
    %quiver([x(1,i), x(1,i)], [x(2,i),x(2,i)], [x1(1), x1(2)],[x1(2), x1(1)])
    quiver([x(1,i),x(1,i)], [x(2,i), x(2,i)], [x1(2), -x1(2)],[-x1(1), x1(1)], .15)
    hold off
    %quiver(x(1,i), x(2,i), -x1(1),-x1(2))
%     axis([0 20 -.5 .5 ])
    xlabel('x (m)')
    ylabel('y (m)')
    title('Accelerating to 5m/s')
    pause(.1)
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
%     drawnow
end