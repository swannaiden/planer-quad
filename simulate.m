%% simulate quadrotor using piccard iteration
clear

x0 = [0,0,pi/6,0,-1,0]';

dt = .1;
time = 15;
x(:,1) = x0;
t = 0:dt:time-dt;

for i = 2:(time/dt)
    x(:,i) = x(:,i-1)+qdynamics(0, x(:,i-1))*dt;
end


figure(1)
subplot(2,3,1)
plot(t,x(1,:))
xlabel('t (s)')
ylabel('x(t)')

subplot(2,3,2)
plot(t,x(2,:))
xlabel('t (s)')
ylabel('y(t)')

subplot(2,3,3)
plot(t,x(3,:))
xlabel('t (s)')
ylabel('\phi (t)')

subplot(2,3,4)
plot(t,x(4,:))
xlabel('t (s)')
ylabel('dx(t)')

subplot(2,3,5)
plot(t,x(5,:))
xlabel('t (s)')
ylabel('dy(t)')

subplot(2,3,6)
plot(t,x(6,:))
xlabel('t (s)')
ylabel('d\phi (t)')


%% animate trajectory
figure(2)
for i = 1:length(x)
    plot(x(1,1:i), x(2,1:i))
    hold on
    plot(10*ones(1,20),linspace(-1,1,20))
    hold off
    theta = 5*x(3,i);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    x1 = R*[0;1];
%     y1 = R*[1;0];
    hold on
    %quiver([x(1,i), x(1,i)], [x(2,i),x(2,i)], [x1(1), x1(2)],[x1(2), x1(1)])
    quiver([x(1,i),x(1,i)], [x(2,i), x(2,i)], [x1(2), -x1(2)],[-x1(1), x1(1)], .15)
    hold off
    %quiver(x(1,i), x(2,i), -x1(1),-x1(2))
    axis([5 11 -1 1])
    drawnow
end


%% simulate quadrotor using ode45

tspan = [0, time];
[t, x2] = ode45(@qdynamics, tspan, x0);


figure(1)
subplot(2,3,1)
hold on
plot(t,x2(:,1))
xlabel('t (s)')
ylabel('x(t)')
legend('Picard', 'ODE45')

subplot(2,3,2)
hold on
plot(t,x2(:,2))
xlabel('t (s)')
ylabel('y(t)')

subplot(2,3,3)
hold on
plot(t,x2(:,3))
xlabel('t (s)')
ylabel('\phi (t)')

subplot(2,3,4)
hold on
plot(t,x2(:,4))
xlabel('t (s)')
ylabel('dx(t)')

subplot(2,3,5)
hold on
plot(t,x2(:,5))
xlabel('t (s)')
ylabel('dy(t)')

subplot(2,3,6)
hold on
plot(t,x2(:,6))
xlabel('t (s)')
ylabel('d\phi (t)')