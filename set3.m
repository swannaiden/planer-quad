%% perturb intial conditions

%% original conditions

clear

x0 = [0,0,0,0,0,0]';

dt = .1;
time = 15;
x(:,1) = x0;
t = 0:dt:time-dt;

for i = 2:(time/dt)
    x(:,i) = x(:,i-1)+qdynamics(0, x(:,i-1))*dt;
end


x02 = [0,0,pi/12,0,0,0]';

dt = .1;
time = 15;
x2(:,1) = x02;
t = 0:dt:time-dt;

for i = 2:(time/dt)
    x2(:,i) = x2(:,i-1)+qdynamics(0, x2(:,i-1))*dt;
end

%% plot

figure(1)
subplot(2,3,1)
plot(t,x(1,:))
hold on
plot(t, x2(1,:))
xlabel('t (s)')
ylabel('x(t)')

subplot(2,3,2)
plot(t,x(2,:))
hold on
plot(t, x2(2,:))
xlabel('t (s)')
ylabel('y(t)')

subplot(2,3,3)
plot(t,x(3,:))
hold on
plot(t, x2(3,:))
xlabel('t (s)')
ylabel('\phi (t)')

subplot(2,3,4)
plot(t,x(4,:))
hold on
plot(t, x2(4,:))
xlabel('t (s)')
ylabel('dx(t)')

subplot(2,3,5)
plot(t,x(5,:))
hold on
plot(t, x2(5,:))
xlabel('t (s)')
ylabel('dy(t)')

subplot(2,3,6)
plot(t,x(6,:))
hold on
plot(t, x2(6,:))
xlabel('t (s)')
ylabel('d\phi (t)')

%% plot norms

figure(2)
for i = 1:length(t)
    error(i) = norm(x2(:,i));
end
plot(t, error);
title('Thm 3.2 bound w/ purturbed initial condition')
xlabel('t (s)','interpreter','latex')
ylabel('$||x(t) - z(t)||$','interpreter','latex')

%% plot bound

L = 4.5;

figure(3)
plot(t(1:10), error(1:10));
syms nor ts
nor(ts) = norm(x0-x02)*exp(L*ts);
hold on
plot(t(1:10), nor(t(1:10)))
hold off

title('Thm3.2 bound on purturbed initial conditions')
xlabel('t (s)','interpreter','latex')
ylabel('$||x(t) - z(t)||$','interpreter','latex')

%% now try purturbing system dynamics


clear

x0 = [0,0,0,0,0,0]';

dt = .1;
time = 15;
x(:,1) = x0;
t = 0:dt:time-dt;

for i = 2:(time/dt)
    x(:,i) = x(:,i-1)+qdynamics(0, x(:,i-1))*dt;
end


x02 = [0,0,0,0,0,0]';

dt = .1;
time = 15;
x2(:,1) = x02;
t = 0:dt:time-dt;

for i = 2:(time/dt)
    x2(:,i) = x2(:,i-1)+mod_qdynamics(0, x2(:,i-1))*dt;
end

%% plot

figure(1)
subplot(2,3,1)
plot(t,x(1,:))
hold on
plot(t, x2(1,:))
xlabel('t (s)')
ylabel('x(t)')

subplot(2,3,2)
plot(t,x(2,:))
hold on
plot(t, x2(2,:))
xlabel('t (s)')
ylabel('y(t)')

subplot(2,3,3)
plot(t,x(3,:))
hold on
plot(t, x2(3,:))
xlabel('t (s)')
ylabel('\phi (t)')

subplot(2,3,4)
plot(t,x(4,:))
hold on
plot(t, x2(4,:))
xlabel('t (s)')
ylabel('dx(t)')

subplot(2,3,5)
plot(t,x(5,:))
hold on
plot(t, x2(5,:))
xlabel('t (s)')
ylabel('dy(t)')

subplot(2,3,6)
plot(t,x(6,:))
hold on
plot(t, x2(6,:))
xlabel('t (s)')
ylabel('d\phi (t)')

%% plot norms

figure(2)
for i = 1:length(t)
    error(i) = norm(x2(:,i));
end
plot(t, error);
title('Thm 3.2 bound w/ purturbed dynamics')
xlabel('t (s)','interpreter','latex')
ylabel('$||x(t) - z(t)||$','interpreter','latex')




%% plot bound

L = 4.5;
mu = 9.8;


figure(3)
plot(t(1:10), error(1:10));
syms nor ts
nor(ts) = mu*exp(L*ts);
hold on
plot(t(1:10), nor(t(1:10)))
hold off

title('Thm 3.2 bound w/ purturbed dynamics')
xlabel('t (s)','interpreter','latex')
ylabel('$||x(t) - z(t)||$','interpreter','latex')


