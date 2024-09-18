clear all
close all
clc

global m xref K1 K2

m = 5; x0 = [0 0]'; xref = 1; %step input for reference
K1 = 33; K2 = 20.0; %set control gains

A = [0 1; -K1/m -K2/m];
[V,D] = eig(A);
a = real(D(1,1));
b = imag(D(1,1));
omega_n = sqrt(a^2+b^2);
zeta = sqrt(1/(1+(b/a)^2));

% calculated response characteristics from gains
Ts = 4/(zeta*omega_n)
Mp = 1+exp(-pi*zeta/(sqrt(1-zeta^2)))
PO = (Mp-1)*100
Tr = (pi-acos(zeta))/(omega_n*sqrt(1-zeta^2))
Tp = pi/(omega_n*sqrt(1-zeta^2))


[T,X] = ode45(@masprd,[0:0.01:10],x0);

figure
axis([-0.5 1.5 -0.5 0.5])
hold on
grid
% for i=1:length(X(:,1))
%     h=plot(X(i,1), 0, 'LineWidth', 2, 'Marker', 'x', 'MarkerSize', 10);
% pause(eps)
% axis([-0.5 1.5 -0.5 0.5])
% %     axis equal
% delete(h)
% i=i+1;
% end

%%
figure
plot(T,X(:,1), 'LineWidth', 2), hold on; grid; xlabel('t [sec]'), ylabel('x_1 [m]')
hold on
plot(Ts, 0, Tr, 0, Tp, 0, 'Marker', 'x', 'MarkerSize', 10)
line([0 T(end)],[1.02 1.02], 'LineStyle', '--', 'Color', 'k')
line([0 T(end)],[0.98 0.98], 'LineStyle', '--', 'Color', 'k')
line([Tr Tr],[0 1.5], 'LineStyle', '--', 'Color', 'r')
line([Tp Tp],[0 1.5], 'LineStyle', '--', 'Color', 'r')

%figure
%plot(T,X(:,2), 'LineWidth', 2), hold on; grid; xlabel('t [sec]'), ylabel('x_2 [m]')


function [xdot] = masprd(t,x)

global m xref K1 K2
xdot = zeros(2,1);

u = -K1*(x(1)-xref)-K2*x(2);

xdot(1) = x(2);
xdot(2) = u/m;

end