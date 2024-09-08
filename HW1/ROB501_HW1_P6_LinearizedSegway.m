%% State Space Of the System
syms x1 x2 x3 x4
syms F
syms M
syms m J l
syms c gamma
syms g


% Substitutions

syms Jt Mt


x1_eq = 0;
x2_eq = 0;
x3_eq = 0;
x4_eq = 0;
x_eq = [x1_eq; x2_eq; x3_eq; x4_eq];

F_eq = 0;
u_eq = [F_eq];

x1dot = x2;
x2dot = (F*(J + m*l^2) - (J + m*l^2)*c*x2 - (J + m*l^2)*l*m*x4^2*sin(x3) + g*l^2*m^2*sin(2*x3)/2 - gamma*l*m*x4*cos(x3))/((J + m*l^2)*(M+m) - l^2*m^2*cos(x3)*cos(x3));
x3dot = x4;
x4dot = (F*l*m*cos(x3) + (M+m)*g*l*m*sin(x3) - (M+m)*gamma*x4 - c*l*m*x2*cos(x3) - l^2*m^2*x4^2*sin(2*x3)/2)/((J + m*l^2)*(M+m) - l^2*m^2*cos(x3)*cos(x3));


% Subs for simplifications
x2dot = subs(x2dot, (J + m*l^2), Jt);
x2dot = subs(x2dot, (M + m), Mt);
x4dot = subs(x4dot, (J + m*l^2), Jt);
x4dot = subs(x4dot, (M + m), Mt);



xdot = [x1dot x2dot x3dot x4dot];
x = [x1; x2; x3; x4];
u = [F];


jacx = jacobian(xdot, x);
jacu = jacobian(xdot, u);


A = subs(jacx, x, x_eq);
A = subs(A, u, u_eq)

B = subs(jacu, u, u_eq);
B = subs(B, x, x_eq)