%==========================================================================
% ME 652, Spring 2017
% Course instructor: Jinwhan Kim
%==========================================================================

function [] = SLAM_skeleton()

rng(1);
DTR = pi/180;   % degree to radian

% true initial condition
x = [1; 1; 45*DTR; 1;   % x, y, psi, V
     4.6454;  8.0242;   % landmark 1 (x1,y1)
     7.5198;  4.7523;   % landmark 2 (x2,y2)
     1.6836;  5.4618;   % landmark 3 (x3,y3)
     5.5984;  3.1042;   % landmark 4 (x4,y4)
     8.4626;  1.6814];  % landmark 5 (x5,y5)

% initial error covariance
Phat = eye(14);
Phat(1,1) = 0;
Phat(2,2) = 0;

% process noise
Qe = zeros(14,14);

% measurement noise
Re = eye(12);
Re(1,1) = (3*DTR)^2;
Re(2,2) = 0.2^2;

for i = 1:5
    Re(i*2+1,i*2+1) = (3*DTR)^2;  % bearing noise
    Re(i*2+2,i*2+2) = 0.2^2;      % range noise
end

% initial estimate
xhat = x;
xhat(5:14) = x(5:14) + sqrt(Phat(5:14,5:14))*randn(10,1);

figure;
set(gcf,'position',[100 100 500 500])

dt = 0.1;   % simulation time step

for t = 0:dt:10

    % true state propagation
    x = propagate_x(x,dt);

    % generate bearing and range measurements
    z = generate_z(x,Re);

    % EKF prediction step
    [xhat,Phat] = EKF_propagate(xhat,Phat,Qe,dt);

    % EKF measurement update step
    [xhat,Phat] = EKF_update(xhat,z,Phat,Re);

    clf
    show_data(t,x,xhat,Phat);
    drawnow;

end

end


%==========================================================================
% Generate measurement vector
%==========================================================================
function z = generate_z(x,Re)

z = zeros(12,1);

% vehicle heading and velocity measurements
z(1) = x(3);   % psi
z(2) = x(4);   % V

% landmark bearing and range measurements
for i = 1:5

    lx_index = i*2 + 3;
    ly_index = i*2 + 4;

    dx = x(lx_index) - x(1);
    dy = x(ly_index) - x(2);

    beta = atan2(dy, dx) - x(3);
    rho = sqrt(dx^2 + dy^2);

    beta_row = i*2 + 1;
    rho_row = i*2 + 2;

    z(beta_row) = wrap_pi(beta);
    z(rho_row) = rho;

end

% add measurement noise
z = z + chol(Re,'lower') * randn(12,1);

% wrap angle measurements
z(1) = wrap_pi(z(1));

for i = 1:5
    beta_row = i*2 + 1;
    z(beta_row) = wrap_pi(z(beta_row));
end

end


%==========================================================================
% EKF prediction step
%==========================================================================
function [xhat,Phat] = EKF_propagate(xhat,Phat,Qe,dt)

psi = xhat(3);
V = xhat(4);

% nonlinear state propagation
xhat(1) = xhat(1) + V*cos(psi)*dt;
xhat(2) = xhat(2) + V*sin(psi)*dt;
xhat(3) = xhat(3);
xhat(4) = xhat(4);

% landmarks do not move

% Jacobian of motion model
F = eye(14);

F(1,3) = -V*sin(psi)*dt;
F(1,4) = cos(psi)*dt;

F(2,3) = V*cos(psi)*dt;
F(2,4) = sin(psi)*dt;

% covariance prediction
Phat = F*Phat*F' + Qe;

% wrap heading
xhat(3) = wrap_pi(xhat(3));

end


%==========================================================================
% EKF measurement update step
%==========================================================================
function [xhat,Phat] = EKF_update(xhat,z,Phat,Re)

zhat = zeros(12,1);
H = zeros(12,14);

% predicted heading and speed measurements
zhat(1) = xhat(3);
zhat(2) = xhat(4);

H(1,3) = 1;
H(2,4) = 1;

% predicted landmark measurements
for i = 1:5

    lx_index = i*2 + 3;
    ly_index = i*2 + 4;

    beta_row = i*2 + 1;
    rho_row = i*2 + 2;

    dx = xhat(lx_index) - xhat(1);
    dy = xhat(ly_index) - xhat(2);

    q = dx^2 + dy^2;
    rho = sqrt(q);

    % predicted bearing and range
    zhat(beta_row) = wrap_pi(atan2(dy, dx) - xhat(3));
    zhat(rho_row) = rho;

    % bearing Jacobian
    H(beta_row,1) = dy/q;
    H(beta_row,2) = -dx/q;
    H(beta_row,3) = -1;
    H(beta_row,lx_index) = -dy/q;
    H(beta_row,ly_index) = dx/q;

    % range Jacobian
    H(rho_row,1) = -dx/rho;
    H(rho_row,2) = -dy/rho;
    H(rho_row,lx_index) = dx/rho;
    H(rho_row,ly_index) = dy/rho;

end

% innovation
innovation = z - zhat;

% wrap angle innovations
innovation(1) = wrap_pi(innovation(1));

for i = 1:5
    beta_row = i*2 + 1;
    innovation(beta_row) = wrap_pi(innovation(beta_row));
end

% Kalman gain
S = H*Phat*H' + Re;
K = (Phat*H') / S;

% state update
xhat = xhat + K*innovation;

% covariance update using Joseph form
I = eye(14);
Phat = (I - K*H)*Phat*(I - K*H)' + K*Re*K';

% wrap heading
xhat(3) = wrap_pi(xhat(3));

end


%==========================================================================
% True state propagation
%==========================================================================
function x = propagate_x(x,dt)

xdot = zeros(14,1);

xdot(1) = x(4)*cos(x(3));   % xdot
xdot(2) = x(4)*sin(x(3));   % ydot
xdot(3) = 0;                % yaw rate
xdot(4) = 0;                % speed change

x = x + xdot*dt;

x(3) = wrap_pi(x(3));

end


%==========================================================================
% Plot data
%==========================================================================
function show_data(t,x,xhat,Phat)

hold on

L = 0.6;     % vehicle length
B = 0.3;     % vehicle breadth

objx = L*[1 -1 -1 1]';
objy = B*[0,1,-1,0]';

% true vehicle
posx = x(1) + objx*cos(x(3)) - objy*sin(x(3));
posy = x(2) + objx*sin(x(3)) + objy*cos(x(3));

fill(posx,posy,'y');
plot(posx,posy,'b');

% estimated vehicle
posx = xhat(1) + objx*cos(xhat(3)) - objy*sin(xhat(3));
posy = xhat(2) + objx*sin(xhat(3)) + objy*cos(xhat(3));

plot(posx,posy,'r');

plot(0,0,'bo',x(1),x(2),'bo',xhat(1),xhat(2),'r+');

% 95% confidence ellipse
alpha = 5.9915;   % chi2inv(0.95,2)

% vehicle uncertainty ellipse
meanval = [xhat(1),xhat(2)];
sigma_u = Phat(1:2,1:2);
plot2Dellipse(alpha,meanval,sigma_u,'r');

% landmark uncertainty ellipses
for i = 1:5

    lx_index = i*2 + 3;
    ly_index = i*2 + 4;

    plot(xhat(lx_index),xhat(ly_index),'r+', ...
         x(lx_index),x(ly_index),'k+');

    meanval = [xhat(lx_index),xhat(ly_index)];
    sigma_u = Phat(lx_index:ly_index,lx_index:ly_index);

    plot2Dellipse(alpha,meanval,sigma_u,'r');

end

s = sprintf('Time = %8.2f',t);
text(1,9,s,'fontsize',15);

xlabel('x','fontsize',15)
ylabel('y','fontsize',15)

set(gca,'fontsize',15,'PlotBoxAspectRatio',[500 500 100]);
set(gca,'Ydir','reverse');

axis([0 10 0 10]);
grid on;
box on;

end


%==========================================================================
% Draw 2D confidence ellipse
%==========================================================================
function [] = plot2Dellipse(alpha,meanval,sigma,color)

[V,D] = eig(sigma);

a = sqrt(alpha*D(1,1));
b = sqrt(alpha*D(2,2));

ang1 = atan2(V(2,1),V(1,1));

theta = (0:360)/180*pi;

x = a*cos(theta);
y = b*sin(theta);

xn = x*cos(ang1) - y*sin(ang1) + meanval(1);
yn = x*sin(ang1) + y*cos(ang1) + meanval(2);

plot(xn,yn,color,'LineWidth',2);

end


%==========================================================================
% Wrap angle to [-pi, pi]
%==========================================================================
function angle = wrap_pi(angle)

angle = mod(angle + pi, 2*pi) - pi;

end