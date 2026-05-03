%%Lecture 9 example: code courtesy of Dr.Kamrin


clear all
close all
global m mus mud g l0 k v0 tol;
% Pick the numerical properties. These have been made "global" so even
% matlab functions know what they are.
m=1; mus=.4; mud=.2; g=9.81; l0=.01; v0=.05; tol=.001*v0;
k=500;
%k=1000;
%k=50000;
% Run ode45 up to the time required for the pring to move ten l0's.
[t,U]=ode45(@stickslip,[0:.01:10*l0/v0],[l0,0]);
for i=1:length(t)
% Plot the back end of the spring with an "x"
plot(v0*t(i),0,'x',MarkerSize=14,LineWidth=3)
hold on
% Plot the box as a box.
plot(U(i,1),0,'s',MarkerSize=20,LineWidth=3)
plot([v0*t(i),U(i,1)],[0,0],LineWidth=3)
axis([0,10*l0,-1,1])
getframe
hold off
end
% Plot the velocity as a function of time.
plot(t,U(:,2))
xlabel('t',FontSize=16)
ylabel('v',FontSize=16)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Udot = stickslip(t,U)
global m mus mud g l0 k v0 tol;
x=U(1);
xdot=U(2);
% Define the x-componenet of the spring force
Fs_x=-k*(abs(x-v0*t)-l0)*sign(x-v0*t);
% Define the x-componenet of the trial force
Ftrial_x=-Fs_x;
% Go through all three friction cases to define F_t
if abs(xdot)<tol & abs(Ftrial_x)<mus*m*g
Ft_x=Ftrial_x;
elseif abs(xdot)<tol & abs(Ftrial_x)>=mus*m*g
Ft_x=mus*m*g*sign(Ftrial_x);
else
Ft_x=-mud*m*g*sign(xdot);
end
Udot1=xdot;
% Use sum(F_x)=m*a_x to update v.
Udot2=(Ft_x+Fs_x)/m;
Udot=[Udot1;Udot2];
end
