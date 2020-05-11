%{
Dimitri Pavlis
PositionModel.m
Last edit: 08/31/2018
%}
%% PositionModel

% This model utilizes kinematic equations and the external forces from
% gravity and lift/drag using density. This model tracks the position of a
% subject at specified time intervals.

clc;clear;clf;close;

% Resolution (editable)
delta_t=10^-1; %measurement time interval [s] 0.257 maximum

% System/model parameters (editable)
W=17.792886461; %magnitude sea-level weight force [N]
g_0=9.81; %magnitude sea-level gravitational acceleration [m/(s^2)]
m=W/g_0; %mass [kg]
r_earth=6371009; %radius of Earth
F_thrust=0; %thrust force [N]
cD=0.34582; %drag coefficient [dimensionless]
cL=1.645; %lift coefficient [dimensionless]
A_wings=1*2; %surface area of wings [m^2]
v_0=10; %initial velocity [m/s]

% System/model parameters
x_0=0; %initial x-position [m]
y_0=20000;%[20000,40000,60000,80000]; %initial altitudes [m]
theta_0=0;%-pi/2:pi/180:pi/2; %launch angles [rad]

for i=1:length(y_0) %'for' loop to repeat for each given maximum altitude

    for j=1:length(theta_0) %'for' loop to iterate on angles

%         Initialize indexing value
        k=1;
        
%         Initial conditions
        t(k)=0; %initial time
        theta(k)=theta_0(j); %initial angle
        v=v_0; %initial velocity [m/s]
        x(k)=x_0; %initial x-position
        vx(k)=v*cos(theta(k)); %initial x-velocity
        y(k)=y_0(i); %initial y-position
        vy(k)=v*sin(theta(k)); %initial y-velocity
        
        while y(k)>0
            
            g=g_0*(r_earth/(r_earth+y(k)))^2;
            
            if y(k)>0 && y(k)<=11000 %conditional for air density
                slope=-6.5*10^(-3); %slope [K/m]
                y_low=0;
                T_low=288.16;
                T(k)=slope*(y(k)-y_low)+T_low;
                rho(k)=(T(k)/T_low)^(-(g_0/(slope*R)+1));
            
            elseif y(k)>11000 && y(k)<=25000
                T(k)=216.66;
                T_low=216.66;
                rho(k)=exp(-(g_0/(R*T(k))*(y(k)-y_low)))*rho_low;
            
            elseif y(k)>25000 && y(k)<=47000
                slope=3*10^(-3); %slope [K/m]
                y_low=25000;
                T_low=216.66;
                T(k)=slope*(y(k)-y_low)+T_low;
                rho(k)=(T(k)/T_low)^(-(g_0/(slope*R)+1));
            
            elseif y(k)>47000 && y(k)<=53000
                T(k)=282.66;
                T_low=282.66;
                rho(k+1)=(T(k)/T_low)^(-(g_0/(slope*R)+1));
            
            elseif y(k)>53000 && y(k)<=79000
                slope=-4.5*10^(-3); %slope [K/m]
                y_low=53000;
                T_low=282.66;
                T(k)=slope*(y(k)-y_low)+T_low;
                rho(k)=(T(k)/T_low)^(-(g_0/(slope*R)+1));
            
            elseif y(k)>79000 && y(k)<=90000
                T(k)=165.66;
                T_low=165.66;
                rho(k+1)=(T(k)/T_low)^(-(g_0/(slope*R)+1));
            
            elseif y(k)>90000 && y(k)<=105000
                slope=4*10^(-3); %slope [K/m]
                y_low=90000;
                T_low=165.66;
                T(k)=slope*(y(k)-y_low)+T_low;
                rho(k)=(T(k)/T_low)^(-(g_0/(slope*R)+1));
                
            end
            
            t(k+1)=t(k)+delta_t;
            
            v(k)=sqrt(vx(k)^2+vy(k)^2); %magnitude velocity
            
            FL(k)=(cL*rho(k)*v(k)^2*A_wings)/2; %magnitude lift force
            FD(k)=(cD*rho(k)*v(k)^2*A_wings)/2; %magnitude drag force
            
%             Forces
            W=m*g;
            Fx(k)=FL(k)*cos(pi/2+theta(k))+FD(k)*cos(pi+theta(k));
            Fy(k)=FL(k)*sin(pi/2+theta(k))+FD(k)*sin(pi+theta(k))-W;
            
%             Accelerations
            ax(k)=Fx(k)/m;
            ay(k)=Fy(k)/m;
            
%             Velocities
            vx(k+1)=v(k)*cos(theta(k))+ax(k)*delta_t;
            vy(k+1)=v(k)*sin(theta(k))+ay(k)*delta_t;
            
%             Positions
            x(k+1)=x(k)+vx(k+1)*delta_t+1/2*ax(k)*delta_t^2;
            y(k+1)=y(k)+vy(k+1)*delta_t+1/2*ay(k)*delta_t^2;
            
%             Change in positions
            delta_x=x(k+1)-x(k);
            delta_y=y(k+1)-y(k);
            
            theta(k+1)=atan2(delta_y,delta_x);
            
%             Iterate k indexing value
            k=k+1;
            
        end
        
        x_land(j)=x(end-1);
        tof(j)=t(end-1);
        
        %figure(1)
        %hold on;
        %grid on;
        %plot(tof,x_land)
        
    end
    
end