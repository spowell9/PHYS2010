function balle2010(y1,speed,airFlag,tau,shape)                          
%% balle2010 - Program to compute the trajectory of a package            
%         using the midpoint method.                                    
% invoke as: balle2010(500,90,1,0.5,0)                                                     
% y1 = intial height of package(meters)                                                        
% speed = intial speed of aircraft (m/s)                                                             
% airFlag specifies whether there is air resistance: enter 1            
% for yes, 0 for no                                                     
% tau = timestep, tau(seconds)
% shape specifies the shape of the package: enter 0 for sphere, 1 for cube %SP 
% Original by AJG; modified by Katrina Carver and Sarah Powell 20200404           
help balle2010; % print header                                 


%% * Set initial position and velocity of the package
r1 = [0, y1];     % Initial vector position
v1 = [speed, 0];     % Initial velocity
r = r1;  v = v1;  % Set initial position and velocity

%% * Set physical parameters (mass, Cd, etc.)
if shape == 0 %SP
    Cd = 0.47;      % Drag coefficient (dimensionless) %Cd=12v/Rv=24/Re Cd~0.47 for sphere %SP
    area = 7.1e-2;  % Cross-sectional area of projectile (m^2) %rad=15cm %SP
else 
    Cd = 1.05;      % Drag coefficient (dimensionless) %Cd=12v/Rv=24/Re Cd~1.05 for cube %SP
    area = 5.85e-2;  % Cross-sectional area of projectile (m^2) %Gives same volume as sphere with r=15cm %SP
end

grav = 9.81;    % Gravitational acceleration (m/s^2)
mass = 4;   % Mass of projectile (kg) 
if( airFlag == 0 )
  rho = 0;      % No air resistance
else
  rho = 1.2;    % Density of air (kg/m^3)
end
air_const = -0.5*Cd*rho*area/mass;  % Air resistance constant

%% * Loop until package hits ground or max steps completed
maxstep = 1500;   % Maximum number of steps

for istep=1:maxstep

  %* Record position (computed and theoretical) for plotting
  xplot(istep) = r(1);   % Record trajectory for plot
  yplot(istep) = r(2);
  t = (istep-1)*tau;     % Current time
  xNoAir(istep) = r1(1) + v1(1)*t;
  yNoAir(istep) = r1(2) + v1(2)*t - 0.5*grav*t^2;
  
  %* Calculate the acceleration of the ball 
  accel = air_const*norm(v)*v;   % Air resistance
  accel(2) = accel(2)-grav;      % Gravity
  
 
  %* Calculate the new position and velocity using midpoint method %SP
  r = r + tau*((v + tau*accel)+v)/2; %uses average of previous and new velocity to calculate position %SP
  v = v + tau*accel;

  rx(istep)=r(1); ry(istep)=r(2); %recording the components of r for each iteration
  
  %* If package reaches ground (y<0), break out of the loop
  if( r(2) < 0 )  
    xplot(istep+1) = r(1);  % Record last values computed
	yplot(istep+1) = r(2);
    break;                  % Break out of the for loop
  end 
end
%% * Correcting maximum range and time %SP
last3_x=[rx(end-2), rx(end-1), rx(end)]; % creating a vector of the last 3 x-components of r
last3_y=[ry(end-2), ry(end-1), ry(end)]; % creating a vector of the last 3 y-components of r

corrected_range=intrpf(0,last3_y,last3_x); %Finds the horizontal range by interpolating between the last 3 position points.


corrected_time=(istep*tau)-((rx(end)-corrected_range)/v(1)); 
%calulates the corrected time by subtracting the change in displacement in 
%the x direction divided by the velocity in the x direction in order to 
%account for extra time elapsed.

%% * Print maximum range and time of flight
fprintf('Maximum range is %g meters\n',corrected_range); %prints max range
fprintf('Time of flight is %g seconds\n',corrected_time); %prints time of flight

%% * Graph the trajectory of the package
figure(1); clf;    % Clear figure window #1 and bring it forward
% Mark the location of the ground by a straight line
xground = [0 max(xNoAir)];  yground = [0 0];
% Plot the computed trajectory and parabolic, no-air curve
plot(xplot,yplot,'+',xNoAir,yNoAir,'-',xground,yground,'-');
ylim([-10 550]);

legend('Quadratic drag','Theory (No air)  '); %SP
xlabel('Range (m)');  ylabel('Height (m)');
title('Projectile motion');


