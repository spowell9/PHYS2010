function balle2010Mod(y1,speed,airFlag,tau,shape,m1,m2,m3,m4,wind)      %add KC/SP
%% balle2010MOD - Program to compute the trajectory of a package  
%         using the midpoint method.                              
% invoke as: balle2010Mod(500,90,1,0.0001,1,4,8,12,16,0)

% Original by AJG; modified by Katrina Carver and Sarah Powell 20200404           
help balle2010MOD; % print header 

% mass combinations that illustrate comparisons well:    4, 8, 12, 16; 5, 10, 50, 100; 5, 50, 100, 500  %KC     

% y1 = intial height of package(meters)                                  
% speed = intial speed of aircraft (m/s)                                                             
% airFlag specifies whether there is air resistance: enter 1   
% for yes, 0 for no                                            
% tau = timestep, tau(seconds) 
% shape specifies the shape of the package: enter 0 for sphere, 1 for cube %SP
% m1, m2, m3, m4 = mass of each projectile (kg), should not be less than 4kg    %KC
% wind = wind in horizontal direction (m/s) %SP

% KC change log 04212020: edited program to include 4 masses instead of 1
%                  in main loop, and in calculation of range and time of
%                  flight
%                  you can now run the program from the command line
%                  changed individual position/velocity vectors into state
%                  vectors for cleanliness
%                  plot only graphs of quadratic drag, not theoretical
%                  plot graphs of all 4 masses on top of one another


%% * Set initial position and velocity of the package
r = [0, y1];     % Initial vector position
v = [speed+wind, 0];     % Initial velocity %mod SP
% Set initial position and velocity
state1 = [r(1) r(2) v(1) v(2)];         %add KC
state2 = [r(1) r(2) v(1) v(2)];         %add KC
state3 = [r(1) r(2) v(1) v(2)];         %add KC
state4 = [r(1) r(2) v(1) v(2)];         %add KC


%% * Set physical parameters (mass, Cd, etc.)
if shape == 0 %SP
    Cd = 0.47;      % Drag coefficient (dimensionless) %Cd=12v/Rv=24/Re Cd~0.47 for sphere %SP
    area = 7.1e-2;  % Cross-sectional area of projectile (m^2) %rad=15cm %SP
else 
    Cd = 1.05;      % Drag coefficient (dimensionless) %Cd=12v/Rv=24/Re Cd~1.05 for cube %SP
    area = 5.85e-2;  % Cross-sectional area of projectile (m^2) %Gives same volume as sphere with r=15cm %SP
end

grav = 9.81;    % Gravitational acceleration (m/s^2)

if( airFlag == 0 )
  rho = 0;      % No air resistance
else
  rho = 1.2;    % Density of air (kg/m^3)
end
air_const1 = -0.5*Cd*rho*area/m1;  % Air resistance constant
air_const2 = -0.5*Cd*rho*area/m2;  % add KC
air_const3 = -0.5*Cd*rho*area/m3;
air_const4 = -0.5*Cd*rho*area/m4;
%% * Loop until package hits ground or max steps completed
maxstep = 200000;   % Maximum number of steps

for istep=1:maxstep
    %* Record position (computed and theoretical) for plotting
    xplot1(istep) = state1(1);        %mod KC
    yplot1(istep) = state1(2);        %mod KC
    
    t = (istep-1)*tau;     % Current time
    
    xNoAir1(istep) = state1(1) + state1(3)*t;        %mod KC
    yNoAir1(istep) = state1(2) + state1(4) - 0.5*grav*t^2;
    
     %* Calculate the acceleration of the ball 
    accel1 = air_const1*norm(state1(3:4))*state1(3:4);   % Air resistance
    accel1(2) = accel1(2)-grav;      % Gravity
    
    %* Calculate the new position and velocity using midpoint method %SP
    state1(1:2) = state1(1:2) + tau*((state1(3:4) + tau *accel1) + state1(3:4))/2; %mod KC
    state1(3:4) = state1(3:4) + tau*accel1; %mod KC
    
    rx1(istep)= state1(1); ry1(istep)=state1(2); %recording the components of r for each iteration  %mod KC
    
     %* If ball reaches ground (y<0), break out of the loop
  if( state1(2) < 0 )  %mod KC
    xplot1(istep+1) = state1(1);  % Record last values computed     %mod KC
	yplot1(istep+1) = state1(2);                                    %mod KC
    break;                  % Break out of the for loop
  end 
end

%repeat for other 3 masses, same code as above using the variables of each respective mass      %section added KC

for istep=1:maxstep
    xplot2(istep) = state2(1);        
    yplot2(istep) = state2(2);        
    t = (istep-1)*tau;     % Current time
    xNoAir2(istep) = state2(1) + state2(3)*t;        
    yNoAir2(istep) = state2(2) + state2(4) - 0.5*grav*t^2; 
    accel2 = air_const2*norm(state2(3:4))*state2(3:4);   % Air resistance
    accel2(2) = accel2(2)-grav;      % Gravity
    state2(1:2) = state2(1:2) + tau*((state2(3:4) + tau *accel2) + state2(3:4))/2;
    state2(3:4) = state2(3:4) + tau*accel2;
    rx2(istep)= state2(1); ry2(istep)=state2(2);
    
    if( state2(2) < 0 )  
    xplot2(istep+1) = state2(1);  % Record last values computed
	yplot2(istep+1) = state2(2);
    break;                  % Break out of the for loop
  end 
    
end

for istep=1:maxstep
    xplot3(istep) = state3(1);        
    yplot3(istep) = state3(2);        
    t = (istep-1)*tau;     % Current time
    xNoAir3(istep) = state3(1) + state3(3)*t;        
    yNoAir3(istep) = state3(2) + state3(4) - 0.5*grav*t^2;
    accel3 = air_const3*norm(state3(3:4))*state3(3:4);   % Air resistance
    accel3(2) = accel3(2)-grav;      % Gravity
    state3(1:2) = state3(1:2) + tau*((state3(3:4) + tau *accel3) + state3(3:4))/2;
    state3(3:4) = state3(3:4) + tau*accel3;
    rx3(istep)= state3(1); ry3(istep)=state3(2);
    
    if( state3(2) < 0 )  
    xplot3(istep+1) = state3(1);  % Record last values computed
	yplot3(istep+1) = state3(2);
    break;                  % Break out of the for loop
  end 
    
end

for istep=1:maxstep
  xplot4(istep) = state4(1);        
  yplot4(istep) = state4(2);        
  t = (istep-1)*tau;     % Current time
  xNoAir4(istep) = state4(1) + state4(3)*t;
  yNoAir4(istep) = state4(2) + state4(4) - 0.5*grav*t^2;
  accel4 = air_const4*norm(state4(3:4))*state4(3:4);   % Air resistance
  accel4(2) = accel4(2)-grav;      % Gravity
  state4(1:2) = state4(1:2) + tau*((state4(3:4) + tau *accel4) + state4(3:4))/2;
  state4(3:4) = state4(3:4) + tau*accel4;
  rx4(istep)= state4(1); ry4(istep)=state4(2);
  %   r1 = r1 + tau*((v1 + tau*accel1)+v1)/2; %uses average of previous and new velocity to calculate position
%   v1 = v1 + tau*accel1;
  if( state4(2) < 0 )  
    xplot4(istep+1) = state4(1);  % Record last values computed
	yplot4(istep+1) = state4(2);
    break;                  % Break out of the for loop
  end 
end
%% * Correcting maximum range and time 

last3_x1=[rx1(end-2), rx1(end-1), rx1(end)]; % creating a vector of the last 3 x-components of r1 %SP
last3_y1=[ry1(end-2), ry1(end-1), ry1(end)]; % creating a vector of the last 3 y-components of r1 %SP

% repeat for other 3 masses             % section added KC
last3_x2=[rx2(end-2), rx2(end-1), rx2(end)]; % creating a vector of the last 3 x-components of r2
last3_y2=[ry2(end-2), ry2(end-1), ry2(end)]; % creating a vector of the last 3 y-components of r2

last3_x3=[rx3(end-2), rx3(end-1), rx3(end)]; % creating a vector of the last 3 x-components of r3
last3_y3=[ry3(end-2), ry3(end-1), ry3(end)]; % creating a vector of the last 3 y-components of r3

last3_x4=[rx4(end-2), rx4(end-1), rx4(end)]; % creating a vector of the last 3 x-components of r4
last3_y4=[ry4(end-2), ry4(end-1), ry4(end)]; % creating a vector of the last 3 y-components of r4

%Finds the horizontal range by interpolating between the last 3 position points.
corrected_range1=intrpf(0,last3_y1,last3_x1); %SP

% repeat for other 3 masses             % section added KC
corrected_range2=intrpf(0,last3_y2,last3_x2);
corrected_range3=intrpf(0,last3_y3,last3_x3);
corrected_range4=intrpf(0,last3_y4,last3_x4);

corrected_time1=(istep*tau)-((rx1(end)-corrected_range1)/state1(3)); %SP
%calulates the corrected time by subtracting the change in displacement in 
%the x direction divided by the velocity in the x direction in order to 
%account for extra time elapsed.
% repeat for other 3 masses             % section added KC
corrected_time2=(istep*tau)-((rx2(end)-corrected_range2)/state2(3));
corrected_time3=(istep*tau)-((rx3(end)-corrected_range3)/state3(3));
corrected_time4=(istep*tau)-((rx4(end)-corrected_range4)/state4(3));
%% * Print maximum range and time of flight

fprintf('Maximum range of the %gkg mass is %g meters\n',m1,corrected_range1); %prints max range           %mod KC
fprintf('Time of flight of the %gkg mass is %.6f seconds\n\n',m1,corrected_time1); %prints time of flight   %mod KC

% repeat for other 3 masses                                                                             % section added KC
fprintf('Maximum range of the %gkg mass is %g meters\n',m2,corrected_range2); %prints max range
fprintf('Time of flight of the %gkg mass is %.6f seconds\n\n',m2,corrected_time2); %prints time of flight

fprintf('Maximum range of the %gkg mass is %g meters\n',m3,corrected_range2); %prints max range
fprintf('Time of flight of the %gkg mass is %.6f seconds\n\n',m3,corrected_time3); %prints time of flight

fprintf('Maximum range of the %gkg mass is %g meters\n',m4,corrected_range4); %prints max range
fprintf('Time of flight of the %gkg mass is %.6f seconds\n\n',m4,corrected_time4); %prints time of flight

% add legend text for each mass                 % add KC
txt1 = (m1 + "kg mass");
txt2 = (m2 + "kg mass");
txt3 = (m3 + "kg mass");
txt4 = (m4 + "kg mass");

%% * Graph the trajectory of the baseball
figure(1); clf;    % Clear figure window #1 and bring it forward
plot(xplot1,yplot1, '-', xplot2,yplot2, '-', xplot3,yplot3, '-', xplot4,yplot4, '-');  
ylim([-10 550]);
xlabel('Range (m)');  ylabel('Height (m)');
title('Projectile motion');
legend(txt1, txt2, txt3, txt4);


% Mark the location of the ground by a straight line
%xground1 = [0 max(xNoAir4)];  yground = [0 0];
% Plot the computed trajectory and parabolic, no-air curve
% plot(xplot1,yplot1,'+',xNoAir1,yNoAir1,'+');
% ylim([-10 550]);
% %hold on
% figure(2); clf;
% plot(xplot2,yplot2,'.',xNoAir2,yNoAir2,'.');
% ylim([-10 550]);
% 
% figure(3); clf;
% plot(xplot3,yplot3,'-',xNoAir1,yNoAir1,'-');
% ylim([-10 550]);
% 
% figure(4); clf;
% plot(xplot4,yplot4,'x',xNoAir4,yNoAir4,'x');
% ylim([-10 550]);
% %plot(xground1,yground,'-');


%The following prints the corect label on the legend for which method was
%used for the calculation.
%legend('Quadratic drag (mass 1)','Theory (No air) (mass 1)', 'Quadratic drag (mass 2)','Theory (No air) (mass 2)', 'Quadratic drag (mass 3)','Theory (No air) (mass 3)', 'Quadratic drag (mass 4)','Theory (No air) (mass 4)');
