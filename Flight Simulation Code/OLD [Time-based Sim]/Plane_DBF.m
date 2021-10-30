% ------------------------------------------------------------------------%
% DBF Mission Simulation
% 
% Original code by Dr. Mark Anderson, first modified for use by the
% University of California San Diego Design-Build-Fly team for the
% 2016-2017 competition.
% Last edited: 8/31/2017
%
% Uses the following function files:
%   EOM_DBF.m       STDATM.m
% 
% ----------------------------------------------------------------------- %

clear all; close all; clc;

% ----------------------------------------------------------------------- %
% Initial Conditions
%
% The original code runs based on SI units, adjustments have been made
% throughout the code to change US to SI units.
% 
% Variation in thrust due to airspeed is taken into account in this model
% based on static thrust testing using the maximum thrust, RPM at max
% thrust, and propeller geometry.
%
% Coordinate presets have been provided for Silent Flyers and both
% competition airfields. Un-comment the desired location and remember to
% comment out the other two locations.
%
% If the competition calls for a hand launch add the height of the throw to
% the provided altitude. otherwise leave the extra value at zero.
%
% ----------------------------------------------------------------------- %

x = zeros(13,1);             % state vector
n = 2;                       % max g load
thr = 70;                    % percent throttle in turns  

x(1) = 0.0;                  % time [sec]
x(2) = 30*.44704;            % drop airspeed [m/s] = [mph]*conversion
x(3) = 0.0*pi/180.0;         % vertical flight path angle [rad] = [deg]*conversion factor

% San Diego, California Takeoff Preset 32.762914, -117.213830
% x(4) = -170.0*pi/180.0;      % flight path heading [rad] = [deg]*conversion
% x(5) = 32.762914*pi/180.0;   % latitude [rad] = [deg]*conversion
% x(6) = -117.213830*pi/180.0; % longitude [rad] = [deg]*conversion
% x(7) = (10+0)*.3048;       % takeoff altitude [m] = ([ft elevation]+[ft launch])*conversion

% Tucson, Arizona Takeoff Preset
% x(4) = -170.0*pi/180.0;      % flight path heading [rad] = [deg]*conversion
% x(5) = 32.265197*pi/180.0;   % latitude [rad] = [deg]*conversion
% x(6) = -111.273469*pi/180.0; % longitude [rad] = [deg]*conversion
% x(7) = (2388+7)*.3048;       % drop altitude [m] = ([ft elevation]+[ft launch])*conversion

% Wichita, Kansas Takeoff Preset 37.647678, -97.250594
x(4) = -170.0*pi/180.0;      % flight path heading [rad] = [deg]*conversion
x(5) = 37.647678*pi/180.0;   % latitude [rad] = [deg]*conversion
x(6) = -97.250594*pi/180.0;  % longitude [rad] = [deg]*conversion
x(7) = (1300+0)*.3048;       % drop altitude [m] = ([ft elevation]+[ft launch])*conversion
    
x(8) = 29*0.0283495;         % mass [kg] = [oz]*conversion
x(9) = 0.0;                  % heat [J]

x(10) = 0.421*(25.6-x(2));   % thrust [N] = [kg/s]*([m/s pitch]-[m/s vehicle])
x(11) = 0.0;                 % thrust angle [rad]
x(12) = 0.992;               % lift coefficient
x(13) = 0.0;                 % bank angle [rad]
x(14) = 0.05;                % drag coefficient
x(15) = 230*.00065;          % wing area [m^2] = [in^2]*conversion factor

dt = 0.001;                  % time step [sec]
Re = 6378000.0;              % Earth radius [m]

flight_phase = 0;            % mission phase
i = 0;                       % iteration counter
deltaX = 0;                  % distance traveled
LAPS = 0;                    % lap counter
DNF = 0;                     % did-not-finish counter

% ----------------------------------------------------------------------- %
% Simulation Loop
% 
% This section of the code will run the simulation itself until three laps
% have been completed or five minutes have passed.
%
% ----------------------------------------------------------------------- %

while LAPS ~= 3 && DNF ~= 1
    
    % save state vector for plotting
    
    i = i + 1;
    x(10) = 0.421*(25.2461-x(2));
    xs(:,i) = x;
    
    % find other variables of interest
    
    dx = EOM_DBF(x);
    
    Rd = Re + x(7);              % radius [m]  
    at(i) = dx(2)/9.8 + (Re/Rd)*(Re/Rd)*sin(x(3));  % tangential accel [g's]
    an(i) = x(2)*dx(3)/9.8 - ((x(2)*x(2)/(9.8*Rd) - (Re/Rd)*(Re/Rd)))*cos(x(3));  % normal accel [g's]
    ac(i) = x(2)*cos(x(3))*dx(4)/9.8;       % lateral accel [g's]
    ac(i) = ac(i) - x(2)*x(2)*cos(x(3))*cos(x(3))*sin(x(4))*tan(x(5))/(9.8*Rd);
    
    heat_rate(i) = dx(9);       % heating rate [W]
    climb_rate(i) = dx(7);      % climb rate [m/s]
    
    [dens,tempr,sos] = STDATM(x(7));
    
    qbar(i) = 0.5*dens*x(2)*x(2);      % dyn. pressure [N/m/m]
    Mach(i) = x(2)/sos;                % Mach number
    stag_temp(i) = tempr*(1.0 + 0.2*Mach(i)*Mach(i)); % stagnation temp. [K]
    
    % set control variables
    
    switch flight_phase
        
        case 0      % launch + 2.5g pitch up
            if (x(7) < 2488*0.3048) % check to see if already at altitude
                x(12) = 2.5*x(8)*9.8/(qbar(i)*x(15));
                if (x(12) > 0.992)
                    x(12) = 0.992;
                end
                if (x(3) > 5.0*pi/180.0)
                    flight_phase = 1;
                end
            else
                flight_phase = 1;
            end
        
         case 1      % hold flight path angle until altitude is reached
            x(12) = x(8)*9.8*cos(5.0*pi/180)/(qbar(i)*x(15));
            if (x(12) > 0.506)
                x(12) = 0.506;
            end
        
            if (x(7) > 2488*0.3048) % level out at desired altitude
                x(12) = -0.5*x(8)*9.8/(qbar(i)*x(15));
                if (x(3) < 0)
                    x(3) = 0;
                flight_phase = 2;
                end
            end
        
        case 2      % level flight until turn
            x(12) = x(8)*9.8/(qbar(i)*x(15));
            if (deltaX > 500)
                x(13) = acos(1/n);
                flight_phase = 3;
            end
        
        case 3      % n-g 180 turn
            x(10) = x(10)*thr/100;
            x(12) = n*x(8)*9.8/(qbar(i)*x(15));    % n-g turn
            if (x(12) > 0.506)
                x(12) = 0.506;
            end
        
            if (x(4) > (-170.0 + 180.0)*pi/180.0)
                x(4) = (-170.0 + 180.0)*pi/180.0;
                x(13) = -acos(1/n);               % roll to 360
                flight_phase = 4;
            end
        
        case 4      % n-g 360 turn
            x(10) = x(10)*thr/100;
            x(12) = n*x(8)*9.8/(qbar(i)*x(15));    % n-g turn
            if (x(12) > 0.506)
                x(12) = 0.506;
            end
        
            if (x(4) <(-350*pi/180))
                x(4) =-350*pi/180;
                x(13) = 0;                        % Return to level flight
                x(10) = 0.421*(25.2461-x(2));
                deltaX = 0;
                flight_phase = 5;
            end
        
        case 5      % flight to next turn
            x(12) = x(8)*9.8/(qbar(i)*x(15));      % set lift equal to drag
            if deltaX > 1000
                x(13) = acos(1/n);
                x(10) = x(10)*thr/100;
                flight_phase = 6;
            end
        
        case 6      % n-g 180 turn
            x(10) = x(10)*thr/100;
            x(12) = n*x(8)*9.8/(qbar(i)*x(15));    % n-g turn
            if (x(12) > 0.506)
                x(12) = 0.506;
            end
            
            if (x(4) > (-350+180)*pi/180)
                x(4) = (-350+180)*pi/180;
                x(13) = 0;                        % Return to level flight
                x(10) = 0.421*(25.2461-x(2));
                deltaX = 0;
                flight_phase = 7;
            end
            
        case 7      % flight to finish line
            x(12) = x(8)*9.8/(qbar(i)*x(15));
            if deltaX > 500
                deltaX = 0;                       % reset distance traveled
                LAPS = LAPS + 1;                  % increase lap counter
                flight_phase = 0;                 % restart flight phase
            end
    end
  
    % Next step for deltaX and applies the Runge-Kutta formula to the state
    % vector.
    
    deltaX = deltaX + x(2)*(1/.3048)*cos(x(3))*dt;
    
    r1 = dt*EOM_DBF(x);
    r2 = dt*EOM_DBF(x + 0.5*r1);
    r3 = dt*EOM_DBF(x + 0.5*r2);
    r4 = dt*EOM_DBF(x + r3);
    x = x + (r1 + 2.0*r2 + 2.0*r3 + r4)/6.0;
    
    % Sets a failsafe to stop the code if the simulation runs for longer
    % than a five minute flight.
    
    if x(1) > 300
        DNF = 1;
    end
end

% ----------------------------------------------------------------------- %
% Plotting
% 
% This section will plot various telemetry that is desired. It is set to
% subplots by default for easy viewing when debugging.
% 
% ----------------------------------------------------------------------- %

subplot(2,3,1)
plot(xs(1,:),xs(2,:));
title('Airspeed vs Time');
xlabel('Time (s)'), ylabel('Velocity (m/s)');

subplot(2,3,2)
plot(xs(1,:),xs(7,:)*(1/.3048)-2388);
title('Altitude vs Time'); % axis([0 100 0 200]);
xlabel('Time(s)'), ylabel('Altitude (ft)');

subplot(2,3,3)
plot(xs(1,:),xs(3,:)*180/pi);
title('Flight Path Angle vs Time');
xlabel('Time (s)'), ylabel('Flight Path Angle (deg)');

subplot(2,3,4)
plot(xs(1,:),xs(10,:));
title('Thrust vs Time');
xlabel('Time (s)'); ylabel('Thrust (N)');

subplot(2,3,5)
plot(xs(1,:),xs(4,:)*180/pi);
title('Flight Path Heading vs Time');
xlabel('Time (s)'); ylabel('Flight Path Heading (deg)');

subplot(2,3,6)
plot(xs(1,:),xs(13,:)*180/pi);
title('Bank Angle vs Time');
xlabel('Time (s)'),ylabel('Bank Angle (deg)');

% ----------------------------------------------------------------------- %
% End of code
% ----------------------------------------------------------------------- %