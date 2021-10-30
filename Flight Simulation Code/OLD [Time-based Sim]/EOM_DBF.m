% ----------------------------------------------------------------------- %
% 3DOF Equations of Motion
%
% Original code by Dr. Mark Anderson, first modified for use by the
% University of California San Diego Design-Build-Fly team for the
% 2016-2017 competition.
% Last edited: 8/31/2017
%
% Used by the following script files:
%   Plane_DBF.m
% 
% ----------------------------------------------------------------------- %

function dxdt = EOM_DBF(x)
    
    g = 9.8;            % sea-level gravity [m/s/s]
    Re = 6378000.0;     % Earth radius [m] 

    time = x(1);        % time [sec]
    
    V = x(2);           % airspeed [m/s]
    gam = x(3);         % vertical flight path angle [rad]
    chi = x(4);         % flight path heading [rad]
    
    lat = x(5);         % latitude [rad]
    lon = x(6);         % longitude [rad]
    alt = x(7);         % altitude [m]
    
    mass = x(8);        % mass [kg]
    heat = x(9);        % heat [J]
    
    Thrust = x(10);     % thrust [N]
    epsilon = x(11);    % thrust angle [rad]
    CL = x(12);         % lift coefficient
    mu = x(13);         % bank angle [rad]
    CD = x(14);         % drag coefficient
    Sref = x(15);       % wing reference area
    
    R = alt + Re;       % radius from Earth center [m]
    
    dxdt = zeros(15,1);
    
    % atmosphere model
    
    [dens,tempr,sos] = STDATM(alt);
    qbar = 0.5*dens*V*V;    % dynamic pressure [N/m2]
    
    % aerodynamics model
    
    M = V/sos;              % mach number
    Cf = CD/10;
    Drag = qbar*Sref*CD;    % drag force
    Lift = qbar*Sref*CL;    % lift force
        
    % equations of motion
    
    dxdt(1) = 1.0;
    
    dxdt(2) = (Thrust*cos(epsilon) - Drag)/mass - g*(Re/R)*(Re/R)*sin(gam);
    dxdt(3) = (Lift + Thrust*sin(epsilon))*cos(mu)/(V*mass) + (V/R - (g/V)*(Re/R)*(Re/R))*cos(gam);
    dxdt(4) = (Lift + Thrust*sin(epsilon))*sin(mu)/(V*mass*cos(gam)) + (V/R)*cos(gam)*sin(chi)*tan(lat);
    
    dxdt(5) = (V/R)*cos(gam)*cos(chi);
    dxdt(6) = (V/R)*cos(gam)*sin(chi)/cos(lat);
    dxdt(7) = V*sin(gam);
    
    dxdt(8) = 0;
    dxdt(9) = 0.5*qbar*V*Sref*Cf;
    
    dxdt(10) = 0.0;
    dxdt(11) = 0.0;
    dxdt(12) = 0.0;
    dxdt(13) = 0.0;
    dxdt(14) = 0.0;
    dxdt(15) = 0.0;
    
end