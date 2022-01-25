% initial prop speed should be same as turn?
% instead of relying on rpm inputs, rely on cruise/turn speed inputs (prop)
function [x, xc, xt] = DBFSD_IC(site);
%  DBFSD_IC declares the initial state vector for 

%% Defining the take-off state vector
x(1)    = 0.0             ; % Start of simulation           [s]           .

x(2)    = 15.0            ; % Aircraft weight               [lb]          .
x(3)    = 4.401           ; % Wing reference area           [ft^2]        .
x(4)    = 1.255           ; % Lift coefficient              [1]           .
x(5)    = 0.0339          ; % Drag coefficient              [1]           .

x(6)    = 10.0            ; % Propeller pitch               [in]          .
x(8)    = 12.0            ; % Propeller diameter            [in]          .
x(9)    = 0.0             ; % Motor mount angle             [deg]         .

x(10)   = 47              ; % Takeoff velocity              [ft/s]        .
x(11)   = 0.0             ; % Initial bank angle            [deg]         .
x(15)   = 0.0             ; % Vertical flight angle         [deg]         .

site    = "SD"            ; % Site location (see below)                   .

%% Defining the cruise-state vector

xc(10)  = 104.1           ;
%% Defining the turn-state vector

xt(10)  = 117.2           ; % Banked-turn velocity          [ft/s]        .
%% Site presets

if site == "SD" | site == "San Diego"
    x(12)   = -117.128333 ; % Latitude  of start position   [deg]         .
    x(13)   =   32.990796 ; % Longitude of start position   [deg]         .
    x(14)   =  700        ; % Elevation of field            [ft]          .      
    x(16)   =   62.5      ; % Initial flight heading        [deg]         .
end

if site == "AZ" | site == "Tucson"    | site == "Arizona"
    x(12)   = -111.273639 ; % Latitude  of start position   [deg]         .
    x(13)   =   32.265278 ; % Longitude of start position   [deg]         .
    x(14)   = 2190        ; % Elevation of field            [ft]          .
    x(16)   =  104.5      ; % Initial flight heading        [deg]         .
end

if site == "KS" | site == "Wichita"   | site == "Kansas"
    
end

if site == "CS" | site == "Custom"
end
   
%% END OF USER INPUTS

x(7)    = 12000           ; % Initial propeller speed       [RPM]         .
end