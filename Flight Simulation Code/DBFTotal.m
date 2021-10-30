%Inputs
mass = 4.082; %kilograms
Cl = 0.354;
k = 0.067;
Cd0 = 0.032;
MotorAngle = 0; %degrees
WingSurfaceArea = .41032176; %m^3
n = 2.5; %g's
dist = 1000; %feet

%Straight
d_straight = dist*0.3048; %feet to meters
v_straight = DBFSteadyFlight(mass,Cl,k,Cd0,MotorAngle,WingSurfaceArea);
t_straight = d_straight/v_straight;

%180 Turn
d_180 = pi; %radians
[v_turn,TurnRate,TurnRadius] = DBFBankTurn(mass,Cl,n,WingSurfaceArea);
t_180 = d_180/TurnRate;

%360 Turn
d_360 = 2*pi; %radians
t_360 = d_360/TurnRate;

%Total time
t_total = 3*(2*t_180 + 2*t_straight + t_360); %seconds


%Drag calculation
drag_straight = 0.5 * 1.225 * (Cd0 + k*Cl^2) *WingSurfaceArea * v_straight;
drag_banked = 0.5 * 1.225 * (Cd0 + k*Cl^2) *WingSurfaceArea * v_turn;

%Outputs in useful units
t_total
v_straight = v_straight * 2.23694 %m/s to mph
v_banked = v_turn * 2.23694 %m/s to mph
drag_straight = drag_straight * 0.224809 %N to lbf
drag_banked = drag_banked * 0.224809 %N to lbf

