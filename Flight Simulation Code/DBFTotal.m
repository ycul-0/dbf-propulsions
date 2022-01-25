%% DBF Flight Simulation Code

%---------------------------------------------------------------%
%   UCSD DBF Propulsions Subteam 
%   DBF Flight Simulation Code
%   
%   This script is used to simulate the conditions that the plane may
%   experience during the flight given a certain inputs: 
%       Dataimport.mat
%       mass = mass of plane [kg]
%       Cl = lift coefficient 
%       k = power draw [A]
%       cd0 = zero lift drag coefficient
%       n = load factor, Lift/Weight ratio of plane. Measures amount of
%       physical stress it is under [g's]
%       MotorAngles = angle of the motors
%       WingSurfaceArea = surface area of wing [m^2]
%       dist = distance of a single straight part of the course [ft]
%
%   This script calls upon the scripts DBFSteadyFlight.m and DBFBankTurn.m
%   to calculate and output:
%       t_total = total flight time [seconds]
%       v_straight = velocity of plane during steady level flight [mph]
%       drag_straight = drag of plane during steady level flight [mph]
%       v_banked = velocity of plane while making a banked turn [lbf]
%       drag_banked = drag of plane while making a banked turn [lbf]
%
%   This file requires DataImport.mat to be loaded in the workspace. Please
%   run Generate_Mat_File.m first before running this script. 
%   
%   First Created by 
%   Ryan Dunn 
%   Propulsions Lead 2019-2021
%   
%   Last Editted by
%   Kevin Vo
%   11/15/2022
%   
%---------------------------------------------------------------%
%   
%   ROOM FOR IMPROVEMENT: 
%   
%   Add a script that automatically calculates the power draw. 
%   
%---------------------------------------------------------------%

%% Inputs 

mass = 5.8967; %kilograms
Cl = 0.28;
k = 11; % Amps
Cd0 = 0.012;
MotorAngle = 0; %degrees
WingSurfaceArea = 0.32980579; %m^2
n = 2.5; %g's
dist = 1000; %feet


%% Calculations

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

%% Outputs in useful units
t_total
v_straight = v_straight * 2.23694 %m/s to mph
v_banked = v_turn * 2.23694 %m/s to mph
drag_straight = drag_straight * 0.224809 %N to lbf
drag_banked = drag_banked * 0.224809 %N to lbf

