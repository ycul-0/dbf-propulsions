clc;
clear all;
close all;

% kv = input('enter motor kv: ');
% pitch = input('enter propeller pitch (in): ');
% R = input('enter motor resistance: '); %%in centiohms
% V = input('enter battery voltage: ');
% I0 = input('enter no load current: ');
% RPMmax = input('enter max RPM wanted: ');
% density = 1.225*input('enter air density (kg/m^3):');
% propdiameter = input('enter propeller diameter (in): ');
% filename = input('Enter file name: ','s');
filename='APC PROP 10x10';
kv = 890;
pitch = 10;
R = 1.4;
V = 12;
I0 = 2.25;
RPMmax = V*kv;
rho = .92*1.225; %density
propdiameter = 10;
d = propdiameter*0.0254; %prodiameter to meters
RPM = linspace(0,RPMmax);
pitchspeed = RPM*pitch*0.00094697; %%conversion to mph
pitchspeedcomp = round(pitchspeed,0);
I = (V- RPM/kv)/(R/100);
Iforgraph = (V- RPM/kv)/R;
kt = 1355/kv;
Q = (kt*(I-I0))*0.00706; % eqn for torque w/ conversion to Nm
%T = 0.5*rho*A*(Vpitch.^2 - Vo.^2); % thrust eqn taken from nasa using propeller momentum
Pin = V*I;
Pout = (Q.*RPM)*2*pi/60;
etaMotor = Pout./Pin;
propdata= xlsread(filename);
Vpoop = propdata(:,1);
Vpoopcomp = round(Vpoop);
RPM2 = Vpoop/(pitch*0.00094697);
J = propdata(:,2);
Ct = propdata(:,4);
Cp = propdata(:,5);
etaProp = Ct.*J./Cp;
n = length(etaProp);
m = length(pitchspeedcomp);
eta = zeros(1,30);
for i = 1:n
   Vo = Vpoopcomp(i);
   for j = 1:m
      Vp = pitchspeedcomp(j);
      if Vo == Vp
          eta(i) = etaProp(i)*etaMotor(j);
      end
   end
end
figure 
hold on
title('Efficiences at max rpm');
xlabel('RPM');
ylabel('Efficiency');
plot(RPM, etaMotor,'b')
plot(RPM2, etaProp,'g')
plot(RPM2, eta, 'r')
legend('motor','propeller','total','Location','northwest');
hold off
% figure
% title('Pitch speed vs. No Load Current');
% hold on
% plot(pitchspeed, Iforgraph,'r');
% xlabel('pitchspeed (mph)');
% ylabel('Current (amps)');
% hold off
