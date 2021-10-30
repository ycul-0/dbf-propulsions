function [v,turnRate,turnRadius] = BankTurn(mass,k,Cl,Cd0,n)
%Input: mass(kg), k(idk the dimensions), Cl(dimensionless), Cd0(dimensionless), n(dimensionless)
%outputs: velocity(m/s), turn rate(rad/s), turn radius(meters)

%We are assuming density is 1.225 kg/m^3, standard day @ sea level 
%k is 1/(pi*e*AR)

weight = mass*9.81;
lift = n*weight;
%Because n = L/W

v = (0.5*1.225*Cl*S)/lift;
%^ this equation doesn't yet work because it requires wing area (S). We
%can find wing area once the plane is built.

turnRadius = v^2/(9.81*sqrt(n^2-1));
turnRate = (9.81*sqrt(n^2-1))/v;       
end