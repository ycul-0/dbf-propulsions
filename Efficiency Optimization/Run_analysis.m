clear all; close all; clc;
numProp = 1;
numRuns = 30;

% M1s = 53.18; %mph
% M2s = 68.18;
% M3s = 68.18;

% mph
M1s = 48.32;
M2s = 70.69;
M3s = 54.80;

% Mission Drag Estimates (lbsf)
D1 = linspace(1.45,2.45,numRuns)/numProp; % 1.95 lbsf
D2 = linspace(3.25,4.25,numRuns)/numProp; % 3.75 lbsf
D3 = linspace(2.05,3.10,numRuns)/numProp; % 2.55 lbsf

% Mission Cruising Airspeed (MPH)
V1 = [M1s-5, M1s, M1s+5];
V2 = [M2s-5, M2s, M2s+5];
V3 = [M3s-5, M3s, M3s+5];

running1 = zeros(41,21);
running2 = zeros(41,21);
running3 = zeros(41,21);
c1 = running1;
c2 = running2;
c3 = running3;
c=1;
Data = cell(numRuns,1);
for n = 1:numRuns
    Mreq{1} = [D1(n) M1s];
    Mreq{2} = [D2(n) M2s];
    Mreq{3} = [D3(n) M3s];
    [best_temp, temp] = Propulsion_analysis_fun(Mreq);
    
    M1{n} = temp{1};
    M2{n} = temp{2};
    M3{n} = temp{3};
    Motor{3*n-2} = best_temp{3,1};
    Motor{3*n-1} = best_temp{3,2};
    Motor{3*n-0} = best_temp{3,3};
    running1 = temp{1} + running1;
    ind1 = find(temp{1}>0);
    temp{1}(ind1) = 1;
    c1 = c1+temp{1};
    running2 = temp{2} + running2;
    ind2 = find(temp{2}>0);
    temp{2}(ind2) = 1;
    c2 = c2+temp{2};
    running3 = temp{3} + running3;
    ind3 = find(temp{3}>0);
    temp{3}(ind3) = 1;
    c3 = c3+temp{3};
    a = sprintf('finished iteration %0.f\n',n);
    fprintf(a);
end
f1 = running1./c1;
f2 = running2./c2;
f3 = running3./c3;
[p1, m1] = find(f1 == max(max(f1)));
[p2, m2] = find(f2 == max(max(f2)));
[p3, m3] = find(f3 == max(max(f3)));

load('DataImport.mat','Propnames','Motornames')
fprintf(sprintf('M1: %s %s\n',Propnames{p1},Motornames{m1}));
fprintf(sprintf('M2: %s %s\n',Propnames{p2},Motornames{m2}));
fprintf(sprintf('M3: %s %s\n',Propnames{p3},Motornames{m3}));
return
% Plotting
figure
plot(f1)
legend(Motornames)

figure
plot(f2)
legend(Motornames)

figure
plot(f3)
legend(Motornames)