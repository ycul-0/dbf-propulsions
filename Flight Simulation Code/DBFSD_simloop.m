clear; clc;
% add per-lap


% move n's to ic script
dt      = 0.01; %S n's are safe loading factors - see V-n diagram
np      = 4; %S n positive
nt      = 3.65; %S n turning 
nn      = -1.75; %S n negative
thr     = 1; %s throttle percentage
invr    = 0;


[x, xc, xt] = DBFSD_IC;

flight  = 'TO'      ;
lap     = 0;
dX      = 0               ; % Distance from reference       [ft]          .
i       = 0               ; % Step counter                                .
stop    = 0               ; % Stop flag                                   .
%% 

while stop == 0
i           = i + 1;
[dx, y]  = DBFSD_EOM(x, thr, invr);
rhoA    = y(1); %s changes each iteration based on Z (elevation)
Th      = y(4);


%% Saving and defining the state vector

tme(i)  = x(1)            ; % Time since simulation start   [s]           .

W  (i)  = x(2)            ; % Aircraft weight               [lbf]         .
S  (i)  = x(3)            ; % Wing reference area           [ft^2]        .
Cl (i)  = x(4)            ; % Lift coefficient              [1]           .
Cd (i)  = x(5)            ; % Drag coefficient              [1]           .

pit(i)  = x(6)            ; % Propeller pitch               [in]          .
rpm(i)  = x(7)            ; % Motor speed                   [RPM]         .
prd(i)  = x(8)            ; % Propeller diameter            [in]          .
alp(i)  = x(9)            ; % Motor mount angle             [deg]         .

V  (i)  = x(10)           ; % Aircraft velocity             [ft/s]        .
tta(i)  = x(11)           ; % Bank angle                    [deg]         .

lon(i)  = x(12)           ; % Aircraft longitude            [deg]         .
lat(i)  = x(13)           ; % Aircraft latitude             [deg]         .
Z  (i)  = x(14)           ; % Aircraft altitude A.M.S.L.    [ft]          .
gam(i)  = x(15)           ; % Vertical flight angle         [deg]         .
hdg(i)  = x(16)           ; % Flight path heading           [deg]         .
    
D(i)    = y(3)            ;    


 xm(:,i) = x; %s keeps an array of x values at every .1 seconds
 ym(:,i) = y;

qbar(i) = 0.5 .* rhoA .* x(10).^(2);

switch flight
case 'TO'

if  gam(i) < 15.0 %S if vertical flight angle is less than 15 degrees
    eps   =  gam(i) + alp(i);       %S epsilon is set to vertical flight angle + mount angle
    Cli   =  np .* (W(i) .* cosd(gam(i)) - Th .* sind(alp(i))) ./ (qbar(i) .* S(i))                             ; %s getting next Cl value from current data

%  Based on aerodynamic data gathered for previous DBF aircraft,
%  the takeoff lift coefficient tends to be roughly 4.5-times that during 
%  its cruise. The lift coefficient should not exceed such a value.

% change to user input
    if  Cli  >  Cl(1)
        Cli  =  Cl(1)                                                     ;                                       
    end
        
else
    flight  = 'SC';
end
    
    case 'SC'
        eps   =  gam(i) + alp(i)                                          ;
        Cli   = (W(i) .* cosd(gam(i)) - Th .* sind(alp(i))) ./ (qbar(i) .* S(i))           ;
    if  Z(i)  >  Z(1) + 50   %S one of those for loops to control rising vertical flight angle? Z = altitude of field
        flight    = 'LO-1'                                                ;
        thr   =  0.7       ; invr  =  1         ;
    end
    
%  Flight conditions as the aircraft levels off (LO).  It completes this 
%  phase when the vertical flight angle returns to about 0.0 degrees.
    case 'LO-1'
    if gam(i) > 0.0
        We    =  W(i) .* cosd(gam(i))                                     ;
        The   =  Th   .* sind(alp(i))                                     ;
        Cfe   =  W(i) ./ 32.17   .*  deg2rad(dx(15)) .* V(i)              ;
        Cli   = (We - The - Cfe) ./ (qbar(i) .* S(i))                     ; 
    else 
     flight   = 'SL-1'                                                    ;
        x(7)  = xc(10)    ./ pit(i) .* 719 .* 1.1                        ; 
        thr   = 0.85      ; invr  = 0         ;
        
    end
    
%  Flight conditions for the aircraft's first steady-level segment (SL-1).
%  The airplane completes this phase once it travels more than 500 feet 
%  from the start, per the fly-off rules.
    case 'SL-1'
    if dX     < 500.0 %s if less than 500 feet from start point
        We    =  W(i) .* cosd(gam(i))                                     ;
        The   =  Th   .* sind(alp(i))                                     ;
        Cli   = (We - The) ./ (qbar(i) .* S(i))                           ; 
    else
      flight  = 'HT-1'                                                    ;
        x(7)  =  xt(10)    ./ pit(i) .* 719 .* 1.1                        ;                                                 ;
        thr   = 0.7       ; invr  = 0         ;
    end
    
%  Flight conditions for the aircraft's first half-turn segment (HT-1).
%  The plane completes this phase once its heading is about 180.0 degrees
%  from its starting heading.
    case 'HT-1'
    if hdg(i) >  hdg(1) - 180

        x(11) = -acosd(1 ./ nt)                                           ;
        
        We    =  W(i) .* cosd(gam(i))                                     ;
        The   =  Th   .* sind(alp(i))                                     ;        
        Cli   = (We - The) ./ cosd(tta(i)) ./ (qbar(i) .* S(i))           ;
    else
       flight = 'SL-2'                                                    ;
        dX    = 0         ;
        x(7)  = xc(10)    ./ pit(i) .* 719 .* 1.1                        ;  
        x(11) = 0         ;
    if gam(i) > 0.05
        thr   = 0.85      ; invr  = 1         ;
    else
        thr   = 0.85      ; invr  = 0         ;
    end
    end
    
    
    case 'SL-2'
    if dX     < 500.0
        We    =  W(i) .* cosd(gam(i))                                     ;
        The   =  Th   .* sind(alp(i))                                     ;
        Cli   = (We - The) ./ (qbar(i) .* S(i))                           ; 
    else
    flight    = 'FT'                                                      ;
        x(7)  =  xt(10)    ./ pit(i) .* 719 .* 1.1                        ;
        x(11) =  acosd(1 ./ nt)                                           ;
        thr   = 0.7       ; invr  = 0         ;
    end
     %   
    case 'FT'
    if hdg(i) <  hdg(1) + 180
        
        We    =  W(i) .* cosd(gam(i))                                     ;
        The   =  Th   .* sind(alp(i))                                     ;        
        Cli   = (We - The) ./ cosd(tta(i)) ./ (qbar(i) .* S(i))           ;
    else
       flight = 'SL-3'    ;
        dX    = 0         ;
        x(7)  = 8000      ; 
        x(11) = 0         ;
    if gam(i) > 0.05
        thr   = 0.85      ; invr  = 1         ;
    else
        thr   = 0.85      ; invr  = 0         ;
    end
    end
    
    %
    case 'SL-3'
    if dX     < 500.0
        We    =  W(i) .* cosd(gam(i))                                     ;
        The   =  Th   .* sind(alp(i))                                     ;
        Cli   = (We - The) ./ (qbar(i) .* S(i))                           ;
    else
       flight = 'HT-2'                                                    ;
        x(7)  =  xt(10)    ./ pit(i) .* 719 .* 1.1                        ;
        x(11) = -acosd(1   ./ nt)                                         ;
        thr   = 0.7       ; invr  = 0         ; 

    end

    case 'HT-2'
    if hdg(i) >  hdg(1) 
        We    =  W(i) .* cosd(gam(i))                                     ;
        The   =  Th   .* sind(alp(i))                                     ;        
        Cli   = (We - The) ./ cosd(tta(i)) ./ (qbar(i) .* S(i))           ;
    else
       flight = 'SL-4'                                                    ;
        dX    = 0         ;
        x(7)  = 8000      ; 
        x(11) = 0         ;
    if gam(i) > 0.05
        thr   = 0.85      ; invr  = 1         ;
    else
        thr   = 0.85      ; invr  = 0         ;
    end
    end
    
    case 'SL-4'
    if dX     < 500.0
        We    =  W(i) .* cosd(gam(i))                                     ;
        The   =  Th   .* sind(alp(i))                                     ;
        Cli   = (We - The) ./ (qbar(i) .* S(i))                           ;
    else
        lap   = lap + 1                                                   ;
       flight = 'SL-1'                                                    ;
        dX    = 0                                                         ;
    end
end
    
       
       x(4)  =  Cli;





k1      = dt .* DBFSD_EOM(x, thr, invr); %Runge Kutta method
k2      = dt .* DBFSD_EOM(x + 0.5 .* k1, thr, invr); %s the equations of motion are the ODE's
k3      = dt .* DBFSD_EOM(x + 0.5 .* k2, thr, invr);
k4      = dt .* DBFSD_EOM(x + k3, thr, invr);

[d, y1] = DBFSD_EOM(x, thr, invr);
[d, y2] = DBFSD_EOM(x + 0.5 .* k1, thr, invr);
[d, y3] = DBFSD_EOM(x + 0.5 .* k2, thr, invr);
[d, y4] = DBFSD_EOM(x + k3, thr, invr);

x       = x  + (k1 + 2 .* k2 + 2 .* k3 + k4) ./ 6; %s value of x is updated via Runge-Kutta 
dy      = dt .* (y1 + 2 .* y2 + 2 .* y3 + y4) ./ 6;  
dX      = dX + sqrt(dy(5).^2 + dy(6).^2); %s update distance of aircraft from starting point


if   lap == 3
    stop    = 1;
end
end

%%
figure(1)
hold on
% plot(xm(1,:),smooth(V,150),'.')
plot(xm(1,:),Cl,'-')

%  plot3(lon, lat, Z, '.')
figure(2)
geoplot(xm(13,:),xm(12,:), '.');
set(gca,'Basemap','satellite','FontName','Arial','FontSize',13,'TickLabelFormat','dd')