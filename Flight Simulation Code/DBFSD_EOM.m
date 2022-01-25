function [dx, y] = DBFSD_EOM(x, thr, invr)
%  DBFSD_EOM computes the change in state vector 


%% Defining the state vector
%
%
%

tme     = x(1)            ; % Time since simulation start   [s]           .

M       = x(2) / 32.17    ; % Aircraft mass                 [slug]        .
S       = x(3)            ; % Wing reference area           [ft^2]        .
Cl      = x(4)            ; % Lift coefficient              [1]           .
Cd      = x(5)            ; % Drag coefficient              [1]           .

pit     = x(6)            ; % Propeller pitch               [in]          .
pps     = x(7) / 719 * pit; % Propeller pitch speed         [ft/s]        .
prd     = x(8) / 12       ; % Propeller diameter            [ft]          .
alp     = x(9) / 180 * pi ; % Motor mount angle             [rad]         .

V       = x(10)           ; % Aircraft velocity             [ft/s]        .
tta     = x(11)/ 180 * pi ; % Bank angle                    [rad]         .

lon     = x(12)/ 180 * pi ; % Aircraft longitude            [rad]         .
lat     = x(13)/ 180 * pi ; % Aircraft latitude             [rad]         .
Z       = x(14)           ; % Aircraft altitude A.M.S.L.    [ft]          .
gam     = x(15)/ 180 * pi ; % Vertical flight angle         [rad]         .
hdg     = x(16)/ 180 * pi ; % Flight path heading           [rad]         .

%% END OF INPUTS, START OF CALCULATIONS
%% Finding atmospheric, aerodynamic, and propulsion properties
%  The following atmospheric model is based on the 1976 U.S. Standard 
%  Atmosphere. Though this model may not completely portray conditions 
%  at flight-- it does not account for variations in temperature and other 
%  such parameters-- it is valid for most DBF applications.

RgA     = 1716.5          ; % Air gas constant              [ft*lb/slug/R]. 
gamA    = 1.4             ; % Specific heat ratio           [1]           .
g       = 32.17           ; % Gravitational constant        [ft/s^2]      .

TA      = 518.67     -  0.00356 .* Z                                      ;
rhoA    = 0.00237   .* (TA ./ 518.67) .^ (g ./ (RgA .* 0.00356) - 1)      ; %density of air
cA      = sqrt(gamA .* RgA .* TA)                                         ;

%  From these atmospheric quantities, we can tabulate the aerodynamic
%  quantities-- such as lift and drag-- that the aircraft experiences.
qbar    = 0.5  .* rhoA .* (V).^2                                          ;
L       = qbar .* S    .* Cl                                              ;
D       = qbar .* S    .* Cd                                              ;

%  The following model estimates the thrust exerted by the propulsion
%  system using a 'straight-line' approximation. It tracks APC's propeller 
%  data relatively well (with some error).
Jmx     = (pit ./ (12  .* prd) + 0.2) / (pit / (12 .* prd))               ;
Vmt     =  pps .* Jmx                                                     ;

J       = V    ./  (x(7) ./ 60) ./ prd                                    ;
Jcu     = (pit ./  (12 .* prd))  - 0.45                                   ;
Vcu     = Jcu  .* prd  .*  x(7) ./ 60                                     ;
Tmx     = 0.11 .* rhoA .* (x(7) ./ 60).^2 .* (prd).^4                     ;

if J <= Jcu
    Th      =  Tmx                                                        ;
else 
    Tem     =  Tmx ./ (Vcu - Vmt)                                         ; 
    Teb     = -Tem .*  Vmt                                                ;
    Th      = (Tem .*  V)  + Teb                                          ;
end

Th      = thr .* Th                                                   ;

%% Equations of motion (output)

Re      =  20925000;
R       =  Re + Z; %s Z is altitude above sea level

dx(1)   =  1.0                                                            ;

dx(2)   =  0.0                                                            ;
dx(3)   =  0.0                                                            ;
dx(4)   =  0.0                                                            ;
dx(5)   =  0.0                                                            ;

dx(6)   =  0.0                                                            ;
dx(7)   =  0.0                                                            ;
dx(8)   =  0.0                                                            ;
dx(9)   =  0.0                                                            ;

dx(10)  = ((Th .* cos(alp)) - (M * 32.17 .* sin(gam)) - D) ./ M       ; %s change in velocity
dx(11)  =  0.0                                                            ;
dx(12)  =  rad2deg(V  .* cos(gam) .* sin(hdg)  ./ (R) ./ cos(lat)) ; %s change in latitude
dx(13)  =  rad2deg(V  .* cos(gam) .* cos(hdg)  ./ (R))             ; %s change in longitude
dx(14)  =  V  .* sin(gam)                                                 ; 
   
dx(15)  = (Th .* sin(alp) + L .* cos(tta) - M .* 32.17 .* cos(gam)) ./ (M .* V); %s change in flight angle
dx(15)  =  rad2deg(dx(15))                                                ;

if invr == 1
dx(15)  = -dx(15);
end

dx(16)  =  (L) .* sin(tta) ./ (M .* V); %s change in flight heading
dx(16)  =  rad2deg(dx(16));


%% Data vector
y(1)    = rhoA; 
y(2)    = L;
y(3)    = D;
y(4)    = Th;

y(5)    = V  .* cos(gam) .* sin(hdg); %s horizontal component of velocity?
y(6)    = V  .* cos(gam) .* cos(hdg); %s vertical component of velocity?

end