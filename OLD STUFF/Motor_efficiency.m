clear all
close all
clc

%% Motor names/data initialization

motor_name = {'760 Kv Cobra 2826';
    '810 Kv Turnigy g10';
    '830 Kv Scorpion 3014';
    '890 Kv Scorpion 3026';
    '900 Kv Scorpion 2215';
    '1110 Kv Scorpion 3020'};

% motor_data matrix holds the data for each given motor, w columns as such:
% Kv Rm Io 
% Kv: Motor Rating (RPM/V)
% Rm: Internal Motor Resistance (Ohms)
% Io: No Load Current (Amps)
motor_data = [
    760,.045,1.1;
    810,.035,1.2;
    830,.042,1.06;
    890,.014,1.9;
    900,.142,.52;
    1110,.016,2.08
    ];
%% Initialize Variables
steps = 500;
V = 22.2;
Kv_Kt = 1355; % Product of Kv & Kt

Kv = zeros(length(motor_name),1);
Rm = zeros(length(motor_name),1);
I0 = zeros(length(motor_name),1);
Kt = zeros(length(motor_name),1);

RPMmax = zeros(length(motor_name),1); % Maximum possible RPM for each motor
RPM = zeros(length(motor_name),steps); % Vectors of RPM values
I_max_op = zeros(length(motor_name),steps); % Max Operating Current at each RPM

I_RPM = zeros(steps,steps); % Vectors of I values for each RPM

Q = zeros(steps,steps); % Matrix holding torque values across RPM-I grid
Pin = zeros(steps,steps); % Matrix holding Pin values across RPM-I grid
Pout = zeros(steps,steps); % Matrix holding Pout values across RPM-I grid
eta_Motor = zeros(steps,steps); % Matrix holding efficiency values across RPM-I grid

RPM_I_grid = cell(length(motor_name),1); % Cell array holding grid for RPM and I values

Q_grid = cell(length(motor_name),1); % Cell array holding torque at each RPM for range of current values
Pin_grid = cell(length(motor_name),1); % Cell array holding Pin at each RPM for range of current values 
Pout_grid = cell(length(motor_name),1); % Cell array holding Pout at each RPM for range of current values
eta_Motor_grid = cell(length(motor_name),1); % Cell array holding efficiency at each RPM for range of current values


%% Data Analysis for all motors
for n = 1:length(motor_name)
% for n = 1
    % Initialize Variables
    Kv(n) = motor_data(n,1);
    Rm(n) = motor_data(n,2);
    I0(n) = motor_data(n,3);
    Kt(n) = Kv_Kt/Kv(n);
    
    RPMmax(n) = Kv(n) * (V - Rm(n)*I0(n));
    RPM(n,:) = linspace(0,RPMmax(n),steps);
    
    I_max_op(n,:) = (V - RPM(n,:)/Kv(n)) / Rm(n); % Max Operating Current at each RPM
    
%     RPM_plot = ones(steps,steps); % Making a matrix for RPM for plotting surface
    
    for m = 1:steps
        I_RPM(:,m) = linspace(I0(n),I_max_op(n,m),steps); % Distribution of I as a function of RPM (row is I, col. is RPM)
        Q(:,m) = Kt(n) * (I_RPM(:,m) - I0(n)) * 0.00706;
        
        Pin(:,m) = V * I_RPM(:,m);
        Pout(:,m) = Q(:,m) * RPM(n,m) * 2*pi/60;
        
        eta_Motor(:,m) = Pout(:,m)./Pin(:,m);
        
%         RPM_plot(m,:) = RPM(n,:);
        
    end
    
%     Q(n,:) = (Kt(n)*(I(n,:)-I0(n)))*0.00706; % eqn for torque w/ conversion to Nm
%     
%     Pin(n,:) = V*I(n,:);
%     Pout(n,:) = (Q(n,:).*RPM(n,:))*2*pi/60;
%     
%     eta_Motor(n,:) = Pout(n,:)./Pin(n,:);
    
    RPM_I_grid{n} = I_RPM; % We keep this in a cell array so we can find our desired operating condition
    
    Q_grid{n} = Q;
    Pin_grid{n} = Pin;
    Pout_grid{n} = Pout;
    eta_Motor_grid{n} = eta_Motor;
    
end

%% Plotting for singular motors

for n = 4
    % Creating Matrix for operating RPM based on the motor index
    RPM_plot = ones(steps,steps); % Making a matrix for RPM for plotting surface
    for m = 1:steps
        RPM_plot(m,:) = RPM(n,:);
    end
    
    figure(1)
    hold on
    plot(RPM(n,:),I_max_op(n,:),'LineWidth',1.5)
    grid on
    xlabel('RPM')
    ylabel('Current (A)')
    legend(motor_name{n},'Location','NorthEast')
%     axis([0 max(RPMmax) 0 1.05*max(I_max_op(n,:),[],'all')])
    
    figure(2)
    hold on
    surf(RPM_plot,RPM_I_grid{n},Q_grid{n},'LineStyle','none')
    grid on
    xlabel('RPM')
    ylabel('Current (A)')
    zlabel('Torque (Nm)')
    
    figure(3)
    hold on
    surf(RPM_plot,RPM_I_grid{n},eta_Motor_grid{n},'EdgeColor','none')
    grid on
    xlabel('RPM')
    ylabel('Current (A)')
    zlabel('Motor Efficiency')
end

%% Old Work/Plots
% Kv = 890;
% Rm = 14e-3; % Internal Motor Resistance (Ohms)
% V = 22.2;
% I0 = 2.25; % No-Load Current (Amps)
% RPMmax = Kv*(V - Rm*I0);
% 
% RPM = linspace(0,RPMmax,1000);

% RPM = Kv * (V-I*Rm);
% I = (V - RPM/Kv)/(Rm);
% Kt = 1355/Kv;
% Q = (Kt*(I-I0))*0.00706; % eqn for torque w/ conversion to Nm
% 
% Pin = V*I;
% Pout = (Q.*RPM)*2*pi/60;
% 
% eta_Motor = Pout./Pin;

%     figure(2)
%     hold on
% %     contour(I_RPM)
%     contour(RPM(n,:),I_max_op(n,:),I_RPM,100)
    
%     figure(2)
%     hold on
%     plot(RPM(n,:),eta_Motor(n,:),'LineWidth',1.5)
%     grid on
%     xlabel('RPM')
%     ylabel('Motor Efficiency')
%     legend(motor_name{n},'Location','NorthWest')
%     axis([0 max(RPMmax) 0 1])
%     
%     figure(3)
%     hold on
%     plot(RPM(n,:),Q(n,:),'LineWidth',1.5)
%     grid on
%     xlabel('RPM')
%     ylabel('Torque (Nm)')
%     legend(motor_name{n},'Location','NorthEast')
%     axis([0 max(RPMmax) 0 1.1*max(Q,[],'all')])

%% Next Steps:
% The above data gives the maximum operating condition at each combination
% of RPM and current; for the torque and efficiency, we can use a 3D plot
% or countour plot to visualize how these parameters vary over a set of
% current and RPM values. 


