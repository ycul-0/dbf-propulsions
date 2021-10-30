function [best_combo,out_array] = Propulsion_analysis_fun(Mreq)
    %% Import Propeller & Motor Datasheets
    load('DataImport.mat')

    %% Propeller Analysis & Mission Performance Criteria

    % Mission-Propeller loop for calculating
    max_eff=ones(3,1);
    Vmission = cell(length(PropFiles),1);
    RPMmission = cell(length(PropFiles),1);
    Pemission = cell(length(PropFiles),1);
    Qpropmission = cell(length(PropFiles),1);
    out_array = cell(3,1);
    out_array{1} = zeros(length(Propnames),length(Motornames));
    out_array{2} = zeros(length(Propnames),length(Motornames));
    out_array{3} = zeros(length(Propnames),length(Motornames));
    
    for MISSION=1:3
        for FILE=1:length(PropFiles)
            Vmission{FILE} = zeros(1,3);
            for n=maxRPM(FILE):-1:1 % Find the minimum RPM where mission criteria is met
                req1 = T{FILE}{n} > Mreq{MISSION}(1);
                req2 = V{FILE}{n} > Mreq{MISSION}(2);
                if nnz(req1) && nnz(req2) % There exists a V where it provides enough thrust
                    for x=30:-1:1
                        if req1(x) && req2(x)
                            Vmission{FILE}(MISSION) = V{FILE}{n}(x);
                            RPMmission{FILE}(MISSION) = n*1000;
                            Pemission{FILE}(MISSION) = Pe{FILE}{n}(x);
                            Qpropmission{FILE}(MISSION) = Qprop{FILE}{n}(x);
                        end
                    end
                elseif (Vmission{FILE}(MISSION)) == 0 % If criteria not met, return zeros
                    Vmission{FILE}(MISSION) = 0;
                    RPMmission{FILE}(MISSION) = 0;
                    Pemission{FILE}(MISSION) = 0;
                    Qpropmission{FILE}(MISSION) = 0;
                end
            end
            if Pemission{FILE}(MISSION) > Pemission{max_eff(MISSION)}(MISSION)
                max_eff(MISSION)=FILE;
            end
        end
    end

    % Print Best Propeller
    for M=1:3
        bestprops{M,1} = Propnames{max_eff(M)};
    end

    %% Motor Analysis

    Voltage = 22.2;
    % Motor Analysis loop
    for Motor = 1:length(Motornames)
        Kt(Motor) = 1355/Kv(Motor);
        RPMmax(Motor) = Kv(Motor) * (Voltage - Rm(Motor)*I0(Motor));
        Imax(Motor) = (Voltage) / Rm(Motor);

        eta_motor{Motor} = @(RPM,A) (Kt(Motor)*(A-I0(Motor))*0.007061552*RPM*2*pi/60) / (Voltage*A);
    end

    %% Mission Combination Analysis
    for MISSION = 1:3    
        for Prop = 1:length(PropFiles)
            if RPMmission{Prop}(MISSION) == 0 % Makes sure Prop meets mission criteria
                continue
            end
            RPMcrit = RPMmission{Prop}(MISSION);
            Qcrit = Qpropmission{FILE}(MISSION)*0.112985; % in-lbf to Nm conversion

            for Motor = 1:length(Motornames)
                eta_Prop{MISSION}(Prop,Motor) = Pemission{Prop}(MISSION);
                Aindex = Qcrit/(Kt(Motor)*0.007061552) + I0(Motor);
                Amax = (Voltage - RPMcrit/Kv(Motor)) / Rm(Motor);
                RPMmax = Kv(Motor)*(Voltage-Rm(Motor)*I0(Motor));
                if (RPMcrit < RPMmax) && (eta_motor{Motor}(RPMcrit,Aindex)<1)
                    eta_Motor{MISSION}(Prop,Motor) = eta_motor{Motor}(RPMcrit,Aindex);
                    amp_draw{MISSION}(Prop,Motor) = Aindex;
                else
                    eta_Motor{MISSION}(Prop,Motor) = 0;
                    amp_draw{MISSION}(Prop,Motor) = 0;
                end
                eta_net{MISSION}(Prop,Motor) = eta_Prop{MISSION}(Prop,Motor)*eta_Motor{MISSION}(Prop,Motor);
                
            end
        end
        out_array{MISSION} = eta_net{MISSION};
    end

    for MISSION=1:3
        maxeff(MISSION) = max(max(eta_net{MISSION}));
        [bestprop, bestmotor] = find(eta_net{MISSION} == maxeff(MISSION));
        best_combo{1,MISSION} = sprintf('Mission %0.f Optimized',MISSION);
        best_combo{2,MISSION} = Propnames{bestprop};
        best_combo{3,MISSION} = Motornames{bestmotor};
        best_combo{4,MISSION} = sprintf('%.4f eta_net',maxeff(MISSION));
        best_combo{5,MISSION} = sprintf('%.4f Amps',amp_draw{MISSION}(bestprop,bestmotor));
        best_combo{6,MISSION} = sprintf('%.4f eta_P',eta_Prop{MISSION}(bestprop,bestmotor));
        best_combo{7,MISSION} = sprintf('%.4f eta_M',eta_Motor{MISSION}(bestprop,bestmotor));
    end
end

