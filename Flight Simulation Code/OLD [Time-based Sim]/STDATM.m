%
% standard atmosphere model (SI units)
%
function [density,temperature,sos] = STDATM(altitude) 
    R_air = 287;            % gas constant [J/kg/K]
    gamma_air = 1.4;        % ratio of specific heats
    g0 = 9.8;               % gravity constant [m/s]
    
    layer = -1.0;           % gradient layer
    gradient = -0.0065;
    altitude_base = 0.0;
    temperature_base = 288.16;
    density_base = 1.2250;
    
    if (altitude > 11000.0)
        layer = 1.0;       % isothermal layer
        altitude_base = 11000.0;
        temperature_base = 216.66;
        density_base = 0.3648;
    end
    
    if (altitude > 25000.0)
        layer = -1.0;      % gradient layer
        gradient = 0.003;
        altitude_base = 25000.0;
        temperature_base = 216.66;
        density_base = 0.04064;
    end
    
    if (altitude > 47000.0)
        layer = 1.0;       % isothermal layer
        altitude_base = 47000.0;
        temperature_base = 282.66;
        density_base = 0.001476;
    end
    
    if (altitude > 53000.0)
        layer = -1.0;      % gradient layer
        gradient = -0.0045;
        altitude_base = 53000.0;
        temperature_base = 282.66;
        density_base = 0.0007579;
    end
    
    if (altitude > 79000.0)
        layer = 1.0;       % isothermal layer
        altitude_base = 79000.0;
        temperature_base = 165.66;
        density_base = 0.0000224;
    end
    
    if (altitude > 90000.0)
        layer = -1.0;      % gradient layer
        gradient = 0.004;
        altitude_base = 90000.0;
        temperature_base = 165.66;
        density_base = 0.00000232;
    end
    
    if (layer < 0.0)
        temperature = temperature_base + gradient*(altitude - altitude_base);
        pow = -1.0*(g0/gradient/R_air + 1.0);
        density = density_base*(temperature/temperature_base)^pow;
    else
        temperature = temperature_base;
        pow = -1.0*g0*(altitude - altitude_base)/R_air/temperature;
        density = density_base*exp(pow);
    end
    sos = sqrt(gamma_air*R_air*temperature);
    
end