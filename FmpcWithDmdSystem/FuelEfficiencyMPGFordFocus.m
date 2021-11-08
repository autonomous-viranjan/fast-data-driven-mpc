function MPG = FuelEfficiencyMPGFordFocus(pos, speed, control, Ts)

    tf = length(speed);

    c00 = 0.267;
    c10 = 0.014;
    c01 = 0.184;
    c20 = 0.001;
    c11 = 0.048;
    c02 = 0.224;
    
    Fd = 0.1;

    % Parameters
    M = 1200;
    Av = 2.5;
    Cd = 0.32;
    rho = 1.184;
    mu = 0.013;
    g = 9.81;
    
    u = [control; 0];
    a = (-1/(2*M))*Cd*rho*Av*speed.^2 - mu*g + u;

    fuel = c00*ones(length(speed),1) + c10*speed + c01*a + c20*speed.^2 + c11*(speed.*a) + c02*(a.^2);
    
    Fuel = zeros(tf,1);
    for i = 1:tf
        if speed(i) == 0 || u(i) < 0
            xi = 1;
        else
            xi = 0;
        end
    
        Fuel(i) = (1-xi)*(fuel(i)) + xi*Fd;
    end

    mL = 0;
    for i = 1:tf
        mL = mL + Fuel(i)*Ts;
    end

    m_mL = (pos(tf) - pos(1))/mL;
    MPG = m_mL*2.35214583;

end
