function MPG = FuelEfficiencyMPG(pos, speed, control, Ts)

    tf = length(speed);

    b0 = 0.1569;
    b1 = 2.450e-02;
    b2 = -7.415e-04;
    b3 = 5.975e-05;
    c0 = 0.07224;
    c1 = 9.681e-02;
    c2 = 1.075e-03;
    Fd = 0.1;

    % Parameters
    M = 1200;
    Av = 2.5;
    Cd = 0.32;
    rho = 1.184;
    mu = 0.013;
    g = 9.81;

    fcruise = b0 + b1*speed + b2*speed.^2 + b3*speed.^3;
    a = (-1/(2*M))*Cd*rho*Av*speed.^2 - mu*g + [control; 0];
    faccel  = a.*(c0 + c1*speed + c2*speed.^2);

    control_mod = [control; 0];
    Fuel = zeros(tf,1);
    for i = 1:tf
        if speed(i) == 0 || control_mod(i) < 0
            xi = 1;
        else
            xi = 0;
        end
    
        Fuel(i) = (1-xi)*(fcruise(i) + faccel(i)) + xi*Fd;
    end

    mL = 0;
    for i = 1:tf
        mL = mL + Fuel(i)*Ts;
    end

    m_mL = (pos(tf) - pos(1))/mL;
    MPG = m_mL*2.35214583;

end
