% Adapted from MFeval

function [Fx, Fy] = MF61(Fz, kappa, alpha, gamma, Vcx, pres, tirParams)   
    %% Set Constraints To Data
    useLimitsCheck = 1;
    prt = 0;
    if useLimitsCheck
        % Maximum/Minimum Vertical Force
        FZMIN = tirParams.FZMIN;
        FZMAX = tirParams.FZMAX;
        
        % Maximum/Minimum Slip Ratio
        KPUMIN = tirParams.KPUMIN;
        KPUMAX = tirParams.KPUMAX;

        % Maximum/Minimum Slip Angle
        ALPMIN = tirParams.ALPMIN;
        ALPMAX = tirParams.ALPMAX;

        % Maximum/Minimum Camber Angle
        CAMMIN = tirParams.CAMMIN;
        CAMMAX = tirParams.CAMMAX;

        % Minimum Speed
        VXLOW = tirParams.VXLOW;

        % Maximum/Minimum Inflation Pressure
        PRESMIN = tirParams.PRESMIN;
        PRESMAX = tirParams.PRESMAX;

        % Flag low speed
        isLowSpeed = (abs(Vcx) <= VXLOW);

        % Create a vector with numbers in [0,1] to apply a reduction factor
        % with smooth transition
        Wvlow = 0.5 .* (1 + cos(pi .* (Vcx(isLowSpeed) ./ VXLOW))); % (page 441 - 9.126)
        reductionSmooth = 1 - Wvlow; 

        % Calculate lateral speed for steady state
        Vsy = tan(alpha) .* Vcx;

        % Sum longitudinal and lateral speeds
        speedSum = abs(Vcx) + abs(Vsy);

        % The slip angle also suffers a reduction when Vx + Vy < VXLOW
        isLowSpeedAlpha = (speedSum < VXLOW);

        % Create a vector with numbers in [0,1] to apply a linear reduction
        % toward 0 for alpha
        reductionLinearAlpha = speedSum(isLowSpeedAlpha) / VXLOW;

        % Apply reductionLinearAlpha to alpha
        alpha(isLowSpeedAlpha) = alpha(isLowSpeedAlpha) .* reductionLinearAlpha;

        % Check limits
        isLowSlip = (alpha < ALPMIN);
        alpha(isLowSlip) = real(ALPMIN);
        if any(isLowSlip) && prt
            warning ('Solver:Limits:Exceeded',['Slip angle below ',...
                'the limit. Values have been saturated.']);
        end

        isHighSlip = (alpha > ALPMAX);
        alpha(isHighSlip) = real(ALPMAX);
        if any(isHighSlip) && prt
            warning ('Solver:Limits:Exceeded',['Slip angle above ',...
                'the limit. Values have been saturated.']);
        end

        isLowCamber = gamma < CAMMIN;
        gamma(isLowCamber) = real(CAMMIN);
        if any(isLowCamber) && prt
            warning ('Solver:Limits:Exceeded',['Inclination angle below ',...
                'the limit. Values have been saturated.']);
        end

        isHighCamber = gamma > CAMMAX;
        gamma(isHighCamber) = real(CAMMAX);
        if any(isHighCamber) && prt
            warning ('Solver:Limits:Exceeded',['Inclination angle below ',...
                'the limit. Values have been saturated.']);
        end
        
        isLowLoad = Fz < FZMIN;
        Fz(isLowLoad) = real(FZMIN);
        if any(isLowLoad) && prt
            warning ('Solver:Limits:Exceeded',['Vertical load below ',...
                'the limit. Values have been saturated.']);
        end
        
        isHighLoad = Fz > FZMAX;
        Fz(isHighLoad) = real(FZMAX);
        if any(isHighLoad) && prt
            warning ('Solver:Limits:Exceeded',['Vertical load above ',...
                'the limit. Values have been saturated.']);
        end
        
        isLowPressure = pres < PRESMIN;
        pres(isLowPressure) = real(PRESMIN);
        if any(isLowPressure) && prt
            warning ('Solver:Limits:Exceeded',['Pressure below ',...
                'the limit. Values have been saturated.']);
        end
        
        isHighPressure = pres > PRESMAX;
        pres(isHighPressure) = real(PRESMAX);
        if any(isHighPressure) && prt
            warning ('Solver:Limits:Exceeded',['Pressure above ',...
                'the limit. Values have been saturated.']);
        end

        isLowSlipR = kappa < KPUMIN;
        kappa(isLowSlipR) = real(KPUMIN);
        if any(isLowSlipR) && prt
            warning ('Solver:Limits:Exceeded',['Slip ratio below ',...
                'the limit. Values have been saturated.']);
        end
        
        isHighSlipR = kappa > KPUMAX;
        kappa(isHighSlipR) = real(KPUMAX);
        if any(isHighSlipR) && prt
            warning ('Solver:Limits:Exceeded',['Slip ratio above ',...
                'the limit. Values have been saturated.']);
        end

        if any(isLowSpeed) && prt
            warning('Solver:Limits:Exceeded',['Speed input VX below ',...
                'the limit. Low speed mode activated.']);
        end

    else
        % Not using limits checks
        isLowSpeed = false(length(Fz),1);
        reductionSmooth = ones(length(Fz),1);
        reductionSharp = ones(length(Fz),1);
        reductionLinear = ones(length(Fz),1);
        isLowSpeedAlpha = false(length(Fz),1);
        reductionLinear_alpha = ones(length(Fz),1);
    end % if/else useLimitsCheck condition
    

    % IMPORTANT NOTE: Vx = Vcx [Eqn (7.4) Page 331 - Book]
    % It is assumed that the difference between the wheel centre
    % longitudinal velocity Vx and the longitudinal velocity Vcx of
    % the contact centre is negligible

    % Parameters not specified in the TIR file
    % Used to avoid low speed singularity
    epsilon  = 1e-6; % [Eqn (4.E6a) Page 178 - Book]
    epsilonv = epsilon;
    epsilonx = epsilon;
    epsilonk = epsilon;
    epsilony = epsilon;

    %% Unpack Parameters for processing
    % tirParams are fitted parameters from TIR file
    V0 = tirParams.LONGVL; %Nominal speed
    pi0	= tirParams.NOMPRES; %Nominal tyre inflation pressure
    Fz0	= tirParams.FNOMIN; %Nominal wheel load
    LFZO = tirParams.LFZO; % Scale factor of nominal (rated) load
    LMUX = tirParams.LMUX; % Scale factor of Fx peak friction coefficient
    LMUY = tirParams.LMUY; % Scale factor of Fy peak friction coefficient

    % New scaling factor in Pacejka 2012 with it's default value.
    % This scaling factor is not present in the standard MF6.1 TIR
    % files.
    LMUV = 0; % Scale factor with slip speed Vs decaying friction

    % epsilonv = internalParams.epsilonv;

    % Velocities in point S (slip point)
    % Note: why use absolute of longitudinal velocity of the contact patch
    Vsx = -kappa.*abs(Vcx); % [Eqn (4.E5) Page 181 - Book]
    Vsy = tan(alpha).*abs(Vcx); % [Eqn (2.12) Page 67 - Book] and [(4.E3) Page 177 - Book]
    % Important Note:
    % Due to the ISO sign convention, equation 2.12 does not need a
    % negative sign. The Pacejka book is written in adapted SAE.
    Vs = sqrt(Vsx.^2+Vsy.^2); % [Eqn (3.39) Page 102 - Book] -> Slip velocity of the slip point S

    % Velocities in point C (contact)
    Vcy = Vsy; % Assumption from page 67 of the book, paragraph above Eqn (2.11)
    Vc = sqrt(Vcx.^2+Vcy.^2); % Velocity of the wheel contact centre C, Not described in the book but is the same as [Eqn (3.39) Page 102 - Book]

    % Use of star (*) definition. Only valid for the book
    % implementation. TNO MF-Tyre does not use this.

    alpha_star = tan(alpha).*sign(Vcx); % [Eqn (4.E3) Page 177 - Book]
    gamma_star = sin(gamma); % [Eqn (4.E4) Page 177 - Book]


    % For the aligning torque at high slip angles
    signVc = sign(Vc);
    signVc(signVc==0) = 1;
    Vc_prime = Vc + epsilonv.*signVc; % [Eqn (4.E6a) Page 178 - Book] [sign(Vc) term explained on page 177]

    alpha_prime = acos(Vcx./Vc_prime); % [Eqn (4.E6) Page 177 - Book]

    % Slippery surface with friction decaying with increasing (slip) speed
    LMUX_star = LMUX./(1 + LMUV.*Vs./V0); % [Eqn (4.E7) Page 179 - Book]
    LMUY_star = LMUY./(1 + LMUV.*Vs./V0); % [Eqn (4.E7) Page 179 - Book]

    % Digressive friction factor
    % On Page 179 of the book is suggested Amu = 10, but after
    % comparing the use of the scaling factors against TNO, Amu = 1
    % was giving perfect match
    Amu = 1;
    LMUX_prime = Amu.*LMUX_star./(1+(Amu-1).*LMUX_star); % [Eqn (4.E8) Page 179 - Book]
    LMUY_prime = Amu.*LMUY_star./(1+(Amu-1).*LMUY_star); % [Eqn (4.E8) Page 179 - Book]

    % Effect of having a tire with a different nominal load
    Fz0_prime =  LFZO.*Fz0; % [Eqn (4.E1) Page 177 - Book]
    % Normalized change in vertical load
    dfz = (Fz - Fz0_prime)./Fz0_prime; % [Eqn (4.E2a) Page 177 - Book]
    % Normalized change in inflation pressure
    dpi = (pres - pi0)./pi0; % [Eqn (4.E2b) Page 177 - Book]

    %% Pure Longitudinal Slip Fx0 and Combined Longitudinal Slip Fx
    %[SCALING_COEFFICIENTS]
    LCX 	= tirParams.LCX ; % Scale factor of Fx shape factor
    LEX  	= tirParams.LEX ; % Scale factor of Fx curvature factor
    LKX  	= tirParams.LKX ; % Scale factor of Fx slip stiffness
    LHX  	= tirParams.LHX ; % Scale factor of Fx horizontal shift
    LVX  	= tirParams.LVX ; % Scale factor of Fx vertical shift
    LXAL    = tirParams.LXAL ; % Scale factor of alpha influence on Fx

    %[LONGITUDINAL_COEFFICIENTS]
    PCX1  	=  tirParams.PCX1 ; %Shape factor Cfx for longitudinal force
    PDX1  	=  tirParams.PDX1 ; %Longitudinal friction Mux at Fznom
    PDX2  	=  tirParams.PDX2 ; %Variation of friction Mux with load
    PDX3  	=  tirParams.PDX3 ; %Variation of friction Mux with camber squared
    PEX1  	=  tirParams.PEX1 ; %Longitudinal curvature Efx at Fznom
    PEX2  	=  tirParams.PEX2 ; %Variation of curvature Efx with load
    PEX3  	=  tirParams.PEX3 ; %Variation of curvature Efx with load squared
    PEX4  	=  tirParams.PEX4 ; %Factor in curvature Efx while driving
    PKX1  	=  tirParams.PKX1 ; %Longitudinal slip stiffness Kfx./Fz at Fznom
    PKX2  	=  tirParams.PKX2 ; %Variation of slip stiffness Kfx./Fz with load
    PKX3  	=  tirParams.PKX3 ; %Exponent in slip stiffness Kfx./Fz with load
    PHX1  	=  tirParams.PHX1 ; %Horizontal shift Shx at Fznom
    PHX2  	=  tirParams.PHX2 ; %Variation of shift Shx with load
    PVX1  	=  tirParams.PVX1 ; %Vertical shift Svx./Fz at Fznom
    PVX2  	=  tirParams.PVX2 ; %Variation of shift Svx./Fz with load
    PPX1  	=  tirParams.PPX1 ; %linear influence of inflation pressure on longitudinal slip stiffness
    PPX2  	=  tirParams.PPX2 ; %quadratic influence of inflation pressure on longitudinal slip stiffness
    PPX3  	=  tirParams.PPX3 ; %linear influence of inflation pressure on peak longitudinal friction
    PPX4  	=  tirParams.PPX4 ; %quadratic influence of inflation pressure on peak longitudinal friction
    RBX1    =  tirParams.RBX1 ; %Slope factor for combined slip Fx reduction
    RBX2    =  tirParams.RBX2 ; %Variation of slope Fx reduction with kappa
    RBX3    =  tirParams.RBX3 ; %Influence of camber on stiffness for Fx combined
    RCX1    =  tirParams.RCX1 ; %Shape factor for combined slip Fx reduction
    REX1    =  tirParams.REX1 ; %Curvature factor of combined Fx
    REX2    =  tirParams.REX2 ; %Curvature factor of combined Fx with load
    RHX1    =  tirParams.RHX1 ; %Shift factor for combined slip Fx reduction

    zeta1 = 1;
    Cx = PCX1.*LCX; % (> 0) (4.E11)
    mux = (PDX1 + PDX2.*dfz).*(1 + PPX3.*dpi + PPX4.*dpi.^2).*(1 - PDX3.*gamma.^2).*LMUX_star; % (4.E13)
    mux(Fz==0) = 0; % Zero Fz correction
    Dx = mux.*Fz.*zeta1; % (> 0) (4.E12)
    Kxk = Fz.*(PKX1 + PKX2.*dfz).*exp(PKX3.*dfz).*(1 + PPX1.*dpi + PPX2.*dpi.^2).*LKX;  % (= BxCxDx = dFxo./dkx at kappax = 0) (= Cfk) (4.E15)
    signDx = sign(Dx);
    signDx(signDx == 0) = 1; % If [Dx = 0] then [sign(0) = 0]. This is done to avoid [Kxk / 0 = NaN] in Eqn 4.E16
    Bx = Kxk./(Cx.*Dx + epsilonx.*signDx); % (4.E16) [sign(Dx) term explained on page 177]
    SHx = (PHX1 + PHX2.*dfz).*LHX; % (4.E17)
    SVx = Fz.*(PVX1 + PVX2.*dfz).*LVX.*LMUX_prime.*zeta1; % (4.E18)
    SVx(isLowSpeed) = SVx(isLowSpeed).*reductionSmooth;
    SHx(isLowSpeed) = SHx(isLowSpeed).*reductionSmooth;
    kappax = kappa + SHx; % (4.E10)
    Ex = (PEX1 + PEX2.*dfz + PEX3.*dfz.^2).*(1 - PEX4.*sign(kappax)).*LEX; % (<=1) (4.E14)
    if(any(Ex > 1))
        warning('Solver:CoeffChecks:Ex','Ex over limit (>1), Eqn(4.E14)')
        Ex(Ex > 1) = 1;
    end % if Ex > 1
    Fx0 = Dx.*sin(Cx.*atan(Bx.*kappax-Ex.*(Bx.*kappax-atan(Bx.*kappax))))+SVx; % (4.E9)
    
    Cxa = RCX1; % (4.E55)
    Exa = REX1 + REX2.*dfz; % (<= 1) (4.E56)
    if(any(Exa > 1))
        warning('Solver:CoeffChecks:Exa','Exa over limit (>1), Eqn(4.E56)')
        Exa(Exa > 1) = 1;
    end % if Exa > 1
    SHxa = RHX1; % (4.E57)
    Bxa = (RBX1 + RBX3.*gamma_star.^2).*cos(atan(RBX2.*kappa)).*LXAL; % (> 0) (4.E54)
    alphas = alpha_star + SHxa; % (4.E53)
    Gxa0 = cos(Cxa.*atan(Bxa.*SHxa-Exa.*(Bxa.*SHxa-atan(Bxa.*SHxa)))); % (4.E52)
    Gxa = cos(Cxa.*atan(Bxa.*alphas-Exa.*(Bxa.*alphas-atan(Bxa.*alphas))))./Gxa0;  % (> 0)(4.E51)
    Fx = Gxa.*Fx0; % (4.E50)

    %% Combined Lateral Slip Fy with Fx as Input Variable
    %[SCALING_COEFFICIENTS]
    LCY   = tirParams.LCY   ; % Scale factor of Fy shape factor
    LEY   = tirParams.LEY   ; % Scale factor of Fy curvature factor
    LKY   = tirParams.LKY   ; % Scale factor of Fy cornering stiffness
    LHY   = tirParams.LHY   ; % Scale factor of Fy horizontal shift
    LVY   = tirParams.LVY   ; % Scale factor of Fy vertical shift
    LKYC  = tirParams.LKYC  ; % Scale factor of camber force stiffness
    
    %[LATERAL_COEFFICIENTS]
    PCY1  =  tirParams.PCY1 	; %Shape factor Cfy for lateral forces
    PDY1  =  tirParams.PDY1 	; %Lateral friction Muy
    PDY2  =  tirParams.PDY2 	; %Variation of friction Muy with load
    PDY3  =  tirParams.PDY3 	; %Variation of friction Muy with squared camber
    PEY1  =  tirParams.PEY1 	; %Lateral curvature Efy at Fznom
    PEY2  =  tirParams.PEY2 	; %Variation of curvature Efy with load
    PEY3  =  tirParams.PEY3 	; %Zero order camber dependency of curvature Efy
    PEY4  =  tirParams.PEY4 	; %Variation of curvature Efy with camber
    PEY5  =  tirParams.PEY5 	; %Variation of curvature Efy with camber squared
    PKY1  =  tirParams.PKY1 	; %Maximum value of stiffness Kfy./Fznom
    PKY2  =  tirParams.PKY2 	; %Load at which Kfy reaches maximum value
    PKY3  =  tirParams.PKY3 	; %Variation of Kfy./Fznom with camber
    PKY4  =  tirParams.PKY4 	; %Curvature of stiffness Kfy
    PKY5  =  tirParams.PKY5 	; %Peak stiffness variation with camber squared
    PKY6  =  tirParams.PKY6 	; %Fy camber stiffness factor
    PKY7  =  tirParams.PKY7 	; %Vertical load dependency of camber stiffness
    PHY1  =  tirParams.PHY1 	; %Horizontal shift Shy at Fznom
    PHY2  =  tirParams.PHY2 	; %Variation of shift Shy with load
    PVY1  =  tirParams.PVY1 	; %Vertical shift in Svy./Fz at Fznom
    PVY2  =  tirParams.PVY2 	; %Variation of shift Svy./Fz with load
    PVY3  =  tirParams.PVY3 	; %Variation of shift Svy./Fz with camber
    PVY4  =  tirParams.PVY4 	; %Variation of shift Svy./Fz with camber and load
    PPY1  =  tirParams.PPY1 	; %influence of inflation pressure on cornering stiffness
    PPY2  =  tirParams.PPY2 	; %influence of inflation pressure on dependency of nominal tyre load on cornering stiffness
    PPY3  =  tirParams.PPY3 	; %linear influence of inflation pressure on lateral peak friction
    PPY4  =  tirParams.PPY4 	; %quadratic influence of inflation pressure on lateral peak friction
    PPY5  =  tirParams.PPY5 	; %Influence of inflation pressure on camber stiffness
    
    zeta2 = 1;
    zeta3 = 1;
    zeta0 = 1;
    zeta4 = 1;

    if kappa == 0 
    
        % Kya is cornering stiffness with Fz as input
        Kya = PKY1.*Fz0_prime.*(1 + PPY1.*dpi).*(1 - PKY3.*abs(gamma_star)).*sin(PKY4.*atan((Fz./Fz0_prime)./((PKY2+PKY5.*gamma_star.^2).*(1+PPY2.*dpi)))).*zeta3.*LKY; % (= ByCyDy = dFyo./dalphay at alphay = 0) (if gamma =0: =Kya0 = CFa) (PKY4=2)(4.E25)
        SVyg = Fz.*(PVY3 + PVY4.*dfz).*gamma_star.* LKYC .* LMUY_prime .* zeta2; % (4.E28)
        % Kyg0 is camber stiffness with Fz as input
        Kyg0 = Fz.*(PKY6 + PKY7 .*dfz).*(1 + PPY5.*dpi).*LKYC; % (=dFyo./dgamma at alpha = gamma = 0) (= CFgamma) (4.E30)
    
        signKya = sign(Kya);
        signKya(signKya == 0) = 1; % If [Kya = 0] then [sign(0) = 0]. This is done to avoid [num / 0 = NaN] in Eqn 4.E27
    
        SHy = (PHY1 + PHY2.*dfz).* LHY + ((Kyg0 .*gamma_star - SVyg)./(Kya + epsilonk.*signKya)).*zeta0 +zeta4 -1; % (4.E27) [sign(Kya) term explained on page 177]
        SVy = Fz.*(PVY1 + PVY2.*dfz).*LVY.*LMUY_prime.*zeta2 + SVyg; % (4.E29)
    
        SVy(isLowSpeed) = SVy(isLowSpeed).*reductionSmooth;
        SHy(isLowSpeed) = SHy(isLowSpeed).*reductionSmooth;
    
        alphay = alpha_star + SHy; % (4.E20)
        Cy = PCY1.*LCY; % (> 0) (4.E21)
        muy = (PDY1 + PDY2 .* dfz).*(1 + PPY3.*dpi + PPY4 .*dpi.^2).*(1 - PDY3.*gamma_star.^2).*LMUY_star; % (4.E23)
        Dy = muy.*Fz.*zeta2; % (4.E22)
        signAlphaY = sign(alphay);
        signAlphaY(signAlphaY == 0) = 1;
        Ey = (PEY1 + PEY2.*dfz).*(1 + PEY5.*gamma_star.^2 - (PEY3 + PEY4.*gamma_star).*signAlphaY).*LEY; % (<=1)(4.E24)
    
        if(any(Ey > 1))
            warning('Solver:CoeffChecks:Ey','Ey over limit (>1), Eqn(4.E24)')
            Ey(Ey > 1) = 1;
        end % if Ey > 1
    
        signDy = sign(Dy);
        signDy(signDy == 0) = 1; % If [Dy = 0] then [sign(0) = 0]. This is done to avoid [Kya / 0 = NaN] in Eqn 4.E26
        By = Kya./(Cy.*Dy + epsilony.*signDy); % (4.E26) [sign(Dy) term explained on page 177]
        Fy = Dy .* sin(Cy.*atan(By.*alphay-Ey.*(By.*alphay - atan(By.*alphay))))+ SVy; % (4.E19)
    
        Fy(Vcx < 0) = -Fy(Vcx < 0);

    else
        % CFa_Fz (Kya) and CFg_Fz(Kyg0) with Fz as input
        CFa_Fz = PKY1.*Fz0_prime.*(1 + PPY1.*dpi).*(1 - PKY3.*abs(gamma_star)).*sin(PKY4.*atan((Fz./Fz0_prime)./((PKY2+PKY5.*gamma_star.^2).*(1+PPY2.*dpi)))).*zeta3.*LKY; % (= ByCyDy = dFyo./dalphay at alphay = 0) (if gamma =0: =Kya0 = CFa) (PKY4=2)(4.E25)
        CFg_Fz = Fz.*(PKY6 + PKY7 .*dfz).*(1 + PPY5.*dpi).*LKYC; % (=dFyo./dgamma at alpha = gamma = 0) (= CFgamma) (4.E30)
    
        % Friction Coefficient of pure longitudinal slip with Fz0 as input
        mux0 = (PDX1 + PDX2.*((Fz0 - Fz0_prime)./Fz0_prime)).*(1 + PPX3.*dpi + PPX4.*dpi.^2).*(1 - PDX3.*gamma.^2).*LMUX_star; % (4.E13)
    
        phix = (mux^2 * Fz^2 - Fx^2)^0.5 / (mux * Fz); % (4.41 page 163), friction coefficient for muy = mux
        phixa = sqrt(1 - (Fx/(mux*Fz))^2); % (4.42a page 164), where n in [2,8]
        Cfa = phixa * (CFa_Fz - 0.5*mux*Fz) + 0.5*(mux*Fz - Fx); % (4.42 page 164)
        Cfg = phix^2 * CFg_Fz; % (4.43 page 164)
        alpha_eq = 1/phix * (Cfa/CFa_Fz)*(mux0*Fz0)/(mux*Fz)*(alpha+(Cfg/Cfa).*gamma); % (4.46 page 164)
    
        % Pure lateral slip with alpha_eq as the input
        Kya = PKY1.*Fz0_prime.*(1 + PPY1.*dpi).*(1 - PKY3.*abs(gamma_star)).*sin(PKY4.*atan((Fz./Fz0_prime)./((PKY2+PKY5.*gamma_star.^2).*(1+PPY2.*dpi)))).*zeta3.*LKY; % (= ByCyDy = dFyo./dalphay at alphay = 0) (if gamma =0: =Kya0 = CFa) (PKY4=2)(4.E25)
        SVyg = Fz.*(PVY3 + PVY4.*dfz).*gamma_star.* LKYC .* LMUY_prime .* zeta2; % (4.E28)
        Kyg0 = Fz.*(PKY6 + PKY7 .*dfz).*(1 + PPY5.*dpi).*LKYC; % (=dFyo./dgamma at alpha = gamma = 0) (= CFgamma) (4.E30)
        signKya = sign(Kya);
        signKya(signKya == 0) = 1; % If [Kya = 0] then [sign(0) = 0]. This is done to avoid [num / 0 = NaN] in Eqn 4.E27
        SHy = (PHY1 + PHY2.*dfz).* LHY + ((Kyg0 .*gamma_star - SVyg)./(Kya + epsilonk.*signKya)).*zeta0 +zeta4 -1; % (4.E27) [sign(Kya) term explained on page 177]
        SVy = Fz.*(PVY1 + PVY2.*dfz).*LVY.*LMUY_prime.*zeta2 + SVyg; % (4.E29)
        SVy(isLowSpeed) = SVy(isLowSpeed).*reductionSmooth;
        SHy(isLowSpeed) = SHy(isLowSpeed).*reductionSmooth;
        alphay = tan(alpha_eq).*sign(Vcx) + SHy; % (4.E20) [alpha_star changed to apply alpha_eq]
        Cy = PCY1.*LCY; % (> 0) (4.E21)
        % consider not using muy as mu = mux
        % muy = (PDY1 + PDY2 .* ((Fz0 - Fz0_prime)./Fz0_prime)).*(1 + PPY3.*dpi + PPY4 .*dpi.^2).*(1 - PDY3.*gamma_star.^2).*LMUY_star; % (4.E23)
        Dy = mux.*Fz.*zeta2; % (4.E22)
        signAlphaY = sign(alphay);
        signAlphaY(signAlphaY == 0) = 1;
        Ey = (PEY1 + PEY2.*dfz).*(1 + PEY5.*gamma_star.^2 - (PEY3 + PEY4.*gamma_star).*signAlphaY).*LEY; % (<=1)(4.E24)
        if(any(Ey > 1))
            warning('Solver:CoeffChecks:Ey','Ey over limit (>1), Eqn(4.E24)')
            Ey(Ey > 1) = 1;
        end % if Ey > 1
        signDy = sign(Dy);
        signDy(signDy == 0) = 1; % If [Dy = 0] then [sign(0) = 0]. This is done to avoid [Kya / 0 = NaN] in Eqn 4.E26
        By = Kya./(Cy.*Dy + epsilony.*signDy); % (4.E26) [sign(Dy) term explained on page 177]
        Fy0eq = Dy .* sin(Cy.*atan(By.*alphay-Ey.*(By.*alphay - atan(By.*alphay))))+ SVy; % (4.E19)
        Fy0eq(Vcx < 0) = -Fy0eq(Vcx < 0);
        Fy = phix * (mux*Fz)/(mux0*Fz0)*Fy0eq; % (4.45 page 164)
    end
