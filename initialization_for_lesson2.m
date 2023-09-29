clear all
%% DFIG parameters -> Rotor parameters referred to the stator side
f = 50;                % Stator frequency (Hz)
Ps = 2e6;              % Rated stator power (W)
n = 1500;              % Rated rotational speed (rev/min)
Vs = 690;              % Rated stator volatge (V)
Is = 1760;             % Rated stator current (A)
Tem = 12732;           % Rated torque (N,m)

p=2;                     % Pole pair
u = 1/3;                 % Stator/rotor turns ratio
Vr = 2070;               % Rated rotor voltage (non-reached) (V)
smax = 1/3;              % Maximum slip
Vr_stator = (Vr*smax)*u; % Rated rotor voltage referred to stator
Rs = 2.6e-3;             % Stator resistance (ohm)
Lsi = 0.087e-3;           % Leakage inductance (stator & rotor)  (H)
Lm = 2.5e-3;             % Magnetizing inductance (H)
Rr = 2.9e-3;             % Rotor resistance referred to stator
Ls = Lm + Lsi;           % Stator inductance (H)
Lr = Lm + Lsi;           % Rotor inductance (H)
Vbus = Vr_stator*sqrt(2); % DC dq bus voltage referred to stator (V)
sigma = 1 - Lm^2/(Ls*Lr);

Vs = 690*sqrt(2/3);
ws = f*2*pi;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Maximum and minimum speeds the 2.4MW

wt_nom = 18;
wt_min = 9;

% Gearbox ratio

N = 100;

% Torque and speed arrays (INPUTS)

wt = [ 0.9425,0.9425,0.9425,0.9524,1.0476,1.1429,1.2381,1.3333,1.4286,...
    1.5238,1.6190,1.7143,1.8095,1.8850,1.8850,1.8850,1.8850,1.8850,...
    1.8850,1.8850,1.8850];
Torque = -1e6*[0.0501,0.1164,0.1925,0.2696,0.3262,0.3883,0.4557,0.5285,...
    0.6067,0.6902,0.7792,0.8736,0.9733,1.0894,1.2474,1.2745,1.2745,...
    1.2745,1.2745,1.2745,1.2745  ];

Temm = Torque/N;                % Convert to machine side
wmm = wt*(60/2/pi)*N;            % Convert to machine side (in rpm)
sss = (1500 - wmm)/1500;        % slip

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% Qs=0  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ref_Qs_ = 0*ones(size(sss));
i = 0;
for ss = sss;
    i = i + 1;
    Tem = Temm(i);
    Ref_Qs = Ref_Qs_(i);
    wm = ws * (1-ss);
    s(i) = ss;
    A = ws*ws*Lm*Lm;
    B = 4*Rs*Tem*ws/3-((Vs)^2);
    C = 4*Rs*Rs/(9*Lm*Lm)*((Ref_Qs^2)/(ws^2)+(Tem.^2)/(p^2));
    x1 = (-B+sqrt(B.^2-4*A.*C))./(2*A);
    x2 = (-B-sqrt(B.^2-4*A.*C))./(2*A);
    Fs = Lm*sqrt(x1);
    Fs_(i) = Fs;
    
    % Stator currents
    ids = 2*Ref_Qs./(2*ws*Fs);
    iqs = 2*Tem./(3*p*Fs);
    Is(i) = sqrt(ids^2 + iqs^2);
    
    % Stator voltages
    uds = Rs * ids;
    uqs = Rs * iqs + ws * Fs;
    Us(i) = sqrt(uds^2 + uqs^2);
    
    % Stator power
    Ps(i) = 1.5*(Rs*ids.^2 + Rs*iqs.^2 + ws*Fs.*iqs);
    Qs(i) = Ref_Qs;
    Mod_Ss = sqrt(Ps.^2 + Qs.^2);
    
    % Rotor currents
    idr = -Ls*ids/Lm + Fs/Lm;
    iqr = -Ls*iqs/Lm;
    Ir(i) = sqrt(idr^2 + iqr^2);
    
    % wr
    wr = ws*ss;
    
    % Rotor voltage
    udr = Rr*idr-Lr*wr.*iqr-Lm*wr.*iqs;
    uqr = Rr*iqr+Lr*wr.*idr+Lm*wr.*ids;
    Vr(i) = sqrt(udr^2 + uqr^2);
    
    % Rotor power
    Pr(i) = 1.5*(udr.*idr + uqr.*iqr);
    Qr(i) = 1.5*(uqr.*idr - udr.*iqr);
    Mod_Sr = sqrt(Pr.^2 + Qr.^2);
    
    % Stator flux
    Fsd = Ls*ids + Lm*idr;
    Fsq = Ls*iqs + Lm*iqr;
    Fs_(i) = sqrt(Fsd*Fsd + Fsq*Fsq);
    
    % Rotor flux
    Frd = Lr*idr + Lm*ids;
    Frq = Lr*iqr + Lm*iqs;
    Fr(i) = sqrt(Frd*Frd + Frq*Frq);
    
    % Mechanical power
    Pmec(i) = Tem*wm/p;
    
    % Efficency
    if Pmec(i)>=0
        R(i)=Pmec(i)/(Ps(i)+Pr(i)); % Motor mode
    else
        R(i)=(Ps(i)+Pr(i))/Pmec(i); % Generator mode
    end
end
    
    figure (1)
    subplot(3,3,1)
    hold on
    plot(wmm,Temm,'r',wmm,Temm,'or','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Tem (Nm)','fontsize',14);
    xlim([700, 2000]);
    
    subplot(3,3,2)
    hold on
    plot(wmm,Ps+Pr,'r',wmm,Ps+Pr,'or','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Pt (W)','fontsize',14);
    xlim([700, 2000]);
    
    subplot(3,3,3)
    hold on
    plot(wmm,Pr,'r',wmm,Pr,'ogr','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Pr (W)','fontsize',14);
    xlim([700, 2000])
    
    subplot(3,3,3)
    hold on
    plot(wmm,Ps,'r',wmm,Ps,'or','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Ps & Pr(w)', 'fontsize',14);
    xlim([700, 2000]);
    
    subplot(3,3,4)
    hold on
    plot(wmm,Is,'r',wmm,Is,'or','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Is (A)','fontsize',14);
    xlim([700, 2000]);
    
    subplot(3,3,5)
    hold on
    plot(wmm,Ir,'r',wmm,Ir,'or','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Ir (A)','fontsize',14);
    xlim([700, 2000]);
    
    subplot(3,3,6)
    hold on
    plot(wmm,Vr,'r',wmm,Vr,'or',wmm,Us,'r',wmm,Us,'or','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Vr & Vs (V)','fontsize',14);
    xlim([700, 2000]);
    
    subplot(3,3,7)
    hold on
    plot(wmm,Qs,'r',wmm,Qs,'or','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Qs (VAR)','fontsize',14);
    xlim([700, 2000]);
    
    subplot(3,3,8)
    hold on
    plot(wmm,Qr,'r',wmm,Qr,'or','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Qr (VAR)','fontsize',14);
    xlim([700, 20000]);
    
    subplot(3,3,9)
    hold on
    plot(wmm,R,'r',wmm,R,'or','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Efficiency (pu)','fontsize',14);
    xlim([700, 2000]);
    
    %%%%%%%%%%%%%%%%%%%%%% idr=0 %%%%%%%%%%%%%%%%
    i = 0;
    for ss = sss;
        i = i + 1;
        Tem = Temm(i);
        wm = ws*(1-ss);
        s(i) = ss;
    A = ws*ws*Lm*Lm + (Rs*Lm/Ls)^2;
    B = 4*Rs*Tem*ws/3/p-((Vs)^2);
    C = 4*Rs*Rs*Tem*Tem*Tem/(9*p*p*Lm*Lm);
    x1 = (-B+sqrt(B.^2-4*A.*C))./(2*A);
    x2 = (-B-sqrt(B.^2-4*A.*C))./(2*A);
    ims = sqrt(x1);
    ims_(i) = ims;
    Fs = ims*Lm;
    
    % Rotor currents
    idr = 0;
    iqr = -2*Tem*Ls./(3*p*Lm*Fs);
    Ir(i) = sqrt(idr^2 + iqr^2);
    
    % Stator currents
    ids = Fs/Ls;
    iqs = -iqr*Lm/Ls;
    Is(i) = sqrt(ids^2+iqs^2);
    
    % Stator voltages
    uds = Rs*ids;
    uqs = Rs*iqs+ws*Fs;
    Us(i) = sqrt(uds^2+uqs^2);
    
    % Stator powers
    Ps(i) = 1.5*(Rs*ids.^2+Rs*iqs.^2+ws*Fs.*iqs);
    Qs(i) = 1.5*(uqs.*ids-uds.*iqs);
    Mod_Ss = sqrt(Ps.^2+Qs.^2);
    
    % wr
    wr = ws*ss;
    
    % Rotor voltages
    udr = Rr*idr-Lr*wr.*iqr-Lm*wr.*iqs;
    uqr = Rr*iqr+Lr*wr.*idr+Lm*wr.*ids;
    Vr(i) = sqrt(udr^2+uqr^2);
    
    % Rotor powers
    Pr(i) = 1.5*(udr.*idr+uqr.*iqr);
    Qr(i) = 1.5*(uqr.*idr-udr.*iqr);
    Mod_Sr = sqrt(Pr.^2+Qr.^2);
    
    % Stator flux
    Fsd = Ls*ids+Lm*idr;
    Fsq = Ls*iqs+Lm*iqr;
    Fs_(i) = sqrt(Fsd*Fsd+Fsq*Fsq);
    
    % Rotor flux
    Frd = Lr*idr+Lm*ids;
    Frq = Lr*iqr+Lm*iqs;
    Fr(i) = sqrt(Frd*Frd+Frq*Frq);
    
    % Mechanical power
    Pmec(i) = Tem*wm/p;
    
    % Efficiency
    if Pmec(i)>=0
        R(i) = Pmec(i)/(Ps(i)+Pr(i)); % Motor mode
    else
        R(i) = (Ps(i)+Pr(i))/Pmec(i); % Generator mode
    end
    end
    
    figure(1)
    subplot(3,3,1)
    hold on
    plot(wmm,Temm,'g',wmm,Temm,'og','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Tem (Nm)','fontsize',14);
    grid
    xlim([700, 2000]);
    
    subplot(3,3,2)
    title('Red: Qs=0, Green:idr=0','fontsize',12)
    hold on
    plot(wmm,Ps+Pr,'g',wmm,Ps+Pr,'og','linewidth',1.5);
    xlabel('n(rpm)','fontsize',14);
    ylabel('Pt (W)','fontsize',14);
    grid
    xlim([700, 2000]);
    
    subplot(3,3,3)
    hold on
    plot(wmm,Ps,'g',wmm,Ps,'*g','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Ps & Pr (W)','fontsize',14);
    grid
    xlim([700, 2000]);
    
    subplot(3,3,4)
    hold on
    plot(wmm,Is,'g',wmm,Is,'og','linewidth',1.5);
    xlabel('n(rpm)','fontsize',14);
    ylabel('Is (A)','fontsize',14);
    grid
    xlim([700, 2000]);
    
    subplot(3,3,5)
    hold on
    plot(wmm,Ir,'g',wmm,Ir,'og','linewidth',1.5);
    xlabel('n(rpm)','fontsize',14);
    ylabel('Ir (A)','fontsize',14);
    grid
    xlim([700, 2000]);
    
    subplot(3,3,6)
    hold on
    plot(wmm,Vr,'g',wmm,Vr,'og',wmm,Us,'g',wmm,Us,'og','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Vr & Vs (V)','fontsize',14);
    grid
    xlim([700, 2000]);
    
    subplot(3,3,7)
    hold on
    plot(wmm,Qs,'g',wmm,Qs,'og','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Qs (VAR)','fontsize',14);
    grid
    xlim([700, 2000]);
    
    subplot(3,3,8)
    hold on
    plot(wmm,Qr,'g',wmm,Qr,'og','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    grid
    xlim([700 2000]);
    
    subplot(3,3,9)
    hold on
    plot(wmm,R,'g',wmm,R,'og','linewidth',1.5);
    xlabel('n (rpm)','fontsize',14);
    ylabel('Efficiency (pu)','fontsize',14);
    grid
    xlim([700, 2000]);
   