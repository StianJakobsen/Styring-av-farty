% Project in TTK4190 Guidance and Control of Vehicles 
%
% Author:           My name
% Study program:    My study program

clear all;
close all;
clc;

load('WP.mat')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h  = 0.1;    % sampling time [s]
Ns = 90000;  % no. of samples

psi_ref = 10 * pi/180;  % desired yaw angle (rad)
U_d = 7;                % desired cruise speed (m/s)
               
% ship parameters 
m = 17.0677e6;          % mass (kg)
Iz = 2.1732e10;         % yaw moment of inertia about CO (kg m^3)
xg = -3.7;              % CG x-ccordinate (m)
L = 161;                % length (m)
B = 21.8;               % beam (m)
T = 8.9;                % draft (m)
KT = 0.7;               % propeller coefficient (-)
Dia = 3.3;              % propeller diameter (m)
rho = 1025;             % density of water (kg/m^3)
visc = 1e-6;            % kinematic viscousity at 20 degrees (m/s^2)
eps = 0.001;            % a small number added to ensure that the denominator of Cf is well defined at u=0
k = 0.1;                % form factor giving a viscous correction
t_thr = 0.05;           % thrust deduction number

PD = 1.5;               % pitch-diamter ratio
AEAO = 0.65;            % blade area ratio
Ja = 0;                 % open-water advance coefficient
z = 4;                  % number of propeller blades


% Compute KT and KQ
[KT,KQ] = wageningen(Ja,PD,AEAO,z);


% rudder limitations
delta_max  = 40 * pi/180;        % max rudder angle      (rad)
Ddelta_max = 5  * pi/180;        % max rudder derivative (rad/s)

% added mass matrix about CO
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;
MA = -[ Xudot 0    0 
        0 Yvdot Yrdot
        0 Nvdot Nrdot ];

% rigid-body mass matrix
MRB = [ m 0    0 
        0 m    m*xg
        0 m*xg Iz ];
    
Minv = inv(MRB + MA); % Added mass is included to give the total inertia

% ocean current in NED
Vc = 1;                             % current speed (m/s)
betaVc = deg2rad(45);               % current direction (rad)


% wind expressed in NED
Vw = 10;                   % wind speed (m/s)
betaVw = deg2rad(135);     % wind direction (rad)
rho_a = 1.247;             % air density at 10 deg celsius
cy = 0.95;                 % wind coefficient in sway
cn = 0.15;                 % wind coefficient in yaw
A_Lw = 10 * L;             % projected lateral area

% linear damping matrix (only valid for zero speed)
T1 = 20; T2 = 20; T6 = 10;

Xu = -(m - Xudot) / T1;
Yv = -(m - Yvdot) / T2;
Nr = -(Iz - Nrdot)/ T6;
D = diag([-Xu -Yv -Nr]);         % zero speed linear damping

% rudder coefficients (Section 9.5)
b = 2;
AR = 8;
CB = 0.8;

lambda = b^2 / AR;
tR = 0.45 - 0.28*CB;
CN = 6.13*lambda / (lambda + 2.25);
aH = 0.75;
xH = -0.4 * L;
xR = -0.5 * L;

X_delta2 = 0.5 * (1 - tR) * rho * AR * CN;
Y_delta = 0.25 * (1 + aH) * rho * AR * CN; 
N_delta = 0.25 * (xR + aH*xH) * rho * AR * CN;   

% input matrix
Bu = @(u_r,delta) [ (1-t_thr)  -u_r^2 * X_delta2 * delta
                        0      -u_r^2 * Y_delta
                        0      -u_r^2 * N_delta            ];
                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                    
% Heading Controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

CRB_star = [0 0 0
             0 0 m*U_d
             0 0 m*xg*U_d];
         
CA_star = [0 0 0
            0 0 -Xudot*U_d
            -0 (Xudot-Yvdot)*U_d -Yrdot*U_d];  
        
M = MRB + MA;
N = CRB_star + CA_star + D;

b_lin = [-2*U_d*Y_delta -2*U_d*N_delta]';


[num,denum] = ss2tf(-M(2:3,2:3)\N(2:3,2:3), M(2:3,2:3)\b_lin, [0 1], 0);

K = num(3)/denum(3);
T_3 = num(2)/num(3);

T_12 = denum(2)/denum(3); % T1 + T2
T_nomoto = T_12 - T_3;
%169.54 nice

d_nomoto = 1/K;
m_nomoto = T_nomoto/K;

%Kalman

A_kc = [0 1 0; 0 -1/T_nomoto -K/T_nomoto; 0 0 0];
B_kc = [0 ; K/T_nomoto; 0];
E_kc = [0 0; 1 0; 0 1];
C_kc = [1 0 0];

A_kd = eye(3) + h*A_kc;
B_kd = B_kc;
C_kd = C_kc;
E_kd = E_kc;




% rudder control law
wb = 0.06;
zeta = 1;
wn = 1 / sqrt( 1 - 2*zeta^2 + sqrt( 4*zeta^4 - 4*zeta^2 + 2) ) * wb;

Kp = m_nomoto* wn^2 - k;
Kd = 2*zeta*wn*m_nomoto - d_nomoto;
Ki = wn*Kp/10;


% linearized sway-yaw model (see (7.15)-(7.19) in Fossen (2021)) used
% for controller design. The code below should be modified.

% initial states
eta = [0 0 0]';
nu  = [0.1 0 0]';
delta = 0;
n = 0;
x_d = [0 0 0]';
e_int = 0; % integral state for the error

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata = zeros(Ns+1,15);                % table of simulation data

Qm = 0;

R_los = 16*L;
step = 1;
x_ref = WP(1,step); y_ref = WP(2,step);
x_t = WP(1, step+1); y_t = WP(2, step+1);

psi_d = guidance(eta, x_t, y_t, x_ref, y_ref, h);
display(psi_d)
% Ki = 0;
for i=1:Ns+1
    
    if (x_t - eta(1))^2 + (y_t - eta(2))^2 <= R_los^2 
        step = step + 1;
        if step < size(WP,2)
            x_ref = WP(1,step); y_ref = WP(2,step);
            x_t = WP(1, step+1); y_t = WP(2, step+1);
        end
    end
    
    chi_d = guidance(eta, x_t, y_t, x_ref, y_ref, h);
    psi_ref = chi_d;
%     psi_ref = deg2rad(-150);
%     display(psi_d)
    t = (i-1) * h;                      % time (s)
    R = Rzyx(0,0,eta(3));
    
    u_c = Vc*cos(betaVc-eta(3));
    v_c = Vc*sin(betaVc-eta(3));
    V_c = [u_c, v_c, 0]';
    
    % current (should be added here)
    nu_r = nu - V_c;
    gamma_w = eta(3) - betaVw - pi;
    
    
    % wind (should be added here)
    if t > 200
        Ywind = 1/2*rho_a*Vw*cy*sin(gamma_w)*A_Lw; % expression for wind moment in sway should be added.
        Nwind = 1/2*rho_a*Vw*cn*sin(2*gamma_w)*A_Lw*L; % expression for wind moment in yaw should be added.
    else
        Ywind = 0;
        Nwind = 0;
    end
    tau_env = [0 Ywind Nwind]';
    
    % state-dependent time-varying matrices
    CRB = m * nu(3) * [ 0 -1 -xg 
                        1  0  0 
                        xg 0  0  ];
                    
    % coriolis due to added mass
    CA = [  0   0   Yvdot * nu_r(2) + Yrdot * nu_r(3)
            0   0   -Xudot * nu_r(1) 
          -Yvdot * nu_r(2) - Yrdot * nu_r(3)    Xudot * nu_r(1)   0];
    
    N = CRB + CA + D;
    
    % nonlinear surge damping
    Rn = L/visc * abs(nu_r(1));
    Cf = 0.075 / ( (log(Rn) - 2)^2 + eps);
    Xns = -0.5 * rho * (B*L) * (1 + k) * Cf * abs(nu_r(1)) * nu_r(1);
    
    % cross-flow drag
    Ycf = 0;
    Ncf = 0;
    dx = L/10;
    Cd_2D = Hoerner(B,T);
    for xL = -L/2:dx:L/2
        vr = nu_r(2);
        r = nu_r(3);
        Ucf = abs(vr + xL * r) * (vr + xL * r);
        Ycf = Ycf - 0.5 * rho * T * Cd_2D * Ucf * dx;
        Ncf = Ncf - 0.5 * rho * T * Cd_2D * xL * Ucf * dx;
    end
    d = -[Xns Ycf Ncf]';
    
    % reference models
    %[A_ref, B_ref, C_ref, D_ref] = tf2ss(wn^3, [1, (2*zeta+1)*wn, (2*zeta+1)*wn^2, wn^3]);
    w_ref = 0.08;
    A_ref = [0 1 0; 0 0 1; -w_ref^3 -(2*zeta+1)*w_ref^2 -(2*zeta+1)*w_ref];
    B_ref = [0 0 w_ref^3]';
    x_d_dot = A_ref*x_d + B_ref*psi_ref;
    psi_d = x_d(1);
    r_d = x_d(2);
    u_d = U_d;
    
   
    
    % thrust 
    thr = rho * Dia^4 * KT * abs(n) * n;    % thrust command (N)
        
    % control law
    e = [ssa(eta(3) - psi_d);nu(3) - r_d];
    delta_c = -(Kp*e(1) + Ki*e_int + Kd*e(2));
    %delta_c = 0.1;              % rudder angle command (rad)
        
    % ship dynamics
    u = [ thr delta ]';
    tau = Bu(nu_r(1),delta) * u;
    nu_dot = Minv * (tau_env + tau - N * nu_r - d); 
    eta_dot = R * nu;    
    
    % Rudder saturation and dynamics (Sections 9.5.2)
    if abs(delta_c) >= delta_max
        delta_c = sign(delta_c)*delta_max;
    end
    
%     Anti-integrator windup
    if Ki~=0
        u_unsat = (Kp*e(1) + Ki*e_int + Kd*e(2));
%         e_int = e_int + h/Ki * (delta_c - u_unsat);
        e_int = euler2(delta_c - u_unsat, e_int, h/Ki);
    end

    
    delta_dot = delta_c - delta;
    if abs(delta_dot) >= Ddelta_max
        delta_dot = sign(delta_dot)*Ddelta_max;
    end    
    
    % propeller dynamics
    Im = 100000; Tm = 10; Km = 0.6;         % propulsion parameters
    n_c = 10;                               % propeller speed (rps)
    
    T_n = rho*Dia^4*KT*abs(n)*n;
    Q_n = rho*Dia^5*KQ*abs(n)*n;
    
    T_d = (U_d*Xu)/(t_thr-1);
    
    n_d = sign(T_d)*sqrt(T_d/(rho*Dia^4*KT));
    
    Q_d = rho*Dia^5*KQ*abs(n_d)*n_d;

    
    % Calculate Qm
    Y = Q_d/Km;
    Qm_dot = 1/Tm*(-Qm+Km*Y);
    
    n_dot = (1/Im)*(Qm - Q_n);
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [t n_c delta_c n delta eta' nu' u_d psi_d r_d chi_d];       
     
    % Euler integration
    eta = euler2(eta_dot,eta,h);
    nu  = euler2(nu_dot,nu,h);
    delta = euler2(delta_dot,delta,h);   
    n  = euler2(n_dot,n,h);
    x_d = euler2(x_d_dot,x_d,h);
    e_int = euler2(e(1), e_int, h);
    Qm = euler2(Qm_dot, Qm, h);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t       = simdata(:,1);                 % s
n_c     = 60 * simdata(:,2);            % rpm
delta_c = (180/pi) * simdata(:,3);      % deg
n       = 60 * simdata(:,4);            % rpm
delta   = (180/pi) * simdata(:,5);      % deg
x       = simdata(:,6);                 % m
y       = simdata(:,7);                 % m
psi     = (180/pi) * simdata(:,8);      % deg
u       = simdata(:,9);                 % m/s
v       = simdata(:,10);                % m/s
r       = (180/pi) * simdata(:,11);     % deg/s
u_d     = simdata(:,12);                % m/s
psi_d   = (180/pi) * simdata(:,13);     % deg
r_d     = (180/pi) * simdata(:,14);     % deg/s

figure(1)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions (m)'); xlabel('time (s)'); 
subplot(312)
plot(t,psi,t,psi_d,'linewidth',2);
title('Actual and desired yaw angles (deg)'); xlabel('time (s)');
legend('Actual', 'Desired')
subplot(313)
plot(t,r,t,r_d,'linewidth',2);
title('Actual and desired yaw rates (deg/s)'); xlabel('time (s)');

figure(2)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocities (m/s)'); xlabel('time (s)');
subplot(312)
plot(t,n,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed (rpm)'); xlabel('time (s)');
legend('Actual', 'Desired')
subplot(313)
plot(t,delta,t,delta_c,'linewidth',2);
title('Actual and commanded rudder angles (deg)'); xlabel('time (s)');


figure(3) 
figure(gcf)
subplot(211)
plot(t,u,'linewidth',2);
title('Actual surge velocity (m/s)'); xlabel('time (s)');
subplot(212)
plot(t,v,'linewidth',2);
title('Actual sway velocity (m/s)'); xlabel('time (s)');

figure(4)
plot(y,x,'linewidth',2); axis('equal');
plot_path(WP);
