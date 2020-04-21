%% Initial conditions
%Earth Initial conditions

omega_E=7.2921*10^-5;       %Earth speed [rad/s]

phi_init=0;                 %y-axis [degrees]
lambda_init=0;              %z-axis [degrees]

r_init=[6378137, 0, 0];     %Equatorial radius [m]

%% Launch Viachel

m=550000;                   %Mass of LV[kg]
area=21;                    %Aerodynamic reference area [m^2]
l=3.24;                     %Lauch Viachle reference length [m]
r_cp=[-3.5; 0; 0];          %Position of center of presure wrt center of gravity [m]
r_gimble=[-21; 0; 0];       %Position of gimble wrt center of gravity [m]


engine_number=9;
dot_m=280;                  %Engine mass flow rate [kg/s]
v_exit=251;                 %Engine exit velocity [m/s]
p_exit=9e5;                 %Engine exit pressure [Pa]
A_nozzle=0.97;              %Engine nozzle exit area [m^2]
t_burn = 250;               % burn time [s]


J11 = 1.2e5;
J22 = 2.1e7;                %Moments of inertia around the three axis [kg*m^2] 
J33 = 2.1e7;

J = [J11; J22; J33];        %Main moments of inertia [kg*m^2]

%% Aerodynamic Coefficient (for v=200m/s)

%Forces
C_A_0=0.2;
C_A_Alpha=0.1;
C_A_Beta=0.1;

C_N_0=0;
C_N_Alpha=2;

C_S_0=0;
C_S_Beta=2;

%Moments
C_Mroll_0 = 0;
C_Mpitch_alpha = 0.02;
C_Myaw_beta = 0.02;

C_Mroll_p = 2;
C_Mpitch_q = 2;
C_Myaw_r = 2;

%% Transformations
% Initialization of transformation matrices.
EGtoB = eye(3);
ECFtoEG = ECEF_to_EG(phi_init, lambda_init);
ECItoECF = eye(3);

