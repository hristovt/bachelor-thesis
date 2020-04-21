%% Altitude
figure;
title('Altitude');
plot(sim_h.time, [sim_h.signals.values/1000], 'Linewidth',2);
xlabel('Time [s]');
ylabel('h [km]');

%% Temperature
figure;
title('Temperature');
plot(sim_temp.signals.values, sim_h.signals.values/1000, 'Linewidth',2);
ylabel('h [km]');
xlabel('T [K]');

%% Pressure
figure;
title('Pressure');
plot(sim_p_a.signals.values/1000, sim_h.signals.values/1000, 'Linewidth',2);
ylabel('h [km]');
xlabel('p [kPa]');

%% Air density
figure;
title('Air density');
plot(sim_airrho.signals.values, sim_h.signals.values/1000, 'Linewidth',2);
ylabel('h [km]');
xlabel('\rho [kgm^{-3}]');

%% Thrust
figure;
title('Thrust');
plot(sim_thrust.time, sim_thrust.signals.values, 'Linewidth',2);
xlabel('time [s]');
ylabel('T [N]');
axis([0 180 7.4e6 8.6e6]);

%% Delta
figure;
title('Delta');
plot(sim_delta.time, [sim_delta.signals.values], 'Linewidth',2);
xlabel('time [s]');
ylabel('\delta [rad]');
legend('\delta_{\theta}', '\delta_{\psi}','location', 'northeast');