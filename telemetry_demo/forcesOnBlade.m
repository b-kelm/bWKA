%% Calculating the centrifugal acceleration

m = 0.170; % [kg] - Mass of one blade
r = 0.25; % [m] - Radius of Center of Mass of one blade
omega_max = 1350/60 * 2*pi; % [rad/s] - Angular Speed


a_cf = r * omega_max^2; % Centrifugal Acceleration
fprintf("Blade centrifugal acceleration a_cf = %4.0f [m/s^2] = %4.0f [g] \n", a_cf, a_cf/9.81);

% Forces
F_cf = m * a_cf; % Centrifugal Force on one blade
fprintf("Centrifugal force on 1 blade F_cf_1 = %4.0f [N], on 2 blades F_cf_2 = %4.0f [N] ^= %3.0f [kg] \n", F_cf, F_cf*2, F_cf*2/9.81);

