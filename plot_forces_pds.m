function plot_forces_pds(sol_pds)

u = sol_pds.u;

m = exp(sol_pds.z);
time = 0:5:sol_pds.t_f;
input_norm = norms(sol_pds.u);
thrust_vectorX = rad2deg(acos(u(1,:)./input_norm));
thrust_vectorY = rad2deg(acos(u(2,:)./input_norm));
thrust_vectorZ = rad2deg(acos(u(3,:)./input_norm));
thrust_magnitude = input_norm .* m;

figure; 
plot(time, u(1,:), time, u(2,:), time, u(3,:));
title('Thrust Each Axis');
xlabel('Timing Index (s)')
ylabel('Acceleration (m/s^2)')
legend('x-accel','y-accel','z-accel')
grid minor

figure;
plot(time, thrust_magnitude./5000/max(thrust_magnitude));
title('Thrust Magnitude- kN');
xlabel('Timing Index (s)')
ylabel('Thrust (kN)')
grid minor

figure;
plot(time, thrust_vectorX, time, thrust_vectorY, time, thrust_vectorZ);
title('Thrust Vector');
xlabel('Timing Index (s)')
ylabel('Angle (deg)')
legend('X','Y','Z')
grid minor
hold off;
end