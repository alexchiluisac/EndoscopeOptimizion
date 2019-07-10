addpath('utils/')
addpath('kinematics/')

id = 1.3;
od = 1.5;

cutouts.h = [1 1 1 1];
cutouts.w = [1.2 1.2 1.2 1.2];
cutouts.u = [1 1 1 1];
cutouts.alpha = [0 0 0 0];

wrist = Wrist(id, od, 4, cutouts);

% Generate arclength array
s = linspace(0, 10, 10);

% Generate a linearly increasing curvature
slope = 0.01;
yIntercept = 0;
kappa = s * slope + yIntercept;

% Generate torsion
torsion = 0;

% The radius of the curve
% r = 1 ./ kappa;
% 
% dr = gradient(r);
% 
% T = dr ./ norm(dr);

% initial conditions
t0 = [0; 0; 1];
n0 = [1; 0; 0];

a = cos(kappa .* s);
b = sin(kappa .* s);
t = t0 * a + n0 * b;
dt = gradient(t);
n = dt ./ kappa;

n(:, 1) = n0
b = cross(t, n);

unit_t = t / norm(t);
unit_n = n / norm(n);
unit_b = b / norm(b);

f = figure(1);

integralfunction = @(length) t0 * ((1 ./ kappa) .* sin(kappa .* length)) + n0 * ((1 ./ kappa) .* (1 - cos(kappa .* length)));

p = integral(integralfunction, 0, s(end), 'ArrayValued', true);
azero = repmat(zeros(3,1), 1, 100);

quiver3(p(1, :), p(2, :), p(3, :), unit_t(1, :), unit_t(2, :), unit_t(3, :));
hold on;
quiver3(p(1, :), p(2, :), p(3, :), unit_n(1, :), unit_n(2, :), unit_n(3, :));
quiver3(p(1, :), p(2, :), p(3, :), unit_b(1, :), unit_b(2, :), unit_b(3, :));

hold off;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
