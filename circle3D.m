% http://mathematica.stackexchange.com/questions/16209/how-to-determine-the-center-and-radius-of-a-circle-given-some-points-in-3d

close all;
%x1 = [1, 3, 5];
%x2 = [2,4,6];
%x3 = [5, 3,7];
x1 = [3, 0, 0];
x2 = [0,3,0];
x3 = [3/sqrt(2), 3/sqrt(2),0];
v1 = x2 - x1;
v1 = v1/norm(v1);
v2 = x3 - x1;
v2 = v2/norm(v2);

n = cross(v1,v2); % n perpendicular to v1 and v2
n = n/norm(n);
syms x y z r

S = solve([(x - x1(1))^2 + (y - x1(2))^2 + (z - x1(3))^2 == r^2, ...
    (x - x2(1))^2 + (y - x2(2))^2 + (z - x2(3))^2 == r^2, ...
    (x - x3(1))^2 + (y - x3(2))^2 + (z - x3(3))^2 == r^2 ], ...
    dot(n,([x,y,z]-x1)), ...
    [x, y, z, r]);

S.x
S.y
S.z
S.r


center = double([S.x(2), S.y(2), S.z(2)]); % center of the circle
normal = n; % axe of the cylinder
radius = double(S.r(2)); % radius of the cylinder

% Creation of the circle
theta=0:0.01:2*pi;
v=null(normal);
points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));

figure;
plot3(x1(1),x1(2),x1(3),'b*');
hold on;
plot3(x2(1),x2(2),x2(3),'b*');
plot3(x3(1),x3(2),x3(3),'b*');
plot3(S.x(2),S.y(2),S.z(2),'g*');
plot3(points(1,:),points(2,:),points(3,:),'r-'); % plot the circle

quiver3(S.x(2),S.y(2),S.z(2),n(1),n(2),n(3)); % plot the vector of ref of the cylinder
hold off;

% http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
pt1dr = center;
pt2dr = center + normal;

pt_test = [18.78, 0, 3];

t = - dot((pt1dr-pt_test),(pt2dr-pt_test))/(norm(pt2dr-pt1dr)^2);

tmp_cross = cross((pt_test-pt1dr),(pt_test-pt2dr));

d = norm(tmp_cross)/norm(pt2dr-pt1dr)

