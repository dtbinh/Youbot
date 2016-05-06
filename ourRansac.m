function [center,radius] = ourRansac(pts)
    nb_points = size(pts,2);
    omega = (1/10);
    n = 3;
    z = 0.025;
    nb_iteration = log10(z)/log10(1-(omega^n));

    pts_cylinder = [];

    figure;
    plot3(pts(1,:), pts(2,:), pts(3,:), 'k*');
    close all;

    for i = 1:nb_iteration
        i
        close all;
        idx = randperm(size(pts,2));
        pts_perm=pts(:,idx);

        x1 = pts_perm(:,1).';
        x2 = pts_perm(:,2).';
        x3 = pts_perm(:,3).';
        figure;
        plot3(x1, x2, x3, 'k*');
        hold on;

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
        
        if size(S.x,1) == 0 % if the 3 points were have 2 of the 3 coordinates the same for any reasons, imp to find the circle => size of x || y || z || r = 0
            continue
        end

        center = double([S.x(2), S.y(2), S.z(2)]); % center of the circle
        normal = n; % axe of the cylinder
        radius = double(S.r(2)) % radius of the cylinder
        % Creation of the circle
        theta=0:0.01:2*pi;
        v=null(normal);
        points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));

        plot3(x1(1),x1(2),x1(3),'g*');
        plot3(x2(1),x2(2),x2(3),'g*');
        plot3(x3(1),x3(2),x3(3),'g*');
        plot3(S.x(2),S.y(2),S.z(2),'b*');
        plot3(points(1,:),points(2,:),points(3,:),'r-'); % plot the circle

        quiver3(S.x(2),S.y(2),S.z(2),n(1),n(2),n(3)); % plot the vector of ref of the cylinder
        xlabel('x');
        ylabel('y');
        zlabel('z');
        hold off;

        % theoritical radius = 0.05m

        if radius < 0.03 || radius > 0.07  || abs(normal(2)) < 0.7
            continue
        end

        a = 5;

        radius

        % http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        pt1dr = center;
        pt2dr = center + normal;

        for j = 1:size(pts,2)-3
            pt_test = pts_perm(:,j).';
            t = - dot((pt1dr-pt_test),(pt2dr-pt_test))/(norm(pt2dr-pt1dr)^2);

            tmp_cross = cross((pt_test-pt1dr),(pt_test-pt2dr));

            d = norm(tmp_cross)/norm(pt2dr-pt1dr);
            if d > 0.03 && d < 0.07
                pts_cylinder = [pts_cylinder;pt_test];
            end

        end

        if size(pts_cylinder,1) >= omega*nb_points
            pts_cylinder = pts_cylinder.';
            hold on;
            fprintf('I think I have found a good cylinder \n');
            plot3(pts_cylinder(1,:),pts_cylinder(2,:),pts_cylinder(3,:),'r*');
            hold off;
            figure;
            %[~,tmp] = max(pts_cylinder(2,:));
            %center = pts_cylinder(:,tmp(1))
            break;
        end

    end
end