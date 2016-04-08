tic
close all;


map_cost_seen = false(size(map)); 
map_cost = flip(map);
map_cost(map_cost == 1) = Inf;
%map_cost(map_cost == 1) = 0 ;
%G = [3,3];
%G = sub2ind(size(map), 35, 18);
G = sub2ind(size(map), 18, 21);
map_cost(G) = 0;
stack = [];
stack = [stack G];
ind_end = sub2ind(size(map), size(map,1), size(map,2));
%while any(~map_cost_seen(:))
%    if isempty(stack)
%        break;
%    end
%cnt = 0;

while ~isempty(stack)
    %cnt = cnt + 1
    curr_point = stack(1);
    stack(1) = [];
    % UP
    up = curr_point-1;
    min_tmp = min(up,ind_end);
    up = max(min_tmp, 1);
    map_cost_seen(up) = true;
    if (map_cost(up) == Inf)
        continue;
    elseif ( map_cost(up) == 0 || (map_cost(curr_point) + 1) < map_cost(up) )
        map_cost(up) = (map_cost(curr_point) + 1);
        stack = [stack up];
    end
    
    % DOWN
    down = curr_point+1;
    min_tmp = min(down,ind_end);
    down = max(min_tmp, 1);
    map_cost_seen(down) = true;
    if (map_cost(down) == Inf)
        continue;
    elseif ( map_cost(down) == 0 || (map_cost(curr_point) + 1) < map_cost(down) )
        map_cost(down) = (map_cost(curr_point) + 1);
        stack = [stack down];
    end
    
    % LEFT
    left = curr_point - size(map,1);
    min_tmp = min(left,ind_end);
    left = max(min_tmp, 1);
    map_cost_seen(left) = true;
    if (map_cost(left) == Inf)
        continue;
    elseif ( map_cost(left) == 0 || (map_cost(curr_point) + 1) < map_cost(left) )
        map_cost(left) = (map_cost(curr_point) + 1);
        stack = [stack left];
    end
    
    % RIGHT
    right = curr_point + size(map,1);
    min_tmp = min(right,ind_end);
    right = max(min_tmp, 1);
    map_cost_seen(right) = true;
    if (map_cost(right) == Inf)
        continue;
    elseif ( map_cost(right) == 0 || (map_cost(curr_point) + 1) < map_cost(right) )
        map_cost(right) = (map_cost(curr_point) + 1);
        stack = [stack right];
    end
    
    % UP RIGHT
    up_right = curr_point + (size(map,1)-1);
    min_tmp = min(up_right,ind_end);
    up_right = max(min_tmp, 1);
    map_cost_seen(up_right) = true;
    if (map_cost(up_right) == Inf)
        continue;
    elseif ( map_cost(up_right) == 0 || (map_cost(curr_point) + 1.42) < map_cost(up_right) )
        map_cost(up_right) = (map_cost(curr_point) + 1.42);
        stack = [stack up_right];
    end
    
    % DOWN RIGHT
    down_right = curr_point + (size(map,1)+1);
    min_tmp = min(down_right,ind_end);
    down_right = max(min_tmp, 1);
    map_cost_seen(down_right) = true;
    if (map_cost(down_right) == Inf)
        continue;
    elseif ( map_cost(down_right) == 0 || (map_cost(curr_point) + 1.42) < map_cost(down_right) )
        map_cost(down_right) = (map_cost(curr_point) + 1.42);
        stack = [stack down_right];
    end
    
    % UP LEFT
    up_left = curr_point - size(map,1) -1;
    min_tmp = min(up_left,ind_end);
    up_left = max(min_tmp, 1);
    map_cost_seen(up_left) = true;
    if (map_cost(up_left) == Inf)
        continue;
    elseif ( map_cost(up_left) == 0 || (map_cost(curr_point) + 1.42) < map_cost(up_left) )
        map_cost(up_left) = (map_cost(curr_point) + 1.42);
        stack = [stack up_left];
    end
    
    % DOWN LEFT
    down_left = curr_point - size(map,1) + 1;
    min_tmp = min(down_left,ind_end);
    down_left = max(min_tmp, 1);
    map_cost_seen(down_left) = true;
    if (map_cost(down_left) == Inf)
        continue;
    elseif ( map_cost(down_left) == 0 || (map_cost(curr_point) + 1.42) < map_cost(down_left) )
        map_cost(down_left) = (map_cost(curr_point) + 1.42);
        stack = [stack down_left];
    end
end
map_cost(G) = 0;


%S = sub2ind(size(map), 6, 5);
S = sub2ind(size(map), 105, 105);
curr_point = S;
path = [S];
while true,
    % UP
    up = curr_point-1;
    min_tmp = min(up,ind_end);
    up = max(min_tmp, 1);
    % DOWN
    down = curr_point+1;
    min_tmp = min(down,ind_end);
    down = max(min_tmp, 1);
    % LEFT
    left = curr_point - size(map,1);
    min_tmp = min(left,ind_end);
    left = max(min_tmp, 1);
    % RIGHT
    right = curr_point + size(map,1);
    min_tmp = min(right,ind_end);
    right = max(min_tmp, 1);
    % UP RIGHT
    up_right = curr_point + (size(map,1)-1);
    min_tmp = min(up_right,ind_end);
    up_right = max(min_tmp, 1);
    % DOWN RIGHT
    down_right = curr_point + (size(map,1)+1);
    min_tmp = min(down_right,ind_end);
    down_right = max(min_tmp, 1);
    % UP LEFT
    up_left = curr_point - size(map,1) -1;
    min_tmp = min(up_left,ind_end);
    up_left = max(min_tmp, 1);
    % DOWN LEFT
    down_left = curr_point - size(map,1) + 1;
    min_tmp = min(down_left,ind_end);
    down_left = max(min_tmp, 1);
    
    pts = [up, down, left, right, up_right, down_right, up_left, down_left];
    [~,i]  = min(map_cost(pts));
    path = [path , pts(i)];
    curr_point = pts(i); 
    if pts(i) == G
        break;
    end
end

[y,x] = ind2sub(size(map),path);
path = [y.',x.'];
figure;
imshow(flip(map));
hold on;
plot(x,y,'.g');
toc