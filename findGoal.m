%We create block of 5m by 5m
function goal = findGoal(map, prev_goal, cell) %prev_goal [x,y];

mapLog = logical(map);

len_block = round(5/cell); % To put outsideif it works

y_block = ceil(prev_goal(2)/len_block);
x_block = ceil(prev_goal(1)/len_block);
%block = mapLog(sub2ind(size(mapLog), (1+(len_block*(y_block-1))):1:(len_block*y_block), (1+(len_block*(x_block-1))):1:(len_block*x_block)))
block = mapLog( (1+(len_block*(y_block-1))):(len_block*y_block) , (1+(len_block*(x_block-1))):(len_block*x_block));
%    if (any(block(:) )) % if there is still smth not seen in the current block
%        fprintf('IFFFFFFFFFFF=>still smth \n');

if sum(block(:)) > 100
    s = zeros(size(block));
    s(:,1) = block(:,1);
    s(1,:) = block(1,:);
    for i = 2:size(block)
        for j=2:size(block)
            if(block(i,j)== true)
                tmp = [s(i-1,j); s(i-1,j-1); s(i,j-1)];
                s(i,j) = min(tmp) + 1;
            end
        end
    end
    [max_s, goal] = max(s(:));
    [row_block,col_block] = ind2sub(size(block),goal);
    row_map = row_block + (y_block-1)*40;
    goal_y = ceil(row_map - max_s/2);
    col_map = col_block + (x_block-1)*40;
    goal_x = ceil(col_map - max_s/2);
    %goal = sub2ind(size(map),row_map,col_map);
    goal = [goal_x goal_y];
    
else % else, OK iff we start with th initial condition the first time of prev_goal with x = 1 and = 120 (right upper block)
    y_block = y_block - 1;
    if y_block < 1
        x_block = x_block + 1;
        y_block = 3;
    end
    if x_block < 4
        goal = findGoal(map, [1+(len_block*(x_block-1)), 1+(len_block*(y_block-1))], cell);
    else
        goal = [75 100]; 
    end
    %         block = mapLog( (1+(len_block*(y_block-1))):(len_block*y_block) , (1+(len_block*(x_block-1))):(len_block*x_block));
    %         s = zeros(size(block));
    %         s(:,1) = block(:,1);
    %         s(1,:) = block(1,:);
    %         for i = 2:size(block)
    %             for j=2:size(block)
    %                 if(block(i,j)== true)
    %                     tmp = [s(i-1,j); s(i-1,j-1); s(i,j-1)];
    %                     s(i,j) = min(tmp) + 1;
    %                 end
    %             end
    %         end
    %       [max_s, goal] = max(s(:));
end

% goal = ceil(ind_s-max_s/2);


end