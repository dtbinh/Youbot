%We create block of 5m by 5m
function goal = findGoal(map, prev_goal, cell) %prev_goal [x,y];
    
    mapLog = logical(map);
    
    len_block = round(5/cell); % To put outsideif it works
    
    y_block = ceil(prev_goal(2)/len_block);
    x_block = ceil(prev_goal(1)/len_block);
    %block = mapLog(sub2ind(size(mapLog), (1+(len_block*(y_block-1))):1:(len_block*y_block), (1+(len_block*(x_block-1))):1:(len_block*x_block)))
    block = mapLog( (1+(len_block*(y_block-1))):(len_block*y_block) , (1+(len_block*(x_block-1))):(len_block*x_block));
    if (any(block(:) )) % if there is still smth not seen in the current block 
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
        [max_s, ind_s] = max(s(:));
        
        goal = ceil(ind_s-max_s/2);
        [row_block,col_block] = ind2sub(size(block),goal)
        row_map = row_block + (y_block-1)*40 
        col_map = col_block + (x_block-1)*40
        goal = sub2ind(size(map),row_map,col_map);
        
    else % else 
        s = zeros(size(map));
        s(:,1) = mapLog(:,1);
        s(1,:) = mapLog(1,:);
        for i = 2:size(mapLog)
            for j=2:size(mapLog)
                if(mapLog(i,j)== true)
                    tmp = [s(i-1,j); s(i-1,j-1); s(i,j-1)];
                    s(i,j) = min(tmp) + 1;
                end
            end
        end
        [max_s, ind_s] = max(s(:));
        
        goal = ceil(ind_s-max_s/2);
    end
    
    
end