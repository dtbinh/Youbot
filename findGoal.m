function goal = findGoal(map)
    
    mapLog = logical(map);
    s = zeros(size(map));
    s(:,1) = mapLog(:,1);
    s(1,:) = mapLog(1,:);
    for i = 2:size(mapLog)
        for j=2:size(mapLog)
            if(map(i,j)== true)
                tmp = [s(i-1,j); s(i-1,j-1); s(i,j-1)];
                s(i,j) = min(tmp) + 1;
            end
        end
    end
    [max_s, ind_s] = max(s(:));
    
    goal = ceil(ind_s-max_s/2);
    
end