path = [
    76   102
    77   101
    78   100
    79    99
    80    98
    81    97
    82    96
    83    95
    84    94
    85    93
    85    92
    85    91
    85    90
    85    89
    85    88
    85    87
    85    86
    86    85
    87    84
    88    83
    89    82
    90    81
    91    80
    92    79
    93    78
    94    77
    95    76
    96    75
    97    74
    98    73
    99    72
   100    71
   100    70
   100    69
   100    68
   100    67
   100    66
   100    65
   100    64
   100    63
   100    62
   100    61
   100    60
   100    59
   100    58
   100    57
   100    56
   100    55
   100    54
   100    53
   100    52
   100    51
   100    50
   100    49
   100    48
   100    47
   100    46
   100    45
   100    44
   100    43
   100    42
   100    41
   100    40];

via_prev = [85    93
    85    86
   100    71
   100    40
]
cell = 0.125;
step_path = ceil(2/cell);
angles = (path(2:end,2)-path(1:end-1,2))./(path(2:end,1)-path(1:end-1,1));
ind_angles = [false(1); angles(2:end) ~= angles(1:end-1)];
ind_angles = find(ind_angles == 1);
true_index_via = [];
for i = 1:length(ind_angles)-1
    true_index_via = [true_index_via, ind_angles(i):step_path:(ind_angles(i+1)-1)];
end
true_index_via = [true_index_via, ind_angles(end)];
true_index_via = [ step_path:step_path:true_index_via(1), true_index_via];

via = path(true_index_via(:),:);
via = [via; path(end,:)]

% 
% figure
% plot(path(:,1), path(:,2),'.g');
% hold on;
% plot(via(:,1), via(:,2),'.r');




%ind = [5 10 12 18 25]
% delta_ind = ind(2:end) - ind(1:end-1)
% nb_add = floor((delta_ind(:)-0.01)/step_path)
% step_path = 3;
% j = 1;
% for i = 1:length(nb_add)
%     ind = [ind(1:j), ind(j)+(step_path:step_path:(nb_add(i)*3)), ind(j+1:end)]
%     j = i + nb_add(i) + 1;
% end
% true_via = [];
% step_path = 3;
% for i = 1:length(ind)-1
%    true_via = [true_via, (ind(i):step_path:(ind(i+1)-1))];
% end
% true_via = [true_via, ind(end)];
% true_via