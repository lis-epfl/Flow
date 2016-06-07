function [ d_est ] = find_best( bins, acc, option)
%FIND_BEST Find bin with highest number of votes and returns the
% corresponding direction
%   bins:   voting bins
%   acc:    accumulator
%   option: 1=raw estimate / 2=averaged

best = 0;	% highest accumulated value
best_n = 0;	% number of indices with same accumulated value

for i = 1:length(acc),
    if acc(i) > best,
        best = acc(i);
        best_n = 1;
    elseif acc(i) == best,
        best_n = best_n + 1;
    end
end

best_x = 0;
best_y = 0;
best_z = 0;

cnt = 0;
for j = 1:length(acc),
    if(acc(j)==best)
        if (option == 2),
        best_x = best_x + bins(j,1)/best_n;
        best_y = best_y + bins(j,2)/best_n;
        best_z = best_z + bins(j,3)/best_n;
        end
        if (option == 1),
        best_x = bins(j,1);
        best_y = bins(j,2);
        best_z = bins(j,3);
        end
        cnt = cnt + 1;
    end
    if(cnt == best_n),
        break;
    end
end

d_est = [best_x, best_y, best_z];
d_est = normr(d_est);

end

