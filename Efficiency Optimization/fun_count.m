function fun_count
% FUN_COUNT increments variable persistent variable count
% to track the number of times this function is called.
% Call format: fun_count

persistent counts

if isempty(counts)
    counts = 0;
end
counts = counts+1;
fprintf('The value of count from fun_count is %d \n',counts)
end