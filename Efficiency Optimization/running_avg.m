function avg = running_avg(x)
% RUNNING_avg computes running average of input x
% Call format: avg = running_avg(x)

persistent sums counts

if isempty(sums)
    sums = 0;
    counts = 0;
end
whos global
sums = sums+x;
counts = counts+1; %keeps track of how many times a number is coming in
avg = sums/counts; % running avg
end