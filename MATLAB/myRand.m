function [a, s] = myRand(Low,High)
a = Low+ rand(3,4)*(High-Low);
v = a(:);
s = sum(v);
end