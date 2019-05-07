% Subset Sum:
% Given a set (or multiset) of integers, is there a non-empty subset whose sum is zero?

%% ENCODING %%

% input "set" of numbers ...though we can use an array to have multiple occurences of numbers
int: n = 5;
array[1..n] of int: numbers = [-7, -3, -2, 5, 8]; 

% an array of variables indicating whether the corresponding numbers[i] is in the subset or not
array[1..n] of var 0..1: subsetsum;

% do the numbers sum up to zero? i.e. the numbers on the array numbers for which there is a "1" in the subsetsum array
constraint sum(i in 1..n where subsetsum[i]=1)(numbers[i]) = 0;

% the subset must be non-empty! 
constraint sum(i in 1..n)(subsetsum[i]) > 0;

solve satisfy;

% NOTE: we coul also turn this into an optimization problem by asking for the largest subset that sums up to zero.