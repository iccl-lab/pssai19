% Rucksack Problem:
% Given a set of n items numbered 1..n, each with a weight w_i and value v_i,
% determine whether or not to include an item in a collection so that the total weight W
% is less than or equal to a given limit Wmax and the total value V is as large as possible.

%% ENCODING %%

% number of items
int: n = 5;
% max weight of the rucksack
int: Wmax = 15;

% an array the weight and value of each item
array[1..n] of int: weights = [1,12,2,1,4];
array[1..n] of int: values  = [2,4,2,1,10];

% an array of variables, indicating which item is in the rucksack (= 1), or not (= 0)
array[1..n] of var 0..1: items;

% 1. hard constraint: do not exceed the weight limit Wmax
constraint sum(i in 1..n where items[i] = 1)(weights[i]) <= Wmax;

% solve and soft constraint: maximize the value of the items in the rucksack
solve maximize sum(i in 1..n where items[i] = 1)(values[i]);