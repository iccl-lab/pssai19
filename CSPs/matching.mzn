% Graph Matching:
% A Matching is a set of edges from the input graph, such that every node ist the endpoint of exactly one edge.

% We enocde the graph(s) adjacent matrix
% An example input graph
array[1..10,1..10] of 0..1: input_graph = [|0,1,1,0,0,0,0,0,0,1|
                                             1,0,1,0,0,0,0,0,0,0|
                                             1,1,0,1,0,0,0,0,0,1|
                                             0,0,1,0,1,0,0,0,0,1|
                                             0,0,0,1,0,1,0,1,1,0|
                                             0,0,0,0,1,0,1,0,0,0|
                                             0,0,0,0,0,1,0,1,0,0|
                                              0,0,0,0,1,0,1,0,1,0|
                                             0,0,0,0,1,0,0,1,0,1|
                                             1,0,1,1,0,0,0,0,1,0|];
                                             
% another array of the same size to indicate which edges belong to the matching
array[1..10,1..10] of var 0..1: matching;

% for each node, either the column sum is 1 and the row sum is 0, or vice versa
constraint forall(i in 1..10) (
   (sum(j in 1..10)(matching[j,i]) = 1 /\ sum(j in 1..10)(matching[i,j])=0) \/ (sum(j in 1..10)(matching[j,i])=0 /\ sum(j in 1..10)(matching[i,j])=1)
);

% there can only be an edge in the matching if this edge exists also in the input graph
constraint forall(i,j in 1..10) (
    if matching[i,j] = 1 then input_graph[i,j]=1 else matching[i,j]=0 endif
   );

solve satisfy;

% pretty print
output 
[show(matching[1,i]) | i in 1..10] ++ ["\n"]  ++
[show(matching[2,i]) | i in 1..10] ++ ["\n"]  ++
[show(matching[3,i]) | i in 1..10] ++ ["\n"] ++
[show(matching[4,i]) | i in 1..10] ++ ["\n"]  ++
[show(matching[5,i]) | i in 1..10] ++ ["\n"] ++
[show(matching[6,i]) | i in 1..10] ++ ["\n"]  ++
[show(matching[7,i]) | i in 1..10] ++["\n"]  ++
[show(matching[8,i]) | i in 1..10] ++ ["\n"] ++
[show(matching[9,i]) | i in 1..10] ++["\n"] ++
[show(matching[10,i]) | i in 1..10];