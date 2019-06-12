% A simple test graph
node(a;b;c;d).
edge(a,b).
edge(b,c).
edge(c,d).
edge(d,a).
% symmetry of edges
edge(X,Y) :- edge(Y,X).

% the actual encoding
% pick exactly one edge ending in node X
{matching(Y,X) : node(Y), edge(Y,X)} = 1 :- node(X).

% rule out that a node might have an incoming and outgoing matching
:- matching(X1,Y), matching(Y,X2), X1!=X2.

% display only the matching predicate
#show matching/2.
