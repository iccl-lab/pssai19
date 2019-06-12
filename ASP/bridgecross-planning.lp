% Simple encoding of the bridge-crossing planning problem (see exercise sheet 2).

% t_max is the maximun lenght of our planning sequence
#const t_max = 5.
% use time steps 0..t_max-1
time(0..t_max-1).

% the presons
person(a;b;c;d).

% we represent the state with left and right predicates.
% left(P,T): person P is on the left side at time T
% initially a..d are on the left
left(a,0).
left(b,0).
left(c,0).
left(d,0).

% ttc = "time to cross" the bridge for the persons
ttc(a,1).
ttc(b,2).
ttc(c,5).
ttc(d,10).

% the state we want to reach: at some time T, a..d are on the right.
goal_state(T) :- right(a,T), right(b,T), right(c,T), right(d,T), T<=t_max.

% for the initial time point 0, we guess which two persons go right
1 { move_right(X,Y,T+1) : left(X,T), left(Y,T), X!=Y, T=0 } 1.
% for later timesteps, we guess only if the time limit is not reached, and someone has moved left before
1 { move_right(X,Y,T+1) : left(X,T), left(Y,T), X!=Y, time(T), T>0 } 1 :- move_left(P,T), T<t_max.

% to make it simpler
move_right(P,T) :- move_right(P,_,T).
move_right(P,T) :- move_right(_,P,T).

% take care of the state
% if some is chosen to move right, change his state to right
right(P,T) :- move_right(P,T).
% people on the left remain there if they are not selected to move
left(P,T) :- left(P,T-1), move_right(X,Y,T), X!=P, Y!=P, person(P).
% people on the right remain there if they are not selected to move 
right(P,T) :- right(P,T-1), move_right(X,Y,T), X!=P, Y!=P, person(P).

% if two people crossed the bridge at time T, one goes back at time T+1, unless we reached the goal state
1 { move_left(P,T+1) : right(P,T), time(T) } 1 :- move_right(X,Y,T), T<t_max, not goal_state(T).

% change the states for left movers
left(P,T) :- move_left(P,T).
% keep the state for peolpe not moving
right(P,T)  :- move_left(X,T), X!=P, person(P), right(P,T-1).
left(P,T) :- move_left(X,T), X!=P, person(P), left(P,T-1).

% goal state criterion, we fail if we do not reach the goal state
% at some time point
goal_reached :- goal_state(T), T=1..t_max.
:- not goal_reached.

% OPTIMIZATION
% Now the optimisation part, where we simply add the costs of the chosen movements
cost(C1+C2+C,T) :- cost(C,T-1), move_right(X,Y,T), ttc(X,C1), ttc(Y,C2).
cost(C1+C,T) :- cost(C,T-1), move_left(X,T), ttc(X,C1).
total_cost(C) :- cost(C,T), goal_state(T).

% optimise such that total cost is minimal
#minimize{ C : total_cost(C) }.

% we are only intereset in displaying those predicates
#show total_cost/1.
#show goal_reached/0.
#show goal_state/1.
#show move_right/3.
#show move_left/2.
#show left/2.
#show right/2.
