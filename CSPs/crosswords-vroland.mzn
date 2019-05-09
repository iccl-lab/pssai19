% Valentin Roland's Crossword Puzzle Encoding

set of int: INDEX3 = 1..3;
set of int: INDEX4 = 1..4;
set of int: INDEX5 = 1..5;

set of int: W3COUNT = 1..5; % # of words of length 3
set of int: W4COUNT = 1..5;
set of int: W5COUNT = 1..5;

enum Alphabet = {A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z};

% words of length 3,4,5
array[W3COUNT, INDEX3] of Alphabet: words3 = [|
A, F, T|
A, L, E|
E, E, L|
L, E, E|
T, I, E|];
array[W4COUNT, INDEX4] of Alphabet: words4 = [|
H, E, E, L|
H, I, K, E|
K, E, E, L|
K, N, O, T|
L, I, N, E|];
array[W5COUNT, INDEX5] of Alphabet: words5 = [|
H, O, S, E, S|
L, A, S, E, R|
S, A, I, L, S|
S, H, E, E, T|
S, T, E, E, R|];

% our actualy variables of interest
array[INDEX5] of var Alphabet: slot1;
array[INDEX5] of var Alphabet: slot2;
array[INDEX5] of var Alphabet: slot3;
array[INDEX4] of var Alphabet: slot4;
array[INDEX4] of var Alphabet: slot5;
array[INDEX3] of var Alphabet: slot6;
array[INDEX3] of var Alphabet: slot7;
array[INDEX5] of var Alphabet: slot8;

% slots of length 5
constraint exists(w in W5COUNT)(forall(i in INDEX5)(slot1[i]=words5[w, i]));
constraint exists(w in W5COUNT)(forall(i in INDEX5)(slot2[i]=words5[w, i]));
constraint exists(w in W5COUNT)(forall(i in INDEX5)(slot3[i]=words5[w, i]));
constraint exists(w in W5COUNT)(forall(i in INDEX5)(slot8[i]=words5[w, i]));

% slots of length 4
constraint exists(w in W4COUNT)(forall(i in INDEX4)(slot4[i]=words4[w, i]));
constraint exists(w in W4COUNT)(forall(i in INDEX4)(slot5[i]=words4[w, i]));

% slots of length 3
constraint exists(w in W3COUNT)(forall(i in INDEX3)(slot6[i]=words3[w, i]));
constraint exists(w in W3COUNT)(forall(i in INDEX3)(slot7[i]=words3[w, i]));

% crossword structure
constraint slot1[3] == slot2[1];
constraint slot1[5] == slot3[1];
constraint slot2[3] == slot4[2];
constraint slot2[4] == slot7[1];
constraint slot2[5] == slot8[3];
constraint slot3[3] == slot4[4];
constraint slot3[4] == slot7[3];
constraint slot3[5] == slot8[5];
constraint slot4[3] == slot5[1];
constraint slot5[2] == slot7[2];
constraint slot5[3] == slot8[4];
constraint slot6[2] == slot8[1];

solve satisfy;
