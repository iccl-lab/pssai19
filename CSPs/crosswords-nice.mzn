% Crossword Puzzle

% definition of the alphabet
enum ABC = {A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z};

% definition of the possible words as arrays
array[1..5] of ABC: HOSES = [H,O,S,E,S];
array[1..5] of ABC: SAILS = [S,A,I,L,S];
array[1..5] of ABC: STEER = [S,T,E,E,R];
array[1..5] of ABC: SHEET = [S,H,E,E,T];
array[1..5] of ABC: LASER = [L,A,S,E,R];


array[1..4] of ABC: HIKE = [H,I,K,E];
array[1..4] of ABC: KEEL = [K,E,E,L];
array[1..4] of ABC: HEEL = [H,E,E,L];
array[1..4] of ABC: KNOT = [K,N,O,T];
array[1..4] of ABC: LINE = [L,I,N,E];

array[1..3] of ABC: TIE = [T,I,E];
array[1..3] of ABC: LEE = [L,E,E];
array[1..3] of ABC: AFT = [A,F,T];
array[1..3] of ABC: ALE = [A,L,E];
array[1..3] of ABC: EEL = [E,E,L];

% 8 variables, each for a possible word in the puzzle
array[1..5] of var ABC: w1;
array[1..5] of var ABC: w2;
array[1..5] of var ABC: w3;
array[1..4] of var ABC: w4;
array[1..4] of var ABC: w5;
array[1..3] of var ABC: w6;
array[1..3] of var ABC: w7;
array[1..5] of var ABC: w8;

% 1: constraints for the crossings in the puzzle
constraint w1[3] = w2[1];
constraint w1[5] = w3[1];
constraint w2[5] = w8[3];
constraint w2[3] = w4[2];
constraint w4[3] = w5[1];
constraint w2[4] = w7[1];
constraint w3[5] = w8[5];
constraint w4[4] = w3[3];
constraint w5[2] = w7[2];
constraint w5[3] = w8[4];
constraint w6[2] = w8[1];

% 2: actually defining the possible words for each variable (but just based on the length)
% i.e. variable 1 can take all words of length 5 etc...
constraint w1 = HOSES \/ w1 = SAILS \/ w1 = STEER \/ w1 = SHEET \/ w1 = LASER;
constraint w2 = HOSES \/ w2 = SAILS \/ w2 = STEER \/ w2 = SHEET \/ w2 = LASER;
constraint w3 = HOSES \/ w3 = SAILS \/ w3 = STEER \/ w3 = SHEET \/ w3 = LASER;
constraint w4 = HIKE \/ w4 = KEEL \/ w4 = HEEL \/ w4 = KNOT \/ w4 = LINE;
constraint w5 = HIKE \/ w5 = KEEL \/ w5 = HEEL \/ w5 = KNOT \/ w5 = LINE;
constraint w6 = TIE \/ w6 = LEE \/ w6 = AFT \/ w6 = ALE \/ w6 = EEL;
constraint w7 = TIE \/ w7 = LEE \/ w7 = AFT \/ w7 = ALE \/ w7 = EEL;
constraint w8 = HOSES \/ w8 = SAILS \/ w8 = STEER \/ w8 = SHEET \/ w8 = LASER;

% we actually do not need the constraints that a word can only used once....but it wouldn't harm ;)

solve satisfy;
