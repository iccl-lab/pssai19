% Crossword Puzzle
% See https://iccl.inf.tu-dresden.de/w/images/8/84/PSSAI2019-T1.pdf for the problem instance.

% This is the most trivial encoding (a bit of cheating involved)

include "globals.mzn";        % use globals library

% the words that can be used
enum Len5 = {HOSES, LASER, SAILS, SHEET, STEER};
enum Len4 = {HEEL, HIKE, KEEL, KNOT, LINE};
enum Len3 = {AFT, ALE, EEL, LEE, TIE};

% now 8 variables that can take a word from Words, according to the numbers in the puzzle
var Len5: w1;
var Len5: w2;
var Len5: w3;
var Len4: w4;
var Len4: w5;
var Len3: w6;
var Len3: w7;
var Len5: w8;

% theses constraints define the pairs of compatible words
% C_1,2 = { (HOSES,SAILS), (HOSES,SHEET), (HOSES,STEER) ...}
constraint ((w1 = HOSES \/ w1 = LASER) /\ (w2 = SAILS \/ w2 = SHEET \/ w2 = STEER));
% C_1,3 = {(HOSES,SAILS), ...}
constraint w1 = HOSES /\ (w3 = SAILS \/ w3 = SHEET \/ w3 = STEER);
% C_2,1
constraint (w2 = SAILS \/ w2 = SHEET \/ w2 = STEER) /\ (w1 = HOSES \/ w1 = LASER);
% C_2,8 = {(SAILS,HOSES), (SAILS,LASER), (HOSES,LASER)} 
constraint (w2 = SAILS /\ (w8 = HOSES \/ w8 = LASER)) \/ (w2 = HOSES /\ (w8 = LASER));
% C_2,7 = {(HOSES,EEL), ..., (LASER,EEL), (SAILS,LEE)} 
constraint (w2 = HOSES \/ w2 = SHEET \/ w2 = STEER \/ w2 = LASER /\ w7 = EEL) \/ (w2 = SAILS /\ w7 = LEE);
% ...
constraint (w3 = SAILS /\ w8 = HOSES) \/ (w3 = STEER /\ w8 = LASER);
constraint w8 = LASER /\ w6 = ALE;
% C_4,2 = {(HIKE,SAILS)}
constraint (w4 = HIKE /\ w2 = SAILS);
constraint (w5 = HEEL \/ w5 = KEEL) /\ (w8 = LASER \/ w8 = HOSES \/ w8 = SHEET \/ w8 = STEER);

% words can only used once
constraint w1 != w2;
constraint w1 != w3;
constraint w1 != w8;
constraint w2 != w3;
constraint w2 != w8;
constraint w3 != w8;

constraint w4 != w5;
constraint w6 != w7;

% just solve
solve satisfy;