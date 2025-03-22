
/*

Group 4
Marino Oliveros Blanco (1668563)
Alejandra Reinares Guerreros (1665499)
Andreu Gascón Marzo (1670919)
Pere Mayol Carbonell (1669503)

Implementation of a Tic-Tac-Toe player that just plays random moves.

When the agent is started it must first perform a 'sayHello' action.
Once all agents have done this, the game or tournament starts.

Each turn the agent will observe the following percepts:

- symbol(x) or symbol(o) 
	This indicates which symbol this agent should use to mark the cells. It will be the same in every turn.

- a number of marks:  e.g. mark(0,0,x) , mark(0,1,o) ,  mark(2,2,x)
  this indicates which cells have been marked with a 'x' or an 'o'. 
  Of course, in the first turn no cell will be marked yet, so there will be no such percept.

- round(Z)
	Indicates in which round of the game we are. 
	Since this will be changing each round, it can be used by the agent as a trigger to start the plan to determine
	its next move.

Furthermore, the agent may also observe the following:

- next 
	This means that it is this agent's turn.
  
- winner(x) or winner(o)
	If the game is over and did not end in a draw. Indicates which player won.
	
- end 
	If the game is finished.
	
- After observing 'end' the agent must perform the action 'confirmEnd'.

To mark a cell, use the 'play' action. For example if you perform the action play(1,1). 
Then the cell with coordinates (1,1) will be marked with your symbol. 
This action will fail if that cell is already marked.

*/


/* Initial beliefs and rules */


// First, define a 'cell' to be a pair of numbers, between 0 and 2. i.e. (0,0) , (0,1), (0,2) ... (2,2).

isCoordinate(0).
isCoordinate(1).
isCoordinate(2).

isCornerCoordinate(0).
isCornerCoordinate(2).

isCell(X,Y) :- isCoordinate(X) & isCoordinate(Y).
isCorner(X,Y) :- isCornerCoordinate(X) & isCornerCoordinate(Y).

/* A cell is 'available' if it does not contain a mark.*/
available(X,Y) :- isCell(X,Y) & not mark(X,Y,_).

/* This two lines are to place a mark everytime a winning move can be assured or the stopping of an opponents winning move can also be assured */
winningMove(X,Y) :- symbol(S) & available(X,Y) & twoInLine(X,Y,S). 
blockingMove(X,Y) :- symbol(S) & opponent(O) & available(X,Y) & twoInLine(X,Y,O).

/* We are trying to choose corners that block the most options for our opponent */
cornerToBlock(X,Y) :- isCorner(X,Y) & available(X,Y) & opponent(O) & createsTwoInLine(X,Y,O). 
createsTwoInLine(X,Y,S) :- mark(A,B,S) & isWinningLine(A,B,X,Y,C,D) & available(C,D).

twoInLine(X,Y,S) :- mark(A,B,S) & mark(C,D,S) & isWinningLine(A,B,C,D,X,Y). // Two marks in a line

isWinningLine(A,B,C,D,X,Y) :- A == C & X == A & not B == D. // Same column
isWinningLine(A,B,C,D,X,Y) :- B = D & Y = B & not A == C.  // Same row
isWinningLine(A,B,C,D,X,Y) :- A = B & C = D & X = Y & not A == C.  // Main diagonal
isWinningLine(A,B,C,D,X,Y) :- A + B = 2 & C + D = 2 & X + Y = 2 & not A == C.  // Anti-diagonal

opponent(o) :- symbol(x).
opponent(x) :- symbol(o).


started.

/* Plans - Hierarchy*/

/* When the agent is started, perform the 'sayHello' action. */
+started <- sayHello.

/*Whenever it's my turn, check if I can make a winning move*/
+round(Z) : next & winningMove(X,Y) <- play(X,Y).

/*Whenever it's my turn, check if I can block a winning move*/
+round(Z) : next & blockingMove(X,Y) <- play(X,Y).

/*Whenever it's my turn, if I can't do any of the above, play the center if available*/
+round(Z) : next & available(1,1) <- play(1,1).

/*Whenever it's my turn, if I can't do any of the above, play a corner that blocks the opposing player's line*/
+round(Z) : next & cornerToBlock(X,Y) <- play(X,Y).

/*Whenever it's my turn, if I can't do any of the above, play a corner*/
+round(Z) : next & isCorner(X,Y) & available(X,Y) <- play(X,Y).

/* Whenever it is my turn, if I can't do any of the above, I play a random move. Specifically:
	- find all available cells and put them in a list called AvailableCells.
	- Get the length L of that list.
	- pick a random integer N between 0 and L.
	- pick the N-th cell of the list, and store its coordinates in the variables A and B.
	- mark that cell by performing the action play(A,B).
*/
+round(Z) : next <- .findall(available(X,Y),available(X,Y),AvailableCells);
						L = .length(AvailableCells);
						N = math.floor(math.random(L));
						.nth(N,AvailableCells,available(A,B));
						 play(A,B).


						 
						 
/* If I am the winner, then print "I won!"  */
+winner(S) : symbol(S) <- .print("I won!").

+end <- confirmEnd.