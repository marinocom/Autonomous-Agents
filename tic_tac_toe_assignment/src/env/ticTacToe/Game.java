package ticTacToe;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import jason.asSyntax.Structure;

public class Game {

	
    String[][] board = new String[3][3];
    int legalActionsPlayed = 0;
    
    List<String> players;
    List<String> symbols = Arrays.asList("x","o");
    
    String symbolOfNextPlayer = null;
    String nameOfNextPlayer = null;
   
    String symbolOfWinner = null;
    String nameOfWinner = null;
    
    boolean finished = false;
    
    List<String> playersToConfirmEnd = new ArrayList<>();
    
    Game(String x_player, String o_player){
    	
    	this.players = Arrays.asList(x_player,o_player);
    	this.playersToConfirmEnd.addAll(this.players);
    	
    	this.symbolOfNextPlayer = symbols.get(0);
    	this.nameOfNextPlayer = players.get(0);
    }
    
    
    
    public boolean playMove(String agentName, int x, int y) {
    	
    	
    	if(board[x][y] != null) {
    		return false;
    	}
    	
    	int agentIndex = players.indexOf(agentName);
    	
    	board[x][y] = symbols.get(agentIndex);
    	
    	
		this.legalActionsPlayed++;
		
		this.symbolOfWinner = getSymbolOfWinner();
		
		if(this.symbolOfWinner != null) {
			
			int winningPlayerIndex = symbols.indexOf(this.symbolOfWinner);
			this.nameOfWinner = players.get(winningPlayerIndex);
			
			
			this.nameOfNextPlayer = null;
			this.symbolOfNextPlayer = null;
			this.finished = true;
		
		}else if(this.legalActionsPlayed == 9) {
			
			this.nameOfNextPlayer = null;
			this.symbolOfNextPlayer = null;
			this.finished = true;
		
		}else {
			
			int indexOfNextPlayer = legalActionsPlayed % 2;
			this.nameOfNextPlayer = players.get(indexOfNextPlayer);
			this.symbolOfNextPlayer = symbols.get(indexOfNextPlayer);
		}
		
		
		return true;
    }
    
    
    /** Returns the symbol of the winner (i.e. "X" or "O", if any. Returns null if there is no winner (yet).*/
    String getSymbolOfWinner() {
    	
    	String winner;
    	
    	for(int i=0; i<3; i++) {
    		
    		winner = checkRow(i);
    		if(winner != null) {
    			return winner;
    		}
    		
    		winner = checkColumn(i);
    		if(winner != null) {
    			return winner;
    		}
    	}
    	
    	winner = checkDiagonals();
    	
    	return winner;
    	
    }
    
    String checkRow(int y) {
    	return areAllEqual(board[0][y], board[1][y], board[2][y]);
    }
    
    String checkColumn(int x) {
    	return areAllEqual(board[x][0], board[x][1], board[x][2]);
    }
    
    String checkDiagonals() {
    	
    	if(areAllEqual(board[0][0], board[1][1], board[2][2]) != null) {
    		return board[1][1];
    	}
    	
    	if(areAllEqual(board[0][2], board[1][1], board[2][0]) != null) {
    		return board[1][1];
    	}
    	
    	return null;
    }
    
    
    /**
     * If all given strings are the same, returns that string. Otherwise, returns null.
     * @param s1
     * @param s2
     * @param s3
     * @return
     */
    String areAllEqual(String s1, String s2, String s3) {
    	
    	if(s1 == null) {
    		return null;
    	}
    	
    	if(s1.equals(s2) && s1.equals(s3)) {
    		return s1;
    	}
    	
    	return null;
    }
    
    
    public String getBoardString() {
    	
    	StringBuilder sb = new StringBuilder();
		
    	sb.append(System.lineSeparator());
    	for(int i=0; i<3; i++) {
        	for(int j=0; j<3; j++) {
        		
        		if(board[i][j] == null) {
        			sb.append("_");
        		}else {
        			sb.append(board[i][j]);
        		}
        	}
        	sb.append(System.lineSeparator());
    	}
    	
		String s = sb.toString();
    	
		return s;
    }
    
    void confirmEnd(String playerName) {
    	this.playersToConfirmEnd.remove(playerName);
    }
}
