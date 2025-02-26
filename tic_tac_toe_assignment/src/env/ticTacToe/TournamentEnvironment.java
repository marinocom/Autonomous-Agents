package ticTacToe;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

import jason.asSyntax.ASSyntax;
import jason.asSyntax.Literal;
import jason.asSyntax.Structure;
import jason.asSyntax.parser.ParseException;
import jason.asSyntax.parser.TokenMgrError;
import jason.environment.Environment;

public class TournamentEnvironment extends Environment{
	
	
	private Logger logger = Logger.getLogger("tic_tac_toe."+TournamentEnvironment.class.getName());
	
	int numExpectedAgents = -1;
	
	/** How often each pair of agents will be playing against each other.*/
	int numGamesPerPair = 1;
	int totalNumGames;
	
	/** The names of all the agents in the tournament.*/
	List<String> agents = new ArrayList<String>();
	List<Game> games = new ArrayList<Game>();
	Game currentGame;
	
	/** The index (in the list of agents) of the next agent to play x */
	int indexOfNextXplayer = 0;
	
	/** The index (in the list of agents) of the next agent to play o */
	int indexOfNextOplayer = 1;


	
	@Override
	public void init(String[] args) {

		this.numExpectedAgents = Integer.parseInt(args[0]);
		
		if(args.length == 2) {
			this.numGamesPerPair = Integer.parseInt(args[1]);
		}
		
		this.totalNumGames = numGamesPerPair * numExpectedAgents * (numExpectedAgents-1);
		
	}
	
    @Override
    public boolean executeAction(String agName, Structure action) {
    	
    	if(agents.size() > numExpectedAgents) {
    		throw new RuntimeException("Error! There are more agents in the environment than specified in the mas2j file. Number of agents: " + agents.size() + ". Expected number of agents: " + numExpectedAgents + " Please update the mas2j file with the correct number of agents.");
    	}

    	
    	logger.info("Action performed: " + agName + " " + action);
    	
    	/* At the start of the tournament, each agent should perform the 'sayHello' action, so that 
    	 * the environment knows which agents are present.
    	 * TODO: find a better solution for this.
    	 * */
    	if(action.getFunctor().equals("sayHello")) {
    		return handleHello(agName, action);
    	}
    	
    	/* The agent is playing a move.*/
    	if( action.getFunctor().equals("play")) {
    		return handlePlay(agName, action);
    	}
    	
    	/* At the end of each game, the two agents who were playing in that game need to perform the 'confirmEnd' action.
    	 * This is in order for the environment to know when it can start a new game.
    	 * TODO: find a better solution for this.
    	 * */
    	if( action.getFunctor().equals("confirmEnd")) {
    		return handleConfirmEnd(agName, action);
    	}
    	
    	
		logger.info("Unknown action: " + action);
		return false;

    }
    


	boolean handleHello(String agName, Structure action) {
    	
		if(this.agents.contains(agName)) {
			logger.info("Error! agent " + agName + " already logged in.");
			return false;
		}

		this.agents.add(agName);
		
		
    	if(this.agents.size() == numExpectedAgents) {
    		
    		logger.info("All expected players have logged in! " + this.agents );
    		startNewGame();
    	}
    	
		if(this.agents.size() > numExpectedAgents) {
			throw new RuntimeException("Error! There are more agents in the environment than specified in the mas2j file. Number of agents: " + agents.size() + ". Expected number of agents: " + numExpectedAgents + " Please update the mas2j file with the correct number of agents.");
		}
    	
    	return true;
    }
    
    /**
     * Starts a new game.
     * Is called at the beginning of the tournament when all agents have performed the 'sayHello' action,
     * or after a game has finished and both players have performed the 'confirmEnd' action (and the tournament is not finished yet).
     */
	private void startNewGame() {
		
		String x_player = this.agents.get(indexOfNextXplayer);
		String o_player = this.agents.get(indexOfNextOplayer);
		
		this.currentGame = new Game(x_player, o_player);
		games.add(currentGame);
		
		logger.info("" + System.lineSeparator() + System.lineSeparator());
		logger.info("STARTING GAME NUMBER: " + games.size());
		
		setPercepts();
		
	}

	
    boolean handlePlay(String agName, Structure action) {
    	
    	/* 1. Check that it is the agent's turn.*/
    	if( ! agName.equals(currentGame.nameOfNextPlayer)) {
    		return false;
    	}
    	
    	/* 2. Parse the performed action.*/

		int x,y;
		
		try {
        	
			x = Integer.parseInt(action.getTerm(0).toString());
        	y = Integer.parseInt(action.getTerm(1).toString());
        	
		}catch(NumberFormatException e) {
			
			/* Undefined move!*/
			logger.info("Unknown action: " + action);
			return false;
			
			
		}
    		
    	if(x<0 || x>2 || y<0 || y>2) {
    		logger.info("Illegal value for x: " + x);
    		return false;
    	}
	
    	if(x<0 || x>2 || y<0 || y>2) {
    		logger.info("Illegal value for y: " + y);
    		return false;
    	}
        	
    	
    	/* 3. Update the game state.*/
    	boolean success = currentGame.playMove(agName, x, y);
    	
    	if(!success) {
    		logger.info("Illegal move! Square [" + x +"," + y + "] already occupied. ");
    		return false;
    	}
    	
    	logger.info(this.currentGame.getBoardString());
    	
    	if(currentGame.finished) {
    		
        	if(currentGame.nameOfWinner == null) {
        		logger.info("Game ended in a draw.");
        	}else {
        		logger.info("Game finished with winner: " + currentGame.nameOfWinner + " " + currentGame.symbolOfWinner);
        	}
    	}
    	
    	
    	
        
		setPercepts();
        
        informAgsEnvironmentChanged(); /*I'm not sure what this is for.*/
        
        
        return true; // the action was executed with success
    }
    
    
    
    /**
     * Determines who will be the next players.
     */
    void setNextPlayers() {
    	
		this.indexOfNextOplayer++;
		if(indexOfNextOplayer == agents.size()) {
			indexOfNextOplayer = 0;
			
			indexOfNextXplayer++;
			if(indexOfNextXplayer == agents.size()) {
				indexOfNextXplayer = 0;
			}
		}
		
		if(this.indexOfNextXplayer == this.indexOfNextOplayer) {
			setNextPlayers();
		}
    }
    
    
 
    
    private boolean handleConfirmEnd(String agName, Structure action) {
    	
    	if( ! this.currentGame.finished) {
    		return false;
    	}

    	this.currentGame.confirmEnd(agName);
    	
    	if(currentGame.playersToConfirmEnd.isEmpty()) {
    		
    		/* IF THE TOURNAMENT HAS FINISHED DISPLAY THE RESULTS */
    		if(this.games.size() == totalNumGames) {
    			logger.info("Tournament finished!");
    			
    			
    			for(String agent : agents) {
    				
    				int numDraws = 0;
    				int numVictories = 0;
    				int numLosses = 0;
    				
    				for(Game game : games) {
    					
    					if(!game.players.contains(agent)) {
    						continue;
    					}
    					
    					
    					if(game.nameOfWinner == null) {
    						numDraws++;
    					}else if(game.nameOfWinner.equals(agent)) {
    						numVictories++;
    					}else {
    						numLosses++;
    					}
    				}
    				
    				int points = 2* numVictories + numDraws;
    				logger.info(agent + " points: " + points + " (W: " + numVictories + " D: " + numDraws + " L: " + numLosses + ")");
    			}
    			
    		
    		}else {
        		
        		setNextPlayers();
    			startNewGame();
    			
    		}
    		
    	}
    	
		return true;
	}
    
	
    void setPercepts() {
    	
    	clearAllPercepts();
    	
    	
    	String x_player = this.currentGame.players.get(0);
    	String o_player = this.currentGame.players.get(1);
    	
    	/* Indicate the board. */
    	for(int x=0; x<3; x++) {
        	for(int y=0; y<3; y++) {
        		
        		if(this.currentGame.board[x][y] == null) {
        			continue;
        		}
        		
                try {
                	
                	Literal mark = ASSyntax.parseLiteral("mark("+ x + "," + y + "," + this.currentGame.board[x][y] + ")");
                	
                    addPercept(x_player, mark);
                    addPercept(o_player, mark);
                    
                    
                } catch (ParseException e) {
                    e.printStackTrace();
                }
        		
        		
        	}
    	}
    	
    	/* Indicate which symbol each player is using. */
		try {
			
			Literal x = ASSyntax.parseLiteral("symbol(x)");
			addPercept(x_player, x);
			
			Literal o = ASSyntax.parseLiteral("symbol(o)");
			addPercept(o_player, o);
			
		} catch (ParseException e) {
			e.printStackTrace();
		} catch (TokenMgrError e) {
			e.printStackTrace();
		}
    	
    	if(currentGame.symbolOfWinner != null) {
    		
    		try {
    			
    			Literal winnerSymbol = ASSyntax.parseLiteral("winner(" + currentGame.symbolOfWinner + ")");
    			
				addPercept(x_player, winnerSymbol);
				addPercept(o_player, winnerSymbol);
				
			} catch (ParseException | TokenMgrError e) {
				e.printStackTrace();
			}
    		
    	}
    	
    	if(currentGame.symbolOfNextPlayer != null) {
    		
    		try {
    			
    			Literal nextPlayerSymbol = ASSyntax.parseLiteral("next");
    			addPercept(currentGame.nameOfNextPlayer, nextPlayerSymbol);
				
			} catch (ParseException | TokenMgrError e) {
				e.printStackTrace();
			}
    		
    	}
    	
    	/* The predicate 'round(X)' is used to indicate to the agents that the a new round has started.
    	 * Initially, we used a 'next(S)' percept for this, but the problem is that sometimes
    	 * the update of the percepts happened too fast, so it changes from next(x) to next(o) and then back to next(x)
    	 * in one reasoning cycle. This means that the agent will not notice any changes, 
    	 * and therefore is's plan to make a move is not triggered.
    	 * 
    	 * TODO: can we find a better solution for this?
    	 * */
		try {
			
			Literal roundNumber = ASSyntax.parseLiteral("round("+ this.currentGame.legalActionsPlayed +  ")");
			
			addPercept(x_player, roundNumber);
			addPercept(o_player, roundNumber);
			
		} catch (ParseException | TokenMgrError e) {
			e.printStackTrace();
		}
    	
    	
    	if(currentGame.finished) {
    		
    		try {
    			
    			Literal end = ASSyntax.parseLiteral("end");
    			
				addPercept(x_player, end);
				addPercept(o_player, end);
				
			} catch (ParseException | TokenMgrError e) {
				e.printStackTrace();
			}
    	}
    	
    	
    }
}
