package ticTacToe;

// Environment code for project jason_tic_tac_toe

import jason.asSyntax.*;
import jason.environment.*;
import jason.stdlib.string;
import jason.asSyntax.parser.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.*;

public class SingleGameEnvironment extends Environment {

    private Logger logger = Logger.getLogger("tic_tac_toe."+SingleGameEnvironment.class.getName());
    
    List<String> agents = new ArrayList<String>();
    Game game;
    

    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
        
        //this.setPercepts();
       
       /* 
        try {
            addPercept(ASSyntax.parseLiteral("percept("+args[0]+")"));
        } catch (ParseException e) {
            e.printStackTrace();
        }
        */
    }

    @Override
    public boolean executeAction(String agName, Structure action) {

    	
    	logger.info("Action performed: " + agName + " " + action);
    	
    	
    	if(action.getFunctor().equals("sayHello")) {
    		return handleHello(agName, action);
    	}
    	
    	if( action.getFunctor().equals("play")) {
    		return handlePlay(agName, action);
    	}
    	
    	/* This is only to be consistent with the TournamentEnvironment
    	 * */
    	if( action.getFunctor().equals("confirmEnd")) {
    		return this.game.finished;
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
		
		
    	if(this.agents.size() == 2) {
    		
    		logger.info("Two players have logged in! " + this.agents );
    		
    		this.game = new Game(this.agents.get(0), this.agents.get(1));
    		
    		setPercepts();
    	
    	}
    	
		if(this.agents.size() > 2) {
			throw new RuntimeException("Error! Too many agents. The SingeGameEnvironment can only handle 2 agents.");
		}
    	
    	return true;
    }
    
    boolean handlePlay(String agName, Structure action) {
    	
    	/* 1. Check that it is the agent's turn.*/
    	if( ! agName.equals(this.game.nameOfNextPlayer)) {
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
    	boolean success = game.playMove(agName, x, y);
    	
    	if(!success) {
    		logger.info("Illegal move! Square [" + x +"," + y + "] already occupied. ");
    		return false;
    	}
    	
    	
    	logger.info(this.game.getBoardString());
    	
    	if(game.finished) {
    		
        	if(game.nameOfWinner == null) {
        		logger.info("Game ended in a draw.");
        	}else {
        		logger.info("Game finished with winner: " + game.nameOfWinner + " " + game.symbolOfWinner);
        	}
    	}
    		
        
		setPercepts();
        
		informAgsEnvironmentChanged(); /*I'm not sure what this is for.*/
		
		
        return true; // the action was executed with success
    }
    
    
    
    

    
    
    void setPercepts() {
    	
    	clearAllPercepts();
    	
    	
    	String x_player = this.game.players.get(0);
    	String o_player = this.game.players.get(1);
    	
    	/* Indicate the board. */
    	for(int x=0; x<3; x++) {
        	for(int y=0; y<3; y++) {
        		
        		if(game.board[x][y] == null) {
        			continue;
        		}
        		
                try {
                    addPercept(ASSyntax.parseLiteral("mark("+ x + "," + y + "," + game.board[x][y] + ")"));
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
    	
    	if(game.symbolOfWinner != null) {
    		
    		
    		
    		try {
				addPercept(ASSyntax.parseLiteral("winner(" + game.symbolOfWinner + ")"));
			} catch (ParseException | TokenMgrError e) {
				e.printStackTrace();
			}
    		
    	}
    	
    	if(game.symbolOfNextPlayer != null) {
    		
    		
    		try {
    			
    			Literal nextPlayerSymbol = ASSyntax.parseLiteral("next");
    			addPercept(game.nameOfNextPlayer, nextPlayerSymbol);
				
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
			
			Literal roundNumber = ASSyntax.parseLiteral("round("+ this.game.legalActionsPlayed +  ")");
			
			addPercept(x_player, roundNumber);
			addPercept(o_player, roundNumber);
			
		} catch (ParseException | TokenMgrError e) {
			e.printStackTrace();
		}
		
		
		
    	if(game.finished) {
    		
    		try {
    			
    			Literal end = ASSyntax.parseLiteral("end");
    			
				addPercept(x_player, end);
				addPercept(o_player, end);
				
			} catch (ParseException | TokenMgrError e) {
				e.printStackTrace();
			}
    	}
    }
    
    
    
    /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
}
