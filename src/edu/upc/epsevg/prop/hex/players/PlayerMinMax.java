package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;

public class PlayerMinMax implements IPlayer, IAuto{
    String name;
    long exploredNodes;
    int depth;

    public PlayerMinMax(String name, int depth) {
        this.name = name;
        this.depth = depth;
    }

    @Override
    public PlayerMove move(HexGameStatus s) {
        Point cell = new Point(0, 0);
        exploredNodes = 0;
        int bestEval = Integer.MIN_VALUE;

        for (int i = 0; i < s.getSize(); i++) {
            for (int j = 0; j < s.getSize(); j++) {
                if (s.getPos(i, j) == 0) {
                    HexGameStatus newState = new HexGameStatus(s);
                    newState.placeStone(new Point(i, j));
                    int eval = minmax(newState, false);
                    if (eval > bestEval) {
                        bestEval = eval;
                        cell = new Point(i, j);
                    }
                }
            }
        }

        return new PlayerMove(cell, exploredNodes, depth, SearchType.MINIMAX);
    }

    public int minmax(HexGameStatus s, boolean max) {
        if (depth == 0) {
            return heuristic(s);
        }

        int bestValue = max ? Integer.MIN_VALUE : Integer.MAX_VALUE;

        for (int i = 0; i < s.getSize(); i++) {
            for (int j = 0; j < s.getSize(); j++) {
                if (s.getPos(i, j) == 0) {
                    HexGameStatus newState = new HexGameStatus(s);
                    newState.placeStone(new Point(i, j));
                    int value = minmax(newState, !max);

                    if (max) {
                        if (value > bestValue) {
                            bestValue = value;
                        }
                    } 
                    else {
                        if (value < bestValue) {
                            bestValue = value;
                        }
                    }

                    exploredNodes++;
                }
            }
        }

        return bestValue;
    }

    @Override
    public void timeout() {
        
    }

    @Override
    public String getName() {
        return "PlayerMinMax(" + name + ")";
    }


    public int heuristic(HexGameStatus s){
        return 0;
    }

}
