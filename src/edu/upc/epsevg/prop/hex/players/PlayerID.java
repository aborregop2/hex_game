package edu.upc.epsevg.prop.hex.players;

import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.SearchType;

public class PlayerID implements IPlayer, IAuto{

    public PlayerID(String name) {
        this.name = name;
        this.depth = 1;
    }

    @Override
    public PlayerMove move(HexGameStatus s) {

        return new PlayerMove(bestMove, exploredNodes, depth, SearchType.MINIMAX_IDS);
    }

    @Override
    public void timeout() {
        
    }

    @Override
    public String getName() {
        return "PlayerID(" + name + ")";
    }


    private List<Point> getPossibleMoves(HexGameStatus s) {
        List<Point> moves = new ArrayList<>();
        for (int i = 0; i < s.getSize(); i++) {
            for (int j = 0; j < s.getSize(); j++) {
                if (s.getPos(i, j) == 0) {
                    moves.add(new Point(i, j));
                }
            }
        }
        return moves;
    }
}


