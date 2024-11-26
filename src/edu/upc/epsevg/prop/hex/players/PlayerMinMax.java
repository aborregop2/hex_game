package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

public class PlayerMinMax implements IPlayer, IAuto {
    String name;
    long exploredNodes;
    int depth;

    public PlayerMinMax(String name, int depth) {
        this.name = name;
        this.depth = depth;
    }

    @Override
    public PlayerMove move(HexGameStatus s) {
        Point bestMove = null;
        exploredNodes = 0;
        int bestEval = Integer.MIN_VALUE;
        int alpha = Integer.MIN_VALUE;
        int beta = Integer.MAX_VALUE;

        List<Point> possibleMoves = getPossibleMoves(s);

        possibleMoves.sort((move1, move2) -> {
            HexGameStatus newState1 = new HexGameStatus(s);
            newState1.placeStone(move1);
            int eval1 = heuristic(newState1, s.getCurrentPlayerColor());

            HexGameStatus newState2 = new HexGameStatus(s);
            newState2.placeStone(move2);
            int eval2 = heuristic(newState2, s.getCurrentPlayerColor());

            return Integer.compare(eval2, eval1);
        });

        for (Point move : possibleMoves) {
            HexGameStatus newState = new HexGameStatus(s);
            newState.placeStone(move);
            int eval = minmax(newState, depth - 1, s.getCurrentPlayerColor(), false, alpha, beta);

            if (eval > bestEval) {
                bestEval = eval;
                bestMove = move;
            }

            alpha = Math.max(alpha, bestEval);
        }

        return new PlayerMove(bestMove, exploredNodes, depth, SearchType.MINIMAX);
    }

    public int minmax(HexGameStatus s, int depth, int color, boolean max, int alpha, int beta) {
        if (depth == 0) {
            return heuristic(s, color);
        }

        int bestValue = max ? Integer.MIN_VALUE : Integer.MAX_VALUE;

        List<Point> possibleMoves = getPossibleMoves(s);

        possibleMoves.sort((move1, move2) -> {
            HexGameStatus newState1 = new HexGameStatus(s);
            newState1.placeStone(move1);
            int eval1 = heuristic(newState1, color);

            HexGameStatus newState2 = new HexGameStatus(s);
            newState2.placeStone(move2);
            int eval2 = heuristic(newState2, color);

            return Integer.compare(eval2, eval1);
        });

        for (Point move : possibleMoves) {
            HexGameStatus newState = new HexGameStatus(s);
            newState.placeStone(move);
            int value = minmax(newState, depth - 1, -color, !max, alpha, beta);

            if (max) {
                bestValue = Math.max(bestValue, value);
                alpha = Math.max(alpha, bestValue);
            } else {
                bestValue = Math.min(bestValue, value);
                beta = Math.min(beta, bestValue);
            }

            exploredNodes++;

            if (beta <= alpha) {
                return bestValue;
            }
        }

        return bestValue;
    }

    @Override
    public void timeout() {
        // Implementación vacía para manejar timeouts
    }

    @Override
    public String getName() {
        return "PlayerMinMax(" + name + ")";
    }

    public int heuristic(HexGameStatus s, int color) {
        // Implementa aquí la heurística para evaluar el estado del tablero
        return 0;
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
