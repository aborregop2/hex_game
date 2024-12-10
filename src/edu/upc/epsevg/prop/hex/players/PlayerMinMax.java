package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

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
        Point sourceNode = new Point(-1, -1); // Nodo virtual para la primera fila
        Point targetNode = new Point(-2, -2); // Nodo virtual para la última fila

        // Construimos el grafo para Dijkstra
        Map<Point, List<Point>> graph = new HashMap<>();
        buildGraph(s, color, graph, sourceNode, targetNode);

        // Ejecutar Dijkstra desde el nodo fuente al nodo destino
        int shortestPath = dijkstra(graph, sourceNode, targetNode);

        // Cuantos más caminos cortos haya, mejor
        int pathCount = countPaths(graph, sourceNode, targetNode);

        // Puntuación basada en el camino más corto y el número de caminos
        if (shortestPath == Integer.MAX_VALUE) {
            return Integer.MIN_VALUE; // No hay conexión
        }
        return (int) (1000.0 / shortestPath + pathCount * 10);
    }

    // Construye el grafo con nodos virtuales y conexiones válidas
    private void buildGraph(HexGameStatus s, int color, Map<Point, List<Point>> graph, Point sourceNode, Point targetNode) {
        int size = s.getSize();

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                Point current = new Point(i, j);
                if (s.getPos(i, j) == color || s.getPos(i, j) == 0) {
                    graph.putIfAbsent(current, new ArrayList<>());

                    // Conectar nodos adyacentes
                    for (Point neighbor : getNeighbors(current, size)) {
                        if (s.getPos(neighbor.x, neighbor.y) == color || s.getPos(neighbor.x, neighbor.y) == 0) {
                            graph.get(current).add(neighbor);
                        }
                    }

                    // Conectar nodos de la primera fila al nodo virtual fuente
                    if (i == 0) {
                        graph.putIfAbsent(sourceNode, new ArrayList<>());
                        graph.get(sourceNode).add(current);
                    }

                    // Conectar nodos de la última fila al nodo virtual destino
                    if (i == size - 1) {
                        graph.putIfAbsent(targetNode, new ArrayList<>());
                        graph.get(targetNode).add(current);
                    }
                }
            }
        }
    }

    // Ejecuta Dijkstra desde el nodo fuente al nodo destino
    private int dijkstra(Map<Point, List<Point>> graph, Point source, Point target) {
        Map<Point, Integer> distances = new HashMap<>();
        PriorityQueue<Point> pq = new PriorityQueue<>(Comparator.comparingInt(distances::get));

        distances.put(source, 0);
        pq.add(source);

        while (!pq.isEmpty()) {
            Point current = pq.poll();
            int currentDistance = distances.get(current);

            if (current.equals(target)) {
                return currentDistance;
            }

            for (Point neighbor : graph.getOrDefault(current, new ArrayList<>())) {
                int newDistance = currentDistance + 1; // Peso uniforme
                if (newDistance < distances.getOrDefault(neighbor, Integer.MAX_VALUE)) {
                    distances.put(neighbor, newDistance);
                    pq.add(neighbor);
                }
            }
        }

        return Integer.MAX_VALUE; // No hay camino
    }

    // Cuenta la cantidad de caminos válidos entre los nodos virtuales
    private int countPaths(Map<Point, List<Point>> graph, Point source, Point target) {
        Set<Point> visited = new HashSet<>();
        return dfs(graph, source, target, visited);
    }

    // DFS para contar caminos
    private int dfs(Map<Point, List<Point>> graph, Point current, Point target, Set<Point> visited) {
        if (current.equals(target)) {
            return 1;
        }
        visited.add(current);
        int pathCount = 0;

        for (Point neighbor : graph.getOrDefault(current, new ArrayList<>())) {
            if (!visited.contains(neighbor)) {
                pathCount += dfs(graph, neighbor, target, visited);
            }
        }

        visited.remove(current);
        return pathCount;
    }

    // Retorna los vecinos válidos dentro del tablero
    private List<Point> getNeighbors(Point p, int size) {
        int x = p.x, y = p.y;
        List<Point> neighbors = new ArrayList<>();

        int[][] directions = {
            {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}
        };

        for (int[] dir : directions) {
            int nx = x + dir[0];
            int ny = y + dir[1];
            if (nx >= 0 && nx < size && ny >= 0 && ny < size) {
                neighbors.add(new Point(nx, ny));
            }
        }

        return neighbors;
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
