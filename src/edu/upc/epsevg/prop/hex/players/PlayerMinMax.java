package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;
import java.util.Map.Entry;
import java.util.ArrayList;
import java.util.Collection;
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

        //System.out.println("Best move: " + bestMove);

        return new PlayerMove(bestMove, exploredNodes, depth, SearchType.MINIMAX);
    }

    public int minmax(HexGameStatus s, int depth, int color, boolean max, int alpha, int beta) {
        //System.out.println("Depth: " + depth);
        if (depth == 0) {
            return heuristic(s, color);
        }

        int bestValue = max ? Integer.MIN_VALUE : Integer.MAX_VALUE;

        List<Point> possibleMoves = getPossibleMoves(s);

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
        Map<Point, List<Point>> myGraph = new HashMap<>();
        Map<Point, List<Point>> hisGraph = new HashMap<>();
        buildGraph(s, color, myGraph, sourceNode, targetNode);
        buildGraph(s, -color, hisGraph, sourceNode, targetNode);

        // Ejecutar Dijkstra desde el nodo fuente al nodo destino
        HashMap<Point, Integer> myDistances = new HashMap<>();
        HashMap<Point, Integer> hisDistances = new HashMap<>();

        int myShortestPath = dijkstra(s, myGraph, myDistances, sourceNode, targetNode);
        int hisShortestPath = dijkstra(s, hisGraph, hisDistances, sourceNode, targetNode);
        //System.out.println("Shortest path: " + shortestPath);

        // Cuantos más caminos cortos haya, mejor
        int myPathCount = countPaths(s, myDistances, myShortestPath);
        int hisPathCount = countPaths(s, hisDistances, hisShortestPath);
       //System.out.println("Path count: " + pathCount);

        // Calculamos el peso del tablero basado en la posición de las fichas
        int boardWeight = calculateBoardWeight(s);
        //System.out.println("Board weight: " + boardWeight);

        // Puntuación basada en el camino más corto y el número de caminos
        if (myShortestPath == Integer.MAX_VALUE) {
            return Integer.MIN_VALUE; // No hay conexión
        }

        
        return (int) (100000.0 / myShortestPath + myPathCount * 100) -  (int) (100000.0 / hisShortestPath + hisPathCount * 100);
    }

    // Construye el grafo con nodos virtuales y conexiones válidas
    private void buildGraph(HexGameStatus s, int color, Map<Point, List<Point>> graph, Point sourceNode, Point targetNode) {
        int size = s.getSize();

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (s.getPos(i, j) == color || s.getPos(i, j) == 0) {
                    Point current = new Point(i, j);
                    graph.putIfAbsent(current, new ArrayList<>());

                    // Conectar nodos adyacentes
                    for (Point neighbor : getNeighbors(current, size)) {
                        if (s.getPos(neighbor.x, neighbor.y) == color || s.getPos(neighbor.x, neighbor.y) == 0) {
                            graph.get(current).add(neighbor);
                        }
                    }

                    // Conectar nodos de la primera fila al nodo virtual fuente
                    if (i == 0 && s.getCurrentPlayerColor() == 1) {
                        graph.putIfAbsent(sourceNode, new ArrayList<>());
                        graph.get(sourceNode).add(current);
                        graph.get(current).add(sourceNode);
                    }
                    else if (j == 0 && s.getCurrentPlayerColor() == -1) {
                        graph.putIfAbsent(sourceNode, new ArrayList<>());
                        graph.get(sourceNode).add(current);
                        graph.get(current).add(sourceNode);
                    }

                    // Conectar nodos de la última fila al nodo virtual destino
                    if ((i == size - 1) && s.getCurrentPlayerColor() == 1) {
                        graph.putIfAbsent(targetNode, new ArrayList<>());
                        graph.get(targetNode).add(current);
                        graph.get(current).add(targetNode);
                    }
                    else if ((j == size - 1) && s.getCurrentPlayerColor() == -1) {
                        graph.putIfAbsent(targetNode, new ArrayList<>());
                        graph.get(targetNode).add(current);
                        graph.get(current).add(targetNode);
                    }
                }
            }
        }
    }


    // Ejecuta Dijkstra desde el nodo fuente al nodo destino
    private int dijkstra(HexGameStatus s, Map<Point, List<Point>> graph, HashMap<Point, Integer> distances, Point source, Point target) {
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
                int newDistance = currentDistance;

                if (!source.equals(neighbor) && !target.equals(neighbor) && s.getPos(neighbor.x, neighbor.y) == 0) {
                    newDistance++; // Peso uniforme
                }
                
                if (newDistance < distances.getOrDefault(neighbor, Integer.MAX_VALUE)) {
                    distances.put(neighbor, newDistance);
                    pq.add(neighbor);
                }

            }
        }

        return Integer.MAX_VALUE; // No hay camino
    }

    // Cuenta la cantidad de caminos válidos entre los nodos virtuales
    private int countPaths(HexGameStatus s, HashMap<Point, Integer> distances, int shortestPath) {
        Set<Entry<Point,Integer>> entries = distances.entrySet();
        int counter = 0;
        if (s.getCurrentPlayerColor() == 1) {
            for (Entry<Point, Integer> entry : entries) {
                if (entry.getKey().x == s.getSize() - 1 && entry.getValue() == shortestPath) {
                    counter++;
                }
            }
        }
        else {
            for (Entry<Point, Integer> entry : entries) {
                if (entry.getKey().y == s.getSize() - 1 && entry.getValue() == shortestPath) {
                    counter++;
                }
            }
        }

        return counter;
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

    private int calculateBoardWeight(HexGameStatus s) {
        int size = s.getSize();
        int center = size / 2; // Coordenada central del tablero
        int weight = 0;

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                int pos = s.getPos(i, j);
                if (pos != 0) { // Si hay una piedra colocada
                    // Calcular la distancia al centro (usamos Manhattan como métrica)
                    int distanceToCenter = Math.abs(i - center) + Math.abs(j - center);

                    // Invertimos el peso: posiciones más cercanas al centro tienen más peso
                    int positionWeight = size - distanceToCenter;

                    // Sumamos el peso positivo o negativo según el jugador
                    weight += (pos == s.getCurrentPlayerColor()) ? positionWeight : -positionWeight;
                }
            }
        }

        return weight;
    }
}
