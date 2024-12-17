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
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Map.Entry;

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
        exploredNodes = 0;
        int alpha = Integer.MIN_VALUE;
        int beta = Integer.MAX_VALUE;

        Point bestMove = minmax(s, depth, s.getCurrentPlayerColor(), true, alpha, beta).move;

        return new PlayerMove(bestMove, exploredNodes, depth, SearchType.MINIMAX);
    }

    private MinMaxResult minmax(HexGameStatus s, int depth, int color, boolean max, int alpha, int beta) {
        if (depth == 0) {
            return new MinMaxResult(null, heuristic(s, color));
        }

        Point bestMove = null;
        int bestValue = max ? Integer.MIN_VALUE : Integer.MAX_VALUE;

        List<Point> possibleMoves = getPossibleMoves(s);
        for (Point move : possibleMoves) {
            HexGameStatus newState = new HexGameStatus(s);
            newState.placeStone(move);

            int value = minmax(newState, depth - 1, -color, !max, alpha, beta).value;
            
            if (max) {
                if (value > bestValue) {
                    bestValue = value;
                    bestMove = move;
                }
                alpha = Math.max(alpha, bestValue);
            } else {
                if (value < bestValue) {
                    bestValue = value;
                    bestMove = move;
                }
                beta = Math.min(beta, bestValue);
            }

            exploredNodes++;
            if (beta <= alpha) {
                break;
            }

        }

        
        return new MinMaxResult(bestMove, bestValue);
    }

    private static class MinMaxResult {
        Point move;
        int value;

        MinMaxResult(Point move, int value) {
            this.move = move;
            this.value = value;
        }
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

        Map<Point, List<Point>> myGraph = new HashMap<>();
        Map<Point, List<Point>> hisGraph = new HashMap<>();
        buildGraph(s, color, myGraph, sourceNode, targetNode);
        buildGraph(s, -color, hisGraph, sourceNode, targetNode);

        HashMap<Point, Integer> myDistances = new HashMap<>();
        HashMap<Point, Integer> hisDistances = new HashMap<>();

        int myShortestPath = dijkstra(s, myGraph, myDistances, sourceNode, targetNode);
        int hisShortestPath = dijkstra(s, hisGraph, hisDistances, sourceNode, targetNode);

        int myPathCount = countPaths(s, myDistances, myShortestPath);
        int hisPathCount = countPaths(s, hisDistances, hisShortestPath);

        int boardWeight = calculateBoardWeight(s);

        if (myShortestPath == Integer.MAX_VALUE) {
            return Integer.MIN_VALUE; // No hay conexión
        }

        return (int) (100000.0 / myShortestPath + myPathCount * 100)
                - (int) (100000.0 / hisShortestPath + hisPathCount * 100);
    }

    private void buildGraph(HexGameStatus s, int color, Map<Point, List<Point>> graph, Point sourceNode, Point targetNode) {
        int size = s.getSize();

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (s.getPos(i, j) == color || s.getPos(i, j) == 0) {
                    Point current = new Point(i, j);
                    graph.putIfAbsent(current, new ArrayList<>());

                    for (Point neighbor : getNeighbors(current, s, color)) {
                        if (s.getPos(neighbor.x, neighbor.y) == color || s.getPos(neighbor.x, neighbor.y) == 0) {
                            graph.get(current).add(neighbor);
                        }
                    }

                    if (i == 0 && color == 1 || j == 0 && color == -1) {
                        graph.putIfAbsent(sourceNode, new ArrayList<>());
                        graph.get(sourceNode).add(current);
                        graph.get(current).add(sourceNode);
                    }

                    if ((i == size - 1 && color == 1) || (j == size - 1 && color == -1)) {
                        graph.putIfAbsent(targetNode, new ArrayList<>());
                        graph.get(targetNode).add(current);
                        graph.get(current).add(targetNode);
                    }
                }
            }
        }

        List<Point> sourceNeighbours = graph.get(sourceNode);
      List<Point> targetNeighbours = graph.get(targetNode);
      int size1 = sourceNeighbours.size();
      int size2 = targetNeighbours.size();

      if (color == 1) {
          for (int i = 0; i < size1-1; ++i) {
              Point n1 = sourceNeighbours.get(i);
              Point n2 = sourceNeighbours.get(i+1);

              if (n1.y + 1 == n2.y) {
                  Point n3 = new Point(n1.x + 1, n1.y);

                  if (graph.containsKey(n3)) {
                    graph.get(sourceNode).add(n3);
                    graph.get(n3).add(sourceNode);
                  }
              }

          }

          for (int i = 0; i < size2-1; ++i) {
              Point n1 = targetNeighbours.get(i);
              Point n2 = targetNeighbours.get(i+1);

              if (n1.y + 1 == n2.y) {
                  Point n3 = new Point(n2.x - 1, n1.y);
                  
                  if (graph.containsKey(n3)) {
                    
                    graph.get(targetNode).add(n3);
                    graph.get(n3).add(targetNode);
                  }
              }

          }
      }
      else  {
          for (int i = 0; i < size1-1; ++i) {
              Point n1 = sourceNeighbours.get(i);
              Point n2 = sourceNeighbours.get(i+1);

              if (n1.x + 1 == n2.x) {
                  Point n3 = new Point(n1.x, n1.y + 1);
                  if (graph.containsKey(n3)) {
                    graph.get(sourceNode).add(n3);
                    graph.get(n3).add(sourceNode);
                  }
              }

          }

          for (int i = 0; i < size2-1; ++i) {
              Point n1 = targetNeighbours.get(i);
              Point n2 = targetNeighbours.get(i+1);

              if (n1.x + 1 == n2.x) {
                  Point n3 = new Point(n2.x, n1.y - 1);
                  if (graph.containsKey(n3)) {

                    graph.get(targetNode).add(n3);
                    graph.get(n3).add(targetNode);
                  }
              }

          }
      }
    }

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
                    newDistance++;
                }

                if (newDistance < distances.getOrDefault(neighbor, Integer.MAX_VALUE)) {
                    distances.put(neighbor, newDistance);
                    pq.add(neighbor);
                }
            }
        }

        return Integer.MAX_VALUE; // No hay camino
    }

    private int countPaths(HexGameStatus s, HashMap<Point, Integer> distances, int shortestPath) {
        Set<Entry<Point, Integer>> entries = distances.entrySet();
        int counter = 0;

        if (s.getCurrentPlayerColor() == 1) {
            for (Entry<Point, Integer> entry : entries) {
                if (entry.getKey().x == s.getSize() - 1 && entry.getValue() == shortestPath) {
                    counter++;
                }
            }
        } else {
            for (Entry<Point, Integer> entry : entries) {
                if (entry.getKey().y == s.getSize() - 1 && entry.getValue() == shortestPath) {
                    counter++;
                }
            }
        }

        return counter;
    }

    private static List<Point> getNeighbors(Point p, HexGameStatus s, int color) {
        int x = p.x, y = p.y;
        List<Point> neighbors = new ArrayList<>();
    
        int[][] directions = {
            {-1, 0}, {0, -1}, {1, -1}, {1, 0}, {0, 1}, {-1, 1}
        };
    
        for (int i = 0; i < directions.length; i++) {
            int nx = x + directions[i][0];
            int ny = y + directions[i][1];
    
            int j = (i + 1) % directions.length;
            int nx2 = x + directions[j][0];
            int ny2 = y + directions[j][1];
    
            if (nx >= 0 && nx < s.getSize() && ny >= 0 && ny < s.getSize()
                && nx2 >= 0 && nx2 < s.getSize() && ny2 >= 0 && ny2 < s.getSize()){
              
              if (s.getPos(nx, ny) == 0 || s.getPos(nx, ny) == color) {
                if (!neighbors.contains(new Point(nx, ny))) {
                  neighbors.add(new Point(nx, ny));
                }
              }
    
              if (s.getPos(nx2, ny2) == 0 || s.getPos(nx2, ny2) == color) {
                if (!neighbors.contains(new Point(nx2, ny2))) {
                  neighbors.add(new Point(nx2, ny2));
                }
              }
    
              if ((s.getPos(nx, ny) == 0) && (s.getPos(nx2, ny2) == 0)) {
                int nx3 = x + directions[i][0] + directions[j][0];
                int ny3 = y + directions[i][1] + directions[j][1];
    
                if (nx3 >= 0 && nx3 < s.getSize() && ny3 >= 0 && ny3 < s.getSize()) {
                  if (s.getPos(nx3, ny3) == 0 || s.getPos(nx3, ny3) == color) {
                    if (!neighbors.contains(new Point(nx3, ny3))) {
                      neighbors.add(new Point(nx3, ny3));
                    }
                  }
                }
              }
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
        int center = size / 2;
        int weight = 0;

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                int pos = s.getPos(i, j);
                if (pos != 0) {
                    int distanceToCenter = Math.abs(i - center) + Math.abs(j - center);
                    int positionWeight = size - distanceToCenter;
                    weight += (pos == s.getCurrentPlayerColor()) ? positionWeight : -positionWeight;
                }
            }
        }

        return weight;
    }
}
