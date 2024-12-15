/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.upc.epsevg.prop.hex;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerType;
import edu.upc.epsevg.prop.hex.players.ProfeGameStatus2;
import edu.upc.epsevg.prop.hex.players.ProfeGameStatus3;
import edu.upc.epsevg.prop.hex.players.ProfeGameStatus3.Result;



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
/**
 *
 * @author bernat
 */
public class UnitTesting {
    
    
    
    public static void main(String[] args) {
    
        
      byte[][] board = {
        //0  1  2  3  4  5  6  7  8  9 10   (!!!!!!!!!!!! X !!!!!!!!!!!!!) ->

        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 0  (!!!!!!!!!!!! Y !!!!!!!!!!!!!) 
          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 1
            { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 2
              { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 3
                { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 4
                  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 5
                    { 0, 0, 0, -1,-1,-1, -1,-1, 0, 0, 0}, // 6
                      { 0, 0, 0, 0, 0, 0,-1, 0, 0, 0, 0}, // 7
                        { 0, 0, 0, 0, 0, 0,-1, 0, 0, 0, 0}, // 8
                          { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 9
                            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // 10
      };


        HexGameStatus gs = new HexGameStatus(board, PlayerType.PLAYER2);  
        System.out.println("EN (0,0) HAY " + gs.getPos(0, 0)); 
        System.out.println("EN (0,1) HAY " + gs.getPos(0, 1));
        System.out.println("EN (1,0) HAY " + gs.getPos(1, 0));
        
        Point sourceNode = new Point(-1, -1); // Nodo virtual para la primera fila
        Point targetNode = new Point(-2, -2); // Nodo virtual para la última fila

        // Construimos el grafo para Dijkstra
        Map<Point, List<Point>> graph = new HashMap<>();
        System.out.println(gs.getCurrentPlayerColor());
        buildGraph(gs, gs.getCurrentPlayerColor(), graph, sourceNode, targetNode);
        printGraph(graph, sourceNode, targetNode);

        HashMap<Point, Integer> distances = new HashMap<>();
        System.out.println("Distancia mínima: " + dijkstra(gs, graph, distances, sourceNode, targetNode));
        //printHashMapVisual(distances);
        
    }

    private static void buildGraph(HexGameStatus s, int color, Map<Point, List<Point>> graph, Point sourceNode, Point targetNode) {
      int size = s.getSize();

      for (int i = 0; i < size; i++) {
          for (int j = 0; j < size; j++) {
              if (s.getPos(i, j) == color || s.getPos(i, j) == 0) {
                  Point current = new Point(i, j);
                  graph.putIfAbsent(current, new ArrayList<>());

                  for (Point neighbor : getNeighbors(current, s)) {
                    graph.get(current).add(neighbor);
                  }

                  ///////////

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
  }

  private static void printGraph(Map<Point, List<Point>> graph, Point sourceNode, Point targetNode) {
    StringBuilder sb = new StringBuilder();
    sb.append("\n--- Representación del Grafo ---\n\n");
    
    // Crear una lista ordenada de todos los nodos
    List<Point> sortedNodes = new ArrayList<>(graph.keySet());
    sortedNodes.sort(Comparator.comparing(Point::toString));
    
    // Mover el nodo fuente al principio
    sortedNodes.remove(sourceNode);
    sortedNodes.add(0, sourceNode);
    
    // Mover el nodo destino al final
    sortedNodes.remove(targetNode);
    sortedNodes.add(targetNode);
    
    // Imprimir nodos
    for (Point node : sortedNodes) {
        sb.append("[" + node + "]\n");
        for (Point connection : graph.getOrDefault(node, new ArrayList<>())) {
            sb.append(" |--> [" + connection + "]\n");
        }
        sb.append("\n");
    }
    
    sb.append("\n--- Fin del Grafo ---\n");
    System.out.println(sb.toString());
}

  private static List<Point> getNeighbors(Point p, HexGameStatus s) {
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
          
          if (s.getPos(nx, ny) == 0 || s.getPos(nx, ny) == s.getCurrentPlayerColor()) {
            if (!neighbors.contains(new Point(nx, ny))) {
              neighbors.add(new Point(nx, ny));
            }
          }

          if (s.getPos(nx2, ny2) == 0 || s.getPos(nx2, ny2) == s.getCurrentPlayerColor()) {
            if (!neighbors.contains(new Point(nx2, ny2))) {
              neighbors.add(new Point(nx2, ny2));
            }
          }

          if (s.getPos(nx, ny) == 0 && s.getPos(nx2, ny2) == 0) {
            int nx3 = x + directions[i][0] + directions[j][0];
            int ny3 = y + directions[i][1] + directions[j][1];

            if (nx3 >= 0 && nx3 < s.getSize() && ny3 >= 0 && ny3 < s.getSize()) {
              if (s.getPos(nx3, ny3) == 0 || s.getPos(nx3, ny3) == s.getCurrentPlayerColor()) {
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

      // Ejecuta Dijkstra desde el nodo fuente al nodo destino
      private static int dijkstra(HexGameStatus s, Map<Point, List<Point>> graph, HashMap<Point, Integer> distances, Point source, Point target) {
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

    public static void printHashMapVisual(HashMap<Point, Integer> map) {
      if (map.isEmpty()) {
          System.out.println("El HashMap está vacío.");
          return;
      }

      System.out.println("Contenido del HashMap:");
      System.out.println("-----------------------");

      for (Map.Entry<Point, Integer> entry : map.entrySet()) {
          Point point = entry.getKey();
          Integer value = entry.getValue();

          System.out.printf("Punto (x: %d, y: %d) -> Valor: %d%n", point.x, point.y, value);
      }

      System.out.println("-----------------------");
    }
  
    
}
