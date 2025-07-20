package geography.agents;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;

import repast.simphony.context.Context;
import repast.simphony.engine.schedule.ScheduleParameters;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.grid.Grid;
import repast.simphony.space.grid.GridPoint;
import repast.simphony.util.ContextUtils;

public class GisAgent {

    private String name;
    private ZoneAgent targetSafeZone;
    
    private Geography<GisAgent> geography;
    private Grid<MapCell> mapCellGrid; // Grilla para consultar si es transitable
    private Grid<GisAgent> agentGrid;  // Grilla para el movimiento y posición de los agentes
    private GeometryFactory geometryFactory = new GeometryFactory();
    private GridPoint safeZoneGridTarget; // Objetivo de la zona segura en la grilla
    
    private List<GridPoint> currentPath;  // Camino calculado por A*
    private int pathIndex;                // Índice del paso actual en el camino


    public GisAgent(String name, ZoneAgent targetSafeZone, Grid<MapCell> mapCellGrid, Grid<GisAgent> agentGrid) {
        this.name = name;
        this.targetSafeZone = targetSafeZone;
        this.mapCellGrid = mapCellGrid;
        this.agentGrid = agentGrid;
        this.currentPath = new ArrayList<>();
        this.pathIndex = 0;

    }

    @ScheduledMethod(start = 1, interval = 1, priority = ScheduleParameters.FIRST_PRIORITY)
    public void step() {
        if (geography == null) {
            Context context = ContextUtils.getContext(this);
            geography = (Geography)context.getProjection("Geography");
        }

        // Obtener el punto objetivo de la zona segura en la grilla
        if (safeZoneGridTarget == null) {
            try {
                Geometry safeZoneGeom = geography.getGeometry(targetSafeZone);
                if (safeZoneGeom != null && safeZoneGeom.getCentroid() != null) {
                    Coordinate safeZoneCoord = safeZoneGeom.getCentroid().getCoordinate();
                    safeZoneGridTarget = ContextCreator.mapGeoToGrid(safeZoneCoord);
                    System.out.println("DEBUG (GisAgent " + name + "): Objetivo de zona segura en grilla establecido: (" + safeZoneGridTarget.getX() + ", " + safeZoneGridTarget.getY() + ")");
                } else {
                    System.err.println("DEBUG (GisAgent " + name + "): Zona segura o su centroide es nulo al iniciar step().");
                    return;
                }
            } catch (Exception e) {
                System.err.println("Error al intentar obtener la geometría/centroide de la zona segura al iniciar step(): " + e.getMessage());
                return;
            }
        }
        moveAlongCalculatedPath(); //Moverse por el camino
    }

    private void moveAlongCalculatedPath(){
        // Obtener la posición actual del agente
        GridPoint currentAgentGridPoint = agentGrid.getLocation(this);

        if (currentAgentGridPoint == null) {
            System.err.println("DEBUG (GisAgent " + name + "): No se pudo obtener la posición del agente en la grilla de agentes.");
            return;
        }

        // Mostrar posición actual del agente
        System.out.println("DEBUG (GisAgent " + name + "): Posición actual en grilla: (" + currentAgentGridPoint.getX() + ", " + currentAgentGridPoint.getY() + ")");
        System.out.println("DEBUG (GisAgent " + name + "): Objetivo de zona segura en grilla: (" + safeZoneGridTarget.getX() + ", " + safeZoneGridTarget.getY() + ")");

        // Si ya llegamos al objetivo, o si el camino actual está terminado/vacío, o si nos desviamos, recalcular.
        if (currentAgentGridPoint.equals(safeZoneGridTarget) || currentPath.isEmpty() || pathIndex >= currentPath.size() || (pathIndex < currentPath.size() && !currentPath.get(pathIndex).equals(currentAgentGridPoint))) {
            // Recalcula solo si no estamos ya en el objetivo Y si el camino necesita ser reevaluado
            if (!currentAgentGridPoint.equals(safeZoneGridTarget)) {
                System.out.println("DEBUG (GisAgent " + name + "): Camino terminado/desviado. Recalculando camino A*.");
                calculatePathToSafeZone(currentAgentGridPoint, safeZoneGridTarget);
            } else {
                 System.out.println("DEBUG (GisAgent " + name + "): Agente ya en zona segura. No se mueve.");
                 return; // Ya llegó
            }
        }

        // No hay ruta posible.
        if (currentPath.isEmpty()) {
            System.err.println("DEBUG (GisAgent " + name + "): No se pudo encontrar un camino transitable.");
            return;
        }

        GridPoint nextGridPoint;
        // Si el agente ya está en el último punto del camino calculado
        if (currentAgentGridPoint.equals(currentPath.get(currentPath.size() - 1))) {
             nextGridPoint = currentAgentGridPoint;
        } else if (pathIndex + 1 < currentPath.size()) {
            if (currentAgentGridPoint.equals(currentPath.get(pathIndex))) {
                 pathIndex++;
                 nextGridPoint = currentPath.get(pathIndex);
            } else {
                 // Si el agente se salió del camino (quizás por un recalculado reciente o un error),
                 // forzarlo al primer paso del camino recalculado, asumiendo que el punto 0 es el actual.
                 pathIndex = 0;
                 nextGridPoint = currentPath.get(pathIndex);
                 System.out.println("DEBUG (GisAgent " + name + "): Agente desviado o camino recalculado. Reiniciando pathIndex y moviendo al primer paso: " + nextGridPoint);
            }
        } else {
            System.err.println("DEBUG (GisAgent " + name + "): Error de lógica en pathIndex (fuera de límites). Recalculando camino.");
            calculatePathToSafeZone(currentAgentGridPoint, safeZoneGridTarget);
            if (!currentPath.isEmpty()) {
                pathIndex = 0;
                nextGridPoint = currentPath.get(pathIndex);
            } else {
                return; // No hay camino válido incluso después de recalcular
            }
        }

        // Consultar la transitabilidad de la celda
        MapCell nextCell = mapCellGrid.getObjectAt(nextGridPoint.getX(), nextGridPoint.getY());
        if (nextCell != null && nextCell.isTraversable()) {
            agentGrid.moveTo(this, nextGridPoint.getX(), nextGridPoint.getY());
            Coordinate newGeoCoord = ContextCreator.mapGridToGeo(nextGridPoint.getX(), nextGridPoint.getY());
            geography.move(this, geometryFactory.createPoint(newGeoCoord));
            System.out.println("DEBUG (GisAgent " + name + "): Movido a GRIDA: " + nextGridPoint + " y GEO: (" + newGeoCoord.x + ", " + newGeoCoord.y + ")");
        } else {
            System.err.println("DEBUG (GisAgent " + name + "): ¡ERROR! El siguiente paso del camino (" + nextGridPoint + ") no es transitable. Recalculando camino.");
            calculatePathToSafeZone(currentAgentGridPoint, safeZoneGridTarget);
        }
    }

    // Método para calcular el camino usando A*
    private void calculatePathToSafeZone(GridPoint start, GridPoint target) {
        currentPath.clear();
        pathIndex = 0;

        if (start == null || target == null) {
            System.err.println("Error: Start o Target es nulo para A*.");
            return;
        }
        if (start.equals(target)) {
            currentPath.add(start); // Ya estamos en el objetivo.
            System.out.println("DEBUG (GisAgent " + name + "): Agente ya en el punto de inicio/destino del A*. No se calcula camino.");
            return;
        }

        Map<GridPoint, GridPoint> cameFrom = new HashMap<>();
        Map<GridPoint, Double> gScore = new HashMap<>();
        Map<GridPoint, Double> fScore = new HashMap<>();

        List<GridPoint> openList = new ArrayList<>();

        gScore.put(start, 0.0);
        fScore.put(start, calculateHeuristic(start, target));
        openList.add(start);

        GridPoint closestToTargetSoFar = start;
        double minDistanceFromTarget = calculateHeuristic(start, target);

        while (!openList.isEmpty()) {
            Collections.sort(openList, Comparator.comparingDouble(fScore::get));
            GridPoint current = openList.remove(0);

            // Actualizar el nodo más cercano al objetivo si es mejor
            double distCurrentToTarget = calculateHeuristic(current, target);
            if (distCurrentToTarget < minDistanceFromTarget) {
                minDistanceFromTarget = distCurrentToTarget;
                closestToTargetSoFar = current;
            }

            if (current.equals(target)) {
                currentPath = reconstructPath(cameFrom, current);
                System.out.println("DEBUG (GisAgent " + name + "): ¡Camino A* encontrado! Longitud: " + currentPath.size());
                return;
            }

            // Vecinos de la celda actual
            for (GridPoint neighbor : getNeighbors(current)) {
                // Obtener las dimensiones de la grilla desde mapCellGrid (ya que es la que tiene la estructura del mapa)
                int gridWidth = mapCellGrid.getDimensions().getWidth();
                int gridHeight = mapCellGrid.getDimensions().getHeight();

                // Asegurarse de que el vecino esté dentro de los límites de la grilla
                if (neighbor.getX() < 0 || neighbor.getX() >= gridWidth ||
                    neighbor.getY() < 0 || neighbor.getY() >= gridHeight) {
                    continue; // Fuera de límites
                }

                // Usar mapCellGrid para consultar la transitabilidad del vecino
                MapCell neighborCell = mapCellGrid.getObjectAt(neighbor.getX(), neighbor.getY());

                // Solo consideramos vecinos transitables
                if (neighborCell != null && neighborCell.isTraversable()) {
                    double tentativeGScore = gScore.getOrDefault(current, Double.POSITIVE_INFINITY) + 1; // Costo fijo de 1 por cada paso

                    if (tentativeGScore < gScore.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
                        cameFrom.put(neighbor, current);
                        gScore.put(neighbor, tentativeGScore);
                        fScore.put(neighbor, tentativeGScore + calculateHeuristic(neighbor, target));

                        if (!openList.contains(neighbor)) {
                            openList.add(neighbor);
                        }
                    }
                }
            }
        }

        // Si no se encontró un camino directo al target, reconstruir el camino al punto transitable más cercano
        if (!closestToTargetSoFar.equals(start)) {
             currentPath = reconstructPath(cameFrom, closestToTargetSoFar);
             System.err.println("DEBUG (GisAgent " + this.name + "): No se pudo encontrar un camino directo al objetivo. Se encontró el camino al punto transitable más cercano: " + closestToTargetSoFar);
        } else {
             System.err.println("DEBUG (GisAgent " + this.name + "): No se pudo encontrar un camino a la zona segura para el agente. (Esto puede ser normal si no hay rutas).");
        }
    }

    // Calcula la distancia euclidiana (heurística) entre dos GridPoints
    private double calculateHeuristic(GridPoint p1, GridPoint p2) {
        return Math.sqrt(
            Math.pow(p1.getX() - p2.getX(), 2) +
            Math.pow(p1.getY() - p2.getY(), 2)
        );
    }

    // Reconstruye el camino desde cameFrom
    private List<GridPoint> reconstructPath(Map<GridPoint, GridPoint> cameFrom, GridPoint current) {
        List<GridPoint> totalPath = new ArrayList<>();
        totalPath.add(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            totalPath.add(current);
        }
        Collections.reverse(totalPath); // El camino se construye al revés, hay que invertirlo
        return totalPath;
    }

    // Obtiene los 8 vecinos (incluyendo diagonales) de un GridPoint
    private List<GridPoint> getNeighbors(GridPoint p) {
        List<GridPoint> neighbors = new ArrayList<>();
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0)
				 {
					continue;
				}

                int nx = p.getX() + dx;
                int ny = p.getY() + dy;

                neighbors.add(new GridPoint(nx, ny));
            }
        }
        return neighbors;
    }

    public String getName() {
        return name;
    }

    @Override
    public String toString(){
        return name;
    }
}