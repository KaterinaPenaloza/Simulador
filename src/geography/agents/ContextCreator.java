package geography.agents;

import java.io.File;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import org.geotools.data.shapefile.ShapefileDataStore;
import org.geotools.data.simple.SimpleFeatureIterator;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.MultiLineString;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.opengis.feature.simple.SimpleFeature;

import repast.simphony.context.Context;
import repast.simphony.context.space.gis.GeographyFactoryFinder;
import repast.simphony.context.space.grid.GridFactoryFinder;
import repast.simphony.dataLoader.ContextBuilder;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.parameter.Parameters;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.gis.GeographyParameters;
import repast.simphony.space.grid.Grid;
import repast.simphony.space.grid.GridBuilderParameters;
import repast.simphony.space.grid.GridPoint;
import repast.simphony.space.grid.SimpleGridAdder;
import repast.simphony.space.grid.StickyBorders;


@SuppressWarnings("rawtypes")
public class ContextCreator implements ContextBuilder {
    int numAgents;
    double zoneDistance;
    private ZoneAgent initialZone;
    private ZoneAgent safeZone;

    // Grilla
    private static final int GRID_WIDTH = 400; //cambiar segun estandar
    private static final int GRID_HEIGHT = 400;
    
    private static final double MIN_LONGITUDE = -71.55435632438811;
    private static final double MAX_LONGITUDE = -71.54257547377763;
    private static final double MIN_LATITUDE = -33.026878836652145;
    private static final double MAX_LATITUDE = -33.00825336021646;

    private static Grid<MapCell> mapCellGrid;
    private static Grid<GisAgent> agentGrid;

    // Conversión de coordenadas geográficas a coordenadas de grilla
    public static GridPoint mapGeoToGrid(Coordinate geoCoord) {
        double normLong = (geoCoord.x - MIN_LONGITUDE) / (MAX_LONGITUDE - MIN_LONGITUDE);
        double normLat = (geoCoord.y - MIN_LATITUDE) / (MAX_LATITUDE - MIN_LATITUDE);

        int gridX = (int) (normLong * GRID_WIDTH);
        int gridY = (int) (normLat * GRID_HEIGHT);

        gridX = Math.max(0, Math.min(gridX, GRID_WIDTH - 1));
        gridY = Math.max(0, Math.min(gridY, GRID_HEIGHT - 1));

        return new GridPoint(gridX, gridY);
    }

    // Mapear de coordenadas de grilla a coordenadas geográficas
    public static Coordinate mapGridToGeo(int gridX, int gridY) {
        double longStep = (MAX_LONGITUDE - MIN_LONGITUDE) / GRID_WIDTH;
        double latStep = (MAX_LATITUDE - MIN_LATITUDE) / GRID_HEIGHT;

        double geoLong = MIN_LONGITUDE + (gridX * longStep) + (longStep / 2.0);
        double geoLat = MIN_LATITUDE + (gridY * latStep) + (latStep / 2.0);
        return new Coordinate(geoLong, geoLat);
    }

    // Obtener MapCell
    public static MapCell getMapCell(int x, int y) {
        if (mapCellGrid != null && x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
            return mapCellGrid.getObjectAt(x, y);
        }
        return null;
    }


    @Override
    public Context build(Context context) {
        Parameters parm = RunEnvironment.getInstance().getParameters();
        numAgents = (Integer)parm.getValue("numAgents");
        zoneDistance = (Double)parm.getValue("zoneDistance");

        GeographyParameters geoParams = new GeographyParameters();
        Geography geography = GeographyFactoryFinder.createGeographyFactory(null).createGeography("Geography", context, geoParams);
        GeometryFactory fac = new GeometryFactory();

        // GRILLA DE MAPCELLS (TERRENO)
        // singleOccupancy2D: cada celda de la grilla solo tendrá UN MapCell
        // multiOccupancy2D: múltiples objetos por celda
        mapCellGrid = GridFactoryFinder.createGridFactory(null).createGrid("MapCellGrid", context,
            GridBuilderParameters.singleOccupancy2D(
                new SimpleGridAdder<MapCell>(),
                new StickyBorders(),
                GRID_WIDTH, GRID_HEIGHT
            )
        );
        System.out.println("DEBUG: Grilla MapCells creada.");

        for (int x = 0; x < GRID_WIDTH; x++) {
            for (int y = 0; y < GRID_HEIGHT; y++) {
                MapCell cell = new MapCell(); // Inicialmente no transitables
                context.add(cell);
                mapCellGrid.moveTo(cell, x, y);
            }
        }

        // INICIALIZACIÓN DE LA GRILLA PARA AGENTES
        agentGrid = GridFactoryFinder.createGridFactory(null).createGrid("AgentGrid", context,
            GridBuilderParameters.multiOccupancy2D(
                new SimpleGridAdder<GisAgent>(),
                new StickyBorders(),
                GRID_WIDTH, GRID_HEIGHT
            )
        );
        System.out.println("DEBUG: Grilla para Agentes creada.");


        // Carga de las Zonas
        System.out.println("DEBUG: Cargando el shapefile de zonas y marcando celdas en la grilla de MapCells...");

        // Zona inicial polygon.shp
        String initialZoneFilename = "./data/POLYGON.shp";
        List<SimpleFeature> featuresForInitialZone = loadFeaturesFromShapefile(initialZoneFilename);

        if (featuresForInitialZone.isEmpty()) {
            System.err.println("ERROR: El shapefile " + initialZoneFilename + " está vacío. No se puede definir la zona inicial.");
            throw new IllegalStateException(initialZoneFilename + " está vacío.");
        }
        Geometry rawInitialGeometry = (Geometry)featuresForInitialZone.iterator().next().getDefaultGeometry();
        Geometry initialGeometryToUse = rawInitialGeometry;

        if (rawInitialGeometry instanceof MultiPolygon) {
            MultiPolygon mp = (MultiPolygon)rawInitialGeometry;
            if (mp.getNumGeometries() > 0) {
                Geometry firstPart = mp.getGeometryN(0);
                if (firstPart instanceof Polygon) {
                    initialGeometryToUse = firstPart;
                }
            }
        }
        this.initialZone = new ZoneAgent("initial");
        context.add(this.initialZone);
        geography.move(this.initialZone, initialGeometryToUse);

        markGridCellsForGeometry(initialGeometryToUse, MapCell.TYPE_INITIAL_ZONE);
        System.out.println("DEBUG: Celdas para zona inicial marcadas.");

        // Zona segura Zones2.shp
        String safeZoneFilename = "./data/Zones2.shp";
        List<SimpleFeature> featuresForSafeZone = loadFeaturesFromShapefile(safeZoneFilename);

        if (featuresForSafeZone.isEmpty()) {
            System.err.println("ERROR: El shapefile " + safeZoneFilename + " está vacío. No se puede definir la zona segura.");
            throw new IllegalStateException(safeZoneFilename + " está vacío.");
        }

        Geometry rawSafeGeometry = (Geometry)featuresForSafeZone.iterator().next().getDefaultGeometry();
        Geometry safeGeometryToUse = rawSafeGeometry;

        if (rawSafeGeometry instanceof MultiPolygon) {
            MultiPolygon mp = (MultiPolygon)rawSafeGeometry;
            if (mp.getNumGeometries() > 0) {
                Geometry firstPart = mp.getGeometryN(0);
                if (firstPart instanceof Polygon) {
                    safeGeometryToUse = firstPart;
                }
            }
            if (safeGeometryToUse == null) {
                System.err.println("Advertencia: MultiPolygon de zona segura vacío o no contiene Polygons. Usando MultiPolygon completo si es válido.");
                safeGeometryToUse = rawSafeGeometry;
            }
        }
        this.safeZone = new ZoneAgent("safe");
        context.add(this.safeZone);
        geography.move(this.safeZone, safeGeometryToUse);

        markGridCellsForGeometry(safeGeometryToUse, MapCell.TYPE_SAFE_ZONE);
        System.out.println("DEBUG: Celdas para zona segura marcadas.");

        // Carga de las Carreteras
        System.out.println("DEBUG: Cargando el shapefile de carreteras y marcando celdas en la grilla de MapCells...");
        String roadsShapefile = "./data/roads.shp";
        List<SimpleFeature> roadFeatures = loadFeaturesFromShapefile(roadsShapefile);

        if (roadFeatures.isEmpty()) {
            System.err.println("ERROR: El shapefile " + roadsShapefile + " está vacío. No se puede marcar las carreteras en la grilla.");
        } else {
            for (SimpleFeature feature : roadFeatures) {
                Geometry geom = (Geometry)feature.getDefaultGeometry();

                if (geom instanceof LineString) {
                    markGridCellsForLineString((LineString) geom, MapCell.TYPE_ROAD);
                } else if (geom instanceof MultiLineString) {
                    MultiLineString mls = (MultiLineString) geom;
                    for (int i = 0; i < mls.getNumGeometries(); i++) {
                        Geometry singleGeom = mls.getGeometryN(i);
                        if (singleGeom instanceof LineString) {
                            markGridCellsForLineString((LineString) singleGeom, MapCell.TYPE_ROAD);
                        } else {
                            System.err.println("Advertencia: Geometría inesperada dentro de MultiLineString en " + roadsShapefile + ": " + singleGeom.getGeometryType());
                        }
                    }
                } else {
                    System.err.println("Advertencia: Geometría inesperada en " + roadsShapefile + ": " + geom.getGeometryType() + ". Se esperaba LineString o MultiLineString.");
                }
            }
            System.out.println("DEBUG: Celdas para carreteras marcadas.");
        }

        // Inicialización de Agentes en la Grilla
        System.out.println("DEBUG: Iniciando creación de agentes.");
        if (initialZone == null || geography.getGeometry(initialZone) == null) {
            System.err.println("ERROR: La 'initial' zone no se pudo crear o su geometría no está asignada. Los agentes no se podrán crear.");
            throw new IllegalStateException("Initial Zone no creada o sin geometría.");
        }
        if (safeZone == null || geography.getGeometry(safeZone) == null) {
            System.err.println("ERROR: La 'safe' zone no se pudo encontrar o su geometría no está asignada.");
            throw new IllegalStateException("Safe Zone no encontrada o sin geometría.");
        }

        // Marcar zonas transitables
        List<GridPoint> initialZoneTraversableCells = new ArrayList<>();
        Geometry initialZoneGeometry = geography.getGeometry(initialZone);

        for (int x = 0; x < GRID_WIDTH; x++) {
            for (int y = 0; y < GRID_HEIGHT; y++) {
                MapCell cell = mapCellGrid.getObjectAt(x,y);
                Coordinate cellCenterGeoCoord = mapGridToGeo(x, y);
                Point cellCenterPoint = fac.createPoint(cellCenterGeoCoord);

                if (cell != null && cell.isTraversable() && initialZoneGeometry.contains(cellCenterPoint)) {
                    initialZoneTraversableCells.add(new GridPoint(x, y));
                }
            }
        }

        if (initialZoneTraversableCells.isEmpty()) {
            System.err.println("ERROR: No se encontraron celdas transitables dentro de la zona inicial. Los agentes no se pueden posicionar.");
            throw new IllegalStateException("No hay celdas transitables en la zona inicial.");
        }

        System.out.println("DEBUG: Encontradas " + initialZoneTraversableCells.size() + " celdas transitables en la zona inicial.");

        int cnt=0;
        for (int i = 0; i < numAgents; i++) {
            GridPoint agentStartGridPoint = initialZoneTraversableCells.get(
                repast.simphony.random.RandomHelper.nextIntFromTo(0, initialZoneTraversableCells.size() - 1)
            );

            GisAgent agent = new GisAgent("Site " + cnt, safeZone, mapCellGrid, agentGrid);
            context.add(agent);
            agentGrid.moveTo(agent, agentStartGridPoint.getX(), agentStartGridPoint.getY());

            Coordinate geoCoord = mapGridToGeo(agentStartGridPoint.getX(), agentStartGridPoint.getY());
            geography.move(agent, fac.createPoint(geoCoord));
            cnt++;
        }

        return context;
    }

    private void markGridCellsForGeometry(Geometry geometry, int cellType) {
        GeometryFactory fac = new GeometryFactory();

        for (int x = 0; x < GRID_WIDTH; x++) {
            for (int y = 0; y < GRID_HEIGHT; y++) {
                MapCell cell = mapCellGrid.getObjectAt(x,y);
                if (cell != null) {
                    Coordinate cellCenterGeoCoord = mapGridToGeo(x, y);
                    Point cellCenterPoint = fac.createPoint(cellCenterGeoCoord);
                    if (geometry.contains(cellCenterPoint) || geometry.intersects(cellCenterPoint)) {
                        cell.setType(cellType);
                    }
                }
            }
        }
    }

    // Marca las celdas de la grilla que son calles
    private void markGridCellsForLineString(LineString line, int cellType) {
        Coordinate[] coords = line.getCoordinates();
        if (coords.length < 2) {
			return;
		}

        GridPoint startGp = mapGeoToGrid(coords[0]);
        if (startGp.getX() >= 0 && startGp.getX() < GRID_WIDTH && startGp.getY() >= 0 && startGp.getY() < GRID_HEIGHT) {
            MapCell cell = mapCellGrid.getObjectAt(startGp.getX(), startGp.getY());
            if (cell != null) {
                cell.setType(cellType);
            }
        }

        // Marcar celdas para cada segmento de línea
        for (int i = 0; i < coords.length - 1; i++) {
            Coordinate p1 = coords[i];
            Coordinate p2 = coords[i+1];

            GridPoint gp1 = mapGeoToGrid(p1);
            GridPoint gp2 = mapGeoToGrid(p2);

            int minX = Math.min(gp1.getX(), gp2.getX());
            int maxX = Math.max(gp1.getX(), gp2.getX());
            int minY = Math.min(gp1.getY(), gp2.getY());
            int maxY = Math.max(gp1.getY(), gp2.getY());

            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
                        MapCell cell = mapCellGrid.getObjectAt(x, y);
                        if (cell != null) {
                            cell.setType(cellType);
                        }
                    }
                }
            }
        }
    }

    // Carga de features shp
    private List<SimpleFeature> loadFeaturesFromShapefile(String filename){
        URL url = null;
        try {
            url = new File(filename).toURI().toURL();
        } catch (MalformedURLException e1) {
            e1.printStackTrace();
        }

        List<SimpleFeature> features = new ArrayList<>();

        SimpleFeatureIterator fiter = null;
        ShapefileDataStore store = null;
        try {
            store = new ShapefileDataStore(url);
            fiter = store.getFeatureSource().getFeatures().features();

            while(fiter.hasNext()){
                features.add(fiter.next());
            }
        } catch (IOException e) {
            System.err.println("Error al cargar el shapefile " + filename + ": " + e.getMessage());
            e.printStackTrace();
        }
        finally{
            if (fiter != null) {
                fiter.close();
            }
            if (store != null) {
                store.dispose();
            }
        }

        return features;
    }

    public ZoneAgent getInitialZone() {
        return initialZone;
    }

    public ZoneAgent getSafeZone() {
        return safeZone;
    }
}