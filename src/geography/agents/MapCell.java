package geography.agents;

public class MapCell {
    public static final int TYPE_UNKNOWN = 0;
    public static final int TYPE_INITIAL_ZONE = 1;
    public static final int TYPE_SAFE_ZONE = 2;
    public static final int TYPE_ROAD = 3;

    private int type;

    public MapCell() {
        this.type = TYPE_UNKNOWN; // no es transitable
    }

    public void setType(int type) {
        this.type = type;
    }

    public int getType() {
        return type;
    }

    public boolean isTraversable() {
        // Un agente SOLO puede moverse por las carreteras (TYPE_ROAD) o dentro de la zona segura (TYPE_SAFE_ZONE).
        // Cualquier otro tipo de celda (UNKNOWN, INITIAL_ZONE si no está en carretera/segura) NO es transitable.
        return type == TYPE_ROAD || type == TYPE_SAFE_ZONE;
    }
}