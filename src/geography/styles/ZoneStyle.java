package geography.styles;

import java.awt.Color;

import geography.agents.ZoneAgent;
import gov.nasa.worldwind.render.SurfacePolygon;
import gov.nasa.worldwind.render.SurfaceShape;
import repast.simphony.visualization.gis3D.style.SurfaceShapeStyle;


public class ZoneStyle implements SurfaceShapeStyle<ZoneAgent>{

	@Override
	public SurfaceShape getSurfaceShape(ZoneAgent object, SurfaceShape shape) {
		return new SurfacePolygon();
	}

	@Override
	public Color getFillColor(ZoneAgent zone) {
		return Color.CYAN;
	}

	@Override
	public double getFillOpacity(ZoneAgent obj) {
		return 0.20;
	}

	@Override
	public Color getLineColor(ZoneAgent obj) {
	    return Color.BLACK;
	}

	@Override
	public double getLineOpacity(ZoneAgent obj) {
	    return 0.2;
	}

	@Override
	public double getLineWidth(ZoneAgent obj) {
	    return 0.2;
	}
}