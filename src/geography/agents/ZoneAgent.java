package geography.agents;

import repast.simphony.engine.schedule.ScheduledMethod;

public class ZoneAgent {

	private String name;
	//private double waterFlowRate;

	public ZoneAgent(String name){
		this.name = name;
	}

	@ScheduledMethod(start = 1, interval = 1)
	public void step() {

	}

/*
	private void checkWaterSupply(){
		Context context = ContextUtils.getContext(this);
		Geography geography = (Geography)context.getProjection("Geography");

		// Find all features that intersect the zone feature
		IntersectsQuery query = new IntersectsQuery(geography, this);
		//waterFlowRate = 0;
*/

		/**
		 * Checks if GisAgents are within a certain distance from the zone and
		 * sets the water status of the agent based on the zone's water status.
		 */
	/*
		Parameters parm = RunEnvironment.getInstance().getParameters();
		double zoneDistance = (Double)parm.getValue("zoneDistance");  // meters

		GeographyWithin within = new GeographyWithin(geography, zoneDistance, this);
		for (Object obj : within.query()) {
			if (obj instanceof GisAgent){
				// If the zone finds a GisAgent, set the agent water rate from the zone
				GisAgent agent = (GisAgent)obj;

*/
//			}
	//	}

		// An alternative approach to using GeographyWithin would be to do an
		//  IntersectsQuery on the BufferZoneAgents and check for GisAgents that
		//  intersect the BufferZone.  That would be computationally faster since
		//  the GeographyWithin internally creates a buffer based on the distance
		//  each time.
//	}


	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}
}