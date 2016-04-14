package roadgraph;

import geography.GeographicPoint;

public class Road {
	GeographicPoint[] endpoints;
	String roadName;
	String roadType;
	double length;
	
	public Road(GeographicPoint start, GeographicPoint finish, String roadName,
			String roadType, double length){
		endpoints = new GeographicPoint[] {start, finish};
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	public GeographicPoint getOtherEnd(GeographicPoint gp) {
		if (endpoints[0].equals(gp)) return endpoints[1];
		return endpoints[0];
	}

	public GeographicPoint[] getEndpoints() {
		return endpoints;
	}

	public String getRoadName() {
		return roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public double getLength() {
		return length;
	}
	
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("[Road from: "+endpoints[0]+" to "+endpoints[1]+"]");
		return sb.toString();
		
	}
	
	

}
