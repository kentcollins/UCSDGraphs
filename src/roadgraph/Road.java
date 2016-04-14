/** 
 * Models a road having a name, type and length constructed between to geographic end points.
 * 
 */
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
	
	/** Determine vertex at other end.
	 * 
	 * @param gp the vertex we already know, one of this road's end points.
	 * @return the opposing vertex or null if the parameter is not an end point.
	 */
	public GeographicPoint getOtherEnd(GeographicPoint gp) {
		if (endpoints[0].equals(gp)) return endpoints[1];
		if (endpoints[1].equals(gp)) return endpoints[0];
		return null;
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
