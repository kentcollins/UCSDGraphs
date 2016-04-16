/** 
 * Models a road having a name, type and length constructed between to geographic end points.
 * 
 */
package roadgraph;

import geography.GeographicPoint;

public class MapRoad {
	GeographicPoint from, to;
	String roadName;
	String roadType;
	double length;
	
	public MapRoad(GeographicPoint start, GeographicPoint finish, String roadName,
			String roadType, double length){
		from = start;
		to = finish;
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
		if (from.equals(gp)) return to;
		if (to.equals(gp)) return from;
		return null;
	}

	public GeographicPoint getFrom(){
		return from;
	}
	
	public GeographicPoint getTo(){
		return to;
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
		sb.append("[Road from: "+from+" to "+to+"]");
		return sb.toString();
	}
	
	

}
