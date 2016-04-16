/**
 * Extends the concept of a geographic point to include as data the actual
 * accumulated distance to this point from an arbitrary starting point.
 * 
 * Maintains a list of roads/edges leading away from this point.
 * 
 */

package roadgraph;

import geography.GeographicPoint;

@SuppressWarnings("serial")
public class MapNode extends geography.GeographicPoint implements Comparable<MapNode>{
	
	private double distanceFromStart=java.lang.Double.POSITIVE_INFINITY;
	
	/** Create a map node an existing geographic point
	 * 
	 * @param gp
	 */
	public MapNode(GeographicPoint gp) {
		super(gp.getX(), gp.getY());
	}
	
	/** Natural ordering rules for instances of this class.
	 * 
	 * @param other a second MapNode, for comparison
	 * @return -1 if this should precede other, +1 if this should come
	 * after other, 0 if they are equally ordered
	 */
	public int compareTo(MapNode other) {
		if (this.distanceFromStart<other.distanceFromStart) return -1;
		if (this.distanceFromStart>other.distanceFromStart) return +1;
		return 0;
	}
	
	

}
