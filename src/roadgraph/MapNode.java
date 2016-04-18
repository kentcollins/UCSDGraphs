/**
 * Extends the concept of a geographic point to include as data the actual
 * accumulated distance to this point from an arbitrary starting point.
 * 
 * Maintains a list of roads/edges leading away from this point.
 * 
 */

package roadgraph;

import java.util.HashMap;
import java.util.HashSet;

import geography.GeographicPoint;

@SuppressWarnings("serial")
public class MapNode extends geography.GeographicPoint implements Comparable<MapNode> {

	private double distanceFromStart;
	private double estimatedCost;
	private HashSet<MapRoad> roads;
	private HashMap<MapNode, java.lang.Double> hopDistances;

	/**
	 * Build a map node from an existing geographic point
	 * 
	 * @param gp
	 */
	public MapNode(GeographicPoint gp) {
		super(gp.getX(), gp.getY());
		roads = new HashSet<>();
		hopDistances = new HashMap<>();
	}

	public boolean addRoad(MapRoad road) {
		if (roads.contains(road) || road == null)
			return false;
		if (!road.getFrom().equals(this))
			throw (new IllegalArgumentException("Assigning road to wrong map node."));
		roads.add(road);
		MapNode destination = ((MapNode) road.getTo());
		java.lang.Double distance = road.getLength();
		hopDistances.put(destination, distance);
		return true;
	}
	
	public HashSet<MapRoad> getRoads() {
		return roads;
	}
	
	public void setDistanceFromStart(double d) {
		this.distanceFromStart = d;
	}
	
	public double getDistanceFromStart() {
		return this.distanceFromStart;
	}

	public double getEstimatedCost() {
		return estimatedCost;
	}
	
	public void setEstimatedCost(double d) {
		this.estimatedCost =  d;
	}

	/**
	 * Natural ordering rules for instances of this class.
	 * 
	 * @param other
	 *            a second MapNode, for comparison
	 * @return -1 if this should precede other, +1 if this should come after
	 *         other, 0 if they are equally ordered
	 */
	public int compareTo(MapNode other) {
		if (this.estimatedCost < other.estimatedCost)
			return -1;
		if (this.estimatedCost > other.estimatedCost)
			return +1;
		return 0;
	}
	
	public String toString(){
		return "Node("+getX()+","+getY()+")";
	}

	public double getHopDistance(MapNode node) {
		return hopDistances.get(node);
	}
	
	public int hashCode() {
		return super.hashCode();
	}

}
