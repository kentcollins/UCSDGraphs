/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between roads
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapGraph {
	// 
	private HashMap<GeographicPoint, List<GeographicPoint>> adjList;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		adjList = new HashMap<>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return adjList.keySet().size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		Set<GeographicPoint> vertices = new HashSet<>();
		for (GeographicPoint gp:adjList.keySet()) {
			vertices.add(gp);
		}
		return vertices;
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		// Access each intersection, accumulate the number of outbound roads
		int numRoads = 0;
		for (GeographicPoint gp : adjList.keySet()) {
			numRoads += adjList.get(gp).size();
		}
		return numRoads;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (location == null || adjList.keySet().contains(location)) {
			return false;
		}
		GeographicPoint gp = new MapNode(location);
		adjList.put(gp, new ArrayList<>());
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {
		// Check for conditions that should throw the requested exception
		if (!adjList.keySet().contains(from) || !adjList.keySet().contains(to))
			throw new IllegalArgumentException("One or both road endpoints not already a known node");
		if (from == null || to == null) 
			throw new IllegalArgumentException("Road endpoints may not be null");
		if (roadName == null || roadType == null)
			throw new IllegalArgumentException("Road properties may not be null");
		if (length < 0)
			throw new IllegalArgumentException("Roads cannot have a negative length");
		// Construct a new road based on the information provided
		MapRoad road = new MapRoad(from, to, roadName, roadType, length);
		((MapNode) getMapNode(from)).addRoad(road);
		adjList.get(from).add(to);
	}
	
	private GeographicPoint getMapNode(GeographicPoint gp) {
		for (GeographicPoint p:adjList.keySet()) {
			if (gp.equals(p)) return p;
		}
		return null;
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		List<GeographicPoint> queue = new LinkedList<>();
		HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		GeographicPoint curr = start;
		parents.put(curr, null);
		queue.add(curr);
		System.out.println("Added "+curr+" to queue.");
		while (queue.size() > 0 && !curr.equals(goal)) {
			curr = queue.remove(0);
			visited.add(curr);
			// provide the current node to our consumer for drawing
			nodeSearched.accept(curr);
			List<GeographicPoint> neighbors = adjList.get(curr);
			for (GeographicPoint node : neighbors) {
				if (!visited.contains(node)) {
					queue.add(node);
					System.out.println("Added "+node+" to queue.");
					parents.put(node, curr);
				}

			}
		}
		System.out.println("Finished BFS");
		if (!curr.equals(goal))
			return null;
		else return buildParentPath(curr, parents);
	}

	/** Helper method to build a path from a map of node parents.
	 * 
	 * @param child the initial node for which a parent is required
	 * @param parents mapping of each node to its parent
	 * @return ordered list beginning with ultimate parent and leading to original child
	 */
	private List<GeographicPoint> buildParentPath(GeographicPoint child, HashMap<GeographicPoint, GeographicPoint> parents ) {
		List<GeographicPoint> path = new LinkedList<>();
		path.add(0, child);
		while (true) {
			child = parents.get(child); 
			if (child!=null)	path.add(0, child); 
			else {break;}
		}
		return path;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		List<GeographicPoint> path = new LinkedList<>();
		// nodeSearched.accept(next.getLocation());

		return path;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		return null;
	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");

		// You can use this method for testing.

		/*
		 * Use this code in Week 3 End of Week Quiz MapGraph theMap = new
		 * MapGraph(); System.out.print("DONE. \nLoading the map...");
		 * GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		 * System.out.println("DONE.");
		 * 
		 * GeographicPoint start = new GeographicPoint(32.8648772,
		 * -117.2254046); GeographicPoint end = new GeographicPoint(32.8660691,
		 * -117.217393);
		 * 
		 * 
		 * List<GeographicPoint> route = theMap.dijkstra(start,end);
		 * List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		 * 
		 */

	}

}
