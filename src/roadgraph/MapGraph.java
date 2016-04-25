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
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapGraph {

	//
	private HashMap<GeographicPoint, List<GeographicPoint>> adjList;
	private HashMap<GeographicPoint, MapNode> nodeMap;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		adjList = new HashMap<>();
		nodeMap = new HashMap<>();
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
		for (GeographicPoint gp : adjList.keySet()) {
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
		MapNode node = new MapNode(location);
		adjList.put(node, new ArrayList<>());
		nodeMap.put(location, node);
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
		for (GeographicPoint p : adjList.keySet()) {
			if (gp.equals(p))
				return p;
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
		while (queue.size() > 0 && !curr.equals(goal)) {
			curr = queue.remove(0);
			// provide the current node to our consumer for drawing
			nodeSearched.accept(curr);
			visited.add(curr);
			List<GeographicPoint> neighbors = adjList.get(curr);
			for (GeographicPoint node : neighbors) {
				if (!visited.contains(node)) {
					queue.add(node);
					parents.put(node, curr);
				}

			}
		}
		if (!curr.equals(goal))
			return null;
		else
			return buildParentPath(curr, parents);
	}

	/**
	 * Helper method to build a path from a map of node parents.
	 * 
	 * @param child
	 *            the initial node for which a parent is required
	 * @param parents
	 *            mapping of each node to its parent
	 * @return ordered list beginning with ultimate parent and leading to
	 *         original child
	 */
	private List<GeographicPoint> buildParentPath(GeographicPoint child,
			HashMap<GeographicPoint, GeographicPoint> parents) {
		List<GeographicPoint> path = new LinkedList<>();
		path.add(0, child);
		while (true) {
			child = parents.get(child);
			if (child != null)
				path.add(0, child);
			else {
				break;
			}
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
		int numVisited = 0;
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		HashSet<GeographicPoint> visited = new HashSet<>();
		HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<>();
		for (GeographicPoint gp : adjList.keySet()) {
			((MapNode) gp).setDistanceFromStart(Double.POSITIVE_INFINITY);
		}
		MapNode curr = nodeMap.get(start);
		queue.add(curr);
		curr.setDistanceFromStart(0);
		parents.put(curr, null);
		while (queue.size() > 0) {
			curr = ((MapNode) queue.remove());
			numVisited++;
			nodeSearched.accept(curr);
			if (!visited.contains(curr)) {
				visited.add(curr);
			}
			if (curr.equals(goal)) {
				break;
			}
			for (GeographicPoint gp : adjList.get(curr)) {
				MapNode node = nodeMap.get(gp);
				if (!visited.contains(node)) {
					double distanceThroughCurr = curr.getDistanceFromStart() + curr.getHopDistance(node);
					if (distanceThroughCurr < node.getDistanceFromStart()) {
						parents.put(node, curr);
						node.setDistanceFromStart(distanceThroughCurr);
						node.setEstimatedCost(node.getDistanceFromStart());
						queue.add(node);
					}
				}
			}
		}
		System.out.println("Dijkstra visited " + numVisited + " nodes");
		return buildParentPath(goal, parents);
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
		int numVisited = 0;
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		HashSet<GeographicPoint> visited = new HashSet<>();
		HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<>();
		for (GeographicPoint gp : adjList.keySet()) {
			((MapNode) gp).setDistanceFromStart(Double.POSITIVE_INFINITY);
			((MapNode) gp).setEstimatedCost(Double.POSITIVE_INFINITY);
		}
		MapNode curr = nodeMap.get(start);
		queue.add(curr);
		curr.setDistanceFromStart(0);
		curr.setEstimatedCost(curr.getDistanceFromStart() + curr.distance(goal));
		parents.put(curr, null);
		while (queue.size() > 0) {
			curr = ((MapNode) queue.remove());
			numVisited++;
			nodeSearched.accept(curr);
			if (!visited.contains(curr)) {
				visited.add(curr);
			}
			if (curr.equals(goal)) {
				break;
			}
			for (GeographicPoint gp : adjList.get(curr)) {
				MapNode node = nodeMap.get(gp);
				if (!visited.contains(node)) {
					double distanceThroughCurr = curr.getDistanceFromStart() + curr.getHopDistance(node);
					double additionalCost = node.distance(goal);
					if (distanceThroughCurr < node.getDistanceFromStart()) {
						parents.put(node, curr);
						node.setDistanceFromStart(distanceThroughCurr);
					}
					double estimatedTotalCost = node.getDistanceFromStart() + additionalCost;
					if (estimatedTotalCost < node.getEstimatedCost()) {
						node.setEstimatedCost(estimatedTotalCost);
						queue.add(node);
					}

				}
			}
		}
		System.out.println("A-Star visited " + numVisited + " nodes");
		return buildParentPath(goal, parents);
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
	public List<List<GeographicPoint>> nPaths(GeographicPoint start, GeographicPoint goal, int numToFind,
			Consumer<GeographicPoint> nodeSearched) {
		List<List<GeographicPoint>> paths = new ArrayList<List<GeographicPoint>>();
		int numSoFar = 0;
		boolean foundAnother = true;
		while (numSoFar < numToFind && foundAnother) {
			int numVisited = 0;
			PriorityQueue<MapNode> queue = new PriorityQueue<>();
			HashSet<GeographicPoint> visited = new HashSet<>();
			HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<>();
			for (GeographicPoint gp : adjList.keySet()) {
				((MapNode) gp).setDistanceFromStart(Double.POSITIVE_INFINITY);
				((MapNode) gp).setEstimatedCost(Double.POSITIVE_INFINITY);
			}
			MapNode curr = nodeMap.get(start);
			queue.add(curr);
			curr.setDistanceFromStart(0);
			curr.setEstimatedCost(curr.getDistanceFromStart() + curr.distance(goal));
			parents.put(curr, null);
			while (queue.size() > 0) {
				curr = ((MapNode) queue.remove());
				numVisited++;
				nodeSearched.accept(curr);
				if (!visited.contains(curr)) {
					visited.add(curr);
				}
				if (curr.equals(goal)) {
					break;
				}
				for (GeographicPoint gp : adjList.get(curr)) {
					MapNode node = nodeMap.get(gp);
					if (!visited.contains(node)) {
						double distanceThroughCurr = curr.getDistanceFromStart() + curr.getHopDistance(node);
						double additionalCost = node.distance(goal);
						if (distanceThroughCurr < node.getDistanceFromStart()) {
							parents.put(node, curr);
							node.setDistanceFromStart(distanceThroughCurr);
						}
						double estimatedTotalCost = node.getDistanceFromStart() + additionalCost;
						if (estimatedTotalCost < node.getEstimatedCost()) {
							node.setEstimatedCost(estimatedTotalCost);
							queue.add(node);
						}

					}
				}
			}
			paths.add(buildParentPath(goal, parents));
		}
		return paths;
	}

	public static void main(String[] args) {

		/*
		 * Use this code in Week 3 End of Week Quiz
		 */
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start, end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start, end);

	}

}
