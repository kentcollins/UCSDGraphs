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
	private HashMap<GeographicPoint, List<GeographicPoint>> adjacencyList;
	private HashMap<GeographicPoint, MapNode> nodeMap;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		adjacencyList = new HashMap<>();
		nodeMap = new HashMap<>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return adjacencyList.keySet().size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		Set<GeographicPoint> vertices = new HashSet<>();
		for (GeographicPoint gp : adjacencyList.keySet()) {
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
		for (GeographicPoint gp : adjacencyList.keySet()) {
			numRoads += adjacencyList.get(gp).size();
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
		if (location == null || adjacencyList.keySet().contains(location)) {
			return false;
		}
		MapNode node = new MapNode(location);
		adjacencyList.put(node, new ArrayList<>());
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
		if (!adjacencyList.keySet().contains(from) || !adjacencyList.keySet().contains(to))
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
		adjacencyList.get(from).add(to);
	}

	private GeographicPoint getMapNode(GeographicPoint gp) {
		for (GeographicPoint p : adjacencyList.keySet()) {
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
			List<GeographicPoint> neighbors = adjacencyList.get(curr);
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
		printPath(path);
		return path;
	}

	public void printPath(List<GeographicPoint> path) {
		System.out.println("Path from " + path.get(0) + " to " + path.get(path.size() - 1));
		System.out.println(path);
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
		for (GeographicPoint gp : adjacencyList.keySet()) {
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
			for (GeographicPoint gp : adjacencyList.get(curr)) {
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
		return buildParentPath(curr, parents);
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
	 * @param goalNode
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goalNode,
			Consumer<GeographicPoint> nodeSearched) {
		int nodesVisited = 0;
		PriorityQueue<MapNode> pQueue = new PriorityQueue<>();
		HashSet<GeographicPoint> visitedNodes = new HashSet<>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		for (GeographicPoint gp : adjacencyList.keySet()) {
			((MapNode) gp).setDistanceFromStart(Double.POSITIVE_INFINITY);
			((MapNode) gp).setEstimatedCost(Double.POSITIVE_INFINITY);
		}
		MapNode currNode = nodeMap.get(start);
		pQueue.add(currNode);
		currNode.setDistanceFromStart(0);
		currNode.setEstimatedCost(currNode.getDistanceFromStart() + currNode.distance(goalNode));
		parentMap.put(currNode, null);
		while (pQueue.size() > 0) {
			currNode = ((MapNode) pQueue.remove());
			nodesVisited++;
			nodeSearched.accept(currNode);
			if (!visitedNodes.contains(currNode)) {
				visitedNodes.add(currNode);
			}
			if (currNode.equals(goalNode)) {
				break;
			}
			addNextHopsToPrioritizedQueue(visitedNodes, parentMap, pQueue, currNode, goalNode);
		}
		System.out.println("A-Star visited " + nodesVisited + " nodes");
		return buildParentPath(currNode, parentMap);
	}

	private void addNextHopsToPrioritizedQueue(HashSet<GeographicPoint> visited,
			HashMap<GeographicPoint, GeographicPoint> parents, PriorityQueue<MapNode> queue, MapNode curr,
			GeographicPoint goal) {
		addNextHopsToPrioritizedQueue(visited, parents, queue, curr, goal, null);
	}

	/**
	 * Adds hops but checks to see whether each edge endpoint comes from a
	 * forbidden road -- if so, does not add that node to the priority queue.
	 * The same node may be added later, by a different edge but it may not be
	 * added as a result of following the forbidden edge.
	 * 
	 * @param visited
	 * @param parents
	 * @param queue
	 * @param currNode
	 * @param goal
	 * @param ommitted
	 */
	private void addNextHopsToPrioritizedQueue(HashSet<GeographicPoint> visited,
			HashMap<GeographicPoint, GeographicPoint> parents, PriorityQueue<MapNode> queue, MapNode currNode,
			GeographicPoint goal, MapRoad forbidden) {
		for (GeographicPoint gp : adjacencyList.get(currNode)) {
			MapNode oneHopNode = nodeMap.get(gp);
			if (!visited.contains(oneHopNode)) {
				// a forbidden node has not been provided or one has but is not the edge
				// leading to oneHopNode
				if (forbidden == null && !currNode.getRoadByDestination(oneHopNode).equals(forbidden)) {
					double distanceThroughCurr = currNode.getDistanceFromStart() + currNode.getHopDistance(oneHopNode);
					double additionalCost = oneHopNode.distance(goal);
					if (distanceThroughCurr < oneHopNode.getDistanceFromStart()) {
						parents.put(oneHopNode, currNode);
						oneHopNode.setDistanceFromStart(distanceThroughCurr);
					}
					double estimatedTotalCost = oneHopNode.getDistanceFromStart() + additionalCost;
					if (estimatedTotalCost < oneHopNode.getEstimatedCost()) {
						oneHopNode.setEstimatedCost(estimatedTotalCost);
						queue.add(oneHopNode);
					}
				}
			}
		}
	}

	/**
	 * Given a path from an aStar search, remove sequentially each edge between
	 * start and finish nodes and determine whether an aStar search yields an
	 * alternate path when each edge is removed. Arrange resulting paths
	 * (alternate routes) from shortest to longest (lowest to highest cost)
	 *
	 * @param path
	 *            A sequence of geographic points lying along an aStar path
	 * @return A list of "next best" paths that would result if for each edge in
	 *         the path, that edge were removed from the graph.
	 */
	public List<ArrayList<GeographicPoint>> altRoutes(List<GeographicPoint> path) {
		List<MapRoad> route = buildRouteFromPath(path);
		List<ArrayList<GeographicPoint>> alts = new ArrayList<ArrayList<GeographicPoint>>();
		for (MapRoad road: route) {
			List<GeographicPoint> altPath = null;//TODO finish this thought
		}

		return alts;
	}

	private List<MapRoad> buildRouteFromPath(List<GeographicPoint> path) {
		List<MapRoad> route = new ArrayList<>();
		for (int i = 0; i < path.size() - 1; i++) {
			MapNode segmentStart = (MapNode) path.get(i);
			MapNode segmentEnd = (MapNode) path.get(i + 1);
			MapRoad roadBetween = segmentStart.getRoadByDestination(segmentEnd);
			route.add(roadBetween);
		}
		return route;
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
