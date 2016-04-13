package maze;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

public class MazeGraph {
	MazeNode[][] nodes;

	public MazeGraph(int r, int c) {
		nodes = new MazeNode[r][c];
	}

	public MazeNode addNode(int r, int c, MazeNode n) {
		MazeNode existing = nodes[r][c];
		nodes[r][c] = n;
		return existing; // may be null
	}

	public MazeNode getNode(int r, int c) {
		return nodes[r][c];
	}

	public void printMazeGraph() {
		for (int r = 0; r < nodes.length; r++) {
			for (int c = 0; c < nodes[r].length; c++) {
				if (nodes[r][c] != null)
					System.out.print(nodes[r][c].getDrawChar());
				else
					System.out.print('X');
			}
			System.out.println();
		}
	}

	public List<MazeNode> bfs(MazeNode start, MazeNode goal) {
		if (start == null || goal == null) return null;
		MazeNode curr = start;
		List<MazeNode> queue = new LinkedList<MazeNode>();
		HashSet<MazeNode> visited = new HashSet<MazeNode>();
		HashMap<MazeNode,MazeNode> parents = new HashMap<MazeNode, MazeNode>();
		queue.add(curr);
		parents.put(curr, null);
		while (queue.size()>0) {
			curr = queue.remove(0);
			visited.add(curr);
			for (MazeNode m:curr.getNeighbors()) {
				if (!visited.contains(m)) {
					queue.add(m);
					parents.put(m, curr);
				}
			}
			if (curr.equals(goal)) {
				List<MazeNode> path = new LinkedList<MazeNode>();
				path.add(goal);
				while(!curr.equals(start)){
					MazeNode parent = getParent(curr, parents);
					path.add(0, parent);
					curr = parent;
				}
				return path;
			}
		}
		return null;
	}
	
	public void markPath(List<MazeNode> path) {
		path.get(0).setDrawChar('S');
		path.get(path.size()-1).setDrawChar('G');
		for (int i = 1; i<path.size()-1; i++) {
			path.get(i).setDrawChar('o');
		}
	}
	
	private MazeNode getParent(MazeNode m, HashMap<MazeNode, MazeNode> parents) {
		if (parents.containsKey(m)) return parents.get(m);
		return null;
	}
}
