package maze;

import java.util.ArrayList;
import java.util.List;

public class MazeGraph {
	MazeNode[][] nodes;
	
	public MazeGraph(int r, int c) {
		nodes = new MazeNode[r][c];
	}
	
	public MazeNode addNode(int r, int c, MazeNode n) {
		MazeNode existing = nodes[r][c];
		nodes[r][c] = n;
		return existing; //  may be null
	}
	
	public MazeNode getNode(int r, int c) {
		return nodes[r][c];
	}
	
	public void printMazeGraph() {
		for (int r = 0; r<nodes.length; r++) {
			for (int c = 0; c<nodes[r].length; c++) {
				if (nodes[r][c]!=null) System.out.print(nodes[r][c].getDrawChar());
				else System.out.print('X');
			}
			System.out.println();
		}
	}
	
	public List<MazeNode> bfs(MazeNode start, MazeNode goal) {
		if (start==null||goal==null) return null;
		List<MazeNode> path = new ArrayList<MazeNode>();
		start.setDrawChar('S');
		goal.setDrawChar('G');
		return path;
	}
}
