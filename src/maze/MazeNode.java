package maze;

import java.util.ArrayList;
import java.util.List;

public class MazeNode {

	private List<MazeNode> neighbors;
	private char drawChar = '-';
	
	public MazeNode(){
		neighbors = new ArrayList<MazeNode>();
	}
	
	public void addNeighbor(MazeNode m) {
		if (neighbors.contains(m)) return;
		neighbors.add(m);
	}
	
	public void setDrawChar(char c) {
		drawChar = c;
	}
	
	public char getDrawChar() {
		return drawChar;
	}
	
	public void pair(MazeNode other) {
		if (other!=null) {
			addNeighbor(other);
			other.addNeighbor(this);
		}
	}
	
	public List<MazeNode> getNeighbors() {
		return neighbors;
	}

}
