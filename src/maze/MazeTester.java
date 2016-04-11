package maze;

public class MazeTester {

	public static void main(String[] args) {
		MazeGraph graph = new MazeGraph(4, 4);
		graph.addNode(0, 0, new MazeNode());
		graph.addNode(0, 1, new MazeNode());
		graph.addNode(0, 2, new MazeNode());
		graph.addNode(0, 3, new MazeNode());
		graph.addNode(1, 0, new MazeNode());
		graph.addNode(1, 1, new MazeNode());
		graph.addNode(1, 3, new MazeNode());
		graph.addNode(2, 0, new MazeNode());
		graph.addNode(2, 3, new MazeNode());
		graph.addNode(3, 0, new MazeNode());
		graph.addNode(3, 1, new MazeNode());
		graph.addNode(3, 2, new MazeNode());
		graph.addNode(3, 3, new MazeNode());
		// pair horizontally
		graph.getNode(0, 0).pair(graph.getNode(0, 1));
		graph.getNode(0, 1).pair(graph.getNode(0, 2));
		graph.getNode(0, 2).pair(graph.getNode(0, 3));
		graph.getNode(1, 0).pair(graph.getNode(1, 1));
		graph.getNode(3, 0).pair(graph.getNode(3, 1));
		graph.getNode(3, 1).pair(graph.getNode(3, 2));
		graph.getNode(3, 2).pair(graph.getNode(3, 3));
		// pair vertically
		graph.getNode(0, 0).pair(graph.getNode(1, 0));
		graph.getNode(0, 1).pair(graph.getNode(1, 1));
		graph.getNode(0, 3).pair(graph.getNode(1, 3));
		graph.getNode(1, 0).pair(graph.getNode(2, 0));
		graph.getNode(1, 3).pair(graph.getNode(2, 3));
		graph.getNode(2, 0).pair(graph.getNode(3, 0));
		graph.getNode(2, 3).pair(graph.getNode(3, 3));
		graph.printMazeGraph();
		graph.bfs(graph.getNode(3, 3), graph.getNode(2, 0));
		System.out.println();
		graph.printMazeGraph();

	}

}
