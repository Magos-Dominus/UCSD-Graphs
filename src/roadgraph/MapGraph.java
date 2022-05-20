/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private Map<GeographicPoint, Node> graph;
	private Set<Edge> edges;
	private Map<Edge, List<GeographicPoint>> searches;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		 edges = new HashSet<>();
		 graph = new HashMap<>();
		 searches = new HashMap<Edge, List<GeographicPoint>>();
		// TODO: Implement in this constructor in WEEK 3
	}
	
	//Private method to obtain all the neighbors of a particular vertex 
	
	private ArrayList<GeographicPoint> getNeighbors(GeographicPoint from){
		List<Edge> list = graph.get(from).getEdges();
		if(list == null) {
			return new ArrayList<>();
		}
		ArrayList<GeographicPoint> result = new ArrayList<GeographicPoint>();
		for(Edge e : list) {
			result.add(e.getTo().getPoint());
		}
		return result;
	}
	
	private ArrayList<Node> getNeighborsNode(GeographicPoint from){
		List<Edge> list = graph.get(from).getEdges();
		if(list == null) {
			return new ArrayList<>();
		}
		ArrayList<Node> result = new ArrayList<Node>();
		for(Edge e : list) {
			result.add(e.getTo());
		}
		return result;
	}
	
	//Private method to generate a path between a goal and its start
	
	private List<GeographicPoint> generatePath(GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap){
		List<GeographicPoint> path = new ArrayList<>();
		GeographicPoint current = goal;
		while(current != null) {
			path.add(current);
			current = parentMap.get(current);
		}
		Collections.reverse(path);
		return path;
	}
	
	private List<Node> getNeighborsNode(Node n) {
		List<Node> result = new ArrayList<Node>();
		List<Edge> list = n.getEdges();
		for(Edge e : list) {
			if(e.getFrom() == n) {
			result.add(e.getOtherNode(n));
			}
		}
		return result;
	}
	

	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return graph.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return graph.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint point)
	{
	    if(graph.containsKey(point) || point == null ) {
			return false;
		}
		else {
			graph.put(point, new Node(point));
			return true;
		}
		// TODO: Implement this method in WEEK 3
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		Node n1 = graph.get(from);
		
		Node n2 = graph.get(to);
		
		if(n1 == null || n2 == null) {
			throw new IllegalArgumentException("To or from not present");
		}
		if(length < 0) {
			throw new IllegalArgumentException();
		}
		if(from == null || to == null || roadName == null || roadType == null || (Double)length == null) {
			throw new IllegalArgumentException();
		}
		
		Edge e = new Edge(n1, n2, roadName, roadType, length);
		edges.add(e);
		n1.addEdge(e);

		}
		//TODO: Implement this method in WEEK 3
	
	//method to reset the priority and distance variables for all nodes after finding a route
	private void resetNodes() {
		for(GeographicPoint p : graph.keySet()) {
			graph.get(p).setDistance(Double.POSITIVE_INFINITY);
			graph.get(p).setPriority(Double.POSITIVE_INFINITY);
		}
	}
	
	private List<GeographicPoint> checkPreviousSearches(GeographicPoint start, GeographicPoint end){
		for(Edge e : searches.keySet()) {
			if(e.getFrom().getPoint()== start && e.getTo().getPoint() == end) {
				return searches.get(e);
			}
		}
		return new ArrayList<GeographicPoint>();
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		List<GeographicPoint> old = checkPreviousSearches(start, goal);
		if(!old.isEmpty()) {
			for(GeographicPoint p : old) {
				System.out.println(p);
			}
			return old;
		}
		List<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		parentMap.put(start, null);
		queue.add(start);
		visited.add(start);
		
		while(!queue.isEmpty()) {
			GeographicPoint curr = queue.remove(0);
			
			nodeSearched.accept(curr);
			if(curr.equals(goal)) {
				resetNodes();
				Edge ed = new Edge(new Node(start), new Node(goal));
				searches.put(ed, generatePath(curr, parentMap));
				return generatePath(curr, parentMap);
			}

			ArrayList<GeographicPoint> neighbors = getNeighbors(curr);
			for(GeographicPoint p : neighbors) {
				if(!visited.contains(p)) {
					visited.add(p);
					parentMap.put(p, curr);
					queue.add(p);
				}
			}
		}
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		resetNodes();
		return null;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		List<GeographicPoint> old = checkPreviousSearches(start, goal);
		if(!old.isEmpty()) {
			for(GeographicPoint p : old) {
				System.out.println(p);
			}
			return old;
		}
		Queue<Node> queue = new PriorityQueue<Node>();
		HashSet<Node> visited = new HashSet<Node>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		Node st = graph.get(start);
		st.setDistance(0);
		st.setTime(0);
		st.setPriority(0);
		parentMap.put(start, null);
		queue.add(st);
		int count = 0;

		while(!queue.isEmpty()) {
			Node curr = queue.poll();
			count ++;
			nodeSearched.accept(curr.getPoint());
			//System.out.println("Now visiting node" + curr + "   " + curr.getPriority());
			if(!visited.contains(curr)) {
				visited.add(curr);
				if(curr.getPoint().equals(goal)) {
					//System.out.println(count);
					Edge ed = new Edge(new Node(start), new Node(goal));
					searches.put(ed, generatePath(goal, parentMap));
					resetNodes();
					return generatePath(goal, parentMap);
				}
				List<Node> neigh = getNeighborsNode(curr);
				
				for(Node n : neigh) {
					//double distance = dist(curr, n) + curr.getDistance();
					double travel = travelTime(curr, n) + curr.getTime();
					if(!visited.contains(n) && travel < n.getPriority()) {
							n.setPriority(travel);
							//n.setPriority(distance);
							n.setTime(travel);
							n.setDistance(travel);
							queue.add(n);
							parentMap.put(n.getPoint(), curr.getPoint());
					}
				}
			}
		}
		// TODO: Implement this method in WEEK 4
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		resetNodes();
		return null;
	}
	
	private double dist(Node n1, Node n2) {
		for(Edge e : edges) {
			if(e.getFrom() == n1 && e.getTo() == n2){
				return e.getLength();
			}
		}
		return 0;
	}
	
	private double travelTime(Node n1, Node n2) {
		for(Edge e : edges) {
			if(e.getFrom() == n1 && e.getTo() == n2){
				return e.getTravelTime();
			}
		}
		return 0;
	}
	
	

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		List<GeographicPoint> old = checkPreviousSearches(start, goal);
		if(!old.isEmpty()) {
			for(GeographicPoint p : old) {
				System.out.println(p);
			}
			return old;
		}
		
		Queue<Node> queue = new PriorityQueue<Node>();
		HashSet<Node> visited = new HashSet<Node>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		Node st = graph.get(start);
		st.setDistance(0);
		st.setPriority(0);
		st.setTime(0);
		parentMap.put(start, null);
		queue.add(st);
		int count = 0;

		while(!queue.isEmpty()) {
			Node curr = queue.poll();
			nodeSearched.accept(curr.getPoint());
			count ++;
			System.out.println("Now visiting node" + curr + "    " + curr.getDistance() +  "   " + curr.getPriority());
			
			if(!visited.contains(curr)) {
				visited.add(curr);
			
				if(curr.getPoint().equals(goal)) {
					System.out.println(count);
					resetNodes();
					Edge ed = new Edge(new Node(start), new Node(goal));
					searches.put(ed, generatePath(goal, parentMap));
					System.out.println(searches.size());
					return generatePath(goal, parentMap);
				}
				List<Node> neigh = getNeighborsNode(curr);
				
				for(Node n : neigh) {
					
					double distance = dist(curr,n) + curr.getDistance();
					//double estimate = n.getPoint().distance(goal) + distance;
					double timeEstimate = n.getPoint().distance(goal) + travelTime(curr, n)/50 + curr.getTime();
					if(!visited.contains(n) && timeEstimate < n.getPriority()) {
							n.setPriority(timeEstimate);
							n.setDistance(distance);
							n.setTime(timeEstimate + travelTime(curr,n));
							//System.out.println(n.getDistance() + "    " + n.getPriority());
							queue.add(n);
							parentMap.put(n.getPoint(), curr.getPoint());
					}
				}
			}
		}
		
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		resetNodes();
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		
		// Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}	
}
