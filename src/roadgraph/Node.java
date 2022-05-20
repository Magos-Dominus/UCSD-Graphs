package roadgraph;


import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import geography.GeographicPoint;

public class Node implements Comparable<Node> {
	
	private GeographicPoint point;
	private List<Edge> edges;
	private double distance;
	private double priority;
	private double time;


	public Node(GeographicPoint point) {
		this.point = point;
		this.distance = Double.POSITIVE_INFINITY;
		this.priority = Double.POSITIVE_INFINITY;
		this.time = Double.POSITIVE_INFINITY;
		this.edges = new ArrayList<Edge>();
	}
	
	public List<Edge> getEdges() {
		return edges;
	}
	
	public void addEdge(Edge e) {
		edges.add(e);
	}

	public void setEdges(List<Edge> edges) {
		this.edges = edges;
	}

	public GeographicPoint getPoint() {
		return point;
	}

	public void setPoint(GeographicPoint point) {
		this.point = point;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}

	public double getPriority() {
		return priority;
	}
	
	public double getTime() {
		return time;
	}

	public void setPriority(double priority) {
		this.priority = priority;
	}
	
	public void setTime(double time) {
		this.time = time;
	}
	
	public double distance(Node n, Node n1) {
		for(Edge e : edges) {
			if(e.getFrom() == n && e.getTo() == n1 ) {
				return e.getLength();
			}
		}
		return 0;
	}

	@Override
	public int compareTo(Node other) {
		if(this.priority == other.priority) {
			return 0;
		}
		if(this.priority < other.priority) {
			return -1;
		}
		if(this.priority > other.priority) {
			return 1;
		}
		return 0;
	}
	
	@Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Node node = (Node) o;

        return point.equals(node.point);

    }
	
	@Override
    public String toString()
    {
        return "[NODE at location (" + point + ")";
    }
	
	@Override
    public int hashCode() {
        return point.hashCode();
    }


}
