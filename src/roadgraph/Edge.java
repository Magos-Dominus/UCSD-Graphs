package roadgraph;

import java.util.Objects;

//Separate object used to represent edges in the graph. Necessary because our edges require additional information 
//such as roadName, length etc.

import geography.GeographicPoint;

public class Edge {
	private Node from;
	private Node to;
	private String roadName;
	private String roadType;
	private double length;
	private double speed;
	private double driveThrough;
	
	
	public Edge(Node from, Node to) {
		this.from = from;
		this.to = to;
	}
	
	public Edge(Node from, Node to, String roadName,
			String roadType, double length){
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		if(roadType == "primary") {
			this.speed = 60;
		}
		else if(roadType == "residential") {
			this.speed = 40;
		}
		else if(roadType == "motorway") {
			this.speed = 130;
		}
		else if(roadType == "tertiary") {
			this.speed = 30;
		}
		else if(roadType == "secondary_link") {
			this.speed = 50;
		}
		else if(roadType == "motorway_link") {
			this.speed = 60;
		}
		else {
			this.speed = 50;
		}
		
		this.driveThrough = this.length/this.speed;


	}
	
	public double getTravelTime() {
		return driveThrough;
	}
	
	public double getSpeedLimit() {
		return speed;
	}
	
	public Node getOtherNode(Node n) {
		if(n == from) {
			return to;
		}
		else if(n == to) {
			return from;
		}
		return null;
	}

	public Node getFrom() {
		return from;
	}


	public Node getTo() {
		return to;
	}


	public String getRoadName() {
		return roadName;
	}


	public String getRoadType() {
		return roadType;
	}


	public double getLength() {
		return length;
	}


}
