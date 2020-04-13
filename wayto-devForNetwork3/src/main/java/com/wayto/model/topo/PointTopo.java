package com.wayto.model.topo;

import com.wayto.model.PointFeature;
import com.wayto.model.StreetEdge;
import com.wayto.model.StreetNode;

public class PointTopo {

	private PointFeature pointFeature;
	
	private StreetNode node;
	
	private StreetEdge controlEdge;
	
	private double distToRoute;
	
	private int type = PASSING_BY_UNKNOW;


	public static final int IN_ROUTE = 01;
	public static final int PASSING_BY_RIGHT = 10;
	public static final int PASSING_BY_LEFT = 11;
	public static final int PASSING_BY_UNKNOW = 100;
	public static final int GLOBAL = 1001;
	
	public PointTopo(PointFeature pointFeature, StreetNode node) {
		super();
		this.pointFeature = pointFeature;
		this.node = node;
		
	}

	public PointFeature getPointFeature() {
		return pointFeature;
	}

	public void setPointFeature(PointFeature pointFeature) {
		this.pointFeature = pointFeature;
	}

	public StreetNode getNode() {
		return node;
	}

	public void setNode(StreetNode node) {
		this.node = node;
	}

	public StreetEdge getControlEdge() {
		return controlEdge;
	}

	public void setControlEdge(StreetEdge controlEdge) {
		this.controlEdge = controlEdge;
	}

	public int getType() {
		return type;
	}

	public void setType(int type) {
		this.type = type;
	}

	public double getDistToRoute() {
		return distToRoute;
	}

	public void setDistToRoute(double distToRoute) {
		this.distToRoute = distToRoute;
	}

	
	
	
	
}
