package com.wayto.model;

import org.locationtech.jts.geom.Point;


public class RoundAboutNode extends StreetNode{

	private int rbId; 
	public RoundAboutNode(int rbId, int nodeId,  Point geoCoordinates, boolean isRoute, int priority) {
		
		super(nodeId, geoCoordinates, isRoute, priority, true);
		this.rbId = rbId;
		this.setRoundAbout(true);
		
		// TODO Auto-generated constructor stub
	}
	public int getRbId() {
		return rbId;
	}
	public void setRbId(int rbId) {
		this.rbId = rbId;
	}
	
	
	
	
	

}
