package com.wayto.model;


import java.awt.Color;

import org.locationtech.jts.geom.Point;

public class PointFeature {

	private int id, nodeId, edgeId, salience, streetLevel;
	private String name, type, geomAsJSON,  icon;
	private Point geom;

	
	public int getId() {
		return id;
	}
	public void setId(int id) {
		this.id = id;
	}
	public String getName() {
		return name;
	}
	public void setName(String name) {
		this.name = name;
	}
	public String getType() {
		return type;
	}
	public void setType(String type) {
		this.type = type;
	}
	public String getGeomAsJSON() {
		return geomAsJSON;
	}
	public void setGeomAsJSON(String geomAsJSON) {
		this.geomAsJSON = geomAsJSON;
	}
	public Point getGeom() {
		return geom;
	}
	public void setGeom(Point geom) {
		this.geom = geom;
	}
//	public Point getXgeom() {
//		return xgeom;
//	}
//	public void setXgeom(Point xgeom) {
//		this.xgeom = xgeom;
//	}
	public int getNodeId() {
		return nodeId;
	}
	public void setNodeId(int nodeId) {
		this.nodeId = nodeId;
	}
	public int getEdgeId() {
		return edgeId;
	}
	public void setEdgeId(int edgeId) {
		this.edgeId = edgeId;
	}
	public int getSalience() {
		return salience;
	}
	public void setSalience(int salience) {
		this.salience = salience;
	}
	public int getStreetLevel() {
		return streetLevel;
	}
	public void setStreetLevel(int streetLevel) {
		this.streetLevel = streetLevel;
	}
	public String getIcon() {
		return icon;
	}
	public void setIcon(String icon) {
		this.icon = icon;
	}
	
	
	
}
