package com.wayto.model;

import org.locationtech.jts.geom.LineString;


public class LinearFeature {

	private int id;
	private String name, type, geomAsJSON;
	private LineString geom, xgeom;
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
	public LineString getGeom() {
		return geom;
	}
	public void setGeom(LineString geom) {
		this.geom = geom;
	}
	public LineString getXgeom() {
		return xgeom;
	}
	public void setXgeom(LineString xgeom) {
		this.xgeom = xgeom;
	}
	
}
