package com.wayto.model;

import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

import com.wayto.operator.GeoConvertionsOperations;

public class PolygonalFeature {

	private int id;
	private int salience, levelAbstraction;
	private String name, type, geomAsJSON;
	private Polygon geocoordsGeom; // geographic coordinate
	
	//private Polygon projectGeom, transformedGeom, xGeom;
	
	
	public int getId() {
		return id;
	}
	public Polygon getGeocoordsGeom() {
		return geocoordsGeom;
	}
	public void setGeocoordsGeom(Polygon geocoordsGeom) {
		this.geocoordsGeom = geocoordsGeom;
	}
	public void setGeocoordsGeom(LineString lineString) {
	
	this.geocoordsGeom = GeoConvertionsOperations.lineStringToPolygon(lineString);
	
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
	public int getSalience() {
		return salience;
	}
	public void setSalience(int salience) {
		this.salience = salience;
	}
	public int getLevelAbstraction() {
		return levelAbstraction;
	}
	public void setLevelAbstraction(int levelAbstraction) {
		this.levelAbstraction = levelAbstraction;
	}


	
	
	
}
