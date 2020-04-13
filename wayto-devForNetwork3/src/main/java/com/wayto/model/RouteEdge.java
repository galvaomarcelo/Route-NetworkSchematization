package com.wayto.model;

import org.codehaus.jettison.json.JSONArray;
import org.codehaus.jettison.json.JSONException;
import org.codehaus.jettison.json.JSONObject;

public class RouteEdge extends StreetEdge implements Cloneable {

	
	
	public RouteEdge() {
		super();
		// TODO Auto-generated constructor stub
	}
	private int seq;
	private double aggCost;
	private boolean isReversed;
	
	
	public int getSeq() {
		return seq;
	}
	public void setSeq(int seq) {
		this.seq = seq;
	}
	public double getAggCost() {
		return aggCost;
	}
	public void setAggCost(double aggCost) {
		this.aggCost = aggCost;
	}
	public boolean isReversed() {
		return isReversed;
	}
	public void setReversed(boolean isReversed) {
		this.isReversed = isReversed;
	}
	
@Override
public JSONObject toGeoJSONFeature(int coordinateType) throws JSONException{
		
		JSONObject feature = new JSONObject();
		JSONObject geometry = new JSONObject();
		JSONObject properties = new JSONObject();
		JSONArray point = new JSONArray();
		JSONArray point1 = new JSONArray();
		JSONArray point2 = new JSONArray();
		JSONArray lineString = new JSONArray();

		switch (coordinateType) {
		case 0:
			lineString = new JSONArray();
			point1.put(getSourcePoint().getGeom().getX());
			point1.put(getSourcePoint().getGeom().getY());
			point2.put(getTargetPoint().getGeom().getX());
			point2.put(getTargetPoint().getGeom().getY());
			lineString.put(point1);
			lineString.put(point2);
			
	    	geometry.put("type", "LineString");
	    	geometry.put("coordinates", lineString);
			break;
		case 1:
			lineString = new JSONArray();
			point1.put(getSourcePoint().getProjectGeom().getX()); 
			point1.put(getSourcePoint().getProjectGeom().getY());
			point2.put(getTargetPoint().getProjectGeom().getX());
			point2.put(getTargetPoint().getProjectGeom().getY());
			lineString.put(point1);
			lineString.put(point2);
			
	    	geometry.put("type", "LineString");
	    	geometry.put("coordinates", lineString);
	    	
			break;
		case 2:
			lineString = new JSONArray();
			point1.put(getSourcePoint().getxGeom().getX());
			point1.put(getSourcePoint().getxGeom().getY());
			point2.put(getTargetPoint().getxGeom().getX());
			point2.put(getTargetPoint().getxGeom().getY());
			lineString.put(point1);
			lineString.put(point2);
			
	    	geometry.put("type", "LineString");
	    	geometry.put("coordinates", lineString);
	    	
	    	break;
		case 3:
			lineString = new JSONArray();
			point1.put(getSourcePoint().getCoordinate().getX());
			point1.put(getSourcePoint().getCoordinate().getY());
			point2.put(getTargetPoint().getCoordinate().getX());
			point2.put(getTargetPoint().getCoordinate().getY());
			lineString.put(point1);
			lineString.put(point2);
			
	    	geometry.put("type", "LineString");
	    	geometry.put("coordinates", lineString);
	    	
	    	break;	
		default:
			lineString = new JSONArray();
			point1.put(getSourcePoint().getGeom().getX());
			point1.put(getSourcePoint().getGeom().getY());
			point2.put(getTargetPoint().getGeom().getX());
			point2.put(getTargetPoint().getGeom().getY());
			lineString.put(point1);
			lineString.put(point2);
			
	    	geometry.put("type", "LineString");
	    	geometry.put("coordinates", lineString);
	    	
	    	break;
		}

    	feature.put("type", "Feature");
    	feature.put("geometry", geometry);
    	
    	properties.put("type", "street-edge");
    	properties.put("streetName", getName());
    	properties.put("route", 1);
    	properties.put("km", getKm());
    	properties.put("clazz", getClazz());  
    	properties.put("special", getSpecial());
    	feature.put("properties", properties );
    	
    	return feature;
		
	}
@Override
public Object clone() throws CloneNotSupportedException {
		// TODO Auto-generated method stub
		return super.clone();
	}

		
	
}
