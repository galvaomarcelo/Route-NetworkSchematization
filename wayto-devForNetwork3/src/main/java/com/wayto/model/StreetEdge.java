package com.wayto.model;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.codehaus.jettison.json.JSONArray;
import org.codehaus.jettison.json.JSONException;
import org.codehaus.jettison.json.JSONObject;

import org.locationtech.jts.geom.LineString;

public class StreetEdge implements Cloneable{
   /**
	Highway1 11
	Highway link 12
	Highway2(trunk) 13
	Highway2(trunk) link 14
	Primary 15
	Primary link 16
	Secondary 21
	Secondary link 22
	Tertiary 31
	Tertiary link 32
	Residential 41
	Unknow 42
	Not classified/Serivice/Rural  43
	Lowpriority(not street)  100
	
	
	Flags:
	00001=1=cars only
	00010=2=bikes only
	00100=4=foot only
	00110=6=bikes and pedestrians only.	
    */

	private int id;
	private String name;
	private int clazz;
	private int flags;
	private int special;
	private double km;
	private int kmh;
	private double cost;
	private double reverseCost;

	private String geomAsJSON;
	private StreetNode sourcePoint;
	private StreetNode targetPoint;
	private boolean isFakeEdge = false;
	private boolean isStubEdge = false;
	private int isPolygonEdge = 0; /*polygon id or 0 if false*/
	
	private LineString geom;
 

	
	
	public StreetEdge() {
		super();
		// TODO Auto-generated constructor stub
	}
	
	public StreetEdge(int id, String name, StreetNode sourcePoint,
			StreetNode targetPoint) {
		super();
		this.id = id;
		this.name = name;
		this.clazz = 100;
		this.flags = 0;
		this.sourcePoint = sourcePoint;
		this.targetPoint = targetPoint;

	}
	
	public StreetEdge(int id, String name, StreetNode sourcePoint,
			StreetNode targetPoint, int isPolygonEdge) {
		super();
		this.id = id;
		this.name = name;
		this.clazz = 100;
		this.flags = 0;
		this.sourcePoint = sourcePoint;
		this.targetPoint = targetPoint;
		this.isPolygonEdge = isPolygonEdge;

	}

	public int getIsPolygonEdge() {
		return isPolygonEdge;
	}
	public void setIsPolygonEdge(int isPolygonEdge) {
		this.isPolygonEdge = isPolygonEdge;
	}

	public int getId() {
		return id;
	}
	public void setId(int id) {
		this.id = id;
	}
	
	public boolean isStubEdge() {
		return isStubEdge;
	}

	public void setStubEdge(boolean isStubEdge) {
		this.isStubEdge = isStubEdge;
	}

	public String getName() {
		return name;
	}
	public void setName(String name) {
		this.name = name;
	}
	public int getClazz() {
		return clazz;
	}
	public void setClazz(int clazz) {
		this.clazz = clazz;
	}
	public int getFlags() {
		return flags;
	}
	public void setFlags(int flags) {
		this.flags = flags;
	}
	
	
	public int getSpecial() {
		return special;
	}

	public void setSpecial(int special) {
		this.special = special;
	}

	public double getKm() {
		return km;
	}
	public void setKm(double km) {
		this.km = km;
	}
	public int getKmh() {
		return kmh;
	}
	public void setKmh(int kmh) {
		this.kmh = kmh;
	}
	public double getCost() {
		return cost;
	}
	public void setCost(double cost) {
		this.cost = cost;
	}
	public double getReverseCost() {
		return reverseCost;
	}
	public void setReverseCost(double reverseCost) {
		this.reverseCost = reverseCost;
	}

	public String getGeomAsJson() {
		return geomAsJSON;
	}
	public void setGeomAsJson(String geomAsJson) {
		this.geomAsJSON = geomAsJson;
	}

	
	public StreetNode getSourcePoint() {
		return sourcePoint;
	}
	public void setSourcePoint(StreetNode sourcePoint) {
		this.sourcePoint = sourcePoint;
	}
	public StreetNode getTargetPoint() {
		return targetPoint;
	}
	public void setTargetPoint(StreetNode targetPoint) {
		this.targetPoint = targetPoint;
	}
	

	
	
	public boolean isFakeEdge() {
		return isFakeEdge;
	}
	public void setFakeEdge(boolean isFakeEdge) {
		this.isFakeEdge = isFakeEdge;
	}
	
	
	public LineString getGeom() {
		return geom;
	}

	public void setGeom(LineString geom) {
		this.geom = geom;
	}

	@Override
	public Object clone() throws CloneNotSupportedException {
		
		return super.clone();
	}
	public JSONObject toGeoJSONFeature(int coordinateType) throws JSONException{
		
		JSONObject feature = new JSONObject();
		JSONObject geometry = new JSONObject();
		JSONObject properties = new JSONObject();
		JSONArray point1 = new JSONArray();
		JSONArray point2 = new JSONArray();
		JSONArray lineString = new JSONArray();

		switch (coordinateType) {
			case 0:
				lineString = new JSONArray();
				point1.put(sourcePoint.getGeom().getX());
				point1.put(sourcePoint.getGeom().getY());
				point2.put(targetPoint.getGeom().getX());
				point2.put(targetPoint.getGeom().getY());
				lineString.put(point1);
				lineString.put(point2);
				
		    	geometry.put("type", "LineString");
		    	geometry.put("coordinates", lineString);
				break;
			case 1:
				lineString = new JSONArray();
				point1.put(sourcePoint.getProjectGeom().getX());
				point1.put(sourcePoint.getProjectGeom().getY());
				point2.put(targetPoint.getProjectGeom().getX());
				point2.put(targetPoint.getProjectGeom().getY());
				lineString.put(point1);
				lineString.put(point2);
				
		    	geometry.put("type", "LineString");
		    	geometry.put("coordinates", lineString);
		    	
				break;
			case 2:
				lineString = new JSONArray();
				point1.put(sourcePoint.getxGeom().getX());
				point1.put(sourcePoint.getxGeom().getY());
				point2.put(targetPoint.getxGeom().getX());
				point2.put(targetPoint.getxGeom().getY());
				lineString.put(point1);
				lineString.put(point2);
				
		    	geometry.put("type", "LineString");
		    	geometry.put("coordinates", lineString);
		    	
		    	break;
			case 3:
				lineString = new JSONArray();
				point1.put(sourcePoint.getCoordinate().getX());
				point1.put(sourcePoint.getCoordinate().getY());
				point2.put(targetPoint.getCoordinate().getX());
				point2.put(targetPoint.getCoordinate().getY());
				lineString.put(point1);
				lineString.put(point2);
				
		    	geometry.put("type", "LineString");
		    	geometry.put("coordinates", lineString);
		    	
		    	break;	
			default:
				lineString = new JSONArray();
				point1.put(sourcePoint.getGeom().getX());
				point1.put(sourcePoint.getGeom().getY());
				point2.put(targetPoint.getGeom().getX());
				point2.put(targetPoint.getGeom().getY());
				lineString.put(point1);
				lineString.put(point2);
				
		    	geometry.put("type", "LineString");
		    	geometry.put("coordinates", lineString);
		    	
		    	break;
		}

    	feature.put("type", "Feature");
    	feature.put("geometry", geometry);
    	
    	properties.put("type", "street-edge");
    	properties.put("streetName", name);
    	if( !(this.getSourcePoint().isRouteNode() && this.getTargetPoint().isRouteNode()) )
    		properties.put("route", 0);
    	else
    		properties.put("route", 1);
    	properties.put("km", km);
    	properties.put("clazz", clazz); 
    	properties.put("special", special);
    	feature.put("properties", properties );
    	
    	return feature;
		
	}
	
	@Override
	public String toString() {
		return "StreetEdge [ id=" + id + ", sourceid: " + sourcePoint.getId() + ", targetid: " + targetPoint.getId() + ", name=" + name + ", clazz=" + clazz
				+ "]";
	}
	
	

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		StreetEdge other = (StreetEdge) obj;
		if (id != other.id)
			return false;
		if (name == null) {
			if (other.name != null)
				return false;
		} else if (!name.equals(other.name))
			return false;
		return true;
	}
	
	
	public ArrayList<Point2D> asPointList(int coordinateType) {
		
		
		ArrayList<Point2D> pointList = new ArrayList<Point2D>();
		switch (coordinateType) {
		case 0:/*original coordinate*/
			
				pointList.add(new Point2D.Double(this.getSourcePoint().getGeom().getX(),this.getSourcePoint().getGeom().getY() ));
				pointList.add(new Point2D.Double(this.getTargetPoint().getGeom().getX(),this.getTargetPoint().getGeom().getY() ));
			
			break;
		case 1:/*projected normalized coordinate*/
			pointList.add(new Point2D.Double(this.getSourcePoint().getProjectGeom().getX(),this.getSourcePoint().getProjectGeom().getY() ));
			pointList.add(new Point2D.Double(this.getTargetPoint().getProjectGeom().getX(),this.getTargetPoint().getProjectGeom().getY() ));
			break;
		case 2:/*X schematic coordinate*/
			pointList.add(new Point2D.Double(this.getSourcePoint().getxGeom().getX(),this.getSourcePoint().getxGeom().getY() ));
			pointList.add(new Point2D.Double(this.getTargetPoint().getxGeom().getX(),this.getTargetPoint().getxGeom().getY() ));
			
			break;

		default:
			pointList.add(new Point2D.Double(this.getSourcePoint().getGeom().getX(),this.getSourcePoint().getGeom().getY() ));
			pointList.add(new Point2D.Double(this.getTargetPoint().getGeom().getX(),this.getTargetPoint().getGeom().getY() ));
			break;
		}

		
		return pointList;

	}
	public boolean isLink() {
		if(this.clazz == 12 || this.clazz == 14 || this.clazz == 16 || this.clazz == 22 || this.clazz == 32 || this.clazz == 43)
			return true;
		else return false;
	}

	public boolean isAdjacent(StreetEdge edge) {
		boolean isAdjacent = false;
		if(this.sourcePoint == edge.getSourcePoint())
			isAdjacent = true;
		else if(this.sourcePoint == edge.getTargetPoint())
			isAdjacent = true;
		else if(this.targetPoint == edge.getSourcePoint())
			isAdjacent = true;
		else if(this.targetPoint == edge.getTargetPoint())
			isAdjacent = true;
		return isAdjacent;
	}

	public int getSourceId() {
		return this.sourcePoint.getId();
		
	}
	
	public int getTargetId() {
		return this.targetPoint.getId();
		
	}
	
	public double getLength(int coordinateType) {
		double lenght = 0;
		switch (coordinateType) {
		case 0:/*projection coordinate*/
			lenght = sourcePoint.getGeom().distance(targetPoint.getGeom());
			break;
		case 1:/*projected normalized coordinate*/
			lenght = sourcePoint.getProjectGeom().distance(targetPoint.getProjectGeom());
			break;
		case 2:/*X schematic coordinate*/
			lenght = sourcePoint.getxGeom().distance(targetPoint.getxGeom());
			break;
		case 3:/*geographic coordinate*/
			lenght = sourcePoint.getCoordinate().distance(targetPoint.getCoordinate());
			break;
		default:
			lenght = sourcePoint.getGeom().distance(targetPoint.getGeom());
			break;
		}		
		return lenght;
		
		
		
	}
	
	
	
	
}
