package com.wayto.model.topo;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;

import org.geotools.geometry.jts.JTSFactoryFinder;

import com.google.common.collect.Lists;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.MultiPoint;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.impl.CoordinateArraySequence;
import com.wayto.model.PolygonalFeature;
import com.wayto.model.Route;
import com.wayto.model.RouteEdge;
import com.wayto.model.StreetEdge;
import com.wayto.model.StreetNode;
import com.wayto.operator.GeoConvertionsOperations;
import com.wayto.operator.GeometricOperation;


public class PolygonalTopo {

	
	private PolygonalFeature polygonalFeature; 
	//private PolygonalFeature polygonalFeatureX;
	private ArrayList<StreetEdge> edgeList;
	
	/*Edge source is polygon and target street network;*/
	private ArrayList<StreetEdge> controlEdges;
	
	private ArrayList<Integer> keyPoints;

	private int type = SIMPLE_CROSSING;


	public static final int SIMPLE_CROSSING = 01;
	public static final int PASSING_ALONG_RIGHT = 10;
	public static final int PASSING_ALONG_LEFT = 11;
	public static final int PASSING_BY_RIGHT = 100;
	public static final int PASSING_BY_LEFT = 101;
	public static final int ROUTE_STARTS_AT = 110;
	public static final int ROUTE_ENDS_AT = 111;
	public static final int ROUTE_IS_INSIDE = 1000;
	public static final int GLOBAL = 1001;
	public static final int PASSING_ALONG_UNKNOW = 1010;



	
	
	
	public PolygonalTopo(PolygonalFeature polygonalFeature, ArrayList<StreetEdge> edgeList) {
		super();
		this.polygonalFeature = polygonalFeature;
		this.edgeList = edgeList;
		this.controlEdges = new ArrayList<StreetEdge>();
		this.keyPoints = new ArrayList<Integer>();
	}



	public ArrayList<StreetEdge> getControlEdges() {
		return controlEdges;
	}



	public void setControlEdges(ArrayList<StreetEdge> controlEdges) {
		this.controlEdges = controlEdges;
	}
	public PolygonalFeature getPolygonalFeature() {
		return polygonalFeature;
	}
	public void setPolygonalFeature(PolygonalFeature polygonalFeature) {
		this.polygonalFeature = polygonalFeature;
	}


	public ArrayList<StreetEdge> getEdgeList() {
		return edgeList;
	}



	public void setEdgeList(ArrayList<StreetEdge> edgeList) {
		this.edgeList = edgeList;
	}



	public int getType() {
		return type;
	}
	public void setType(int type) {
		this.type = type;
	}
	
	
	public Polygon asPolygon(int coordinateType) {
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		Coordinate[] coords = new Coordinate[this.getEdgeList().size()];
		int i = 0;
		switch (coordinateType) {
		
		case 0:/*projected coordinate*/
			for (StreetEdge e: this.getEdgeList()) 
				coords[i++] = new Coordinate(e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getGeom().getY());
			break;
		case 1:/*projected normalized coordinate*/
			for (StreetEdge e: this.getEdgeList()) 
				coords[i++] = new Coordinate(e.getSourcePoint().getProjectGeom().getX(), e.getSourcePoint().getProjectGeom().getY());
			break;
		case 2:/*X schematic coordinate*/
			for (StreetEdge e: this.getEdgeList()) 
				coords[i++] = new Coordinate(e.getSourcePoint().getxGeom().getX(), e.getSourcePoint().getxGeom().getY());
			break;
		case 3:/*geographic coordinate*/
			for (StreetEdge e: this.getEdgeList()) 
				coords[i++] = new Coordinate(e.getSourcePoint().getCoordinate().getX(), e.getSourcePoint().getCoordinate().getY());
			break;		
		default:
			for (StreetEdge e: this.getEdgeList()) 
				coords[i++] = new Coordinate(e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getCoordinate().getY());
			break;	
		}
		
		LineString ls = geometryFactory.createLineString(coords);
		return GeoConvertionsOperations.lineStringToPolygon(ls);
		
	}
	
	

	public ArrayList<Point2D> asJava2DList(int coordinateType) {
		ArrayList<Point2D> pointList = new ArrayList<Point2D>();
		switch (coordinateType) {
		case 0:/*original coordinate*/
			for( StreetEdge e: this.getEdgeList())
				pointList.add(new Point2D.Double(e.getSourcePoint().getGeom().getX(),e.getSourcePoint().getGeom().getY() ));
			
			break;
		case 1:/*projected normalized coordinate*/
			for( StreetEdge e: this.getEdgeList())
				pointList.add(new Point2D.Double(e.getSourcePoint().getProjectGeom().getX(),e.getSourcePoint().getProjectGeom().getY() ));
			break;
		case 2:/*X schematic coordinate*/
			for( StreetEdge e: this.getEdgeList())
				pointList.add(new Point2D.Double(e.getSourcePoint().getxGeom().getX(),e.getSourcePoint().getxGeom().getY() ));
			
			break;
		case 3:/*Geographic coordinate*/
			for( StreetEdge e: this.getEdgeList())
				pointList.add(new Point2D.Double(e.getSourcePoint().getCoordinate().getX(),e.getSourcePoint().getCoordinate().getY() ));
			
			break;	

		default:
			for( StreetEdge e: this.getEdgeList())
				pointList.add(new Point2D.Double(e.getSourcePoint().getGeom().getX(),e.getSourcePoint().getGeom().getY() ));
			break;
		}

		
		return pointList;
		
		
		
	}



	public void setKeyPoints() {
		for( int i = 0; i < this.getEdgeList().size(); i++) {
			if( this.getEdgeList().get(i).getSourcePoint().getTopoRelations().size() > 0 ||
					this.getEdgeList().get(i).getSourcePoint().getDegree() != 2 ||
					i == 0)
				this.keyPoints.add(i);
			this.keyPoints.add(this.getEdgeList().size());
		}
	}



	public ArrayList<Integer> getKeyPoints() {
		return keyPoints;
	}



	public void setKeyPoints(ArrayList<Integer> keyPoints) {
		this.keyPoints = keyPoints;
	}



	public void setSmothData(ArrayList<Point2D> relevantPtList, ArrayList<Integer> keyPtIndex) {
		int addNodeIndex = 0;
		for( int i = 0; i < this.getEdgeList().size(); i++) {
			if( this.getEdgeList().get(i).getSourcePoint().isRelevantRouteNode() 
					) {
				relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getEdgeList().get(i).getSourcePoint().getxGeom()));
				
//				if( this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() &&
//						i != 0){
				if( this.getEdgeList().get(i).getSourcePoint().getDegree() > 2 &&
						i != 0){
//				if( (this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() ||
//						this.getEdgeList().get(i).getSourcePoint().getDegree() != 2) &&
//						i != 0) {
					keyPtIndex.add(addNodeIndex);

				}
				addNodeIndex++;
				
			}
		}
		relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getEdgeList().get(this.getEdgeList().size() -1).getTargetPoint().getxGeom()));
		//keyPtIndex.add(addNodeIndex);

	}
	





}
