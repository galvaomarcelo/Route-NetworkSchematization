package com.wayto.model;



import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;

import org.geotools.geometry.jts.Geometries;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.PrecisionModel;
import com.wayto.mapFeaturesModel.Layer;
import com.wayto.mapFeaturesModel.PolyLineLayer;
import com.wayto.operator.GeoConvertionsOperations;
import com.wayto.operator.GeometricOperation;
import com.wayto.operator.PointsPolar;


public class Route {


	private String startName, endName;
	private ArrayList<RouteEdge> edgeList;
	private LineString geom;
//	private LineString rescaledGeom;
//	private LineString xGeom;
//	private LineString smothtedXgeom;
	
	//private ArrayList<Integer> relevantPointIndex;
	
	public LineString getGeom() {
		return geom;
	}
	public void setGeom(LineString geom) {
		this.geom = geom;
	}
	public StreetNode getStart() {
		if(!edgeList.get(0).isReversed())
			return edgeList.get(0).getSourcePoint();
		else
			return edgeList.get(0).getTargetPoint();
	}

	public StreetNode getEnd() {
		if(!edgeList.get(edgeList.size() -1).isReversed())
			return edgeList.get(edgeList.size() -1).getTargetPoint();
		else
			return edgeList.get(edgeList.size() -1).getSourcePoint();
		
	}

	public String getStartName() {
		return startName;
	}
	public void setStartName(String startName) {
		this.startName = startName;
	}
	public String getEndName() {
		return endName;
	}
	public void setEndName(String endName) {
		this.endName = endName;
	}
	public ArrayList<RouteEdge> getEdgeList() {
		return edgeList;
	}
	public void setEdgeList(ArrayList<RouteEdge> edgeList) {
		this.edgeList = edgeList;
	}
	
	
//	public LineString getxGeom() {
//		return xGeom;
//	}
//	public void setxGeom(LineString xGeom) {
//		this.xGeom = xGeom;
//	}
//	public LineString getSmothtedXgeom() {
//		return smothtedXgeom;
//	}
//	public void setSmothtedXgeom(LineString smothtedXgeom) {
//		this.smothtedXgeom = smothtedXgeom;
//	}
	public Path getRoutePath() {
		Path rPath = new Path(this.asNodeList());
		rPath.setRoute(true);
		return rPath;
	}
//	public void setRoutePath(Path routePath) {
//		this.routePath = routePath;
//	}
//	public ArrayList<Integer> getRelevantPointIndex() {
//		return relevantPointIndex;
//	}
//	public void setRelevantPointIndex(ArrayList<Integer> relevantPointIndex) {
//		this.relevantPointIndex = relevantPointIndex;
//	}
//	
//	public LineString getRescaledGeom() {
//		return rescaledGeom;
//	}
//	public void setRescaledGeom(LineString rescaledGeom) {
//		this.rescaledGeom = rescaledGeom;
//	}
	@Override
	public String toString() {
		return "Route [start=" + getStart() + ", end=" + getEnd() + ", startName=" + startName + ", endName=" + endName + "]";
	}

//	public int hashCode() {
//		final int prime = 31;
//		int result = 1;
//		result = prime * result + ((end == null) ? 0 : end.hashCode());
//		result = prime * result + ((endName == null) ? 0 : endName.hashCode());
//		result = prime * result + ((edgeList == null) ? 0 : edgeList.hashCode());
//		result = prime * result + ((start == null) ? 0 : start.hashCode());
//		result = prime * result + ((startName == null) ? 0 : startName.hashCode());
//		return result;
//	}

//	public Layer toLayer(Envelope env) {
//		Coordinate[] coordinates = this.getGeom().getCoordinates();
//		PolyLineLayer layer = new PolyLineLayer("route");
//		layer.setColor(Color.RED);
//		ArrayList<Point2D> polyline = new ArrayList<Point2D>();
//		
//
//		
//		for(Coordinate c : coordinates){
//			
//			if(c!=null){
//			
//				polyline.add( new Point2D.Double(c.x - env.getMinX(), env.getMaxY() - c.y ));
//			}
//		}
//		if( this.getGeom().isClosed() )
//			polyline.remove(polyline.size() - 1);
//		layer.getLines().add(polyline);
//		return layer;
//	}
//	
//	public Layer toLayer2(Envelope env) {
//
//		PolyLineLayer layer = new PolyLineLayer("route2");
//		layer.setColor(Color.RED);
//		ArrayList<Point2D> polyline = new ArrayList<Point2D>();
//		
//
//		
//		for(RouteEdge e: this.edgeList){
//			if(!e.isReversed())
//				polyline.add( new Point2D.Double(e.getSourcePoint().getGeom().getX() - env.getMinX()
//						, env.getMaxY() - e.getSourcePoint().getGeom().getY() ));
//			else
//				polyline.add( new Point2D.Double(e.getTargetPoint().getGeom().getX() - env.getMinX()
//						, env.getMaxY() - e.getTargetPoint().getGeom().getY() ));
//				
//		}
//		layer.getLines().add(polyline);
//		return layer;
//	}
	
	public RouteEdge getAnEdgeWithNode(int nodeId){
		for(RouteEdge e: this.getEdgeList()){
			if(e.getSourceId() == nodeId || e.getTargetId() == nodeId){
				return e;
				
			}
			
		}
		return null;
	}
	public ArrayList<Integer> getDPindex() {
		ArrayList<Integer> decisionPointsIndexes = new ArrayList<Integer>();
		Path routePath = this.getRoutePath();
		for(int i = 0; i < routePath.getNodeList().size(); i++){
			if(routePath.getNodeList().get(i).isDecisionPoint())
				decisionPointsIndexes.add(i );
		}
		return decisionPointsIndexes;
	}
	public ArrayList<Integer> getDPAnd2DegreeIndex() {
		ArrayList<Integer> decisionPointsIndexes = new ArrayList<Integer>();
		Path routePath = this.getRoutePath();
		for(int i = 1; i < routePath.getNodeList().size(); i++){
			if(routePath.getNodeList().get(i).isDecisionPoint() ||  routePath.getNodeList().get(i).getDegree() > 2)
				decisionPointsIndexes.add(i );
		}
		return decisionPointsIndexes;
	}
	public ArrayList<Integer> getDPAndTopoAlongIndex() {
		ArrayList<Integer> decisionPointsIndexes = new ArrayList<Integer>();
		Path routePath = this.getRoutePath();
		for(int i = 1; i < routePath.getNodeList().size() -1; i++){
			if(routePath.getNodeList().get(i).isDecisionPoint() ||  routePath.getNodeList().get(i).isTopoAlong())
				decisionPointsIndexes.add(i );
		}
		return decisionPointsIndexes;
	}
	
	public ArrayList<Integer> getDPAndTopoAlongIndex2() {
		ArrayList<Integer> decisionPointsIndexes = new ArrayList<Integer>();
		boolean previousIsDP = true;
		boolean currentIsDP = false;
		Path routePath = this.getRoutePath();
		double distToPrevious = routePath.getNodeList().get(1).getGeom().distance(routePath.getNodeList().get(0).getGeom()) ;
		for(int i = 1; i < routePath.getNodeList().size() -1; i++){

			if(routePath.getNodeList().get(i).isDecisionPoint() ||  (routePath.getNodeList().get(i).isTopoAlong() ) ) {
				if(routePath.getNodeList().get(i).isDecisionPoint())
					currentIsDP = true;
				else
					currentIsDP = false;
				if(decisionPointsIndexes.size() > 0)
					distToPrevious = routePath.getNodeList().get(i).getGeom().distance(routePath.getNodeList().get(decisionPointsIndexes.get(decisionPointsIndexes.size()-1)).getGeom()) ;
				if(distToPrevious > 100 || currentIsDP==previousIsDP) {

					decisionPointsIndexes.add(i );
					previousIsDP = currentIsDP;
					
				}
				
				
			}
			
		}
		return decisionPointsIndexes;
	}
	
	public ArrayList<Integer> getDPAndTopoAlongCrossIndex2() {
		ArrayList<Integer> decisionPointsIndexes = new ArrayList<Integer>();
		boolean previousIsDP = true;
		boolean currentIsDP = false;
		Path routePath = this.getRoutePath();
		double distToPrevious = routePath.getNodeList().get(1).getGeom().distance(routePath.getNodeList().get(0).getGeom()) ;
		for(int i = 1; i < routePath.getNodeList().size() -1; i++){

			if(routePath.getNodeList().get(i).isDecisionPoint() ||  routePath.getNodeList().get(i).isTopoAlong() ||  routePath.getNodeList().get(i).isTopoCrossing()  ) {
				if(routePath.getNodeList().get(i).isDecisionPoint())
					currentIsDP = true;
				else
					currentIsDP = false;
				if(decisionPointsIndexes.size() > 0) {
					distToPrevious = routePath.getNodeList().get(i).getGeom().distance(routePath.getNodeList().get(decisionPointsIndexes.get(decisionPointsIndexes.size()-1)).getGeom()) ;
					System.out.println("Point index: " + i + " previous added: " + decisionPointsIndexes.get(decisionPointsIndexes.size()-1) + " Dist to previous = " + distToPrevious);
				}
				if(distToPrevious > 200 || (currentIsDP && previousIsDP) ) {
					System.out.println("Add: " + i + " is DP " + currentIsDP);
					decisionPointsIndexes.add(i );
					previousIsDP = currentIsDP;
					
				}
				
				
			}
			
		}
		return decisionPointsIndexes;
	}
	
	
	public ArrayList<Integer> getDPAndUrbanCrossIndex() {
		ArrayList<Integer> decisionPointsIndexes = new ArrayList<Integer>();
		boolean previousIsDP = true;
		boolean currentIsDP = false;
		Path routePath = this.getRoutePath();
		double distToPrevious = routePath.getNodeList().get(1).getGeom().distance(routePath.getNodeList().get(0).getGeom()) ;
		for(int i = 1; i < routePath.getNodeList().size() -1; i++){

			if(routePath.getNodeList().get(i).isDecisionPoint() ||   ( routePath.getNodeList().get(i).isTopoCrossing()  ) ) {
				if(routePath.getNodeList().get(i).isDecisionPoint())
					currentIsDP = true;
				else
					currentIsDP = false;
				if(decisionPointsIndexes.size() > 0) {
					distToPrevious = routePath.getNodeList().get(i).getGeom().distance(routePath.getNodeList().get(decisionPointsIndexes.get(decisionPointsIndexes.size()-1)).getGeom()) ;
					System.out.println("Point index: " + i + " previous added: " + decisionPointsIndexes.get(decisionPointsIndexes.size()-1) + " Dist to previous = " + distToPrevious);
				}
				if(distToPrevious > 200 || (currentIsDP && previousIsDP) ) {
					System.out.println("Add: " + i + " is DP " + currentIsDP);
					decisionPointsIndexes.add(i );
					previousIsDP = currentIsDP;
					
				}
				
				
			}
			
		}
		return decisionPointsIndexes;
	}
	
	
	public ArrayList<Integer> getAllCrossingsIndex2() {
		ArrayList<Integer> decisionPointsIndexes = new ArrayList<Integer>();
		boolean previousIsDP = true;
		boolean currentIsDP = false;
		Path routePath = this.getRoutePath();
		double distToPrevious = routePath.getNodeList().get(1).getGeom().distance(routePath.getNodeList().get(0).getGeom()) ;
		for(int i = 1; i < routePath.getNodeList().size() -1; i++){

			if(routePath.getNodeList().get(i).isDecisionPoint() ||  routePath.getNodeList().get(i).isTopoAlong() ||  routePath.getNodeList().get(i).isTopoCrossing() ||  routePath.getNodeList().get(i).getDegree() > 2) {
				
				if(routePath.getNodeList().get(i).isDecisionPoint() ||  routePath.getNodeList().get(i).isTopoAlong() ||  routePath.getNodeList().get(i).isTopoCrossing()  )
					currentIsDP = true;
				else
					currentIsDP = false;
				if(decisionPointsIndexes.size() > 0) {
					distToPrevious = routePath.getNodeList().get(i).getGeom().distance(routePath.getNodeList().get(decisionPointsIndexes.get(decisionPointsIndexes.size()-1)).getGeom()) ;
					System.out.println("dist to previous: " + distToPrevious);
				}
				if(distToPrevious > 200 || currentIsDP==previousIsDP) {

					decisionPointsIndexes.add(i );
					previousIsDP = currentIsDP;
					
				}
				
				
			}
			
		}
		return decisionPointsIndexes;
	}
	
	public ArrayList<Integer> getTopoAlongCrossIndex2() {
		ArrayList<Integer> decisionPointsIndexes = new ArrayList<Integer>();
		boolean previousIsDP = true;
		boolean currentIsDP = false;
		Path routePath = this.getRoutePath();
		double distToPrevious = routePath.getNodeList().get(1).getGeom().distance(routePath.getNodeList().get(0).getGeom()) ;
		for(int i = 1; i < routePath.getNodeList().size() -1; i++){

			if( routePath.getNodeList().get(i).isTopoAlong() ||  routePath.getNodeList().get(i).isTopoCrossing()  ) {
				if(routePath.getNodeList().get(i).isDecisionPoint())
					currentIsDP = true;
				else
					currentIsDP = false;
				if(decisionPointsIndexes.size() > 0)
					distToPrevious = routePath.getNodeList().get(i).getGeom().distance(routePath.getNodeList().get(decisionPointsIndexes.get(decisionPointsIndexes.size()-1)).getGeom()) ;
				if(distToPrevious > 100 || currentIsDP==previousIsDP) {

					decisionPointsIndexes.add(i );
					previousIsDP = currentIsDP;
					
				}
				
				
			}
			
		}
		return decisionPointsIndexes;
	}
	
	public ArrayList<Integer> getPredefinedIndex() {
		
		/**Route3 Experiments rescale node*/
		//int[] predefinedNodesId = {14618, 14165, 14131, 38923, 43370,  40320, 75687, 32963, 38979, 33049, 32956, 32614,	 60004};
		/**Route2 Experiments rescale node*/
		int[] predefinedNodesId = {38694, 33459, 16575,32396, 48562, 42058, 38747, 34584, 23314, 23352, 32769, 33055, 32606 };
		
		ArrayList<Integer> nodeIdList = new ArrayList<Integer>();
		for(int id:predefinedNodesId) {
			nodeIdList.add(id);
	      }
		System.out.println("All route ids:");

		ArrayList<Integer> decisionPointsIndexes = new ArrayList<Integer>();
		Path routePath = this.getRoutePath();
		for(int i = 0; i < routePath.getNodeList().size(); i++){
			System.out.println(routePath.getNodeList().get(i).getId());
			if(	nodeIdList.contains( routePath.getNodeList().get(i).getId() ) ) {
					
				decisionPointsIndexes.add(i );
			}
		}
		return decisionPointsIndexes;
	}
	
//	public void uptadeXGeom() {
//		this.xGeom = (LineString)GeoConvertionsOperations.Java2DToJTSGeometry(this.routePath.asJava2DList(2), Geometries.LINESTRING);	
//		
//	}
	
	
	public ArrayList<StreetNode> asNodeList() {
		ArrayList<StreetNode> routeNodeList = new ArrayList<StreetNode>();
		for(RouteEdge re: this.edgeList){
			if(!re.isReversed())
				routeNodeList.add(re.getSourcePoint());
			else
				routeNodeList.add(re.getTargetPoint());
			
		}
		if(!edgeList.get(edgeList.size() -1).isReversed())
			routeNodeList.add(edgeList.get(edgeList.size() -1).getTargetPoint());
		else
			routeNodeList.add(edgeList.get(edgeList.size() -1).getSourcePoint());
		
		return routeNodeList;
	}
	
	
	
	
	
	
}
