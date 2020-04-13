package com.wayto.model;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Comparator;

import org.geotools.geometry.jts.Geometries;
import org.geotools.geometry.jts.JTSFactoryFinder;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.PrecisionModel;

import com.wayto.operator.GeoConvertionsOperations;
import com.wayto.operator.GeometricOperation;
import com.wayto.operator.PointsPolar;

public class Path implements Cloneable{

	private ArrayList<StreetNode> nodeList;

	private boolean isRoute = false;
	private boolean isDisconneted = false;
	private boolean isRouteAdjEdge = false;
	private boolean isChunkPath = false;

	private int isPolygon = 0; /*polygon id or 0 if false*/
	private int clazz = 0; 
	//private ArrayList<Integer> relevantPointIndex;
	private boolean wasSchematized = false;
	
	
	
	
	

	
	public Path(ArrayList<StreetNode> nodeList) {
		super();
		this.nodeList = nodeList;

//		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
//		Coordinate[] coords = new Coordinate[nodeList.size()];
//		for (int i = 0; i < nodeList.size(); i++) {
//
//			coords[i] = new Coordinate(nodeList.get(i).getGeom().getX(), nodeList.get(i).getGeom().getY());
//
//		}
//		//this.geom = geometryFactory.createLineString(coords);

	}
	public LineString asLineString(int coordinateType) {
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		Coordinate[] coords = new Coordinate[nodeList.size()];
		switch (coordinateType) {
	
		case 0:/*projected coordinate*/
			for (int i = 0; i < nodeList.size(); i++) 
				coords[i] = new Coordinate(nodeList.get(i).getGeom().getX(), nodeList.get(i).getGeom().getY());
			break;
		case 1:/*projected normalized coordinate*/
			for (int i = 0; i < nodeList.size(); i++) 
				coords[i] = new Coordinate(nodeList.get(i).getProjectGeom().getX(), nodeList.get(i).getProjectGeom().getY());
			break;
		case 2:/*X schematic coordinate*/
			for (int i = 0; i < nodeList.size(); i++) 
				coords[i] = new Coordinate(nodeList.get(i).getxGeom().getX(), nodeList.get(i).getxGeom().getY());
			break;
		case 3:/*geographic coordinate*/
			for (int i = 0; i < nodeList.size(); i++) 
				coords[i] = new Coordinate(nodeList.get(i).getCoordinate().getX(), nodeList.get(i).getCoordinate().getY());
			break;		
		default:
			for (int i = 0; i < nodeList.size(); i++) 
				coords[i] = new Coordinate(nodeList.get(i).getCoordinate().getX(), nodeList.get(i).getCoordinate().getY());
			break;	
		}
		return geometryFactory.createLineString(coords);
	}
	
	public ArrayList<Point2D> asJava2DList(int coordinateType) {
		ArrayList<Point2D> pointList = new ArrayList<Point2D>();
		switch (coordinateType) {
		case 0:/*projection coordinate*/
			for(StreetNode n: this.getNodeList())
				pointList.add(new Point2D.Double(n.getGeom().getX(),n.getGeom().getY() ));
			
			break;
		case 1:/*projected normalized coordinate*/
			for(StreetNode n: this.getNodeList())
				pointList.add(new Point2D.Double(n.getProjectGeom().getX(),n.getProjectGeom().getY() ));
			break;
		case 2:/*X schematic coordinate*/
			for(StreetNode n: this.getNodeList())
				pointList.add(new Point2D.Double(n.getxGeom().getX(),n.getxGeom().getY() ));
			
			break;
		case 3:/*geographic coordinate*/
			for(StreetNode n: this.getNodeList())
				pointList.add(new Point2D.Double(n.getCoordinate().getX(),n.getCoordinate().getY() ));
			
			break;
		default:
			for(StreetNode n: this.getNodeList())
				pointList.add(new Point2D.Double(n.getGeom().getX(),n.getGeom().getY() ));
			break;
		}

		
		return pointList;
		
		
		
	}
	public static Comparator<Path> PathLenghtComparator = new Comparator<Path>() {

		public int compare(Path p1, Path p2) {
			
			Double pathLenght1 = p1.asLineString(0).getLength();
			Double pathLenght2 = p1.asLineString(0).getLength();


			//ascending order
			

			//descending order
			return pathLenght2.compareTo(pathLenght1);
			//return StudentName2.compareTo(StudentName1);
		}

	};
	
	
	
//	public LineString getxGeom() {
//		return xGeom;
//	}
//	public void setxGeom(LineString xGeom) {
//		this.xGeom = xGeom;
//	}
	public ArrayList<StreetNode> getNodeList() {
		return nodeList;
	}
	public void setNodeList(ArrayList<StreetNode> nodeList) {
		this.nodeList = nodeList;
	}
//	public LineString getGeom() {
//		return geom;
//	}
//	public void setGeom(LineString geom) {
//		this.geom = geom;
//	}
	public boolean isRoute() {
		return isRoute;
	}
	public void setRoute(boolean isRoute) {
		this.isRoute = isRoute;
	}
	public boolean isRouteAdjEdge() {
		return isRouteAdjEdge;
	}
	public void setRouteAdjEdge(boolean isRouteAdjEdge) {
		this.isRouteAdjEdge = isRouteAdjEdge;
	}
	
	public int getIsPolygon() {
		return isPolygon;
	}
	public void setIsPolygon(int isPolygon) {
		this.isPolygon = isPolygon;
	}
	public boolean isDisconneted() {
		return isDisconneted;
	}
	public void setDisconneted(boolean isDisconneted) {
		this.isDisconneted = isDisconneted;
	}
	
	
	public boolean isChunkPath() {
		return isChunkPath;
	}
	public void setChunkPath(boolean isChunkPath) {
		this.isChunkPath = isChunkPath;
	}
	public boolean isWasSchematized() {
		return wasSchematized;
	}
	public void setWasSchematized(boolean wasSchematized) {
		this.wasSchematized = wasSchematized;
	}
	public ArrayList<Integer> getRelevantPointIndex() {
		ArrayList<Integer> relevantPoints1 = new ArrayList<Integer>();
		for(int i = 0; i < nodeList.size(); i++) {
			if( i==0 || i == nodeList.size() -1 || nodeList.get(i).isRelevantRouteNode() )
				relevantPoints1.add(i);
		}
		return relevantPoints1;
	}
//	public void setRelevantPointIndex(ArrayList<Integer> relevantPointIndex) {
//		this.relevantPointIndex = relevantPointIndex;
//	}
	


	public int getClazz() {
		return clazz;
	}
	public void setClazz(int clazz) {
		this.clazz = clazz;
	}
	/********Update the XPosition of the Node in the path according to the position given in the final Points
	 * Set the the edgeangles connect to each node as well ( "setXStaionEdges")///
	 * CONSIDER INCLUDE THOSE TWO FUNCTION IN ANOTHER CLASS (geometric operation)*/
	public void updatePathXNodes( ArrayList<Point2D> finalPoints) {
		GeometryFactory geometryFactory = new GeometryFactory( new PrecisionModel(PrecisionModel.FLOATING), 4326);
		PointsPolar bestSolutionPolarFormat = new PointsPolar();
		bestSolutionPolarFormat  = GeometricOperation.toPolar(finalPoints);

		for(int i = 0; i < this.getNodeList().size(); i++ ){
			/*I remove this condition because i need to update schematic coordinates even if its not null. not sure if I can have error because of that*/
			if(false) {
			//if( i == 0 && this.getNodeList().get(i).getxGeom() != null ){
				this.getNodeList().get(i).setxGeom( this.getNodeList().get(i).getxGeom());
				
				/*Set bytes to record with edges are connected to this station */
				this.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i).getTheta(), this.getNodeList().get(i).getEdges()));

			}	
			else{
				Coordinate coordSource = new Coordinate(finalPoints.get(i).getX(),
						finalPoints.get(i).getY());

				this.getNodeList().get(i).setxGeom(	 geometryFactory.createPoint(coordSource) );
				
				if( i !=0 )
					if(i < this.getNodeList().size() -1 ){
						this.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i - 1).getTheta() + Math.PI, this.getNodeList().get(i).getEdges()));
						this.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i).getTheta() , this.getNodeList().get(i).getEdges()));
					}
					else{
						this.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i - 1).getTheta() + Math.PI, this.getNodeList().get(i).getEdges()));
					}
				
				
			}	
		}
		
	}
	
//	public void uptadeXGeom() {
//		this.xGeom = (LineString)GeoConvertionsOperations.Java2DToJTSGeometry(this.asJava2DList(2), Geometries.LINESTRING);	
//		
//	}
	
	public void updatePathXNodes2( ArrayList<Point2D> finalPoints) {
		GeometryFactory geometryFactory = new GeometryFactory( new PrecisionModel(PrecisionModel.FLOATING), 4326);
		PointsPolar bestSolutionPolarFormat = new PointsPolar();
		bestSolutionPolarFormat  = GeometricOperation.toPolar(finalPoints);

		for(int i = 0; i < this.getNodeList().size(); i++ ){
			Coordinate coordSource = new Coordinate(finalPoints.get(i).getX(),
					finalPoints.get(i).getY());
			this.getNodeList().get(i).setxGeom(	 geometryFactory.createPoint(coordSource) );
			
			if(i==0)
				this.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i).getTheta(), this.getNodeList().get(i).getEdges()));
			else if(i < this.getNodeList().size() -1 ){
				this.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i - 1).getTheta() + Math.PI, this.getNodeList().get(i).getEdges()));
				this.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i).getTheta() , this.getNodeList().get(i).getEdges()));
			}
			else{
				this.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i - 1).getTheta() + Math.PI, this.getNodeList().get(i).getEdges()));
			}
			
		
		}
		
	}
	
	private byte setXStaionEdges(double theta, byte edges) {
		int angle = (int)Math.round( Math.toDegrees(theta)) ;
		if(angle >= 360)
			angle = angle - 360;

		
		switch (angle) {
		case 0:
			edges |= (1 << 0);
			break;
		case 45:
			edges |= (1 << 1);	
			break;
		case 90:
			edges |= (1 << 2);	
			break;
		case 135:
			edges |= (1 << 3);	
			break;
		case 180:
			edges |= (1 << 4);	
			break;
		case 225:
			edges |= (1 << 5);
			break;
		case 270:
			edges |= (1 << 6);
			break;
		case 315:
			edges |= (1 << 7);
			break;

		}
		return edges;
	}
	
	public static Comparator<Path> PathPriorityComparator = new Comparator<Path>() {

		public int compare(Path p1, Path p2) {
			
			Integer pathPritorty1, pathPritorty2;
			if(p1.isRoute)
				pathPritorty1 = 0;
			else if(p1.getNodeList().get(0).isRouteNode())
				pathPritorty1 = 1;
			else if (p1.getNodeList().get(p1.getNodeList().size() -1).isRouteNode())
				pathPritorty1 = 2;
			else
				pathPritorty1 = 3;
			
			if(p2.isRoute)
				pathPritorty2 = 0;
			else if(p2.getNodeList().get(0).isRouteNode())
				pathPritorty2 = 1;
			else if (p2.getNodeList().get(p2.getNodeList().size() -1).isRouteNode())
				pathPritorty2 = 2;
			else
				pathPritorty2 = 3;

			//ascending order
			return pathPritorty1.compareTo(pathPritorty2);

			//descending order
			//return StudentName2.compareTo(StudentName1);
		}

	};







	public void setSmothData(ArrayList<Point2D> relevantPtList, ArrayList<Integer> keyPtIndex, int coorditateType) {
		int addNodeIndex = 0;
		switch (coorditateType) {
		case 0:
			for( int i = 0; i < this.getNodeList().size(); i++) {
				if( this.getNodeList().get(i).isRelevantRouteNode() 
						) {

					relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getNodeList().get(i).getGeom()));

					if( (
							this.getNodeList().get(i).getDegree() > 2 ) && 
							(i != 0 && i != this.getNodeList().size() -1)) {

						keyPtIndex.add(addNodeIndex);

					}
					addNodeIndex++;

				}
			}
			break;
		case 1:
			for( int i = 0; i < this.getNodeList().size(); i++) {
				if( this.getNodeList().get(i).isRelevantRouteNode() 
						) {

					relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getNodeList().get(i).getProjectGeom()));

					if( (
							this.getNodeList().get(i).getDegree() > 2 ) && 
							(i != 0 && i != this.getNodeList().size() -1)) {

						keyPtIndex.add(addNodeIndex);

					}
					addNodeIndex++;

				}
			}

			break;
		case 2:
			for( int i = 0; i < this.getNodeList().size(); i++) {
				if( this.getNodeList().get(i).isRelevantRouteNode() 
						) {

					relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getNodeList().get(i).getxGeom()));

					if( (
							this.getNodeList().get(i).getDegree() > 2 ) && 
							(i != 0 && i != this.getNodeList().size() -1)) {
						//	if( this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() &&
						//	i != 0){
						//			if( (this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() ||
						//					this.getEdgeList().get(i).getSourcePoint().getDegree() != 2) &&
						//					i != 0) {
						keyPtIndex.add(addNodeIndex);

					}
					addNodeIndex++;

				}
			}

			break;
		case 3:
			for( int i = 0; i < this.getNodeList().size(); i++) {
				if( this.getNodeList().get(i).isRelevantRouteNode() 
						) {

					relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getNodeList().get(i).getCoordinate()));

					if( (
							this.getNodeList().get(i).getDegree() > 2 ) && 
							(i != 0 && i != this.getNodeList().size() -1)) {
						//	if( this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() &&
						//	i != 0){
						//			if( (this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() ||
						//					this.getEdgeList().get(i).getSourcePoint().getDegree() != 2) &&
						//					i != 0) {
						keyPtIndex.add(addNodeIndex);

					}
					addNodeIndex++;

				}
			}

			break;	

		default:
			for( int i = 0; i < this.getNodeList().size(); i++) {
				if( this.getNodeList().get(i).isRelevantRouteNode() 
						) {

					relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getNodeList().get(i).getGeom()));

					if( (
							this.getNodeList().get(i).getDegree() > 2 ) && 
							(i != 0 && i != this.getNodeList().size() -1)) {
						//	if( this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() &&
						//	i != 0){
						//			if( (this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() ||
						//					this.getEdgeList().get(i).getSourcePoint().getDegree() != 2) &&
						//					i != 0) {
						keyPtIndex.add(addNodeIndex);

					}
					addNodeIndex++;

				}
			}
			
			break;
		}

		

		//relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getEdgeList().get(this.getEdgeList().size() -1).getTargetPoint().getxGeom()));
		//keyPtIndex.add(addNodeIndex);

	}
	
	
	public void setSmothData2(ArrayList<Point2D> relevantPtList, ArrayList<Integer> keyPtIndex, int coorditateType) {
		int addNodeIndex = 0;
		switch (coorditateType) {
		case 0:
			for( int i = 0; i < this.getNodeList().size(); i++) {
				if( this.getNodeList().get(i).isRelevantRouteNode() 
						) {

					relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getNodeList().get(i).getGeom()));

					if( (this.getNodeList().get(i).getTopoRelations().size() > 0 ||
							this.getNodeList().get(i).getDegree() > 2 ) && 
							(i != 0 && i != this.getNodeList().size() -1)) {

						keyPtIndex.add(addNodeIndex);

					}
					addNodeIndex++;

				}
			}
			break;
		case 1:
			for( int i = 0; i < this.getNodeList().size(); i++) {
				if( this.getNodeList().get(i).isRelevantRouteNode() 
						) {

					relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getNodeList().get(i).getProjectGeom()));

					if( (this.getNodeList().get(i).getTopoRelations().size() > 0 ||
							this.getNodeList().get(i).getDegree() > 2 ) && 
							(i != 0 && i != this.getNodeList().size() -1)) {

						keyPtIndex.add(addNodeIndex);

					}
					addNodeIndex++;

				}
			}

			break;
		case 2:
			for( int i = 0; i < this.getNodeList().size(); i++) {
				if( this.getNodeList().get(i).isRelevantRouteNode() 
						) {

					relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getNodeList().get(i).getxGeom()));

					if( (this.getNodeList().get(i).getTopoRelations().size() > 0 ||
							this.getNodeList().get(i).getDegree() > 2 ) && 
							(i != 0 && i != this.getNodeList().size() -1)) {
						//	if( this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() &&
						//	i != 0){
						//			if( (this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() ||
						//					this.getEdgeList().get(i).getSourcePoint().getDegree() != 2) &&
						//					i != 0) {
						keyPtIndex.add(addNodeIndex);

					}
					addNodeIndex++;

				}
			}

			break;

		default:
			for( int i = 0; i < this.getNodeList().size(); i++) {
				if( this.getNodeList().get(i).isRelevantRouteNode() 
						) {

					relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getNodeList().get(i).getProjectGeom()));

					if( (this.getNodeList().get(i).getTopoRelations().size() > 0 ||
							this.getNodeList().get(i).getDegree() > 2 ) && 
							(i != 0 && i != this.getNodeList().size() -1)) {
						//	if( this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() &&
						//	i != 0){
						//			if( (this.getEdgeList().get(i).getSourcePoint().isTopoCrossing() ||
						//					this.getEdgeList().get(i).getSourcePoint().getDegree() != 2) &&
						//					i != 0) {
						keyPtIndex.add(addNodeIndex);

					}
					addNodeIndex++;

				}
			}
			
			break;
		}

		

		//relevantPtList.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(this.getEdgeList().get(this.getEdgeList().size() -1).getTargetPoint().getxGeom()));
		//keyPtIndex.add(addNodeIndex);

	}
	
	public double getLength(int coordType) {
		double length = 0;
		switch (coordType) {
		
		case 0:
			for(int i = 0; i < this.getNodeList().size()-1; i++ ) {
				Point pt1 = this.getNodeList().get(i).getGeom();
				Point pt2 = this.getNodeList().get(i+1).getGeom();
				length+= pt1.distance(pt2);
				
			}
			/*aproximate round to meters from degrees*/
			//length = length*140000;
			break;
		case 1:
			for(int i = 0; i < this.getNodeList().size()-1; i++ ) {
				Point pt1 = this.getNodeList().get(i).getProjectGeom();
				Point pt2 = this.getNodeList().get(i+1).getProjectGeom();
				length+= pt1.distance(pt2);
				
			}
			
			break;
		case 2:
			for(int i = 0; i < this.getNodeList().size()-1; i++ ) {
				Point pt1 = this.getNodeList().get(i).getxGeom();
				Point pt2 = this.getNodeList().get(i+1).getxGeom();
				length+= pt1.distance(pt2);
				
			}
						
			break;

		default:
			for(int i = 0; i < this.getNodeList().size()-1; i++ ) {
				Point pt1 = this.getNodeList().get(i).getGeom();
				Point pt2 = this.getNodeList().get(i+1).getGeom();
				length+= pt1.distance(pt2);
				
			}
			/*aproximate round to meters from degrees*/
			length = length*140000;
			break;
		}
		return length;
	}
	
	public ArrayList<Path> divideByClazzChange(StreetNetworkTopological streetNetwork) {
		ArrayList<Path> pathList = new ArrayList<Path>();
		try {
			Path p = (Path)this.clone();
			p.setNodeList(new ArrayList<StreetNode>());
			p.getNodeList().add(this.getNodeList().get(0));
			p.getNodeList().add(this.getNodeList().get(1));
			
			
			int lastClazz = streetNetwork.getEdge(this.getNodeList().get(0), this.getNodeList().get(1)).getClazz();
			p.setClazz(lastClazz);
			for(int i = 1; i < this.getNodeList().size() -1; i++ ) {
				StreetNode n1 = this.getNodeList().get(i);
				StreetNode n2 = this.getNodeList().get(i+1);
				
				
				if( streetNetwork.getEdge(n1, n2).getClazz() == lastClazz ) {
					p.getNodeList().add(n2);
					
				}
				else {
					pathList.add(p);
					lastClazz = streetNetwork.getEdge(n1, n2).getClazz();
					p = (Path)this.clone();
					p.setNodeList(new ArrayList<StreetNode>());
					p.getNodeList().add(n1);
					p.getNodeList().add(n2);
					p.setClazz(lastClazz);
					
				}
				
				
				
			}
			pathList.add(p);
			
	
		} catch (CloneNotSupportedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		
		return pathList;
	}
	
	public ArrayList<Path> divideByDP(StreetNetworkTopological streetNetwork) {
		ArrayList<Path> pathList = new ArrayList<Path>();
		try {
			Path p = (Path)this.clone();
			p.setNodeList(new ArrayList<StreetNode>());
			p.getNodeList().add(this.getNodeList().get(0));
			

			for(int i = 1; i < this.getNodeList().size(); i++ ) {
				StreetNode n1 = this.getNodeList().get(i);
				
				
				
				if( !n1.isDecisionPoint() || i == this.getNodeList().size() -1 ) {
					p.getNodeList().add(n1);
					
				}
				else {
					p.getNodeList().add(n1);
					pathList.add(p);

					p = (Path)this.clone();
					p.setNodeList(new ArrayList<StreetNode>());
					p.getNodeList().add(n1);

					
				}
				
				
				
			}
			pathList.add(p);
			
	
		} catch (CloneNotSupportedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		
		return pathList;
	}
	
	
	@Override
	public Object clone() throws CloneNotSupportedException {
			// TODO Auto-generated method stub
			return super.clone();
		}













	
	
	
}
