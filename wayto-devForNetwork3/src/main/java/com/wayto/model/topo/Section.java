package com.wayto.model.topo;

import java.util.ArrayList;

import org.locationtech.jts.geom.LineString;
import com.wayto.model.StreetNode;
import com.wayto.operator.GeometricOperation;

public class Section {
	/**
	 * CP1 stars section
	 * CP2 ends section
	 */
	private int  CP1Index;
	private int CP2Index;
	
	/***Edges id to guarantee correct direction on the intersection with the route, 
	 * remains null if section extreme is relative CP 
	 * (setted in in setextremeEdge() ***/
	private Integer startFakeEdgeId = null; 
	private Integer endFakeEdgeId = null;
	
	private LineString shape;
	private LineString shapeProjected;
	private LineString shapeTranformed;
	private LineString shapeX;
	
	private boolean followTheRoute = false;

	public Section(int cP1Index, int cP2Index, LineString shape, boolean followTheRoute) {
		super();
		CP1Index = cP1Index;
		CP2Index = cP2Index;
		this.followTheRoute = followTheRoute;
		this.shape = shape;
	}

	public int getCP1Index() {
		return CP1Index;
	}

	public void setCP1Index(int cP1Index) {
		CP1Index = cP1Index;
	}

	public int getCP2Index() {
		return CP2Index;
	}

	public void setCP2Index(int cP2Index) {
		CP2Index = cP2Index;
	}

	public LineString getShape() {
		return shape;
	}

	public void setShape(LineString shape) {
		this.shape = shape;
	}

	public boolean isFollowTheRoute() {
		return followTheRoute;
	}

	public void setFollowTheRoute(boolean followTheRoute) {
		this.followTheRoute = followTheRoute;
	}

	public LineString getShapeX() {
		return shapeX;
	}

	public void setShapeX(LineString shapeX) {
		this.shapeX = shapeX;
	}

	public Integer getStartFakeEdgeId() {
		return startFakeEdgeId;
	}

	public void setStartFakeEdgeId(Integer startFakeEdgeId) {
		this.startFakeEdgeId = startFakeEdgeId;
	}

	public Integer getEndFakeEdgeId() {
		return endFakeEdgeId;
	}

	public void setEndFakeEdgeId(Integer endFakeEdgeId) {
		this.endFakeEdgeId = endFakeEdgeId;
	}

	public LineString getShapeProjected() {
		return shapeProjected;
	}

	public void setShapeProjected(LineString shapeProjected) {
		this.shapeProjected = shapeProjected;
	}

	public LineString getShapeTranformed() {
		return shapeTranformed;
	}

	public void setShapeTranformed(LineString shapeTranformed) {
		this.shapeTranformed = shapeTranformed;
	}
	
	
	
//	public int getFirstEdgeDirection(ControlPoint controlPoint, ArrayList<StreetNode> neibNodeList) {
//		double nodeX = controlPoint.getNode().getGeom().getX();
//		double nodeY =controlPoint.getNode().getGeom().getY();
//		ArrayList<Integer> order = new ArrayList<Integer>();
//		
//		ArrayList<Double> angles = new ArrayList<Double>();
//		for(int i = 0; i < neibNodeList.size(); i++){
//			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(
//					nodeX, nodeY,
//					neibNodeList.get(i).getGeom().getX(), neibNodeList.get(i).getGeom().getY() 
//				);
//			angles.add(angle);
//		}
//		angles.add(GeometricOperation.getAngleBetweenPointsRelativeToAxisX(nodeX, nodeY,
//				this.getShape().getCoordinateN(1).x, this.getShape().getCoordinateN(1).y));
//		for(int i = 0; i < angles.size(); i++){
//			order.add(i);
//			
//		}
//		for(int i = 0; i< angles.size(); i++){
//			for(int j = 0; j < order.size(); i++){
//				if(angles.get(i) < angles.get( order.get(j) ) )
//			
//			}	
//			
//		}
//		
//		
//		
//		
//		double minimalAngle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(nodeX, nodeY,
//				this.getShape().getCoordinateN(1).x, this.getShape().getCoordinateN(1).y);
//		for(int i = 0; i < neibNodeList.size(); i++){
//			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(
//					nodeX, nodeY,
//					neibNodeList.get(i).getGeom().getX(), neibNodeList.get(i).getGeom().getY() 
//				);
//			if (angle < minimalAngle)
//				minimalAngle = angle;
//		}
//		
//		
//		
//		
//		Integer firstEdgeOrignalDir = GeometricOperation.sectorOf(GeometricOperation.getAngleBetweenPointsRelativeToAxisX(nodeX, nodeY,
//				this.getShape().getCoordinateN(1).x, this.getShape().getCoordinateN(1).y));
//
//
//		int firstNodeBestDir = GeometricOperation.sectorOf(
//				GeometricOperation.getAngleBetweenPointsRelativeToAxisX(
//						nodeX, nodeY,
//						neibNodeList.get(0).getGeom().getX(), neibNodeList.get(0).getGeom().getY() 
//					)
//			);
//		int nodeIdBottomBound = 0;
//		int nodeIdTopBound = 0;
//		double minimalAngle =firstNodeBestDir;
//		int topBound = firstNodeBestDir;
//		
//		
//		for(int i = 1; i < neibNodeList.size(); i++){
//			int thisBestDir = GeometricOperation.sectorOf(
//					GeometricOperation.getAngleBetweenPointsRelativeToAxisX(
//							nodeX, nodeY,
//							neibNodeList.get(i).getGeom().getX(), neibNodeList.get(i).getGeom().getY() 
//						)
//				);
//			if(origBestDir >= bottomBound && origBestDir <=  firstEdgeOrignalDir){
//				nodeIdBottomBound = i;
//				bottomBound = origBestDir;
//			}
//				
//			
//			if(origBestDir <= topBound && origBestDir >= firstEdgeOrignalDir)
//				nodeIdTopBound = i;
//			
//				
//			
//			
//		}
//		
//		
//		ArrayList<Integer> dirArrayList = new ArrayList<Integer>();
//		for(StreetNode n: neibNodeList){
//			Integer origBestDir = GeometricOperation.sectorOf(
//					GeometricOperation.getAngleBetweenPointsRelativeToAxisX(
//							nodeX, nodeY,
//							n.getGeom().getX(), n.getGeom().getY() 
//						)
//					);
//			dirArrayList.add(origBestDir);
//		}
//		Integer firstEdgeOrignalDir = GeometricOperation.sectorOf(GeometricOperation.getAngleBetweenPointsRelativeToAxisX(nodeX, nodeY,
//				this.getShape().getCoordinateN(1).x, this.getShape().getCoordinateN(1).y));
//		dirArrayList.add(firstEdgeOrignalDir);
//		
//		
//		
//		
//		this.getShape().getCoordinateN(1)
//		
//		return 0;
//	}

	
	



	
	
}
