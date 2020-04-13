package com.wayto.operator;

import java.util.Comparator;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Point;

import com.wayto.model.StreetEdge;


public class StreetEdgeDistanceToPointComparator implements Comparator<StreetEdge>{
	
	Coordinate coordinateReference;

	public StreetEdgeDistanceToPointComparator(Coordinate ptReference) {
		this.coordinateReference = ptReference;
	}
    public int compare(StreetEdge e1, StreetEdge e2) {
    	Point sourceOrigPoint1 = e1.getSourcePoint().getProjectGeom();
		Point targetOrigPoint1 = e1.getTargetPoint().getProjectGeom();
		
		Point sourceOrigPoint2 = e2.getSourcePoint().getProjectGeom();
		Point targetOrigPoint2 = e2.getTargetPoint().getProjectGeom();
    	
    	Double distE1 = (coordinateReference.distance(sourceOrigPoint1.getCoordinate()) + coordinateReference.distance(targetOrigPoint1.getCoordinate()))/2;
    	Double distE2 = (coordinateReference.distance(sourceOrigPoint2.getCoordinate()) + coordinateReference.distance(targetOrigPoint2.getCoordinate()))/2;


        return Double.compare(distE1, distE2);
    }
	
}
