package com.wayto.operator;

import java.util.Comparator;

import org.locationtech.jts.geom.Point;
import org.locationtech.jts.linearref.LengthIndexedLine;


public class PointRouteIndexedOrderComparator implements Comparator<Point>{
	
	LengthIndexedLine indexdRoute;

	public PointRouteIndexedOrderComparator(LengthIndexedLine indexdRoute) {
		this.indexdRoute = indexdRoute;
	}
    public int compare(Point p1, Point p2) {
    	Double indexP1 = indexdRoute.indexOf(p1.getCoordinate()); 
    	Double indexP2 = indexdRoute.indexOf(p2.getCoordinate());

        return Double.compare(indexP1, indexP2);
    }
	
}
