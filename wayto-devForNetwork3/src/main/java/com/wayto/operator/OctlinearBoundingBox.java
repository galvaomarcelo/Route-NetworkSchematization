package com.wayto.operator;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.locationtech.jts.geom.Polygon;


public class OctlinearBoundingBox {
	
	private double maxAdjDist, maxX, minX, maxY, minY, maxZ1, minZ1, maxZ2, minZ2;
	private double xExtend ;
	private double yExtend ;
	private double z1Extend;
	private double z2Extend ;
	

	

	
	
	public OctlinearBoundingBox(ArrayList<Point2D> polyLine, double distMin, double extendLimit) {
		super();


		maxX = polyLine.get(0).getX();
		minX = polyLine.get(0).getX();
		maxY = polyLine.get(0).getY();
		minY = polyLine.get(0).getY();
		maxZ1 = polyLine.get(0).getX() + polyLine.get(0).getY();
		minZ1 = maxZ1;
		maxZ2 = polyLine.get(0).getX() - polyLine.get(0).getY();
		minZ2 = maxZ2;
		maxAdjDist = polyLine.get(0).distance(polyLine.get(1));
		for (int i = 1; i < polyLine.size(); i++) {
			
			double x =polyLine.get(i).getX();
			double y =polyLine.get(i).getY();
		
			double z1 = x + y;
			double z2 = x - y;
			maxX = (x > maxX) ? x : maxX;
			minX = (x < minX) ? x : minX;
			maxY = (y > maxY) ? y : maxY;
			minY = (y < minY) ? y : minY;
			maxZ1 = ( z1 > maxZ1) ? z1 : maxZ1;
			minZ1 = (z1 < minZ1) ? z1 : minZ1;
			maxZ2 = (z2 > maxZ2) ? z2 : maxZ2;
			minZ2 = (z2 < minZ2) ? z2 : minZ2;
			double dist = polyLine.get(i).distance(polyLine.get(i-1));
			maxAdjDist = (dist > maxAdjDist)? dist : maxAdjDist;
		}
//		minZ1 = (minY + minX);
//		maxZ1 = (maxX + maxY);
	//	
//		minZ2 = (minX - maxY);
//		maxZ2 = (maxX - minY);
		

	    
	    
		xExtend = Math.max(maxX - minX, distMin*10);
		yExtend = Math.max(maxY - minY, distMin*10);
		z1Extend = Math.max(maxZ1 - minZ1, distMin*10) ;
		z2Extend = Math.max(maxZ2 - minZ2, distMin*10);
		System.out.println("xExtend: " + xExtend + ", yExtend: " + yExtend);
		System.out.println("z1Extend: " + z1Extend + ", z2Extend: " + z2Extend);
	//
//	    System.out.println("maxD: " + maxD);
//	    System.out.println("N: " + n);

		/*********COORDINATES CONSTRAINT*****************/
	    
		minX = minX - extendLimit*xExtend;
	    maxX = maxX + extendLimit*xExtend;
	    minY = minY - extendLimit*yExtend;
	    maxY = maxY + extendLimit*yExtend;
	    minZ1 = minZ1 - extendLimit*z1Extend;
	    maxZ1 = maxZ1 + extendLimit*z1Extend;
	    minZ2 = minZ2 - extendLimit*z2Extend;
	    maxZ2 =  maxZ2 + extendLimit*z2Extend;
	    

		xExtend = maxX - minX;
		yExtend = maxY - minY;
		z1Extend = maxZ1 - minZ1 ;
		z2Extend = maxZ2 - minZ2;
		System.out.println(polyLine);
		System.out.println("max X: " + maxX + ", min X: " + minX);
	    System.out.println("max Y: " + maxY + ", min Y: " + minY);
	    System.out.println("max Z1: " + maxZ1 + ", min Z1: " + minZ1);
	    System.out.println("max Z2: " + maxZ2 + ", min Z2: " + minZ2);
		
		
	}
	
	/*type 1: x+y. type 2 x+y/2.  default accurate*/
	public OctlinearBoundingBox(ArrayList<Point2D> polyLine, double distMin, double extendLimit, int type) {
		super();


		switch (type) {
		case 1:
			maxX = polyLine.get(0).getX();
			minX = polyLine.get(0).getX();
			maxY = polyLine.get(0).getY();
			minY = polyLine.get(0).getY();
			maxZ1 = polyLine.get(0).getX() + polyLine.get(0).getY();
			minZ1 = maxZ1;
			maxZ2 = polyLine.get(0).getX() - polyLine.get(0).getY();
			minZ2 = maxZ2;
			maxAdjDist = polyLine.get(0).distance(polyLine.get(1));
			for (int i = 1; i < polyLine.size(); i++) {
				
				double x =polyLine.get(i).getX();
				double y =polyLine.get(i).getY();
			
				double z1 = x + y;
				double z2 = x - y;
				maxX = (x > maxX) ? x : maxX;
				minX = (x < minX) ? x : minX;
				maxY = (y > maxY) ? y : maxY;
				minY = (y < minY) ? y : minY;
				maxZ1 = ( z1 > maxZ1) ? z1 : maxZ1;
				minZ1 = (z1 < minZ1) ? z1 : minZ1;
				maxZ2 = (z2 > maxZ2) ? z2 : maxZ2;
				minZ2 = (z2 < minZ2) ? z2 : minZ2;
				double dist = polyLine.get(i).distance(polyLine.get(i-1));
				maxAdjDist = (dist > maxAdjDist)? dist : maxAdjDist;
			}
			break;
		case 2:
			maxX = polyLine.get(0).getX();
			minX = polyLine.get(0).getX();
			maxY = polyLine.get(0).getY();
			minY = polyLine.get(0).getY();
			maxZ1 = (polyLine.get(0).getX() + polyLine.get(0).getY())/2;
			minZ1 = maxZ1;
			maxZ2 = (polyLine.get(0).getX() - polyLine.get(0).getY())/2;
			minZ2 = maxZ2;
			maxAdjDist = polyLine.get(0).distance(polyLine.get(1));
			for (int i = 1; i < polyLine.size(); i++) {
				
				double x =polyLine.get(i).getX();
				double y =polyLine.get(i).getY();
			
				double z1 = (x + y)/2;
				double z2 = (x - y)/2;
				maxX = (x > maxX) ? x : maxX;
				minX = (x < minX) ? x : minX;
				maxY = (y > maxY) ? y : maxY;
				minY = (y < minY) ? y : minY;
				maxZ1 = ( z1 > maxZ1) ? z1 : maxZ1;
				minZ1 = (z1 < minZ1) ? z1 : minZ1;
				maxZ2 = (z2 > maxZ2) ? z2 : maxZ2;
				minZ2 = (z2 < minZ2) ? z2 : minZ2;
				double dist = polyLine.get(i).distance(polyLine.get(i-1));
				maxAdjDist = (dist > maxAdjDist)? dist : maxAdjDist;
			}
			
			break;	

		default:
			System.out.println("True BB");
			maxX = polyLine.get(0).getX();
			minX = polyLine.get(0).getX();
			maxY = polyLine.get(0).getY();
			minY = polyLine.get(0).getY();
			double range = 2;
			double range2 = 2;
			if(maxX == 0 && maxY == 0) {
				maxZ1 = 0;
				maxZ2 = 0;
			}else {
				
				Point2D.Double pt = GeometricOperation.getClosestPointOnSegment(-range, -range, range2, range2, maxX, maxY);
				if(pt.getX()>0)
					maxZ1=pt.distance(0, 0);
				else
					maxZ1=-pt.distance(0, 0);
				
				//maxZ1 = GeometricOperation.getClosestPointOnSegment(-range, -range, range2, range2, maxX, maxY).distance(0, 0);

				pt = GeometricOperation.getClosestPointOnSegment(-range, range, range2, -range2, maxX, maxY);
				if(pt.getX()>0)
					maxZ2=pt.distance(0, 0);
				else
					maxZ2=-pt.distance(0, 0);
				//maxZ1 = GeometricOperation.getClosestPointOnVector(0, 0, Math.max(maxX, maxY), Math.max(maxX, maxY), maxX, maxY);
				//maxZ2 =GeometricOperation.getClosestPointOnVector(0, 0, Math.max(maxX, maxY), -Math.max(maxX, maxY), maxX, maxY);
			}			
			minZ1 = maxZ1;
			minZ2 = maxZ2;
			maxAdjDist = polyLine.get(0).distance(polyLine.get(1));
			for (int i = 1; i < polyLine.size(); i++) {
				
				double x =polyLine.get(i).getX();
				double y =polyLine.get(i).getY();
				
				Point2D.Double pt = GeometricOperation.getClosestPointOnSegment(-range, -range, range2, range2, x, y);
				double z1=pt.distance(0, 0);
				if(pt.getX()<0)
					z1=-z1;
				pt = GeometricOperation.getClosestPointOnSegment(-range, range, range2, -range2, x, y);
				double z2 =pt.distance(0, 0);
				if(pt.getX()<0)
					z2=-z2;
				maxX = (x > maxX) ? x : maxX;
				minX = (x < minX) ? x : minX;
				maxY = (y > maxY) ? y : maxY;
				minY = (y < minY) ? y : minY;
				maxZ1 = ( z1 > maxZ1) ? z1 : maxZ1;
				minZ1 = (z1 < minZ1) ? z1 : minZ1;
				maxZ2 = (z2 > maxZ2) ? z2 : maxZ2;
				minZ2 = (z2 < minZ2) ? z2 : minZ2;
				double dist = polyLine.get(i).distance(polyLine.get(i-1));
				maxAdjDist = (dist > maxAdjDist)? dist : maxAdjDist;
			}
			
			break;
		}

//		minZ1 = (minY + minX);
//		maxZ1 = (maxX + maxY);
	//	
//		minZ2 = (minX - maxY);
//		maxZ2 = (maxX - minY);
		    
		xExtend = Math.max(maxX - minX, distMin*10);
		yExtend = Math.max(maxY - minY, distMin*10);
		z1Extend = Math.max(maxZ1 - minZ1, distMin*10) ;
		z2Extend = Math.max(maxZ2 - minZ2, distMin*10);
		System.out.println("xExtend: " + xExtend + ", yExtend: " + yExtend);
		System.out.println("z1Extend: " + z1Extend + ", z2Extend: " + z2Extend);

		minX = minX - extendLimit*xExtend;
	    maxX = maxX + extendLimit*xExtend;
	    minY = minY - extendLimit*yExtend;
	    maxY = maxY + extendLimit*yExtend;
	    minZ1 = minZ1 - extendLimit*z1Extend;
	    maxZ1 = maxZ1 + extendLimit*z1Extend;
	    minZ2 = minZ2 - extendLimit*z2Extend;
	    maxZ2 =  maxZ2 + extendLimit*z2Extend;
	    

		xExtend = maxX - minX;
		yExtend = maxY - minY;
		z1Extend = maxZ1 - minZ1 ;
		z2Extend = maxZ2 - minZ2;
//		System.out.println(polyLine);
//		System.out.println("max X: " + maxX + ", min X: " + minX);
//	    System.out.println("max Y: " + maxY + ", min Y: " + minY);
//	    System.out.println("max Z1: " + maxZ1 + ", min Z1: " + minZ1);
//	    System.out.println("max Z2: " + maxZ2 + ", min Z2: " + minZ2);
		
		
	}
	
	public OctlinearBoundingBox(ArrayList<Point2D> polyLine, double distMin, double extendLimit, boolean accutare) {
		super();

		System.out.println("True BB");
		maxX = polyLine.get(0).getX();
		minX = polyLine.get(0).getX();
		maxY = polyLine.get(0).getY();
		minY = polyLine.get(0).getY();
		maxZ1 = GeometricOperation.getClosestPointOnVector(0, 0, Math.max(maxX, maxY), Math.max(maxX, maxY), maxX, maxY);
		minZ1 = maxZ1;
		maxZ2 =GeometricOperation.getClosestPointOnVector(0, 0, Math.max(maxX, maxY), -Math.max(maxX, maxY), maxX, maxY);
		minZ2 = maxZ2;
		maxAdjDist = polyLine.get(0).distance(polyLine.get(1));
		for (int i = 1; i < polyLine.size(); i++) {
			
			double x =polyLine.get(i).getX();
			double y =polyLine.get(i).getY();
			GeometricOperation.getClosestPointOnVector(0, 0, Math.max(x, y), Math.max(x, y), x, y);
			double z1 = GeometricOperation.getClosestPointOnVector(0, 0, Math.max(x, y), Math.max(x, y), x, y);
			double z2 = GeometricOperation.getClosestPointOnVector(0, 0, Math.max(x, y), -Math.max(x, y), x, y);
			maxX = (x > maxX) ? x : maxX;
			minX = (x < minX) ? x : minX;
			maxY = (y > maxY) ? y : maxY;
			minY = (y < minY) ? y : minY;
			maxZ1 = ( z1 > maxZ1) ? z1 : maxZ1;
			minZ1 = (z1 < minZ1) ? z1 : minZ1;
			maxZ2 = (z2 > maxZ2) ? z2 : maxZ2;
			minZ2 = (z2 < minZ2) ? z2 : minZ2;
			double dist = polyLine.get(i).distance(polyLine.get(i-1));
			maxAdjDist = (dist > maxAdjDist)? dist : maxAdjDist;
		}
//		minZ1 = (minY + minX);
//		maxZ1 = (maxX + maxY);
	//	
//		minZ2 = (minX - maxY);
//		maxZ2 = (maxX - minY);
		

	    
	    
		xExtend = Math.max(maxX - minX, distMin*10);
		yExtend = Math.max(maxY - minY, distMin*10);
		z1Extend = Math.max(maxZ1 - minZ1, distMin*10) ;
		z2Extend = Math.max(maxZ2 - minZ2, distMin*10);
		System.out.println("xExtend: " + xExtend + ", yExtend: " + yExtend);
		System.out.println("z1Extend: " + z1Extend + ", z2Extend: " + z2Extend);
	//
//	    System.out.println("maxD: " + maxD);
//	    System.out.println("N: " + n);

		/*********COORDINATES CONSTRAINT*****************/
	    
		minX = minX - extendLimit*xExtend;
	    maxX = maxX + extendLimit*xExtend;
	    minY = minY - extendLimit*yExtend;
	    maxY = maxY + extendLimit*yExtend;
	    minZ1 = minZ1 - extendLimit*z1Extend;
	    maxZ1 = maxZ1 + extendLimit*z1Extend;
	    minZ2 = minZ2 - extendLimit*z2Extend;
	    maxZ2 =  maxZ2 + extendLimit*z2Extend;
	    

		xExtend = maxX - minX;
		yExtend = maxY - minY;
		z1Extend = maxZ1 - minZ1 ;
		z2Extend = maxZ2 - minZ2;
//		System.out.println(polyLine);
//		System.out.println("max X: " + maxX + ", min X: " + minX);
//	    System.out.println("max Y: " + maxY + ", min Y: " + minY);
//	    System.out.println("max Z1: " + maxZ1 + ", min Z1: " + minZ1);
//	    System.out.println("max Z2: " + maxZ2 + ", min Z2: " + minZ2);
		
		
	}

	/*I took pictures of the calculation 19-nov2018*/
	public ArrayList<Point2D> getBoundingPolygon() {
		Polygon p = null;
		double cos45 = Math.cos(Math.toRadians(45));
		System.out.println("cos45: " + cos45);
		ArrayList<Point2D> polyPtList = new ArrayList<Point2D>();

		
		polyPtList.add(new Point2D.Double(maxX , -maxX + maxZ1/cos45) );
		polyPtList.add(new Point2D.Double(maxZ1/cos45 - maxY , maxY));
		polyPtList.add(new Point2D.Double( maxY + minZ2/cos45, maxY));
		polyPtList.add(new Point2D.Double(minX , minX -minZ2/cos45));
		polyPtList.add(new Point2D.Double(minX , -minX +minZ1/cos45));
		polyPtList.add(new Point2D.Double(-minY + minZ1/cos45 , minY));
		polyPtList.add(new Point2D.Double(minY + maxZ2/cos45 , minY));
		polyPtList.add(new Point2D.Double(maxX , maxX - maxZ2/cos45));
		polyPtList.add(new Point2D.Double(maxX , -maxX + maxZ1/cos45) );
		
		System.out.println(polyPtList);
		
		
		return polyPtList;
			
	}

	public double getMaxAdjDist() {
		return maxAdjDist;
	}
	
	/*I took pictures of the calculation 19-nov2018*/
	public ArrayList<Point2D> getBoundingPolygon2() {
		Polygon p = null;
		double cos45 = Math.cos(Math.toRadians(45));
		System.out.println("cos45: " + cos45);
		ArrayList<Point2D> polyPtList = new ArrayList<Point2D>();

		
		polyPtList.add(new Point2D.Double(maxX , -maxX + maxZ1*2) );
		polyPtList.add(new Point2D.Double(maxZ1*2 - maxY , maxY));
		polyPtList.add(new Point2D.Double( maxY + minZ2*2, maxY));
		polyPtList.add(new Point2D.Double(minX , minX -minZ2*2));
		polyPtList.add(new Point2D.Double(minX , -minX +minZ1*2));
		polyPtList.add(new Point2D.Double(-minY + minZ1*2 , minY));
		polyPtList.add(new Point2D.Double(minY + maxZ2*2 , minY));
		polyPtList.add(new Point2D.Double(maxX , maxX - maxZ2*2));
		
		System.out.println(polyPtList);
		
		
		return polyPtList;
			
	}





	public double getMaxX() {
		return maxX;
	}




	public double getMinX() {
		return minX;
	}




	public double getMaxY() {
		return maxY;
	}




	public double getMinY() {
		return minY;
	}




	public double getMaxZ1() {
		return maxZ1;
	}




	public double getMinZ1() {
		return minZ1;
	}




	public double getMaxZ2() {
		return maxZ2;
	}




	public double getMinZ2() {
		return minZ2;
	}




	public double getMaxExtend() {
		double maxExtend = 0;
		maxExtend = (xExtend > yExtend) ? xExtend : yExtend;
		maxExtend = (z1Extend > maxExtend) ? z1Extend : maxExtend;
		maxExtend = (z2Extend > maxExtend) ? z2Extend : maxExtend;
		
		return maxExtend;
	}




	public double getxExtend() {
		return xExtend;
	}




	public double getyExtend() {
		return yExtend;
	}




	public double getZ1Extend() {
		return z1Extend;
	}




	public double getZ2Extend() {
		return z2Extend;
	}
	
	
	

}
