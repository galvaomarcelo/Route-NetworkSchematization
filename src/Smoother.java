package com.wayto.operator;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import org.geotools.geometry.jts.JTS;
import org.geotools.geometry.jts.JTSFactoryFinder;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryCollection;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.operation.linemerge.LineMerger;

public final class Smoother {


	public static Geometry smoothSpline(LineString targetLine,ArrayList<Integer> indexPointList, double fitness)
	{
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		//split the polyline by the deciding points and apply the spline function directly
		//ArrayList<Point> pointList = new ArrayList<Point>();//store the angle keeping  points
		ArrayList<LineString> splitedLineString = new ArrayList<LineString>();
		Coordinate[] tempCoords =  targetLine.getCoordinates();
		ArrayList<LineString> splinedCurves = new ArrayList<LineString>();
		for(int i=0;i<indexPointList.size();i++)
		{
			//Point tempPoint = targetLine.getPointN(indexPointList.get(i));
			//pointList.add(tempPoint);
			//split
			Coordinate[] subTempCoords;
			if(i==0)
			{
				subTempCoords = Arrays.copyOfRange(tempCoords, 0, indexPointList.get(i));
			}else
			{
				subTempCoords = Arrays.copyOfRange(tempCoords, indexPointList.get(i-1)-1, indexPointList.get(i));
			}

			LineString tempLine = geometryFactory.createLineString(subTempCoords);
			splitedLineString.add(tempLine);
			splinedCurves.add((LineString)JTS.smooth(splitedLineString.get(i), fitness));
			//drawOnMap(splinedCurves.get(i));
		}
		//last segment
		Coordinate[] subTempCoords1 = Arrays.copyOfRange(tempCoords, indexPointList.get(indexPointList.size()-1)-1, tempCoords.length);
		LineString tempLine = geometryFactory.createLineString(subTempCoords1);
		splitedLineString.add(tempLine);
		splinedCurves.add((LineString)JTS.smooth(splitedLineString.get(splitedLineString.size()-1), fitness));
		//drawOnMap(splinedCurves.get(splinedCurves.size()-1));
		//System.out.println("splitedLineString"+splitedLineString.toString());
		//merge the curve
		GeometryCollection geometryCollection = (GeometryCollection) geometryFactory.buildGeometry(splinedCurves);
		//System.out.println("geometryCollection:"+geometryCollection.toString());
		return geometryCollection.union();
	}

	public static LineString smoothSpline2(LineString targetLine,ArrayList<Integer> indexPointList, double fitness)
	{
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		if(!indexPointList.contains(targetLine.getNumPoints() - 1))
			indexPointList.add(targetLine.getNumPoints() - 1);
		Coordinate[] tempCoords = targetLine.getCoordinates();
		//split the polyline by the deciding points and apply the spline function directly
		ArrayList<Point> pointList = new ArrayList<Point>();//store the angle keeping  points
		ArrayList<LineString> splitedLineString = new ArrayList<LineString>();
		ArrayList<LineString> splinedCurves = new ArrayList<LineString>();
		LineMerger merger = new LineMerger();
		for(int i=0;i<indexPointList.size();i++)
		{
			Point tempPoint = targetLine.getPointN(indexPointList.get(i));
			pointList.add(tempPoint);
			//split
			Coordinate[] subTempCoords;
			if(i==0)
			{
				subTempCoords = Arrays.copyOfRange(tempCoords, 0, indexPointList.get(i) + 1);
			}else
			{
				subTempCoords = Arrays.copyOfRange(tempCoords, indexPointList.get(i-1), indexPointList.get(i) + 1);

			}

			// simplify the point
			List<Coordinate> coordinatesList = new ArrayList<Coordinate>();

			for (int a=0; a<subTempCoords.length; a++) {  
				coordinatesList.add(subTempCoords[a]);  
			}  
			//angle filter
			for(int p = 0;p< subTempCoords.length-2;p++)
			{
				Point startPoint = geometryFactory.createPoint(subTempCoords[p]);
				Point midPoint = geometryFactory.createPoint(subTempCoords[p+1]);
				Point endPoint = geometryFactory.createPoint(subTempCoords[p+2]);
				boolean isLine = GeometricOperation.areAligned(startPoint,midPoint,endPoint);
				if(isLine == true)
				{
					coordinatesList.remove(midPoint.getCoordinate());
				}
			}
			Coordinate[] finalSubCoords =  coordinatesList.toArray(new Coordinate[1]);
			//draw the rest point
			ArrayList<Point> tempPointList = new ArrayList<Point>();
			for(int b =0 ;b< finalSubCoords.length;b++)
			{
				tempPointList.add(geometryFactory.createPoint(finalSubCoords[b]));
			}
			LineString tempLine = geometryFactory.createLineString(finalSubCoords);
			splitedLineString.add(tempLine);
			merger.add((LineString)JTS.smooth(splitedLineString.get(i), fitness));
		}
		//merge the curve
				Collection<LineString> collection = merger.getMergedLineStrings();
				LineString tempLine = null;
				for (LineString l : collection) {
					tempLine = l;
				}
		return tempLine;
	}


	public static LineString geometrySmoothCut(LineString targetLine,ArrayList<Integer> indexPointList,double tension, int nOfinteration)
	{
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		if(!indexPointList.contains(targetLine.getNumPoints() - 1))
			indexPointList.add(targetLine.getNumPoints() - 1);
		Coordinate[] tempCoords = targetLine.getCoordinates();
		//split the polyline by the deciding points and apply the spline function directly
		ArrayList<Point> pointList = new ArrayList();//store the angle keeping  points
		ArrayList<LineString> splitedLineString = new ArrayList();
		LineMerger merger = new LineMerger();
		for(int i=0;i<indexPointList.size();i++)
		{
			Point tempPoint = targetLine.getPointN(indexPointList.get(i));
			pointList.add(tempPoint);
			//split
			Coordinate[] subTempCoords;
			if(i==0)
			{
				subTempCoords = Arrays.copyOfRange(tempCoords, 0, indexPointList.get(i) + 1);
			}else
			{
				subTempCoords = Arrays.copyOfRange(tempCoords, indexPointList.get(i-1), indexPointList.get(i) + 1);

			}

			// simplify the point
			List<Coordinate> coordinatesList = new ArrayList<Coordinate>();

			for (int a=0; a<subTempCoords.length; a++) {  
				coordinatesList.add(subTempCoords[a]);  
			}  
			//angle filter
			for(int p = 0;p< subTempCoords.length-2;p++)
			{
				Point startPoint = geometryFactory.createPoint(subTempCoords[p]);
				Point midPoint = geometryFactory.createPoint(subTempCoords[p+1]);
				Point endPoint = geometryFactory.createPoint(subTempCoords[p+2]);
				boolean isLine = GeometricOperation.areAligned(startPoint,midPoint,endPoint);
				if(isLine == true)
				{
					coordinatesList.remove(midPoint.getCoordinate());
				}
			}
			Coordinate[] finalSubCoords =  coordinatesList.toArray(new Coordinate[1]);
			//draw the rest point
			ArrayList<Point> tempPointList = new ArrayList();
			for(int b =0 ;b< finalSubCoords.length;b++)
			{
				tempPointList.add(geometryFactory.createPoint(finalSubCoords[b]));
			}
			//drawPointsOnMap(tempPointList,Color.GREEN);

			LineString tempLine = geometryFactory.createLineString(finalSubCoords);
			splitedLineString.add(tempLine);
			merger.add(cutEdge(splitedLineString.get(i),tension,nOfinteration));
		}
		//merge the curve
		Collection<LineString> collection = merger.getMergedLineStrings();
		LineString tempLine = null;
		for (LineString l : collection) {
			tempLine = l;
		}
		return tempLine;
	}

	private static LineString cutEdge(LineString targetLine, double tension, int nOfinteration){
		
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		int numPts = targetLine.getNumPoints();
		List<Point2D.Double> mPoints = new ArrayList<Point2D.Double>();
		CuttingCurve cutCurve =  new CuttingCurve();
		for(int i =0; i<numPts;i++)
		{
			//insidePoint.add(targetLine.getPointN(i));
			Point2D.Double tempMPoint = new Point2D.Double();
			tempMPoint.x = targetLine.getCoordinateN(i).x;
			tempMPoint.y = targetLine.getCoordinateN(i).y;
			cutCurve.addPoint(tempMPoint);
		}
		mPoints = cutCurve.smoothByChaikin(tension, nOfinteration);
		List<Coordinate> coordinates = new ArrayList<Coordinate>();
		ArrayList<Point> createdPointList = new ArrayList<Point>();
		for(int i =0;i <mPoints.size();i++)
		{
			Coordinate tempCoord1  = new Coordinate(mPoints.get(i).getX(),mPoints.get(i).getY());
			createdPointList.add(geometryFactory.createPoint(tempCoord1));
			coordinates.add(tempCoord1);
		}
		Coordinate[] finalCoords =  coordinates.toArray(new Coordinate[1]);
		LineString tempLine = geometryFactory.createLineString(finalCoords);
		return tempLine;
	}


	


}
