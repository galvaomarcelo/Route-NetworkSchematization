package com.wayto.operator;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.geotools.geometry.jts.Geometries;
import org.geotools.geometry.jts.JTS;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.referencing.CRS;
import org.opengis.geometry.MismatchedDimensionException;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.CoordinateList;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryCollection;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineSegment;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.LinearRing;
import org.locationtech.jts.geom.MultiLineString;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.util.AffineTransformation;

public final class GeoConvertionsOperations {
	
	
	
	
	
	public static ArrayList<Point2D> JTSGeometryToJavaD2(Geometry geom) {
		Coordinate[] coordinates = null;
		

		LineString lineString = null;

		
		ArrayList<Point2D> normalized2DLineString = new ArrayList<Point2D>();
		
		switch ( Geometries.get(geom) ) {
        case LINESTRING:
        	lineString = (LineString)geom;
        	coordinates = lineString.getCoordinates();
        	
        	break;
        case MULTILINESTRING:
        	MultiLineString lines = (MultiLineString) geom;
        	lineString = (LineString)lines.getGeometryN(0);
        	coordinates = lineString.getCoordinates();       	
        	break;
        case POLYGON:
        	
        	Polygon polygon = (Polygon)geom;
        	lineString = polygon.getExteriorRing();
        	coordinates = lineString.getCoordinates();
        	break;
        case MULTIPOLYGON:
        	MultiPolygon multiPolygon = (MultiPolygon)geom;
        	lineString= ((Polygon)multiPolygon.getGeometryN(0)).getExteriorRing();
        	coordinates =lineString.getCoordinates();

        	break;           
        default:
        	break;

        }
		for(Coordinate c : coordinates){
			
			if(c!=null){
				Point2D.Double ptNormalized = new Point2D.Double(c.x , c.y );
			//System.out.println(ptNormalized + "" + c);
				normalized2DLineString.add( ptNormalized );
			}
		}
//		if( lineString.isClosed() )
//			normalized2DLineString.remove(normalized2DLineString.size() - 1);
		//double scale = 0.9/Math.max( (env.getMaxX() - env.getMinX())/schematicMap.getWidth() , (env.getMaxY() - env.getMinY())/schematicMap.getHeight());
		//System.out.println("Scale: " + scale);

		return normalized2DLineString;
		
		
	}
	
	
	public static ArrayList<Point2D> JTSGeometryToJavaD2Normalized(Geometry geom, Envelope env) {
		Coordinate[] coordinates = null;
		
		
		LineString lineString = null;
		double length = 0;
		
		ArrayList<Point2D> normalized2DLineString = new ArrayList<Point2D>();
		
		switch ( Geometries.get(geom) ) {
        case LINESTRING:
        	lineString = (LineString)geom;
        	
        	length = lineString.getLength();
        	coordinates = lineString.getCoordinates();
        	
        	break;
        case MULTILINESTRING:
        	MultiLineString lines = (MultiLineString) geom;
        	lineString = (LineString)lines.getGeometryN(0);
        	coordinates = lineString.getCoordinates();
        	length = lineString.getLength();
	               	
        	break;
        case POLYGON:
        	
        	Polygon polygon = (Polygon)geom;
        	lineString = polygon.getExteriorRing();
        	coordinates = lineString.getCoordinates();
        	length = lineString.getLength();
        	
        	break;
        case MULTIPOLYGON:
        	MultiPolygon multiPolygon = (MultiPolygon)geom;
        	lineString= ((Polygon)multiPolygon.getGeometryN(0)).getExteriorRing();
        	coordinates =lineString.getCoordinates();
        	length = lineString.getLength();
        	

        	break;           
        default:
        	break;

        }
		for(Coordinate c : coordinates){
			
			if(c!=null){
				normalized2DLineString.add( new Point2D.Double(c.x - env.getMinX(), env.getMaxY() - c.y ));
				
			}
		}
		//if( lineString.isClosed() )
			//normalized2DLineString.remove(normalized2DLineString.size() - 1);
		//double scale = 0.9/Math.max( (env.getMaxX() - env.getMinX())/schematicMap.getWidth() , (env.getMaxY() - env.getMinY())/schematicMap.getHeight());
		//System.out.println("Scale: " + scale);

		return normalized2DLineString;
		
		
	}
	
	public static Point Java2DPointToJTSGeometry (Point2D pt){
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		Coordinate coordTarget = new Coordinate();
		coordTarget.x = pt.getX();
		coordTarget.y = pt.getY();
		return geometryFactory.createPoint(coordTarget);
	}
	
	public static Point CreateJTSPoint (double x, double y){
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		Coordinate coordTarget = new Coordinate();
		coordTarget.x = x;
		coordTarget.y = y;
		return geometryFactory.createPoint(coordTarget);
	}
	
	public static Point CreateJTSPoint (Coordinate c){
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		return geometryFactory.createPoint(c);
	}
	
	
	public static Point2D PointJTSGeometryToJava2D (Point pt){
		return new Point2D.Double(pt.getX(), pt.getY() );

	}
	
	public static Polygon toPolygon(Coordinate[] coords){
		
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		LinearRing ring = geometryFactory.createLinearRing( coords );
		LinearRing holes[] = null; // use LinearRing[] to represent holes
		Polygon polygon = geometryFactory.createPolygon(ring, holes );
		return polygon;
	}
	
	public static LineString Java2DToJTSLineString(ArrayList<Point2D> xPath) {
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		LineString lineString = null;
		
		Coordinate[] coords  = new Coordinate[xPath.size()] ;
		for(int i = 0 ; i < xPath.size(); i++){

			coords[i] = new Coordinate(xPath.get(i).getX() ,
					xPath.get(i).getY()  );
					
		}
		lineString = geometryFactory.createLineString(coords);
		
		
		return lineString;
	}
	
	public static Geometry Java2DToJTSGeometry( ArrayList<Point2D> schematicLineString, Geometries geometryType, Envelope env) {

		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		LineString lineString = null;
		LinearRing linearRing = null;
		Polygon polygon = null;
		Geometry targetGeom = null;
		Coordinate[] polygonCoords = null;
		Coordinate[] coords  = new Coordinate[schematicLineString.size()] ;
		for(int i = 0 ; i < schematicLineString.size(); i++){

			coords[i] = new Coordinate(schematicLineString.get(i).getX() + env.getMinX(),
					-schematicLineString.get(i).getY() + env.getMaxY() );
					
		}
		
		switch ( geometryType ) {
        case LINESTRING:
        	lineString = geometryFactory.createLineString(coords);
        	targetGeom = lineString;
        	break;
        case MULTILINESTRING:
        	lineString = geometryFactory.createLineString(coords);
        	LineString[] arrayLineString = new LineString[1];
        	arrayLineString[0] = lineString;
        	MultiLineString multiLineString = geometryFactory.createMultiLineString(arrayLineString);
        	targetGeom = multiLineString;
        	break;
        case POLYGON:
        	polygonCoords = Arrays.copyOf(coords, coords.length + 1);
        	polygonCoords[polygonCoords.length - 1] = polygonCoords[0];        	
        	linearRing = geometryFactory.createLinearRing(polygonCoords);
        	polygon = geometryFactory.createPolygon(linearRing);     	       	
        	targetGeom = polygon;
        	break;
        case MULTIPOLYGON:
        	polygonCoords = Arrays.copyOf(coords, coords.length + 1);
        	polygonCoords[polygonCoords.length - 1] = polygonCoords[0];        	
        	linearRing = geometryFactory.createLinearRing(polygonCoords);
        	polygon = geometryFactory.createPolygon(linearRing);
        	Polygon[] polygons = new Polygon[1];
        	polygons[0] = polygon;
        	MultiPolygon multiPolygon = geometryFactory.createMultiPolygon(polygons);
        	targetGeom =  multiPolygon;
        	break;           
        default:
        	break;

        }
		
		return targetGeom;
	}
	
	
	public static Geometry Java2DToJTSGeometry( ArrayList<Point2D> schematicLineString, Geometries geometryType) {

		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		LineString lineString = null;
		LinearRing linearRing = null;
		Polygon polygon = null;
		Geometry targetGeom = null;
		CoordinateList polygonCoordLst = null;
		Coordinate[] coords  = new Coordinate[schematicLineString.size()] ;
		for(int i = 0 ; i < schematicLineString.size(); i++){

			coords[i] = new Coordinate(schematicLineString.get(i).getX(),
					schematicLineString.get(i).getY() );
					
		}
		
		switch ( geometryType ) {
        case LINESTRING:
        	lineString = geometryFactory.createLineString(coords);
        	targetGeom = lineString;
        	break;
        case MULTILINESTRING:
        	lineString = geometryFactory.createLineString(coords);
        	LineString[] arrayLineString = new LineString[1];
        	arrayLineString[0] = lineString;
        	MultiLineString multiLineString = geometryFactory.createMultiLineString(arrayLineString);
        	targetGeom = multiLineString;
        	break;
        case POLYGON:
        	polygonCoordLst = new CoordinateList( coords );
        	polygonCoordLst.closeRing();
        	linearRing = geometryFactory.createLinearRing( polygonCoordLst.toCoordinateArray() );
        	polygon = geometryFactory.createPolygon(linearRing);     	       	
        	targetGeom = polygon;
        	break;
        case MULTIPOLYGON:
        	polygonCoordLst = new CoordinateList( coords );
        	polygonCoordLst.closeRing();
        	linearRing = geometryFactory.createLinearRing( polygonCoordLst.toCoordinateArray() );
        	polygon = geometryFactory.createPolygon(linearRing);
        	Polygon[] polygons = new Polygon[1];
        	polygons[0] = polygon;
        	MultiPolygon multiPolygon = geometryFactory.createMultiPolygon(polygons);
        	targetGeom =  multiPolygon;
        	break;           
        default:
        	break;

        }
		
		return targetGeom;
	}
	

	public static Polygon lineStringToPolygon(LineString lineString) {
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		Coordinate[] coords  = lineString.getCoordinates();

		CoordinateList polygonCoordLst = new CoordinateList( coords );
		polygonCoordLst.closeRing();
		LinearRing linearRing = geometryFactory.createLinearRing( polygonCoordLst.toCoordinateArray() );
		Polygon polygon = geometryFactory.createPolygon(linearRing);     	       	

		return polygon;
	}
	
	public static Geometry convertProjection (Geometry geom, String sourceCRSName, String targetCRSName){
	
		  CoordinateReferenceSystem targetCRS;
	      CoordinateReferenceSystem sourceCRS;
	        
			MathTransform transform = null;
	        
			try {
				sourceCRS = CRS.decode(sourceCRSName);
				//System.out.println(CRS.getAxisOrder( sourceCRS ));
				
				targetCRS = CRS.decode(targetCRSName);
				//System.out.println(CRS.getAxisOrder( targetCRS ));
				
				transform = CRS.findMathTransform(sourceCRS, targetCRS, false);
		        
			} catch (NoSuchAuthorityCodeException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (FactoryException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} 
			
	        Geometry target = null;
			try {
				target = JTS.transform( geom, transform);
			} catch (MismatchedDimensionException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (TransformException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			return target;
	        
	
	}


	public static ArrayList<Point2D> normalizePointList(ArrayList<Point2D> path, Envelope env) {

		ArrayList<Point2D> normalizedList = new ArrayList<Point2D>();
		for(Point2D pt: path)
			normalizedList.add(new Point2D.Double(pt.getX()- env.getMinX(), pt.getY() - (env.getMaxY() ) ));
		return normalizedList;
	}
	
	public static ArrayList<Point2D> deNormalizePointList(ArrayList<Point2D> normalizedPointList, Envelope env) {

		ArrayList<Point2D> deNormalizedList = new ArrayList<Point2D>();
		for(Point2D pt: normalizedPointList)
			deNormalizedList.add(new Point2D.Double(pt.getX() + env.getMinX(), pt.getY() + env.getMaxY()  ));
			

		
		return deNormalizedList;
	}


	public static Point normalizeJTSPoint(Point pt, Envelope env) {
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		Coordinate coordTarget = new Coordinate();
		coordTarget.x = pt.getX() - env.getMinX();
		coordTarget.y = env.getMaxY() - pt.getY();
		return geometryFactory.createPoint(coordTarget);
		
	}
	
	public static Point normalizeJTSPoint2(Point pt, Envelope env) {
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		Coordinate coordTarget = new Coordinate();
		double longerAxe = Math.max(env.getHeight(), env.getWidth());
		coordTarget.x = (pt.getX() - env.getMinX())/longerAxe;
		coordTarget.y = (env.getMaxY() - pt.getY())/longerAxe;
		return geometryFactory.createPoint(coordTarget);
		
	}
	
	


	// first argument stands for schematized single lineString; second stands for decision point list and the third one stands for the proportion ranging from (0-1)
    public static LineString ScaleProcessing(LineString targetLine,ArrayList<Integer> indexPointList, double proportion, double raiseFactor){//proportion between 0-100
    
    	GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
    	//step 1: segment by decision point 
    	if (!indexPointList.contains(targetLine.getNumPoints()-1))
    		indexPointList.add(targetLine.getNumPoints()-1);
    	Coordinate[] tempCoords = targetLine.getCoordinates();
    	//split the polyline by the deciding points and apply the spline function directly
    	ArrayList<Point> pointList = new ArrayList<Point>();//store the angle keeping  points
    	ArrayList<LineString> splitedLineString = new ArrayList<LineString>();// store the segments
    	ArrayList<Double> lengthOfSegment = new ArrayList<Double>(); 
    	double shortestSegment = 0;
    	int shortestFlag = 0;
        for(int i=0;i<indexPointList.size();i++)
        {
        	Point tempPoint = targetLine.getPointN(indexPointList.get(i));
        	pointList.add(tempPoint);
        	//split
        	Coordinate[] subTempCoords;
        	if(i==0)
        	{
        		subTempCoords = Arrays.copyOfRange(tempCoords,0, indexPointList.get(i)+1);
        	}else
        	{
        		subTempCoords = Arrays.copyOfRange(tempCoords, indexPointList.get(i-1),indexPointList.get(i)+1);
        	}
        	
        	// simplify the point
        	List<Coordinate> coordinatesList = new ArrayList<Coordinate>();

        	for (int a=0; a<subTempCoords.length; a++) {  
        		coordinatesList.add(subTempCoords[a]);  
            }  
        	//angle filter
/*        	for(int p = 0;p< subTempCoords.length-2;p++)
        	{
        		Point startPoint = geometryFactory.createPoint(subTempCoords[p]);
        		Point midPoint = geometryFactory.createPoint(subTempCoords[p+1]);
        		Point endPoint = geometryFactory.createPoint(subTempCoords[p+2]);
        		boolean isLine = GeometricOperation.areAligned(startPoint,midPoint,endPoint);
        		if(isLine == true)
        		{
        			coordinatesList.remove(midPoint.getCoordinate());
        		}
        	}*/
        	Coordinate[] finalSubCoords =  coordinatesList.toArray(new Coordinate[1]);
        	LineString tempLine = geometryFactory.createLineString(finalSubCoords);
        	splitedLineString.add(tempLine);
        	//step 2: find out the shortest segment
        	// calculate distance of each segment
        	lengthOfSegment.add(tempLine.getLength());
        	if(i==0)
        	{
        		shortestSegment = tempLine.getLength();
        		shortestFlag = i;
        	}
        	if(shortestSegment > tempLine.getLength())
        	{
        		shortestSegment = tempLine.getLength();
        		shortestFlag = i;
        	}
        }
        //System.out.println("shortest number: "+shortestFlag);
        //lengthOfSegment contain all the length of segment include shortest segment  
        //step 3: compare to each segment(sum up distance) and apply the proportion (scale function for Geotool?)
       // proportion = 1-proportion;
        ArrayList<LineString> afterTranSegs = new ArrayList<LineString>(); 
        for(int i=0;i<lengthOfSegment.size();i++)
        {
        	
        	/**Zendong equation**/
        	//double newSegLen = (lengthOfSegment.get(i)-shortestSegment)*(1+proportion);
        	//calulate the new proportion
        	// double realScale = proportion - (shortestSegment*(proportion-1))/lengthOfSegment.get(i);
        	
        	
        	
        	 /***My esquation*/
        	// double realScale = 1- proportion + (proportion)*(shortestSegment/lengthOfSegment.get(i));
        	 //System.out.println("realScale number:" + realScale);
        	//step 4: affine transformation, the origin will be set at the begining point
        	
        	/**This equation reduces the lenght of long segments even more the effect is calculated by the raise of the lenght factor*/
        	double raiseOfLenghtFactor = raiseFactor; /* [0..1] the larger bigger is the effect*/
        	//double raiseOfLenghtFactor = 0.2; /* [0..1] the larger bigger is the effect*/
        	double lengthFactor =  Math.pow( (shortestSegment/lengthOfSegment.get(i)) , raiseOfLenghtFactor);
        	double newLength = shortestSegment + (lengthOfSegment.get(i) - shortestSegment)*(1- Math.pow( proportion, lengthFactor));
        	double realScale = newLength/lengthOfSegment.get(i);
        	
        	 AffineTransformation scale= new AffineTransformation(realScale,0.0,0.0, 0.0,realScale,0.0);
        	// System.out.println("Scale:" + scale.toString());
        	 LineString afterTran = (LineString) scale.transform(splitedLineString.get(i));
        	// System.out.println("afterTRAN: "+ afterTran.toString());
        	 afterTranSegs.add(afterTran);
        }
        //afterTranSegs contain the shortest but after affine transformation but it should keep the same coordinates
        // according to the begin and end point to merge the line
        //n != 0 means shortestFlag
        // final lineString finalLineString
        ArrayList<LineString> finalLineStrings = new ArrayList<LineString>();
        if(shortestFlag != 0)
        {
        	ArrayList<LineString> preFinalLineStrings = new ArrayList<LineString>();
            for (int n = shortestFlag; n > 0; n--)
            {
            	Point basePoint = null;
            	//get the begining point of shortest segment
            	if(n == shortestFlag)
            	{
            		basePoint = splitedLineString.get(n).getPointN(0);
            	}else
            	{
            		basePoint = preFinalLineStrings.get(preFinalLineStrings.size()-1).getPointN(0);
            	}
            	//Point endPointLastSeg =  splitedLineString.get(n-1).getPointN(splitedLineString.get(n-1).getNumPoints()-1);
            	Point endPointLastSeg = afterTranSegs.get(n-1).getEndPoint();
            	// translate 
            	AffineTransformation scale= new AffineTransformation(1,0.0,(basePoint.getX()-endPointLastSeg.getX()), 0.0,1,(basePoint.getY()-endPointLastSeg.getY()));
           	 	LineString afterTranlateSeg = (LineString) scale.transform(afterTranSegs.get(n-1));
           	 	preFinalLineStrings.add(afterTranlateSeg);
           	
            }
            //switch the order of finalLineString
            for(int r =preFinalLineStrings.size()-1;r>=0;r--)
            {
            	finalLineStrings.add(preFinalLineStrings.get(r));
            }
        }
        //add shortest one inside 
        finalLineStrings.add(splitedLineString.get(shortestFlag));
        //add rest segment which after shortest one except the shortest segment is the last one
        if(shortestFlag != (splitedLineString.size()-1))
        {
        	Point basePoint = null;
            //preFinalLineStrings=null;
            for(int p = shortestFlag; p < (splitedLineString.size()-1);p++)
            {
            	//get the end point of shortest segment 
            	if(p == shortestFlag)
            	{
            		basePoint = splitedLineString.get(p).getEndPoint();
            	}else
            	{
            		basePoint = finalLineStrings.get(finalLineStrings.size()-1).getEndPoint();
            	}
            	//get the first point of rest segment which after shortest one
            	Point firstPoint = afterTranSegs.get(p+1).getStartPoint();
            	//translate
            	AffineTransformation scale= new AffineTransformation(1,0.0,(basePoint.getX()-firstPoint.getX()), 0.0,1,(basePoint.getY()-firstPoint.getY()));
           	 	LineString afterTranlateSeg = (LineString) scale.transform(afterTranSegs.get(p+1));
           	 	finalLineStrings.add(afterTranlateSeg);
            }
        }
        LineString tempLine = myMerge(finalLineStrings,false);
        return tempLine;
    }
    public static LineString myMerge(ArrayList<LineString> sectionList, boolean closed) {

        GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
        Coordinate[] coordinates = null;
        ArrayList<Coordinate> coordlist = new ArrayList<Coordinate>();
        LineString mergedLineString;
        for (int i = 0 ; i < sectionList.size(); i++)
        {
           coordinates = sectionList.get(i).getCoordinates();
           for (int j = 0; j < coordinates.length - 1; j++)
           {
              coordlist.add(coordinates[j]);
           }
        }
        coordlist.add(sectionList.get(sectionList.size()-1).getEndPoint().getCoordinate());
        if(closed)
        	coordlist.add(sectionList.get(0).getCoordinateN(0));
        mergedLineString = geometryFactory.createLineString(coordlist.toArray(new Coordinate[coordlist.size()]));
        return mergedLineString;

    }


	public static Envelope getEnvelope(ArrayList<Point2D> transPointList) {
		LineString l = GeoConvertionsOperations.Java2DToJTSLineString(transPointList);
		return l.getEnvelopeInternal();
		
	}


	/*check https://github.com/maptimeBoston/cartographic-design for label positions*/
	public static Geometry getLabelBB(Point pt, double labelHeight, double labelWidght, int pos) {
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		double x1 = pt.getX();
		double y1 = pt.getY();
		double x2 = x1 + labelWidght;
		double y2 = y1 + labelHeight;
		double gap = labelHeight/3;
		
		
		switch (pos) {
		case 0:
			x1 = pt.getX() + gap/2 ;
			y1 = pt.getY() -  4*gap;
			x2 = x1 + labelWidght;
			y2 = y1 + labelHeight;
			
			break;
		case 1:
			x1 = pt.getX() + gap/2 ;
			y1 = pt.getY() +  1*gap;
			x2 = x1 + labelWidght;
			y2 = y1 + labelHeight;
			
			break;
		case 2:
			x1 = pt.getX() - gap/2;
			y1 = pt.getY()  -  4*gap;
			x2 = x1 - labelWidght;
			y2 = y1 + labelHeight;
			
			break;
		case 3:
			x1 = pt.getX() - gap/2 ;
			y1 = pt.getY()  +  1*gap;
			x2 = x1 - labelWidght;
			y2 = y1 + labelHeight;
			
			break;
		case 4:
			x1 = pt.getX() - labelWidght/2 ;
			y1 = pt.getY() -  4*gap;
			x2 = x1 + labelWidght;
			y2 = y1 + labelHeight;

			break;
		case 5:
			x1 = pt.getX() - labelWidght/2 ;
			y1 = pt.getY() +  1*gap;
			x2 = x1 + labelWidght;
			y2 = y1 + labelHeight;
			break;
		case 6:
			x1 = pt.getX() + 1.5*gap ;
			y1 = pt.getY() -  gap;
			x2 = x1 + labelWidght;
			y2 = y1 + labelHeight;
			break;
		case 7:
			x1 = pt.getX() - 1.5*gap ;
			y1 = pt.getY() -  gap;
			x2 = x1 - labelWidght;
			y2 = y1 + labelHeight;
			break;
			

		default:
			x1 = pt.getX() + labelHeight/2 ;
			y1 = pt.getY() -  labelHeight;
			x2 = x1 + labelWidght;
			y2 = y1 + labelHeight;
			break;
		}
			
		Coordinate[] coords  = new Coordinate[2] ;
		coords[0] = new Coordinate(x1, y1);
		coords[1] = new Coordinate(x2, y2);

        LineString lineString = geometryFactory.createLineString(coords);
		return lineString.getEnvelope();
	}
	
	
	public static ArrayList<LineString> splitLineStringIntoParts(LineString ls, double length){
	    // result list for linestrings
	    ArrayList<LineString> resultList = new ArrayList();
	    // list for linesegments from input linestring
	    ArrayList<LineSegment> lineSegmentList = new ArrayList();
	    // create LineSegment objects from input linestring and add them to list
	    for(int i = 1; i < ls.getCoordinates().length; i++){
	        lineSegmentList.add(new LineSegment(ls.getCoordinates()[i-1], ls.getCoordinates()[i]));
	    }
	    LineString currentLineString = null;
	    double neededLength = length;
	    for(LineSegment s : lineSegmentList){
	        while(s.getLength() > 0){
	            // case: current segment is small enough to be added to the linestring
	            if(s.getLength() <= neededLength){
	                // create linestring if it does not exist 
	                if(currentLineString == null){
	                    currentLineString = new GeometryFactory().createLineString(new Coordinate[]{new Coordinate(s.p0), new Coordinate(s.p1)});
	                // just add the new endpoint otherwise
	                } else {
	                    Coordinate[] coords = new Coordinate[currentLineString.getCoordinates().length + 1];
	                    // copy old coordinates
	                    System.arraycopy(currentLineString.getCoordinates(), 0, coords, 0, currentLineString.getCoordinates().length);
	                    // add new coordinate at the end
	                    coords[coords.length-1] = new Coordinate(s.p1);
	                    // create new linestring
	                    currentLineString = new GeometryFactory().createLineString(coords);
	                }
	                neededLength -= s.getLength();
	                s.setCoordinates(s.p1, s.p1);
	                // add linestring to result list if needed length is 0
	                if(neededLength == 0){
	                    resultList.add(currentLineString);
	                    currentLineString = null;
	                    neededLength = length;
	                }
	            // current segment needs to be cut and added to the linestring
	            } else {
	                // get coordinate at desired distance (endpoint of linestring)
	                Coordinate endPoint = s.pointAlong(neededLength/s.getLength());
	                // create linestring if it does not exist 
	                if(currentLineString == null){
	                    currentLineString = new GeometryFactory().createLineString(new Coordinate[]{new Coordinate(s.p0), endPoint});
	                // just add the new endpoint otherwise
	                } else {
	                    // add new coordinate to linestring
	                    Coordinate[] coords = new Coordinate[currentLineString.getCoordinates().length + 1];
	                    // copy old coordinates
	                    System.arraycopy(currentLineString.getCoordinates(), 0, coords, 0, currentLineString.getCoordinates().length);
	                    // add new coordinate at the end
	                    coords[coords.length-1] = endPoint;
	                    currentLineString = new GeometryFactory().createLineString(coords);
	                }
	                // add linestring to result list
	                resultList.add(currentLineString);
	                // reset needed length
	                neededLength = length;
	                // reset current linestring
	                currentLineString = null;
	                // adjust segment (calculated endpoint is the new startpoint)
	                s.setCoordinates(endPoint, s.p1);
	            }
	        }
	    }
	    // add last linestring if there is a rest
	    if(neededLength < length){
	        resultList.add(currentLineString);
	    }
	    return resultList;
	}
	
//	public static LineString myMerge(ArrayList<LineString> sectionList, boolean closed) {
//		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
//		Coordinate[] coordinates = null;
//		ArrayList<Coordinate> coordlist = new ArrayList<Coordinate>();
//		LineString mergedLineString;
//		
//		for (int i = 0 ; i < sectionList.size(); i++){
//			coordinates = sectionList.get(i).getCoordinates();
//			for (int j = 0; j < coordinates.length - 1; j++){
//				coordlist.add(coordinates[j]);
//			}
//			
//		}
//		if(closed)
//			coordlist.add(sectionList.get(0).getCoordinateN(0));
//
//		mergedLineString = geometryFactory.createLineString(coordlist.toArray(new Coordinate[coordlist.size()]));
//		
//		return mergedLineString;
//	}
	


}
