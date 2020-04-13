package com.wayto.operator;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;





 






/* Classe utilizada realizar opera��es geometricas necessarias*/


public final class GeometricOperation {

	

	
	public GeometricOperation() {
		super();

	}

	public static double getAngleBetweenVectors(double origemX, double origemY, 
			double v1x,	double v1y, 
			double v2x, double v2y) {

		double ax = v1x - origemX;
		double ay = v1y - origemY;
		double bx = v2x - origemX;
		double by = v2y - origemY;

		double angle = Math.acos( ((ax * bx) + (ay * by)) / ( Math.sqrt(ax*ax + ay*ay) * Math.sqrt(bx*bx + by*by)) );

//		return( Math.atan2(v2y - origemY, v2x - origemX) - Math.atan2(v1y - origemY, v1x - origemX) );
		return (angle);		
	}	
	
	/*retorna o angulo entre dois pontos relativo ao eixo X */
	public static double getAngleBetweenPointsRelativeToAxisX(double origemX, double origemY, double v1x,
			double v1y) {
		double angle;	
		
		angle =  - Math.atan2(v1y - origemY, v1x - origemX);
		
		if (angle < 0)
			angle = 2*Math.PI + angle;
			
		return( angle );
				
	}
	
	public static Point2D.Double getNewSchematicPosition( double pivoX, double pivoY, double pointX , double pointY , double rotation){
		
		double angle = getAngleBetweenPointsRelativeToAxisX(pivoX, pivoY, pointX, pointY);
		angle = getBestAngle(angle , rotation);
		double dist = Point2D.distance(pivoX, pivoY, pointX, pointY);
		Point2D.Double pt = new Point2D.Double(pivoX + dist*Math.cos(angle) ,(pivoY - dist*Math.sin(angle))); 
		
		return pt;
		
	}
	/*PAREI por AQUI*/
	public static Point2D.Double getNewSchematicPosition2( Point2D previousXPt, Point2D previousPt ,  Point2D originalPt, double rotation){
		
		
		double angle = getAngleBetweenPointsRelativeToAxisX(previousXPt.getX(), previousXPt.getY(), originalPt.getX(), originalPt.getY());
		double octAngle = getBestAngle(angle , rotation);
		
		double angle2 = getAngleBetweenPointsRelativeToAxisX(previousPt.getX(), previousPt.getY(), originalPt.getX(), originalPt.getY());
		double octAngle2 = getBestAngle(angle2 , rotation);
		
		
		
			
		
		
		/*Buscando o melhor angulo e distancia para simples esquematizacao. (angle - octAngle): Used to find the best distance*/
		double dist = Math.cos(angle - octAngle2)*previousXPt.distance(originalPt);
		if(dist < 0)
			dist = previousPt.distance(originalPt);
		
		Point2D.Double pt = new Point2D.Double(previousXPt.getX() + dist*Math.cos(octAngle2) ,(previousXPt.getY() - dist*Math.sin(octAngle2))); 
		
		double difAngle = octAngle2 -angle2;
		if(Math.abs(difAngle) > 0.4)
			difAngle  += 2*Math.PI;
		
		//System.out.println("Dist: " + dist + " True angle: " + Math.toDegrees(angle2) + " XAngle: " + Math.toDegrees(octAngle2) +  " Dif Angle: " + difAngle );

		
		double octAngle3;
		Point2D.Double pt2;
		
		if(difAngle > 0.25){
			octAngle3 = octAngle2 - Math.PI/4;
			dist = Math.cos(angle - octAngle3)*previousXPt.distance(originalPt);
			if(dist < 0)
				dist = previousPt.distance(originalPt);
			pt2 = new Point2D.Double(previousXPt.getX() + dist*Math.cos(octAngle3) ,(previousXPt.getY() - dist*Math.sin(octAngle3))); 
			if(pt2.distance(originalPt) < (pt.distance(originalPt)*1.5))
				pt = pt2;
				
		}
		else if(difAngle < -0.25)
		{
			octAngle3 = octAngle2 + Math.PI/4;
			dist = Math.cos(angle - octAngle3)*previousXPt.distance(originalPt);
			if(dist < 0)
				dist = previousPt.distance(originalPt);
			pt2 = new Point2D.Double(previousXPt.getX() + dist*Math.cos(octAngle3) ,(previousXPt.getY() - dist*Math.sin(octAngle3))); 
			if(pt2.distance(originalPt) < (pt.distance(originalPt)*1.5))
				pt = pt2;
				
		}
			
			
			
		return pt;
		
	}
	
	/*converte uma lista de pontos2d em ponto com coordenadas polares relativas */
	public static PointsPolar toPolar(ArrayList<Point2D> schematicPoints) {
		PointsPolar polarPoints = new PointsPolar( schematicPoints.get(0));
		for(int i = 1; i < schematicPoints.size(); i++ ){
			polarPoints.add( toPolar(schematicPoints.get(i-1) ,schematicPoints.get(i) ));
		}
		return polarPoints;
	}

	/* encontra a coordenada polar de point2d2 relativo ao point2d */
	public static RelativPolarPoint toPolar(Point2D originPoint, Point2D referencePoint) {
		
		double r = originPoint.distance( referencePoint );
		double theta = getAngleBetweenPointsRelativeToAxisX(originPoint.getX(), originPoint.getY(),
				referencePoint.getX(), referencePoint.getY());
		RelativPolarPoint p = new RelativPolarPoint( r , theta );
		
		
		
		return p;
	}
	
	/* converte Lista de pontos con coordenadas polares relativa a uma lista de coordenadas cartezianas*/
	public static ArrayList<Point2D> toCartezian( PointsPolar polarPoints) {
		
		ArrayList<Point2D> cartezainPoints = new ArrayList<Point2D>();
		cartezainPoints.add( polarPoints.getOrigem() );
		for(int i = 0 ; i < polarPoints.getPoints().size() ; i++ ){
			cartezainPoints.add( toCartezian( cartezainPoints.get(i), polarPoints.getPoints().get(i) ));
		}

		return cartezainPoints;
	}

	/* encontra a coordenada carteziana a partir de uma coordenada polar relativa*/
	public static Point2D toCartezian(Point2D point2D,
			RelativPolarPoint polarPoint) {
		double x = Math.cos( polarPoint.getTheta() )*polarPoint.getR() + point2D.getX();
		double y = -1*Math.sin( polarPoint.getTheta() )*polarPoint.getR() + point2D.getY();

		
		return new Point2D.Double(x , y);
	}
	
	/* encontra a coordenada carteziana a partir de uma coordenada polar relativa*/
	public static Point2D toCartezian(Point2D point2D,
			double theta, double r) {
		double x = Math.cos( theta )*r + point2D.getX();
		double y = -1*Math.sin( theta )*r + point2D.getY();
		
		return new Point2D.Double(x , y);
	}
	
	
	/* getBest angle2 mostrou melhor resultado*/
	private static double getBestAngle(double angle, double rotation) {

		double auxMinDif = 2*Math.PI;
		double finalAngle = 0;
		for(int i = 0; i <= 8; i++){
			/* Solu��o gambiarra: mod (2PI + 1�) para 360mod <> 0*/
			double dif = Math.abs( angle - (i*Math.PI/4 + rotation)%(2*Math.PI + Math.PI/180) );
			
			if(dif < auxMinDif){
				auxMinDif = dif;
				finalAngle = (i*Math.PI/4 + rotation)%(2*Math.PI);
			}
				
		}
		return finalAngle;
	}
	
	private static double getBestAngle2(double angleX, double angle, double rotation) {

		double auxMinDif = 2*Math.PI;
		double finalAngle = 0;
		for(int i = 0; i <= 8; i++){
			/* Solu��o gambiarra: mod (2PI + 1�) para 360mod <> 0*/
			double dif = Math.abs( angle - (i*Math.PI/4 + rotation)%(2*Math.PI + Math.PI/180) );
			
			if(dif < auxMinDif){
				auxMinDif = dif;
				finalAngle = (i*Math.PI/4 + rotation)%(2*Math.PI);
			}
				
		}
		return finalAngle;
	}
	
	public static double length(ArrayList<Point2D> string, int start, int end){
		double length = 0;
		for (int i = start; i < end; i ++)
			length += string.get(i).distance(string.get(i+1));	
		return length;
	}
	
	
	public static double length(ArrayList<Point2D> string){
		double length = 0;
		for (int i = 1; i < string.size(); i ++)
			length += string.get(i).distance(string.get(i-1));	
		return length;
	}
	
	
	private static double getBestAngle2(double angle) {

		double finalAngle = 0;
		
		if (angle <= Math.PI/8 )
			finalAngle = 0;
		else if (angle <= 3* Math.PI/8)
			finalAngle = Math.PI/4;
		else if (angle <= 5* Math.PI/8)
			finalAngle = Math.PI/2;
		else if (angle <= 7* Math.PI/8)
			finalAngle = 3*Math.PI/4;
		else if (angle <= 9* Math.PI/8)
			finalAngle = Math.PI;
		else if (angle <= 11* Math.PI/8)
			finalAngle = 5*Math.PI/4;
		else if (angle <= 13* Math.PI/8)
			finalAngle = 3*Math.PI/2;
		else if (angle <= 15* Math.PI/8)
			finalAngle = 7*Math.PI/4;
		else finalAngle = 0;
		
		
	
		

		return finalAngle;
	}

	public static ArrayList<Point2D> toSimpleOctalinear(ArrayList<Point2D> pixelPolygon) {
		
		ArrayList<Point2D> pixelPolygonFinal = new ArrayList<Point2D>();
		Point2D.Double ptaux = new Point2D.Double();
		
	

		
		pixelPolygonFinal.add(new Point2D.Double(pixelPolygon.get(0).getX(), pixelPolygon.get(0).getY()));
		for (int i = 1; i < pixelPolygon.size(); i ++){

			ptaux = getNewSchematicPosition(pixelPolygonFinal.get(i-1).getX(), pixelPolygonFinal.get(i-1).getY(),
					pixelPolygon.get(i).getX(), pixelPolygon.get(i).getY() , 0);	
			pixelPolygonFinal.add(ptaux);
		}
		
		
		return pixelPolygonFinal;
	}
	
	/*PAREI por AQUI*/
	public static ArrayList<Point2D> toSimpleOctalinear2(ArrayList<Point2D> pixelPolygon) {
		
		ArrayList<Point2D> pixelPolygonFinal = new ArrayList<Point2D>();
		Point2D.Double ptaux = new Point2D.Double();
		
	

		
		pixelPolygonFinal.add(new Point2D.Double(pixelPolygon.get(0).getX(), pixelPolygon.get(0).getY()));
		for (int i = 1; i < pixelPolygon.size(); i ++){

			ptaux = getNewSchematicPosition2(pixelPolygonFinal.get(i-1), pixelPolygon.get(i -1) ,
					pixelPolygon.get(i) , 0);	
			pixelPolygonFinal.add(ptaux);
		}
		
		
		return pixelPolygonFinal;
	}

	public static boolean areConsistent(ArrayList<Point2D> originalLineString,
			ArrayList<Point2D> schematicLineString) {
		
		boolean isConsistent = true;
		
		for ( int i = 0;  i < originalLineString.size(); i++){
			ArrayList<OrthoAngle> orthogalAnglesOrginal = new ArrayList<OrthoAngle>();
			ArrayList<OrthoAngle> orthogalAnglesSchematic = new ArrayList<OrthoAngle>();
			//double minDist = schematicLineString.get(i).distance(schematicLineString.get(i+1));
			for( int j = 0;  j < schematicLineString.size(); j++){
				//double minDist = schematicLineString.get(i).distance(schematicLineString.get(i+1));
				if( j != i ){
					double theta = getAngleBetweenPointsRelativeToAxisX(originalLineString.get(i).getX(), originalLineString.get(i).getY()
							, originalLineString.get(j).getX(), originalLineString.get(j).getY());
					
							orthogalAnglesOrginal.add(new OrthoAngle(j,theta));
							
							theta = getAngleBetweenPointsRelativeToAxisX(schematicLineString.get(i).getX(), schematicLineString.get(i).getY()
									, schematicLineString.get(j).getX(), schematicLineString.get(j).getY());
							
							orthogalAnglesSchematic.add(new OrthoAngle(j,theta));
					
				}
				
				orthogalAnglesOrginal.sort(OrthoAngle.AngleComparator);
				
				orthogalAnglesSchematic.sort(OrthoAngle.AngleComparator);
				
				isConsistent = compareOrder(orthogalAnglesOrginal, orthogalAnglesSchematic);
				
			}
			
			
		}
		

		return true;
	}

	private static boolean compareOrder(ArrayList<OrthoAngle> orthogalAnglesOrginal,
			ArrayList<OrthoAngle> orthogalAnglesSchematic) {
		
		for(int i = 0; i < orthogalAnglesOrginal.size(); i ++){
			System.out.println(orthogalAnglesOrginal.get(i).toString());
			System.out.println(orthogalAnglesSchematic.get(i).toString());
			
		}

		
		return false;
	}

	public static boolean validadeOrthogonalOrder(ArrayList<Point2D> originalLineString,
			ArrayList<Point2D> schematicLineString) {


		boolean isConsistent = true;

		for ( int i = 0;  i < originalLineString.size(); i++){

			for( int j = 0;  j < schematicLineString.size(); j++){
				
				if( j != i ){
					
					isConsistent =	isSameOrthogonalOrder( originalLineString.get(i) , originalLineString.get(j), 
							schematicLineString.get(i),schematicLineString.get(j) );

					if(!isConsistent){
						System.out.println("PT" + i + " Pt" + j);
						System.out.println( originalLineString.get(i) + " " +  originalLineString.get(j) );
						System.out.println( schematicLineString.get(i) + " " +  schematicLineString.get(j) );
						//break;
					}	
				}

				

			}
			//if(!isConsistent)
				//break;


		}

		System.out.println(isConsistent);
		return isConsistent;
		
		
	}

	private static boolean isSameOrthogonalOrder(Point2D pt1a, Point2D pt2a, Point2D pt1b, Point2D pt2b) {
		
		boolean isSameOrthogonalOrder = true;
		
		if( compareDoubles(pt1a.getX(), pt2a.getX()) <= 0)
			if(compareDoubles(pt1b.getX(), pt2b.getX()) > 0 )
				isSameOrthogonalOrder = false;
		
		if( compareDoubles(pt1a.getX(), pt2a.getX()) >= 0)
			if(compareDoubles(pt1b.getX(), pt2b.getX()) < 0 )
				isSameOrthogonalOrder = false;
	
		if( compareDoubles(pt1a.getY(), pt2a.getY()) <= 0)
			if(compareDoubles(pt1b.getY(), pt2b.getY()) > 0 )
				isSameOrthogonalOrder = false;

		if( compareDoubles(pt1a.getY(), pt2a.getY()) >= 0)
			if(compareDoubles(pt1b.getY(), pt2b.getY()) < 0 )
				isSameOrthogonalOrder = false;
		
		
		return isSameOrthogonalOrder;
		
		
	}
	
	
	
	private static int compareDoubles(double value1, double value2){
		double dif = value1 - value2;
		
		if(Math.abs(dif) < 1E-10)
			return 0;
		else if (dif > 0)
			return 1;
		else 
			return -1;
		
	 
		
		
	}

	public static int sectorOf(double theta) {
		int sector = 0;
		double d = Math.PI/8 /*22,5 degrees */;
		if (theta <= d )
			sector = 0;
		else if (theta <= 3* d)
			sector = 1;
		else if (theta <= 5* d)
			sector = 2;
		else if (theta <= 7* d)
			sector = 3;
		else if (theta <= 9* d)
			sector = 4;
		else if (theta <= 11* d)
			sector = 5;
		else if (theta <= 13* d)
			sector = 6;
		else if (theta <= 15*d)
			sector = 7;
		else sector = 0;
		
		
	
		

		return sector;
	}
	public static int sectorOfOrthogonalPreferenceOld(double theta, double preference) {
		
		double angDegree = Math.toDegrees(theta);
		int sector = 0;
		if (angDegree <= 25 )
			sector = 0;
		else if (angDegree <= 65)
			sector = 1;
		else if (angDegree <= 115)
			sector = 2;
		else if (angDegree <= 155)
			sector = 3;
		else if (angDegree <= 205)
			sector = 4;
		else if (angDegree <= 245)
			sector = 5;
		else if (angDegree <= 295)
			sector = 6;
		else if (angDegree <= 335)
			sector = 7;
		else sector = 0;
		return sector;
	}
	
	
	public static int sectorOfOrthogonalPreference(double theta, double preference) {
		
		double range = preference*22.5;
		double angDegree = Math.toDegrees(theta);
		int sector = 0;
		if (angDegree <= 22.5 + range )
			sector = 0;
		else if (angDegree <= 67.5 - range)
			sector = 1;
		else if (angDegree <= 112.5 + range)
			sector = 2;
		else if (angDegree <= 157.5 - range)
			sector = 3;
		else if (angDegree <= 202.5 + range)
			sector = 4;
		else if (angDegree <= 247.5 - range)
			sector = 5;
		else if (angDegree <= 292.5 + range)
			sector = 6;
		else if (angDegree <= 337.5 - range)
			sector = 7;
		else sector = 0;
		return sector;
	}
	
	/* Distancia proporcial a distancia em ralacao ao tamanho total*/
	public static  ArrayList<Point2D> fillLine ( ArrayList<Point2D> newSchematicPoints , ArrayList<Point2D> originalPoints ,int size , int start){
		ArrayList<Point2D> points = new ArrayList<Point2D>();
		points.addAll( newSchematicPoints );
		
		double deltaX = (points.get( start + 1 ).getX() - points.get( start ).getX() ) ;
		double deltaY =  (points.get( start + 1 ).getY() - points.get( start ).getY() ) ;
		
		double totalLength = GeometricOperation.length(originalPoints, start, start + size);

		for (int i = start + 1 ; i < size + start; i ++ ){

			double partLength = GeometricOperation.length(originalPoints, start, i);
			double proportion = partLength/totalLength;

			Point2D newPt = new Point2D.Double(points.get( start ).getX() + deltaX*proportion ,
					points.get( start ).getY() + deltaY*proportion);
			points.add(i,  newPt);
			
			
		}
		
		return points;
	}

	
	public static double lengthOf(ArrayList<Point2D> string){
		double stringLength = 0;
		for(int i = 1; i < string.size(); i ++){
			stringLength = stringLength + string.get(i).distance(string.get(i -1));
		}
		return stringLength;
	}
	
	public static double diagonalOf(ArrayList<Point2D> lineString){
		double  maxX, minX, maxY, minY;
		maxX = lineString.get(0).getX();
		minX = lineString.get(0).getX();
		maxY = lineString.get(0).getY();
		minY = lineString.get(0).getY();
		for(int i = 0; i < lineString.size(); i ++){
			maxX = (lineString.get(i).getX() > maxX) ? lineString.get(i).getX() : maxX;
			minX = (lineString.get(i).getX() < minX) ? lineString.get(i).getX() : minX;
			maxY = (lineString.get(i).getY() > maxY) ? lineString.get(i).getY() : maxY;
			minY = (lineString.get(i).getY() < minY) ? lineString.get(i).getY() : minY;
		}
		return Math.sqrt( Math.pow(maxY - minY, 2)  +  Math.pow(maxX - minX, 2));
	}


	


	

	  public static Point2D.Double getClosestPointOnSegment(double sx1, double sy1, double sx2, double sy2, double px, double py)
	  {
	    double xDelta = sx2 - sx1;
	    double yDelta = sy2 - sy1;

	    if ((xDelta == 0) && (yDelta == 0))
	    {
	    	/*Gambiarra:Corrigir*/
	    	/***Por favor- Conferir se não há edges com tamanho 0. Ou seja, que comecem e terminem no mesmo lugar****/
	     // throw new IllegalArgumentException("Segment start equals segment end");
	    	return new Point2D.Double(sx1, sy1);
	    }

	    double u = ((px - sx1) * xDelta + (py - sy1) * yDelta) / (xDelta * xDelta + yDelta * yDelta);

	    final Point2D.Double closestPoint;
	    if (u < 0)
	    {
	      closestPoint = new Point2D.Double(sx1, sy1);
	     
	    }
	    else if (u > 1)
	    {
	      closestPoint = new Point2D.Double(sx2, sy2);
	      
	    }
	    else
	    {
	      closestPoint = new Point2D.Double( sx1 + u * xDelta,  sy1 + u * yDelta);
	    }

	    return closestPoint;
	  }

	  
	  public static double getClosestPointOnVector(double sx1, double sy1, double xDelta, double yDelta, double px, double py)
	  {
		  double dist;

	    if ((xDelta == 0) && (yDelta == 0))
	    {
	    	/*Gambiarra:Corrigir*/
	      throw new IllegalArgumentException("Segment start equals segment end");
	    	//return new Point2D.Double(sx1, sy1);
	    }

	    double u = ((px - sx1) * xDelta + (py - sy1) * yDelta) / (xDelta * xDelta + yDelta * yDelta);

	  
	    Point2D.Double  closestPoint = new Point2D.Double( sx1 + u * xDelta,  sy1 + u * yDelta);
	    dist = closestPoint.distance(sx1, sy1);

	    return dist;
	  }
	  
	public static void addDummyVertexAfter(ArrayList<Point2D> simplifiedLineList, int i, double proportion) {


		double dist = proportion*simplifiedLineList.get(i).distance(simplifiedLineList.get(i+1));
		
		double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(simplifiedLineList.get(i).getX(), simplifiedLineList.get(i).getY(), simplifiedLineList.get(i+1).getX(), simplifiedLineList.get(i+1).getY());
//		System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//		System.out.println("O Point" + simplifiedLineList.get(i));
//		System.out.println("O Point" + simplifiedLineList.get(i+1));
//		System.out.println("Dist:" + dist);
		
		double newX = simplifiedLineList.get(i).getX() + dist*Math.cos(angle);
		double newY = simplifiedLineList.get(i).getY() - dist*Math.sin(angle);
		System.out.println("N Point" + new Point2D.Double(newX, newY));
		simplifiedLineList.add(i+1, new Point2D.Double(newX, newY));
		
	}

	public static int turnDirectionSectorOfOrthogonalPreference(double theta, double theta2, double preference) {
		
		double range = preference*22.5;

		
		int sector = 0;
		double angDegree =  theta - theta2;
		if (angDegree < 0)
			angDegree = angDegree + 2*Math.PI;
		angDegree = Math.toDegrees(angDegree);

		if (angDegree <= 22.5 + range )
			sector = 0;
		else if (angDegree <= 67.5 - range)
			sector = 1;
		else if (angDegree <= 112.5 + range)
			sector = 2;
		else if (angDegree <= 157.5 - range)
			sector = 3;
		else if (angDegree <= 202.5 + range)
			sector = 4;
		else if (angDegree <= 247.5 - range)
			sector = 5;
		else if (angDegree <= 292.5 + range)
			sector = 6;
		else if (angDegree <= 337.5 - range)
			sector = 7;
		else sector = 0;
		return sector;
	}
	
	public static int turnDirectionOf(double theta, double theta2, int type) {
		/* 0 => 0-straight , 1=>315-veer right, 2=> 270-right, 3=>225sharp right, 
				 * 4=>180backturn, 5=>135-sharp left, 6=> 90left, 7=>45veer left */
		
		int dir = 0;
		double diff =  theta - theta2;
		if (diff < 0)
			diff = diff + 2*Math.PI;
		double d;
		switch (type) {
		case 0: /*traditional*/
			 d = Math.PI/8 /*22,5 degrees */;
			if (diff <= d )
				dir = 0;
			else if (diff <= 3* d)
				dir = 1;
			else if (diff <= 5* d)
				dir = 2;
			else if (diff <= 7* d)
				dir = 3;
			else if (diff <= 9* d)
				dir = 4;
			else if (diff <= 11* d)
				dir = 5;
			else if (diff <= 13* d)
				dir = 6;
			else if (diff <= 15*d)
				dir = 7;
			else dir = 0;
			
			break;
		case 1: /*klippel*/
			d = Math.PI/16 /*11,25 degrees */;
			if (diff <= d )
				dir = 0;
			else if (diff <= 5* d)
				dir = 1;
			else if (diff <= 9* d)
				dir = 2;
			else if (diff <= 15* d)
				dir = 3;
			else if (diff <= 17* d)
				dir = 4;
			else if (diff <= 23* d)
				dir = 5;
			else if (diff <= 27* d)
				dir = 6;
			else if (diff <= 31*d)
				dir = 7;
			else dir = 0;
			
			break;
		case 2: /*right angles*/
			d = Math.PI/16 /*11,25 degrees */;
			if (diff <= 3*d )
				dir = 0;
			else if (diff <= 5* d)
				dir = 1;
			else if (diff <= 11* d)
				dir = 2;
			else if (diff <= 13* d)
				dir = 3;
			else if (diff <= 19* d)
				dir = 4;
			else if (diff <= 21* d)
				dir = 5;
			else if (diff <= 27* d)
				dir = 6;
			else if (diff <= 29*d)
				dir = 7;
			else dir = 0;
			
			break;

		default:
			break;
		}
		
		
		return dir;
	}
	
	public static boolean areAligned(Point p1,Point p2,Point p3){  
		boolean flag=false;  
		float k1=0.0f;//slope
		float k2=0.0f;//slope

		//  
		if((p1.getX()==p2.getX()&&p1.getY()==p2.getY())  
				||(p1.getX()==p3.getX()&&p1.getY()==p3.getY())  
				||(p2.getX()==p3.getX()&&p2.getY()==p3.getY())){  
			flag=true;  
			return flag;  
		}  

		//
		if((p1.getY()==p2.getY())&&(p1.getY()==p3.getY())  
				&&(p1.getX()!=p2.getX())&&(p1.getX()!=p3.getX())){  
			flag=true;  
			return flag;  
		}  
		// 
		if((p1.getX()==p2.getX())&&(p1.getX()==p3.getX())  
				&&(p1.getY()!=p2.getY())&&(p1.getY()!=p3.getY())){  
			flag=true;  
			return flag;  
		}else{//
			k1=(float) ((p3.getY()-p2.getY())/(p3.getX()-p2.getX()));  
			k2=(float) ((p1.getY()-p2.getY())/(p1.getX()-p2.getX()));  
			if(k1==k2){  
				flag=true;  
				return flag;  
			}  
		}  

		//  
		if(p1.getX()==p2.getX()&&p1.getX()==p3.getX()  
				&&p1.getY()==p2.getY()&&p1.getY()==p3.getY()){  
			flag=true;  
		}  
		//   
		//
		float n=(float) (p3.getX()-p2.getX());  
		float m=(float) (p1.getX()-p2.getX());  
		if(n!=0&&m!=0){
			k1=(float) ((p3.getY()-p2.getY())/n);  
			k2=(float) ((p1.getY()-p2.getY())/m);  
			if(k1==k2){  
				flag=true;  
			}  
		}  
		return flag;  
	}



	public static boolean intersects(double e1x1, double e1y1, double e1x2, double e1y2, double e2x1, double e2y1,
			double e2x2, double e2y2) {

		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();

		Coordinate[] coordsE1  = new Coordinate[] {new Coordinate(e1x1, e1y1), new Coordinate(e1x2, e1y2)};
		LineString segmentE1 = geometryFactory.createLineString(coordsE1);
		
		Coordinate[] coordsE2  = new Coordinate[] {new Coordinate(e2x1, e2y1), new Coordinate(e2x2, e2y2)};
		LineString segmentE2 = geometryFactory.createLineString(coordsE2);
		
		return segmentE1.intersects(segmentE2);
		
		
	}  

//	public double  maxExtend(ArrayList<Point2D> points) {
//		double maxX, minX, maxY, minY, diagonal;
//		maxX = points.get(0).getX();
//		minX = points.get(0).getX();
//		maxY = points.get(0).getY();
//		minY = points.get(0).getY();
//		
//		for (int i = 1; i < points.size(); i++) {
//			maxX = (points.get(i).getX() > maxX) ? points.get(i).getX() : maxX;
//			minX = (points.get(i).getX() < minX) ? points.get(i).getX() : minX;
//			maxY = (points.get(i).getY() > maxY) ? points.get(i).getY() : maxY;
//			minY = (points.get(i).getY() < minY) ? points.get(i).getY() : minY;
//
//		}
//		diagonal = (maxX - minX)*(maxX - minX);
//		return 0;
//	}
	  
	  
	
	

	
}
