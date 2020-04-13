package com.wayto.operator;

import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.geotools.referencing.operation.transform.AffineTransform2D;
import org.geotools.geometry.jts.Geometries;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.util.AffineTransformation;
import com.wayto.model.Path;
import com.wayto.model.StreetNode;
import com.wayto.model.topo.ControlPoint;
import com.wayto.model.topo.NodeTopoRelation;
import com.wayto.model.topo.PolygonalTopo;
import com.wayto.model.topo.Section;

/**
 * @author m_deli02
 *
 */
/**
 * @author m_deli02
 *
 */
/**
 * @author m_deli02
 *
 */
/**
 * @author m_deli02
 *
 */
public class LinearTransformation2 {
	
	
	/*		According to equations 4.18 and 419
	 * SIMiLARITY 

	Minimal 2 points
	if more then 2 points it requires require Least Square Methods (approximation)
			Xt1 = 1*a0 + 0*b0 + Xs1*a + Ys1*b
			Yt1 = 0*a0 + 1*b0 + Ys1*a - Xs1*b
			Xt2 = 1*a0 + 0*b0 + Xs2*a + Ys2*b
			Yt2 = 0*a0 + 1*b0 + Ys2*a - Xs2*b
			Xt3 = 1*a0 + 0*b0 + Xs3*a + Ys2*b
			Yt3 = 0*a0 + 1*b0 + Ys3*a - Xs3*b 
	 
	 		
	 		AFFINE
	According to equations 4.32 and 4.32

	MINIMAL 3 Points
	if more then 3 points it requires Least Square Methods (approximation)
			Xt1 = 1*a0 + 0*b0 + Xs1*a1 + 0*b1   + Ys1*a2 + 0*b2
			Yt1 = 0*a0 + 1*b0 + 0*a1   + Xs1*b1 + 0*a2   + Ys1*b2		
			Xt2 = 1*a0 + 0*b0 + Xs2*a1 + 0*b1   + Ys2*a2 + 0*b2
			Yt2 = 0*a0 + 1*b0 + 0*a1   + Xs2*b1 + 0*a2   + Ys2*b2		
			Xt3 = 1*a0 + 0*b0 + Xs3*a1 + 0*b1   + Ys3*a2 + 0*b2
			Yt3 = 0*a0 + 1*b0 + 0*a1   + Xs3*b1 + 0*a2   + Ys3*b2


	Similarity is a kind of Affine where

	a1 = b2 = a
	a2 = -b1 = b


	To use AffineTransform2D(geotools) or AffineTransform

	the matrix parameter are:

	m00 = a1, m01 = a2, m02 = a0, m10 = b1, m11 = b2, m12 = b0

	      [ x']   [  m00  m01  m02  ] [ x ]   [ m00x + m01y + m02 ]
	      [ y'] = [  m10  m11  m12  ] [ y ] = [ m10x + m11y + m12 ]
	      [ 1 ]   [   0    0    1   ] [ 1 ]   [         1         ]
			*/
	
	public ArrayList<Point2D> transformNetworkPath(Path p, double proportion) {
		
		ArrayList<Point2D> transfPath = new ArrayList<Point2D>();
		
		Point source1 = p.getNodeList().get(0).getProjectGeom();
		Point source2 = p.getNodeList().get(p.getNodeList().size() -1).getProjectGeom();
		
		Point target1 = p.getNodeList().get(0).getxGeom();
		Point target2 = p.getNodeList().get(p.getNodeList().size() -1).getxGeom();

		if(p.asLineString(0).getLength() == 0){
			for(StreetNode n: p.getNodeList()){
				transfPath.add( GeoConvertionsOperations.PointJTSGeometryToJava2D(target1) );
				
				
			}
			
		}
		/*if path is ring: starts and end in the point*/
		else if(target1 != null && target2 != null 
				&& p.getNodeList().get(0).equals(p.getNodeList().get(p.getNodeList().size() -1))){
			AffineTransform scale = AffineTransform.getScaleInstance(proportion, proportion);
			for(StreetNode n: p.getNodeList()){
				transfPath.add( scale.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
				
				
			}
			transfPath.get(0);
			AffineTransform translate= AffineTransform.getTranslateInstance(target1.getX() - transfPath.get(0).getX() ,
					 target1.getY() - transfPath.get(0).getY() );
			
			for(int i = 0; i < transfPath.size(); i++){
				translate.transform(transfPath.get(i), transfPath.get(i));
				
				
			}
			
		}
		else if(target1 != null && target2 != null){
			
			double xS1, yS1, xS2, yS2;
			double xT1, yT1, xT2, yT2;
			
			xS1 = source1.getX();
			yS1 = source1.getY();
			xS2 = source2.getX();
			yS2 = source2.getY();
			
			xT1 = target1.getX();
			yT1 = target1.getY();
			xT2 = target2.getX();
			yT2 = target2.getY();
			
			RealMatrix coefficients =
				    new Array2DRowRealMatrix(new double[][] { { 1 , 0 , xS1 , yS1 }, 
				    										  { 0 , 1 , yS1 , -xS1 },
				    										  { 1 , 0 , xS2 , yS2 },
				    										  { 0 , 1 , yS2 , -xS2}},
				    										  false);
			DecompositionSolver solver = new LUDecomposition(coefficients).getSolver();
			
			RealVector constants = new ArrayRealVector(new double[] {xT1, yT1, xT2, yT2 }, false);
			RealVector parameters = solver.solve(constants);
			
			double m00, m10, m01, m11, m02, m12;
			m00 = parameters.getEntry(2);
			m10 = -1*parameters.getEntry(3);
			m01 = parameters.getEntry(3);
			m11 = parameters.getEntry(2);
			m02 = parameters.getEntry(0);
			m12 = parameters.getEntry(1);
			
			AffineTransform similarityTransform = new AffineTransform(m00, m10, m01, m11, m02, m12);
			
			for(StreetNode n: p.getNodeList()){
				transfPath.add( similarityTransform.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
				
				
			}
			

			
			
		}
		else if(target1 != null && target2 == null){
			
			AffineTransform scale = AffineTransform.getScaleInstance(proportion, proportion);
			for(StreetNode n: p.getNodeList()){
				transfPath.add( scale.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
				
				
			}
			transfPath.get(0);
			AffineTransform translate= AffineTransform.getTranslateInstance(target1.getX() - transfPath.get(0).getX() ,
					 target1.getY() - transfPath.get(0).getY() );
			
			for(int i = 0; i < transfPath.size(); i++){
				translate.transform(transfPath.get(i), transfPath.get(i));
				
				
			}
					
//			AffineTransform translate= AffineTransform.getTranslateInstance(target1.getX() - source1.getX() ,
//					 target1.getY() - source1.getY() );
//			
//			
//			
//			for(StreetNode n: p.getNodeList()){
//				transfPath.add( translate.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
//				
//				
//			}
			
		}
		else if(target1 == null && target2 != null){
			
			
			AffineTransform scale = AffineTransform.getScaleInstance(proportion, proportion);
			for(StreetNode n: p.getNodeList()){
				transfPath.add( scale.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
				
				
			}
			transfPath.get(0);
			AffineTransform translate= AffineTransform.getTranslateInstance(target2.getX() - transfPath.get(transfPath.size()-1).getX() ,
					 target2.getY() - transfPath.get(transfPath.size()-1).getY() );
			
			for(int i = 0; i < transfPath.size(); i++){
				translate.transform(transfPath.get(i), transfPath.get(i));
				
				
			}
			
			
//			AffineTransform translate= AffineTransform.getTranslateInstance(target2.getX() - source2.getX() ,
//					 target2.getY() - source2.getY() );
//			
//			for(StreetNode n: p.getNodeList()){
//				transfPath.add( translate.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
//				
//				
//			}
			
		}
		//probably disconnected
		else if(target1 == null && target2 == null){
			
			AffineTransform scale = AffineTransform.getScaleInstance(proportion, proportion);
			for(StreetNode n: p.getNodeList()){
				transfPath.add( scale.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
				
				
			}

			
		}
		
		return transfPath;
		
	}
	public ArrayList<Point2D> transformFullPolygon(Path p, int type, double proportion) {
		StreetNode cp1 = null;
		StreetNode cp2 = null;
		boolean achouCP1 =false;
		boolean achouCP2 =false;
		ArrayList<Point2D> transfPath = new ArrayList<Point2D>();
		
		
		for(int i =0; i <p.getNodeList().size() -1; i++) {
			for(NodeTopoRelation ntr: p.getNodeList().get(i).getTopoRelations())
			{
				if (ntr.getType() == NodeTopoRelation.ANCHOR) {
					if(!achouCP1) {
						cp1 = p.getNodeList().get(i);
						achouCP1 = true;
						break;
					}
					else {
						cp2 = p.getNodeList().get(i);
						achouCP2 =true;
						break;
						
					}
					
				}
			}
			if(achouCP2)
				break;
			
			
		}
		if(cp1 !=null && cp2 !=null) {
			Point source1 = cp1.getProjectGeom();
			Point source2 = cp2.getProjectGeom();

			Point target1 = cp1.getxGeom();
			Point target2 = cp2.getxGeom();

			if(target1 != null && target2 != null){

				double xS1, yS1, xS2, yS2;
				double xT1, yT1, xT2, yT2;

				xS1 = source1.getX();
				yS1 = source1.getY();
				xS2 = source2.getX();
				yS2 = source2.getY();

				xT1 = target1.getX();
				yT1 = target1.getY();
				xT2 = target2.getX();
				yT2 = target2.getY();

				RealMatrix coefficients =
						new Array2DRowRealMatrix(new double[][] { { 1 , 0 , xS1 , yS1 }, 
							{ 0 , 1 , yS1 , -xS1 },
							{ 1 , 0 , xS2 , yS2 },
							{ 0 , 1 , yS2 , -xS2}},
								false);
				DecompositionSolver solver = new LUDecomposition(coefficients).getSolver();

				RealVector constants = new ArrayRealVector(new double[] {xT1, yT1, xT2, yT2 }, false);
				RealVector parameters = solver.solve(constants);

				double m00, m10, m01, m11, m02, m12;
				m00 = parameters.getEntry(2);
				m10 = -1*parameters.getEntry(3);
				m01 = parameters.getEntry(3);
				m11 = parameters.getEntry(2);
				m02 = parameters.getEntry(0);
				m12 = parameters.getEntry(1);

				AffineTransform similarityTransform = new AffineTransform(m00, m10, m01, m11, m02, m12);
				double scale = similarityTransform.getScaleX();

				ArrayList<Point2D> tempTransfPath = new ArrayList<Point2D>();
				if(type == PolygonalTopo.ROUTE_IS_INSIDE||
						type == PolygonalTopo.GLOBAL ) {


					double angle = extractAngle(similarityTransform);
					System.out.println("Turn angle of it is: " + Math.toDegrees(angle) + " X Scale is: " + similarityTransform.getScaleX() + " Y Scale is: " + similarityTransform.getScaleY() + " Best proportion is: " + proportion);
					Point2D centroid = GeoConvertionsOperations.PointJTSGeometryToJava2D(p.asLineString(1).getCentroid());
					AffineTransform rotationFixTransform = new AffineTransform();

					rotationFixTransform.translate(centroid.getX(), centroid.getY());      // S3: final translation
					//scaleAndRotationFixTransform.scale(-scale, -scale);
					rotationFixTransform.rotate(-angle); // S2: rotate around anchor					
					rotationFixTransform.translate(-centroid.getX(), -centroid.getY());    // S1: translate anchor to origin
					//scaleAndRotationFixTransform.rotate(-angle, centroid.getX(), centroid.getY());
					//


					ArrayList<Point2D> tempTransfPath2 = new ArrayList<Point2D>();
					for(StreetNode n: p.getNodeList()){
						//transfPath.add( similarityTransform.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
						tempTransfPath2.add(rotationFixTransform.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );

					}


					for(Point2D pt: tempTransfPath2) {
						tempTransfPath.add( similarityTransform.transform(pt, null) );
					}


				}
				else {

					for(StreetNode n: p.getNodeList()){
						tempTransfPath.add( similarityTransform.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
					}

				}

				double originalLenght = GeometricOperation.length(p.asJava2DList(1));
				double transformedLendth = GeometricOperation.length(tempTransfPath);
				
				double perimeterScale = transformedLendth/originalLenght;
				
				System.out.println("pemrimeter scale = " + perimeterScale);
				double proportionScale = proportion/perimeterScale;

				double mixedScale = ((transformedLendth + originalLenght*proportion)/2)/transformedLendth;


				//Point2D transCentroid = GeoConvertionsOperations.PointJTSGeometryToJava2D(GeoConvertionsOperations.Java2DToJTSGeometry(tempTransfPath, Geometries.POLYGON).getCentroid());

				Point2D.Double scaleCenter = new Point2D.Double( (target1.getX() + target2.getX())/2 , (target1.getY() + target2.getY())/2 );
				AffineTransform scaleFixTransform = new AffineTransform();

				scaleFixTransform.translate(scaleCenter.getX(), scaleCenter.getY());

				//System.out.println("inverted scale = " + proportionScale);
				scaleFixTransform.scale(proportionScale, proportionScale);
				scaleFixTransform.translate(-scaleCenter.getX(), -scaleCenter.getY());
				for(Point2D pt: tempTransfPath) {
					transfPath.add( scaleFixTransform.transform(pt, null) );
				}



			}
			//				else {
			//					for(StreetNode n: p.getNodeList()){
			//						transfPath.add( similarityTransform.transform(GeoConvertionsOperations.PointJTSGeometryToJava2D(n.getProjectGeom()), null) );
			//	
			//	
			//					}
			//					
			//				}
			//				

			//						
			//				//similarityTransform.rotate(-angle, centroid.getX(), centroid.getY());

			//			}	

		}

		return transfPath;
	}

	public ArrayList<Point2D> transformRouteAdjEdge(Path p, double proportion, double minDist) {

		ArrayList<Point2D> transfPath = new ArrayList<Point2D>();
		//System.out.println("new length: " + p.getNodeList().get(0).distance(p.getNodeList().get(1)) );
		if(p.getNodeList().get(0).isRouteNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(0).getxGeom()));

			double dist = Math.max(minDist,proportion*p.getNodeList().get(0).getProjectGeom().distance(p.getNodeList().get(1).getProjectGeom()) );
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(0).getxGeom().getX(), p.getNodeList().get(0).getxGeom().getY(),
					p.getNodeList().get(1).getxGeom().getX(), p.getNodeList().get(1).getxGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(0).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(0).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );
			//System.out.println(transfPath );
						
			
		}
		else if(p.getNodeList().get(1).isRouteNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(1).getxGeom()));

			double dist = Math.max(minDist, proportion*p.getNodeList().get(1).getProjectGeom().distance(p.getNodeList().get(0).getProjectGeom()) );
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(1).getxGeom().getX(), p.getNodeList().get(1).getxGeom().getY(),
					p.getNodeList().get(0).getxGeom().getX(), p.getNodeList().get(0).getxGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(1).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(1).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(0,new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );			
			//System.out.println(transfPath );	
			
			
		}

		
		
		return transfPath;
	}
	
	
	public ArrayList<Point2D> transformRouteAdjEdgeFixDist(Path p, double minDist) {

		ArrayList<Point2D> transfPath = new ArrayList<Point2D>();
		//System.out.println("new length: " + p.getNodeList().get(0).distance(p.getNodeList().get(1)) );
		if(p.getNodeList().get(0).isRouteNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(0).getxGeom()));

			double dist = minDist;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(0).getxGeom().getX(), p.getNodeList().get(0).getxGeom().getY(),
					p.getNodeList().get(1).getxGeom().getX(), p.getNodeList().get(1).getxGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(0).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(0).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );
			//System.out.println(transfPath );
						
			
		}
		else if(p.getNodeList().get(1).isRouteNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(1).getxGeom()));

			double dist = minDist;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(1).getxGeom().getX(), p.getNodeList().get(1).getxGeom().getY(),
					p.getNodeList().get(0).getxGeom().getX(), p.getNodeList().get(0).getxGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(1).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(1).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(0,new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );			
			//System.out.println(transfPath );	
			
			
		}

		
		
		return transfPath;
	}
	
	/**Adjusct Schematized Control Edge length based on the proportion**/
	public ArrayList<Point2D> transformControlEdge(Path p, double proportion) {

		ArrayList<Point2D> transfPath = new ArrayList<Point2D>();
		//System.out.println("new length: " + p.getNodeList().get(0).distance(p.getNodeList().get(1)) );
		if(p.getNodeList().get(0).isRouteNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(0).getxGeom()));

			double dist = proportion*p.getNodeList().get(0).getProjectGeom().distance(p.getNodeList().get(1).getProjectGeom()) ;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(0).getProjectGeom().getX(), p.getNodeList().get(0).getProjectGeom().getY(),
					p.getNodeList().get(1).getProjectGeom().getX(), p.getNodeList().get(1).getProjectGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(0).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(0).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );
			//System.out.println(transfPath );
						
			
		}
		else if(p.getNodeList().get(1).isRouteNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(1).getxGeom()));

			double dist =  proportion*p.getNodeList().get(1).getProjectGeom().distance(p.getNodeList().get(0).getProjectGeom()) ;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(1).getProjectGeom().getX(), p.getNodeList().get(1).getProjectGeom().getY(),
					p.getNodeList().get(0).getProjectGeom().getX(), p.getNodeList().get(0).getProjectGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(1).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(1).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(0,new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );			
			//System.out.println(transfPath );	
			
			
		}
		else if(p.getNodeList().get(0).isStreetNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(0).getxGeom()));

			double dist = proportion*p.getNodeList().get(0).getProjectGeom().distance(p.getNodeList().get(1).getProjectGeom()) ;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(0).getProjectGeom().getX(), p.getNodeList().get(0).getProjectGeom().getY(),
					p.getNodeList().get(1).getProjectGeom().getX(), p.getNodeList().get(1).getProjectGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(0).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(0).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );
			//System.out.println(transfPath );
						
			
		}
		else if(p.getNodeList().get(1).isStreetNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(1).getxGeom()));

			double dist =  proportion*p.getNodeList().get(1).getProjectGeom().distance(p.getNodeList().get(0).getProjectGeom()) ;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(1).getProjectGeom().getX(), p.getNodeList().get(1).getProjectGeom().getY(),
					p.getNodeList().get(0).getProjectGeom().getX(), p.getNodeList().get(0).getProjectGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(1).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(1).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(0,new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );			
			//System.out.println(transfPath );	
			
			
		}

		
		
		return transfPath;
	}
	
	
	/**Adjusct Schematized Control Edge length based on the proportion**/
	public ArrayList<Point2D> transformControlEdgeFixDist(Path p, double fixDist) {

		ArrayList<Point2D> transfPath = new ArrayList<Point2D>();
		//System.out.println("new length: " + p.getNodeList().get(0).distance(p.getNodeList().get(1)) );
		if(p.getNodeList().get(0).isRouteNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(0).getxGeom()));

			double dist = fixDist ;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(0).getProjectGeom().getX(), p.getNodeList().get(0).getProjectGeom().getY(),
					p.getNodeList().get(1).getProjectGeom().getX(), p.getNodeList().get(1).getProjectGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(0).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(0).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );
			//System.out.println(transfPath );
						
			
		}
		else if(p.getNodeList().get(1).isRouteNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(1).getxGeom()));

			double dist =  fixDist ;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(1).getProjectGeom().getX(), p.getNodeList().get(1).getProjectGeom().getY(),
					p.getNodeList().get(0).getProjectGeom().getX(), p.getNodeList().get(0).getProjectGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(1).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(1).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(0,new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );			
			//System.out.println(transfPath );	
			
			
		}
		else if(p.getNodeList().get(0).isStreetNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(0).getxGeom()));

			double dist = fixDist;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(0).getProjectGeom().getX(), p.getNodeList().get(0).getProjectGeom().getY(),
					p.getNodeList().get(1).getProjectGeom().getX(), p.getNodeList().get(1).getProjectGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(0).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(0).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );
			//System.out.println(transfPath );
						
			
		}
		else if(p.getNodeList().get(1).isStreetNode()){
			
			transfPath.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(1).getxGeom()));

			double dist =  fixDist ;
			
			double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(p.getNodeList().get(1).getProjectGeom().getX(), p.getNodeList().get(1).getProjectGeom().getY(),
					p.getNodeList().get(0).getProjectGeom().getX(), p.getNodeList().get(0).getProjectGeom().getY());
//			System.out.println("ANGLE: " + angle + "deg " + Math.toDegrees(angle) );
//			System.out.println("O Point" + simplifiedLineList.get(i));
//			System.out.println("O Point" + simplifiedLineList.get(i+1));
//			System.out.println("Dist:" + dist);
			
			double newX = p.getNodeList().get(1).getxGeom().getX() + dist*Math.cos(angle);
			double newY = p.getNodeList().get(1).getxGeom().getY() - dist*Math.sin(angle);
			transfPath.add(0,new Point2D.Double(newX, newY));
			//System.out.println("new length: " + transfPath.get(0).distance(transfPath.get(1)) );			
			//System.out.println(transfPath );	
			
			
		}

		
		
		return transfPath;
	}
	
	



//	public void transformCrossedPolygons(PolygonalTopo p, Envelope env) {
//		System.out.println("FINDING PARAMENTERS OF ROUTE_STARTS_AT" + p.getPolygonalFeature().getName());
//		System.out.println("Number of sections: " + p.getSectionList().size());
//		System.out.println("Number of Control Points: " + p.getControlPoints().size());
//		for(Section s : p.getSectionList()){
//			if(!s.isFollowTheRoute()){
//				double xS1, yS1, xS2, yS2;
//				double xT1, yT1, xT2, yT2;
//				
//				xS1 = p.getControlPoints().get(s.getCP1Index()).getOriginalPosition().getX();
//				yS1 = p.getControlPoints().get(s.getCP1Index()).getOriginalPosition().getY();
//				xS2 = p.getControlPoints().get(s.getCP2Index()).getOriginalPosition().getX();
//				yS2 = p.getControlPoints().get(s.getCP2Index()).getOriginalPosition().getY();
//				
//				if(p.getControlPoints().get(s.getCP1Index()).isRelative()){
//					Point2D pt =	GeoConvertionsOperations.PointJTSGeometryToJava2D(								
//							GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(s.getCP1Index()).getNode().getxGeom(), env)																			
//							);
//					
//					
//					pt = GeometricOperation.toCartezian(pt, p.getControlPoints().get(s.getCP1Index()).getRelativeToNodePosition());
//					xT1 = pt.getX();
//					yT1 = pt.getY();
//				}
//				else{
//					Point CP1XDenormilize = GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(s.getCP1Index()).getNode().getxGeom(), env);
//
//					
//					xT1 = CP1XDenormilize.getX();
//					yT1 = CP1XDenormilize.getY();
//					
//				}
//				
//				
//				if(p.getControlPoints().get(s.getCP2Index()).isRelative()){
//					Point2D pt =	GeoConvertionsOperations.PointJTSGeometryToJava2D(
//							GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(s.getCP2Index()).getNode().getxGeom(), env)
//							);
//					
//					pt = GeometricOperation.toCartezian(pt, p.getControlPoints().get(s.getCP2Index()).getRelativeToNodePosition());
//					xT2 = pt.getX();
//					yT2 = pt.getY();
//				}
//				else{
//					Point CP2XDenormilize = GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(s.getCP2Index()).getNode().getxGeom(), env);
//					xT2 = CP2XDenormilize.getX();
//					yT2 = CP2XDenormilize.getY();
//					
//				}
//					
//				/*GeometricOperation.toCartezian(point2D, polarPoint)
//				xT1 = p.getControlPoints().get(s.getCP1Index()).getNode().getxGeom().getX();
//				yT1 = p.getControlPoints().get(s.getCP1Index()).getNode().getxGeom().getY();
//				xT2 = p.getControlPoints().get(s.getCP2Index()).getNode().getxGeom().getX();
//				yT2 = p.getControlPoints().get(s.getCP2Index()).getNode().getxGeom().getY();*/
//				
//				
//				RealMatrix coefficients =
//					    new Array2DRowRealMatrix(new double[][] { { 1 , 0 , xS1 , yS1 }, 
//					    										  { 0 , 1 , yS1 , -xS1 },
//					    										  { 1 , 0 , xS2 , yS2 },
//					    										  { 0 , 1 , yS2 , -xS2}},
//					    										  false);
//				DecompositionSolver solver = new LUDecomposition(coefficients).getSolver();
//				
//				RealVector constants = new ArrayRealVector(new double[] {xT1, yT1, xT2, yT2 }, false);
//				RealVector parameters = solver.solve(constants);
//				transformSimmilarity(s, parameters);
//
//				
//				
//			}
//		}
//		
//	}
	
	
	
//	public void transformPassAlong(PolygonalTopo p, Envelope env) {
//		System.out.println("FINDING PARAMENTERS OF PASSING_ALONG_LEFT" + p.getPolygonalFeature().getName());
//		System.out.println("Number of sections: " + p.getSectionList().size());
//		System.out.println("Number of Control Points: " + p.getControlPoints().size());
//	
//		
//		
//				double xS1, yS1, xS2, yS2;
//				double xT1, yT1, xT2, yT2;
//
//				
//				xS1 = p.getControlPoints().get(0).getOriginalPosition().getX();
//				yS1 = p.getControlPoints().get(0).getOriginalPosition().getY();
//				xS2 = p.getControlPoints().get(1).getOriginalPosition().getX();
//				yS2 = p.getControlPoints().get(1).getOriginalPosition().getY();
//				
//				Point CP1XDenormilize = null;
//				Point CP2XDenormilize = null;
//				
//				
//				if(p.getControlPoints().get(0).isRelative()){
//					Point2D pt =	GeoConvertionsOperations.PointJTSGeometryToJava2D(								
//							GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(0).getNode().getxGeom(), env)																			
//							);
//					
//					
//					CP1XDenormilize = GeoConvertionsOperations.Java2DPointToJTSGeometry(GeometricOperation.toCartezian(pt, p.getControlPoints().get(0).getRelativeToNodePosition()));
//					
//				}
//				else{
//					CP1XDenormilize = GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(0).getNode().getxGeom(), env);
//				
//				}
//				
//				
//				if(p.getControlPoints().get(1).isRelative()){
//					Point2D pt =	GeoConvertionsOperations.PointJTSGeometryToJava2D(
//							GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(1).getNode().getxGeom(), env)
//							);
//					
//					CP2XDenormilize = GeoConvertionsOperations.Java2DPointToJTSGeometry(GeometricOperation.toCartezian(pt, p.getControlPoints().get(1).getRelativeToNodePosition()));
//
//				}
//				else{
//					CP2XDenormilize = GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(1).getNode().getxGeom(), env);
//
//					
//				}
//				
//			
////				Point CP1XDenormilize = GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(0).getNode().getxGeom(), env);
////				Point CP2XDenormilize = GeoConvertionsOperations.denormalizeJTSPoint(p.getControlPoints().get(1).getNode().getxGeom(), env);
////
////				
//				xT1 = CP1XDenormilize.getX();
//				yT1 = CP1XDenormilize.getY();
//				xT2 = CP2XDenormilize.getX();
//				yT2 = CP2XDenormilize.getY();
//
//				
//				
//				RealMatrix coefficients =
//					    new Array2DRowRealMatrix(new double[][] { { 1 , 0 , xS1 , yS1 }, 
//					    										  { 0 , 1 , yS1 , -xS1 },
//					    										  { 1 , 0 , xS2 , yS2 },
//					    										  { 0 , 1 , yS2 , -xS2}},
//					    										  false);
//				DecompositionSolver solver = new LUDecomposition(coefficients).getSolver();
//				
//				RealVector constants = new ArrayRealVector(new double[] {xT1, yT1, xT2, yT2 }, false);
//				RealVector parameters = solver.solve(constants);
//				//transformSimmilarity(s, parameters);
//				System.out.println("Parameters: " + parameters);
//				/*first section connects the firs and last control point in the route*/
//				//if(p.getSectionList().indexOf(s) == 0){
//					transformSimmilarity(p, parameters, CP2XDenormilize);
//				//}
//				
//
//	}
	
//	public void transformPassAlong2(PolygonalTopo p, Envelope env, double alongness) {
//		System.out.println("FINDING PARAMENTERS OF " + p.getPolygonalFeature().getName());
//		System.out.println("Number of sections: " + p.getSectionList().size());
//		System.out.println("Number of Control Points: " + p.getControlPoints().size());
//		
//		ArrayList<Point> sources = new ArrayList<Point>();
//		ArrayList<Point> targets = new ArrayList<Point>();
//		
//		for(ControlPoint cp: p.getControlPoints()){
//			sources.add(cp.getOriginalPosition());
//			
//			if(cp.isRelative()){
//				Point2D pt =	GeoConvertionsOperations.PointJTSGeometryToJava2D(								
//						GeoConvertionsOperations.denormalizeJTSPoint(cp.getNode().getxGeom(), env)																			
//						);
//				
//				
//				targets.add( GeoConvertionsOperations.Java2DPointToJTSGeometry(GeometricOperation.toCartezian(pt, cp.getRelativeToNodePosition().getTheta(), ((100 - alongness)/100)*cp.getRelativeToNodePosition().getR() )) );
//				
//			}
//			else{
//				targets.add( GeoConvertionsOperations.denormalizeJTSPoint(cp.getNode().getxGeom(), env) );
//			
//			}
//			
//			
//			
//		}
//		/**Simple tranlation **/
//		if(sources.size() == 1){
//			double xTranslation = targets.get(0).getX() - sources.get(0).getX();
//			double yTranslation = targets.get(0).getY() - sources.get(0).getY();
//			AffineTransformation simpleTranslationTransform = new AffineTransformation(1, 0, xTranslation, 0, 1, yTranslation);
//			p.setTransformedShape((LineString)simpleTranslationTransform.transform(p.getPolygonalFeature().getGeom().getExteriorRing()));
//			
//		}	
//		/**Affine Transformation **/
//		else if ( sources.size() == 2) {
//			
//			double xS1, yS1, xS2, yS2;
//			double xT1, yT1, xT2, yT2;
//
//			
//			xS1 = sources.get(0).getX();
//			yS1 = sources.get(0).getY();
//			xS2 = sources.get(1).getX();
//			yS2 = sources.get(1).getY();
//			
//			
//			xT1 = targets.get(0).getX();
//			yT1 = targets.get(0).getY();
//			xT2 = targets.get(1).getX();
//			yT2 = targets.get(1).getY();
//			
//			RealMatrix coefficients =
//				    new Array2DRowRealMatrix(new double[][] { { 1 , 0 , xS1 , yS1 }, 
//				    										  { 0 , 1 , yS1 , -xS1 },
//				    										  { 1 , 0 , xS2 , yS2 },
//				    										  { 0 , 1 , yS2 , -xS2}},
//				    										  false);
//			DecompositionSolver solver = new LUDecomposition(coefficients).getSolver();
//			
//			RealVector constants = new ArrayRealVector(new double[] {xT1, yT1, xT2, yT2 }, false);
//			RealVector parameters = solver.solve(constants);
//			//transformSimmilarity(s, parameters);
//			System.out.println("Parameters: " + parameters);
//			/*first section connects the firs and last control point in the route*/
//			//if(p.getSectionList().indexOf(s) == 0){
//				transformSimmilarity(p, parameters, targets.get(1));
//			//}
//			
//			
//		}
//		/**Least square aproximaiton **/
//		else if (sources.size() > 2 ){
//			
//			
//		}
//				
//				
//				
//
//	}
	
	

	

	private void transformSimmilarity(Section s, RealVector parameters) {
		
		/*Similarity is a kind of Affine where

		a1 = b2 = a
		a2 = -b1 = b


		To use AffineTransform2D(geotools) or AffineTransform

		the matrix parameter are:

		m00 = a1, m01 = a2, m02 = a0, m10 = b1, m11 = b2, m12 = b0
		
		scalefactor = sqrt(a1^2 + a2^2) 
		rotation = tan(angle) = b/a*/
		
		
		double m00, m10, m01, m11, m02, m12;
		m00 = parameters.getEntry(2);
		m10 = -1*parameters.getEntry(3);
		m01 = parameters.getEntry(3);
		m11 = parameters.getEntry(2);
		m02 = parameters.getEntry(0);
		m12 = parameters.getEntry(1);
		
		System.out.println("Scale = " + Math.sqrt(m00*m00 + m01*m01));
		System.out.println("Rotation = " + Math.atan(m01/m00));
		
		AffineTransformation shapeTransform = new AffineTransformation(m00, m01, m02, m10, m11, m12);
		
		
		
		//AffineTransform2D transformation = new AffineTransform2D(m00, m10, m01, m11, m02, m12);
		
		
		s.setShapeTranformed( (LineString)shapeTransform.transform(s.getShape()));
		
	}


	 private static double extractAngle(AffineTransform at)
	    {
		 	
		    Point2D.Double p0 = new Point2D.Double();
		    Point2D.Double p1 = new Point2D.Double(1,0);
	        Point2D pp0 = at.transform(p0, null);
	        Point2D pp1 = at.transform(p1, null);
	        double dx = pp1.getX() - pp0.getX();
	        double dy = pp1.getY() - pp0.getY();
	        double angle = Math.atan2(dy, dx);
	        return angle;
	    }





	

	
//	/**
//	 * 
//	 * @param polyTopo
//	 * @param parameters
//	 * @param anchor : used to derotate
//	 */
//	private void transformSimmilarity(PolygonalTopo polyTopo, RealVector parameters, Point anchor) {
//		
//		double m00, m10, m01, m11, m02, m12;
//		m00 = parameters.getEntry(2);
//		m10 = -1*parameters.getEntry(3);
//		m01 = parameters.getEntry(3);
//		m11 = parameters.getEntry(2);
//		m02 = parameters.getEntry(0);
//		m12 = parameters.getEntry(1);
//		
//		
//		double rotation = -Math.atan(m01/m00);
//		double scale = Math.sqrt(m00*m00 + m01*m01);
//		System.out.println("Scale = " + scale);
//		System.out.println("Rotation = " + Math.toDegrees(rotation) + " " + rotation);
//		System.out.println("Translation X = " + m02);
//		System.out.println("Translation Y = " + m12);
//		
//		AffineTransformation shapeTransform = new AffineTransformation(m00, m01, m02, m10, m11, m12);
//		
//		if(scale > 1.20){
//			AffineTransformation descale= AffineTransformation.scaleInstance((1.20/scale) , (1.20/scale) ,
//					anchor.getX(), anchor.getY());
//			shapeTransform.compose(descale);
//		}
////		if(scale < 0.8){
////			AffineTransformation descale= AffineTransformation.scaleInstance((0.8/scale) , (0.8/scale) ,
////					anchor.getX(), anchor.getY());
////			shapeTransform.compose(descale);
////		}
//		
////		
//		if(rotation > 0.1){
//			AffineTransformation derotate= AffineTransformation.rotationInstance((-rotation) + 0.1,
//						anchor.getX(), anchor.getY());
//				shapeTransform.compose(derotate);
//			
//		}
//		if(rotation < -0.1){
//			AffineTransformation derotate= AffineTransformation.rotationInstance((-rotation) - 0.1,
//						anchor.getX(), anchor.getY());
//				shapeTransform.compose(derotate);
//			
//		}
////		
//		
//		
//		
//		
//		//shapeTransform.scale(-Math.sqrt(m00*m00 + m01*m01), -Math.sqrt(m00*m00 + m01*m01));
//		//shapeTransform.rotate(-Math.atan(m01/m00), -m02, -m12);
//		//AffineTransform2D transformation = new AffineTransform2D(m00, m10, m01, m11, m02, m12);
//		System.out.println("OriginalLineString: " + polyTopo.getPolygonalFeature().getGeom().getExteriorRing());
//		polyTopo.setTransformedShape((LineString)shapeTransform.transform(polyTopo.getPolygonalFeature().getGeom().getExteriorRing()));
//		
//		System.out.println("TranfLineString: " + polyTopo.getTransformedShape());
//
//		
////
////		
////		
////		
////		AffineTransformation reajust = new AffineTransformation(descale);
////		reajust.compose(derotate);
////		polyTopo.setTransformedShape((LineString)reajust.transform(polyTopo.getTransformedShape()));
//
//		
//		//s.setShapeX( (LineString)shapeTransform.transform(s.getShape()));
//		
//	}

}