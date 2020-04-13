package com.wayto.operator;


import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;

import com.goebl.simplify.PointExtractor;
import com.goebl.simplify.Simplify;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.PrecisionModel;
import com.wayto.model.Path;
import com.wayto.model.StreetNode;


  





public class GeneticOperation { 

	private double rotation = 0;
	private double mutationIndex = 50;
	private final int populationSize = 10;
	
	Simplify<Point2D> simplify = new Simplify<Point2D>(new Point2D[0], point2DPointExtractor);
	private double stringLength = 0;
	private double trackDiagonal; 

	private  ArrayList<Point2D> originalPoints;
	private  ArrayList<Integer> angles;
	private  ArrayList<Integer> relevantPoints1;
	private  ArrayList<Integer> relevantPoints2;
	
	private ArrayList<Gene> population, populationAux;
	private ArrayList<Gene> base1, base2, base3;
	
	private int cutPoint1 = 0;
	private int cutPoint2 = 0;
	private int randomIndex1 = 0;
	private int randomIndex2 = 0;
	private double bestValue;
	private int generationsCount;
	
	private FisheyeTransform fisheyeTransform;


	private double angleCoeficient = 1;
	


	public GeneticOperation() {
		originalPoints = new ArrayList<Point2D>();
		angles = new ArrayList<Integer>();
		relevantPoints1 = new ArrayList<Integer>();
		relevantPoints2 = new ArrayList<Integer>();
		
		/*instancia os dez genes das geracoees */
		population = new ArrayList<Gene>() ;
		for (int i = 0 ; i < populationSize ; i++)
			population.add( new Gene() );
		
		populationAux = new ArrayList<Gene>() ;
		for (int i = 0 ; i < populationSize ; i++)
			populationAux.add( new Gene() );

		base1 = new ArrayList<Gene>();
		for (int i = 0 ; i < 3 ; i++) 
			base1.add( new Gene() );
		
		
		base2 = new ArrayList<Gene>();
		for (int i = 0 ; i < 4 ; i++)
			base2.add( new Gene() );
		
		

		
	}
	
	private static PointExtractor<Point2D> point2DPointExtractor = new PointExtractor<Point2D>() {
	    public double getX(Point2D point) {
	        return point.getX();
	    }

	    public double getY(Point2D point) {
	        return point.getY();
	    }
	};


	public void schematize(Path path) {
		long toalGenerationsCount;
		/*Detect bounds of the track for setting tolerences to line simplification */
		
		originalPoints.clear();
		
		


		
		
		/* cria originalPoints2d */
		for(StreetNode s: path.getNodeList()){
			/*Se já existir posicao esquematica, utilizar. Senao utilizar posicao original*/
			if(s.getxGeom() == null)
			originalPoints.add( new Point2D.Double(s.getGeom().getX(), s.getGeom().getY() ) );
			//originalPoints.add(  fisheyeTransform.transform( new Point2D.Double(s.getGeom().getX(), s.getGeom().getY() ) ) ) ;
			else /*Em tese, somente o primeiro ponto da track deve possuir posicao esquematica*/ 
				originalPoints.add( new Point2D.Double(s.getxGeom().getX(), s.getxGeom().getY() ) );
		}
		stringLength = 0;
		for(int i = 1; i < originalPoints.size(); i ++){
			stringLength = stringLength + originalPoints.get(i).distance(originalPoints.get(i -1));
		}
		

		
		/* inicializa genes inciais */
		for (int i = 0 ; i < populationSize ; i++){

			population.get(i).setSchematicPoints( getPopulation1( i ));
		}
		
		for (int i = 0 ; i < 3 ; i++){

			base1.get(i).setSchematicPoints( getPopulation1( i ));
		}
		for (int i = 4 ; i < 8 ; i++){

			base2.get(i -4).setSchematicPoints(getPopulation1( i ));
		}
		// mudan�a provisoria para teste
		//geneticMaps1.get(7).setSchematicPoints(
			//	geneticOperator.crossOver(geneticMaps1.get(0).getSchematicPoints() , geneticMaps1.get(5).getSchematicPoints()));
		
		
		/*population.get(8).setSchematicPoints(
				crossOver(population.get(1).getSchematicPoints() , population.get(4).getSchematicPoints()));
		
		population.get(9).setSchematicPoints(
				mutateLine( population.get(4).getSchematicPoints() ) );*/

		for (int i = 0 ; i < populationSize ; i++){
			population.get(i).setValue( 
					evalueteGene( population.get(i).getSchematicPoints()) );
		}
 
		Collections.sort( population );
		
		bestValue = 1000;
		
		
		/* calcula o n�mero de gera��es por rodada*/
		generationsCount = 200;
		int size = originalPoints.size();
		if( size < 10 )
			generationsCount = 20;
		else if( size < 20 )
			generationsCount = 50;
		else if( size < 50 )
			generationsCount = 100;
		else if( size < 100 )
			generationsCount = 200;
		else if( size < 150 )
			generationsCount = 300;
		else if( size < 300 )
			generationsCount = 400;
		else 
			generationsCount = 600;

		toalGenerationsCount = generationsCount;
		
		
		for (int i = 0 ; i < generationsCount; i++ )
			nextGeneration();
		

		//int j = 2;
		while ( population.get(0).getValue() < bestValue ){
			//toalGenerationsCount = toalGenerationsCount*j++;
			for (int i = 0 ; i < generationsCount; i++ )
				nextGeneration();
			bestValue = population.get(0).getValue();
		}


		//System.out.print(toalGenerationsCount + ", ");
		/*Save schematic values to STREETNODES*/
		GeometryFactory geometryFactory = new GeometryFactory( new PrecisionModel(PrecisionModel.FLOATING), 4326);
		PointsPolar bestSolutionPolarFormat = new PointsPolar();
		bestSolutionPolarFormat  = GeometricOperation.toPolar(population.get(0).getSchematicPoints());

		for(int i = 0; i < path.getNodeList().size(); i++ ){
			if( i == 0 && path.getNodeList().get(i).getxGeom() != null ){
				path.getNodeList().get(i).setxGeom( path.getNodeList().get(i).getxGeom());
				
				/*Set bytes to record with edges are connected to this station */
				path.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i).getTheta(), path.getNodeList().get(i).getEdges()));

			}	
			else{
				Coordinate coordSource = new Coordinate(population.get(0).getSchematicPoints().get(i).getX(),
						population.get(0).getSchematicPoints().get(i).getY());

				path.getNodeList().get(i).setxGeom(	 geometryFactory.createPoint(coordSource) );
				
				if( i !=0 )
					if(i < path.getNodeList().size() -1 ){
						path.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i - 1).getTheta() + Math.PI, path.getNodeList().get(i).getEdges()));
						path.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i).getTheta() , path.getNodeList().get(i).getEdges()));
					}
					else{
						path.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i - 1).getTheta() + Math.PI, path.getNodeList().get(i).getEdges()));
					}
				
				
			}	
		}
		int m = 10;
		
		originalPoints.clear();
		
		
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

	private void nextGeneration(){
		resetCutPoints();
		resetRandomIndex();
		
		populationAux.get(0).setSchematicPoints( population.get(0).getSchematicPoints());
		
		
		populationAux.get(1).setSchematicPoints(
				crossOver(population.get(0).getSchematicPoints() , population.get(randomIndex1).getSchematicPoints() ));
		
		populationAux.get(2).setSchematicPoints(
				crossOver(population.get(randomIndex2).getSchematicPoints() , population.get(0).getSchematicPoints() ));
		
		resetCutPoints();
		resetRandomIndex();
		
		populationAux.get(3).setSchematicPoints(
				crossOver(population.get(randomIndex2).getSchematicPoints() , population.get(randomIndex1).getSchematicPoints() ));
		
		resetRandomIndex();
		
		populationAux.get(4).setSchematicPoints(
				crossOver(population.get(randomIndex1).getSchematicPoints() , population.get(randomIndex2).getSchematicPoints()));
		
		resetCutPoints();
		resetRandomIndex();
		
		
		populationAux.get(5).setSchematicPoints(
				crossOver(base1.get( (int)Math.random()*3 ).getSchematicPoints() , base2.get( (int)Math.random()*3 ).getSchematicPoints() ));
		
		resetRandomIndex();
		
		populationAux.get(6).setSchematicPoints(
				crossOver(population.get(randomIndex2).getSchematicPoints() , population.get(randomIndex1).getSchematicPoints() ));
		
		resetRandomIndex();
		
		populationAux.get(7).setSchematicPoints(
				crossOver(population.get(randomIndex2).getSchematicPoints() , population.get(randomIndex1).getSchematicPoints() ));
		
		resetRandomIndex();
		
		populationAux.get(8).setSchematicPoints( population.get(1).getSchematicPoints());

		populationAux.get(9).setSchematicPoints(
				mutateLine( population.get(0).getSchematicPoints() ) );
		
		
		for (int i = 0 ; i < populationSize ; i++){

			populationAux.get(i).setValue( 
					evalueteGene( populationAux.get(i).getSchematicPoints()) );
		}
		//System.out.println("------------------");
		Collections.sort( populationAux );
		


		population = populationAux;		
		

	}
	
	
	private  ArrayList<Point2D> getPopulation1( int geneNumber ){
		
		ArrayList<Point2D> points = new ArrayList<Point2D>();
		Point2D.Double pt = new Point2D.Double();
		float g = geneNumber + 1;
		//double toleranceIndex = trackDiagonal*(Math.pow((g-1)/10, 3))/(80/(g)) ;
		double toleranceIndex = stringLength*(Math.pow((g-1)/10, 3))/(80/(g)) ;
		switch (geneNumber) {
		/*Cases 1 and 2 move original point!!!!! Case 1 was reapeated*/
		case 0:
			setRelevantAnglesIndex3(toleranceIndex ,true);
			/* trata caso inicial */
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  ),
						relevantPoints1.get( relevantPoints1.size() - 1  ) );
			
			
			break;
			
			
			/*points.add( originalPoints.get(0) );
			for (int i = 1; i < originalPoints.size(); i ++){

				pt = GeometricOperation.getNewSchematicPosition(points.get(i-1).getX(), points.get(i-1).getY(),
						originalPoints.get(i).getX(), originalPoints.get(i).getY() , rotation);	
				points.add( pt );

			}
			break;*/
		
		case 1:	
			setRelevantAnglesIndex3(toleranceIndex ,true);
			/* trata caso inicial */
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  ),
						relevantPoints1.get( relevantPoints1.size() - 1  ) );
			
			
			break;
			/*int lastIndex = originalPoints.size() - 1;
			points.add( originalPoints.get(  lastIndex ) );
			
			for (int i = lastIndex - 1 ; i >= 0 ; i --){

				pt = GeometricOperation.getNewSchematicPosition(points.get(0).getX(), points.get(0).getY(),
						originalPoints.get(i).getX(), originalPoints.get(i).getY() , rotation);	
				points.add( 0 , pt );

			}
			
			break; */
			
		case 2:	
			setRelevantAnglesIndex3(toleranceIndex,true);
			/* trata caso inicial */
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  ),
						relevantPoints1.get( relevantPoints1.size() - 1  ) );
			
			
			break;
		/*		int halfPointndex = (int)(originalPoints.size()/2);
			points.add( originalPoints.get( halfPointndex ) );
			
			for (int i = halfPointndex + 1 ; i < originalPoints.size(); i ++){

				pt = GeometricOperation.getNewSchematicPosition(points.get(i - (halfPointndex + 1) ).getX(), points.get(i- (halfPointndex + 1)).getY(),
						originalPoints.get(i).getX(), originalPoints.get(i).getY() , rotation);	
				points.add( pt );

			}
			for (int i = (int)(originalPoints.size()/2) - 1 ; i >= 0; i-- ){

				pt = GeometricOperation.getNewSchematicPosition(points.get(0).getX(), points.get(0).getY(),
						originalPoints.get(i).getX(), originalPoints.get(i).getY(), rotation);	
				points.add( 0 , pt );

			}
			
			break;*/
		
		case 3:
			setRelevantAnglesIndex3(toleranceIndex,true);
			/* trata caso inicial */
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  ),
						relevantPoints1.get( relevantPoints1.size() - 1  ) );
			
			
			break;
			/*points.add( originalPoints.get(0) );
			pt = GeometricOperation.getNewSchematicPosition(points.get(0).getX(), points.get(0).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation );	
			points.add( pt );

			points = fillLine( points , originalPoints.size() - 1 , 0 );
			break;*/
		
		case 4:/* Esquematiza somente os pontos com angula��o relevante */
			setRelevantAnglesIndex3(toleranceIndex,true);
		
			/*	setAngles();
			setRelevantAnglesIndex(); */
			/* trata caso inicial */
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  ),
						relevantPoints1.get( relevantPoints1.size() - 1  ) );
			
			
			break;
		
		case 5:/* Esquematiza somente os pontos com angula��o relevante */
			setRelevantAnglesIndex3(toleranceIndex,true);
			/*setAngles();
			setRelevantAnglesIndex2();*/
			
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints2){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
				
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints2.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints2.get( relevantPoints2.size() - 1  ),
					 relevantPoints2.get( relevantPoints2.size() - 1  ) );
			
			break;
		
		case 6:
			setRelevantAnglesIndex3(toleranceIndex,true);
			/*setAngles2();
			setRelevantAnglesIndex2();*/
			
			
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints2){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
				
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints2.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints2.get( relevantPoints2.size() - 1  )
						, relevantPoints2.get( relevantPoints2.size() - 1  ) );

			
			break;
		
		case 7:
			setRelevantAnglesIndex3(toleranceIndex,true);
			/*setAngles2();
			setRelevantAnglesIndex();*/
			
			
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
				
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  )
						, relevantPoints1.get( relevantPoints1.size() - 1  ) );
			
			
			break;
			
			
		case 8:

			setRelevantAnglesIndex3(toleranceIndex ,true);
			/* trata caso inicial */
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  ),
						relevantPoints1.get( relevantPoints1.size() - 1  ) );
			
			
			break;
		
		case 9:

			setRelevantAnglesIndex3(toleranceIndex ,true);
			/* trata caso inicial */
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  ),
						relevantPoints1.get( relevantPoints1.size() - 1  ) );
			
			
			break;
		
		case 10:

			setRelevantAnglesIndex3(toleranceIndex ,true);
			/* trata caso inicial */
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition(points.get( lastIndexInPoints  ).getX(), points.get(lastIndexInPoints ).getY(),
						originalPoints.get( i ).getX(), originalPoints.get( i ).getY() , rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition(points.get( points.size() - 1 ).getX(), points.get(points.size() - 1).getY(),
					originalPoints.get( originalPoints.size() - 1 ).getX(), originalPoints.get( originalPoints.size() - 1 ).getY() , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  ),
						relevantPoints1.get( relevantPoints1.size() - 1  ) );
			
			
			break;
			
		default:
			break;
		}
		
	
		return points;
	}
	
	
	
	/* completa */
	private ArrayList<Point2D> fillLine2( ArrayList<Point2D> original , int size , int start){
		ArrayList<Point2D> points = new ArrayList<Point2D>();
		points.addAll( original );
		double deltaX = (points.get( start + 1 ).getX() - points.get( start ).getX() ) / size;
		double deltaY =  (points.get( start + 1 ).getY() - points.get( start ).getY() ) / size;
		
		for (int i = start + 1 ; i < size + start; i ++ ){
			points.add(i,  new Point2D.Double( points.get( start ).getX() + (i - start)*deltaX ,
					points.get( start ).getY() + (i - start)*deltaY ));
			
//			points.add(i,  new Point2D.Double( points.get(0).getX() + i*xDif ,
//					points.get(0).getY() + i*yDif ));
			
		}
		
		return points;
	}
	
	
	/* Distancia proporcial a distancia em ralacao ao tamanho total*/
	private ArrayList<Point2D> fillLine ( ArrayList<Point2D> original , int size , int start){
		ArrayList<Point2D> points = new ArrayList<Point2D>();
		points.addAll( original );
		
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

	



	private void setAngles() {
		angles.clear();
		int lastIndex = originalPoints.size() - 1;
		for (Point2D ponto: originalPoints){
			
			double angleRad =	GeometricOperation.getAngleBetweenVectors( ponto.getX(), ponto.getY() ,
					originalPoints.get(0).getX() , originalPoints.get(0).getY() ,
					originalPoints.get(lastIndex).getX() , originalPoints.get(lastIndex).getY()  );

			
			//double degAng = (int)Math.toDegrees( angleRad );
			/* convers�o para inteiro necess�ria, pois NaN vira 0*/
			angles.add((int)Math.toDegrees( angleRad ) );
		}
	}
	
	private void setAngles2() {
		angles.clear();
		int size = originalPoints.size();
		int halfSize = size/2;
		
		for (int i = 0 ; i < halfSize; i ++){
			
			double angleRad =	GeometricOperation.getAngleBetweenVectors( originalPoints.get(i).getX(), originalPoints.get(i).getY() ,
					originalPoints.get(0).getX() , originalPoints.get(0).getY() ,
					originalPoints.get(halfSize).getX() , originalPoints.get(halfSize).getY()  );

			/* convers�o para inteiro necess�ria, pois NaN vira 0*/
			angles.add((int)Math.toDegrees( angleRad ) );
		}
		
		for (int i = halfSize ; i < size; i ++){
			
			double angleRad =	GeometricOperation.getAngleBetweenVectors( originalPoints.get(i).getX(), originalPoints.get(i).getY() ,
					originalPoints.get(halfSize).getX() , originalPoints.get(halfSize).getY() ,
					originalPoints.get(size -1).getX() , originalPoints.get(size -1).getY()  );

			/* convers�o para inteiro necess�ria, pois NaN vira 0*/
			angles.add((int)Math.toDegrees( angleRad ) );
		}
		
	}


	private void setRelevantAnglesIndex() {
		relevantPoints1.clear();			
		
		for ( int i = 1 ; i < originalPoints.size() - 1 ; i++){
			if ( angles.get(i) <= angles.get(i + 1) && i == 1){
				relevantPoints1.add(i);	
				
			}
			else if ( angles.get(i) <= angles.get(i + 1) && 
					angles.get(i) <= angles.get(i - 1)){
				relevantPoints1.add(i);
				
			}

		}
		

	}
	
	private void setRelevantAnglesIndex2() {
		relevantPoints2.clear();
		setRelevantAnglesIndex();
		relevantPoints2.addAll(relevantPoints1);
		int lastIndex = originalPoints.size() - 1;
		for ( int i = 1 ; i < lastIndex ; i++){
			if ( angles.get(i) >= angles.get(i + 1) && i == 1){
				if ( !relevantPoints2.contains( i + 1 ) && !relevantPoints2.contains( i - 1 ) && !relevantPoints2.contains( i  ) )
					relevantPoints2.add(i);					
			}
			else if ( angles.get(i) >= angles.get(i + 1) && 
					angles.get(i) >= angles.get(i - 1)){
				if ( !relevantPoints2.contains( i + 1 ) && !relevantPoints2.contains( i - 1 ) && !relevantPoints2.contains( i ) )
				relevantPoints2.add(i);
			}

		}
		Collections.sort( relevantPoints2 );
		
	}
	private void setRelevantAnglesIndex3(double tolerance, boolean highQuality) {
		relevantPoints1.clear();
		Point2D simplifiedLine[] = simplify.simplify(originalPoints.toArray(new Point2D.Double[originalPoints.size()]), tolerance, highQuality);
		int j = 1;
		for ( int i = 1 ; i < originalPoints.size() - 1 ; i++){
			if ( originalPoints.get(i) == simplifiedLine[j]){
				relevantPoints1.add(i);	
				j++;
				
			}

		}
		//System.out.println("Tolerance: " +tolerance+  " NumPTsOriginal: "+ originalPoints.size()+ " NumPTRelevant: " + relevantPoints1.size());

	}
	


	private ArrayList<Point2D> crossOver(ArrayList<Point2D> schematicPoints1,
			ArrayList<Point2D> schematicPoints2) {
		
		PointsPolar polarSchematicPointsFinal = new PointsPolar();
		
		PointsPolar polarSchematicPoints1 = new PointsPolar();
		polarSchematicPoints1  = GeometricOperation.toPolar(schematicPoints1);
		
		PointsPolar polarSchematicPoints2 = new PointsPolar();
		polarSchematicPoints2  = GeometricOperation.toPolar(schematicPoints2);
		
		int cutPoint1 = (int) ( schematicPoints1.size()*Math.random() );
		int cutPoint2 = (int) ( schematicPoints1.size()*Math.random() );
		
		
		
		/* cutPoint1 deve ser menor ou igual que cutPoint2*/
		if(cutPoint1 > cutPoint2 ){
			int aux = cutPoint1;
			cutPoint1 = cutPoint2;
			cutPoint2 = aux;
		}
		/* cross over caso cutPoint 1 == 0*/
		if (cutPoint1 == 0){
			while ( cutPoint2 == 0 )
				cutPoint2 = (int) ( schematicPoints1.size()*Math.random() );
			
			polarSchematicPointsFinal.setOrigem(schematicPoints1.get(0));
			
			for(int i = 0; i < cutPoint2; i ++ )
				polarSchematicPointsFinal.add( polarSchematicPoints1.getPoints().get( i ) );
			
			for(int i = cutPoint2; i < schematicPoints1.size() - 1; i ++)
				polarSchematicPointsFinal.add( polarSchematicPoints2.getPoints().get( i  ) );
		}
		/* cross over caso cutPoint 1 > 0*/
		else{
			polarSchematicPointsFinal.setOrigem(schematicPoints2.get(0));
			
			for(int i = 0; i < cutPoint1; i ++ )
				polarSchematicPointsFinal.add( polarSchematicPoints2.getPoints().get( i ) );
			
			for(int i = cutPoint1; i < cutPoint2; i ++)
				polarSchematicPointsFinal.add( polarSchematicPoints1.getPoints().get( i  ) );
			
			for(int i = cutPoint2; i < schematicPoints1.size() - 1; i ++)
				polarSchematicPointsFinal.add( polarSchematicPoints2.getPoints().get( i  ) );
			
		}
		
		
		return GeometricOperation.toCartezian( polarSchematicPointsFinal );
	}

	private ArrayList<Point2D> crossOver(ArrayList<Point2D> schematicPoints1,
			ArrayList<Point2D> schematicPoints2, int cutPoint1, int cutPoint2) {
		
		PointsPolar polarSchematicPointsFinal = new PointsPolar();
		
		PointsPolar polarSchematicPoints1 = new PointsPolar();
		polarSchematicPoints1  = GeometricOperation.toPolar(schematicPoints1);
		
		PointsPolar polarSchematicPoints2 = new PointsPolar();
		polarSchematicPoints2  = GeometricOperation.toPolar(schematicPoints2);
		

		/* cross over caso cutPoint 1 == 0*/
		if (cutPoint1 == 0){
			
			polarSchematicPointsFinal.setOrigem(schematicPoints1.get(0));
			
			for(int i = 0; i < cutPoint2; i ++ )
				polarSchematicPointsFinal.add( polarSchematicPoints1.getPoints().get( i ) );
			
			for(int i = cutPoint2; i < schematicPoints1.size() - 1; i ++)
				polarSchematicPointsFinal.add( polarSchematicPoints2.getPoints().get( i  ) );
		}
		/* cross over caso cutPoint 1 > 0*/
		else{
			polarSchematicPointsFinal.setOrigem(schematicPoints2.get(0));
			
			for(int i = 0; i < cutPoint1; i ++ )
				polarSchematicPointsFinal.add( polarSchematicPoints2.getPoints().get( i ) );
			
			for(int i = cutPoint1; i < cutPoint2; i ++)
				polarSchematicPointsFinal.add( polarSchematicPoints1.getPoints().get( i  ) );
			
			for(int i = cutPoint2; i < schematicPoints1.size() - 1; i ++)
				polarSchematicPointsFinal.add( polarSchematicPoints2.getPoints().get( i  ) );
			
		}
		
		if( Math.random()*100 < mutationIndex )
			polarSchematicPointsFinal = mutateLine(polarSchematicPointsFinal);
		
		return GeometricOperation.toCartezian( polarSchematicPointsFinal );
	}

	private ArrayList<Point2D> mutateLine( ArrayList<Point2D> schematicPoints ){
		
		
		PointsPolar polarSchematicPoints1 = new PointsPolar();
		polarSchematicPoints1  = GeometricOperation.toPolar(schematicPoints);
		
		int mutationIndex = (int) ( (schematicPoints.size() - 1)*Math.random() );
		
		RelativPolarPoint newPoint = new RelativPolarPoint();
		newPoint = polarSchematicPoints1.getPoints().get( mutationIndex );
		
		double rCoeficiente = 15*(Math.random()) + 15;
		if( Math.random() < 0.5)
			rCoeficiente = -1*rCoeficiente;
		
		int thetaCoeficiente;
		if( Math.random() < 0.5)
			thetaCoeficiente = -1;
		else
			thetaCoeficiente = 1;
		
		//if (Math.random() - 0.5 > 0)
		newPoint.setR( newPoint.getR() + newPoint.getR()*rCoeficiente/100 );
		newPoint.setTheta( newPoint.getTheta() + thetaCoeficiente*Math.PI/4 );
		polarSchematicPoints1.getPoints().set( mutationIndex , newPoint);
		
		return GeometricOperation.toCartezian( polarSchematicPoints1 );
		
	}
	
	private PointsPolar mutateLine( PointsPolar polarSchematicPoints ){
		
		
		
		int mutationIndex = (int) ( (polarSchematicPoints.getPoints().size())*Math.random() );
		
		RelativPolarPoint newPoint = new RelativPolarPoint();
		newPoint = polarSchematicPoints.getPoints().get( mutationIndex );
		
		double rCoeficiente = 15*(Math.random()) + 15;
		if( Math.random() < 0.5)
			rCoeficiente = -1*rCoeficiente;
		
		int thetaCoeficiente;
		if( Math.random() < 0.5)
			thetaCoeficiente = -1;
		else
			thetaCoeficiente = 1;
		
		//if (Math.random() - 0.5 > 0)
		newPoint.setR( newPoint.getR() + newPoint.getR()*rCoeficiente/100 );
		newPoint.setTheta( newPoint.getTheta() + thetaCoeficiente*Math.PI/4 );
		polarSchematicPoints.getPoints().set( mutationIndex , newPoint);
		
		return  polarSchematicPoints ;
		
	}
	
	private double evalueteGene(  ArrayList<Point2D> schematicPoints ){
		
		/* Avaliar n�mero de mudan�as de dire��o*/
		PointsPolar polarSchematicPoints = new PointsPolar();
		polarSchematicPoints  = GeometricOperation.toPolar(schematicPoints);
		
		double lastTheta = polarSchematicPoints.getPoints().get(0).getTheta();
		double dif = 0;
		double difSum = 0;
		for(RelativPolarPoint p: polarSchematicPoints.getPoints()){
			
			dif = Math.abs( p.getTheta() - lastTheta);
			if (dif > Math.PI)
				dif = 2*Math.PI - dif;
			
			if ( dif > (3*Math.PI/4))
				dif += 5;
			else if(dif > Math.PI/2 )
				dif += 3;
			else if (dif > Math.PI/4 )
				dif += 1;
			
			difSum += dif;
			lastTheta = p.getTheta();
			
				
			
		}
		
		double bendCoe = difSum / schematicPoints.size();
		//double bendCoe = schematicPoints.size() / numberOfBends + 1;
		
		/* Avaliar dist�ncia entre esquematico e original */
		// USE DESVIO PADR�O E COEFICIENTE DE VARIA��O */
		double totalDist = 0;

		for( int i = 0 ; i < schematicPoints.size() ; i++){
			//distance[i] = originalPoints.get(i).distance( schematicPoints.get(i));
			relevantPoints1.contains(i);
			totalDist += originalPoints.get(i).distance( schematicPoints.get(i));			
		}
		
		double mediaDist = totalDist / schematicPoints.size(); 
		

		//double finalCoe = ( (mediaDist/(13) + dp )/2) + bendCoe/1;
		
		
		//double finalCoe = ( (mediaDist/stringLength )/2 + dp) + (bendCoe)*angleCoeficient/100;
		double finalCoe = mediaDist/stringLength  + (bendCoe)*angleCoeficient/200;
		
		//System.out.println(" Length: " + stringLength +  " media: " +  mediaDist + " mediaDistN: " + mediaDist/stringLength + "  distFactor: " + mediaDist/stringLength +  "  bendFactor: " + bendCoe );
		//System.out.println("Final: " + finalCoe);
		

		
		return finalCoe ;
	}

	private void resetRandomIndex() {
		int aux = (int)(Math.random()*9) + 1;
		if(randomIndex1 != aux)
			randomIndex1 = aux;
		randomIndex2 = (int)(Math.random()*9) + 1;
		while(randomIndex1 == randomIndex2)
			randomIndex2 = (int)(Math.random()*9) + 1;
		
	}

	private void resetCutPoints() {
		cutPoint1 = (int) ( originalPoints.size()*Math.random() );
		cutPoint2 = (int) ( originalPoints.size()*Math.random() );


		/* cutPoint1 deve ser menor ou igual que cutPoint2*/
		if(cutPoint1 > cutPoint2 ){
			int aux = cutPoint1;
			cutPoint1 = cutPoint2;
			cutPoint2 = aux;
		}

		if (cutPoint1 == 0){
			while ( cutPoint2 == 0 )
				cutPoint2 = (int) ( originalPoints.size()*Math.random() );
		}
	}


	public void setRotation(double rotation) {
		this.rotation = rotation;
	}
	
	public void setAngleCoeficient(double angleCoeficient) {
		this.angleCoeficient = angleCoeficient;
	}
	
	public int getGenerationsCount() {
		return generationsCount;
	}

	public void setGenerationsCount(int generationsCount) {
		this.generationsCount = generationsCount;
	}

	public double getStringLength() {
		return stringLength;
	}

	public void setStringLength(double stringLenght) {
		this.stringLength = stringLenght;
	}

	public double getTrackDiagonal() {
		return trackDiagonal;
	}

	public void setTrackDiagonal(double trackDiagonal) {
		this.trackDiagonal = trackDiagonal;
	}

	public FisheyeTransform getFisheyeTransform() {
		return fisheyeTransform;
	}

	public void setFisheyeTransform(FisheyeTransform fisheyeTransform) {
		this.fisheyeTransform = fisheyeTransform;
	}

	
	

}
