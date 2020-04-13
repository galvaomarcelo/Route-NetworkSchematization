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





 
 



public class GeneticOperation2 { 

	private double rotation = 0;
	private int populationSize = 20;
	
	
	
	private Simplify<Point2D> simplify = new Simplify<Point2D>(new Point2D[0], point2DPointExtractor);
	
	private  ArrayList<Point2D> originalPoints;
	private PointsPolar polarOriginalPoints;
	
	private double stringLength = 0;
	private double trackDiagonal = 0; 

	


	
	private ArrayList<Gene> initialPopulation, population, populationAux;


	private int generationsCount;


	private double bendFactor = 40;
	private double distFactor = 1;
	


	public GeneticOperation2(double bendFactor, double distFactor) {
		this.bendFactor = bendFactor;
		this.distFactor = distFactor;
		
	}
	public GeneticOperation2() {
		this.bendFactor = 1;

		
	}
	
	private static PointExtractor<Point2D> point2DPointExtractor = new PointExtractor<Point2D>() {
	    public double getX(Point2D point) {
	        return point.getX();
	    }

	    public double getY(Point2D point) {
	        return point.getY();
	    }
	};


	public ArrayList<Point2D> schematize(ArrayList<Point2D> track) {
		
		
		/*Detect bounds of the track for setting tolerences to line simplification */
		originalPoints = new ArrayList<Point2D>(track);
		polarOriginalPoints =  new PointsPolar();
		polarOriginalPoints  = GeometricOperation.toPolar(originalPoints);
		
		stringLength = GeometricOperation.lengthOf(track);
		trackDiagonal = GeometricOperation.diagonalOf(track);
		
		initialPopulation = new ArrayList<Gene>() ;
		population = new ArrayList<Gene>() ;
		
		Gene gene = new Gene(GeometricOperation.toSimpleOctalinear2(track));
		initialPopulation.add( gene );
		/* inicializa genes inciais */
		for (int i = 1 ; i < populationSize ; i++){
			gene = new Gene(getPopulation1(i));
			initialPopulation.add( gene );
		}
		


		for (int i = 0 ; i < populationSize ; i++){
			initialPopulation.get(i).setValue( 
					evalueteGene2( originalPoints, initialPopulation.get(i).getSchematicPoints()) );
		}
 
		Collections.sort( initialPopulation );
		population = initialPopulation;
		
		
		
		int genCoe = 7;
		/* calcula o n�mero de gera��es por rodada*/
		generationsCount = 200;
		int size = originalPoints.size();
		if( size < 3 )
			generationsCount = 2*genCoe;
		else if( size < 6 )
			generationsCount = 5*genCoe;
		else if( size < 10 )
			generationsCount = 8*genCoe;
		else if( size < 15 )
			generationsCount = 15*genCoe;
		else if( size < 25 )
			generationsCount = 35*genCoe;
		else if( size < 40 )
			generationsCount = 50*genCoe;
		else 
			generationsCount = 100*genCoe;

		
		
		
		for (int i = 0 ; i < generationsCount; i++ )
			nextGeneration();
		

		//int j = 2;
		double bestValue = Double.MAX_VALUE;
		while ( population.get(0).getValue() < bestValue ){
			//toalGenerationsCount = toalGenerationsCount*j++;
			for (int i = 0 ; i < generationsCount; i++ )
				nextGeneration();
			bestValue = population.get(0).getValue();
		}


		//System.out.print(toalGenerationsCount + ", ");
		return population.get(0).getSchematicPoints();
		
		
	}
	
	public void schematize(Path path) {
		
		ArrayList<Point2D> lineString = new ArrayList<Point2D>();
		/* cria originalPoints2d */
		for(StreetNode s: path.getNodeList()){
			/*Se já existir posicao esquematica, utilizar. Senao utilizar posicao original*/
			if(s.getxGeom() == null)
				lineString.add( new Point2D.Double(s.getGeom().getX(), s.getGeom().getY() ) );
			//originalPoints.add(  fisheyeTransform.transform( new Point2D.Double(s.getGeom().getX(), s.getGeom().getY() ) ) ) ;
			else /*Em tese, somente o primeiro ponto da track deve possuir posicao esquematica*/ 
				lineString.add( new Point2D.Double(s.getxGeom().getX(), s.getxGeom().getY() ) );
		}

		lineString = schematize(lineString );
		
		GeometryFactory geometryFactory = new GeometryFactory( new PrecisionModel(PrecisionModel.FLOATING), 4326);
		PointsPolar bestSolutionPolarFormat = new PointsPolar();
		bestSolutionPolarFormat  = GeometricOperation.toPolar(lineString);

		for(int i = 0; i < path.getNodeList().size(); i++ ){
			if( i == 0 && path.getNodeList().get(i).getxGeom() != null ){
				path.getNodeList().get(i).setxGeom( path.getNodeList().get(i).getxGeom());
				
				/*Set bytes to record with edges are connected to this station */
				path.getNodeList().get(i).setEdges(setXStaionEdges(bestSolutionPolarFormat.getPoints().get(i).getTheta(), path.getNodeList().get(i).getEdges()));

			}	
			else{
				Coordinate coordSource = new Coordinate(lineString.get(i).getX(),
						lineString.get(i).getY());

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
		populationAux = new ArrayList<Gene>() ;
		for (int i = 0; i < populationSize; i++) {
			populationAux.add( makeGene(i) );
			
		}
		for (int i = 0 ; i < populationAux.size() ; i++){
			populationAux.get(i).setValue( 
					evalueteGene2( originalPoints, populationAux.get(i).getSchematicPoints()) );
		}
		//System.out.println("------------------");
		Collections.sort( populationAux );
		population = populationAux;	
		
		
	}
	
	private Gene makeGene(int index){
		Gene g = new Gene();
		int randomIndex1 = generateRandomIndex(populationSize);
		//System.out.println("Random test: " + randomIndex1);
		int randomIndex2 = generateRandomIndex(populationSize);
		while(randomIndex2 == randomIndex1)
			randomIndex2 = generateRandomIndex(populationSize);
		switch (index) {
		case 0:
			g.setSchematicPoints(population.get(0).getSchematicPoints());
			break;
		case 1:
			g.setSchematicPoints(mutateR(population.get(0).getSchematicPoints()));
			break;
		case 2:
			g.setSchematicPoints(mutateTheta(population.get(0).getSchematicPoints()));
			break;
		case 3:
			g.setSchematicPoints(
					crossOver(population.get(0).getSchematicPoints() , 
							population.get(randomIndex1).getSchematicPoints() ));
			break;	
		case 4:
			g.setSchematicPoints(
					crossOver(population.get(randomIndex1).getSchematicPoints() , 
							population.get(0).getSchematicPoints() ));
			break;	
		case 5:
			g.setSchematicPoints(mutateTheta(population.get(randomIndex1).getSchematicPoints()));
			break;	
		case 6:
			g.setSchematicPoints(mutateR(population.get(randomIndex1).getSchematicPoints()));
			break;	
			
		case 7:
			g.setSchematicPoints(
					crossOver(initialPopulation.get(randomIndex1).getSchematicPoints() , 
							population.get(randomIndex2).getSchematicPoints() ));
		case 8:
			g.setSchematicPoints(
					crossOver(initialPopulation.get(randomIndex1).getSchematicPoints() , 
							population.get(0).getSchematicPoints() ));	
		case 9:
			g.setSchematicPoints(
					crossOver(initialPopulation.get(randomIndex1).getSchematicPoints() , 
							initialPopulation.get(randomIndex2).getSchematicPoints() ));	
			break;	
		default:
			g.setSchematicPoints(
					crossOver(population.get(randomIndex2).getSchematicPoints() , 
							population.get(randomIndex1).getSchematicPoints() ));
			break;
		}
		return g;
		
	}
	
	
	
	
	private  ArrayList<Point2D> getPopulation1( int geneNumber ){
		
		ArrayList<Point2D> points = new ArrayList<Point2D>();
		Point2D.Double pt = new Point2D.Double();
		float g = (float)(geneNumber) /populationSize;
		//double toleranceIndex = trackDiagonal*(Math.pow((g)/20, 3))	/(80/(g)) ;
		
		double toleranceIndex = (trackDiagonal/4)*(Math.pow(g, 6));
		
		
		/*Cases 1 and 2 move original point!!!!! Case 1 was reapeated*/
		
		ArrayList<Integer> relevantPoints1 = setRelevantAnglesIndex3(toleranceIndex ,true);
			/* trata caso inicial */
			points.add( originalPoints.get(0) );
			
			for(Integer i: relevantPoints1){
				int lastIndexInPoints = points.size() - 1;
				pt = GeometricOperation.getNewSchematicPosition2(points.get( lastIndexInPoints  ), originalPoints.get( i - 1),
						originalPoints.get( i ), rotation);	
				points.add( pt );
				points = fillLine( points , i - lastIndexInPoints   , lastIndexInPoints );
			}
			
			/* trata caso final */
			pt = GeometricOperation.getNewSchematicPosition2(points.get( points.size() - 1 ),
					originalPoints.get( originalPoints.size() - 2 ), originalPoints.get( originalPoints.size() - 1 ) , rotation);
			points.add( pt );
			
			if(relevantPoints1.isEmpty())
				points = fillLine( points , originalPoints.size() - 1 ,
						0 );
			else
				points = fillLine( points , originalPoints.size() - 1 - relevantPoints1.get( relevantPoints1.size() - 1  ),
						relevantPoints1.get( relevantPoints1.size() - 1  ) );
			

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

	





	private ArrayList<Integer> setRelevantAnglesIndex3(double tolerance, boolean highQuality) {
		ArrayList<Integer> relevantPoints = new ArrayList<Integer>();
		Point2D simplifiedLine[] = simplify.simplify(originalPoints.toArray(new Point2D.Double[originalPoints.size()]), tolerance, highQuality);
		int j = 1;
		for ( int i = 1 ; i < originalPoints.size() - 1 ; i++){
			if ( originalPoints.get(i) == simplifiedLine[j]){
				relevantPoints.add(i);	
				j++;
				
			}

		}
		//System.out.println("Tolerance: " +tolerance+  " NumPTsOriginal: "+ originalPoints.size()+ "NumPTRelevant: " + relevantPoints.size());
		return relevantPoints;

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


	
	private ArrayList<Point2D> mutateR( ArrayList<Point2D> schematicPoints ){
		
		PointsPolar polarSchematicPoints = new PointsPolar();
		polarSchematicPoints  = GeometricOperation.toPolar(schematicPoints);
		int mutationIndex = (int) ( (polarSchematicPoints.getPoints().size())*Math.random() );
		
		RelativPolarPoint newPoint = new RelativPolarPoint();
		newPoint = polarSchematicPoints.getPoints().get( mutationIndex );
		
		double rCoeficiente = 0.2*(Math.random()) ;
		if( Math.random() < 0.5)
			rCoeficiente = -1*rCoeficiente;	
		
		//if (Math.random() - 0.5 > 0)
		newPoint.setR( newPoint.getR()*(1.0 + rCoeficiente ) );
		polarSchematicPoints.getPoints().set( mutationIndex , newPoint);
		
		return  GeometricOperation.toCartezian( polarSchematicPoints ); 
		
	}
	
	private ArrayList<Point2D> mutateTheta( ArrayList<Point2D> schematicPoints ){
		
		
		PointsPolar polarSchematicPoints = new PointsPolar();
		polarSchematicPoints  = GeometricOperation.toPolar(schematicPoints);
		
		int mutationIndex = (int) ( (polarSchematicPoints.getPoints().size())*Math.random() );

		double original = polarOriginalPoints.getPoints().get(mutationIndex).getTheta();
		double current = polarSchematicPoints.getPoints().get(mutationIndex).getTheta();
		double opt1 = (current + Math.PI/4) % (2*Math.PI);
		double opt2 = (current - Math.PI/4) % (2*Math.PI);
		double dif1 =  Math.abs(original - opt1) < Math.PI ? Math.abs(original - opt1): 2*Math.PI - Math.abs(original - opt1);
		double dif2 =  Math.abs(original - opt2) < Math.PI  ? Math.abs(original - opt2): 2*Math.PI - Math.abs(original - opt2);
			
		
		double best = dif1 < dif2 ? opt1 : opt2;
	

		
		polarSchematicPoints.getPoints().get( mutationIndex ).setTheta(best);
		
	
		return GeometricOperation.toCartezian( polarSchematicPoints );
		
	}
	
	private double evalueteGene(   ArrayList<Point2D> originalPoints, ArrayList<Point2D> schematicPoints ){
		
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
		Double distance[] = new Double[schematicPoints.size()];
		for( int i = 0 ; i < schematicPoints.size() ; i++){
			distance[i] = originalPoints.get(i).distance( schematicPoints.get(i));
			//WHAT A FUCK IS THIS? relevantPoints1.contains(i);
			totalDist += originalPoints.get(i).distance( schematicPoints.get(i));			
		}
		
		double mediaDist = totalDist / schematicPoints.size(); 
		

		//double finalCoe = ( (mediaDist/(13) + dp )/2) + bendCoe/1;
		double dp = 0;
		for (int i = 0; i < schematicPoints.size() ; i++){
			dp +=(distance[i] - mediaDist)*(distance[i] - mediaDist);
		}
		dp = Math.sqrt( dp/(schematicPoints.size() ) );
		
		//double finalCoe = ( (mediaDist/stringLength )/2 + dp) + (bendCoe)*angleCoeficient/100;
		double finalCoe = mediaDist/stringLength  + (bendCoe)*bendFactor/200;
		//double finalCoe = ( (mediaDist/(13) + dp )/2) + (bendCoe/4)*angleCoeficient;
		
	//	System.out.println(" Length: " + stringLength +  " media: " +  mediaDist + " mediaDistN: " + mediaDist/stringLength + "  distCoe: " + mediaDist/stringLength +  "  bendCoe: " + bendCoe );
		
		

		
		return finalCoe ;
	}
	
	private double evalueteGene2(  ArrayList<Point2D> originalPoints, ArrayList<Point2D> schematicPoints ){
		
		/* Avaliar n�mero de mudan�as de dire��o*/
		PointsPolar polarSchematicPoints = new PointsPolar();
		polarSchematicPoints  = GeometricOperation.toPolar(schematicPoints);
		
		double lastTheta = polarSchematicPoints.getPoints().get(0).getTheta();
		double dif = 0;
		double bendCost = 0;
		for(RelativPolarPoint p: polarSchematicPoints.getPoints()){
			
			dif = Math.abs( p.getTheta() - lastTheta);
			if (dif + 0.02 >= Math.PI)
				dif = 2*Math.PI - dif;
			
			if ( dif + 0.02 >= (3*Math.PI/4))
				bendCost += 3;
			else if(dif + 0.02 >= Math.PI/2 )
				bendCost += 2;
			else if (dif + 0.02 >= Math.PI/4 )
				bendCost += 1;
			
			
			lastTheta = p.getTheta();
			
			
		}

		/*DISTANCE*/
		Double dx[] = new Double[schematicPoints.size()];
		Double dy[] = new Double[schematicPoints.size()];
		Double sumDX = 0.0;
		Double sumDY = 0.0;
		Double distance[] = new Double[schematicPoints.size()];
		for( int i = 0 ; i < schematicPoints.size() ; i++){
			distance[i] = originalPoints.get(i).distance( schematicPoints.get(i));
			dx[i] = Math.abs(schematicPoints.get(i).getX() - originalPoints.get(i).getX());
			sumDX += dx[i];
			dy[i] = Math.abs(schematicPoints.get(i).getY() - originalPoints.get(i).getY());
			sumDY += dy[i];	
		}
		//System.out.println("sumDx =  " + sumDX + " sumDy =  " + sumDY  + " Total =  " + (sumDX + sumDY));
		double distCost = (sumDX + sumDY)/(stringLength*schematicPoints.size());
		

		double finalCoe = 1000*distFactor*distCost + bendFactor*bendCost;
		
		
		return finalCoe ;
	}
	
	
public double evalueteGenePublic(  ArrayList<Point2D> originalPoints, ArrayList<Point2D> schematicPoints ){
		
	/* Avaliar n�mero de mudan�as de dire��o*/
	PointsPolar polarSchematicPoints = new PointsPolar();
	polarSchematicPoints  = GeometricOperation.toPolar(schematicPoints);
	
	double lastTheta = polarSchematicPoints.getPoints().get(0).getTheta();
	double dif = 0;
	double bendCost = 0;
	for(RelativPolarPoint p: polarSchematicPoints.getPoints()){
		
		dif = Math.abs( p.getTheta() - lastTheta);
		if (dif + 0.02 >= Math.PI)
			dif = 2*Math.PI - dif;
		
		if ( dif + 0.02 >= (3*Math.PI/4))
			bendCost += 3;
		else if(dif + 0.02 >= Math.PI/2 )
			bendCost += 2;
		else if (dif + 0.02 >= Math.PI/4 )
			bendCost += 1;
		
		
		lastTheta = p.getTheta();
		
		
	}

	/*DISTANCE*/
	Double dx[] = new Double[schematicPoints.size()];
	Double dy[] = new Double[schematicPoints.size()];
	Double sumDX = 0.0;
	Double sumDY = 0.0;
	Double distance[] = new Double[schematicPoints.size()];
	for( int i = 0 ; i < schematicPoints.size() ; i++){
		distance[i] = originalPoints.get(i).distance( schematicPoints.get(i));
		dx[i] = Math.abs(schematicPoints.get(i).getX() - originalPoints.get(i).getX());
		sumDX += dx[i];
		dy[i] = Math.abs(schematicPoints.get(i).getY() - originalPoints.get(i).getY());
		sumDY += dy[i];	
	}
	//System.out.println("sumDx =  " + sumDX + " sumDy =  " + sumDY  + " Total =  " + (sumDX + sumDY));
	double distCost = (sumDX + sumDY)/(stringLength*schematicPoints.size());
	

	double finalCoe = 1000*distFactor*distCost + bendFactor*bendCost;
		
	/*System.out.println(" DistCost: " + distCost + " distF: " + distFactor +  "  DistCostNormal: " + 1000*distFactor*distCost +
			" bendCost: " +  bendCost + " bendF: " + bendFactor + "  benCostNormal: " + bendFactor*bendCost + 
			"  FINAL: " + finalCoe );*/
		
		

		
		return finalCoe ;
	}
	
	

	private int generateRandomIndex(int size) {
		
		return (int)(size*Math.pow(Math.random(),1.3));
	
	}




	public void setRotation(double rotation) {
		this.rotation = rotation;
	}
	

	public double getBendFactor() {
		return bendFactor;
	}
	public void setBendFactor(double bendFactor) {
		this.bendFactor = bendFactor;
	}
	public double getDistFactor() {
		return distFactor;
	}
	public void setDistFactor(double distFactor) {
		this.distFactor = distFactor;
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

	
	

}
