package com.wayto.operator;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Map;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.apache.commons.math3.geometry.spherical.twod.Edge;
import org.geotools.geometry.jts.Geometries;
import org.locationtech.jts.algorithm.*;
import com.wayto.model.Path;
import com.wayto.model.Route;
import com.wayto.model.RouteEdge;
import com.wayto.model.StreetEdge;
import com.wayto.model.StreetNetworkTopological;
import com.wayto.model.StreetNode;
import com.wayto.model.topo.PolygonalTopo;
import com.wayto.resultReport.PathReport;
import com.wayto.resultReport.ResultReport;
import com.wayto.resultReport.SoftConstraintValues;

import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

public class OptimizerOperator2 {
	

	
	
	
	public static ArrayList<Point2D> routeOptimizerLazyTopologyCheck(Map<Integer,StreetNode> streetNodeMap, Map<Integer,ArrayList<StreetNode>> circularOrderList, Route route, LineString rescaledRouteGeom,  
			double bendFactor, double crossingBendFactor, double dirFactor, double distFactor, double proportionDPFactor, double proportionAllPtsFactor, double rescaleFactor, int directionModel, 
			double adjVerticesMinDist,  double minNonAdjEdgeDist, double stubFixLength, double LMfixDist, double levelOfOrthogonality, double turnLevelOfOrthogonality, boolean checkTopology, int executionTimeLimit, ResultReport resultReport) throws IloException, Exception  {
			
			long start, end;
			start = System.currentTimeMillis(); 
			ArrayList<ArrayList<Integer>> edgePaarToCheck = new ArrayList<ArrayList<Integer>>();
			int executionsCount = 1;
			
			boolean violateTopology = false;
			violateTopology =  routeOptimizer3(streetNodeMap,  circularOrderList, route, rescaledRouteGeom,  
				bendFactor, crossingBendFactor, dirFactor, distFactor,  proportionDPFactor, proportionAllPtsFactor,  rescaleFactor,  directionModel, 
				 adjVerticesMinDist,   minNonAdjEdgeDist,  stubFixLength,  LMfixDist, levelOfOrthogonality, turnLevelOfOrthogonality, false,  executionTimeLimit,  resultReport, edgePaarToCheck);
		
			if(checkTopology)
				while(violateTopology) {
					System.out.println("Paar of edges that violates topology:");
					System.out.println(edgePaarToCheck);
					executionsCount++;
					violateTopology = routeOptimizer3(streetNodeMap,  circularOrderList, route, rescaledRouteGeom,  
							bendFactor, crossingBendFactor, dirFactor, distFactor,  proportionDPFactor, proportionAllPtsFactor,  rescaleFactor,  directionModel, 
							 adjVerticesMinDist,   minNonAdjEdgeDist,  stubFixLength,  LMfixDist, levelOfOrthogonality, turnLevelOfOrthogonality, true,  executionTimeLimit,  resultReport, edgePaarToCheck);
				}
			end = System.currentTimeMillis();
			resultReport.getRouteReport().setExecutionTime(end - start);
			resultReport.getRouteReport().setFixedExtraCrossings(edgePaarToCheck.size());
			resultReport.getRouteReport().setExecutions(executionsCount);
		return null;
	
	}
	

	public static boolean routeOptimizer3(Map<Integer,StreetNode> streetNodeMap, Map<Integer,ArrayList<StreetNode>> circularOrderList, Route route, LineString rescaledRouteGeom,  
			double bendFactor, double crossingBendFactor, double dirFactor, double distFactor, double proportionDPFactor, double proportionAllPtsFactor, double rescaleFactor, int directionModel, 
			double adjVerticesMinDist,  double minNonAdjEdgeDist, double stubFixLength, double LMfixDist, double levelOfOrthogonality, double turnLevelOfOrthogonality, boolean checkTopology, int executionTimeLimit, ResultReport resultReport,  ArrayList<ArrayList<Integer>> edgePaarToCheck) throws IloException, Exception  {
		
		long start, end;
		start = System.currentTimeMillis(); 
		boolean topologyViolation = false;
		
		ArrayList<Point2D> mipLineString = new ArrayList<Point2D>();
		ArrayList<Integer> routeNodeIdList = new ArrayList<Integer>();
		ArrayList<Integer> intersectionNodeIdList = new ArrayList<Integer>();
		ArrayList<Integer> adjacentNodeIdList = new ArrayList<Integer>();
		ArrayList<Integer> allNodeIdList = new ArrayList<Integer>();
		
		ArrayList<Integer[]> edgeList = new ArrayList<Integer[]>();

		ArrayList<Point2D> rescaledRouteNodePositions = GeoConvertionsOperations.JTSGeometryToJavaD2(rescaledRouteGeom);
//		ArrayList<Point2D> rescaledRouteNodePositions = new ArrayList<Point2D>();
//		if(rescaleFactor > 5)
//			rescaledRouteNodePositions = GeoConvertionsOperations.JTSGeometryToJavaD2(GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getDPindex(), rescaleFactor/100));
//		else
//			rescaledRouteNodePositions = route.getRoutePath().asJava2DList(1);
		
		
		ArrayList<Point2D> rescaledRelevantRoutePoints = new ArrayList<Point2D>();
		ArrayList<Point2D> proportionKeepingRescaledRouteDPPoints = new ArrayList<Point2D>();
		ArrayList<Integer> proportionKeepingRescaledRouteDPPointsId = new ArrayList<Integer>();
		
		/**
		 * Reads route path create list of ids for nodes and edges
		 */
		Integer[] edge = new Integer[2];
		edge[0] = route.getRoutePath().getNodeList().get(0).getId();
		routeNodeIdList.add(route.getRoutePath().getNodeList().get(0).getId());
		rescaledRelevantRoutePoints.add(rescaledRouteNodePositions.get(0));

		double rescaledRouteLenght = GeometricOperation.length(rescaledRouteNodePositions);
		
		proportionKeepingRescaledRouteDPPoints.add(rescaledRouteNodePositions.get(0));
		proportionKeepingRescaledRouteDPPointsId.add(route.getRoutePath().getNodeList().get(0).getId());

		
		for(int i = 1 ; i < route.getRoutePath().getNodeList().size(); i++){
			if(route.getRoutePath().getNodeList().get(i).isRelevantRouteNode()){
				
				routeNodeIdList.add(route.getRoutePath().getNodeList().get(i).getId());
				edge[1] = route.getRoutePath().getNodeList().get(i).getId();
				edgeList.add(edge);
				if(i  < route.getRoutePath().getNodeList().size() -1){
					edge = new Integer[2];
					edge[0] = route.getRoutePath().getNodeList().get(i).getId();
				}
				rescaledRelevantRoutePoints.add(rescaledRouteNodePositions.get(i));
			}
			/***Keep proportion between all relevant nodes, or olny between some selected???*/
			if(route.getRoutePath().getNodeList().get(i).isRelevantRouteNode() &&
					( route.getRoutePath().getNodeList().get(i).isDecisionPoint() ||   route.getRoutePath().getNodeList().get(i).getTopoRelations().size() > 0 || i == route.getRoutePath().getNodeList().size()-1) ) {
			//if(route.getRoutePath().getNodeList().get(i).isRelevantRouteNode()){
				proportionKeepingRescaledRouteDPPoints.add(rescaledRouteNodePositions.get(i));
				proportionKeepingRescaledRouteDPPointsId.add(route.getRoutePath().getNodeList().get(i).getId());
			}
			/***with or without round about?*/
			if(route.getRoutePath().getNodeList().get(i).getDegree() > 2 
					&& !route.getRoutePath().getNodeList().get(i).isRoundAbout() ){
			//if(route.getRoutePath().getNodeList().get(i).getDegree() > 2 ){
				
				intersectionNodeIdList.add(route.getRoutePath().getNodeList().get(i).getId());
			}

			
		}
		int mR = edgeList.size();
		
		/**
		 * Reads adjacent nodes and add to list of id for nodes and edges
		 */
		for(Integer id: routeNodeIdList){
			for(StreetNode node: circularOrderList.get(id)){
//				if(!node.isRouteNode() && !adjacentNodeIdList.contains(node.getId()) 
//						&& !streetNodeMap.get(id).isRoundAboutStart() &&  !streetNodeMap.get(id).isRoundAboutEnd() ){
				if(!node.isRouteNode()  ){
					if(!adjacentNodeIdList.contains(node.getId())){
						adjacentNodeIdList.add(node.getId());
						edge = new Integer[2];
						edge[0] = id;
						edge[1] = node.getId();
						edgeList.add(edge);
					}
					else System.out.println("ATENTION ADJACENT EDGE with repeated node!! ");
					
				}
			}
		}
		for(Integer nodeId: routeNodeIdList )
			allNodeIdList.add(nodeId);
		for(Integer nodeId: adjacentNodeIdList )
			allNodeIdList.add(nodeId);
		
		//System.out.println("Intersetion size = " +  intersectionNodeIdList.size());
//		for(Integer i : intersectionNodeIdList){
//			System.out.println("Route relevant point n  " + routeNodeIdList.indexOf(i) + " of id "+i+"  is intersection and its degree is " + circularOrderList.get(i).size() );
//		}
		
		int n, nR, nA; /*n number of node, nR number of route nodes, nA number of adjacent nodes*/
		int m; /*number of edges*/
		int b; /*number of bends in a path*/
		
		nR = routeNodeIdList.size();
		nA = adjacentNodeIdList.size();

		n = allNodeIdList.size();
		m = edgeList.size();
		b = nR - 2;
		
		resultReport.getRouteReport().setNumberOfNodes(n);
		resultReport.getRouteReport().setNumberOfEdges(m);
		resultReport.getRouteReport().setNumberOfStubs(nA);
		resultReport.getRouteReport().setNubmerOfRouteNodes(nR);
		resultReport.getRouteReport().setScaleVariation(rescaleFactor/100);
		resultReport.getRouteReport().setPathLength(rescaledRouteLenght); 
		double maxAdjD = 0;
		for (Integer[] e: edgeList){
			double dist = streetNodeMap.get(e[0]).getProjectGeom().distance( streetNodeMap.get(e[1]).getProjectGeom() );
			if(dist > maxAdjD)
				maxAdjD=dist;
			
		}
		
		
		/*********COORDINATES CONSTRAINT*****************/
		
		
		
		
		IloCplex cplex = new IloCplex();
		double extendLimit = 0.2;
		
		OctlinearBoundingBox octBox = new OctlinearBoundingBox(GeoConvertionsOperations.JTSGeometryToJavaD2(rescaledRouteGeom), 3*minNonAdjEdgeDist, extendLimit,2);
		

		IloNumVar[] x  = cplex.numVarArray(n, octBox.getMinX(), octBox.getMaxX());
		IloNumVar[] y  = cplex.numVarArray(n,  octBox.getMinY(), octBox.getMaxY());
		IloNumVar[] z1  = cplex.numVarArray(n, octBox.getMinZ1(), octBox.getMaxZ1());
		IloNumVar[] z2  = cplex.numVarArray(n, octBox.getMinZ2(), octBox.getMaxZ2());
		
		
	
		IloNumExpr[] z1Constraint = new IloNumExpr[n];
		IloNumExpr[] z2Constraint = new IloNumExpr[n];
		
		for (int i = 0; i < n; i++) {
			/*do we need to multiply by 2?*/
			z1Constraint[i] = cplex.prod(0.5,cplex.sum(x[i],y[i]));
			z2Constraint[i] = cplex.prod(0.5, cplex.diff(x[i],y[i]));
			
		}
		
		/*Add z constraint to model constranint*/ 
		for (int i = 0; i < n; i++) {
			cplex.addEq(z1[i], z1Constraint[i]);
			cplex.addEq(z2[i], z2Constraint[i]);
			
		}
		/*fix position of first vertex*/
		/*and last?*/
		
//		cplex.addEq(x[0], rescaledRelevantRoutePoints.get(0).getX());
//		cplex.addEq(y[0], rescaledRelevantRoutePoints.get(0).getY());
		
		//cplex.addEq(x[nR-1], streetNodeMap.get(allNodeIdList.get(nR-1)).getProjectGeom().getX());
		//cplex.addEq(y[nR-1], streetNodeMap.get(allNodeIdList.get(nR-1)).getProjectGeom().getY());
		
		/*********OCTALINEARITY CONSTRAINT*****************/
		
		/*sec[m][d][(pred,orig,succ)] defines the sector of the octilinear position an edges could lie
		 * m = number of edges
		 * d = direction of the edge 0 is going and 1 is back
		 * (pred,orig,succ) = 0 lies on the previous sector, 1 lies ont the original best sector, and 2 lies on the succecive sector*/ 
		/*orthogonal preference [0,..1] tells the preference for orthogolality 1 is max*/
		double orthogonalPreference = levelOfOrthogonality;
		int[][][] sec = new int[m][2][3];		

		for (int i = 0; i < m; i++) {
			Point pt1 =  streetNodeMap.get(edgeList.get(i)[0]).getProjectGeom();
			Point pt2 =  streetNodeMap.get(edgeList.get(i)[1]).getProjectGeom();
			
			sec[i][0][1] = GeometricOperation.sectorOfOrthogonalPreference(
					GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ),
					orthogonalPreference
					);
			sec[i][0][0] =  Math.floorMod(sec[i][0][1] - 1, 8);
			sec[i][0][2] =  Math.floorMod(sec[i][0][1] + 1, 8);
			
			
			sec[i][1][1] = Math.floorMod(sec[i][0][1] + 4,8);
			sec[i][1][0] = Math.floorMod(sec[i][0][0] + 4,8);
			sec[i][1][2] = Math.floorMod(sec[i][0][2] + 4,8);
		}
		/**
		 * *Boolean variable to restrict the orientation of the edge to orig = 1, succ = 2 and prd = 0*
		 */

		IloNumVar[][] alpha = new IloNumVar[m][];
		
		for (int i = 0; i < m; i++) {
			alpha[i] = cplex.boolVarArray(3);
			for(int j = 0; j < 3; j++)
				alpha[i][j].setName("OctAlphaEdge"+i+"ori"+j);
			
		}

		
		IloLinearNumExpr[] constraintEdgeOrientation = new IloLinearNumExpr[m];
		
	
		/***
		 * Force edeges on decision points alpha[i][1] = 1: original best orientation 
		 * */
		
		int numDPs = 0;
		for (int i = 0; i < m; i++) {
			
			/*If direrection model is best direction of edges incident in DP*/
			if(directionModel == 1 ){
				/*rotue edges starting on DP*/
				if(streetNodeMap.get(edgeList.get(i)[0]).isDecisionPoint() &&
						streetNodeMap.get(edgeList.get(i)[1]).isRouteNode() 
						){
					//System.out.println("DP starting in: " + streetNodeMap.get(edgeList.get(i)[0]).getId() + " end in: " + streetNodeMap.get(edgeList.get(i)[1]).getId() +  " best dir: " + sec[i][0][1] + " invert: " + sec[i][1][1]);
					cplex.addEq(alpha[i][0], 0);
					cplex.addEq(alpha[i][1], 1);
					cplex.addEq(alpha[i][2], 0);
						
				}
				/*route edges ending on DP*/
				if(streetNodeMap.get(edgeList.get(i)[1]).isDecisionPoint() &&
						streetNodeMap.get(edgeList.get(i)[0]).isRouteNode() ){
					numDPs++;
					//System.out.println("DP ending in: " + streetNodeMap.get(edgeList.get(i)[1]).getId() + " start in: " + streetNodeMap.get(edgeList.get(i)[0]).getId() +  " best dir: " + sec[i][0][1] + " invert: " + sec[i][1][1]);
	
					cplex.addEq(alpha[i][0], 0);
					cplex.addEq(alpha[i][1], 1);
					cplex.addEq(alpha[i][2], 0);
				
				}
			}
			constraintEdgeOrientation[i] = cplex.linearNumExpr();
			constraintEdgeOrientation[i].addTerm(1.0,alpha[i][0]);
			constraintEdgeOrientation[i].addTerm(1.0,alpha[i][1]);
			constraintEdgeOrientation[i].addTerm(1.0,alpha[i][2]);
			cplex.addEq(constraintEdgeOrientation[i], 1.0);
		}
		
		
		/**
		 * *Variable dir[i][j] defines the direction of of edege i (j means de drirection of the edege*
		 */

		IloNumVar[][] dir  = new IloNumVar[m][]; /*leave the last dimension to be defined with cplex*/
		orthogonalPreference = levelOfOrthogonality;
		for (int i = 0; i < m; i++) {
			dir[i] = cplex.intVarArray(2, 0, 7);/* array of size 2 because we need uv an vu*/
			if(directionModel == 1 &&  i < mR ){
				Point pt1 =  streetNodeMap.get(edgeList.get(i)[0]).getProjectGeom();
				Point pt2 =  streetNodeMap.get(edgeList.get(i)[1]).getProjectGeom();
				/*rotue edges starting on DP*/
				if(streetNodeMap.get(edgeList.get(i)[0]).isDecisionPoint() &&
						streetNodeMap.get(edgeList.get(i)[1]).isRouteNode() 
						){
//					System.out.println("DP starting in: " + streetNodeMap.get(edgeList.get(i)[0]).getId() + " end in: " + streetNodeMap.get(edgeList.get(i)[1]).getId() +  " best dir: " + sec[i][0][1] + " invert: " + sec[i][1][1]);
//					System.out.println("Anlge: " + Math.toDegrees(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
//									pt1.getX(), pt1.getY(),
//									pt2.getX(), pt2.getY() 					
//									)));
					cplex.addEq( GeometricOperation.sectorOfOrthogonalPreference(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(),pt2.getX(), pt2.getY()) , orthogonalPreference ) 
							, dir[i][0] );
						
				}
				/*route edges ending on DP*/
				if(streetNodeMap.get(edgeList.get(i)[1]).isDecisionPoint() &&
						streetNodeMap.get(edgeList.get(i)[0]).isRouteNode() ){
					
//					System.out.println("DP ending in: " + streetNodeMap.get(edgeList.get(i)[1]).getId() + " start in: " + streetNodeMap.get(edgeList.get(i)[0]).getId() +  " best dir: " + sec[i][0][1] + " invert: " + sec[i][1][1]);
//					System.out.println("Anlge: " + Math.toDegrees(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
//							pt1.getX(), pt1.getY(),
//							pt2.getX(), pt2.getY() 					
//							)));
					cplex.addEq( GeometricOperation.sectorOfOrthogonalPreference(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY()), orthogonalPreference ) 
							, dir[i][0] );
				
				}
			}
			
		}
		
		
		/*For each i ∈ {pred, orig, succ} we have the following set of constraints
		 dir(u, v)−seciu(v) ≤M(1−αi(u, v))
		−dir(u, v)+seciu(v) ≤M(1−αi(u, v))
		dir(v,u)−seciv(u) ≤M(1−αi(u, v))
		−dir(v,u)+seciv(u) ≤M(1−αi(u, v))
		
		∀{u, v} ∈ E,
		
		 *Here, if αi(u, v) = 0, the constraints in (4) are trivially fulfilled and do not influence the left-hand sides. 
		 *On the other hand, if αi(u, v) = 1, the four inequalities are equivalent to dir(u, v) = seciu(v) and
			dir(v,u) = seciv(u) as desired (equality
		 *
		 *
		 *
		 */
		double MDir = 8; /* |sec - dir| < 8*/

		for (int i = 0; i < m; i++) {
			for (int j = 0; j < 3; j++) {
				IloNumExpr rightSide = cplex.prod(MDir, cplex.diff(1, alpha[i][j]));
				/*natural direction of the edge d=0*/
				IloNumExpr leftSide1 = cplex.diff(dir[i][0], sec[i][0][j]);
				IloNumExpr leftSide2 = cplex.diff(sec[i][0][j],  dir[i][0]);
				/*counter direction of the edge d= 1*/
				IloNumExpr leftSide3 = cplex.diff(dir[i][1], sec[i][1][j]);
				IloNumExpr leftSide4 = cplex.diff(sec[i][1][j],  dir[i][1]);
				
				cplex.addLe(leftSide1, rightSide);
				cplex.addLe(leftSide2, rightSide);
				cplex.addLe(leftSide3, rightSide);
				cplex.addLe(leftSide4, rightSide);
			
			}
			
		
		}
		
		/***contraints to force the correct position of the vertices
		 * if sec original is 2 and alphaoriginal is true then forces x(u) and x(v) to equal
		 * and y(v) > y(u)
		 * 
		 * x(u)−x(v) ≤ M(1−αorig(u, v))
		  −x(u)+x(v) ≤ M(1−αorig(u, v)) 
		  −y(u)+y(v) ≥ −M(1−αorig(u, v))+Luv,
		 * 
		 * 
		 * */
		double MAdjVertice = 2*maxAdjD; /*  D max distance between two adjacent vertices vertices! it was MaxD*/

		/**
		 * dir(u, v) condition 1 condition 2

		 */
//		M = maxExtend*1.5; /*da pra diminuir*/
//		double distMin = maxExtend/500;
		
		for (int i = 0; i < m; i++) {
			
			int v1Index = allNodeIdList.indexOf(edgeList.get(i)[0]);
			int v2Index = allNodeIdList.indexOf(edgeList.get(i)[1]);
			/*the incidentedges have always v1 in route and v2 must the adjancent verts*/
			boolean isAdjEdge = false;
			double fixDist = adjVerticesMinDist;
			double edgeMinLength = adjVerticesMinDist;
			//double fixDist = rescaledRouteLenght/150;
			/**if it is adjs edge calculates fix edge lenght*/
			if(v2Index >= nR){
				isAdjEdge = true;
				if(streetNodeMap.get(edgeList.get(i)[1]).getIsPointLMNode()>0) {
					fixDist = LMfixDist;
				}
				else if(streetNodeMap.get(edgeList.get(i)[1]).getDegree() == 1) {
					fixDist = stubFixLength;
				}
				else
					//fixDist = stubFixLength;
					fixDist = Math.max(stubFixLength,streetNodeMap.get(edgeList.get(i)[0]).getProjectGeom().distance( streetNodeMap.get(edgeList.get(i)[1]).getProjectGeom() )/rescaleFactor);
			}	
			else
				isAdjEdge = false;
				/*v2 is one of the the adj. vertex*/
				
			
			if(i < mR && ((streetNodeMap.get(edgeList.get(i)[0]).isDecisionPoint() && 	streetNodeMap.get(edgeList.get(i)[1]).isRouteNode() )
					|| (streetNodeMap.get(edgeList.get(i)[1]).isDecisionPoint() && streetNodeMap.get(edgeList.get(i)[0]).isRouteNode()))
					){
				//edgeMinLength =  Math.min(LMfixDist,streetNodeMap.get(edgeList.get(i)[0]).getProjectGeom().distance( streetNodeMap.get(edgeList.get(i)[1]).getProjectGeom() )/rescaleFactor); 
				edgeMinLength =  Math.min(stubFixLength/4.0,rescaledRelevantRoutePoints.get(i).distance(rescaledRelevantRoutePoints.get(i+1))); 
		
				//rescaledRelevantRoutePoints.get(i).distance(rescaledRelevantRoutePoints.get(i+1);
			}
			else if(i < mR && (streetNodeMap.get(edgeList.get(i)[0]).getDegree() > 2 || streetNodeMap.get(edgeList.get(i)[1]).getDegree() > 2 )) {
				edgeMinLength =  Math.min( stubFixLength/4.0, rescaledRelevantRoutePoints.get(i).distance(rescaledRelevantRoutePoints.get(i+1))); 
				
			}
			
			for (int j = 0; j < 3; j++) {
				IloNumExpr rightSide, rightSide2, rightSide3, rightSide4, leftSide1, leftSide2, leftSide3;
				/*do i need to constraint edges for both directions(sec[m][1])?*/
				switch (sec[i][0][j]) {
				case 0:
					
					/**y(u) = y(v)
					x(v)−x(u) ≥ ?uv
					 This is reversed because y axes is reversed!! z2<>z1***/
					rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
					
					
					leftSide1 = cplex.diff(y[v1Index], y[v2Index]);
					leftSide2 = cplex.diff(y[v2Index],  y[v1Index]);
					
					rightSide2 = cplex.diff(edgeMinLength, rightSide);
					leftSide3 = cplex.diff(x[v2Index],  x[v1Index]);
					
					cplex.addLe(leftSide1, rightSide);
					cplex.addLe(leftSide2, rightSide);
					
					cplex.addGe(leftSide3, rightSide2);
					/*condition to fix size incident edges*/
					if(isAdjEdge){
						rightSide3 =cplex.diff(fixDist, rightSide);
						rightSide4 =cplex.sum(rightSide, fixDist);
						cplex.addGe(leftSide3, rightSide3);
						cplex.addLe(leftSide3, rightSide4);
						
					}	
				
					break;
				case 7:
					/**z1(u) = z1(v)
					z2(v)−z2(u) ≥ 2?uv
					 This is reversed because y axes is reversed!! z2<>z1**/
					rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
					leftSide1 = cplex.diff(z2[v1Index], z2[v2Index]);
					leftSide2 = cplex.diff(z2[v2Index],  z2[v1Index]);
					
					rightSide2 = cplex.diff(2*edgeMinLength, rightSide);
					leftSide3 = cplex.diff(z1[v2Index],  z1[v1Index]);
					
					cplex.addLe(leftSide1, rightSide);
					cplex.addLe(leftSide2, rightSide);
					
					cplex.addGe(leftSide3, rightSide2);
					
					/*condition to fix size incident edges*/
					if(isAdjEdge){
						rightSide3 =cplex.diff(fixDist, rightSide);
						rightSide4 =cplex.sum(rightSide, fixDist);
						cplex.addGe(leftSide3, rightSide3);
						cplex.addLe(leftSide3, rightSide4);
						
					}
					break;
					
				case 6:
					/**x(u) = x(v) 
					 * y(u)−y(v) ≥ ?uv
					 * */
					rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
					leftSide1 = cplex.diff(x[v1Index], x[v2Index]);
					leftSide2 = cplex.diff(x[v2Index],  x[v1Index]);
					
					rightSide2 = cplex.diff(edgeMinLength, rightSide);
					leftSide3 = cplex.diff(y[v2Index],  y[v1Index]);
					
					cplex.addLe(leftSide1, rightSide);
					cplex.addLe(leftSide2, rightSide);
					
					cplex.addGe(leftSide3, rightSide2);
					
					/*condition to fix size incident edges*/
					if(isAdjEdge){
						rightSide3 =cplex.diff(fixDist, rightSide);
						rightSide4 =cplex.sum(rightSide, fixDist);
						cplex.addGe(leftSide3, rightSide3);
						cplex.addLe(leftSide3, rightSide4);
						
					}

					break;
					
				case 5:
					/**z2(u) = z2(v) 
					 * z1(u)−z1(v) ≥ 2?uv
					 *  This is reversed because y axes is reversed!! z2<>z1
					 * */
					rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
					leftSide1 = cplex.diff(z1[v1Index], z1[v2Index]);
					leftSide2 = cplex.diff(z1[v2Index],  z1[v1Index]);
					
					rightSide2 = cplex.diff(2*edgeMinLength, rightSide);
					leftSide3 = cplex.diff(z2[v1Index],  z2[v2Index]);
					
					cplex.addLe(leftSide1, rightSide);
					cplex.addLe(leftSide2, rightSide);
					
					cplex.addGe(leftSide3, rightSide2);
					
					/*condition to fix size incident edges*/
					if(isAdjEdge){
						rightSide3 =cplex.diff(fixDist, rightSide);
						rightSide4 =cplex.sum(rightSide, fixDist);
						cplex.addGe(leftSide3, rightSide3);
						cplex.addLe(leftSide3, rightSide4);
						
					}
					break;	
				case 4:
					/**y(u) = y(v) 
					 * x(u)−x(v) ≥ ?uv
					 * */
					rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
					leftSide1 = cplex.diff(y[v1Index], y[v2Index]);
					leftSide2 = cplex.diff(y[v2Index],  y[v1Index]);
					
					rightSide2 = cplex.diff(edgeMinLength, rightSide);
					leftSide3 = cplex.diff(x[v1Index],  x[v2Index]);
					
					cplex.addLe(leftSide1, rightSide);
					cplex.addLe(leftSide2, rightSide);
					
					cplex.addGe(leftSide3, rightSide2);
					
					/*condition to fix size incident edges*/
					if(isAdjEdge){
						rightSide3 =cplex.diff(fixDist, rightSide);
						rightSide4 =cplex.sum(rightSide, fixDist);
						cplex.addGe(leftSide3, rightSide3);
						cplex.addLe(leftSide3, rightSide4);
						
					}
					break;
				case 3:
					/**z1(u) = z1(v) 
					 * z2(u)−z2(v) ≥ 2?uv
					 This is reversed because y axes is reversed!! z2<>z1
					 * */
					rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
					leftSide1 = cplex.diff(z2[v1Index], z2[v2Index]);
					leftSide2 = cplex.diff(z2[v2Index],  z2[v1Index]);
					
					rightSide2 = cplex.diff(2*edgeMinLength, rightSide);
					leftSide3 = cplex.diff(z1[v1Index],  z1[v2Index]);
					
					cplex.addLe(leftSide1, rightSide);
					cplex.addLe(leftSide2, rightSide);
					
					cplex.addGe(leftSide3, rightSide2);
					
					/*condition to fix size incident edges*/
					if(isAdjEdge){
						rightSide3 =cplex.diff(fixDist, rightSide);
						rightSide4 =cplex.sum(rightSide, fixDist);
						cplex.addGe(leftSide3, rightSide3);
						cplex.addLe(leftSide3, rightSide4);
						
					}
					break;	
				case 2:
					/**x(u) = x(v) 
					 * y(v)−y(u) ≥ ?uv
					 * */
					rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
					leftSide1 = cplex.diff(x[v1Index], x[v2Index]);
					leftSide2 = cplex.diff(x[v2Index],  x[v1Index]);
					
					rightSide2 = cplex.diff(edgeMinLength, rightSide);
					leftSide3 = cplex.diff(y[v1Index],  y[v2Index]);
					
					cplex.addLe(leftSide1, rightSide);
					cplex.addLe(leftSide2, rightSide);
					
					cplex.addGe(leftSide3, rightSide2);
					
					/*condition to fix size incident edges*/
					if(isAdjEdge){
						rightSide3 =cplex.diff(fixDist, rightSide);
						rightSide4 =cplex.sum(rightSide, fixDist);
						cplex.addGe(leftSide3, rightSide3);
						cplex.addLe(leftSide3, rightSide4);
						
					}

					break;	
				case 1:
					/**z2(u) = z2(v)
					 * z1(v)−z1(u) ≥ 2?uv
					 *  This is reversed because y axes is reversed!! z2<>z1
					 * */
					rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
					leftSide1 = cplex.diff(z1[v1Index], z1[v2Index]);
					leftSide2 = cplex.diff(z1[v2Index],  z1[v1Index]);
					
					rightSide2 = cplex.diff(2*edgeMinLength, rightSide);
					leftSide3 = cplex.diff(z2[v2Index],  z2[v1Index]);
					
					cplex.addLe(leftSide1, rightSide);
					cplex.addLe(leftSide2, rightSide);
					
					cplex.addGe(leftSide3, rightSide2);
					
					/*condition to fix size incident edges*/
					if(isAdjEdge){
						rightSide3 =cplex.diff(fixDist, rightSide);
						rightSide4 =cplex.sum(rightSide, fixDist);
						cplex.addGe(leftSide3, rightSide3);
						cplex.addLe(leftSide3, rightSide4);
						
					}
					break;	
				default:
					break;
				}
				
			}
			
		}
		

		
		
		/***BEND DIFF (TURN ANGLE _ USED FOR DIRECTION MODEL AND CIRCULAR ORDER)***/
		
		double MBend = 15; /*diference between 2 dir */
		/* bendDir[1] = dir[0] - dir[1]* bends angles (-7 to 7)*/
		IloNumVar[] bendDiff = cplex.intVarArray(b, -7, 7);

		for (int i = 0; i < b; i++) {
			//routeNodeIdList.get(i+1)
			cplex.addEq(bendDiff[i], cplex.diff(dir[i][0], dir[i + 1][0]));
		}
		
		
//		/***FORCE CORRECT CHANGE OF DIRECTION ONF DECISION POINTS***/
		if(directionModel == 2 || directionModel == 3){
			IloNumVar[][] epsilon = new IloNumVar[b][];

			for (int i = 0; i < b; i++) {
				epsilon[i] = cplex.boolVarArray(2);
				epsilon[i][0].setName("epsiloDPDIR0");
				epsilon[i][1].setName("epsiloDPDIR1");
			}

			IloLinearNumExpr[] constraintBenDirectionDPBooleans = new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				constraintBenDirectionDPBooleans[i] = cplex.linearNumExpr();
				constraintBenDirectionDPBooleans[i].addTerm(1.0,epsilon[i][0]);
				constraintBenDirectionDPBooleans[i].addTerm(1.0,epsilon[i][1]);
				cplex.addEq(constraintBenDirectionDPBooleans[i], 1.0);
			}



			for (int i = 0; i < b; i++) {
				cplex.addGe(bendDiff[i], cplex.diff(0 , cplex.prod(MBend, epsilon[i][0])));
				cplex.addLe(bendDiff[i], cplex.diff(cplex.prod(MBend, epsilon[i][1]),  1));

			}
			/*BendDir is turn direction
			 * 0 => 0-straight , 1=>315-veer right, 2=> 270-right, 3=>225sharp right, 
			 * 4=>180backturn, 5=>135-sharp left, 6=> 90left, 7=>45veer left
			 * */
			IloNumVar[] bendDir = cplex.intVarArray(b, 0, 7);
			IloLinearNumExpr[] bendDirConstraint =  new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				bendDirConstraint[i] = cplex.linearNumExpr();
				bendDirConstraint[i].addTerm(1, bendDiff[i]);
				bendDirConstraint[i].addTerm(8, epsilon[i][0]);
				cplex.addEq(bendDir[i], bendDirConstraint[i] );		
				//cplex.addLe(bendDir[i], bendCost[i]);
				//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));

			}

			int[] origTurnDir = new int[b];
			int dpindex = 0;
			for(int i = 0; i < b; i++){ 
				Point pt1 =  streetNodeMap.get(routeNodeIdList.get(i)).getProjectGeom();
				Point pt2 =  streetNodeMap.get(routeNodeIdList.get(i + 1)).getProjectGeom();
				Point pt3 =  streetNodeMap.get(routeNodeIdList.get(i + 2)).getProjectGeom();
				
				/*Just to check algles at decision points*/
//				if(streetNodeMap.get(routeNodeIdList.get(i + 1)).isDecisionPoint()){
//					System.out.println("DP: " + dpindex++);
//					System.out.println("in e dge: "  + Math.toDegrees(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() )));
//					System.out.println("out edge: "  + Math.toDegrees(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() )));
//					System.out.println("Turn angle: " + Math.toDegrees(GeometricOperation.getAngleBetweenVectors(pt2.getX(), pt2.getY(), pt1.getX(), pt1.getY(), pt3.getX(), pt3.getY()) ));
//					System.out.println("Best zone Traditional " + GeometricOperation.turnDirectionOf(
//							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
//							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
//							0));
//					System.out.println("Best zone KP " + GeometricOperation.turnDirectionOf(
//							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
//							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
//							2));
//					
//					
//				}
				
				if(directionModel == 2){
					origTurnDir[i] = GeometricOperation.turnDirectionSectorOfOrthogonalPreference(
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
							turnLevelOfOrthogonality);
				}
				else if(directionModel == 3){
					origTurnDir[i] = GeometricOperation.turnDirectionOf(
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
							1);
				}

			}


			for (int i = 0; i < b; i++) {
				if(streetNodeMap.get(routeNodeIdList.get(i + 1)).isDecisionPoint()){
					cplex.addEq(bendDir[i] ,origTurnDir[i] );
					
					
				}
			}

		}
		
		
		/***CIRCULAR ORDER CONSTRAINTS********/
		double MCircularOrder = 17; /*Diference between 2 dir */
		int numInterPoints = intersectionNodeIdList.size();
		IloNumVar[][] beta = new IloNumVar[numInterPoints][];
		IloLinearNumExpr[] constraintCircularOrder = new IloLinearNumExpr[numInterPoints];
		for (int i = 0; i < numInterPoints; i++) {
			
			int degree = circularOrderList.get(intersectionNodeIdList.get(i)).size();
			
			beta[i] = cplex.boolVarArray(degree);
			for(int j = 0 ; j < degree; j++)
				beta[i][j].setName("BetaIntersect"+i+"deg"+j);
			constraintCircularOrder[i] = cplex.linearNumExpr();
			
			/**only one of the beta[i] for each v incident holds 1**/
			for(int j = 0; j < degree; j ++ )			
				constraintCircularOrder[i].addTerm(1, beta[i][j]);
			cplex.addEq(constraintCircularOrder[i], 1.0,  "Intersec"+i+"BetaCircOrder");	
			
		}
		
		for (int i = 0; i < numInterPoints; i++) {
			

			int vId = intersectionNodeIdList.get(i);
//			System.out.println();
//			System.out.println("Route relevant point n  " + routeNodeIdList.indexOf(vId) + " of id "+vId+"  is intersection and its degree is " + circularOrderList.get(vId).size() );
//			System.out.println("Interpoint"+i+" Intersection Id: " + vId);
			IloNumExpr rightSide, leftSide;
			
			for(int j = 0; j < beta[i].length ; j ++ ){
				int indexDirVU1 = 0, indexDirVU2 = 0;
//				System.out.println("--Edge " + j + " and " + (j+1));
				int u1Id, u2Id;
				u1Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j).getId();
//				System.out.println("u1ID="+u1Id); 
				/*se for vertive um orden circular seleciona u2 como o primeiro da lista*/
				if(j < (beta[i].length -1))
					u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j + 1).getId();
				else 
					u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(0).getId();
//				System.out.println("u2ID="+u2Id);
				/***identify the edges (v, u1) and (v, u2)**/
				boolean foundEdgeToU1 = false, foundEdgeToU2 = false;
				int edgeToU1IsInverted = 0, edgeToU2IsInverted = 0;
				
				for(int k = 0; k < edgeList.size(); k++){

					/*intersectionNOdeid contain only route nodes, the adj node are always edge[1]. Otherwise is necessary to check the inverse*/
					if(edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u1Id) {
						indexDirVU1 = k;
						foundEdgeToU1 = true;
						edgeToU1IsInverted = 0;
					}
					else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u1Id){
						indexDirVU1 = k;
						foundEdgeToU1 = true;
						edgeToU1IsInverted = 1;
					}
					if( edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u2Id ){
						indexDirVU2 = k;
						foundEdgeToU2 = true;
						edgeToU2IsInverted = 0;
					}
					else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u2Id){
						indexDirVU2 = k;
						foundEdgeToU2 = true;
						edgeToU2IsInverted = 1;
					}
					if(foundEdgeToU1 && foundEdgeToU2)
						break;
				}
				
//				System.out.println("EdgeIndex to u1ID= "+ indexDirVU1 + " found: " + foundEdgeToU1  + " is inverted: " + edgeToU1IsInverted); 
//				System.out.println("EdgeIndex to u2ID= "+ indexDirVU2 + " found: " + foundEdgeToU2 + " is inverted: " + edgeToU2IsInverted); 

				leftSide = cplex.diff(dir[indexDirVU2][edgeToU2IsInverted], dir[indexDirVU1][edgeToU1IsInverted]);
				rightSide = cplex.diff(1,  cplex.prod(MCircularOrder, beta[i][j]));
				cplex.addGe(leftSide, rightSide, "CircOrderIneqIntersec"+i+"Adja"+j);
				


			}
			//System.out.println(); 


		}
		
		/*******CHECK SELF PLANARITY*******///
		
		System.out.println("checkin topology: " + checkTopology);
		if(checkTopology){
			IloNumVar[][][] gama = new IloNumVar[m][m][];
			/*RElation of m edges from the path and r route edges check relative position to 8 orientations ∈{N,S,E,W,NE,NW,SE,SW}*/
			for (int i = 0; i < m; i++) 
				for(int j = 0; j < m; j++)
					gama[i][j] = cplex.boolVarArray(8);
			
		
		
			IloLinearNumExpr[][] constraintEdgeTopology = new IloLinearNumExpr[m][m];
			/*SumOF{ γi(e1, e2) } ≥ 1   
			 * i∈{N,S,E,W,NE,NW,SE,SW} */
			for (int i = 0; i < m; i++) 
				for(int j = 0; j < m; j++){
					//				for (int k = 0; k < 8; k++){
					//					if(k!=0)
					//						cplex.addEq(gama[i][j][k], 0);
					//					else
					//						cplex.addEq(gama[i][j][k], 1);
					//				}


					constraintEdgeTopology[i][j] = cplex.linearNumExpr();
					constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][0]);
					constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][1]);
					constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][2]);
					constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][3]);
					constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][4]);
					constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][5]);
					constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][6]);
					constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][7]);
					cplex.addGe(constraintEdgeTopology[i][j], 1.0);

				}

			for(ArrayList<Integer> edgePaar: edgePaarToCheck) {
				int i = edgePaar.get(0);
				int j = edgePaar.get(1);
				int edge1UIndex = allNodeIdList.indexOf(edgeList.get(i)[0]); 
				int edge1VIndex = allNodeIdList.indexOf(edgeList.get(i)[1]);
//
				int edge2UIndex = allNodeIdList.indexOf(edgeList.get(j)[0]) ;
				int edge2VIndex = allNodeIdList.indexOf(edgeList.get(j)[1]) ;
				
				for(int k =0; k < 8; k ++){
					/*whats the value of M(max X ou y dist of points) and dist Min*/ 
					double MTopology = rescaledRouteLenght; /*da pra diminuir*/
					IloNumExpr rightSide,  leftSide1, leftSide2, leftSide3, leftSide4;


					switch (k) {
					case 0: /* E (e2(route edge) ist east of e1 (path edge)) : 0 : x*/

						/*
						 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
						 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
						 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
						 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
						 * ∀(e1,e2) no incident
						 */
						rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
						leftSide1 = cplex.diff(x[edge1UIndex], x[edge2UIndex]);
						leftSide2 = cplex.diff(x[edge1UIndex], x[edge2VIndex]);
						leftSide3 = cplex.diff(x[edge1VIndex], x[edge2UIndex] );
						leftSide4 = cplex.diff(x[edge1VIndex], x[edge2VIndex]);

						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);

						break;
					case 1: /* e2 (route edge) is NE (SE because y is inverted) of e1: 45 (315) : z1 */

						/*Can I use 
						 * route.get(j).getX() + route.get(j).getY() == (route.get(j).getX() / Math.sin(Math.toRadians(45)));
						 */

						rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
						leftSide1 = cplex.diff(z1[edge1UIndex], z1[edge2UIndex]);
						leftSide2 = cplex.diff(z1[edge1UIndex], z1[edge2VIndex]);
						leftSide3 = cplex.diff(z1[edge1VIndex], z1[edge2UIndex]);
						leftSide4 = cplex.diff(z1[edge1VIndex], z1[edge2VIndex]);

						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);


						break;
					case 2: /* e2 (route edge) is N (S because y is inverted) of e1: 90 (270) : y */
						rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

						leftSide1 = cplex.diff(y[edge1UIndex], y[edge2UIndex]);
						leftSide2 = cplex.diff(y[edge1UIndex], y[edge2VIndex]);
						leftSide3 = cplex.diff(y[edge1VIndex], y[edge2UIndex]);
						leftSide4 = cplex.diff(y[edge1VIndex], y[edge2VIndex]);

						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);

						break;	
					case 3: /* e2 (route edge) is NW (SW because y is inverted) of e1: 135 (225) : -z2 */

						rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

						leftSide1 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
						leftSide2 = cplex.diff( z2[edge2VIndex],  z2[edge1UIndex] );
						leftSide3 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
						leftSide4 = cplex.diff( z2[edge2VIndex],  z2[edge1VIndex] );

						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);

						break;
					case 4: /*  e2 (route edge) is W  of e1:  180:  -x */

						//						/*
						//						 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
						//						 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
						//						 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
						//						 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
						//						 * ∀(e1,e2) no incident
						//						 */

						rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

						leftSide1 = cplex.diff(x[edge2UIndex] , x[edge1UIndex] );
						leftSide2 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );
						leftSide3 = cplex.diff(x[edge2UIndex] , x[edge1VIndex] );
						leftSide4 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );

						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);

						break;
					case 5: /*  e2 (route edge) is SW (NW because y is inverted) of e1: 225 (135): -z1 */


						rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

						leftSide1 = cplex.diff( z1[edge2UIndex] , z1[edge1UIndex]);
						leftSide2 = cplex.diff( z1[edge2VIndex] , z1[edge1UIndex]);
						leftSide3 = cplex.diff( z1[edge2UIndex] , z1[edge1VIndex]);
						leftSide4 = cplex.diff( z1[edge2VIndex] , z1[edge1VIndex]);

						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);

						break;
					case 6:/*  e2 (route edge) is S (N because y is inverted) of e1: 270 (90)  : -y */

						rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
						leftSide1 = cplex.diff(  y[edge2UIndex] ,y[edge1UIndex] );
						leftSide2 = cplex.diff(  y[edge2VIndex], y[edge1UIndex]);
						leftSide3 = cplex.diff(  y[edge2UIndex] ,y[edge1VIndex] );
						leftSide4 = cplex.diff(  y[edge2VIndex], y[edge1VIndex]);

						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);

						break;
					case 7:/*  e2 (route edge) is SE (NE because y is inverted) of e1: 315 (45)  : z2*/

						rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
						leftSide1 = cplex.diff(z2[edge1UIndex], z2[edge2UIndex] );
						leftSide2 = cplex.diff(z2[edge1UIndex], z2[edge2VIndex] );
						leftSide3 = cplex.diff(z2[edge1VIndex], z2[edge2UIndex] );
						leftSide4 = cplex.diff(z2[edge1VIndex], z2[edge2VIndex] );

						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);


						break;
					default:
						break;
					}

				}
				
			}



		}
		
		
		
		
		/****Intersection(Crossings)Angles difference Minimization
		 * REquires Rvision****/
		
		MBend = 15; /*diference between 2 dir */
		/* bendDir[1] = dir[0] - dir[1]* bends angles (-7 to 7)*/
		int adjEdgNumber = adjacentNodeIdList.size();
		adjEdgNumber = m - mR;
		IloNumVar[] turnAdjEdgesGoing = cplex.intVarArray(adjEdgNumber, -7, 7);
		int[] origBestTurnAdjEdgesGoing = new int[adjEdgNumber];
		
		IloNumVar[] turnAdjEdgesBack = cplex.intVarArray(adjEdgNumber, -7, 7);
		int[] origBestTurnAdjEdgesBack = new int[adjEdgNumber];

		/*seleciona vertices de interseccao*/
		for (int i = 0; i < adjEdgNumber; i++) {
			
			/* U node in the route, V node of the intersection and W node of the adj dge*/
			int uId, vId, wId;
			
			wId = adjacentNodeIdList.get(i);
			int indexDirVW =0;
			vId = 0;
			boolean achouIndexDirVW = false;
			for(int j = mR; j< m; j++) {
				if(edgeList.get(j)[1] ==  wId) {
					vId = edgeList.get(j)[0];
					indexDirVW = j;
					achouIndexDirVW = true;
					break;
					
					
				}
				
			}
			int indexDirUV = 0;
			uId= 0;
			/*in route edge direction*/
			boolean achouIndexDirUV = false;
			for(int j = 0; j< mR; j++) {
				if(edgeList.get(j)[1] == vId ) {
					indexDirUV = j;
					uId=edgeList.get(j)[0];
					achouIndexDirUV = true;
					break;
				}
			}
			/**That is the case where the adj edge is in the begning of the route**/
			boolean achouIndexDirUV2 = false;
			if(!achouIndexDirUV) {
				for(int j = 0; j< mR; j++) {
					if(edgeList.get(j)[0] == vId ) {
						indexDirUV = j;
						uId=edgeList.get(j)[1];
						achouIndexDirUV2 = true;
						break;
					}
				}
				
			}
			/*in this case I'm considering the going and back route edges of the crossings, but for faster schematization I could just include the going edge, and if doesnt exist the back edges*/ 
			/**found going route edge*/
			if(achouIndexDirUV && achouIndexDirVW ) {
				cplex.addEq(turnAdjEdgesGoing[i], cplex.diff(dir[indexDirUV][0], dir[indexDirVW][0]));
				Point pt1 =  streetNodeMap.get(uId).getProjectGeom();
				Point pt2 =  streetNodeMap.get(vId).getProjectGeom();
				Point pt3 =  streetNodeMap.get(wId).getProjectGeom();
				origBestTurnAdjEdgesGoing[i] = GeometricOperation.turnDirectionOf(
						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
						0);
			}
			else if(achouIndexDirUV2 && achouIndexDirVW ) {
				System.out.println( "No route edge found before the intersection, but after");
				/*be aware the in that case we need to use dir[indexDirUV][1], because is the reversed way*/
				cplex.addEq(turnAdjEdgesGoing[i], cplex.diff(dir[indexDirUV][1], dir[indexDirVW][0]));
				Point pt1 =  streetNodeMap.get(uId).getProjectGeom();
				Point pt2 =  streetNodeMap.get(vId).getProjectGeom();
				Point pt3 =  streetNodeMap.get(wId).getProjectGeom();
				origBestTurnAdjEdgesGoing[i] = GeometricOperation.turnDirectionOf(
						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
						0);
			}	
			else {
				if(!achouIndexDirUV) 
					System.out.println( "No route edge found before the intersection");
			
				
				
				if(!achouIndexDirVW) 
					System.out.println( "No adje edge found on the intersection");
				
				
				cplex.addEq(turnAdjEdgesGoing[i], 0);
				origBestTurnAdjEdgesGoing[i] = 0;
			}
			
			/**found back route edge*/
//			if(achouIndexDirUV2 && achouIndexDirVW ) {
//				System.out.println( "No route edge found before the intersection, but after");
//				/*be aware the in that case we need to use dir[indexDirUV][1], because is the reversed way*/
//				cplex.addEq(turnAdjEdgesBack[i], cplex.diff(dir[indexDirUV][1], dir[indexDirVW][0]));
//				Point pt1 =  streetNodeMap.get(uId).getProjectGeom();
//				Point pt2 =  streetNodeMap.get(vId).getProjectGeom();
//				Point pt3 =  streetNodeMap.get(wId).getProjectGeom();
//				origBestTurnAdjEdgesBack[i] = GeometricOperation.turnDirectionOf(
//						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
//						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
//						0);
//			}			
//			else {
//				
//				if(!achouIndexDirUV2) 
//					System.out.println( "No route edge found after the intersection");
//				
//				
//				if(!achouIndexDirVW) 
//					System.out.println( "No adje edge found on the intersection");
//				
//				
//				cplex.addEq(turnAdjEdgesBack[i], 0);
//				origBestTurnAdjEdgesBack[i] = 0;
//			}
		}
		

		
		
		/**Cross representation on going way*/
//		
//		/*espstion to get absolut value of turn adj edges*/
		IloNumVar[][] epsilon2 = new IloNumVar[adjEdgNumber][];

		for (int i = 0; i < adjEdgNumber; i++) {
			epsilon2[i] = cplex.boolVarArray(2);
			epsilon2[i][0].setName("epsilo2AdjIR0");
			epsilon2[i][1].setName("epsilo2AdjIR1");
		}

		IloLinearNumExpr[] constraintEpsilon2 = new IloLinearNumExpr[adjEdgNumber];
		for (int i = 0; i < adjEdgNumber; i++) {
			constraintEpsilon2[i] = cplex.linearNumExpr();
			constraintEpsilon2[i].addTerm(1.0,epsilon2[i][0]);
			constraintEpsilon2[i].addTerm(1.0,epsilon2[i][1]);
			cplex.addEq(constraintEpsilon2[i], 1.0);
		}
////
////
////
		for (int i = 0; i < adjEdgNumber; i++) {
			cplex.addGe(turnAdjEdgesGoing[i], cplex.diff(0 , cplex.prod(MBend, epsilon2[i][0])));
			cplex.addLe(turnAdjEdgesGoing[i], cplex.diff(cplex.prod(MBend, epsilon2[i][1]),  1));

		}
		/*absolut tunr direction*/
		/*BendDir is turn direction
		 * 0 => 0-straight , 1=>315-veer right, 2=> 270-right, 3=>225sharp right, 
		 * 4=>180backturn, 5=>135-sharp left, 6=> 90left, 7=>45veer left
		 * */
		IloNumVar[] absolutTunrAdjEdges = cplex.intVarArray(adjEdgNumber, 0, 7);
		IloLinearNumExpr[] adjBendDirConstraint =  new IloLinearNumExpr[adjEdgNumber];
		for (int i = 0; i < adjEdgNumber; i++) {
			adjBendDirConstraint[i] = cplex.linearNumExpr();
			adjBendDirConstraint[i].addTerm(1, turnAdjEdgesGoing[i]);
			adjBendDirConstraint[i].addTerm(8, epsilon2[i][0]);
			cplex.addEq(absolutTunrAdjEdges[i], adjBendDirConstraint[i] );		
			//cplex.addLe(bendDir[i], bendCost[i]);
			//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));

		}
		
		/*Difference from original turn and resulting one*/
		IloNumVar[] turnDiffFromOriginal = cplex.intVarArray(adjEdgNumber, -7, 7);

		for (int i = 0; i < adjEdgNumber; i++) {
			//routeNodeIdList.get(i+1)
			cplex.addEq(turnDiffFromOriginal[i], cplex.diff(absolutTunrAdjEdges[i], origBestTurnAdjEdgesGoing[i]));
		}
		
		
		IloNumVar[][] zeta2 = new IloNumVar[adjEdgNumber][];
		
		for (int i = 0; i < adjEdgNumber; i++) {
			zeta2[i] = cplex.boolVarArray(3);
			zeta2[i][0].setName("ZETA2AdjTunrDiff0");
			zeta2[i][1].setName("ZETA2AdjTunrDiff1");
			zeta2[i][2].setName("ZETA2AdjTunrDiff2");
			
		}
		IloLinearNumExpr[] constraintZeta2 = new IloLinearNumExpr[adjEdgNumber];
		for (int i = 0; i < adjEdgNumber; i++) {
			constraintZeta2[i] = cplex.linearNumExpr();
			constraintZeta2[i].addTerm(1.0,zeta2[i][0]);
			constraintZeta2[i].addTerm(1.0,zeta2[i][1]);
			constraintZeta2[i].addTerm(1.0,zeta2[i][2]);
			cplex.addEq(constraintZeta2[i], 2.0);
		}
//
		MDir = 8;
		for (int i = 0; i < adjEdgNumber; i++) {
			cplex.addLe(turnDiffFromOriginal[i], cplex.diff(cplex.prod(MDir, zeta2[i][0]), 5));
			cplex.addGe(turnDiffFromOriginal[i], cplex.diff(5 , cplex.prod(MDir, zeta2[i][1])));
			cplex.addLe(turnDiffFromOriginal[i], cplex.sum(4 , cplex.prod(MDir, zeta2[i][2])));
			cplex.addGe(turnDiffFromOriginal[i], cplex.diff(-4 , cplex.prod(MDir, zeta2[i][2])));
			
		}
//		/*costs vary from 0 to 4, same approach for bendcost of dir cos*/
		IloNumVar[] adJEdgeTurnCost = cplex.intVarArray(adjEdgNumber, 0, 4);
		IloLinearNumExpr[] adJEdgeTurnCostConstraint =  new IloLinearNumExpr[adjEdgNumber];
		for (int i = 0; i < adjEdgNumber; i++) {
			adJEdgeTurnCostConstraint[i] = cplex.linearNumExpr();
			adJEdgeTurnCostConstraint[i].addTerm(1, turnDiffFromOriginal[i]);
			adJEdgeTurnCostConstraint[i].addTerm(-8, zeta2[i][0]);
			adJEdgeTurnCostConstraint[i].addTerm(8, zeta2[i][1]);
			cplex.addGe(adJEdgeTurnCostConstraint[i], cplex.prod(-1, adJEdgeTurnCost[i]) );
			cplex.addLe(adJEdgeTurnCostConstraint[i], adJEdgeTurnCost[i] );

			
		}
		IloNumExpr sumAdjTunrCost =  cplex.sum(adJEdgeTurnCost);  
//		IloNumExpr sumAdjTunrCostFactor = cplex.prod((1000),sumAdjTunrCost);

		
		/**Cross representation on back way*/
//		
//		/*espstion to get absolut value of turn adj edges*/
//		IloNumVar[][] epsilon2back = new IloNumVar[adjEdgNumber][];
//
//		for (int i = 0; i < adjEdgNumber; i++) {
//			epsilon2back[i] = cplex.boolVarArray(2);
//			epsilon2back[i][0].setName("epsilo2backAdjIR0");
//			epsilon2back[i][1].setName("epsilo2backAdjIR1");
//		}
//
//		IloLinearNumExpr[] constraintEpsilon2Back = new IloLinearNumExpr[adjEdgNumber];
//		for (int i = 0; i < adjEdgNumber; i++) {
//			constraintEpsilon2Back[i] = cplex.linearNumExpr();
//			constraintEpsilon2Back[i].addTerm(1.0,epsilon2back[i][0]);
//			constraintEpsilon2Back[i].addTerm(1.0,epsilon2back[i][1]);
//			cplex.addEq(constraintEpsilon2Back[i], 1.0);
//		}
//////
//////
//////
//		for (int i = 0; i < adjEdgNumber; i++) {
//			cplex.addGe(turnAdjEdgesBack[i], cplex.diff(0 , cplex.prod(MBend, epsilon2back[i][0])));
//			cplex.addLe(turnAdjEdgesBack[i], cplex.diff(cplex.prod(MBend, epsilon2back[i][1]),  1));
//
//		}
//		/*absolut tunr direction*/
//		/*BendDir is turn direction
//		 * 0 => 0-straight , 1=>315-veer right, 2=> 270-right, 3=>225sharp right, 
//		 * 4=>180backturn, 5=>135-sharp left, 6=> 90left, 7=>45veer left
//		 * */
//		IloNumVar[] absolutTunrAdjEdgesBack = cplex.intVarArray(adjEdgNumber, 0, 7);
//		IloLinearNumExpr[] adjBendDirConstraintBack =  new IloLinearNumExpr[adjEdgNumber];
//		for (int i = 0; i < adjEdgNumber; i++) {
//			adjBendDirConstraintBack[i] = cplex.linearNumExpr();
//			adjBendDirConstraintBack[i].addTerm(1, turnAdjEdgesBack[i]);
//			adjBendDirConstraintBack[i].addTerm(8, epsilon2back[i][0]);
//			cplex.addEq(absolutTunrAdjEdgesBack[i], adjBendDirConstraintBack[i] );		
//			//cplex.addLe(bendDir[i], bendCost[i]);
//			//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));
//
//		}
//		
//		/*Difference from original turn and resulting one*/
//		IloNumVar[] turnDiffFromOriginalBack = cplex.intVarArray(adjEdgNumber, -7, 7);
//
//		for (int i = 0; i < adjEdgNumber; i++) {
//			//routeNodeIdList.get(i+1)
//			cplex.addEq(turnDiffFromOriginalBack[i], cplex.diff(absolutTunrAdjEdgesBack[i], origBestTurnAdjEdgesBack[i]));
//		}
//		
//		
//		IloNumVar[][] zeta2Back = new IloNumVar[adjEdgNumber][];
//		
//		for (int i = 0; i < adjEdgNumber; i++) {
//			zeta2Back[i] = cplex.boolVarArray(3);
//			zeta2Back[i][0].setName("ZETA2AdjTunrDiff0");
//			zeta2Back[i][1].setName("ZETA2AdjTunrDiff1");
//			zeta2Back[i][2].setName("ZETA2AdjTunrDiff2");
//			
//		}
//		IloLinearNumExpr[] constraintZeta2Back = new IloLinearNumExpr[adjEdgNumber];
//		for (int i = 0; i < adjEdgNumber; i++) {
//			constraintZeta2Back[i] = cplex.linearNumExpr();
//			constraintZeta2Back[i].addTerm(1.0,zeta2Back[i][0]);
//			constraintZeta2Back[i].addTerm(1.0,zeta2Back[i][1]);
//			constraintZeta2Back[i].addTerm(1.0,zeta2Back[i][2]);
//			cplex.addEq(constraintZeta2Back[i], 2.0);
//		}
////
//		MDir = 8;
//		for (int i = 0; i < adjEdgNumber; i++) {
//			cplex.addLe(turnDiffFromOriginalBack[i], cplex.diff(cplex.prod(MDir, zeta2Back[i][0]), 5));
//			cplex.addGe(turnDiffFromOriginalBack[i], cplex.diff(5 , cplex.prod(MDir, zeta2Back[i][1])));
//			cplex.addLe(turnDiffFromOriginalBack[i], cplex.sum(4 , cplex.prod(MDir, zeta2Back[i][2])));
//			cplex.addGe(turnDiffFromOriginalBack[i], cplex.diff(-4 , cplex.prod(MDir, zeta2Back[i][2])));
//			
//		}
////		/*costs vary from 0 to 4, same approach for bendcost of dir cos*/
//		IloNumVar[] adJEdgeTurnCostBack = cplex.intVarArray(adjEdgNumber, 0, 4);
//		IloLinearNumExpr[] adJEdgeTurnCostConstraintBack =  new IloLinearNumExpr[adjEdgNumber];
//		for (int i = 0; i < adjEdgNumber; i++) {
//			adJEdgeTurnCostConstraintBack[i] = cplex.linearNumExpr();
//			adJEdgeTurnCostConstraintBack[i].addTerm(1, turnDiffFromOriginalBack[i]);
//			adJEdgeTurnCostConstraintBack[i].addTerm(-8, zeta2Back[i][0]);
//			adJEdgeTurnCostConstraintBack[i].addTerm(8, zeta2Back[i][1]);
//			cplex.addGe(adJEdgeTurnCostConstraintBack[i], cplex.prod(-1, adJEdgeTurnCostBack[i]) );
//			cplex.addLe(adJEdgeTurnCostConstraintBack[i], adJEdgeTurnCostBack[i] );
//
//			
//		}
//		IloNumExpr sumAdjTunrCostBack =  cplex.sum(adJEdgeTurnCostBack);  
////		IloNumExpr sumAdjTunrCostFactor = cplex.prod((1000),sumAdjTunrCost);		
		
		
		/*********BEND MINIMIZATION*****************/

		/*Boolean variable to make condition of bends angle <-5, >-4 e <4, ou >5 measure bends angles (-7 to 7)*/
		/***∆benddiff(u, v,w) = dir(u, v) − dir(v,w)***/
		/***∆benddiff(u, v,w) ranges from −7 to 7****/
		
//		bend(u, v,w) = ?|∆dir(u, v,w)| if |∆dir(u, v,w)| ≤ 4 
//						8−|∆dir(u, v,w)| if |∆dir(u, v,w)| ≥ 5.
				
		MBend = 15;
		//M = 8;
		IloNumVar[][] delta = new IloNumVar[b][];
		
		for (int i = 0; i < b; i++) {
			delta[i] = cplex.boolVarArray(3);
			delta[i][0].setName("DeltaBendMin0");
			delta[i][1].setName("DeltaBendMin1");
			delta[i][2].setName("DeltaBendMin2");
			
		}
		IloLinearNumExpr[] constraintBenMinimizationBooleans = new IloLinearNumExpr[b];
		for (int i = 0; i < b; i++) {
			constraintBenMinimizationBooleans[i] = cplex.linearNumExpr();
			constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][0]);
			constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][1]);
			constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][2]);
			cplex.addEq(constraintBenMinimizationBooleans[i], 2);
		}

		
		for (int i = 0; i < b; i++) {
			cplex.addLe(bendDiff[i], cplex.diff(cplex.prod(MBend, delta[i][0]), 5));
			cplex.addGe(bendDiff[i], cplex.diff(5 , cplex.prod(MBend, delta[i][1])));
			cplex.addLe(bendDiff[i], cplex.sum(4 , cplex.prod(MBend, delta[i][2])));
			cplex.addGe(bendDiff[i], cplex.diff(-4 , cplex.prod(MBend, delta[i][2])));
			
		}
		
		IloNumVar[] bendCost = cplex.intVarArray(b, 0, 4);
		IloLinearNumExpr[] bendCostConstraint =  new IloLinearNumExpr[b];
		for (int i = 0; i < b; i++) {
			
			if(!streetNodeMap.get(routeNodeIdList.get(i + 1)).isDecisionPoint()){
				bendCostConstraint[i] = cplex.linearNumExpr();
				bendCostConstraint[i].addTerm(1, bendDiff[i]);
				bendCostConstraint[i].addTerm(-8, delta[i][0]);
				bendCostConstraint[i].addTerm(8, delta[i][1]);
				cplex.addGe(bendCostConstraint[i], cplex.prod(-1, bendCost[i]) );
				cplex.addLe(bendCostConstraint[i], bendCost[i] );
			}
			else
				cplex.addEq(bendCost[i], 0);
			
			
			//cplex.addLe(bendDir[i], bendCost[i]);
			//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));
			
		}
		
		IloNumVar[] bendCost2 = cplex.intVarArray(b, 0, 16);
		for (int i = 0; i < b; i++) {
			if(streetNodeMap.get(routeNodeIdList.get(i + 1)).getDegree() > 2){
			
				cplex.addEq(bendCost2[i], cplex.prod(4, bendCost[i]));
			}
			else {
				
				cplex.addEq(bendCost2[i], bendCost[i]);
			}
			
			//cplex.addLe(bendDir[i], bendCost[i]);
			//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));
			
		}

		//System.out.println("factor= " + factor);
		IloNumExpr sumBendCost =  cplex.sum(bendCost2);

		
		
		/****Edges orientation difference minimization*********/
		IloNumVar[] dirDiff  = cplex.numVarArray(m, -7, 7);
		for (int i = 0; i < m; i++) {
			cplex.addEq(dirDiff[i], cplex.diff(sec[i][0][1] ,dir[i][0]));
		}
		
//		IloNumVar[][] zetaBinary = new IloNumVar[m][];
//		
//		for (int i = 0; i < m; i++) {
//			zetaBinary[i] = cplex.boolVarArray(2);
//			
//		}
//		
//		IloLinearNumExpr[] constraintZetaBinary = new IloLinearNumExpr[m];
//		for (int i = 0; i < m; i++) {
//			constraintZetaBinary[i] = cplex.linearNumExpr();
//			constraintZetaBinary[i].addTerm(1.0,zetaBinary[i][0]);
//			constraintZetaBinary[i].addTerm(1.0,zetaBinary[i][1]);
//			cplex.addEq(constraintZetaBinary[i], 1.0);
//		}
//		MDir = 8;
//		for (int i = 0; i < m; i++) {
//			IloNumExpr rightSide = cplex.prod(MDir, cplex.diff(1, zetaBinary[i][0]));
//			cplex.addGe(dirDiff[i], rightSide);
//			cplex.addLe(dirDiff[i], cplex.negative(rightSide));
//			
//		}
//		IloNumVar[] dirCost = cplex.intVarArray(mR, 0, 2);
//		for (int i = 0; i < mR; i++) {
//			cplex.addEq(dirCost[i], zetaBinary[i][1]);
//		}
//		
//		IloNumVar[] dirCostAdj = cplex.boolVarArray(m-mR);
//		for (int i = mR; i < m; i++) {
//			cplex.addEq(dirCostAdj[i-mR], zetaBinary[i][1]);
//		}

		IloNumVar[] dirCost = cplex.boolVarArray(mR);
		for (int i = 0; i < mR; i++) {

			cplex.addLe(dirDiff[i],cplex.prod(8, dirCost[i]));
			cplex.addGe(dirDiff[i],  cplex.prod(-8, dirCost[i]));
		}
		IloNumExpr sumDirCost =  cplex.sum(dirCost); 
		
		
		IloNumExpr sumDirCostAdj = null;
		if(m > mR) {
			IloNumVar[] dirCostAdj = cplex.boolVarArray(m-mR);
			for (int i = mR; i < m; i++) {
				cplex.addLe(dirDiff[i],cplex.prod(8, dirCostAdj[i-mR]));
				cplex.addGe(dirDiff[i], cplex.prod(-8, dirCostAdj[i-mR]));
			}
			sumDirCostAdj  =  cplex.sum(dirCostAdj);
		}

//		
		
		
		
//		IloNumVar[][] zeta = new IloNumVar[m][];
//		
//		for (int i = 0; i < m; i++) {
//			zeta[i] = cplex.boolVarArray(3);
//			zeta[i][0].setName("ZETADirMin0");
//			zeta[i][1].setName("ZETADirMin1");
//			zeta[i][2].setName("ZETADirMin2");
//			
//		}
//		IloLinearNumExpr[] constraintDirDiffMinimizationBooleans = new IloLinearNumExpr[m];
//		for (int i = 0; i < m; i++) {
//			constraintDirDiffMinimizationBooleans[i] = cplex.linearNumExpr();
//			constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][0]);
//			constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][1]);
//			constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][2]);
//			cplex.addEq(constraintDirDiffMinimizationBooleans[i], 2.0);
//		}
//
//		MDir = 8;
//		for (int i = 0; i < m; i++) {
//			cplex.addLe(dirDiff[i], cplex.diff(cplex.prod(MDir, zeta[i][0]), 5));
//			cplex.addGe(dirDiff[i], cplex.diff(5 , cplex.prod(MDir, zeta[i][1])));
//			cplex.addLe(dirDiff[i], cplex.sum(4 , cplex.prod(MDir, zeta[i][2])));
//			cplex.addGe(dirDiff[i], cplex.diff(-4 , cplex.prod(MDir, zeta[i][2])));
//			
//		}
//		IloNumVar[] dirCost = cplex.intVarArray(mR, 0, 4);
//		IloNumVar[] dirCostAdj = cplex.intVarArray(m-mR, 0, 4);
//		IloLinearNumExpr[] dirCostConstraint =  new IloLinearNumExpr[m];
//		for (int i = 0; i < mR; i++) {
//			dirCostConstraint[i] = cplex.linearNumExpr();
//			dirCostConstraint[i].addTerm(1, dirDiff[i]);
//			dirCostConstraint[i].addTerm(-8, zeta[i][0]);
//			dirCostConstraint[i].addTerm(8, zeta[i][1]);
//			cplex.addGe(dirCostConstraint[i], cplex.prod(-1, dirCost[i]) );
//			cplex.addLe(dirCostConstraint[i], dirCost[i] );
//
//			
//		}
//		for (int i = mR; i < m; i++) {
//			dirCostConstraint[i] = cplex.linearNumExpr();
//			dirCostConstraint[i].addTerm(1, dirDiff[i]);
//			dirCostConstraint[i].addTerm(-8, zeta[i][0]);
//			dirCostConstraint[i].addTerm(8, zeta[i][1]);
//			cplex.addGe(dirCostConstraint[i], cplex.prod(-1, dirCostAdj[i-mR]) );
//			cplex.addLe(dirCostConstraint[i], dirCostAdj[i-mR] );
//
//			
//		}

		
		 
		
		
	
		/*********Distance Minization*****************/
		/**
		 * 
		 * If you are minimizing an increasing function of |x| (or maximizing a decreasing function, of course), you can always have the aboslute value of any quantity x in a lp as a variable absx such as:

			absx >= x
			absx >= -x
			{leftside = x
			dX = absx}
			It works because the value absx will 'tend' to its lower bound, so it will either reach x or -x.

			On the other hand, if you are minimizing a decreasing function of |x|, your problem is not convex and cannot be modelled as a lp.
			
			example: x = 1 -> absx >= 1 and absx >= -1  -> abx >= 1 
					 x = -1 -> absx >= -1 and absx >= 1 -> absx >= 1 and we want to minimize absx, so it will tend to 1!
		 */

		/*distance reduction*/
		IloNumVar[] dX  = cplex.numVarArray(nR, 0, Double.MAX_VALUE);
		IloNumVar[] dY  = cplex.numVarArray(nR, 0, Double.MAX_VALUE);
		//IloNumVar[] dMax  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
		
		
	for (int i = 0; i < nR; i++) {
			double reScaledx = rescaledRelevantRoutePoints.get(i).getX();
			double reScaledy = rescaledRelevantRoutePoints.get(i).getY();
//			IloNumExpr xLeftSide = cplex.diff(streetNodeMap.get(routeNodeIdList.get(i)).getProjectGeom().getX(), x[i]);
//			IloNumExpr yLeftSide = cplex.diff(streetNodeMap.get(routeNodeIdList.get(i)).getProjectGeom().getY(), y[i]);
			
			IloNumExpr xLeftSide = cplex.diff(reScaledx, x[i]);
			IloNumExpr yLeftSide = cplex.diff(reScaledy, y[i]);
			
			cplex.addLe(xLeftSide, dX[i]);
			cplex.addGe(xLeftSide, cplex.prod(-1, dX[i]));
			
			
			cplex.addLe(yLeftSide, dY[i]);
			cplex.addGe(yLeftSide, cplex.prod(-1, dY[i]));
			
		}
		
		
		
		IloNumExpr sumXDist =  cplex.sum(dX);
		IloNumExpr sumYDist =  cplex.sum(dY);

		
		
		
		/*************Proportion diference minimization***********/

		/*** lambda: binary variable to determine if (xi - xi+1) is > 0 or < 0**
		 * lambda1 + lambda2 =1 -> if lambda1 = 1 - > lambda2 = 0 ->  (xi - xi+1) >= 0
		 * 							else lambda1 = 0 -> lambda2 = 1 -> (xi - xi+1) < 0
		 * 
		 * we want dx = |xi - xi+1| -> dx >= |xi - xi+1| & dx <= |xi - xi+1|
		 * if (xi - xi+1) >= 0 -> dx >= xi - xi+1 & dx <= xi - xi+1
		 * if (xi - xi+1) < 0  -> dx >= -(xi - xi+1) & dx <= -(xi - xi+1) 
		 * 
		 * so
		 * dx <= (xi - xi+1) + M(1-lambda1) eq1
		 * dx >= (xi - xi+1) - M(1-lambda1) eq2
		 * 
		 * or
		 * dx >= -(xi - xi+1) - M(1-lambda2) eq3
		 * dx <= -(xi - xi+1) + M(1-lambda2) eq4
		 * 
		 * if lambda1 = 1 & lambda2 = 0  => constraint dx = (xi - xi+1) & e3 and eq4 are trivialy fulfiled(makes no diference) 
		 * else lambda1 = 0 & lambda2 = 1  => constraint dx = -(xi - xi+1) & e1 and eq2 are trivialy fulfiled(makes no diference) 
		 * 
		 * 
		 * 
		 * */
		
		int mP = proportionKeepingRescaledRouteDPPoints.size() -1; /*route edges number: respect order on edge list*/ 
		
		IloNumVar[][] xLambdaDP = new IloNumVar[mP][];
		IloNumVar[][] yLambdaDP = new IloNumVar[mP][];
		
		for (int i = 0; i < mP; i++) {
			xLambdaDP[i] = cplex.boolVarArray(2);
			yLambdaDP[i] = cplex.boolVarArray(2);
			xLambdaDP[i][0].setName("proporMinxLambda0");
			xLambdaDP[i][1].setName("proporMinxLambda1");
			yLambdaDP[i][0].setName("proporMinyLambda0");
			yLambdaDP[i][1].setName("proporMinyLambda1");
			
		}
		IloLinearNumExpr[] xLambdaConstraintDP = new IloLinearNumExpr[mP];
		IloLinearNumExpr[] yLambdaConstraintDP = new IloLinearNumExpr[mP];
		for (int i = 0; i < mP; i++) {
			xLambdaConstraintDP[i] = cplex.linearNumExpr();
			xLambdaConstraintDP[i].addTerm(1.0,xLambdaDP[i][0]);
			xLambdaConstraintDP[i].addTerm(1.0,xLambdaDP[i][1]);

			cplex.addEq(xLambdaConstraintDP[i], 1.0);
			
			
			yLambdaConstraintDP[i] = cplex.linearNumExpr();
			yLambdaConstraintDP[i].addTerm(1.0,yLambdaDP[i][0]);
			yLambdaConstraintDP[i].addTerm(1.0,yLambdaDP[i][1]);

			cplex.addEq(yLambdaConstraintDP[i], 1.0);
			
		}
		
		 
		/*  maxAdjD distance between two adjacent vertices vertices! it was MaxD*/
		MAdjVertice = rescaledRouteLenght;
		IloNumVar[] absoluteDiffXDP  = cplex.numVarArray(mP, 0, 4* maxAdjD);
		IloNumVar[] absoluteDiffYDP  = cplex.numVarArray(mP, 0, 4* maxAdjD);
		
		for (int i = 0; i < mP; i++) {
			
			int v1Index = allNodeIdList.indexOf(proportionKeepingRescaledRouteDPPointsId.get(i));
			int v2Index = allNodeIdList.indexOf(proportionKeepingRescaledRouteDPPointsId.get(i + 1));	
			
			/***dx = |xi - xi+1|***/
			IloNumExpr diffX = cplex.diff(x[v1Index], x[v2Index]);
			
			cplex.addLe(absoluteDiffXDP[i], cplex.sum(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambdaDP[i][0])))); //eq1. 
			cplex.addGe(absoluteDiffXDP[i], cplex.diff(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambdaDP[i][0])))); //eq2.
			
			cplex.addGe(absoluteDiffXDP[i], cplex.diff(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambdaDP[i][1])))); //eq3. 
			cplex.addLe(absoluteDiffXDP[i], cplex.sum(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambdaDP[i][1])))); //eq4.
			
			/***dy = |xi - yi+1|***/
			IloNumExpr diffY = cplex.diff(y[v1Index], y[v2Index]);
			
			cplex.addLe(absoluteDiffYDP[i], cplex.sum(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambdaDP[i][0])))); //eq1. 
			cplex.addGe(absoluteDiffYDP[i], cplex.diff(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambdaDP[i][0])))); //eq2.
			
			cplex.addGe(absoluteDiffYDP[i], cplex.diff(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambdaDP[i][1])))); //eq3. 
			cplex.addLe(absoluteDiffYDP[i], cplex.sum(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambdaDP[i][1])))); //eq4.

			
			
		}
		/***WE have absDiffX and absDiffY
		 * We wanto to minimize:
		 * Abs((absDiffX[i] + absDiffY[i])L - lengthEdge[i]/L)
		 * 
		 * ****/
		/**
		 * If you are minimizing an increasing function of |x| (or maximizing a decreasing function, of course), you can always have the aboslute value of any quantity x in a lp as a variable absx such as:

			absx >= x -> x<=absx
			absx >= -x -> x >= -absx
			{leftside = x
			dX = absx}
			It works because the value absx will 'tend' to its lower bound, so it will either reach x or -x.

			On the other hand, if you are minimizing a decreasing function of |x|, your problem is not convex and cannot be modelled as a lp.
		 */
		ArrayList<Double> edgeProportion = new ArrayList<Double>();
		for (int i = 0; i < mP; i++) {
			Point2D pt1 =  proportionKeepingRescaledRouteDPPoints.get(i);
			Point2D pt2 =  proportionKeepingRescaledRouteDPPoints.get(i+1);
			double manhantamDist = (Math.abs(pt1.getX() - pt2.getX()) + Math.abs( pt1.getY() - pt2.getY() ) );
			//edgeProportion.add(manhantamDist/routeLenght);
			edgeProportion.add(proportionKeepingRescaledRouteDPPoints.get(i).distance(proportionKeepingRescaledRouteDPPoints.get(i+1))/rescaledRouteLenght);
		}
		IloNumVar[] absolutEdgeProportionDifferenceDP  = cplex.numVarArray(mP, 0, Double.MAX_VALUE);
		IloNumExpr[] finalEdgeProportionDP = new IloNumExpr[mP];
		for (int i = 0; i < mP; i++) {
			finalEdgeProportionDP[i] = cplex.prod(1/rescaledRouteLenght,cplex.sum(absoluteDiffXDP[i],absoluteDiffYDP[i]));
			/*can I use cplex.Equal direct? yes absX and absY is always positiv*/
			cplex.addLe(cplex.diff(finalEdgeProportionDP[i], edgeProportion.get(i)), absolutEdgeProportionDifferenceDP[i]);
			cplex.addGe(cplex.diff(finalEdgeProportionDP[i], edgeProportion.get(i)), cplex.prod(-1, absolutEdgeProportionDifferenceDP[i]));
		}
		
		IloNumExpr sumAbsolutEdgeProportionDifferenceDP =  cplex.sum(absolutEdgeProportionDifferenceDP);

//		

		//			
		/*************PROPORTION DIFFERENCE MINIMIZATION ALL EDEGES***********/

		/*** lambda: binary variable to determine if (xi - xi+1) is > 0 or < 0**
		 * lambda1 + lambda2 =1 -> if lambda1 = 1 - > lambda2 = 0 ->  (xi - xi+1) >= 0
		 * 							else lambda1 = 0 -> lambda2 = 1 -> (xi - xi+1) < 0
		 * 
		 * we want dx = |xi - xi+1| -> dx >= |xi - xi+1| & dx <= |xi - xi+1|
		 * if (xi - xi+1) >= 0 -> dx >= xi - xi+1 & dx <= xi - xi+1
		 * if (xi - xi+1) < 0  -> dx >= -(xi - xi+1) & dx <= -(xi - xi+1) 
		 * 
		 * so
		 * dx <= (xi - xi+1) + M(1-lambda1) eq1
		 * dx >= (xi - xi+1) - M(1-lambda1) eq2
		 * 
		 * or
		 * dx >= -(xi - xi+1) - M(1-lambda2) eq3
		 * dx <= -(xi - xi+1) + M(1-lambda2) eq4
		 * 
		 * if lambda1 = 1 & lambda2 = 0  => constraint dx = (xi - xi+1) & e3 and eq4 are trivialy fulfiled(makes no diference) 
		 * else lambda1 = 0 & lambda2 = 1  => constraint dx = -(xi - xi+1) & e1 and eq2 are trivialy fulfiled(makes no diference) 
		 * 
		 * 
		 * 
		 * */
		
		//int mP = proportionKeepingRescaledRouteDPPoints.size() -1; /*route edges number: respect order on edge list*/ 
		
		IloNumVar[][] xLambda = new IloNumVar[mR][];
		IloNumVar[][] yLambda = new IloNumVar[mR][];
		
		for (int i = 0; i < mR; i++) {
			xLambda[i] = cplex.boolVarArray(2);
			yLambda[i] = cplex.boolVarArray(2);
			xLambda[i][0].setName("proporMinxLambda0");
			xLambda[i][1].setName("proporMinxLambda1");
			yLambda[i][0].setName("proporMinyLambda0");
			yLambda[i][1].setName("proporMinyLambda1");
			
		}
		IloLinearNumExpr[] xLambdaConstraint = new IloLinearNumExpr[mR];
		IloLinearNumExpr[] yLambdaConstraint = new IloLinearNumExpr[mR];
		for (int i = 0; i < mR; i++) {
			xLambdaConstraint[i] = cplex.linearNumExpr();
			xLambdaConstraint[i].addTerm(1.0,xLambda[i][0]);
			xLambdaConstraint[i].addTerm(1.0,xLambda[i][1]);

			cplex.addEq(xLambdaConstraint[i], 1.0);
			
			
			yLambdaConstraint[i] = cplex.linearNumExpr();
			yLambdaConstraint[i].addTerm(1.0,yLambda[i][0]);
			yLambdaConstraint[i].addTerm(1.0,yLambda[i][1]);

			cplex.addEq(yLambdaConstraint[i], 1.0);
			
		}
		
		 
		/*  maxAdjD distance between two adjacent vertices vertices! it was MaxD*/
		MAdjVertice = rescaledRouteLenght;
		IloNumVar[] absoluteDiffX  = cplex.numVarArray(mR, 0, 4* maxAdjD);
		IloNumVar[] absoluteDiffY  = cplex.numVarArray(mR, 0, 4* maxAdjD);
		
		for (int i = 0; i < mR; i++) {
			
			int v1Index = allNodeIdList.indexOf(routeNodeIdList.get(i));
			int v2Index = allNodeIdList.indexOf(routeNodeIdList.get(i + 1));	
			
			/***dx = |xi - xi+1|***/
			IloNumExpr diffX = cplex.diff(x[v1Index], x[v2Index]);
			
			cplex.addLe(absoluteDiffX[i], cplex.sum(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq1. 
			cplex.addGe(absoluteDiffX[i], cplex.diff(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq2.
			
			cplex.addGe(absoluteDiffX[i], cplex.diff(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq3. 
			cplex.addLe(absoluteDiffX[i], cplex.sum(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq4.
			
			/***dy = |xi - yi+1|***/
			IloNumExpr diffY = cplex.diff(y[v1Index], y[v2Index]);
			
			cplex.addLe(absoluteDiffY[i], cplex.sum(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq1. 
			cplex.addGe(absoluteDiffY[i], cplex.diff(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq2.
			
			cplex.addGe(absoluteDiffY[i], cplex.diff(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq3. 
			cplex.addLe(absoluteDiffY[i], cplex.sum(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq4.

			
			
		}
		/***WE have absDiffX and absDiffY
		 * We wanto to minimize:
		 * Abs((absDiffX[i] + absDiffY[i])L - lengthEdge[i]/L)
		 * 
		 * ****/
		/**
		 * If you are minimizing an increasing function of |x| (or maximizing a decreasing function, of course), you can always have the aboslute value of any quantity x in a lp as a variable absx such as:

			absx >= x -> x<=absx
			absx >= -x -> x >= -absx
			{leftside = x
			dX = absx}
			It works because the value absx will 'tend' to its lower bound, so it will either reach x or -x.

			On the other hand, if you are minimizing a decreasing function of |x|, your problem is not convex and cannot be modelled as a lp.
		 */
		edgeProportion = new ArrayList<Double>();
		for (int i = 0; i < mR; i++) {

			
			//edgeProportion.add(pt1.distance(pt2)/routeLenght);
			edgeProportion.add(rescaledRelevantRoutePoints.get(i).distance(rescaledRelevantRoutePoints.get(i+1))/rescaledRouteLenght);
		}
		IloNumVar[] absolutEdgeProportionDifference  = cplex.numVarArray(mR, 0, Double.MAX_VALUE);
		IloNumExpr[] finalEdgeProportion = new IloNumExpr[mR];
		for (int i = 0; i < mR; i++) {
			finalEdgeProportion[i] = cplex.prod(1/rescaledRouteLenght,cplex.sum(absoluteDiffX[i],absoluteDiffY[i]));
			/*can I use cplex.Equal direct? yes absX and absY is always positiv*/
			cplex.addLe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), absolutEdgeProportionDifference[i]);
			cplex.addGe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), cplex.prod(-1, absolutEdgeProportionDifference[i]));
		}
		
		IloNumExpr sumAbsolutEdgeProportionDifference =  cplex.sum(absolutEdgeProportionDifference);

		
		
		
		
		/*********OBJECTIVE*****************/
		
//		IloNumExpr sumBendCostNormal =  cplex.prod(1.0/b,sumBendCost);
//		IloNumExpr sumBendCostFactor = cplex.prod(bendFactor,sumBendCostNormal);
//		
//		IloNumExpr sumDistCostNormal = cplex.prod(1.0/nR, cplex.sum(sumXDist, sumYDist ));			
//		IloNumExpr sumDistCostFactor = cplex.prod(distFactor, sumDistCostNormal);
		
//		IloNumExpr sumDirCostNormal =  cplex.prod(1.5/mR,sumDirCost); 
//		
//		IloNumExpr sumDirCostFactor = null;
//		if( sumDirCostAdj != null) {
//			IloNumExpr sumDirCostAdjNormal =  cplex.prod(0.1/(m-mR),sumDirCostAdj);			
//			sumDirCostFactor = cplex.sum(cplex.prod(dirFactor,sumDirCostNormal),cplex.prod((dirFactor),sumDirCostAdjNormal));
//		}
//		else {
//			 sumDirCostFactor = cplex.prod(dirFactor,sumDirCostNormal);
//		}
//		IloNumExpr sumAbsolutEdgeProportionDifferenceNormalDP = cplex.prod(1.5/mP, sumAbsolutEdgeProportionDifferenceDP);
//		IloNumExpr sumEdgeProportionFactorDP = cplex.prod(proportionDPFactor, sumAbsolutEdgeProportionDifferenceNormalDP);
//		
//		IloNumExpr sumAbsolutEdgeProportionDifferenceNormal = cplex.prod(2.0/mR, sumAbsolutEdgeProportionDifference);
//		IloNumExpr sumEdgeProportionAllPtsFactor = cplex.prod(proportionAllPtsFactor, sumAbsolutEdgeProportionDifferenceNormal);

		double adjTurnWeight = 1.0*crossingBendFactor; /*normalized under number of bends*/
		double adjTurnNormalized = adjTurnWeight/adjEdgNumber;
		IloNumExpr sumAdjTurnCostFactor = cplex.prod(adjTurnNormalized,sumAdjTunrCost);		
		/**use this for back way crossing edges**/
		//IloNumExpr sumCrossCostGoingAndBack = cplex.sum(sumAdjTunrCost, sumAdjTunrCostBack); 
		//IloNumExpr sumAdjTurnCostFactor = cplex.prod(adjTurnNormalized,sumCrossCostGoingAndBack);
		
		
		
		double bendWeight = 1.0*bendFactor; /*normalized under number of bends*/
		double bendWeightNormalized = bendWeight/b;
		//IloNumExpr sumBendCostNormal =  cplex.prod(1.0/b,sumBendCost);
		IloNumExpr sumBendCostFactor = cplex.prod(bendWeightNormalized,sumBendCost);
//		
		double distWeight = 1.0*distFactor; /*normalized under number of nodes*/
		double distWeightNormalized = distWeight/nR;
		//IloNumExpr sumDistCostNormal = cplex.prod(1.0/nR, cplex.sum(sumXDist, sumYDist ));			
		IloNumExpr sumDistCostFactor = cplex.prod(distWeightNormalized, cplex.sum(sumXDist, sumYDist ));
		
		double dirWeight = 1.5*dirFactor; /*normalized by number of edges*/
		double dirWeightNormalized = dirWeight/mR;
		IloNumExpr sumDirCostFactor = cplex.prod(dirWeightNormalized,sumDirCost);
		
		double dirAdjWeight = 0.1*dirFactor; /*normalized by number of adj edges edges*/
		double dirAdjNormilized = dirAdjWeight/(m-mR);
		
		
		if( sumDirCostAdj != null) {				
			IloNumExpr sumDirAdjCostFactor =  cplex.prod(dirAdjNormilized,sumDirCostAdj); 
			sumDirCostFactor = cplex.sum(sumDirCostFactor, sumDirAdjCostFactor);
		}
		
		double proportionDPWeight = 4*proportionDPFactor;
		double proportionDPWeightNormalized  = proportionDPWeight/mP;
		double proportionAllWeight = 3.5*proportionAllPtsFactor;
		double proportionAllWeightNormalized = proportionAllWeight/mR;

		IloNumExpr sumEdgeProportionFactorDP = cplex.prod(proportionDPWeightNormalized, sumAbsolutEdgeProportionDifferenceDP);
		IloNumExpr sumEdgeProportionAllPtsFactor = cplex.prod(proportionAllWeightNormalized, sumAbsolutEdgeProportionDifference);
		

		
		IloNumExpr objective = cplex.sum(  sumAdjTurnCostFactor, sumDistCostFactor, sumBendCostFactor, sumDirCostFactor, sumEdgeProportionFactorDP, sumEdgeProportionAllPtsFactor) ;
//		IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumBendCostFactor, sumDirCostFactor, sumEdgeProportionFactorDP, sumEdgeProportionFactor, sumAdjTunrCostFactor) ;

		//IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumBendCostFactor, sumDirCostFactor) ;

		//IloNumExpr objective = sumBendCostFactor ;
		//IloNumExpr objective = cplex.sum(sumXDist, sumYDist ) ;

		
		cplex.addMinimize(objective);
		
		if(executionTimeLimit > 0 ){
			double limitTimeInSeconds = ((double)(executionTimeLimit))/1000;
			cplex.setParam(IloCplex.DoubleParam.TiLim, limitTimeInSeconds);
		}
		else if(executionTimeLimit <0 ){
			//cplex.setParam(IloCplex.Param.MIP.Limits.Solutions, 1);
			cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, (-(double)executionTimeLimit)/100.0);
			resultReport.getRouteReport().setGap((-(double)executionTimeLimit)/100.0 );
			
			cplex.setParam(IloCplex.DoubleParam.TiLim, 120);
		}
		

//		if(executionTimeLimit > 0 ){
//			double limitTimeInSeconds = ((double)(executionTimeLimit))/1000;
//			cplex.setParam(IloCplex.DoubleParam.TiLim, limitTimeInSeconds);
//		}
//		else if(executionTimeLimit <0 ){
//			cplex.setParam(IloCplex.Param.MIP.Limits.Solutions, 1);
//		}

		/*control display information*/
		//cplex.setParam(IloCplex.Param.MIP.Display, 1);
		
		if(cplex.solve()){
			
			//System.out.println(allNodeIdList);
			/*Check if there is extra crossings*/
			for (int i = 0; i < m; i++) {
				
				for(int j = i; j < m; j++){
					

					int edge1UIndex = allNodeIdList.indexOf(edgeList.get(i)[0]); 
					int edge1VIndex = allNodeIdList.indexOf(edgeList.get(i)[1]);
//
					int edge2UIndex = allNodeIdList.indexOf(edgeList.get(j)[0]) ;
					int edge2VIndex = allNodeIdList.indexOf(edgeList.get(j)[1]) ;
					

					if(i!=j && edge1UIndex != edge2UIndex && edge1UIndex != edge2VIndex && edge1VIndex != edge2UIndex && edge1VIndex != edge2VIndex ) {
						double e1x1 = cplex.getValue(x[edge1UIndex]);
						double e1y1 = cplex.getValue(y[edge1UIndex]);
						double e1x2 = cplex.getValue(x[edge1VIndex]);
						double e1y2 = cplex.getValue(y[edge1VIndex]);
						
						double e2x1 = cplex.getValue(x[edge2UIndex]);
						double e2y1 = cplex.getValue(y[edge2UIndex]);
						double e2x2 = cplex.getValue(x[edge2VIndex]);
						double e2y2 = cplex.getValue(y[edge2VIndex]);
						
						if(GeometricOperation.intersects(e1x1, e1y1, e1x2, e1y2,
								e2x1, e2y1, e2x2, e2y2)) {
							
							topologyViolation = true;
							System.out.println("Edges violates topology: " + i + " nodes: "  + edgeList.get(i)[0] + " " + edgeList.get(i)[1] + " "
									+ j + " nodes: "  + edgeList.get(j)[0] + " " + edgeList.get(j)[1]);
							
							ArrayList<Integer> edgeTopoViolation = new ArrayList<Integer>();
							edgeTopoViolation.add(i);
							edgeTopoViolation.add(j);
							edgePaarToCheck.add(edgeTopoViolation);
							
						}
						
						
					}
					
				}
			}
			

			for (int i = 0; i < nR; i++) {
				Point2D.Double pt = new Point2D.Double(cplex.getValue(x[i]),cplex.getValue(y[i])); 
				mipLineString.add(pt);
			}	
			ArrayList<Point2D> finalPoints = new ArrayList<Point2D>();
			ArrayList<Point2D> routePathNormalized =  route.getRoutePath().asJava2DList(1);
			finalPoints.add(mipLineString.get(0));
			ArrayList<Integer> relevantNodeIndex = route.getRoutePath().getRelevantPointIndex();
			for(int i = 1; i < relevantNodeIndex.size() ; i++){
				int index = relevantNodeIndex.get(i);
				int lastIndexInFinalPoints = finalPoints.size() - 1;
				/*adicional o proximo ponto relevante entre ele e o anterior na linha reta com os ponts complementares*/
				finalPoints.add( mipLineString.get(i) );
				finalPoints = GeometricOperation.fillLine( finalPoints , routePathNormalized,  index - lastIndexInFinalPoints   , lastIndexInFinalPoints );
			}

			
			route.getRoutePath().updatePathXNodes( finalPoints);
			

			//System.out.println("Only ADj");
		
			/**IMPORTANTA - Uptade X coordinates of Asjacent edges to Route***/
			for (int i = nR; i < n ; i++){
				
				//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
				streetNodeMap.get(allNodeIdList.get(i)).setxGeom(GeoConvertionsOperations.CreateJTSPoint(cplex.getValue(x[i]),cplex.getValue(y[i])));

			}
			for (int i = 0; i < mR; i++) {
				System.out.println("edge"+ i + " =  " + cplex.getValue(dirCost[i]));

			}

			for (int i = 0; i < adjEdgNumber; i++) {
				
				System.out.println("adj edge"+ i + ": original angle with route (going way): " + origBestTurnAdjEdgesGoing[i]  + " angle with route =  " + cplex.getValue(adjBendDirConstraint[i])  + " difference =  " + cplex.getValue(turnDiffFromOriginal[i]) + " cost =  " + cplex.getValue(adJEdgeTurnCost[i])  );
	
				//cplex.addLe(bendDir[i], bendCost[i]);
				//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));

			}
			
//			for(int i = 0; i < n ; i++){
//				//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//
//			}
			
//			for (int i = 0; i < mP; i++){
//				System.out.println("Edge: " + i);
//				System.out.println("AbsDiffX-> =  " + cplex.getValue(absoluteDiffX[i]) + " AbsDiffY-> =  " + cplex.getValue(absoluteDiffY[i]) );
//				System.out.println("True edge proportion: " + edgeProportion.get(i) + "Final edgeProportion: " + cplex.getValue(finalEdgeProportion[i]) + "Absolut Difference: " + cplex.getValue(absolutEdgeProportionDifference[i]));
//			}
//			for (int i = 0; i < b ; i++){
//				System.out.println("benDir-> =  " + cplex.getValue(bendDiff[i]) + " bencost-> =  " + cplex.getValue(bendCost[i]) );
//				//System.out.println("gama0-> =  " + cplex.getValue(gama[i-2][0]) + " gama1-> =  " + cplex.getValue(gama[i-2][1])  + " gama2-> =  " + cplex.getValue(gama[i-2][2]));
//				
//				//System.out.println("origTurnDir->= "+ origTurnDir[i -1] + " benDir-> =  " + cplex.getValue(bendDir[i-1]) + " bencost-> =  " + cplex.getValue(bendCost[i-1]) );
//			}
////			
//			for (int i = 0; i < nR ; i++){
//				System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//			}
//			
//			for (int i = 0; i < mR ; i++){
//				//System.out.println("benDir-> =  " + cplex.getValue(bendDir[i-2]) + " bencost-> =  " + cplex.getValue(bendCost[i-2]) );
//				System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dirDiff-> =  " + cplex.getValue(dir[i][0]) + " dirDiff-> =  " + cplex.getValue(dirDiff[i]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]) );
//				
//			}
//			for (int i = 0; i < nA ; i++){
//				//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//			}

//			for (int i = nR -1; i < m ; i++){
//				
//				System.out.println("x"+ i+1 + " =  " + cplex.getValue(x[i+1]) + " y"+ i+1 + " =  " + cplex.getValue(y[i+1]) + " z1"+ i+1 + " =  " + cplex.getValue(z1[i+1]) + " z2"+ i+1 + " =  " + cplex.getValue(z2[i+1]));
//			//	System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//
//				System.out.println("Route Point: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom() );
//				System.out.println("Adj Point: " +streetNodeMap.get(edgeList.get(i)[1]).getxGeom() );
//				System.out.println("Dist: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom().distance(streetNodeMap.get(edgeList.get(i)[1]).getxGeom()) );
//				System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dir-> =  " + cplex.getValue(dir[i][0]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]));
//				System.out.println();
//			}
			
	
			
			
//			numInterPoints = intersectionNodeIdList.size();
			

			
//			for (int i = 0; i < numInterPoints; i++) {
//				
//
//				int vId = intersectionNodeIdList.get(i);
////				System.out.println();
////				System.out.println("Route relevant point n  " + routeNodeIdList.indexOf(vId) + " of id "+vId+"  is intersection and its degree is " + circularOrderList.get(vId).size() );
////				System.out.println("Interpoint"+i+" Intersection Id: " + vId);
//				
//				for(int j = 0; j < beta[i].length ; j ++ ){
//					int indexDirVU1 = 0, indexDirVU2 = 0;
////					System.out.println("--Edge " + j + " and " + (j+1));
//					int u1Id, u2Id;
//					u1Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j).getId();
////					System.out.println("u1ID="+u1Id); 
//					/*se for vertive una orden circular seleciona u2 como o primeiro da lista*/
//					if(j < (beta[i].length -1))
//						u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j + 1).getId();
//					else 
//						u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(0).getId();
////					System.out.println("u2ID="+u2Id);
//					/***identify the edges (v, u1) and (v, u2)**/
//					boolean foundEdgeToU1 = false, foundEdgeToU2 = false;
//					int edgeToU1IsInverted = 0, edgeToU2IsInverted = 0;
//					
//					for(int k = 0; k < edgeList.size(); k++){
//
//						/*intersectionNOdeid contain only route nodes, the adj node are always edge[1]. Otherwise is necessary to check the inverse*/
//						if(edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u1Id) {
//							indexDirVU1 = k;
//							foundEdgeToU1 = true;
//							edgeToU1IsInverted = 0;
//						}
//						else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u1Id){
//							indexDirVU1 = k;
//							foundEdgeToU1 = true;
//							edgeToU1IsInverted = 1;
//						}
//						if( edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u2Id ){
//							indexDirVU2 = k;
//							foundEdgeToU2 = true;
//							edgeToU2IsInverted = 0;
//						}
//						else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u2Id){
//							indexDirVU2 = k;
//							foundEdgeToU2 = true;
//							edgeToU2IsInverted = 1;
//						}
//						if(foundEdgeToU1 && foundEdgeToU2)
//							break;
//					}
//					
////					System.out.println("EdgeIndex to u1ID= "+ indexDirVU1 + " dir: " + cplex.getValue(dir[indexDirVU1][edgeToU1IsInverted]) + " found: " + foundEdgeToU1  + " is inverted: " + edgeToU1IsInverted); 
////					System.out.println("EdgeIndex to u2ID= "+ indexDirVU2 + " dir: " + cplex.getValue(dir[indexDirVU2][edgeToU2IsInverted]) + " found: " + foundEdgeToU2 + " is inverted: " + edgeToU2IsInverted); 
//			
//
//			
//				}
////				System.out.println();
//			}
			
			
			
			
			
			
			//System.out.println("sumDx =  " + cplex.getValue(sumXDist) + " sumDy =  " + cplex.getValue(sumYDist));
			System.out.println("Direction model is: "  + directionModel);
			System.out.println("Rescaled route lenght: " + rescaledRouteLenght);
			//System.out.println("distCost/Normal =  " + (cplex.getValue(sumXDist) + cplex.getValue(sumYDist)) + " / " +cplex.getValue(sumDistCostNormal) + " sumBendCost/Normal = " + cplex.getValue(sumBendCost)+ " / " +cplex.getValue(sumBendCostNormal) + " sumDirCost/Normal = " + cplex.getValue(sumDirCost) + " / " + cplex.getValue(sumDirCostNormal) + " sumEdgeProportionDPCost/Normal = " + cplex.getValue(sumAbsolutEdgeProportionDifferenceDP)+ " / " + cplex.getValue(sumAbsolutEdgeProportionDifferenceNormalDP) + " sumEdgeProportionDPCost/Normal = " + cplex.getValue(sumAbsolutEdgeProportionDifference)+ " / " + cplex.getValue(sumAbsolutEdgeProportionDifferenceNormal));
			System.out.println("distFactor =  " + distFactor + " bendFactor = " + bendFactor + " dirFactor = " + dirFactor + " proporDPFactor = " + proportionDPFactor + " proporAllPtsFactor = " + proportionDPFactor);
			System.out.println("distCostNormalFactor =  " + cplex.getValue(sumDistCostFactor) + " sumBendCostFactor = " + cplex.getValue(sumBendCostFactor) + " sumDirCostFactor = " + cplex.getValue(sumDirCostFactor) + " sumProportionDPFactor = " + cplex.getValue(sumEdgeProportionFactorDP) + " sumProportionAllPtsFactor = " + cplex.getValue(sumEdgeProportionAllPtsFactor));

			System.out.println("Final = " + cplex.getObjValue());
			resultReport.getRouteReport().setObjectiveFunctionValue(cplex.getObjValue());
			
			resultReport.getRouteReport().setBendSC(new SoftConstraintValues(bendWeight, bendWeightNormalized, cplex.getValue(sumBendCostFactor)));
			resultReport.getRouteReport().setDistanceSC(new SoftConstraintValues(dirWeight, distWeightNormalized, cplex.getValue(sumDistCostFactor)));
			resultReport.getRouteReport().setOrientationSC(new SoftConstraintValues(dirWeight, dirWeightNormalized, cplex.getValue(sumDirCostFactor)));
			resultReport.getRouteReport().setProportionSC(new SoftConstraintValues(proportionAllWeight, proportionAllWeightNormalized, cplex.getValue(sumEdgeProportionAllPtsFactor)));
			resultReport.getRouteReport().setProportionDPSC(new SoftConstraintValues(proportionDPWeight, proportionDPWeightNormalized, cplex.getValue(sumEdgeProportionFactorDP)));
			resultReport.getRouteReport().setNumberOfDP(numDPs);
			
			cplex.end();
			end = System.currentTimeMillis();
			System.out.println("Route Scehmatization- Nodes: " +n+ " Execution Time: "   + (end - start) );
			System.out.println("mipfinalroutepts: " + finalPoints.get(0));
			
			return topologyViolation;
			
		}
		else{
			
			throw new Exception("Cannot solve MIP Model on time");
			
		}
		
			
		
		
		
		
	}

	


	/********########***ROUTE OPTIMIZER ***##############################################
	 * /********########***ROUTE OPTIMIZER ***##############################################
	 * /********########***ROUTE OPTIMIZER ***##############################################

		
		/**
		 * @param streetNodeMap
		 * @param circularOrderList
		 * @param route
		 * @param bendFactor
		 * @param dirFactor
		 * @param distFactor
		 * @param directionModel 0:none , 1:bestdir, 2:trad, 3:klippel
		 * @param executionTimeLimit
		 * @return
		 * @throws IloException
		 * @throws Exception
		 */
		public static ArrayList<Point2D> routeOptimizer4(Map<Integer,StreetNode> streetNodeMap, Map<Integer,ArrayList<StreetNode>> circularOrderList, Route route, LineString rescaledRouteGeom,  
				double bendFactor, double dirFactor, double distFactor, double proportionDPFactor, double proportionAllPtsFactor, double rescaleFactor, int directionModel, 
				double adjVerticesMinDist,  double minNonAdjEdgeDist, double stubFixLength, double LMfixDist, boolean checkTopology, int executionTimeLimit, ResultReport resultReport) throws IloException, Exception  {
			
			long start, end;
			start = System.currentTimeMillis(); 
			
			ArrayList<Point2D> mipLineString = new ArrayList<Point2D>();
			ArrayList<Integer> routeNodeIdList = new ArrayList<Integer>();
			ArrayList<Integer> intersectionNodeIdList = new ArrayList<Integer>();
			ArrayList<Integer> adjacentNodeIdList = new ArrayList<Integer>();
			ArrayList<Integer> allNodeIdList = new ArrayList<Integer>();
			
			ArrayList<Integer[]> edgeList = new ArrayList<Integer[]>();

			ArrayList<Point2D> rescaledRouteNodePositions = GeoConvertionsOperations.JTSGeometryToJavaD2(rescaledRouteGeom);
//			ArrayList<Point2D> rescaledRouteNodePositions = new ArrayList<Point2D>();
//			if(rescaleFactor > 5)
//				rescaledRouteNodePositions = GeoConvertionsOperations.JTSGeometryToJavaD2(GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getDPindex(), rescaleFactor/100));
//			else
//				rescaledRouteNodePositions = route.getRoutePath().asJava2DList(1);
			
			
			ArrayList<Point2D> rescaledRelevantRoutePoints = new ArrayList<Point2D>();
			ArrayList<Point2D> proportionKeepingRescaledRouteDPPoints = new ArrayList<Point2D>();
			ArrayList<Integer> proportionKeepingRescaledRouteDPPointsId = new ArrayList<Integer>();
			
			/**
			 * Reads route path create list of ids for nodes and edges
			 */
			Integer[] edge = new Integer[2];
			edge[0] = route.getRoutePath().getNodeList().get(0).getId();
			routeNodeIdList.add(route.getRoutePath().getNodeList().get(0).getId());
			rescaledRelevantRoutePoints.add(rescaledRouteNodePositions.get(0));

			double rescaledRouteLenght = GeometricOperation.length(rescaledRouteNodePositions);
			
			proportionKeepingRescaledRouteDPPoints.add(rescaledRouteNodePositions.get(0));
			proportionKeepingRescaledRouteDPPointsId.add(route.getRoutePath().getNodeList().get(0).getId());

			
			for(int i = 1 ; i < route.getRoutePath().getNodeList().size(); i++){
				if(route.getRoutePath().getNodeList().get(i).isRelevantRouteNode()){
					
					routeNodeIdList.add(route.getRoutePath().getNodeList().get(i).getId());
					edge[1] = route.getRoutePath().getNodeList().get(i).getId();
					edgeList.add(edge);
					if(i  < route.getRoutePath().getNodeList().size() -1){
						edge = new Integer[2];
						edge[0] = route.getRoutePath().getNodeList().get(i).getId();
					}
					rescaledRelevantRoutePoints.add(rescaledRouteNodePositions.get(i));
				}
				/***Keep proportion between all relevant nodes, or olny between some selected???*/
				if(route.getRoutePath().getNodeList().get(i).isRelevantRouteNode() &&
						(route.getRoutePath().getNodeList().get(i).getDegree() > 2 || route.getRoutePath().getNodeList().get(i).isDecisionPoint() ||   route.getRoutePath().getNodeList().get(i).getTopoRelations().size() > 0 || i == route.getRoutePath().getNodeList().size()-1) ) {
				//if(route.getRoutePath().getNodeList().get(i).isRelevantRouteNode()){
					proportionKeepingRescaledRouteDPPoints.add(rescaledRouteNodePositions.get(i));
					proportionKeepingRescaledRouteDPPointsId.add(route.getRoutePath().getNodeList().get(i).getId());
				}
				/***with or without round about?*/
				if(route.getRoutePath().getNodeList().get(i).getDegree() > 2 
						&& !route.getRoutePath().getNodeList().get(i).isRoundAbout() ){
				//if(route.getRoutePath().getNodeList().get(i).getDegree() > 2 ){
					
					intersectionNodeIdList.add(route.getRoutePath().getNodeList().get(i).getId());
				}

				
			}
			int mR = edgeList.size();
			
			/**
			 * Reads adjacent nodes and add to list of id for nodes and edges
			 */
			for(Integer id: routeNodeIdList){
				for(StreetNode node: circularOrderList.get(id)){
//					if(!node.isRouteNode() && !adjacentNodeIdList.contains(node.getId()) 
//							&& !streetNodeMap.get(id).isRoundAboutStart() &&  !streetNodeMap.get(id).isRoundAboutEnd() ){
					if(!node.isRouteNode()  ){
						if(!adjacentNodeIdList.contains(node.getId())){
							adjacentNodeIdList.add(node.getId());
							edge = new Integer[2];
							edge[0] = id;
							edge[1] = node.getId();
							edgeList.add(edge);
						}
						else System.out.println("ATENTION ADJACENT EDGE with repeated node!! ");
						
					}
				}
			}
			for(Integer nodeId: routeNodeIdList )
				allNodeIdList.add(nodeId);
			for(Integer nodeId: adjacentNodeIdList )
				allNodeIdList.add(nodeId);
			
			//System.out.println("Intersetion size = " +  intersectionNodeIdList.size());
//			for(Integer i : intersectionNodeIdList){
//				System.out.println("Route relevant point n  " + routeNodeIdList.indexOf(i) + " of id "+i+"  is intersection and its degree is " + circularOrderList.get(i).size() );
//			}
			
			int n, nR, nA; /*n number of node, nR number of route nodes, nA number of adjacent nodes*/
			int m; /*number of edges*/
			int b; /*number of bends in a path*/
			
			nR = routeNodeIdList.size();
			nA = adjacentNodeIdList.size();

			n = allNodeIdList.size();
			m = edgeList.size();
			b = nR - 2;
			
			resultReport.getRouteReport().setNumberOfNodes(n);
			resultReport.getRouteReport().setNumberOfEdges(m);
			resultReport.getRouteReport().setNumberOfStubs(nA);
			resultReport.getRouteReport().setNubmerOfRouteNodes(nR);
			resultReport.getRouteReport().setScaleVariation(rescaleFactor/100);
			
			double maxAdjD = 0;
			for (Integer[] e: edgeList){
				double dist = streetNodeMap.get(e[0]).getProjectGeom().distance( streetNodeMap.get(e[1]).getProjectGeom() );
				if(dist > maxAdjD)
					maxAdjD=dist;
				
			}
			
			
			/*********COORDINATES CONSTRAINT*****************/
			
			
			
			
			IloCplex cplex = new IloCplex();
			double extendLimit = 0.2;
			
			OctlinearBoundingBox octBox = new OctlinearBoundingBox(GeoConvertionsOperations.JTSGeometryToJavaD2(rescaledRouteGeom), 3*minNonAdjEdgeDist, extendLimit,2);
			
   
			IloNumVar[] x  = cplex.numVarArray(n, octBox.getMinX(), octBox.getMaxX());
			IloNumVar[] y  = cplex.numVarArray(n,  octBox.getMinY(), octBox.getMaxY());
			IloNumVar[] z1  = cplex.numVarArray(n, octBox.getMinZ1(), octBox.getMaxZ1());
			IloNumVar[] z2  = cplex.numVarArray(n, octBox.getMinZ2(), octBox.getMaxZ2());
			
			
		
			IloNumExpr[] z1Constraint = new IloNumExpr[n];
			IloNumExpr[] z2Constraint = new IloNumExpr[n];
			
			for (int i = 0; i < n; i++) {
				/*do we need to multiply by 2?*/
				z1Constraint[i] = cplex.prod(0.5,cplex.sum(x[i],y[i]));
				z2Constraint[i] = cplex.prod(0.5, cplex.diff(x[i],y[i]));
				
			}
			
			/*Add z constraint to model constranint*/ 
			for (int i = 0; i < n; i++) {
				cplex.addEq(z1[i], z1Constraint[i]);
				cplex.addEq(z2[i], z2Constraint[i]);
				
			}
			/*fix position of first vertex*/
			/*and last?*/
			
//			cplex.addEq(x[0], rescaledRelevantRoutePoints.get(0).getX());
//			cplex.addEq(y[0], rescaledRelevantRoutePoints.get(0).getY());
			
			//cplex.addEq(x[nR-1], streetNodeMap.get(allNodeIdList.get(nR-1)).getProjectGeom().getX());
			//cplex.addEq(y[nR-1], streetNodeMap.get(allNodeIdList.get(nR-1)).getProjectGeom().getY());
			
			/*********OCTALINEARITY CONSTRAINT*****************/
			
			/*sec[m][d][(pred,orig,succ)] defines the sector of the octilinear position an edges could lie
			 * m = number of edges
			 * d = direction of the edge 0 is going and 1 is back
			 * (pred,orig,succ) = 0 lies on the previous sector, 1 lies ont the original best sector, and 2 lies on the succecive sector*/ 
			int[][][] sec = new int[m][2][3];		

			for (int i = 0; i < m; i++) {
				Point pt1 =  streetNodeMap.get(edgeList.get(i)[0]).getProjectGeom();
				Point pt2 =  streetNodeMap.get(edgeList.get(i)[1]).getProjectGeom();
				
				sec[i][0][1] = GeometricOperation.sectorOf(
						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
								pt1.getX(), pt1.getY(),
								pt2.getX(), pt2.getY() 					
								)
						);
				sec[i][0][0] =  Math.floorMod(sec[i][0][1] - 1, 8);
				sec[i][0][2] =  Math.floorMod(sec[i][0][1] + 1, 8);
				
				
				sec[i][1][1] = Math.floorMod(sec[i][0][1] + 4,8);
				sec[i][1][0] = Math.floorMod(sec[i][0][0] + 4,8);
				sec[i][1][2] = Math.floorMod(sec[i][0][2] + 4,8);
			}
			/**
			 * *Boolean variable to restrict the orientation of the edge to orig = 1, succ = 2 and prd = 0*
			 */

			IloNumVar[][] alpha = new IloNumVar[m][];
			
			for (int i = 0; i < m; i++) {
				alpha[i] = cplex.boolVarArray(3);
				for(int j = 0; j < 3; j++)
					alpha[i][j].setName("OctAlphaEdge"+i+"ori"+j);
				
			}

			
			IloLinearNumExpr[] constraintEdgeOrientation = new IloLinearNumExpr[m];
			
		
			/***
			 * Force edeges on decision points alpha[i][1] = 1: original best orientation 
			 * */
			
		
			for (int i = 0; i < m; i++) {
				
				/*If direrection model is best direction of edges incident in DP*/
				if(directionModel == 1 ){
					/*rotue edges starting on DP*/
					if(streetNodeMap.get(edgeList.get(i)[0]).isDecisionPoint() &&
							streetNodeMap.get(edgeList.get(i)[1]).isRouteNode() 
							){
						//System.out.println("DP starting in: " + streetNodeMap.get(edgeList.get(i)[0]).getId() + " end in: " + streetNodeMap.get(edgeList.get(i)[1]).getId() +  " best dir: " + sec[i][0][1] + " invert: " + sec[i][1][1]);
						cplex.addEq(alpha[i][0], 0);
						cplex.addEq(alpha[i][1], 1);
						cplex.addEq(alpha[i][2], 0);
							
					}
					/*route edges ending on DP*/
					if(streetNodeMap.get(edgeList.get(i)[1]).isDecisionPoint() &&
							streetNodeMap.get(edgeList.get(i)[0]).isRouteNode() ){
						
						//System.out.println("DP ending in: " + streetNodeMap.get(edgeList.get(i)[1]).getId() + " start in: " + streetNodeMap.get(edgeList.get(i)[0]).getId() +  " best dir: " + sec[i][0][1] + " invert: " + sec[i][1][1]);
		
						cplex.addEq(alpha[i][0], 0);
						cplex.addEq(alpha[i][1], 1);
						cplex.addEq(alpha[i][2], 0);
					
					}
				}
				constraintEdgeOrientation[i] = cplex.linearNumExpr();
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][0]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][1]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][2]);
				cplex.addEq(constraintEdgeOrientation[i], 1.0);
			}
			
			
			/**
			 * *Variable dir[i][j] defines the direction of of edege i (j means de drirection of the edege*
			 */

			IloNumVar[][] dir  = new IloNumVar[m][]; /*leave the last dimension to be defined with cplex*/
			
			for (int i = 0; i < m; i++) {
				dir[i] = cplex.intVarArray(2, 0, 7);/* array of size 2 because we need uv an vu*/
				if(directionModel == 1 &&  i < mR ){
					Point pt1 =  streetNodeMap.get(edgeList.get(i)[0]).getProjectGeom();
					Point pt2 =  streetNodeMap.get(edgeList.get(i)[1]).getProjectGeom();
					/*rotue edges starting on DP*/
					if(streetNodeMap.get(edgeList.get(i)[0]).isDecisionPoint() &&
							streetNodeMap.get(edgeList.get(i)[1]).isRouteNode() 
							){
//						System.out.println("DP starting in: " + streetNodeMap.get(edgeList.get(i)[0]).getId() + " end in: " + streetNodeMap.get(edgeList.get(i)[1]).getId() +  " best dir: " + sec[i][0][1] + " invert: " + sec[i][1][1]);
//						System.out.println("Anlge: " + Math.toDegrees(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
//										pt1.getX(), pt1.getY(),
//										pt2.getX(), pt2.getY() 					
//										)));
						cplex.addEq( GeometricOperation.sectorOf(
								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
										pt1.getX(), pt1.getY(),
										pt2.getX(), pt2.getY() 					
										) ) , dir[i][0] );
							
					}
					/*route edges ending on DP*/
					if(streetNodeMap.get(edgeList.get(i)[1]).isDecisionPoint() &&
							streetNodeMap.get(edgeList.get(i)[0]).isRouteNode() ){
						
//						System.out.println("DP ending in: " + streetNodeMap.get(edgeList.get(i)[1]).getId() + " start in: " + streetNodeMap.get(edgeList.get(i)[0]).getId() +  " best dir: " + sec[i][0][1] + " invert: " + sec[i][1][1]);
//						System.out.println("Anlge: " + Math.toDegrees(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
//								pt1.getX(), pt1.getY(),
//								pt2.getX(), pt2.getY() 					
//								)));
						cplex.addEq( GeometricOperation.sectorOf(
								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
										pt1.getX(), pt1.getY(),
										pt2.getX(), pt2.getY() 					
										) ) , dir[i][0] );
					
					}
				}
				
			}
			
			
			/*For each i ∈ {pred, orig, succ} we have the following set of constraints
			 dir(u, v)−seciu(v) ≤M(1−αi(u, v))
			−dir(u, v)+seciu(v) ≤M(1−αi(u, v))
			dir(v,u)−seciv(u) ≤M(1−αi(u, v))
			−dir(v,u)+seciv(u) ≤M(1−αi(u, v))
			
			∀{u, v} ∈ E,
			
			 *Here, if αi(u, v) = 0, the constraints in (4) are trivially fulfilled and do not influence the left-hand sides. 
			 *On the other hand, if αi(u, v) = 1, the four inequalities are equivalent to dir(u, v) = seciu(v) and
				dir(v,u) = seciv(u) as desired (equality
			 *
			 *
			 *
			 */
			double MDir = 8; /* |sec - dir| < 8*/

			for (int i = 0; i < m; i++) {
				for (int j = 0; j < 3; j++) {
					IloNumExpr rightSide = cplex.prod(MDir, cplex.diff(1, alpha[i][j]));
					/*natural direction of the edge d=0*/
					IloNumExpr leftSide1 = cplex.diff(dir[i][0], sec[i][0][j]);
					IloNumExpr leftSide2 = cplex.diff(sec[i][0][j],  dir[i][0]);
					/*counter direction of the edge d= 1*/
					IloNumExpr leftSide3 = cplex.diff(dir[i][1], sec[i][1][j]);
					IloNumExpr leftSide4 = cplex.diff(sec[i][1][j],  dir[i][1]);
					
					cplex.addLe(leftSide1, rightSide);
					cplex.addLe(leftSide2, rightSide);
					cplex.addLe(leftSide3, rightSide);
					cplex.addLe(leftSide4, rightSide);
				
				}
				
			
			}
			
			/***contraints to force the correct position of the vertices
			 * if sec original is 2 and alphaoriginal is true then forces x(u) and x(v) to equal
			 * and y(v) > y(u)
			 * 
			 * x(u)−x(v) ≤ M(1−αorig(u, v))
			  −x(u)+x(v) ≤ M(1−αorig(u, v)) 
			  −y(u)+y(v) ≥ −M(1−αorig(u, v))+Luv,
			 * 
			 * 
			 * */
			double MAdjVertice = 2*maxAdjD; /*  D max distance between two adjacent vertices vertices! it was MaxD*/

			/**
			 * dir(u, v) condition 1 condition 2

			 */
//			M = maxExtend*1.5; /*da pra diminuir*/
//			double distMin = maxExtend/500;
			
			for (int i = 0; i < m; i++) {
				
				int v1Index = allNodeIdList.indexOf(edgeList.get(i)[0]);
				int v2Index = allNodeIdList.indexOf(edgeList.get(i)[1]);
				/*the incidentedges have always v1 in route and v2 must the adjancent verts*/
				boolean isIncidentEdge = false;
				double fixDist = adjVerticesMinDist;
				//double fixDist = rescaledRouteLenght/150;
				if(v2Index >= nR){
					isIncidentEdge = true;
					if(streetNodeMap.get(edgeList.get(i)[1]).getIsPointLMNode()>0) {
						fixDist = LMfixDist;
					}
					else if(streetNodeMap.get(edgeList.get(i)[1]).getDegree() == 1) {
						fixDist = stubFixLength;
					}
					else
						//fixDist = stubFixLength;
						fixDist = Math.max(stubFixLength,streetNodeMap.get(edgeList.get(i)[0]).getProjectGeom().distance( streetNodeMap.get(edgeList.get(i)[1]).getProjectGeom() )/rescaleFactor);
				}	
				else
					isIncidentEdge = false;
					/*v2 is one of the the adj. vertex*/
					
						
				
				
				for (int j = 0; j < 3; j++) {
					IloNumExpr rightSide, rightSide2, rightSide3, rightSide4, leftSide1, leftSide2, leftSide3;
					/*do i need to constraint edges for both directions(sec[m][1])?*/
					switch (sec[i][0][j]) {
					case 0:
						
						/**y(u) = y(v)
						x(v)−x(u) ≥ ?uv
						 This is reversed because y axes is reversed!! z2<>z1***/
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						
						
						leftSide1 = cplex.diff(y[v1Index], y[v2Index]);
						leftSide2 = cplex.diff(y[v2Index],  y[v1Index]);
						
						rightSide2 = cplex.diff(adjVerticesMinDist, rightSide);
						leftSide3 = cplex.diff(x[v2Index],  x[v1Index]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						/*condition to fix size incident edges*/
						if(isIncidentEdge){
							rightSide3 =cplex.diff(fixDist, rightSide);
							rightSide4 =cplex.sum(rightSide, fixDist);
							cplex.addGe(leftSide3, rightSide3);
							cplex.addLe(leftSide3, rightSide4);
							
						}	
					
						break;
					case 7:
						/**z1(u) = z1(v)
						z2(v)−z2(u) ≥ 2?uv
						 This is reversed because y axes is reversed!! z2<>z1**/
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[v1Index], z2[v2Index]);
						leftSide2 = cplex.diff(z2[v2Index],  z2[v1Index]);
						
						rightSide2 = cplex.diff(2*adjVerticesMinDist, rightSide);
						leftSide3 = cplex.diff(z1[v2Index],  z1[v1Index]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						
						/*condition to fix size incident edges*/
						if(isIncidentEdge){
							rightSide3 =cplex.diff(fixDist, rightSide);
							rightSide4 =cplex.sum(rightSide, fixDist);
							cplex.addGe(leftSide3, rightSide3);
							cplex.addLe(leftSide3, rightSide4);
							
						}
						break;
						
					case 6:
						/**x(u) = x(v) 
						 * y(u)−y(v) ≥ ?uv
						 * */
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[v1Index], x[v2Index]);
						leftSide2 = cplex.diff(x[v2Index],  x[v1Index]);
						
						rightSide2 = cplex.diff(adjVerticesMinDist, rightSide);
						leftSide3 = cplex.diff(y[v2Index],  y[v1Index]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						
						/*condition to fix size incident edges*/
						if(isIncidentEdge){
							rightSide3 =cplex.diff(fixDist, rightSide);
							rightSide4 =cplex.sum(rightSide, fixDist);
							cplex.addGe(leftSide3, rightSide3);
							cplex.addLe(leftSide3, rightSide4);
							
						}

						break;
						
					case 5:
						/**z2(u) = z2(v) 
						 * z1(u)−z1(v) ≥ 2?uv
						 *  This is reversed because y axes is reversed!! z2<>z1
						 * */
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[v1Index], z1[v2Index]);
						leftSide2 = cplex.diff(z1[v2Index],  z1[v1Index]);
						
						rightSide2 = cplex.diff(2*adjVerticesMinDist, rightSide);
						leftSide3 = cplex.diff(z2[v1Index],  z2[v2Index]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						
						/*condition to fix size incident edges*/
						if(isIncidentEdge){
							rightSide3 =cplex.diff(fixDist, rightSide);
							rightSide4 =cplex.sum(rightSide, fixDist);
							cplex.addGe(leftSide3, rightSide3);
							cplex.addLe(leftSide3, rightSide4);
							
						}
						break;	
					case 4:
						/**y(u) = y(v) 
						 * x(u)−x(v) ≥ ?uv
						 * */
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(y[v1Index], y[v2Index]);
						leftSide2 = cplex.diff(y[v2Index],  y[v1Index]);
						
						rightSide2 = cplex.diff(adjVerticesMinDist, rightSide);
						leftSide3 = cplex.diff(x[v1Index],  x[v2Index]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						
						/*condition to fix size incident edges*/
						if(isIncidentEdge){
							rightSide3 =cplex.diff(fixDist, rightSide);
							rightSide4 =cplex.sum(rightSide, fixDist);
							cplex.addGe(leftSide3, rightSide3);
							cplex.addLe(leftSide3, rightSide4);
							
						}
						break;
					case 3:
						/**z1(u) = z1(v) 
						 * z2(u)−z2(v) ≥ 2?uv
						 This is reversed because y axes is reversed!! z2<>z1
						 * */
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[v1Index], z2[v2Index]);
						leftSide2 = cplex.diff(z2[v2Index],  z2[v1Index]);
						
						rightSide2 = cplex.diff(2*adjVerticesMinDist, rightSide);
						leftSide3 = cplex.diff(z1[v1Index],  z1[v2Index]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						
						/*condition to fix size incident edges*/
						if(isIncidentEdge){
							rightSide3 =cplex.diff(fixDist, rightSide);
							rightSide4 =cplex.sum(rightSide, fixDist);
							cplex.addGe(leftSide3, rightSide3);
							cplex.addLe(leftSide3, rightSide4);
							
						}
						break;	
					case 2:
						/**x(u) = x(v) 
						 * y(v)−y(u) ≥ ?uv
						 * */
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[v1Index], x[v2Index]);
						leftSide2 = cplex.diff(x[v2Index],  x[v1Index]);
						
						rightSide2 = cplex.diff(adjVerticesMinDist, rightSide);
						leftSide3 = cplex.diff(y[v1Index],  y[v2Index]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						
						/*condition to fix size incident edges*/
						if(isIncidentEdge){
							rightSide3 =cplex.diff(fixDist, rightSide);
							rightSide4 =cplex.sum(rightSide, fixDist);
							cplex.addGe(leftSide3, rightSide3);
							cplex.addLe(leftSide3, rightSide4);
							
						}

						break;	
					case 1:
						/**z2(u) = z2(v)
						 * z1(v)−z1(u) ≥ 2?uv
						 *  This is reversed because y axes is reversed!! z2<>z1
						 * */
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[v1Index], z1[v2Index]);
						leftSide2 = cplex.diff(z1[v2Index],  z1[v1Index]);
						
						rightSide2 = cplex.diff(2*adjVerticesMinDist, rightSide);
						leftSide3 = cplex.diff(z2[v2Index],  z2[v1Index]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						
						/*condition to fix size incident edges*/
						if(isIncidentEdge){
							rightSide3 =cplex.diff(fixDist, rightSide);
							rightSide4 =cplex.sum(rightSide, fixDist);
							cplex.addGe(leftSide3, rightSide3);
							cplex.addLe(leftSide3, rightSide4);
							
						}
						break;	
					default:
						break;
					}
					
				}
				
			}
			

			
			
			/***BEND DIFF (TURN ANGLE _ USED FOR DIRECTION MODEL AND CIRCULAR ORDER)***/
			
			double MBend = 15; /*diference between 2 dir */
			/* bendDir[1] = dir[0] - dir[1]* bends angles (-7 to 7)*/
			IloNumVar[] bendDiff = cplex.intVarArray(b, -7, 7);

			for (int i = 0; i < b; i++) {
				//routeNodeIdList.get(i+1)
				cplex.addEq(bendDiff[i], cplex.diff(dir[i][0], dir[i + 1][0]));
			}
			
			
//			/***FORCE CORRECT CHANGE OF DIRECTION ONF DECISION POINTS***/
			if(directionModel == 2 || directionModel == 3){
				IloNumVar[][] epsilon = new IloNumVar[b][];

				for (int i = 0; i < b; i++) {
					epsilon[i] = cplex.boolVarArray(2);
					epsilon[i][0].setName("epsiloDPDIR0");
					epsilon[i][1].setName("epsiloDPDIR1");
				}

				IloLinearNumExpr[] constraintBenDirectionDPBooleans = new IloLinearNumExpr[b];
				for (int i = 0; i < b; i++) {
					constraintBenDirectionDPBooleans[i] = cplex.linearNumExpr();
					constraintBenDirectionDPBooleans[i].addTerm(1.0,epsilon[i][0]);
					constraintBenDirectionDPBooleans[i].addTerm(1.0,epsilon[i][1]);
					cplex.addEq(constraintBenDirectionDPBooleans[i], 1.0);
				}



				for (int i = 0; i < b; i++) {
					cplex.addGe(bendDiff[i], cplex.diff(0 , cplex.prod(MBend, epsilon[i][0])));
					cplex.addLe(bendDiff[i], cplex.diff(cplex.prod(MBend, epsilon[i][1]),  1));

				}
				/*BendDir is turn direction
				 * 0 => 0-straight , 1=>315-veer right, 2=> 270-right, 3=>225sharp right, 
				 * 4=>180backturn, 5=>135-sharp left, 6=> 90left, 7=>45veer left
				 * */
				IloNumVar[] bendDir = cplex.intVarArray(b, 0, 7);
				IloLinearNumExpr[] bendDirConstraint =  new IloLinearNumExpr[b];
				for (int i = 0; i < b; i++) {
					bendDirConstraint[i] = cplex.linearNumExpr();
					bendDirConstraint[i].addTerm(1, bendDiff[i]);
					bendDirConstraint[i].addTerm(8, epsilon[i][0]);
					cplex.addEq(bendDir[i], bendDirConstraint[i] );		
					//cplex.addLe(bendDir[i], bendCost[i]);
					//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));

				}

				int[] origTurnDir = new int[b];
				int dpindex = 0;
				for(int i = 0; i < b; i++){ 
					Point pt1 =  streetNodeMap.get(routeNodeIdList.get(i)).getProjectGeom();
					Point pt2 =  streetNodeMap.get(routeNodeIdList.get(i + 1)).getProjectGeom();
					Point pt3 =  streetNodeMap.get(routeNodeIdList.get(i + 2)).getProjectGeom();
					
					/*Just to check algles at decision points*/
//					if(streetNodeMap.get(routeNodeIdList.get(i + 1)).isDecisionPoint()){
//						System.out.println("DP: " + dpindex++);
//						System.out.println("in e dge: "  + Math.toDegrees(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() )));
//						System.out.println("out edge: "  + Math.toDegrees(GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() )));
//						System.out.println("Turn angle: " + Math.toDegrees(GeometricOperation.getAngleBetweenVectors(pt2.getX(), pt2.getY(), pt1.getX(), pt1.getY(), pt3.getX(), pt3.getY()) ));
//						System.out.println("Best zone Traditional " + GeometricOperation.turnDirectionOf(
//								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
//								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
//								0));
//						System.out.println("Best zone KP " + GeometricOperation.turnDirectionOf(
//								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
//								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
//								2));
//						
//						
//					}
					
					if(directionModel == 2){
						origTurnDir[i] = GeometricOperation.turnDirectionOf(
								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
								0);
					}
					else if(directionModel == 3){
						origTurnDir[i] = GeometricOperation.turnDirectionOf(
								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
								GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
								1);
					}

				}


				for (int i = 0; i < b; i++) {
					if(streetNodeMap.get(routeNodeIdList.get(i + 1)).isDecisionPoint()){
						cplex.addEq(bendDir[i] ,origTurnDir[i] );
						
						
					}
				}

			}
			
			
			/***CIRCULAR ORDER CONSTRAINTS********/
			double MCircularOrder = 17; /*Diference between 2 dir */
			int numInterPoints = intersectionNodeIdList.size();
			IloNumVar[][] beta = new IloNumVar[numInterPoints][];
			IloLinearNumExpr[] constraintCircularOrder = new IloLinearNumExpr[numInterPoints];
			for (int i = 0; i < numInterPoints; i++) {
				
				int degree = circularOrderList.get(intersectionNodeIdList.get(i)).size();
				
				beta[i] = cplex.boolVarArray(degree);
				for(int j = 0 ; j < degree; j++)
					beta[i][j].setName("BetaIntersect"+i+"deg"+j);
				constraintCircularOrder[i] = cplex.linearNumExpr();
				
				/**only one of the beta[i] for each v incident holds 1**/
				for(int j = 0; j < degree; j ++ )			
					constraintCircularOrder[i].addTerm(1, beta[i][j]);
				cplex.addEq(constraintCircularOrder[i], 1.0,  "Intersec"+i+"BetaCircOrder");	
				
			}
			
			for (int i = 0; i < numInterPoints; i++) {
				

				int vId = intersectionNodeIdList.get(i);
//				System.out.println();
//				System.out.println("Route relevant point n  " + routeNodeIdList.indexOf(vId) + " of id "+vId+"  is intersection and its degree is " + circularOrderList.get(vId).size() );
//				System.out.println("Interpoint"+i+" Intersection Id: " + vId);
				IloNumExpr rightSide, leftSide;
				
				for(int j = 0; j < beta[i].length ; j ++ ){
					int indexDirVU1 = 0, indexDirVU2 = 0;
//					System.out.println("--Edge " + j + " and " + (j+1));
					int u1Id, u2Id;
					u1Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j).getId();
//					System.out.println("u1ID="+u1Id); 
					/*se for vertive um orden circular seleciona u2 como o primeiro da lista*/
					if(j < (beta[i].length -1))
						u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j + 1).getId();
					else 
						u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(0).getId();
//					System.out.println("u2ID="+u2Id);
					/***identify the edges (v, u1) and (v, u2)**/
					boolean foundEdgeToU1 = false, foundEdgeToU2 = false;
					int edgeToU1IsInverted = 0, edgeToU2IsInverted = 0;
					
					for(int k = 0; k < edgeList.size(); k++){

						/*intersectionNOdeid contain only route nodes, the adj node are always edge[1]. Otherwise is necessary to check the inverse*/
						if(edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u1Id) {
							indexDirVU1 = k;
							foundEdgeToU1 = true;
							edgeToU1IsInverted = 0;
						}
						else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u1Id){
							indexDirVU1 = k;
							foundEdgeToU1 = true;
							edgeToU1IsInverted = 1;
						}
						if( edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u2Id ){
							indexDirVU2 = k;
							foundEdgeToU2 = true;
							edgeToU2IsInverted = 0;
						}
						else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u2Id){
							indexDirVU2 = k;
							foundEdgeToU2 = true;
							edgeToU2IsInverted = 1;
						}
						if(foundEdgeToU1 && foundEdgeToU2)
							break;
					}
					
//					System.out.println("EdgeIndex to u1ID= "+ indexDirVU1 + " found: " + foundEdgeToU1  + " is inverted: " + edgeToU1IsInverted); 
//					System.out.println("EdgeIndex to u2ID= "+ indexDirVU2 + " found: " + foundEdgeToU2 + " is inverted: " + edgeToU2IsInverted); 

					leftSide = cplex.diff(dir[indexDirVU2][edgeToU2IsInverted], dir[indexDirVU1][edgeToU1IsInverted]);
					rightSide = cplex.diff(1,  cplex.prod(MCircularOrder, beta[i][j]));
					cplex.addGe(leftSide, rightSide, "CircOrderIneqIntersec"+i+"Adja"+j);
					


				}
				//System.out.println(); 


			}
			
			/*******CHECK SELF PLANARITY*******///
			
			System.out.println("checkin topology: " + checkTopology);
			if(checkTopology){
				IloNumVar[][][] gama = new IloNumVar[m][m][];
				/*RElation of m edges from the path and r route edges check relative position to 8 orientations ∈{N,S,E,W,NE,NW,SE,SW}*/
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < m; j++)
						gama[i][j] = cplex.boolVarArray(8);
				
			
			
				IloLinearNumExpr[][] constraintEdgeTopology = new IloLinearNumExpr[m][m];
				/*SumOF{ γi(e1, e2) } ≥ 1   
				 * i∈{N,S,E,W,NE,NW,SE,SW} */
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < m; j++){
						//				for (int k = 0; k < 8; k++){
						//					if(k!=0)
						//						cplex.addEq(gama[i][j][k], 0);
						//					else
						//						cplex.addEq(gama[i][j][k], 1);
						//				}


						constraintEdgeTopology[i][j] = cplex.linearNumExpr();
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][0]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][1]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][2]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][3]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][4]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][5]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][6]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][7]);
						cplex.addGe(constraintEdgeTopology[i][j], 1.0);

					}


				for (int i = 0; i < m; i++) {
					/**I stoped here**/

					
					for(int j = i; j < m  ; j++){
						


						int edge1UIndex = allNodeIdList.indexOf(edgeList.get(i)[0]); 
						int edge1VIndex = allNodeIdList.indexOf(edgeList.get(i)[1]);
//
						int edge2UIndex = allNodeIdList.indexOf(edgeList.get(j)[0]) ;
						int edge2VIndex = allNodeIdList.indexOf(edgeList.get(j)[1]) ;

						/*condition to guarantee that path edege i and j are not adjacents*/
						if(i!=j && edge1UIndex != edge2UIndex && edge1UIndex != edge2VIndex && edge1VIndex != edge2UIndex && edge1VIndex != edge2VIndex )
							for(int k =0; k < 8; k ++){
								/*whats the value of M(max X ou y dist of points) and dist Min*/ 
								double MTopology = rescaledRouteLenght; /*da pra diminuir*/
								IloNumExpr rightSide,  leftSide1, leftSide2, leftSide3, leftSide4;


								switch (k) {
								case 0: /* E (e2(route edge) ist east of e1 (path edge)) : 0 : x*/

									/*
									 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
									 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
									 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									 * ∀(e1,e2) no incident
									 */
									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(x[edge1UIndex], x[edge2UIndex]);
									leftSide2 = cplex.diff(x[edge1UIndex], x[edge2VIndex]);
									leftSide3 = cplex.diff(x[edge1VIndex], x[edge2UIndex] );
									leftSide4 = cplex.diff(x[edge1VIndex], x[edge2VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 1: /* e2 (route edge) is NE (SE because y is inverted) of e1: 45 (315) : z1 */

									/*Can I use 
									 * route.get(j).getX() + route.get(j).getY() == (route.get(j).getX() / Math.sin(Math.toRadians(45)));
									 */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(z1[edge1UIndex], z1[edge2UIndex]);
									leftSide2 = cplex.diff(z1[edge1UIndex], z1[edge2VIndex]);
									leftSide3 = cplex.diff(z1[edge1VIndex], z1[edge2UIndex]);
									leftSide4 = cplex.diff(z1[edge1VIndex], z1[edge2VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);


									break;
								case 2: /* e2 (route edge) is N (S because y is inverted) of e1: 90 (270) : y */
									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff(y[edge1UIndex], y[edge2UIndex]);
									leftSide2 = cplex.diff(y[edge1UIndex], y[edge2VIndex]);
									leftSide3 = cplex.diff(y[edge1VIndex], y[edge2UIndex]);
									leftSide4 = cplex.diff(y[edge1VIndex], y[edge2VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;	
								case 3: /* e2 (route edge) is NW (SW because y is inverted) of e1: 135 (225) : -z2 */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
									leftSide2 = cplex.diff( z2[edge2VIndex],  z2[edge1UIndex] );
									leftSide3 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
									leftSide4 = cplex.diff( z2[edge2VIndex],  z2[edge1VIndex] );

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 4: /*  e2 (route edge) is W  of e1:  180:  -x */

									//						/*
									//						 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
									//						 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									//						 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
									//						 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									//						 * ∀(e1,e2) no incident
									//						 */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff(x[edge2UIndex] , x[edge1UIndex] );
									leftSide2 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );
									leftSide3 = cplex.diff(x[edge2UIndex] , x[edge1VIndex] );
									leftSide4 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 5: /*  e2 (route edge) is SW (NW because y is inverted) of e1: 225 (135): -z1 */


									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff( z1[edge2UIndex] , z1[edge1UIndex]);
									leftSide2 = cplex.diff( z1[edge2VIndex] , z1[edge1UIndex]);
									leftSide3 = cplex.diff( z1[edge2UIndex] , z1[edge1VIndex]);
									leftSide4 = cplex.diff( z1[edge2VIndex] , z1[edge1VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 6:/*  e2 (route edge) is S (N because y is inverted) of e1: 270 (90)  : -y */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(  y[edge2UIndex] ,y[edge1UIndex] );
									leftSide2 = cplex.diff(  y[edge2VIndex], y[edge1UIndex]);
									leftSide3 = cplex.diff(  y[edge2UIndex] ,y[edge1VIndex] );
									leftSide4 = cplex.diff(  y[edge2VIndex], y[edge1VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 7:/*  e2 (route edge) is SE (NE because y is inverted) of e1: 315 (45)  : z2*/

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(z2[edge1UIndex], z2[edge2UIndex] );
									leftSide2 = cplex.diff(z2[edge1UIndex], z2[edge2VIndex] );
									leftSide3 = cplex.diff(z2[edge1VIndex], z2[edge2UIndex] );
									leftSide4 = cplex.diff(z2[edge1VIndex], z2[edge2VIndex] );

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);


									break;
								default:
									break;
								}

							}

					}

				}

			}
			
			
			
			
			/****Intersection Angles difference Minimization
			 * REquires Rvision****/
			
//			MBend = 15; /*diference between 2 dir */
//			/* bendDir[1] = dir[0] - dir[1]* bends angles (-7 to 7)*/
//			int adjEdgNumber = adjacentNodeIdList.size();
//			adjEdgNumber = m - mR;
//			IloNumVar[] turnAdjEdges = cplex.intVarArray(adjEdgNumber, -7, 7);
//			int[] origBestTurnAdjEdges = new int[adjEdgNumber];
//
//			/*seleciona vertices de interseccao*/
//			for (int i = 0; i < adjEdgNumber; i++) {
//				int uId, vId, wId;
//				
//				wId = adjacentNodeIdList.get(i);
//				int indexDirVW =0;
//				vId = 0;
//				boolean achouIndexDirVW = false;
//				for(int j = mR; j< m; j++) {
//					if(edgeList.get(j)[1] ==  wId) {
//						vId = edgeList.get(j)[0];
//						indexDirVW = j;
//						achouIndexDirVW = true;
//						break;
//						
//						
//					}
//					
//				}
//				int indexDirUV = 0;
//				uId= 0;
//				/*in route edge direction*/
//				boolean achouIndexDirUV = false;
//				for(int j = 0; j< mR; j++) {
//					if(edgeList.get(j)[1] == vId ) {
//						indexDirUV = j;
//						uId=edgeList.get(j)[0];
//						achouIndexDirUV = true;
//						break;
//					}
//				}
//				if(achouIndexDirUV && achouIndexDirVW && streetNodeMap.get(wId).getDegree() == 1) {
//					cplex.addEq(turnAdjEdges[i], cplex.diff(dir[indexDirUV][0], dir[indexDirVW][0]));
//					Point pt1 =  streetNodeMap.get(uId).getProjectGeom();
//					Point pt2 =  streetNodeMap.get(vId).getProjectGeom();
//					Point pt3 =  streetNodeMap.get(wId).getProjectGeom();
//					origBestTurnAdjEdges[i] = GeometricOperation.turnDirectionOf(
//							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY() ), 
//							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( pt2.getX(), pt2.getY(), pt3.getX(), pt3.getY() ), 
//							0);
//				}
//				else {
//					cplex.addEq(turnAdjEdges[i], 0);
//					origBestTurnAdjEdges[i] = 0;
//				}
//			}	
//			
////			
////			/*espstion to get absolut value of turn adj edges*/
//			IloNumVar[][] epsilon2 = new IloNumVar[adjEdgNumber][];
//
//			for (int i = 0; i < adjEdgNumber; i++) {
//				epsilon2[i] = cplex.boolVarArray(2);
//				epsilon2[i][0].setName("epsilo2AdjIR0");
//				epsilon2[i][1].setName("epsilo2AdjIR1");
//			}
//
//			IloLinearNumExpr[] constraintEpsilon2 = new IloLinearNumExpr[adjEdgNumber];
//			for (int i = 0; i < adjEdgNumber; i++) {
//				constraintEpsilon2[i] = cplex.linearNumExpr();
//				constraintEpsilon2[i].addTerm(1.0,epsilon2[i][0]);
//				constraintEpsilon2[i].addTerm(1.0,epsilon2[i][1]);
//				cplex.addEq(constraintEpsilon2[i], 1.0);
//			}
////
////
////
//			for (int i = 0; i < adjEdgNumber; i++) {
//				cplex.addGe(turnAdjEdges[i], cplex.diff(0 , cplex.prod(MBend, epsilon2[i][0])));
//				cplex.addLe(turnAdjEdges[i], cplex.diff(cplex.prod(MBend, epsilon2[i][1]),  1));
//
//			}
////			/*absolut tunr direction*/
////			/*BendDir is turn direction
////			 * 0 => 0-straight , 1=>315-veer right, 2=> 270-right, 3=>225sharp right, 
////			 * 4=>180backturn, 5=>135-sharp left, 6=> 90left, 7=>45veer left
////			 * */
//			IloNumVar[] absolutTunrAdjEdges = cplex.intVarArray(adjEdgNumber, 0, 7);
//			IloLinearNumExpr[] adjBendDirConstraint =  new IloLinearNumExpr[adjEdgNumber];
//			for (int i = 0; i < adjEdgNumber; i++) {
//				adjBendDirConstraint[i] = cplex.linearNumExpr();
//				adjBendDirConstraint[i].addTerm(1, turnAdjEdges[i]);
//				adjBendDirConstraint[i].addTerm(8, epsilon2[i][0]);
//				cplex.addEq(absolutTunrAdjEdges[i], adjBendDirConstraint[i] );		
//				//cplex.addLe(bendDir[i], bendCost[i]);
//				//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));
//
//			}
//			/*Difference from original turn and resulting one*/
//			IloNumVar[] turnToOriginalDiff  = cplex.numVarArray(adjEdgNumber, -7, 7);
//			for (int i = 0; i < adjEdgNumber; i++) {
//				cplex.addEq(turnToOriginalDiff[i], cplex.diff(absolutTunrAdjEdges[i] ,origBestTurnAdjEdges[i]));
//			}
//			IloNumVar[][] zeta2 = new IloNumVar[adjEdgNumber][];
//			
//			for (int i = 0; i < adjEdgNumber; i++) {
//				zeta2[i] = cplex.boolVarArray(3);
//				zeta2[i][0].setName("ZETA2AdjTunrDiff0");
//				zeta2[i][1].setName("ZETA2AdjTunrDiff1");
//				zeta2[i][2].setName("ZETA2AdjTunrDiff2");
//				
//			}
//			IloLinearNumExpr[] constraintZeta2 = new IloLinearNumExpr[adjEdgNumber];
//			for (int i = 0; i < adjEdgNumber; i++) {
//				constraintZeta2[i] = cplex.linearNumExpr();
//				constraintZeta2[i].addTerm(1.0,zeta2[i][0]);
//				constraintZeta2[i].addTerm(1.0,zeta2[i][1]);
//				constraintZeta2[i].addTerm(1.0,zeta2[i][2]);
//				cplex.addEq(constraintZeta2[i], 2.0);
//			}
////
//			MDir = 8;
//			for (int i = 0; i < adjEdgNumber; i++) {
//				cplex.addLe(turnToOriginalDiff[i], cplex.diff(cplex.prod(MDir, zeta2[i][0]), 5));
//				cplex.addGe(turnToOriginalDiff[i], cplex.diff(5 , cplex.prod(MDir, zeta2[i][1])));
//				cplex.addLe(turnToOriginalDiff[i], cplex.sum(4 , cplex.prod(MDir, zeta2[i][2])));
//				cplex.addGe(turnToOriginalDiff[i], cplex.diff(-4 , cplex.prod(MDir, zeta2[i][2])));
//				
//			}
//			/*costs vary from 0 to 4, same approach for bendcost of dir cos*/
//			IloNumVar[] adJEdgeTurnCost = cplex.intVarArray(adjEdgNumber, 0, 4);
//			IloLinearNumExpr[] adJEdgeTurnCostConstraint =  new IloLinearNumExpr[adjEdgNumber];
//			for (int i = 0; i < adjEdgNumber; i++) {
//				adJEdgeTurnCostConstraint[i] = cplex.linearNumExpr();
//				adJEdgeTurnCostConstraint[i].addTerm(1, turnToOriginalDiff[i]);
//				adJEdgeTurnCostConstraint[i].addTerm(-8, zeta2[i][0]);
//				adJEdgeTurnCostConstraint[i].addTerm(8, zeta2[i][1]);
//				cplex.addGe(adJEdgeTurnCostConstraint[i], cplex.prod(-1, adJEdgeTurnCost[i]) );
//				cplex.addLe(adJEdgeTurnCostConstraint[i], adJEdgeTurnCost[i] );
//
//				
//			}
//			IloNumExpr sumAdjTunrCost =  cplex.sum(adJEdgeTurnCost);  
//			IloNumExpr sumAdjTunrCostFactor = cplex.prod((1000),sumAdjTunrCost);
	
			
			
			
			
			/*********BEND MINIMIZATION*****************/

			/*Boolean variable to make condition of bends angle <-5, >-4 e <4, ou >5 measure bends angles (-7 to 7)*/
			/***∆benddiff(u, v,w) = dir(u, v) − dir(v,w)***/
			/***∆benddiff(u, v,w) ranges from −7 to 7****/
			
//			bend(u, v,w) = ?|∆dir(u, v,w)| if |∆dir(u, v,w)| ≤ 4 
//							8−|∆dir(u, v,w)| if |∆dir(u, v,w)| ≥ 5.
					
			MBend = 15;
			//M = 8;
			IloNumVar[][] delta = new IloNumVar[b][];
			
			for (int i = 0; i < b; i++) {
				delta[i] = cplex.boolVarArray(3);
				delta[i][0].setName("DeltaBendMin0");
				delta[i][1].setName("DeltaBendMin1");
				delta[i][2].setName("DeltaBendMin2");
				
			}
			IloLinearNumExpr[] constraintBenMinimizationBooleans = new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				constraintBenMinimizationBooleans[i] = cplex.linearNumExpr();
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][0]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][1]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][2]);
				cplex.addEq(constraintBenMinimizationBooleans[i], 2);
			}

			
			for (int i = 0; i < b; i++) {
				cplex.addLe(bendDiff[i], cplex.diff(cplex.prod(MBend, delta[i][0]), 5));
				cplex.addGe(bendDiff[i], cplex.diff(5 , cplex.prod(MBend, delta[i][1])));
				cplex.addLe(bendDiff[i], cplex.sum(4 , cplex.prod(MBend, delta[i][2])));
				cplex.addGe(bendDiff[i], cplex.diff(-4 , cplex.prod(MBend, delta[i][2])));
				
			}
			
			IloNumVar[] bendCost = cplex.intVarArray(b, 0, 4);
			IloLinearNumExpr[] bendCostConstraint =  new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				
				if(!streetNodeMap.get(routeNodeIdList.get(i + 1)).isDecisionPoint()){
					bendCostConstraint[i] = cplex.linearNumExpr();
					bendCostConstraint[i].addTerm(1, bendDiff[i]);
					bendCostConstraint[i].addTerm(-8, delta[i][0]);
					bendCostConstraint[i].addTerm(8, delta[i][1]);
					cplex.addGe(bendCostConstraint[i], cplex.prod(-1, bendCost[i]) );
					cplex.addLe(bendCostConstraint[i], bendCost[i] );
				}
				else
					cplex.addEq(bendCost[i], 0);
				
				
				//cplex.addLe(bendDir[i], bendCost[i]);
				//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));
				
			}

			//System.out.println("factor= " + factor);
			IloNumExpr sumBendCost =  cplex.sum(bendCost);

			
			
			/****Edges orientation difference minimization*********/
			IloNumVar[] dirDiff  = cplex.numVarArray(m, -7, 7);
			for (int i = 0; i < m; i++) {
				cplex.addEq(dirDiff[i], cplex.diff(sec[i][0][1] ,dir[i][0]));
			}
			
//			IloNumVar[][] zetaBinary = new IloNumVar[m][];
//			
//			for (int i = 0; i < m; i++) {
//				zetaBinary[i] = cplex.boolVarArray(2);
//				
//			}
//			
//			IloLinearNumExpr[] constraintZetaBinary = new IloLinearNumExpr[m];
//			for (int i = 0; i < m; i++) {
//				constraintZetaBinary[i] = cplex.linearNumExpr();
//				constraintZetaBinary[i].addTerm(1.0,zetaBinary[i][0]);
//				constraintZetaBinary[i].addTerm(1.0,zetaBinary[i][1]);
//				cplex.addEq(constraintZetaBinary[i], 1.0);
//			}
//			MDir = 8;
//			for (int i = 0; i < m; i++) {
//				IloNumExpr rightSide = cplex.prod(MDir, cplex.diff(1, zetaBinary[i][0]));
//				cplex.addGe(dirDiff[i], rightSide);
//				cplex.addLe(dirDiff[i], cplex.negative(rightSide));
//				
//			}
//			IloNumVar[] dirCost = cplex.intVarArray(mR, 0, 2);
//			for (int i = 0; i < mR; i++) {
//				cplex.addEq(dirCost[i], zetaBinary[i][1]);
//			}
//			
//			IloNumVar[] dirCostAdj = cplex.boolVarArray(m-mR);
//			for (int i = mR; i < m; i++) {
//				cplex.addEq(dirCostAdj[i-mR], zetaBinary[i][1]);
//			}

			IloNumVar[] dirCost = cplex.boolVarArray(mR);
			for (int i = 0; i < mR; i++) {

				cplex.addLe(dirDiff[i],cplex.prod(8, dirCost[i]));
				cplex.addGe(dirDiff[i],  cplex.prod(-8, dirCost[i]));
			}
			IloNumExpr sumDirCost =  cplex.sum(dirCost); 
			
			
			IloNumExpr sumDirCostAdj = null;
			if(m > mR) {
				IloNumVar[] dirCostAdj = cplex.boolVarArray(m-mR);
				for (int i = mR; i < m; i++) {
					cplex.addLe(dirDiff[i],cplex.prod(8, dirCostAdj[i-mR]));
					cplex.addGe(dirDiff[i], cplex.prod(-8, dirCostAdj[i-mR]));
				}
				sumDirCostAdj  =  cplex.sum(dirCostAdj);
			}

//			
			
			
			
//			IloNumVar[][] zeta = new IloNumVar[m][];
//			
//			for (int i = 0; i < m; i++) {
//				zeta[i] = cplex.boolVarArray(3);
//				zeta[i][0].setName("ZETADirMin0");
//				zeta[i][1].setName("ZETADirMin1");
//				zeta[i][2].setName("ZETADirMin2");
//				
//			}
//			IloLinearNumExpr[] constraintDirDiffMinimizationBooleans = new IloLinearNumExpr[m];
//			for (int i = 0; i < m; i++) {
//				constraintDirDiffMinimizationBooleans[i] = cplex.linearNumExpr();
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][0]);
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][1]);
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][2]);
//				cplex.addEq(constraintDirDiffMinimizationBooleans[i], 2.0);
//			}
//
//			MDir = 8;
//			for (int i = 0; i < m; i++) {
//				cplex.addLe(dirDiff[i], cplex.diff(cplex.prod(MDir, zeta[i][0]), 5));
//				cplex.addGe(dirDiff[i], cplex.diff(5 , cplex.prod(MDir, zeta[i][1])));
//				cplex.addLe(dirDiff[i], cplex.sum(4 , cplex.prod(MDir, zeta[i][2])));
//				cplex.addGe(dirDiff[i], cplex.diff(-4 , cplex.prod(MDir, zeta[i][2])));
//				
//			}
//			IloNumVar[] dirCost = cplex.intVarArray(mR, 0, 4);
//			IloNumVar[] dirCostAdj = cplex.intVarArray(m-mR, 0, 4);
//			IloLinearNumExpr[] dirCostConstraint =  new IloLinearNumExpr[m];
//			for (int i = 0; i < mR; i++) {
//				dirCostConstraint[i] = cplex.linearNumExpr();
//				dirCostConstraint[i].addTerm(1, dirDiff[i]);
//				dirCostConstraint[i].addTerm(-8, zeta[i][0]);
//				dirCostConstraint[i].addTerm(8, zeta[i][1]);
//				cplex.addGe(dirCostConstraint[i], cplex.prod(-1, dirCost[i]) );
//				cplex.addLe(dirCostConstraint[i], dirCost[i] );
//
//				
//			}
//			for (int i = mR; i < m; i++) {
//				dirCostConstraint[i] = cplex.linearNumExpr();
//				dirCostConstraint[i].addTerm(1, dirDiff[i]);
//				dirCostConstraint[i].addTerm(-8, zeta[i][0]);
//				dirCostConstraint[i].addTerm(8, zeta[i][1]);
//				cplex.addGe(dirCostConstraint[i], cplex.prod(-1, dirCostAdj[i-mR]) );
//				cplex.addLe(dirCostConstraint[i], dirCostAdj[i-mR] );
//
//				
//			}

			
			 
			
			
		
			/*********Distance Minization*****************/
			/**
			 * 
			 * If you are minimizing an increasing function of |x| (or maximizing a decreasing function, of course), you can always have the aboslute value of any quantity x in a lp as a variable absx such as:

				absx >= x
				absx >= -x
				{leftside = x
				dX = absx}
				It works because the value absx will 'tend' to its lower bound, so it will either reach x or -x.

				On the other hand, if you are minimizing a decreasing function of |x|, your problem is not convex and cannot be modelled as a lp.
				
				example: x = 1 -> absx >= 1 and absx >= -1  -> abx >= 1 
						 x = -1 -> absx >= -1 and absx >= 1 -> absx >= 1 and we want to minimize absx, so it will tend to 1!
			 */

			/*distance reduction*/
			IloNumVar[] dX  = cplex.numVarArray(nR, 0, Double.MAX_VALUE);
			IloNumVar[] dY  = cplex.numVarArray(nR, 0, Double.MAX_VALUE);
			//IloNumVar[] dMax  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			
			
		for (int i = 0; i < nR; i++) {
				double reScaledx = rescaledRelevantRoutePoints.get(i).getX();
				double reScaledy = rescaledRelevantRoutePoints.get(i).getY();
//				IloNumExpr xLeftSide = cplex.diff(streetNodeMap.get(routeNodeIdList.get(i)).getProjectGeom().getX(), x[i]);
//				IloNumExpr yLeftSide = cplex.diff(streetNodeMap.get(routeNodeIdList.get(i)).getProjectGeom().getY(), y[i]);
				
				IloNumExpr xLeftSide = cplex.diff(reScaledx, x[i]);
				IloNumExpr yLeftSide = cplex.diff(reScaledy, y[i]);
				
				cplex.addLe(xLeftSide, dX[i]);
				cplex.addGe(xLeftSide, cplex.prod(-1, dX[i]));
				
				
				cplex.addLe(yLeftSide, dY[i]);
				cplex.addGe(yLeftSide, cplex.prod(-1, dY[i]));
				
			}
			
			
			
			IloNumExpr sumXDist =  cplex.sum(dX);
			IloNumExpr sumYDist =  cplex.sum(dY);

			
			
			
			/*************Proportion diference minimization***********/

			/*** lambda: binary variable to determine if (xi - xi+1) is > 0 or < 0**
			 * lambda1 + lambda2 =1 -> if lambda1 = 1 - > lambda2 = 0 ->  (xi - xi+1) >= 0
			 * 							else lambda1 = 0 -> lambda2 = 1 -> (xi - xi+1) < 0
			 * 
			 * we want dx = |xi - xi+1| -> dx >= |xi - xi+1| & dx <= |xi - xi+1|
			 * if (xi - xi+1) >= 0 -> dx >= xi - xi+1 & dx <= xi - xi+1
			 * if (xi - xi+1) < 0  -> dx >= -(xi - xi+1) & dx <= -(xi - xi+1) 
			 * 
			 * so
			 * dx <= (xi - xi+1) + M(1-lambda1) eq1
			 * dx >= (xi - xi+1) - M(1-lambda1) eq2
			 * 
			 * or
			 * dx >= -(xi - xi+1) - M(1-lambda2) eq3
			 * dx <= -(xi - xi+1) + M(1-lambda2) eq4
			 * 
			 * if lambda1 = 1 & lambda2 = 0  => constraint dx = (xi - xi+1) & e3 and eq4 are trivialy fulfiled(makes no diference) 
			 * else lambda1 = 0 & lambda2 = 1  => constraint dx = -(xi - xi+1) & e1 and eq2 are trivialy fulfiled(makes no diference) 
			 * 
			 * 
			 * 
			 * */
			
			int mP = proportionKeepingRescaledRouteDPPoints.size() -1; /*route edges number: respect order on edge list*/ 
			
			IloNumVar[][] xLambdaDP = new IloNumVar[mP][];
			IloNumVar[][] yLambdaDP = new IloNumVar[mP][];
			
			for (int i = 0; i < mP; i++) {
				xLambdaDP[i] = cplex.boolVarArray(2);
				yLambdaDP[i] = cplex.boolVarArray(2);
				xLambdaDP[i][0].setName("proporMinxLambda0");
				xLambdaDP[i][1].setName("proporMinxLambda1");
				yLambdaDP[i][0].setName("proporMinyLambda0");
				yLambdaDP[i][1].setName("proporMinyLambda1");
				
			}
			IloLinearNumExpr[] xLambdaConstraintDP = new IloLinearNumExpr[mP];
			IloLinearNumExpr[] yLambdaConstraintDP = new IloLinearNumExpr[mP];
			for (int i = 0; i < mP; i++) {
				xLambdaConstraintDP[i] = cplex.linearNumExpr();
				xLambdaConstraintDP[i].addTerm(1.0,xLambdaDP[i][0]);
				xLambdaConstraintDP[i].addTerm(1.0,xLambdaDP[i][1]);

				cplex.addEq(xLambdaConstraintDP[i], 1.0);
				
				
				yLambdaConstraintDP[i] = cplex.linearNumExpr();
				yLambdaConstraintDP[i].addTerm(1.0,yLambdaDP[i][0]);
				yLambdaConstraintDP[i].addTerm(1.0,yLambdaDP[i][1]);

				cplex.addEq(yLambdaConstraintDP[i], 1.0);
				
			}
			
			 
			/*  maxAdjD distance between two adjacent vertices vertices! it was MaxD*/
			MAdjVertice = rescaledRouteLenght;
			IloNumVar[] absoluteDiffXDP  = cplex.numVarArray(mP, 0, 4* maxAdjD);
			IloNumVar[] absoluteDiffYDP  = cplex.numVarArray(mP, 0, 4* maxAdjD);
			
			for (int i = 0; i < mP; i++) {
				
				int v1Index = allNodeIdList.indexOf(proportionKeepingRescaledRouteDPPointsId.get(i));
				int v2Index = allNodeIdList.indexOf(proportionKeepingRescaledRouteDPPointsId.get(i + 1));	
				
				/***dx = |xi - xi+1|***/
				IloNumExpr diffX = cplex.diff(x[v1Index], x[v2Index]);
				
				cplex.addLe(absoluteDiffXDP[i], cplex.sum(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambdaDP[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffXDP[i], cplex.diff(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambdaDP[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffXDP[i], cplex.diff(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambdaDP[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffXDP[i], cplex.sum(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambdaDP[i][1])))); //eq4.
				
				/***dy = |xi - yi+1|***/
				IloNumExpr diffY = cplex.diff(y[v1Index], y[v2Index]);
				
				cplex.addLe(absoluteDiffYDP[i], cplex.sum(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambdaDP[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffYDP[i], cplex.diff(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambdaDP[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffYDP[i], cplex.diff(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambdaDP[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffYDP[i], cplex.sum(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambdaDP[i][1])))); //eq4.

				
				
			}
			/***WE have absDiffX and absDiffY
			 * We wanto to minimize:
			 * Abs((absDiffX[i] + absDiffY[i])L - lengthEdge[i]/L)
			 * 
			 * ****/
			/**
			 * If you are minimizing an increasing function of |x| (or maximizing a decreasing function, of course), you can always have the aboslute value of any quantity x in a lp as a variable absx such as:

				absx >= x -> x<=absx
				absx >= -x -> x >= -absx
				{leftside = x
				dX = absx}
				It works because the value absx will 'tend' to its lower bound, so it will either reach x or -x.

				On the other hand, if you are minimizing a decreasing function of |x|, your problem is not convex and cannot be modelled as a lp.
			 */
			ArrayList<Double> edgeProportion = new ArrayList<Double>();
			for (int i = 0; i < mP; i++) {
				Point2D pt1 =  proportionKeepingRescaledRouteDPPoints.get(i);
				Point2D pt2 =  proportionKeepingRescaledRouteDPPoints.get(i+1);
				double manhantamDist = (Math.abs(pt1.getX() - pt2.getX()) + Math.abs( pt1.getY() - pt2.getY() ) );
				//edgeProportion.add(manhantamDist/routeLenght);
				edgeProportion.add(proportionKeepingRescaledRouteDPPoints.get(i).distance(proportionKeepingRescaledRouteDPPoints.get(i+1))/rescaledRouteLenght);
			}
			IloNumVar[] absolutEdgeProportionDifferenceDP  = cplex.numVarArray(mP, 0, Double.MAX_VALUE);
			IloNumExpr[] finalEdgeProportionDP = new IloNumExpr[mP];
			for (int i = 0; i < mP; i++) {
				finalEdgeProportionDP[i] = cplex.prod(1/rescaledRouteLenght,cplex.sum(absoluteDiffXDP[i],absoluteDiffYDP[i]));
				/*can I use cplex.Equal direct? yes absX and absY is always positiv*/
				cplex.addLe(cplex.diff(finalEdgeProportionDP[i], edgeProportion.get(i)), absolutEdgeProportionDifferenceDP[i]);
				cplex.addGe(cplex.diff(finalEdgeProportionDP[i], edgeProportion.get(i)), cplex.prod(-1, absolutEdgeProportionDifferenceDP[i]));
			}
			
			IloNumExpr sumAbsolutEdgeProportionDifferenceDP =  cplex.sum(absolutEdgeProportionDifferenceDP);

//			

			//			
			/*************PROPORTION DIFFERENCE MINIMIZATION ALL EDEGES***********/

			/*** lambda: binary variable to determine if (xi - xi+1) is > 0 or < 0**
			 * lambda1 + lambda2 =1 -> if lambda1 = 1 - > lambda2 = 0 ->  (xi - xi+1) >= 0
			 * 							else lambda1 = 0 -> lambda2 = 1 -> (xi - xi+1) < 0
			 * 
			 * we want dx = |xi - xi+1| -> dx >= |xi - xi+1| & dx <= |xi - xi+1|
			 * if (xi - xi+1) >= 0 -> dx >= xi - xi+1 & dx <= xi - xi+1
			 * if (xi - xi+1) < 0  -> dx >= -(xi - xi+1) & dx <= -(xi - xi+1) 
			 * 
			 * so
			 * dx <= (xi - xi+1) + M(1-lambda1) eq1
			 * dx >= (xi - xi+1) - M(1-lambda1) eq2
			 * 
			 * or
			 * dx >= -(xi - xi+1) - M(1-lambda2) eq3
			 * dx <= -(xi - xi+1) + M(1-lambda2) eq4
			 * 
			 * if lambda1 = 1 & lambda2 = 0  => constraint dx = (xi - xi+1) & e3 and eq4 are trivialy fulfiled(makes no diference) 
			 * else lambda1 = 0 & lambda2 = 1  => constraint dx = -(xi - xi+1) & e1 and eq2 are trivialy fulfiled(makes no diference) 
			 * 
			 * 
			 * 
			 * */
			
			//int mP = proportionKeepingRescaledRouteDPPoints.size() -1; /*route edges number: respect order on edge list*/ 
			
			IloNumVar[][] xLambda = new IloNumVar[mR][];
			IloNumVar[][] yLambda = new IloNumVar[mR][];
			
			for (int i = 0; i < mR; i++) {
				xLambda[i] = cplex.boolVarArray(2);
				yLambda[i] = cplex.boolVarArray(2);
				xLambda[i][0].setName("proporMinxLambda0");
				xLambda[i][1].setName("proporMinxLambda1");
				yLambda[i][0].setName("proporMinyLambda0");
				yLambda[i][1].setName("proporMinyLambda1");
				
			}
			IloLinearNumExpr[] xLambdaConstraint = new IloLinearNumExpr[mR];
			IloLinearNumExpr[] yLambdaConstraint = new IloLinearNumExpr[mR];
			for (int i = 0; i < mR; i++) {
				xLambdaConstraint[i] = cplex.linearNumExpr();
				xLambdaConstraint[i].addTerm(1.0,xLambda[i][0]);
				xLambdaConstraint[i].addTerm(1.0,xLambda[i][1]);

				cplex.addEq(xLambdaConstraint[i], 1.0);
				
				
				yLambdaConstraint[i] = cplex.linearNumExpr();
				yLambdaConstraint[i].addTerm(1.0,yLambda[i][0]);
				yLambdaConstraint[i].addTerm(1.0,yLambda[i][1]);

				cplex.addEq(yLambdaConstraint[i], 1.0);
				
			}
			
			 
			/*  maxAdjD distance between two adjacent vertices vertices! it was MaxD*/
			MAdjVertice = rescaledRouteLenght;
			IloNumVar[] absoluteDiffX  = cplex.numVarArray(mR, 0, 4* maxAdjD);
			IloNumVar[] absoluteDiffY  = cplex.numVarArray(mR, 0, 4* maxAdjD);
			
			for (int i = 0; i < mR; i++) {
				
				int v1Index = allNodeIdList.indexOf(routeNodeIdList.get(i));
				int v2Index = allNodeIdList.indexOf(routeNodeIdList.get(i + 1));	
				
				/***dx = |xi - xi+1|***/
				IloNumExpr diffX = cplex.diff(x[v1Index], x[v2Index]);
				
				cplex.addLe(absoluteDiffX[i], cplex.sum(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffX[i], cplex.diff(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffX[i], cplex.diff(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffX[i], cplex.sum(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq4.
				
				/***dy = |xi - yi+1|***/
				IloNumExpr diffY = cplex.diff(y[v1Index], y[v2Index]);
				
				cplex.addLe(absoluteDiffY[i], cplex.sum(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffY[i], cplex.diff(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffY[i], cplex.diff(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffY[i], cplex.sum(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq4.

				
				
			}
			/***WE have absDiffX and absDiffY
			 * We wanto to minimize:
			 * Abs((absDiffX[i] + absDiffY[i])L - lengthEdge[i]/L)
			 * 
			 * ****/
			/**
			 * If you are minimizing an increasing function of |x| (or maximizing a decreasing function, of course), you can always have the aboslute value of any quantity x in a lp as a variable absx such as:

				absx >= x -> x<=absx
				absx >= -x -> x >= -absx
				{leftside = x
				dX = absx}
				It works because the value absx will 'tend' to its lower bound, so it will either reach x or -x.

				On the other hand, if you are minimizing a decreasing function of |x|, your problem is not convex and cannot be modelled as a lp.
			 */
			edgeProportion = new ArrayList<Double>();
			for (int i = 0; i < mR; i++) {

				
				//edgeProportion.add(pt1.distance(pt2)/routeLenght);
				edgeProportion.add(rescaledRelevantRoutePoints.get(i).distance(rescaledRelevantRoutePoints.get(i+1))/rescaledRouteLenght);
			}
			IloNumVar[] absolutEdgeProportionDifference  = cplex.numVarArray(mR, 0, Double.MAX_VALUE);
			IloNumExpr[] finalEdgeProportion = new IloNumExpr[mR];
			for (int i = 0; i < mR; i++) {
				finalEdgeProportion[i] = cplex.prod(1/rescaledRouteLenght,cplex.sum(absoluteDiffX[i],absoluteDiffY[i]));
				/*can I use cplex.Equal direct? yes absX and absY is always positiv*/
				cplex.addLe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), absolutEdgeProportionDifference[i]);
				cplex.addGe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), cplex.prod(-1, absolutEdgeProportionDifference[i]));
			}
			
			IloNumExpr sumAbsolutEdgeProportionDifference =  cplex.sum(absolutEdgeProportionDifference);

			
			
			
			
			/*********OBJECTIVE*****************/
			
			double bendWeight = 1.0*bendFactor; /*normalized under number of bends*/		
			//IloNumExpr sumBendCostNormal =  cplex.prod(1.0/b,sumBendCost);
			IloNumExpr sumBendCostFactor = cplex.prod(bendFactor/b,sumBendCost);
			
			double distWeight = 1.0*distFactor; /*normalized under number of nodes*/
			//IloNumExpr sumDistCostNormal = cplex.prod(1.0/nR, cplex.sum(sumXDist, sumYDist ));			
			IloNumExpr sumDistCostFactor = cplex.prod(distWeight/nR, cplex.sum(sumXDist, sumYDist ));
			
			
			double dirWeight = 1.5*dirFactor; /*normalized by number of edges*/
			IloNumExpr sumDirCostFactor = cplex.prod(dirWeight/mR,sumDirCost);
			
			
			double dirAdjWeight = 0.1*dirFactor; /*normalized by number of adj edges edges*/
			if( sumDirCostAdj != null) {				
				IloNumExpr sumDirAdjCostFactor =  cplex.prod(dirAdjWeight/(m-mR),sumDirCostAdj); 
				sumDirCostFactor = cplex.sum(sumDirCostFactor, sumDirAdjCostFactor);
			}
			
			
			

			
			double proportionDPWeight = 1.5*proportionDPFactor;
			
			double proportionAllWeight = 2.0*proportionAllPtsFactor;
			

			IloNumExpr sumEdgeProportionFactorDP = cplex.prod(proportionDPWeight/mP, sumAbsolutEdgeProportionDifferenceDP);
			IloNumExpr sumEdgeProportionAllPtsFactor = cplex.prod(proportionAllWeight/mR, sumAbsolutEdgeProportionDifference);
			
			

			
			IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumBendCostFactor, sumDirCostFactor, sumEdgeProportionFactorDP, sumEdgeProportionAllPtsFactor) ;
//			IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumBendCostFactor, sumDirCostFactor, sumEdgeProportionFactorDP, sumEdgeProportionFactor, sumAdjTunrCostFactor) ;

			//IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumBendCostFactor, sumDirCostFactor) ;

			//IloNumExpr objective = sumBendCostFactor ;
			//IloNumExpr objective = cplex.sum(sumXDist, sumYDist ) ;

			
			cplex.addMinimize(objective);
			
			if(executionTimeLimit > 0 ){
				double limitTimeInSeconds = ((double)(executionTimeLimit))/1000;
				cplex.setParam(IloCplex.DoubleParam.TiLim, limitTimeInSeconds);
			}
			else if(executionTimeLimit <0 ){
				//cplex.setParam(IloCplex.Param.MIP.Limits.Solutions, 1);
				cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, (-(double)executionTimeLimit)/100.0);
				resultReport.getRouteReport().setGap((-(double)executionTimeLimit)/100.0 );
				
				cplex.setParam(IloCplex.DoubleParam.TiLim, 120);
			}
			

//			if(executionTimeLimit > 0 ){
//				double limitTimeInSeconds = ((double)(executionTimeLimit))/1000;
//				cplex.setParam(IloCplex.DoubleParam.TiLim, limitTimeInSeconds);
//			}
//			else if(executionTimeLimit <0 ){
//				cplex.setParam(IloCplex.Param.MIP.Limits.Solutions, 1);
//			}

			/*control display information*/
			//cplex.setParam(IloCplex.Param.MIP.Display, 1);
			
			if(cplex.solve()){
				
				for (int i = mR; i < mR + 20; i++) {
					
					for(int j = mR; j < mR + 20 ; j++){
					}
				}
				
				System.out.println("obj =  " +cplex.getObjValue());
				for (int i = 0; i < nR; i++) {
					Point2D.Double pt = new Point2D.Double(cplex.getValue(x[i]),cplex.getValue(y[i])); 
					mipLineString.add(pt);
				}	
				ArrayList<Point2D> finalPoints = new ArrayList<Point2D>();
				ArrayList<Point2D> routePathNormalized =  route.getRoutePath().asJava2DList(1);
				finalPoints.add(mipLineString.get(0));
				ArrayList<Integer> relevantNodeIndex = route.getRoutePath().getRelevantPointIndex();
				for(int i = 1; i < relevantNodeIndex.size() ; i++){
					int index = relevantNodeIndex.get(i);
					int lastIndexInFinalPoints = finalPoints.size() - 1;
					/*adicional o proximo ponto relevante entre ele e o anterior na linha reta com os ponts complementares*/
					finalPoints.add( mipLineString.get(i) );
					finalPoints = GeometricOperation.fillLine( finalPoints , routePathNormalized,  index - lastIndexInFinalPoints   , lastIndexInFinalPoints );
				}
				route.getRoutePath().updatePathXNodes( finalPoints);
				//System.out.println("Only ADj");
			
				/**IMPORTANTA - Uptade X coordinates of Asjacent edges to Route***/
				for (int i = nR; i < n ; i++){
					
					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
					streetNodeMap.get(allNodeIdList.get(i)).setxGeom(GeoConvertionsOperations.CreateJTSPoint(cplex.getValue(x[i]),cplex.getValue(y[i])));

				}
				
//				for(int i = 0; i < n ; i++){
//					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
	//
//				}
				
//				for (int i = 0; i < mP; i++){
//					System.out.println("Edge: " + i);
//					System.out.println("AbsDiffX-> =  " + cplex.getValue(absoluteDiffX[i]) + " AbsDiffY-> =  " + cplex.getValue(absoluteDiffY[i]) );
//					System.out.println("True edge proportion: " + edgeProportion.get(i) + "Final edgeProportion: " + cplex.getValue(finalEdgeProportion[i]) + "Absolut Difference: " + cplex.getValue(absolutEdgeProportionDifference[i]));
//				}
//				for (int i = 0; i < b ; i++){
//					System.out.println("benDir-> =  " + cplex.getValue(bendDiff[i]) + " bencost-> =  " + cplex.getValue(bendCost[i]) );
//					//System.out.println("gama0-> =  " + cplex.getValue(gama[i-2][0]) + " gama1-> =  " + cplex.getValue(gama[i-2][1])  + " gama2-> =  " + cplex.getValue(gama[i-2][2]));
//					
//					//System.out.println("origTurnDir->= "+ origTurnDir[i -1] + " benDir-> =  " + cplex.getValue(bendDir[i-1]) + " bencost-> =  " + cplex.getValue(bendCost[i-1]) );
//				}
////				
//				for (int i = 0; i < nR ; i++){
//					System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//				}
//				
//				for (int i = 0; i < mR ; i++){
//					//System.out.println("benDir-> =  " + cplex.getValue(bendDir[i-2]) + " bencost-> =  " + cplex.getValue(bendCost[i-2]) );
//					System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dirDiff-> =  " + cplex.getValue(dir[i][0]) + " dirDiff-> =  " + cplex.getValue(dirDiff[i]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]) );
//					
//				}
//				for (int i = 0; i < nA ; i++){
//					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//				}

//				for (int i = nR -1; i < m ; i++){
//					
//					System.out.println("x"+ i+1 + " =  " + cplex.getValue(x[i+1]) + " y"+ i+1 + " =  " + cplex.getValue(y[i+1]) + " z1"+ i+1 + " =  " + cplex.getValue(z1[i+1]) + " z2"+ i+1 + " =  " + cplex.getValue(z2[i+1]));
//				//	System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
	//
//					System.out.println("Route Point: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom() );
//					System.out.println("Adj Point: " +streetNodeMap.get(edgeList.get(i)[1]).getxGeom() );
//					System.out.println("Dist: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom().distance(streetNodeMap.get(edgeList.get(i)[1]).getxGeom()) );
//					System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dir-> =  " + cplex.getValue(dir[i][0]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]));
//					System.out.println();
//				}
				
		
				
				
//				numInterPoints = intersectionNodeIdList.size();
				

				
//				for (int i = 0; i < numInterPoints; i++) {
//					
	//
//					int vId = intersectionNodeIdList.get(i);
////					System.out.println();
////					System.out.println("Route relevant point n  " + routeNodeIdList.indexOf(vId) + " of id "+vId+"  is intersection and its degree is " + circularOrderList.get(vId).size() );
////					System.out.println("Interpoint"+i+" Intersection Id: " + vId);
//					
//					for(int j = 0; j < beta[i].length ; j ++ ){
//						int indexDirVU1 = 0, indexDirVU2 = 0;
////						System.out.println("--Edge " + j + " and " + (j+1));
//						int u1Id, u2Id;
//						u1Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j).getId();
////						System.out.println("u1ID="+u1Id); 
//						/*se for vertive una orden circular seleciona u2 como o primeiro da lista*/
//						if(j < (beta[i].length -1))
//							u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j + 1).getId();
//						else 
//							u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(0).getId();
////						System.out.println("u2ID="+u2Id);
//						/***identify the edges (v, u1) and (v, u2)**/
//						boolean foundEdgeToU1 = false, foundEdgeToU2 = false;
//						int edgeToU1IsInverted = 0, edgeToU2IsInverted = 0;
//						
//						for(int k = 0; k < edgeList.size(); k++){
	//
//							/*intersectionNOdeid contain only route nodes, the adj node are always edge[1]. Otherwise is necessary to check the inverse*/
//							if(edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u1Id) {
//								indexDirVU1 = k;
//								foundEdgeToU1 = true;
//								edgeToU1IsInverted = 0;
//							}
//							else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u1Id){
//								indexDirVU1 = k;
//								foundEdgeToU1 = true;
//								edgeToU1IsInverted = 1;
//							}
//							if( edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u2Id ){
//								indexDirVU2 = k;
//								foundEdgeToU2 = true;
//								edgeToU2IsInverted = 0;
//							}
//							else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u2Id){
//								indexDirVU2 = k;
//								foundEdgeToU2 = true;
//								edgeToU2IsInverted = 1;
//							}
//							if(foundEdgeToU1 && foundEdgeToU2)
//								break;
//						}
//						
////						System.out.println("EdgeIndex to u1ID= "+ indexDirVU1 + " dir: " + cplex.getValue(dir[indexDirVU1][edgeToU1IsInverted]) + " found: " + foundEdgeToU1  + " is inverted: " + edgeToU1IsInverted); 
////						System.out.println("EdgeIndex to u2ID= "+ indexDirVU2 + " dir: " + cplex.getValue(dir[indexDirVU2][edgeToU2IsInverted]) + " found: " + foundEdgeToU2 + " is inverted: " + edgeToU2IsInverted); 
//				
	//
//				
//					}
////					System.out.println();
//				}
				
				
				
				
				
				
				//System.out.println("sumDx =  " + cplex.getValue(sumXDist) + " sumDy =  " + cplex.getValue(sumYDist));
				System.out.println("Direction model is: "  + directionModel);
				System.out.println("Rescaled route lenght: " + rescaledRouteLenght);
				//System.out.println("distCost/Normal =  " + (cplex.getValue(sumXDist) + cplex.getValue(sumYDist)) + " / " +cplex.getValue(sumDistCostNormal) + " sumBendCost/Normal = " + cplex.getValue(sumBendCost)+ " / " +cplex.getValue(sumBendCostNormal) + " sumDirCost/Normal = " + cplex.getValue(sumDirCost) + " / " + cplex.getValue(sumDirCostNormal) + " sumEdgeProportionDPCost/Normal = " + cplex.getValue(sumAbsolutEdgeProportionDifferenceDP)+ " / " + cplex.getValue(sumAbsolutEdgeProportionDifferenceNormalDP) + " sumEdgeProportionDPCost/Normal = " + cplex.getValue(sumAbsolutEdgeProportionDifference)+ " / " + cplex.getValue(sumAbsolutEdgeProportionDifferenceNormal));
				System.out.println("distFactor =  " + distFactor + " bendFactor = " + bendFactor + " dirFactor = " + dirFactor + " proporDPFactor = " + proportionDPFactor + " proporAllPtsFactor = " + proportionDPFactor);
				System.out.println("distCostNormalFactor =  " + cplex.getValue(sumDistCostFactor) + " sumBendCostFactor = " + cplex.getValue(sumBendCostFactor) + " sumDirCostFactor = " + cplex.getValue(sumDirCostFactor) + " sumProportionDPFactor = " + cplex.getValue(sumEdgeProportionFactorDP) + " sumProportionAllPtsFactor = " + cplex.getValue(sumEdgeProportionAllPtsFactor));

				System.out.println("Final = " + cplex.getObjValue());
				
				cplex.end();
				end = System.currentTimeMillis();
				System.out.println("Route Scehmatization- Nodes: " +n+ " Execution Time: "   + (end - start) );
				resultReport.getRouteReport().setExecutionTime(end - start);
				
				return mipLineString;
				
			}
			else{
				
				throw new Exception("Cannot solve MIP Model on time");
				
			}
			
				
			
			
			
			
		}

		public static ArrayList<Point2D> streetPathOptimizer3DirTopoRelevant( Path path, ArrayList<Path> pathList, ArrayList<Point2D> transPointList, Map<Integer, StreetNode> nodeMap, StreetNetworkTopological streetNetworkTopological, double bendFactor,  double distFactor, double proportionFactor, double dirFactor, Boolean checkSelfTopology, Boolean checkTopology,double minNonAdjEdgeDist, int executionTimeLimit ) throws IloException, Exception   {
			//System.out.println("OPTIMIZER Network PATH - > ACTION!!!!!");	

			long start, end;
			start = System.currentTimeMillis(); 
			
			IloCplex cplex = new IloCplex();
			
			
			ArrayList<Point2D> mipLineString =  new ArrayList<Point2D>();
			ArrayList<Point2D> pathRelevantPointsList = new ArrayList<Point2D>();
			ArrayList<Integer> pathRelevantPointsIndex = new ArrayList<Integer>();

			for(int i = 0; i < transPointList.size(); i++) {
				if(path.getNodeList().get(i).isRelevantRouteNode()) {
					pathRelevantPointsList.add(transPointList.get(i));
					pathRelevantPointsIndex.add(i);
				}
			}
//			boolean isClose = true;
//			if(isClose)
//				transPointList.add(transPointList.get(0));
			
			//System.out.println(lineString);
			int n = pathRelevantPointsList.size();
			int m = n -1; /*number of edges in a paths*/
			int b = m - 1; /*number of bends in a path*/
			
			
			double maxExtend = GeometricOperation.diagonalOf(transPointList);
			double pahtLength = GeometricOperation.length(transPointList);
			double L = pahtLength/(n*100); /* Minimal distance between two vertices*/
			System.out.println("Path Relevant Points: " + n);
			System.out.println("Minimal distance between two vertices: " + L);
			System.out.println("Mininal distance from route " + minNonAdjEdgeDist);
			System.out.println("Linestring Legth " + pahtLength);
			System.out.println("Max Extend " + maxExtend);
			double extendLimit = 0.05;
			
			
			
			ArrayList<Integer[]> topoCheckEdgeList = new ArrayList<Integer[]>();
			ArrayList<Point2D> pointsToBoundingBox = new ArrayList<Point2D>(transPointList);
			Polygon boundingPolygon = (Polygon)GeoConvertionsOperations.Java2DToJTSGeometry( new OctlinearBoundingBox(transPointList, 2*minNonAdjEdgeDist, extendLimit +0.1, 0).getBoundingPolygon(), Geometries.POLYGON);


			
			if(checkTopology)
				for(Path p: pathList) {
	//				if(p.isRoute() ||
	//						p.isRouteAdjEdge())
					if(p.isRoute())		
					if(p.isWasSchematized()) {
						PointsPolar polarPoints = new PointsPolar();
						polarPoints  = GeometricOperation.toPolar(p.asJava2DList(2));
						for(int i = 0; i < p.getNodeList().size() - 1; i++) {
							if(!path.getNodeList().contains(p.getNodeList().get(i)) && !path.getNodeList().contains(p.getNodeList().get(i +1))) {
								
								if (boundingPolygon.contains( p.getNodeList().get(i).getxGeom()  ) || boundingPolygon.contains( p.getNodeList().get(i +1).getxGeom() )){
									int indexU = i;
									int indexV = i+1;
									boolean foundBend = false; 
									while(!foundBend ) {
									
										if(indexV < p.getNodeList().size() -1 && !path.getNodeList().contains(p.getNodeList().get(indexV + 1))
												&& Math.abs(polarPoints.getPoints().get(indexV -1).getTheta() - polarPoints.getPoints().get(indexV).getTheta()) < 0.001
												&& boundingPolygon.contains( p.getNodeList().get(indexV + 1).getxGeom() ) )
											indexV++;
										else {
											Integer[] edge = new Integer[2];
											edge[0] = p.getNodeList().get(indexU).getId();
											edge[1] = p.getNodeList().get(indexV).getId();
											pointsToBoundingBox.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(indexU).getxGeom()));
											pointsToBoundingBox.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(indexV).getxGeom()));
											topoCheckEdgeList.add(edge);
											foundBend = true;
											i = indexV -1;
										}
									}
										
									
									
									
								}
							}
													
						}
					}
				}
			
			OctlinearBoundingBox octBox = new OctlinearBoundingBox(pointsToBoundingBox, 2*minNonAdjEdgeDist, extendLimit +0.1, 2);
			double maxAdjDist = octBox.getMaxAdjDist();
			
			
	//
//	        System.out.println("maxD: " + maxD);
//	        System.out.println("N: " + n);

			/*********COORDINATES CONSTRAINT*****************/
	       

	        
			IloNumVar[] x  = cplex.numVarArray(n, octBox.getMinX(), octBox.getMaxX());
			IloNumVar[] y  = cplex.numVarArray(n,  octBox.getMinY(), octBox.getMaxY());
			IloNumVar[] z1  = cplex.numVarArray(n, octBox.getMinZ1(), octBox.getMaxZ1());
			IloNumVar[] z2  = cplex.numVarArray(n, octBox.getMinZ2(), octBox.getMaxZ2());
			
			
//			IloNumVar[] x  = cplex.numVarArray(n, -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] y  = cplex.numVarArray(n,  -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] z1  = cplex.numVarArray(n,  -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] z2  = cplex.numVarArray(n, -Double.MAX_VALUE, Double.MAX_VALUE);
			
			IloNumExpr[] z1Constraint = new IloNumExpr[n];
			IloNumExpr[] z2Constraint = new IloNumExpr[n];
			
			for (int i = 0; i < n; i++) {
				/*do we need to multiply by 2?*/
//				z1Constraint[i] = cplex.sum(x[i],y[i]);
//				z2Constraint[i] = cplex.diff(x[i],y[i]);
				
//				/*do we need to multiply by 2?*/
				z1Constraint[i] = cplex.prod(0.5,cplex.sum(x[i],y[i]));
				z2Constraint[i] = cplex.prod(0.5, cplex.diff(x[i],y[i]));
			}
			
			/*Add z constraint to model constranint*/ 
			for (int i = 0; i < n; i++) {
				cplex.addEq(z1[i], z1Constraint[i]);
				cplex.addEq(z2[i], z2Constraint[i]);
							
				
			}
//			/*fix position of first vertex*/
//			cplex.addEq(x[0], lineString.get(0).getX());
//			cplex.addEq(y[0], lineString.get(0).getY());
//			
//			/***fix position of already schematized vertices*/
//			/*I cannot fix position of 2 adjacent edges*/
			
//			for(int i = 0; i < n -1; i++) {
//				
//				
//				if(path.getNodeList().get(i).getxGeom() != null && !path.getNodeList().get(i).isAnchor()) {
//					
//					cplex.addEq(x[i], path.getNodeList().get(i).getxGeom().getX());
//					cplex.addEq(y[i], path.getNodeList().get(i).getxGeom().getY());
//				}
//				
//			}
//////			
////			
			int startEdgeDirection = -1;
			int endEdgeDirection = -1;
			
			if(path.getNodeList().get(0).getId() == path.getNodeList().get(path.getNodeList().size() -1).getId()) {				
				cplex.addEq(x[n-1], x[0]);
				cplex.addEq(y[n-1], y[0]);
			}
			else {
				if(path.getNodeList().get(0).getxGeom() != null ) {				
					cplex.addEq(x[0], path.getNodeList().get(0).getxGeom().getX());
					cplex.addEq(y[0], path.getNodeList().get(0).getxGeom().getY());
					if(path.getNodeList().get(1).getxGeom() != null ) {
					startEdgeDirection = GeometricOperation.sectorOf(
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
									path.getNodeList().get(0).getxGeom().getX(), path.getNodeList().get(0).getxGeom().getY(),
									path.getNodeList().get(1).getxGeom().getX(), path.getNodeList().get(1).getxGeom().getY() 					
									));
					}
				}
				else {
					cplex.addEq(x[0], transPointList.get(0).getX());
					cplex.addEq(y[0], transPointList.get(0).getY());
				}
				if(path.getNodeList().get(path.getNodeList().size() -1).getxGeom() != null ) {				
					cplex.addEq(x[n -1], path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getX());
					cplex.addEq(y[n -1], path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getY());
					if(path.getNodeList().get(path.getNodeList().size() -2).getxGeom() != null ) {
					endEdgeDirection = GeometricOperation.sectorOf(
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
									path.getNodeList().get(path.getNodeList().size() -2).getxGeom().getX(), path.getNodeList().get(path.getNodeList().size() -2).getxGeom().getY(),
									path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getX(), path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getY() 					
									));
					}
				}
				else {
					cplex.addEq(x[n -1], transPointList.get(transPointList.size() -1).getX());
					cplex.addEq(y[n -1], transPointList.get(transPointList.size() -1).getY());

				}
			}

			
//			
			
			/*********OCTALINEARITY CONSTRAINT*****************/
			
			/*sec[m][d][(pred,orig,succ)] defines the sector of the octilinear position an edges could lie
			 * m = number of edges
			 * d = direction of the edge 0 is going and 1 is back
			 * (pred,orig,succ) = 0 lies on the previous sector, 1 lies ont the original best sector, and 2 lies on the succecive sector*/ 
			int[][][] sec = new int[m][2][3];		
			/*int[] origBestSec = new int[m];
			int[] succBestSec = new int[m];
			int[] predBestSec = new int[m];*/
			
			for (int i = 0; i < m; i++) {

				if(i == 0 && startEdgeDirection >  -1)
					sec[i][0][1] = startEdgeDirection;

				else if (i == m-1 && endEdgeDirection >  -1)
					sec[i][0][1] = endEdgeDirection;
				else				
					sec[i][0][1] = GeometricOperation.sectorOf(
						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
								pathRelevantPointsList.get(i).getX(), pathRelevantPointsList.get(i).getY(),
								pathRelevantPointsList.get(i + 1).getX(), pathRelevantPointsList.get(i + 1).getY()					
								)
						);
				sec[i][0][0] =  Math.floorMod(sec[i][0][1] - 1, 8);
				sec[i][0][2] =  Math.floorMod(sec[i][0][1] + 1, 8);
				
				
				sec[i][1][1] = Math.floorMod(sec[i][0][1] + 4,8);
				sec[i][1][0] = Math.floorMod(sec[i][0][0] + 4,8);
				sec[i][1][2] = Math.floorMod(sec[i][0][2] + 4,8);
			}
		

			
			/*Boolean variable to restrict the orientation of the edge to orig = 1, succ = 2 and prd = 0*/
			IloNumVar[][] alpha = new IloNumVar[m][];
			
			for (int i = 0; i < m; i++) {
				alpha[i] = cplex.boolVarArray(3);
				
			}
			/**force alfa to best direction if start and end edges directions are set **/
			if(startEdgeDirection >  -1){

				cplex.addEq(alpha[0][0], 0);
				cplex.addEq(alpha[0][1], 1);
				cplex.addEq(alpha[0][2], 0);



			}
			if(endEdgeDirection >  -1){

				cplex.addEq(alpha[m-1][0], 0);
				cplex.addEq(alpha[m-1][1], 1);
				cplex.addEq(alpha[m-1][2], 0);



			}
			
			/*IloNumVar[] isOrig = cplex.boolVarArray(m);
			IloNumVar[] isSucc = cplex.boolVarArray(m);
			IloNumVar[] isPred = cplex.boolVarArray(m);*/
			
			IloLinearNumExpr[] constraintEdgeOrientation = new IloLinearNumExpr[m];
			
			for (int i = 0; i < m; i++) {
				constraintEdgeOrientation[i] = cplex.linearNumExpr();
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][0]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][1]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][2]);
				cplex.addEq(constraintEdgeOrientation[i], 1.0);
			}
			
			
			/*Variable dir[i][j] defines the direction of of edege i (j means de drirection of the edege*/
			IloNumVar[][] dir  = new IloNumVar[m][]; /*leave the last dimension to be defined with cplex*/
			
			for (int i = 0; i < m; i++) {
				dir[i] = cplex.intVarArray(2, 0, 7);/* array of size 2 because we need uv an vu*/
				
			}
			
			
			/*For each i ∈ {pred, orig, succ} we have the following set of constraints
			 dir(u, v)−seciu(v) ≤M(1−αi(u, v))
			−dir(u, v)+seciu(v) ≤M(1−αi(u, v))
			dir(v,u)−seciv(u) ≤M(1−αi(u, v))
			−dir(v,u)+seciv(u) ≤M(1−αi(u, v))
			
			∀{u, v} ∈ E,
			
			 *Here, if αi(u, v) = 0, the constraints in (4) are trivially fulfilled and do not influence the left-hand sides. 
			 *On the other hand, if αi(u, v) = 1, the four inequalities are equivalent to dir(u, v) = seciu(v) and
				dir(v,u) = seciv(u) as desired (equality
			 *
			 *
			 *
			 */
			
			double MDir = 8; 
			for (int i = 0; i < m; i++) {
				if(i == 0 && startEdgeDirection >  -1){
					cplex.addEq(dir[i][0],startEdgeDirection);
					cplex.addEq(dir[i][1],Math.floorMod(startEdgeDirection + 4,8));
				}
				else if (i == m-1 && endEdgeDirection >  -1){
					cplex.addEq(dir[i][0],endEdgeDirection);
					cplex.addEq(dir[i][1],Math.floorMod(endEdgeDirection + 4,8));
				}
				else{
					for (int j = 0; j < 3; j++) {
						IloNumExpr rightSide = cplex.prod(MDir, cplex.diff(1, alpha[i][j]));
						/*natural direction of the edge d=0*/
						IloNumExpr leftSide1 = cplex.diff(dir[i][0], sec[i][0][j]);
						IloNumExpr leftSide2 = cplex.diff(sec[i][0][j],  dir[i][0]);
						/*counter direction of the edge d= 1*/
						IloNumExpr leftSide3 = cplex.diff(dir[i][1], sec[i][1][j]);
						IloNumExpr leftSide4 = cplex.diff(sec[i][1][j],  dir[i][1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);
					
					}
				}
				
			
			}
			
			/*contraints to force the correct position of the vertices
			 * if sec original is 2 and alphaoriginal is true then forces x(u) and x(v) to equal
			 * and y(v) > y(u)*/
			double MAdjVertice = 2*maxAdjDist; /* D max distance between two vertices*/
			for (int i = 0; i < m; i++) {
				
				for (int j = 0; j < 3; j++) {
					IloNumExpr rightSide, rightSide2, leftSide1, leftSide2, leftSide3;
					/*do i need to constraint edges for both directions(sec[m][1])?*/
					switch (sec[i][0][j]) {
					case 0:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(y[i], y[i+1]);
						leftSide2 = cplex.diff(y[i+1],  y[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(x[i+1],  x[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
					
						break;
					case 7:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[i], z2[i+1]);
						leftSide2 = cplex.diff(z2[i+1],  z2[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z1[i+1],  z1[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;
						
					case 6:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[i], x[i+1]);
						leftSide2 = cplex.diff(x[i+1],  x[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(y[i+1],  y[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);

						break;
						
					case 5:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[i], z1[i+1]);
						leftSide2 = cplex.diff(z1[i+1],  z1[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z2[i],  z2[i + 1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					case 4:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(y[i], y[i+1]);
						leftSide2 = cplex.diff(y[i+1],  y[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(x[i],  x[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;
					case 3:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[i], z2[i+1]);
						leftSide2 = cplex.diff(z2[i+1],  z2[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z1[i],  z1[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					case 2:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[i], x[i+1]);
						leftSide2 = cplex.diff(x[i+1],  x[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(y[i],  y[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);

						break;	
					case 1:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[i], z1[i+1]);
						leftSide2 = cplex.diff(z1[i+1],  z1[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z2[i+1],  z2[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					default:
						break;
					}
					
				}
				
			}
			
			
//			/*******CHECK SELF PLANARITY*******///
			System.out.println("checkin topology: " + checkSelfTopology);
			if(checkSelfTopology){
				IloNumVar[][][] gama = new IloNumVar[m][m][];
				/*RElation of m edges from the path and r route edges check relative position to 8 orientations ∈{N,S,E,W,NE,NW,SE,SW}*/
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < m; j++)
						gama[i][j] = cplex.boolVarArray(8);
				
			
			
				IloLinearNumExpr[][] constraintEdgeTopology = new IloLinearNumExpr[m][m];
				/*SumOF{ γi(e1, e2) } ≥ 1   
				 * i∈{N,S,E,W,NE,NW,SE,SW} */
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < m; j++){
						//				for (int k = 0; k < 8; k++){
						//					if(k!=0)
						//						cplex.addEq(gama[i][j][k], 0);
						//					else
						//						cplex.addEq(gama[i][j][k], 1);
						//				}


						constraintEdgeTopology[i][j] = cplex.linearNumExpr();
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][0]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][1]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][2]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][3]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][4]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][5]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][6]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][7]);
						cplex.addGe(constraintEdgeTopology[i][j], 1.0);

					}


				for (int i = 0; i < m; i++) {
					for(int j = 0; j < m ; j++){

						int edge1UIndex = i ;
						int edge1VIndex = i + 1 ;

						int edge2UIndex = j ;
						int edge2VIndex = j + 1 ;

						/*condition to guarantee that path edege i and j are not adjacents*/
						if( Math.abs(i - j) > 1  && Math.abs(i - j) < m -1 )
							for(int k =0; k < 8; k ++){
								/*whats the value of M(max X ou y dist of points) and dist Min*/ 
								double MTopology = maxExtend*3; /*da pra diminuir*/
								IloNumExpr rightSide,  leftSide1, leftSide2, leftSide3, leftSide4;


								switch (k) {
								case 0: /* E (e2(route edge) ist east of e1 (path edge)) : 0 : x*/

									/*
									 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
									 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
									 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									 * ∀(e1,e2) no incident
									 */
									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(x[edge1UIndex], x[edge2UIndex]);
									leftSide2 = cplex.diff(x[edge1UIndex], x[edge2VIndex]);
									leftSide3 = cplex.diff(x[edge1VIndex], x[edge2UIndex] );
									leftSide4 = cplex.diff(x[edge1VIndex], x[edge2VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 1: /* e2 (route edge) is NE (SE because y is inverted) of e1: 45 (315) : z1 */

									/*Can I use 
									 * route.get(j).getX() + route.get(j).getY() == (route.get(j).getX() / Math.sin(Math.toRadians(45)));
									 */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(z1[edge1UIndex], z1[edge2UIndex]);
									leftSide2 = cplex.diff(z1[edge1UIndex], z1[edge2VIndex]);
									leftSide3 = cplex.diff(z1[edge1VIndex], z1[edge2UIndex]);
									leftSide4 = cplex.diff(z1[edge1VIndex], z1[edge2VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);


									break;
								case 2: /* e2 (route edge) is N (S because y is inverted) of e1: 90 (270) : y */
									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff(y[edge1UIndex], y[edge2UIndex]);
									leftSide2 = cplex.diff(y[edge1UIndex], y[edge2VIndex]);
									leftSide3 = cplex.diff(y[edge1VIndex], y[edge2UIndex]);
									leftSide4 = cplex.diff(y[edge1VIndex], y[edge2VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;	
								case 3: /* e2 (route edge) is NW (SW because y is inverted) of e1: 135 (225) : -z2 */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
									leftSide2 = cplex.diff( z2[edge2VIndex],  z2[edge1UIndex] );
									leftSide3 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
									leftSide4 = cplex.diff( z2[edge2VIndex],  z2[edge1VIndex] );

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 4: /*  e2 (route edge) is W  of e1:  180:  -x */

									//						/*
									//						 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
									//						 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									//						 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
									//						 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									//						 * ∀(e1,e2) no incident
									//						 */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff(x[edge2UIndex] , x[edge1UIndex] );
									leftSide2 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );
									leftSide3 = cplex.diff(x[edge2UIndex] , x[edge1VIndex] );
									leftSide4 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 5: /*  e2 (route edge) is SW (NW because y is inverted) of e1: 225 (135): -z1 */


									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff( z1[edge2UIndex] , z1[edge1UIndex]);
									leftSide2 = cplex.diff( z1[edge2VIndex] , z1[edge1UIndex]);
									leftSide3 = cplex.diff( z1[edge2UIndex] , z1[edge1VIndex]);
									leftSide4 = cplex.diff( z1[edge2VIndex] , z1[edge1VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 6:/*  e2 (route edge) is S (N because y is inverted) of e1: 270 (90)  : -y */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(  y[edge2UIndex] ,y[edge1UIndex] );
									leftSide2 = cplex.diff(  y[edge2VIndex], y[edge1UIndex]);
									leftSide3 = cplex.diff(  y[edge2UIndex] ,y[edge1VIndex] );
									leftSide4 = cplex.diff(  y[edge2VIndex], y[edge1VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 7:/*  e2 (route edge) is SE (NE because y is inverted) of e1: 315 (45)  : z2*/

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(z2[edge1UIndex], z2[edge2UIndex] );
									leftSide2 = cplex.diff(z2[edge1UIndex], z2[edge2VIndex] );
									leftSide3 = cplex.diff(z2[edge1VIndex], z2[edge2UIndex] );
									leftSide4 = cplex.diff(z2[edge1VIndex], z2[edge2VIndex] );

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);


									break;
								default:
									break;
								}

							}

					}

				}

			}
			
			/*********EDGE DISTANCE - EXTRA CROSSING CONSTRAINT - TOPOLOGY*****************/
			
			int e = topoCheckEdgeList.size();
			/*Boolean variable to indicate if path edge is {N,S,E,W,NE,NW,SE,SW} of route edge*/
			IloNumVar[][][] gama2 = new IloNumVar[m][e][];
			
			/*RElation of m edges from the path and r route edges check relative position to 8 orientations ∈{N,S,E,W,NE,NW,SE,SW}*/
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < e; j++)
						gama2[i][j] = cplex.boolVarArray(8);
					
				IloLinearNumExpr[][] constraintEdgeTopology2 = new IloLinearNumExpr[m][e];
				
				
				/*SumOF{ γi(e1, e2) } ≥ 1   
				 * i∈{N,S,E,W,NE,NW,SE,SW} */
				
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < e; j++){
		//				for (int k = 0; k < 8; k++){
		//					if(k!=0)
		//						cplex.addEq(gama[i][j][k], 0);
		//					else
		//						cplex.addEq(gama[i][j][k], 1);
		//				}
						
						
						constraintEdgeTopology2[i][j] = cplex.linearNumExpr();
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][0]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][1]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][2]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][3]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][4]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][5]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][6]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][7]);
						cplex.addGe(constraintEdgeTopology2[i][j], 1.0);
			
					}
						
				if(checkTopology)
						for (int i = 0; i < m; i++) {
						for(int j = 0; j < e ; j++){
	
							int edge1UIndex = i ;
							int edge1VIndex = i + 1 ;
	
							StreetNode edge2U = nodeMap.get(topoCheckEdgeList.get(j)[0]);
							StreetNode edge2V = nodeMap.get(topoCheckEdgeList.get(j)[1]);
							
							double xedge2U = edge2U.getxGeom().getX();
							double yedge2U = edge2U.getxGeom().getY();
							double z1edge2U = 0.5*(xedge2U + yedge2U);
							double z2edge2U = 0.5*(xedge2U - yedge2U);
							
							double xedge2V = edge2V.getxGeom().getX();
							double yedge2V = edge2V.getxGeom().getY();
							double z1edge2V = 0.5*(xedge2V + yedge2V);
							double z2edge2V = 0.5*(xedge2V - yedge2V);
							
							
	
						
								for(int k =0; k < 8; k ++){
									/*whats the value of M(max X ou y dist of points) and dist Min*/ 
									double MTopology = maxExtend*3; /*da pra diminuir*/
									IloNumExpr rightSide,  leftSide1, leftSide2, leftSide3, leftSide4;
	
	
									switch (k) {
									case 0: /* E (e2(route edge) ist east of e1 (path edge)) : 0 : x*/
	
										/*
										 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
										 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
										 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										 * ∀(e1,e2) no incident
										 */
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(x[edge1UIndex],xedge2U);
										leftSide2 = cplex.diff(x[edge1UIndex], xedge2V);
										leftSide3 = cplex.diff(x[edge1VIndex],xedge2U );
										leftSide4 = cplex.diff(x[edge1VIndex], xedge2V);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 1: /* e2 (route edge) is NE (SE because y is inverted) of e1: 45 (315) : z1 */
	
										/*Can I use 
										 * route.get(j).getX() + route.get(j).getY() == (route.get(j).getX() / Math.sin(Math.toRadians(45)));
										 */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(z1[edge1UIndex], z1edge2U);
										leftSide2 = cplex.diff(z1[edge1UIndex], z1edge2V);
										leftSide3 = cplex.diff(z1[edge1VIndex], z1edge2U);
										leftSide4 = cplex.diff(z1[edge1VIndex], z1edge2V);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
	
										break;
									case 2: /* e2 (route edge) is N (S because y is inverted) of e1: 90 (270) : y */
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff(y[edge1UIndex], yedge2U);
										leftSide2 = cplex.diff(y[edge1UIndex], yedge2V);
										leftSide3 = cplex.diff(y[edge1VIndex], yedge2U);
										leftSide4 = cplex.diff(y[edge1VIndex], yedge2V);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;	
									case 3: /* e2 (route edge) is NW (SW because y is inverted) of e1: 135 (225) : -z2 */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff( z2edge2U , z2[edge1UIndex] );
										leftSide2 = cplex.diff( z2edge2V,  z2[edge1UIndex] );
										leftSide3 = cplex.diff( z2edge2U , z2[edge1VIndex] );
										leftSide4 = cplex.diff( z2edge2V,  z2[edge1VIndex] );
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 4: /*  e2 (route edge) is W  of e1:  180:  -x */
	
										//						/*
										//						 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
										//						 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										//						 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
										//						 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										//						 * ∀(e1,e2) no incident
										//						 */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff(xedge2U , x[edge1UIndex] );
										leftSide2 = cplex.diff(xedge2V , x[edge1UIndex] );
										leftSide3 = cplex.diff(xedge2U , x[edge1VIndex] );
										leftSide4 = cplex.diff(xedge2V , x[edge1VIndex] );
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 5: /*  e2 (route edge) is SW (NW because y is inverted) of e1: 225 (135): -z1 */
	
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff( z1edge2U , z1[edge1UIndex]);
										leftSide2 = cplex.diff( z1edge2V , z1[edge1UIndex]);
										leftSide3 = cplex.diff( z1edge2U , z1[edge1VIndex]);
										leftSide4 = cplex.diff( z1edge2V , z1[edge1VIndex]);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 6:/*  e2 (route edge) is S (N because y is inverted) of e1: 270 (90)  : -y */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(  yedge2U ,y[edge1UIndex] );
										leftSide2 = cplex.diff(  yedge2V, y[edge1UIndex]);
										leftSide3 = cplex.diff(  yedge2U ,y[edge1VIndex] );
										leftSide4 = cplex.diff(  yedge2V, y[edge1VIndex]);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 7:/*  e2 (route edge) is SE (NE because y is inverted) of e1: 315 (45)  : z2*/
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(z2[edge1UIndex], z2edge2U );
										leftSide2 = cplex.diff(z2[edge1UIndex], z2edge2V );
										leftSide3 = cplex.diff(z2[edge1VIndex], z2edge2U );
										leftSide4 = cplex.diff(z2[edge1VIndex], z2edge2V );
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
	
										break;
									default:
										break;
									}
	
								}
	
						}
				
				}
			
			
			
			/*********BEND MINIMIZATION*****************/

			
			
			
			/*Boolean variable to make condition of bends angle <-5, >-4 e <4, ou >5 measure bends angles (-7 to 7)*/
			/***∆dir(u, v,w) = dir(u, v) − dir(v,w)***/
			/***∆dir(u, v,w) ranges from −7 to 7****/
			
			IloNumVar[][] delta = new IloNumVar[b][];
			
			for (int i = 0; i < b; i++) {
				delta[i] = cplex.boolVarArray(3);
				
			}
			IloLinearNumExpr[] constraintBenMinimizationBooleans = new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				constraintBenMinimizationBooleans[i] = cplex.linearNumExpr();
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][0]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][1]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][2]);
				cplex.addEq(constraintBenMinimizationBooleans[i], 2.0);
			}
			double MBend = 15; /*diference between 2 dir */
			IloNumVar[] bendDir = cplex.intVarArray(b, -7, 7);
			/* bendDir[1] = dir[0] - dir[1]*/

			for (int i = 0; i < b; i++) {
				cplex.addEq(bendDir[i], cplex.diff(dir[i][0], dir[i + 1][0]));
			}
			
			for (int i = 0; i < b; i++) {
				cplex.addLe(bendDir[i], cplex.diff(cplex.prod(MBend, delta[i][0]), 5));
				cplex.addGe(bendDir[i], cplex.diff(5 , cplex.prod(MBend, delta[i][1])));
				cplex.addLe(bendDir[i], cplex.sum(4 , cplex.prod(MBend, delta[i][2])));
				cplex.addGe(bendDir[i], cplex.diff(-4 , cplex.prod(MBend, delta[i][2])));
				
			}
			
			IloNumVar[] bendCost = cplex.intVarArray(b, 0, 3);
			IloLinearNumExpr[] bendCostConstraint =  new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				bendCostConstraint[i] = cplex.linearNumExpr();
				bendCostConstraint[i].addTerm(1, bendDir[i]);
				bendCostConstraint[i].addTerm(-8, delta[i][0]);
				bendCostConstraint[i].addTerm(8, delta[i][1]);
				cplex.addGe(bendCostConstraint[i], cplex.prod(-1, bendCost[i]) );
				cplex.addLe(bendCostConstraint[i], bendCost[i] );
				

				
			}

			IloNumExpr sumBendCost =  cplex.sum(bendCost);

			
			
			/*********Distance Minization Anchors*****************/
			ArrayList<Integer> distNodeIndex = new ArrayList<Integer>();
			for (int i = 0; i < n; i++) {
				int pathIndex = pathRelevantPointsIndex.get(i);
				if( 	//path.getNodeList().get(pathIndex).getDegree() != 2 ||
						path.getNodeList().get(pathIndex).isTopoAnchor()
						) 
					distNodeIndex.add(i);				
			}
			int ndA = distNodeIndex.size();
			IloNumVar[] dXA  = cplex.numVarArray(ndA, 0, Double.MAX_VALUE);
			IloNumVar[] dYA  = cplex.numVarArray(ndA, 0, Double.MAX_VALUE);
			for (int i = 0; i < ndA; i++) {
				int pathPtIndex = distNodeIndex.get(i);
				
				IloNumExpr xLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getX(), x[pathPtIndex]);
				IloNumExpr yLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getY(), y[pathPtIndex]);
				
				cplex.addLe(xLeftSide,dXA[i]);
				cplex.addGe(xLeftSide, cplex.prod(-1, dXA[i]));
				
				
				cplex.addLe(yLeftSide, dYA[i]);
				cplex.addGe(yLeftSide, cplex.prod(-1, dYA[i]));

				
			}
			IloNumExpr sumXDistA =  cplex.sum(dXA);
			IloNumExpr sumYDistA =  cplex.sum(dYA);
			
			
			
			/*********Distance Minization *****************/

//			ArrayList<Integer> distNodeIndex = new ArrayList<Integer>();
//			for (int i = 0; i < n; i++) {
//				int pathIndex = pathRelevantPointsIndex.get(i);
////				if( 	path.getNodeList().get(pathIndex).getDegree() != 2 ||
////						path.getNodeList().get(pathIndex).isTopoAnchor()
////						) 
//					distNodeIndex.add(i);				
//			}
//			int nd = distNodeIndex.size();
//			IloNumVar[] dX  = cplex.numVarArray(nd, 0, Double.MAX_VALUE);
//			IloNumVar[] dY  = cplex.numVarArray(nd, 0, Double.MAX_VALUE);
//			double factor = 1;
//			for (int i = 0; i < nd; i++) {
//				int pathPtIndex = distNodeIndex.get(i);
//				
//				IloNumExpr xLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getX(), x[pathPtIndex]);
//				IloNumExpr yLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getY(), y[pathPtIndex]);
//				
//				cplex.addLe(xLeftSide,cplex.prod(factor, dX[i]));
//				cplex.addGe(xLeftSide, cplex.prod(-1*factor, dX[i]));
//				
//				
//				cplex.addLe(yLeftSide, cplex.prod(factor,dY[i]));
//				cplex.addGe(yLeftSide, cplex.prod(-1*factor, dY[i]));
//
//				
//			}
//			IloNumExpr sumXDist =  cplex.sum(dX);
//			IloNumExpr sumYDist =  cplex.sum(dY);
			
			
//			/*distance reduction*/
			
			IloNumVar[] dX  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			IloNumVar[] dY  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			//IloNumVar[] dMax  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			
			for (int i = 0; i < n; i++) {

					
				IloNumExpr xLeftSide = cplex.diff(pathRelevantPointsList.get(i).getX(), x[i]);
				IloNumExpr yLeftSide = cplex.diff(pathRelevantPointsList.get(i).getY(), y[i]);

				cplex.addLe(xLeftSide,dX[i]);
				cplex.addGe(xLeftSide, cplex.prod(-1, dX[i]));


				cplex.addLe(yLeftSide, dY[i]);
				cplex.addGe(yLeftSide, cplex.prod(-1, dY[i]));
				
			}
//			
			IloNumExpr sumXDist =  cplex.sum(dX);
			IloNumExpr sumYDist =  cplex.sum(dY);
			

			
			
			/*************Proportion diference minimization***********/

			/*** lambda: binary variable to determine if (xi - xi+1) is > 0 or < 0**
			 * lambda1 + lambda2 =1 -> if lambda1 = 1 - > lambda2 = 0 ->  (xi - xi+1) >= 0
			 * 							else lambda1 = 0 -> lambda2 = 1 -> (xi - xi+1) < 0
			 * 
			 * we want dx = |xi - xi+1| -> dx >= |xi - xi+1| & dx <= |xi - xi+1|
			 * if (xi - xi+1) >= 0 -> dx >= xi - xi+1 & dx <= xi - xi+1
			 * if (xi - xi+1) < 0  -> dx >= -(xi - xi+1) & dx <= -(xi - xi+1) 
			 * 
			 * so
			 * dx <= (xi - xi+1) + M(1-lambda1) eq1
			 * dx >= (xi - xi+1) - M(1-lambda1) eq2
			 * 
			 * or
			 * dx >= -(xi - xi+1) - M(1-lambda2) eq3
			 * dx <= -(xi - xi+1) + M(1-lambda2) eq4
			 * 
			 * if lambda1 = 1 & lambda2 = 0  => constraint dx = (xi - xi+1) & e3 and eq4 are trivialy fulfiled(makes no diference) 
			 * else lambda1 = 0 & lambda2 = 1  => constraint dx = -(xi - xi+1) & e1 and eq2 are trivialy fulfiled(makes no diference) 
			 * 
			 * 
			 * 
			 * */
			
			int mP = m; /*route edges number: respect order on edge list*/ 
			
			IloNumVar[][] xLambda = new IloNumVar[mP][];
			IloNumVar[][] yLambda = new IloNumVar[mP][];
			
			for (int i = 0; i < mP; i++) {
				xLambda[i] = cplex.boolVarArray(2);
				yLambda[i] = cplex.boolVarArray(2);
				xLambda[i][0].setName("proporMinxLambda0");
				xLambda[i][1].setName("proporMinxLambda1");
				yLambda[i][0].setName("proporMinyLambda0");
				yLambda[i][1].setName("proporMinyLambda1");
				
			}
			IloLinearNumExpr[] xLambdaConstraint = new IloLinearNumExpr[mP];
			IloLinearNumExpr[] yLambdaConstraint = new IloLinearNumExpr[mP];
			for (int i = 0; i < mP; i++) {
				xLambdaConstraint[i] = cplex.linearNumExpr();
				xLambdaConstraint[i].addTerm(1.0,xLambda[i][0]);
				xLambdaConstraint[i].addTerm(1.0,xLambda[i][1]);

				cplex.addEq(xLambdaConstraint[i], 1.0);
				
				
				yLambdaConstraint[i] = cplex.linearNumExpr();
				yLambdaConstraint[i].addTerm(1.0,yLambda[i][0]);
				yLambdaConstraint[i].addTerm(1.0,yLambda[i][1]);

				cplex.addEq(yLambdaConstraint[i], 1.0);
				
			}
			
			 
			/*  maxAdjD distance between two adjacent vertices vertices! it was MaxD*/
			MAdjVertice = pahtLength;
			IloNumVar[] absoluteDiffX  = cplex.numVarArray(mP, 0, 4* maxAdjDist);
			IloNumVar[] absoluteDiffY  = cplex.numVarArray(mP, 0, 4* maxAdjDist);
			
			for (int i = 0; i < mP; i++) {
				
				int v1Index = i;
				int v2Index = i+1;
				
				/***dx = |xi - xi+1|***/
				IloNumExpr diffX = cplex.diff(x[v1Index], x[v2Index]);
				
				cplex.addLe(absoluteDiffX[i], cplex.sum(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffX[i], cplex.diff(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffX[i], cplex.diff(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffX[i], cplex.sum(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq4.
				
				/***dy = |xi - yi+1|***/
				IloNumExpr diffY = cplex.diff(y[v1Index], y[v2Index]);
				
				cplex.addLe(absoluteDiffY[i], cplex.sum(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffY[i], cplex.diff(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffY[i], cplex.diff(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffY[i], cplex.sum(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq4.

				
				
			}
			/***WE have absDiffX and absDiffY
			 * We wanto to minimize:
			 * Abs((absDiffX[i] + absDiffY[i])L - lengthEdge[i]/L)
			 * 
			 * ****/
			/**
			 * If you are minimizing an increasing function of |x| (or maximizing a decreasing function, of course), you can always have the aboslute value of any quantity x in a lp as a variable absx such as:

				absx >= x -> x<=absx
				absx >= -x -> x >= -absx
				{leftside = x
				dX = absx}
				It works because the value absx will 'tend' to its lower bound, so it will either reach x or -x.

				On the other hand, if you are minimizing a decreasing function of |x|, your problem is not convex and cannot be modelled as a lp.
			 */
			ArrayList<Double> edgeProportion = new ArrayList<Double>();
			for (int i = 0; i < mP; i++) {
	
				//edgeProportion.add(pt1.distance(pt2)/routeLenght);
				edgeProportion.add(pathRelevantPointsList.get(i).distance(pathRelevantPointsList.get(i+1))/pahtLength);
			}
			IloNumVar[] absolutEdgeProportionDifference  = cplex.numVarArray(mP, 0, Double.MAX_VALUE);
			IloNumExpr[] finalEdgeProportion = new IloNumExpr[mP];
			for (int i = 0; i < mP; i++) {
				finalEdgeProportion[i] = cplex.prod(1/pahtLength,cplex.sum(absoluteDiffX[i],absoluteDiffY[i]));
				/*can I use cplex.Equal direct? yes absX and absY is always positiv*/
				cplex.addLe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), absolutEdgeProportionDifference[i]);
				cplex.addGe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), cplex.prod(-1, absolutEdgeProportionDifference[i]));
			}
			
			IloNumExpr sumAbsolutEdgeProportionDifference =  cplex.sum(absolutEdgeProportionDifference);

			
			/****Edges orientation difference minimization*********/
			IloNumVar[] dirDiff  = cplex.numVarArray(m, -7, 7);
			for (int i = 0; i < m; i++) {
				cplex.addEq(dirDiff[i], cplex.diff(sec[i][0][1] ,dir[i][0]));
			}
			
			IloNumVar[] dirCost = cplex.boolVarArray(m);
			for (int i = 0; i < m; i++) {

				cplex.addLe(dirDiff[i],cplex.prod(8, dirCost[i]));
				cplex.addGe(dirDiff[i],  cplex.prod(-8, dirCost[i]));
			}

//			IloNumVar[][] zeta = new IloNumVar[m][];
//			
//			for (int i = 0; i < m; i++) {
//				zeta[i] = cplex.boolVarArray(3);
//				zeta[i][0].setName("ZETADirMin0");
//				zeta[i][1].setName("ZETADirMin1");
//				zeta[i][2].setName("ZETADirMin2");
//				
//			}
//			IloLinearNumExpr[] constraintDirDiffMinimizationBooleans = new IloLinearNumExpr[m];
//			for (int i = 0; i < m; i++) {
//				constraintDirDiffMinimizationBooleans[i] = cplex.linearNumExpr();
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][0]);
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][1]);
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][2]);
//				cplex.addEq(constraintDirDiffMinimizationBooleans[i], 2.0);
//			}
//
//			MDir = 8;
//			for (int i = 0; i < m; i++) {
//				cplex.addLe(dirDiff[i], cplex.diff(cplex.prod(MDir, zeta[i][0]), 5));
//				cplex.addGe(dirDiff[i], cplex.diff(5 , cplex.prod(MDir, zeta[i][1])));
//				cplex.addLe(dirDiff[i], cplex.sum(4 , cplex.prod(MDir, zeta[i][2])));
//				cplex.addGe(dirDiff[i], cplex.diff(-4 , cplex.prod(MDir, zeta[i][2])));
//				
//			}
//			IloNumVar[] dirCost = cplex.intVarArray(m, 0, 4);
//			IloLinearNumExpr[] dirCostConstraint =  new IloLinearNumExpr[m];
//			for (int i = 0; i < m; i++) {
//				dirCostConstraint[i] = cplex.linearNumExpr();
//				dirCostConstraint[i].addTerm(1, dirDiff[i]);
//				dirCostConstraint[i].addTerm(-8, zeta[i][0]);
//				dirCostConstraint[i].addTerm(8, zeta[i][1]);
//				cplex.addGe(dirCostConstraint[i], cplex.prod(-1, dirCost[i]) );
//				cplex.addLe(dirCostConstraint[i], dirCost[i] );
//
//				
//			}
			IloNumExpr sumDirCost =  cplex.sum(dirCost);  
			
			
			
			/*********OBJECTIVE*****************/
			IloNumExpr sumBendCostNormal = cplex.prod(0.5/Math.sqrt(b),sumBendCost);
			IloNumExpr sumBendCostFactor = cplex.prod(bendFactor,sumBendCostNormal);
			
			double maxExted = octBox.getMaxExtend();
			IloNumExpr sumDistCostNormal = cplex.prod((20.0)/(n*maxExted), cplex.sum(sumXDist, sumYDist ));			

			//IloNumExpr sumDistCostNormal = cplex.prod((50.0)/(n*pahtLength), cplex.sum(sumXDist, sumYDist ));			
			IloNumExpr sumDistCostFactor = cplex.prod(distFactor, sumDistCostNormal);
			
			
			IloNumExpr sumDistCostNormalA = cplex.prod((10.0)/(ndA), cplex.sum(sumXDistA, sumYDistA ));
			IloNumExpr sumDistCostFactorA = cplex.prod(distFactor, sumDistCostNormalA);
			
			
			IloNumExpr sumAbsolutEdgeProportionDifferenceNormal = cplex.prod(10.0/(mP), sumAbsolutEdgeProportionDifference);			
			IloNumExpr sumEdgeProportionFactor = cplex.prod(proportionFactor, sumAbsolutEdgeProportionDifferenceNormal);
			
			IloNumExpr sumDirCostNormal = cplex.prod((5.0/m),sumDirCost);
			IloNumExpr sumDirCostFactor = cplex.prod((dirFactor),sumDirCostNormal);
			
			IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumDistCostFactorA,  sumBendCostFactor,  sumEdgeProportionFactor, sumDirCostFactor ) ;
			//IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumBendCostFactor, sumDirCostFactor) ;

			//IloNumExpr objective = sumBendCostFactor ;
			//IloNumExpr objective = cplex.sum(sumXDist, sumYDist ) ;

			

			
			cplex.addMinimize(objective);
			if(executionTimeLimit > 0 ){
				double limitTimeInSeconds = ((double)(executionTimeLimit))/1000;
				cplex.setParam(IloCplex.DoubleParam.TiLim, limitTimeInSeconds);
			}
			else if(executionTimeLimit <0 ){
				//cplex.setParam(IloCplex.Param.MIP.Limits.Solutions, 1);
				cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, (-(double)executionTimeLimit)/100.0);
				
				cplex.setParam(IloCplex.DoubleParam.TiLim, 120);
			}
			

			/*control display information*/
			//cplex.setParam(IloCplex.Param.MIP.Display, 1);
			
			if(cplex.solve()){
				System.out.println("obj =  " +cplex.getObjValue());
				for (int i = 0; i < n; i++) {
					Point2D.Double pt = new Point2D.Double(cplex.getValue(x[i]),cplex.getValue(y[i])); 
					mipLineString.add(pt);
				}	
				ArrayList<Point2D> finalPoints = new ArrayList<Point2D>();
				/****USE IT TO REFILL PATH IF YOU NEED TO SIMPLIFY****/
				
				finalPoints.add(mipLineString.get(0));
				for(int i = 1; i < pathRelevantPointsIndex.size()  ; i++){
					int index = pathRelevantPointsIndex.get(i);
					int lastIndexInFinalPoints = finalPoints.size() - 1;
					/*adicional o proximo ponto relevante entre ele e o anterior na linha reta com os ponts complementares*/
					finalPoints.add( mipLineString.get(i) );
					finalPoints = GeometricOperation.fillLine( finalPoints , transPointList,  index - lastIndexInFinalPoints   , lastIndexInFinalPoints );
				}
				
//				ArrayList<Point2D> routePathNormalized =  route.getRoutePath().asJava2DList(1);
//				finalPoints.add(mipLineString.get(0));
//				for(int i = 1; i < route.getRelevantPointIndex().size() ; i++){
//					int index = route.getRelevantPointIndex().get(i);
//					int lastIndexInFinalPoints = finalPoints.size() - 1;
//					/*adicional o proximo ponto relevante entre ele e o anterior na linha reta com os ponts complementares*/
//					finalPoints.add( mipLineString.get(i) );
//					finalPoints = GeometricOperation.fillLine( finalPoints , routePathNormalized,  index - lastIndexInFinalPoints   , lastIndexInFinalPoints );
//				}
//				route.getRoutePath().updatePathXNodes( finalPoints);
				
				
				//System.out.println("Only ADj");
			

				
//				for(int i = 0; i < n ; i++){
//					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
	//
//				}
				
//				for (int i = 0; i < mP; i++){
//					System.out.println("Edge: " + i);
//					System.out.println("AbsDiffX-> =  " + cplex.getValue(absoluteDiffX[i]) + " AbsDiffY-> =  " + cplex.getValue(absoluteDiffY[i]) );
//					System.out.println("True edge proportion: " + edgeProportion.get(i) + "Final edgeProportion: " + cplex.getValue(finalEdgeProportion[i]) + "Absolut Difference: " + cplex.getValue(absolutEdgeProportionDifference[i]));
//				}
//				for (int i = 0; i < b ; i++){
//					//System.out.println("benDir-> =  " + cplex.getValue(bendDir[i-2]) + " bencost-> =  " + cplex.getValue(bendCost[i-2]) );
//					//System.out.println("gama0-> =  " + cplex.getValue(gama[i-2][0]) + " gama1-> =  " + cplex.getValue(gama[i-2][1])  + " gama2-> =  " + cplex.getValue(gama[i-2][2]));
//					
//					//System.out.println("origTurnDir->= "+ origTurnDir[i -1] + " benDir-> =  " + cplex.getValue(bendDir[i-1]) + " bencost-> =  " + cplex.getValue(bendCost[i-1]) );
//				}
//				
//				for (int i = 0; i <n ; i++){
//					System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//				}
//				for (int i = 0; i <nd ; i++){
//					System.out.println("dx"+ i + " =  " + cplex.getValue(dX[i]) + " dy"+ i + " =  " + cplex.getValue(dY[i]) );
//				}
				
//				for (int i = 0; i < m ; i++){
//					//System.out.println("benDir-> =  " + cplex.getValue(bendDir[i-2]) + " bencost-> =  " + cplex.getValue(bendCost[i-2]) );
//					System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dir-> =  " + cplex.getValue(dir[i][0]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]));
//					
//				}
//				for (int i = 0; i < nA ; i++){
//					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//				}

//				for (int i = nR -1; i < m ; i++){
//					
//					System.out.println("x"+ i+1 + " =  " + cplex.getValue(x[i+1]) + " y"+ i+1 + " =  " + cplex.getValue(y[i+1]) + " z1"+ i+1 + " =  " + cplex.getValue(z1[i+1]) + " z2"+ i+1 + " =  " + cplex.getValue(z2[i+1]));
//				//	System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
	//
//					System.out.println("Route Point: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom() );
//					System.out.println("Adj Point: " +streetNodeMap.get(edgeList.get(i)[1]).getxGeom() );
//					System.out.println("Dist: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom().distance(streetNodeMap.get(edgeList.get(i)[1]).getxGeom()) );
//					System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dir-> =  " + cplex.getValue(dir[i][0]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]));
//					System.out.println();
//				}
				
				
				
//				numInterPoints = intersectionNodeIdList.size();							
//				for (int i = 0; i < numInterPoints; i++) {
//					
	//
//					int vId = intersectionNodeIdList.get(i);
////					System.out.println();
////					System.out.println("Route relevant point n  " + routeNodeIdList.indexOf(vId) + " of id "+vId+"  is intersection and its degree is " + circularOrderList.get(vId).size() );
////					System.out.println("Interpoint"+i+" Intersection Id: " + vId);
//					
//					for(int j = 0; j < beta[i].length ; j ++ ){
//						int indexDirVU1 = 0, indexDirVU2 = 0;
////						System.out.println("--Edge " + j + " and " + (j+1));
//						int u1Id, u2Id;
//						u1Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j).getId();
////						System.out.println("u1ID="+u1Id); 
//						/*se for vertive una orden circular seleciona u2 como o primeiro da lista*/
//						if(j < (beta[i].length -1))
//							u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j + 1).getId();
//						else 
//							u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(0).getId();
////						System.out.println("u2ID="+u2Id);
//						/***identify the edges (v, u1) and (v, u2)**/
//						boolean foundEdgeToU1 = false, foundEdgeToU2 = false;
//						int edgeToU1IsInverted = 0, edgeToU2IsInverted = 0;
//						
//						for(int k = 0; k < edgeList.size(); k++){
	//
//							/*intersectionNOdeid contain only route nodes, the adj node are always edge[1]. Otherwise is necessary to check the inverse*/
//							if(edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u1Id) {
//								indexDirVU1 = k;
//								foundEdgeToU1 = true;
//								edgeToU1IsInverted = 0;
//							}
//							else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u1Id){
//								indexDirVU1 = k;
//								foundEdgeToU1 = true;
//								edgeToU1IsInverted = 1;
//							}
//							if( edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u2Id ){
//								indexDirVU2 = k;
//								foundEdgeToU2 = true;
//								edgeToU2IsInverted = 0;
//							}
//							else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u2Id){
//								indexDirVU2 = k;
//								foundEdgeToU2 = true;
//								edgeToU2IsInverted = 1;
//							}
//							if(foundEdgeToU1 && foundEdgeToU2)
//								break;
//						}
//						
////						System.out.println("EdgeIndex to u1ID= "+ indexDirVU1 + " dir: " + cplex.getValue(dir[indexDirVU1][edgeToU1IsInverted]) + " found: " + foundEdgeToU1  + " is inverted: " + edgeToU1IsInverted); 
////						System.out.println("EdgeIndex to u2ID= "+ indexDirVU2 + " dir: " + cplex.getValue(dir[indexDirVU2][edgeToU2IsInverted]) + " found: " + foundEdgeToU2 + " is inverted: " + edgeToU2IsInverted); 
//				
	//
//				
//					}
////					System.out.println();
//				}
				
				
				System.out.println("distCost/Normal =  " + (cplex.getValue(sumXDist) + cplex.getValue(sumYDist)) + " / " +cplex.getValue(sumDistCostNormal) + " sumBendCost/Normal = " + cplex.getValue(sumBendCost)+ " / " +cplex.getValue(sumBendCostNormal) + " sumDirCost/Normal = " + cplex.getValue(sumDirCost) + " / " + cplex.getValue(sumDirCostNormal) + " sumEdgeProportionCost/Normal = " + cplex.getValue(sumAbsolutEdgeProportionDifference)+ " / " + cplex.getValue(sumAbsolutEdgeProportionDifferenceNormal) );
				System.out.println("distFactor =  " + distFactor + " bendFactor = " + bendFactor + " dirFactor = " + dirFactor + " proporFactor = " + proportionFactor );
				System.out.println("distCostNormalFactor =  " + cplex.getValue(sumDistCostFactor) + " sumBendCostFactor = " + cplex.getValue(sumBendCostFactor) + " sumDirCostFactor = " + cplex.getValue(sumDirCostFactor) + " sumProportionFactor = " + cplex.getValue(sumEdgeProportionFactor) );

				
				System.out.println("Final = " + cplex.getObjValue());
				
				cplex.end();
				path.updatePathXNodes2( finalPoints );
				end = System.currentTimeMillis();
				System.out.println("Path Scehmatization- Nodes: " + n+ " Execution Time: "   + (end - start) );

				return finalPoints;
				
			}
			else{
				networkpathOptimizer2(path,transPointList);
				throw new Exception("Cannot solve MIP Model on time");
				
			}
			
			
			
			
			
		}
		
		
		public static ArrayList<Point2D> landmarkAbstractOptimizerLazy( 
				Path path, PolygonalTopo polygonTopo, ArrayList<Path> pathList, ArrayList<Point2D> transPointList, double proportion, Polygon boundingPolygon,
				Map<Integer, StreetNode> nodeMap, StreetNetworkTopological streetNetworkTopological, 
				double bendFactor,  double distFactor, double proportionFactor, double dirFactor, 
				Boolean checkSelfTopology, Boolean checkTopology,
				double minNonAdjEdgeDist, 
				int executionTimeLimit, PathReport pathResultReport) throws IloException, Exception   {
			
			
			/*Schematized edges inside octilinear bounding box to check for topology*/
			ArrayList<Integer[]> inOctBoxXEdges = new ArrayList<Integer[]>();
			
			//Polygon boundingPolygon = (Polygon)GeoConvertionsOperations.Java2DToJTSGeometry( new OctlinearBoundingBox(transPointList, 2*minNonAdjEdgeDist, extendLimit +0.1, 0).getBoundingPolygon(), Geometries.POLYGON);

			if( polygonTopo.getPolygonalFeature().getType().equals("urban")  ) {
				System.out.println(polygonTopo.getPolygonalFeature().getName() + " is urban bastard!");
			}
			
			
			for(Path p: pathList) {
				//if(p.isRoute() ||p.isRouteAdjEdge())
				//if(p.isRoute())		
				/*if is it was schematiza and it is not a point like landmark edge*/
				if(p.isWasSchematized() && !(p.getNodeList().get(0).getIsPointLMNode() > 0 || p.getNodeList().get(p.getNodeList().size() -1).getIsPointLMNode() > 0)) {
					if( !(polygonTopo.getPolygonalFeature().getType().equals("urban")  && p.isRouteAdjEdge()  ) ) {	
						PointsPolar polarPoints = new PointsPolar();
						polarPoints  = GeometricOperation.toPolar(p.asJava2DList(2));
						for(int i = 0; i < p.getNodeList().size() - 1; i++) {
							if(!path.getNodeList().contains(p.getNodeList().get(i)) && !path.getNodeList().contains(p.getNodeList().get(i +1))) {

								if (boundingPolygon.contains( p.getNodeList().get(i).getxGeom()  ) || boundingPolygon.contains( p.getNodeList().get(i +1).getxGeom() )){
									int indexU = i;
									int indexV = i+1;
									boolean foundBend = false; 
									while(!foundBend ) {

										if(indexV < p.getNodeList().size() -1 && !path.getNodeList().contains(p.getNodeList().get(indexV + 1))
												&& Math.abs(polarPoints.getPoints().get(indexV -1).getTheta() - polarPoints.getPoints().get(indexV).getTheta()) < 0.001
												&& boundingPolygon.contains( p.getNodeList().get(indexV + 1).getxGeom() ) )
											indexV++;
										else {
											Integer[] edge = new Integer[2];
											edge[0] = p.getNodeList().get(indexU).getId();
											edge[1] = p.getNodeList().get(indexV).getId();
											inOctBoxXEdges.add(edge);
											foundBend = true;
											i = indexV -1;
										}
									}




								}
							}

						}
					}
				}
			}
			
			long start, end;
			start = System.currentTimeMillis(); 
			ArrayList<ArrayList<Integer>> selfEdgePaarToCheck = new ArrayList<ArrayList<Integer>>();
			ArrayList<ArrayList<Integer>> withNonPathEdgePaarToCheck = new ArrayList<ArrayList<Integer>>();
			int executionsCount = 1;

			boolean violateTopology = false;

			violateTopology = landmarkAbstractOptimizer3DirTopoRelevant2( 
					path, pathList,  transPointList, proportion,inOctBoxXEdges, 
					nodeMap,  streetNetworkTopological, 
					bendFactor,  distFactor,  proportionFactor, dirFactor, 
					checkSelfTopology,  checkTopology,
					minNonAdjEdgeDist, 
					executionTimeLimit,  pathResultReport, selfEdgePaarToCheck, withNonPathEdgePaarToCheck) ;

			System.out.println("Paar of self edges that violates topology:");
			System.out.println(selfEdgePaarToCheck);
			
			System.out.println("Paar of non path edges that violates topology:");
			System.out.println(withNonPathEdgePaarToCheck);
			
			if(checkSelfTopology || checkTopology)
				while(violateTopology && executionsCount < 10) {

					executionsCount++;
					violateTopology = landmarkAbstractOptimizer3DirTopoRelevant2( 
							path, pathList,  transPointList, proportion, inOctBoxXEdges, 
							nodeMap,  streetNetworkTopological, 
							bendFactor,  distFactor,  proportionFactor, dirFactor, 
							checkSelfTopology,  checkTopology,
							minNonAdjEdgeDist, 
							executionTimeLimit,  pathResultReport, selfEdgePaarToCheck, withNonPathEdgePaarToCheck) ;
				}

			end = System.currentTimeMillis();
			pathResultReport.setExecutionTime(end - start);
			pathResultReport.setPathType(polygonTopo.getType());
			pathResultReport.setProportion(proportion);
			pathResultReport.setFixedExtraCrossings(selfEdgePaarToCheck.size() +  withNonPathEdgePaarToCheck.size());
			pathResultReport.setExecutions(executionsCount);

			return null;
		}
		
		
		public static boolean landmarkAbstractOptimizer3DirTopoRelevant2( 
				Path path, ArrayList<Path> pathList, ArrayList<Point2D> transPointList, double proportion, ArrayList<Integer[]> topoCheckEdgeList,
				Map<Integer, StreetNode> nodeMap, StreetNetworkTopological streetNetworkTopological, 
				double bendFactor,  double distFactor, double proportionFactor, double dirFactor, 
				Boolean checkSelfTopology, Boolean checkTopology,
				double minNonAdjEdgeDist, 
				int executionTimeLimit, PathReport pathResultReport, ArrayList<ArrayList<Integer>> selfEdgePaarToCheck , ArrayList<ArrayList<Integer>> nonPathPaarToCheck) throws IloException, Exception   {
			//System.out.println("OPTIMIZER Network PATH - > ACTION!!!!!");	

			long start, end;
			start = System.currentTimeMillis(); 
			
			IloCplex cplex = new IloCplex();
			
			
			ArrayList<Point2D> mipLineString =  new ArrayList<Point2D>();
			ArrayList<Point2D> pathRelevantPointsList = new ArrayList<Point2D>();
			ArrayList<Integer> pathRelevantPointsIndex = new ArrayList<Integer>();

			for(int i = 0; i < transPointList.size(); i++) {
				if(path.getNodeList().get(i).isRelevantRouteNode()) {
					pathRelevantPointsList.add(transPointList.get(i));
					pathRelevantPointsIndex.add(i);
				}
			}
//			boolean isClose = true;
//			if(isClose)
//				transPointList.add(transPointList.get(0));
			
			
			
			//System.out.println(lineString);
			int n = pathRelevantPointsList.size();
			int m = n -1; /*number of edges in a paths*/
			int b = m - 1; /*number of bends in a path*/
			
		
			ArrayList<Point2D> pointsToBoundingBox = new ArrayList<Point2D>(transPointList);
			for(int i = 0; i < topoCheckEdgeList.size(); i++) {
				pointsToBoundingBox.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(nodeMap.get(topoCheckEdgeList.get(i)[0]).getxGeom()));
				pointsToBoundingBox.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(nodeMap.get(topoCheckEdgeList.get(i)[1]).getxGeom()));
			}

			double extendLimit = 0.05;
			OctlinearBoundingBox octBox = new OctlinearBoundingBox(pointsToBoundingBox, 2*minNonAdjEdgeDist, extendLimit +0.1, 2);
			double maxAdjDist = octBox.getMaxAdjDist();
			
			
	//
//	        System.out.println("maxD: " + maxD);
//	        System.out.println("N: " + n);
			
			double maxExtend = GeometricOperation.diagonalOf(transPointList);
			double octBoxMaxExted = octBox.getMaxExtend();
			double pathLength = GeometricOperation.length(transPointList);
			double L = pathLength/(n*100); /* Minimal distance between two vertices*/
			System.out.println("Minimal distance between two vertices: " + L);
			System.out.println("Mininal distance from route " + minNonAdjEdgeDist);
			System.out.println("Linestring Legth " + pathLength);
			System.out.println("Max Extend " + maxExtend);
			System.out.println("OctBox Max Extend " + octBoxMaxExted);

			//PathReport report = new PathReport();
			pathResultReport.setNumberOfNodes(n);
			pathResultReport.setNumberOfEdges(m);
			pathResultReport.setPathLenght(pathLength);
			/*********COORDINATES CONSTRAINT*****************/
	       

	        
			IloNumVar[] x  = cplex.numVarArray(n, octBox.getMinX(), octBox.getMaxX());
			IloNumVar[] y  = cplex.numVarArray(n,  octBox.getMinY(), octBox.getMaxY());
			IloNumVar[] z1  = cplex.numVarArray(n, octBox.getMinZ1(), octBox.getMaxZ1());
			IloNumVar[] z2  = cplex.numVarArray(n, octBox.getMinZ2(), octBox.getMaxZ2());
			
			
//			IloNumVar[] x  = cplex.numVarArray(n, -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] y  = cplex.numVarArray(n,  -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] z1  = cplex.numVarArray(n,  -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] z2  = cplex.numVarArray(n, -Double.MAX_VALUE, Double.MAX_VALUE);
			
			IloNumExpr[] z1Constraint = new IloNumExpr[n];
			IloNumExpr[] z2Constraint = new IloNumExpr[n];
			
			for (int i = 0; i < n; i++) {
				/*do we need to multiply by 2?*/
//				z1Constraint[i] = cplex.sum(x[i],y[i]);
//				z2Constraint[i] = cplex.diff(x[i],y[i]);
				
//				/*do we need to multiply by 2?*/
				z1Constraint[i] = cplex.prod(0.5,cplex.sum(x[i],y[i]));
				z2Constraint[i] = cplex.prod(0.5, cplex.diff(x[i],y[i]));
			}
			
			/*Add z constraint to model constranint*/ 
			for (int i = 0; i < n; i++) {
				cplex.addEq(z1[i], z1Constraint[i]);
				cplex.addEq(z2[i], z2Constraint[i]);
							
				
			}
//			/*fix position of first vertex*/
//			cplex.addEq(x[0], lineString.get(0).getX());
//			cplex.addEq(y[0], lineString.get(0).getY());
//			
//			/***fix position of already schematized vertices*/
//			/*I cannot fix position of 2 adjacent edges*/
			
//			for(int i = 0; i < n -1; i++) {
//				
//				
//				if(path.getNodeList().get(i).getxGeom() != null && !path.getNodeList().get(i).isAnchor()) {
//					
//					cplex.addEq(x[i], path.getNodeList().get(i).getxGeom().getX());
//					cplex.addEq(y[i], path.getNodeList().get(i).getxGeom().getY());
//				}
//				
//			}
//////			
////			
			int startEdgeDirection = -1;
			int endEdgeDirection = -1;
			
			if(path.getNodeList().get(0).getId() == path.getNodeList().get(path.getNodeList().size() -1).getId()) {				
				cplex.addEq(x[n-1], x[0]);
				cplex.addEq(y[n-1], y[0]);
			}
			else {
				if(path.getNodeList().get(0).getxGeom() != null ) {				
					cplex.addEq(x[0], path.getNodeList().get(0).getxGeom().getX());
					cplex.addEq(y[0], path.getNodeList().get(0).getxGeom().getY());
					if(path.getNodeList().get(1).getxGeom() != null ) {
					startEdgeDirection = GeometricOperation.sectorOf(
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
									path.getNodeList().get(0).getxGeom().getX(), path.getNodeList().get(0).getxGeom().getY(),
									path.getNodeList().get(1).getxGeom().getX(), path.getNodeList().get(1).getxGeom().getY() 					
									));
					}
				}
				else {
					cplex.addEq(x[0], transPointList.get(0).getX());
					cplex.addEq(y[0], transPointList.get(0).getY());
				}
				if(path.getNodeList().get(path.getNodeList().size() -1).getxGeom() != null ) {				
					cplex.addEq(x[n -1], path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getX());
					cplex.addEq(y[n -1], path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getY());
					if(path.getNodeList().get(path.getNodeList().size() -2).getxGeom() != null ) {
					endEdgeDirection = GeometricOperation.sectorOf(
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
									path.getNodeList().get(path.getNodeList().size() -2).getxGeom().getX(), path.getNodeList().get(path.getNodeList().size() -2).getxGeom().getY(),
									path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getX(), path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getY() 					
									));
					}
				}
				else {
					cplex.addEq(x[n -1], transPointList.get(transPointList.size() -1).getX());
					cplex.addEq(y[n -1], transPointList.get(transPointList.size() -1).getY());

				}
			}

			
//			
			
			/*********OCTALINEARITY CONSTRAINT*****************/
			
			/*sec[m][d][(pred,orig,succ)] defines the sector of the octilinear position an edges could lie
			 * m = number of edges
			 * d = direction of the edge 0 is going and 1 is back
			 * (pred,orig,succ) = 0 lies on the previous sector, 1 lies ont the original best sector, and 2 lies on the succecive sector*/ 
			int[][][] sec = new int[m][2][3];		
			/*int[] origBestSec = new int[m];
			int[] succBestSec = new int[m];
			int[] predBestSec = new int[m];*/
			
			for (int i = 0; i < m; i++) {

				if(i == 0 && startEdgeDirection >  -1)
					sec[i][0][1] = startEdgeDirection;

				else if (i == m-1 && endEdgeDirection >  -1)
					sec[i][0][1] = endEdgeDirection;
				else				
					sec[i][0][1] = GeometricOperation.sectorOf(
						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
								pathRelevantPointsList.get(i).getX(), pathRelevantPointsList.get(i).getY(),
								pathRelevantPointsList.get(i + 1).getX(), pathRelevantPointsList.get(i + 1).getY()					
								)
						);
				sec[i][0][0] =  Math.floorMod(sec[i][0][1] - 1, 8);
				sec[i][0][2] =  Math.floorMod(sec[i][0][1] + 1, 8);
				
				
				sec[i][1][1] = Math.floorMod(sec[i][0][1] + 4,8);
				sec[i][1][0] = Math.floorMod(sec[i][0][0] + 4,8);
				sec[i][1][2] = Math.floorMod(sec[i][0][2] + 4,8);
			}
		

			
			/*Boolean variable to restrict the orientation of the edge to orig = 1, succ = 2 and prd = 0*/
			IloNumVar[][] alpha = new IloNumVar[m][];
			
			for (int i = 0; i < m; i++) {
				alpha[i] = cplex.boolVarArray(3);
				
			}
			/**force alfa to best direction if start and end edges directions are set **/
			if(startEdgeDirection >  -1){

				cplex.addEq(alpha[0][0], 0);
				cplex.addEq(alpha[0][1], 1);
				cplex.addEq(alpha[0][2], 0);



			}
			if(endEdgeDirection >  -1){

				cplex.addEq(alpha[m-1][0], 0);
				cplex.addEq(alpha[m-1][1], 1);
				cplex.addEq(alpha[m-1][2], 0);



			}
			
			/*IloNumVar[] isOrig = cplex.boolVarArray(m);
			IloNumVar[] isSucc = cplex.boolVarArray(m);
			IloNumVar[] isPred = cplex.boolVarArray(m);*/
			
			IloLinearNumExpr[] constraintEdgeOrientation = new IloLinearNumExpr[m];
			
			for (int i = 0; i < m; i++) {
				constraintEdgeOrientation[i] = cplex.linearNumExpr();
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][0]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][1]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][2]);
				cplex.addEq(constraintEdgeOrientation[i], 1.0);
			}
			
			
			/*Variable dir[i][j] defines the direction of of edege i (j means de drirection of the edege*/
			IloNumVar[][] dir  = new IloNumVar[m][]; /*leave the last dimension to be defined with cplex*/
			
			for (int i = 0; i < m; i++) {
				dir[i] = cplex.intVarArray(2, 0, 7);/* array of size 2 because we need uv an vu*/
				
			}
			
			
			/*For each i ∈ {pred, orig, succ} we have the following set of constraints
			 dir(u, v)−seciu(v) ≤M(1−αi(u, v))
			−dir(u, v)+seciu(v) ≤M(1−αi(u, v))
			dir(v,u)−seciv(u) ≤M(1−αi(u, v))
			−dir(v,u)+seciv(u) ≤M(1−αi(u, v))
			
			∀{u, v} ∈ E,
			
			 *Here, if αi(u, v) = 0, the constraints in (4) are trivially fulfilled and do not influence the left-hand sides. 
			 *On the other hand, if αi(u, v) = 1, the four inequalities are equivalent to dir(u, v) = seciu(v) and
				dir(v,u) = seciv(u) as desired (equality
			 *
			 *
			 *
			 */
			
			double MDir = 8; 
			for (int i = 0; i < m; i++) {
				if(i == 0 && startEdgeDirection >  -1){
					cplex.addEq(dir[i][0],startEdgeDirection);
					cplex.addEq(dir[i][1],Math.floorMod(startEdgeDirection + 4,8));
				}
				else if (i == m-1 && endEdgeDirection >  -1){
					cplex.addEq(dir[i][0],endEdgeDirection);
					cplex.addEq(dir[i][1],Math.floorMod(endEdgeDirection + 4,8));
				}
				else{
					for (int j = 0; j < 3; j++) {
						IloNumExpr rightSide = cplex.prod(MDir, cplex.diff(1, alpha[i][j]));
						/*natural direction of the edge d=0*/
						IloNumExpr leftSide1 = cplex.diff(dir[i][0], sec[i][0][j]);
						IloNumExpr leftSide2 = cplex.diff(sec[i][0][j],  dir[i][0]);
						/*counter direction of the edge d= 1*/
						IloNumExpr leftSide3 = cplex.diff(dir[i][1], sec[i][1][j]);
						IloNumExpr leftSide4 = cplex.diff(sec[i][1][j],  dir[i][1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);
					
					}
				}
				
			
			}
			
			/*contraints to force the correct position of the vertices
			 * if sec original is 2 and alphaoriginal is true then forces x(u) and x(v) to equal
			 * and y(v) > y(u)*/
			double MAdjVertice = 2*maxAdjDist; /* D max distance between two vertices*/
			for (int i = 0; i < m; i++) {
				
				for (int j = 0; j < 3; j++) {
					IloNumExpr rightSide, rightSide2, leftSide1, leftSide2, leftSide3;
					/*do i need to constraint edges for both directions(sec[m][1])?*/
					switch (sec[i][0][j]) {
					case 0:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(y[i], y[i+1]);
						leftSide2 = cplex.diff(y[i+1],  y[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(x[i+1],  x[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
					
						break;
					case 7:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[i], z2[i+1]);
						leftSide2 = cplex.diff(z2[i+1],  z2[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z1[i+1],  z1[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;
						
					case 6:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[i], x[i+1]);
						leftSide2 = cplex.diff(x[i+1],  x[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(y[i+1],  y[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);

						break;
						
					case 5:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[i], z1[i+1]);
						leftSide2 = cplex.diff(z1[i+1],  z1[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z2[i],  z2[i + 1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					case 4:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(y[i], y[i+1]);
						leftSide2 = cplex.diff(y[i+1],  y[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(x[i],  x[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;
					case 3:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[i], z2[i+1]);
						leftSide2 = cplex.diff(z2[i+1],  z2[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z1[i],  z1[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					case 2:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[i], x[i+1]);
						leftSide2 = cplex.diff(x[i+1],  x[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(y[i],  y[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);

						break;	
					case 1:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[i], z1[i+1]);
						leftSide2 = cplex.diff(z1[i+1],  z1[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z2[i+1],  z2[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					default:
						break;
					}
					
				}
				
			}
			
			
//			/*******CHECK SELF PLANARITY*******///
			System.out.println("checkin topology: " + checkSelfTopology);
			if(checkSelfTopology){
				IloNumVar[][][] gama = new IloNumVar[m][m][];
				/*RElation of m edges from the path and r route edges check relative position to 8 orientations ∈{N,S,E,W,NE,NW,SE,SW}*/
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < m; j++)
						gama[i][j] = cplex.boolVarArray(8);
				
			
			
				IloLinearNumExpr[][] constraintEdgeTopology = new IloLinearNumExpr[m][m];
				/*SumOF{ γi(e1, e2) } ≥ 1   
				 * i∈{N,S,E,W,NE,NW,SE,SW} */
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < m; j++){
						//				for (int k = 0; k < 8; k++){
						//					if(k!=0)
						//						cplex.addEq(gama[i][j][k], 0);
						//					else
						//						cplex.addEq(gama[i][j][k], 1);
						//				}


						constraintEdgeTopology[i][j] = cplex.linearNumExpr();
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][0]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][1]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][2]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][3]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][4]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][5]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][6]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][7]);
						cplex.addGe(constraintEdgeTopology[i][j], 1.0);

					}


				for(ArrayList<Integer> edgePaar: selfEdgePaarToCheck) {

					/**need to review this + 1*/
					int i = edgePaar.get(0) + 1;
					int j = edgePaar.get(1) + 1;

					System.out.println("Checkin topology violation: " + i + " " + j );
					int edge1UIndex = i ;
					int edge1VIndex = i + 1 ;

					int edge2UIndex = j ;
					int edge2VIndex = j + 1 ;

					/*condition to guarantee that path edege i and j are not adjacents*/
					if( Math.abs(i - j) > 1  && Math.abs(i - j) < m -1 )
						for(int k =0; k < 8; k ++){
							/*whats the value of M(max X ou y dist of points) and dist Min*/ 
							double MTopology = maxExtend*2; /*da pra diminuir*/
							IloNumExpr rightSide,  leftSide1, leftSide2, leftSide3, leftSide4;


							switch (k) {
							case 0: /* E (e2(route edge) ist east of e1 (path edge)) : 0 : x*/

								/*
								 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
								 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
								 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
								 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
								 * ∀(e1,e2) no incident
								 */
								rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
								leftSide1 = cplex.diff(x[edge1UIndex], x[edge2UIndex]);
								leftSide2 = cplex.diff(x[edge1UIndex], x[edge2VIndex]);
								leftSide3 = cplex.diff(x[edge1VIndex], x[edge2UIndex] );
								leftSide4 = cplex.diff(x[edge1VIndex], x[edge2VIndex]);

								cplex.addLe(leftSide1, rightSide);
								cplex.addLe(leftSide2, rightSide);
								cplex.addLe(leftSide3, rightSide);
								cplex.addLe(leftSide4, rightSide);

								break;
							case 1: /* e2 (route edge) is NE (SE because y is inverted) of e1: 45 (315) : z1 */

								/*Can I use 
								 * route.get(j).getX() + route.get(j).getY() == (route.get(j).getX() / Math.sin(Math.toRadians(45)));
								 */

								rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
								leftSide1 = cplex.diff(z1[edge1UIndex], z1[edge2UIndex]);
								leftSide2 = cplex.diff(z1[edge1UIndex], z1[edge2VIndex]);
								leftSide3 = cplex.diff(z1[edge1VIndex], z1[edge2UIndex]);
								leftSide4 = cplex.diff(z1[edge1VIndex], z1[edge2VIndex]);

								cplex.addLe(leftSide1, rightSide);
								cplex.addLe(leftSide2, rightSide);
								cplex.addLe(leftSide3, rightSide);
								cplex.addLe(leftSide4, rightSide);


								break;
							case 2: /* e2 (route edge) is N (S because y is inverted) of e1: 90 (270) : y */
								rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

								leftSide1 = cplex.diff(y[edge1UIndex], y[edge2UIndex]);
								leftSide2 = cplex.diff(y[edge1UIndex], y[edge2VIndex]);
								leftSide3 = cplex.diff(y[edge1VIndex], y[edge2UIndex]);
								leftSide4 = cplex.diff(y[edge1VIndex], y[edge2VIndex]);

								cplex.addLe(leftSide1, rightSide);
								cplex.addLe(leftSide2, rightSide);
								cplex.addLe(leftSide3, rightSide);
								cplex.addLe(leftSide4, rightSide);

								break;	
							case 3: /* e2 (route edge) is NW (SW because y is inverted) of e1: 135 (225) : -z2 */

								rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

								leftSide1 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
								leftSide2 = cplex.diff( z2[edge2VIndex],  z2[edge1UIndex] );
								leftSide3 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
								leftSide4 = cplex.diff( z2[edge2VIndex],  z2[edge1VIndex] );

								cplex.addLe(leftSide1, rightSide);
								cplex.addLe(leftSide2, rightSide);
								cplex.addLe(leftSide3, rightSide);
								cplex.addLe(leftSide4, rightSide);

								break;
							case 4: /*  e2 (route edge) is W  of e1:  180:  -x */

								//						/*
								//						 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
								//						 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
								//						 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
								//						 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
								//						 * ∀(e1,e2) no incident
								//						 */

								rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

								leftSide1 = cplex.diff(x[edge2UIndex] , x[edge1UIndex] );
								leftSide2 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );
								leftSide3 = cplex.diff(x[edge2UIndex] , x[edge1VIndex] );
								leftSide4 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );

								cplex.addLe(leftSide1, rightSide);
								cplex.addLe(leftSide2, rightSide);
								cplex.addLe(leftSide3, rightSide);
								cplex.addLe(leftSide4, rightSide);

								break;
							case 5: /*  e2 (route edge) is SW (NW because y is inverted) of e1: 225 (135): -z1 */


								rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

								leftSide1 = cplex.diff( z1[edge2UIndex] , z1[edge1UIndex]);
								leftSide2 = cplex.diff( z1[edge2VIndex] , z1[edge1UIndex]);
								leftSide3 = cplex.diff( z1[edge2UIndex] , z1[edge1VIndex]);
								leftSide4 = cplex.diff( z1[edge2VIndex] , z1[edge1VIndex]);

								cplex.addLe(leftSide1, rightSide);
								cplex.addLe(leftSide2, rightSide);
								cplex.addLe(leftSide3, rightSide);
								cplex.addLe(leftSide4, rightSide);

								break;
							case 6:/*  e2 (route edge) is S (N because y is inverted) of e1: 270 (90)  : -y */

								rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
								leftSide1 = cplex.diff(  y[edge2UIndex] ,y[edge1UIndex] );
								leftSide2 = cplex.diff(  y[edge2VIndex], y[edge1UIndex]);
								leftSide3 = cplex.diff(  y[edge2UIndex] ,y[edge1VIndex] );
								leftSide4 = cplex.diff(  y[edge2VIndex], y[edge1VIndex]);

								cplex.addLe(leftSide1, rightSide);
								cplex.addLe(leftSide2, rightSide);
								cplex.addLe(leftSide3, rightSide);
								cplex.addLe(leftSide4, rightSide);

								break;
							case 7:/*  e2 (route edge) is SE (NE because y is inverted) of e1: 315 (45)  : z2*/

								rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
								leftSide1 = cplex.diff(z2[edge1UIndex], z2[edge2UIndex] );
								leftSide2 = cplex.diff(z2[edge1UIndex], z2[edge2VIndex] );
								leftSide3 = cplex.diff(z2[edge1VIndex], z2[edge2UIndex] );
								leftSide4 = cplex.diff(z2[edge1VIndex], z2[edge2VIndex] );

								cplex.addLe(leftSide1, rightSide);
								cplex.addLe(leftSide2, rightSide);
								cplex.addLe(leftSide3, rightSide);
								cplex.addLe(leftSide4, rightSide);


								break;
							default:
								break;
							}

						}

				}



			}

			/*********EDGE DISTANCE - EXTRA CROSSING CONSTRAINT - TOPOLOGY*****************/
			
			int e = topoCheckEdgeList.size();
			/*Boolean variable to indicate if path edge is {N,S,E,W,NE,NW,SE,SW} of route edge*/
			IloNumVar[][][] gama2 = new IloNumVar[m][e][];
			
			/*RElation of m edges from the path and r route edges check relative position to 8 orientations ∈{N,S,E,W,NE,NW,SE,SW}*/
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < e; j++)
						gama2[i][j] = cplex.boolVarArray(8);
					
				IloLinearNumExpr[][] constraintEdgeTopology2 = new IloLinearNumExpr[m][e];
				
				
				/*SumOF{ γi(e1, e2) } ≥ 1   
				 * i∈{N,S,E,W,NE,NW,SE,SW} */
				
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < e; j++){
		//				for (int k = 0; k < 8; k++){
		//					if(k!=0)
		//						cplex.addEq(gama[i][j][k], 0);
		//					else
		//						cplex.addEq(gama[i][j][k], 1);
		//				}
						
						
						constraintEdgeTopology2[i][j] = cplex.linearNumExpr();
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][0]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][1]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][2]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][3]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][4]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][5]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][6]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][7]);
						cplex.addGe(constraintEdgeTopology2[i][j], 1.0);
			
					}
						
				if(checkTopology)
					for(ArrayList<Integer> edgePaar: nonPathPaarToCheck) {
	
						    int i = edgePaar.get(0);
							int edge1UIndex = i ;
							int edge1VIndex = i + 1 ;
							
						    int j = edgePaar.get(1);
							StreetNode edge2U = nodeMap.get(topoCheckEdgeList.get(j)[0]);
							StreetNode edge2V = nodeMap.get(topoCheckEdgeList.get(j)[1]);
							
							double xedge2U = edge2U.getxGeom().getX();
							double yedge2U = edge2U.getxGeom().getY();
							double z1edge2U = 0.5*(xedge2U + yedge2U);
							double z2edge2U = 0.5*(xedge2U - yedge2U);
							
							double xedge2V = edge2V.getxGeom().getX();
							double yedge2V = edge2V.getxGeom().getY();
							double z1edge2V = 0.5*(xedge2V + yedge2V);
							double z2edge2V = 0.5*(xedge2V - yedge2V);
							
							
	
						
								for(int k =0; k < 8; k ++){
									/*whats the value of M(max X ou y dist of points) and dist Min*/ 
									double MTopology = maxExtend*3; /*da pra diminuir*/
									IloNumExpr rightSide,  leftSide1, leftSide2, leftSide3, leftSide4;
	
	
									switch (k) {
									case 0: /* E (e2(route edge) ist east of e1 (path edge)) : 0 : x*/
	
										/*
										 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
										 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
										 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										 * ∀(e1,e2) no incident
										 */
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(x[edge1UIndex],xedge2U);
										leftSide2 = cplex.diff(x[edge1UIndex], xedge2V);
										leftSide3 = cplex.diff(x[edge1VIndex],xedge2U );
										leftSide4 = cplex.diff(x[edge1VIndex], xedge2V);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 1: /* e2 (route edge) is NE (SE because y is inverted) of e1: 45 (315) : z1 */
	
										/*Can I use 
										 * route.get(j).getX() + route.get(j).getY() == (route.get(j).getX() / Math.sin(Math.toRadians(45)));
										 */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(z1[edge1UIndex], z1edge2U);
										leftSide2 = cplex.diff(z1[edge1UIndex], z1edge2V);
										leftSide3 = cplex.diff(z1[edge1VIndex], z1edge2U);
										leftSide4 = cplex.diff(z1[edge1VIndex], z1edge2V);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
	
										break;
									case 2: /* e2 (route edge) is N (S because y is inverted) of e1: 90 (270) : y */
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff(y[edge1UIndex], yedge2U);
										leftSide2 = cplex.diff(y[edge1UIndex], yedge2V);
										leftSide3 = cplex.diff(y[edge1VIndex], yedge2U);
										leftSide4 = cplex.diff(y[edge1VIndex], yedge2V);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;	
									case 3: /* e2 (route edge) is NW (SW because y is inverted) of e1: 135 (225) : -z2 */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff( z2edge2U , z2[edge1UIndex] );
										leftSide2 = cplex.diff( z2edge2V,  z2[edge1UIndex] );
										leftSide3 = cplex.diff( z2edge2U , z2[edge1VIndex] );
										leftSide4 = cplex.diff( z2edge2V,  z2[edge1VIndex] );
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 4: /*  e2 (route edge) is W  of e1:  180:  -x */
	
										//						/*
										//						 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
										//						 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										//						 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
										//						 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										//						 * ∀(e1,e2) no incident
										//						 */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff(xedge2U , x[edge1UIndex] );
										leftSide2 = cplex.diff(xedge2V , x[edge1UIndex] );
										leftSide3 = cplex.diff(xedge2U , x[edge1VIndex] );
										leftSide4 = cplex.diff(xedge2V , x[edge1VIndex] );
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 5: /*  e2 (route edge) is SW (NW because y is inverted) of e1: 225 (135): -z1 */
	
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff( z1edge2U , z1[edge1UIndex]);
										leftSide2 = cplex.diff( z1edge2V , z1[edge1UIndex]);
										leftSide3 = cplex.diff( z1edge2U , z1[edge1VIndex]);
										leftSide4 = cplex.diff( z1edge2V , z1[edge1VIndex]);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 6:/*  e2 (route edge) is S (N because y is inverted) of e1: 270 (90)  : -y */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(  yedge2U ,y[edge1UIndex] );
										leftSide2 = cplex.diff(  yedge2V, y[edge1UIndex]);
										leftSide3 = cplex.diff(  yedge2U ,y[edge1VIndex] );
										leftSide4 = cplex.diff(  yedge2V, y[edge1VIndex]);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 7:/*  e2 (route edge) is SE (NE because y is inverted) of e1: 315 (45)  : z2*/
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(z2[edge1UIndex], z2edge2U );
										leftSide2 = cplex.diff(z2[edge1UIndex], z2edge2V );
										leftSide3 = cplex.diff(z2[edge1VIndex], z2edge2U );
										leftSide4 = cplex.diff(z2[edge1VIndex], z2edge2V );
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
	
										break;
									default:
										break;
									}
	
								
	
						}
				
				}
			
			
			
			/*********BEND MINIMIZATION*****************/

			
			
			
			/*Boolean variable to make condition of bends angle <-5, >-4 e <4, ou >5 measure bends angles (-7 to 7)*/
			/***∆dir(u, v,w) = dir(u, v) − dir(v,w)***/
			/***∆dir(u, v,w) ranges from −7 to 7****/
			
			IloNumVar[][] delta = new IloNumVar[b][];
			
			for (int i = 0; i < b; i++) {
				delta[i] = cplex.boolVarArray(3);
				
			}
			IloLinearNumExpr[] constraintBenMinimizationBooleans = new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				constraintBenMinimizationBooleans[i] = cplex.linearNumExpr();
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][0]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][1]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][2]);
				cplex.addEq(constraintBenMinimizationBooleans[i], 2.0);
			}
			double MBend = 15; /*diference between 2 dir */
			IloNumVar[] bendDir = cplex.intVarArray(b, -7, 7);
			/* bendDir[1] = dir[0] - dir[1]*/

			for (int i = 0; i < b; i++) {
				cplex.addEq(bendDir[i], cplex.diff(dir[i][0], dir[i + 1][0]));
			}
			
			for (int i = 0; i < b; i++) {
				cplex.addLe(bendDir[i], cplex.diff(cplex.prod(MBend, delta[i][0]), 5));
				cplex.addGe(bendDir[i], cplex.diff(5 , cplex.prod(MBend, delta[i][1])));
				cplex.addLe(bendDir[i], cplex.sum(4 , cplex.prod(MBend, delta[i][2])));
				cplex.addGe(bendDir[i], cplex.diff(-4 , cplex.prod(MBend, delta[i][2])));
				
			}
			
			IloNumVar[] bendCost = cplex.intVarArray(b, 0, 3);
			IloLinearNumExpr[] bendCostConstraint =  new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				bendCostConstraint[i] = cplex.linearNumExpr();
				bendCostConstraint[i].addTerm(1, bendDir[i]);
				bendCostConstraint[i].addTerm(-8, delta[i][0]);
				bendCostConstraint[i].addTerm(8, delta[i][1]);
				cplex.addGe(bendCostConstraint[i], cplex.prod(-1, bendCost[i]) );
				cplex.addLe(bendCostConstraint[i], bendCost[i] );
				

				
			}

			IloNumExpr sumBendCost =  cplex.sum(bendCost);

			
			
			/*********Distance Minization Anchors*****************/
			ArrayList<Integer> distNodeIndex = new ArrayList<Integer>();
			for (int i = 0; i < n; i++) {
				int pathIndex = pathRelevantPointsIndex.get(i);
				if( 	//path.getNodeList().get(pathIndex).getDegree() != 2 ||
						path.getNodeList().get(pathIndex).isTopoAnchor()
						) 
					distNodeIndex.add(i);				
			}
			int ndA = distNodeIndex.size();
			IloNumVar[] dXA  = cplex.numVarArray(ndA, 0, Double.MAX_VALUE);
			IloNumVar[] dYA  = cplex.numVarArray(ndA, 0, Double.MAX_VALUE);
			for (int i = 0; i < ndA; i++) {
				int pathPtIndex = distNodeIndex.get(i);
				
				IloNumExpr xLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getX(), x[pathPtIndex]);
				IloNumExpr yLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getY(), y[pathPtIndex]);
				
				cplex.addLe(xLeftSide,dXA[i]);
				cplex.addGe(xLeftSide, cplex.prod(-1, dXA[i]));
				
				
				cplex.addLe(yLeftSide, dYA[i]);
				cplex.addGe(yLeftSide, cplex.prod(-1, dYA[i]));

				
			}
			IloNumExpr sumXDistA =  cplex.sum(dXA);
			IloNumExpr sumYDistA =  cplex.sum(dYA);
			
			
			
			/*********Distance Minization *****************/

//			ArrayList<Integer> distNodeIndex = new ArrayList<Integer>();
//			for (int i = 0; i < n; i++) {
//				int pathIndex = pathRelevantPointsIndex.get(i);
////				if( 	path.getNodeList().get(pathIndex).getDegree() != 2 ||
////						path.getNodeList().get(pathIndex).isTopoAnchor()
////						) 
//					distNodeIndex.add(i);				
//			}
//			int nd = distNodeIndex.size();
//			IloNumVar[] dX  = cplex.numVarArray(nd, 0, Double.MAX_VALUE);
//			IloNumVar[] dY  = cplex.numVarArray(nd, 0, Double.MAX_VALUE);
//			double factor = 1;
//			for (int i = 0; i < nd; i++) {
//				int pathPtIndex = distNodeIndex.get(i);
//				
//				IloNumExpr xLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getX(), x[pathPtIndex]);
//				IloNumExpr yLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getY(), y[pathPtIndex]);
//				
//				cplex.addLe(xLeftSide,cplex.prod(factor, dX[i]));
//				cplex.addGe(xLeftSide, cplex.prod(-1*factor, dX[i]));
//				
//				
//				cplex.addLe(yLeftSide, cplex.prod(factor,dY[i]));
//				cplex.addGe(yLeftSide, cplex.prod(-1*factor, dY[i]));
//
//				
//			}
//			IloNumExpr sumXDist =  cplex.sum(dX);
//			IloNumExpr sumYDist =  cplex.sum(dY);
			
			
//			/*distance reduction*/
			
			IloNumVar[] dX  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			IloNumVar[] dY  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			//IloNumVar[] dMax  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			
			for (int i = 0; i < n; i++) {

					
				IloNumExpr xLeftSide = cplex.diff(pathRelevantPointsList.get(i).getX(), x[i]);
				IloNumExpr yLeftSide = cplex.diff(pathRelevantPointsList.get(i).getY(), y[i]);

				cplex.addLe(xLeftSide,dX[i]);
				cplex.addGe(xLeftSide, cplex.prod(-1, dX[i]));


				cplex.addLe(yLeftSide, dY[i]);
				cplex.addGe(yLeftSide, cplex.prod(-1, dY[i]));
				
			}
//			
			IloNumExpr sumXDist =  cplex.sum(dX);
			IloNumExpr sumYDist =  cplex.sum(dY);
			

			
			
			/*************Proportion diference minimization***********/

			/*** lambda: binary variable to determine if (xi - xi+1) is > 0 or < 0**
			 * lambda1 + lambda2 =1 -> if lambda1 = 1 - > lambda2 = 0 ->  (xi - xi+1) >= 0
			 * 							else lambda1 = 0 -> lambda2 = 1 -> (xi - xi+1) < 0
			 * 
			 * we want dx = |xi - xi+1| -> dx >= |xi - xi+1| & dx <= |xi - xi+1|
			 * if (xi - xi+1) >= 0 -> dx >= xi - xi+1 & dx <= xi - xi+1
			 * if (xi - xi+1) < 0  -> dx >= -(xi - xi+1) & dx <= -(xi - xi+1) 
			 * 
			 * so
			 * dx <= (xi - xi+1) + M(1-lambda1) eq1
			 * dx >= (xi - xi+1) - M(1-lambda1) eq2
			 * 
			 * or
			 * dx >= -(xi - xi+1) - M(1-lambda2) eq3
			 * dx <= -(xi - xi+1) + M(1-lambda2) eq4
			 * 
			 * if lambda1 = 1 & lambda2 = 0  => constraint dx = (xi - xi+1) & e3 and eq4 are trivialy fulfiled(makes no diference) 
			 * else lambda1 = 0 & lambda2 = 1  => constraint dx = -(xi - xi+1) & e1 and eq2 are trivialy fulfiled(makes no diference) 
			 * 
			 * 
			 * 
			 * */
			
			int mP = m; /*route edges number: respect order on edge list*/ 
			
			IloNumVar[][] xLambda = new IloNumVar[mP][];
			IloNumVar[][] yLambda = new IloNumVar[mP][];
			
			for (int i = 0; i < mP; i++) {
				xLambda[i] = cplex.boolVarArray(2);
				yLambda[i] = cplex.boolVarArray(2);
				xLambda[i][0].setName("proporMinxLambda0");
				xLambda[i][1].setName("proporMinxLambda1");
				yLambda[i][0].setName("proporMinyLambda0");
				yLambda[i][1].setName("proporMinyLambda1");
				
			}
			IloLinearNumExpr[] xLambdaConstraint = new IloLinearNumExpr[mP];
			IloLinearNumExpr[] yLambdaConstraint = new IloLinearNumExpr[mP];
			for (int i = 0; i < mP; i++) {
				xLambdaConstraint[i] = cplex.linearNumExpr();
				xLambdaConstraint[i].addTerm(1.0,xLambda[i][0]);
				xLambdaConstraint[i].addTerm(1.0,xLambda[i][1]);

				cplex.addEq(xLambdaConstraint[i], 1.0);
				
				
				yLambdaConstraint[i] = cplex.linearNumExpr();
				yLambdaConstraint[i].addTerm(1.0,yLambda[i][0]);
				yLambdaConstraint[i].addTerm(1.0,yLambda[i][1]);

				cplex.addEq(yLambdaConstraint[i], 1.0);
				
			}
			
			 
			/*  maxAdjD distance between two adjacent vertices vertices! it was MaxD*/
			MAdjVertice = pathLength*1.5;
			IloNumVar[] absoluteDiffX  = cplex.numVarArray(mP, 0, 4* maxAdjDist);
			IloNumVar[] absoluteDiffY  = cplex.numVarArray(mP, 0, 4* maxAdjDist);
			
			for (int i = 0; i < mP; i++) {
				
				int v1Index = i;
				int v2Index = i+1;
				
				/***dx = |xi - xi+1|***/
				IloNumExpr diffX = cplex.diff(x[v1Index], x[v2Index]);
				
				cplex.addLe(absoluteDiffX[i], cplex.sum(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffX[i], cplex.diff(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffX[i], cplex.diff(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffX[i], cplex.sum(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq4.
				
				/***dy = |xi - yi+1|***/
				IloNumExpr diffY = cplex.diff(y[v1Index], y[v2Index]);
				
				cplex.addLe(absoluteDiffY[i], cplex.sum(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffY[i], cplex.diff(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffY[i], cplex.diff(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffY[i], cplex.sum(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq4.

				
				
			}
			/***WE have absDiffX and absDiffY
			 * We wanto to minimize:
			 * Abs((absDiffX[i] + absDiffY[i])L - lengthEdge[i]/L)
			 * 
			 * ****/
			/**
			 * If you are minimizing an increasing function of |x| (or maximizing a decreasing function, of course), you can always have the aboslute value of any quantity x in a lp as a variable absx such as:
				absx >= x -> x<=absx
				absx >= -x -> x >= -absx
				{leftside = x
				dX = absx}
				It works because the value absx will 'tend' to its lower bound, so it will either reach x or -x.
				On the other hand, if you are minimizing a decreasing function of |x|, your problem is not convex and cannot be modelled as a lp.
			 */
			ArrayList<Double> edgeProportion = new ArrayList<Double>();
			for (int i = 0; i < mP; i++) {
	
				//edgeProportion.add(pt1.distance(pt2)/routeLenght);
				edgeProportion.add(proportion*pathRelevantPointsList.get(i).distance(pathRelevantPointsList.get(i+1)));
			}
			IloNumVar[] absolutEdgeProportionDifference  = cplex.numVarArray(mP, 0, Double.MAX_VALUE);
			IloNumExpr[] finalEdgeProportion = new IloNumExpr[mP];
			for (int i = 0; i < mP; i++) {
				finalEdgeProportion[i] = cplex.prod(1,cplex.sum(absoluteDiffX[i],absoluteDiffY[i]));
				/*can I use cplex.Equal direct? yes absX and absY is always positiv*/
				cplex.addLe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), absolutEdgeProportionDifference[i]);
				cplex.addGe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), cplex.prod(-1, absolutEdgeProportionDifference[i]));
			}
			
			IloNumExpr sumAbsolutEdgeProportionDifference =  cplex.sum(absolutEdgeProportionDifference);

			
			/****Edges orientation difference minimization*********/
			IloNumVar[] dirDiff  = cplex.numVarArray(m, -7, 7);
			for (int i = 0; i < m; i++) {
				cplex.addEq(dirDiff[i], cplex.diff(sec[i][0][1] ,dir[i][0]));
			}
			
			IloNumVar[] dirCost = cplex.boolVarArray(m);
			for (int i = 0; i < m; i++) {

				cplex.addLe(dirDiff[i],cplex.prod(8, dirCost[i]));
				cplex.addGe(dirDiff[i],  cplex.prod(-8, dirCost[i]));
			}

//			IloNumVar[][] zeta = new IloNumVar[m][];
//			
//			for (int i = 0; i < m; i++) {
//				zeta[i] = cplex.boolVarArray(3);
//				zeta[i][0].setName("ZETADirMin0");
//				zeta[i][1].setName("ZETADirMin1");
//				zeta[i][2].setName("ZETADirMin2");
//				
//			}
//			IloLinearNumExpr[] constraintDirDiffMinimizationBooleans = new IloLinearNumExpr[m];
//			for (int i = 0; i < m; i++) {
//				constraintDirDiffMinimizationBooleans[i] = cplex.linearNumExpr();
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][0]);
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][1]);
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][2]);
//				cplex.addEq(constraintDirDiffMinimizationBooleans[i], 2.0);
//			}
//
//			MDir = 8;
//			for (int i = 0; i < m; i++) {
//				cplex.addLe(dirDiff[i], cplex.diff(cplex.prod(MDir, zeta[i][0]), 5));
//				cplex.addGe(dirDiff[i], cplex.diff(5 , cplex.prod(MDir, zeta[i][1])));
//				cplex.addLe(dirDiff[i], cplex.sum(4 , cplex.prod(MDir, zeta[i][2])));
//				cplex.addGe(dirDiff[i], cplex.diff(-4 , cplex.prod(MDir, zeta[i][2])));
//				
//			}
//			IloNumVar[] dirCost = cplex.intVarArray(m, 0, 4);
//			IloLinearNumExpr[] dirCostConstraint =  new IloLinearNumExpr[m];
//			for (int i = 0; i < m; i++) {
//				dirCostConstraint[i] = cplex.linearNumExpr();
//				dirCostConstraint[i].addTerm(1, dirDiff[i]);
//				dirCostConstraint[i].addTerm(-8, zeta[i][0]);
//				dirCostConstraint[i].addTerm(8, zeta[i][1]);
//				cplex.addGe(dirCostConstraint[i], cplex.prod(-1, dirCost[i]) );
//				cplex.addLe(dirCostConstraint[i], dirCost[i] );
//
//				
//			}
			IloNumExpr sumDirCost =  cplex.sum(dirCost);  
			
			
			
			/*********OBJECTIVE*****************/
			
//			IloNumExpr sumBendCostNormal = cplex.prod(0.5/Math.sqrt(b),sumBendCost);
//			IloNumExpr sumBendCostFactor = cplex.prod(bendFactor,sumBendCostNormal);
			
//			double maxExted = octBox.getMaxExtend();
//			IloNumExpr sumDistCostNormal = cplex.prod((20.0)/(n*maxExted), cplex.sum(sumXDist, sumYDist ));			
//			//IloNumExpr sumDistCostNormal = cplex.prod((50.0)/(n*pahtLength), cplex.sum(sumXDist, sumYDist ));			
//			IloNumExpr sumDistCostFactor = cplex.prod(distFactor, sumDistCostNormal);
			
			
//			IloNumExpr sumDistCostNormalA = cplex.prod((10.0)/(ndA), cplex.sum(sumXDistA, sumYDistA ));
//			IloNumExpr sumDistCostFactorA = cplex.prod(distFactor, sumDistCostNormalA);
			
			
//			IloNumExpr sumAbsolutEdgeProportionDifferenceNormal = cplex.prod(10.0/(mP), sumAbsolutEdgeProportionDifference);			
//			IloNumExpr sumEdgeProportionFactor = cplex.prod(proportionFactor, sumAbsolutEdgeProportionDifferenceNormal);
			
//			IloNumExpr sumDirCostNormal = cplex.prod((5.0/m),sumDirCost);
//			IloNumExpr sumDirCostFactor = cplex.prod((dirFactor),sumDirCostNormal);
			
			
		
//			double bendWeight = bendFactor*0.6* Math.pow((pahtLength*100),0.5);
//			double bendWeightNormalized = bendWeight/b;
//			//IloNumExpr sumBendCostNormal = cplex.prod(0.5/Math.sqrt(b),sumBendCost);
//			IloNumExpr sumBendCostFactor = cplex.prod(bendWeightNormalized,sumBendCost);
////			
////			
//			double distWeight = distFactor*200.0;
//			
//			double distWeightNormalized = distWeight/(n);
//			//IloNumExpr sumDistCostNormal = cplex.prod((20.0)/(n*maxExted), cplex.sum(sumXDist, sumYDist ));			
//
//			//IloNumExpr sumDistCostNormal = cplex.prod((50.0)/(n*pahtLength), cplex.sum(sumXDist, sumYDist ));			
//			IloNumExpr sumDistCostFactor = cplex.prod(distWeightNormalized, cplex.sum(sumXDist, sumYDist ));
////			
////			
//			double distWeightA = distFactor*10.0;
//			double distWeightANormalized = distWeightA/ndA;
////			//IloNumExpr sumDistCostNormalA = cplex.prod((10.0)/(ndA), cplex.sum(sumXDistA, sumYDistA ));
//			IloNumExpr sumDistCostFactorA = cplex.prod(distWeightANormalized, cplex.sum(sumXDistA, sumYDistA ));
////			
////			
//			double propWeight = proportionFactor*10.0;
//			double propNormalWeight = propWeight/mP;
////			//IloNumExpr sumAbsolutEdgeProportionDifferenceNormal = cplex.prod(10.0/(mP), sumAbsolutEdgeProportionDifference);			
//			IloNumExpr sumEdgeProportionFactor = cplex.prod(propNormalWeight, sumAbsolutEdgeProportionDifference);
////			
//			double dirWeight = dirFactor*5.0;
//			double dirWeightNormalized = dirWeight/m;
////			//IloNumExpr sumDirCostNormal = cplex.prod((5.0/m),sumDirCost);
//			IloNumExpr sumDirCostFactor = cplex.prod(dirWeightNormalized,sumDirCost);
			
			
			
			
			double bendWeight = bendFactor*0.5;
			double bendWeightNormalized = bendWeight/b;
			//IloNumExpr sumBendCostNormal = cplex.prod(0.5/Math.sqrt(b),sumBendCost);
			IloNumExpr sumBendCostFactor = cplex.prod(bendWeightNormalized,sumBendCost);
//			
//			
			double distWeight = distFactor*20.0;
			if(proportion < 0.9 ||  proportion > 1.1)
				distWeight = 0;
			
			
			double distWeightNormalized = distWeight/(n);
			//double distWeightNormalized = distWeight/(n*octBoxMaxExted);
			//IloNumExpr sumDistCostNormal = cplex.prod((20.0)/(n*maxExted), cplex.sum(sumXDist, sumYDist ));			
			//IloNumExpr sumDistCostNormal = cplex.prod((50.0)/(n*pahtLength), cplex.sum(sumXDist, sumYDist ));			
			IloNumExpr sumDistCostFactor = cplex.prod(distWeightNormalized, cplex.sum(sumXDist, sumYDist ));
//			
//			
			double distWeightA = distFactor*10.0;
			//double distWeightA = distFactor/2;
			double distWeightANormalized = distWeightA/ndA;
//			//IloNumExpr sumDistCostNormalA = cplex.prod((10.0)/(ndA), cplex.sum(sumXDistA, sumYDistA ));
			IloNumExpr sumDistCostFactorA = cplex.prod(distWeightANormalized, cplex.sum(sumXDistA, sumYDistA ));
//			
//			
			double propWeight = proportionFactor*10.0;
			double propNormalWeight = propWeight/mP;
//			//IloNumExpr sumAbsolutEdgeProportionDifferenceNormal = cplex.prod(10.0/(mP), sumAbsolutEdgeProportionDifference);			
			IloNumExpr sumEdgeProportionFactor = cplex.prod(propNormalWeight, sumAbsolutEdgeProportionDifference);
//			
			double dirWeight = dirFactor*1.1;
			double dirWeightNormalized = dirWeight/m;
//			//IloNumExpr sumDirCostNormal = cplex.prod((5.0/m),sumDirCost);
			IloNumExpr sumDirCostFactor = cplex.prod(dirWeightNormalized,sumDirCost);
			
			IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumDistCostFactorA,  sumBendCostFactor,  sumEdgeProportionFactor, sumDirCostFactor ) ;
			//IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumBendCostFactor, sumDirCostFactor) ;

			//IloNumExpr objective = sumBendCostFactor ;
			//IloNumExpr objective = cplex.sum(sumXDist, sumYDist ) ;

			

			
			cplex.addMinimize(objective);
			if(executionTimeLimit > 0 ){
				double limitTimeInSeconds = ((double)(executionTimeLimit))/1000;
				cplex.setParam(IloCplex.DoubleParam.TiLim, limitTimeInSeconds);
			}
			else if(executionTimeLimit <0 ){
				//cplex.setParam(IloCplex.Param.MIP.Limits.Solutions, 1);
				cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, (-(double)executionTimeLimit)/100.0);
				pathResultReport.setGap(-((double)executionTimeLimit)/100.0);
				
				cplex.setParam(IloCplex.DoubleParam.TiLim, 120);
			}
			

			/*control display information*/
			//cplex.setParam(IloCplex.Param.MIP.Display, 1);
			boolean topologyViolation = false;
			if(cplex.solve()){
				
				/*Check if there is self extra crossings*/
				for (int i = 0; i < m; i++) {					
					for(int j = i; j < m -1; j++){
						

						int edge1UIndex = i ;
						int edge1VIndex = i + 1 ;

						int edge2UIndex = j ;
						int edge2VIndex = j + 1 ;
						
						
						if(i!=j && edge1UIndex != edge2UIndex && edge1UIndex != edge2VIndex && edge1VIndex != edge2UIndex && edge1VIndex != edge2VIndex ) {
							double e1x1 = cplex.getValue(x[edge1UIndex]);
							double e1y1 = cplex.getValue(y[edge1UIndex]);
							double e1x2 = cplex.getValue(x[edge1VIndex]);
							double e1y2 = cplex.getValue(y[edge1VIndex]);
							
							double e2x1 = cplex.getValue(x[edge2UIndex]);
							double e2y1 = cplex.getValue(y[edge2UIndex]);
							double e2x2 = cplex.getValue(x[edge2VIndex]);
							double e2y2 = cplex.getValue(y[edge2VIndex]);
							
							if(GeometricOperation.intersects(e1x1, e1y1, e1x2, e1y2,
									e2x1, e2y1, e2x2, e2y2)) {
								
								topologyViolation = true;
								System.out.println("Edges violates topology: " + i + " " + j );
								
								ArrayList<Integer> edgeTopoViolation = new ArrayList<Integer>();
								edgeTopoViolation.add(i);
								edgeTopoViolation.add(j);
								selfEdgePaarToCheck.add(edgeTopoViolation);
								
							}
							
							
						}
						
					}
				}
				
				/*Check if there is  extra crossings with previous schematized edges*/
				for (int i = 0; i < m; i++) {					
					for(int j = 0; j < topoCheckEdgeList.size(); j ++){
						

						int edge1UIndex = i ;
						int edge1VIndex = i + 1 ;
						
						double e1x1 = cplex.getValue(x[edge1UIndex]);
						double e1y1 = cplex.getValue(y[edge1UIndex]);
						double e1x2 = cplex.getValue(x[edge1VIndex]);
						double e1y2 = cplex.getValue(y[edge1VIndex]);
						

						StreetNode edge2U = nodeMap.get(topoCheckEdgeList.get(j)[0]);
						StreetNode edge2V = nodeMap.get(topoCheckEdgeList.get(j)[1]);

						double e2x1 = edge2U.getxGeom().getX();
						double e2y1 = edge2U.getxGeom().getY();
						double e2x2 = edge2V.getxGeom().getX();
						double e2y2 = edge2V.getxGeom().getY();

						if(GeometricOperation.intersects(e1x1, e1y1, e1x2, e1y2,
								e2x1, e2y1, e2x2, e2y2)) {

							topologyViolation = true;
							System.out.println("Topology Violation with previous schematized edges:  path edge index" + i + "  topocheck edge index" + j );
							
							ArrayList<Integer> edgeTopoViolation = new ArrayList<Integer>();
							edgeTopoViolation.add(i);
							edgeTopoViolation.add(j);
							nonPathPaarToCheck.add(edgeTopoViolation);

						}




					}
				}
				
				
				
				System.out.println("obj =  " +cplex.getObjValue());
				for (int i = 0; i < n; i++) {
					Point2D.Double pt = new Point2D.Double(cplex.getValue(x[i]),cplex.getValue(y[i])); 
					mipLineString.add(pt);
				}	
				ArrayList<Point2D> finalPoints = new ArrayList<Point2D>();
				/****USE IT TO REFILL PATH IF YOU NEED TO SIMPLIFY****/
				
				finalPoints.add(mipLineString.get(0));
				for(int i = 1; i < pathRelevantPointsIndex.size()  ; i++){
					int index = pathRelevantPointsIndex.get(i);
					int lastIndexInFinalPoints = finalPoints.size() - 1;
					/*adicional o proximo ponto relevante entre ele e o anterior na linha reta com os ponts complementares*/
					finalPoints.add( mipLineString.get(i) );
					finalPoints = GeometricOperation.fillLine( finalPoints , transPointList,  index - lastIndexInFinalPoints   , lastIndexInFinalPoints );
				}
				
//				ArrayList<Point2D> routePathNormalized =  route.getRoutePath().asJava2DList(1);
//				finalPoints.add(mipLineString.get(0));
//				for(int i = 1; i < route.getRelevantPointIndex().size() ; i++){
//					int index = route.getRelevantPointIndex().get(i);
//					int lastIndexInFinalPoints = finalPoints.size() - 1;
//					/*adicional o proximo ponto relevante entre ele e o anterior na linha reta com os ponts complementares*/
//					finalPoints.add( mipLineString.get(i) );
//					finalPoints = GeometricOperation.fillLine( finalPoints , routePathNormalized,  index - lastIndexInFinalPoints   , lastIndexInFinalPoints );
//				}
//				route.getRoutePath().updatePathXNodes( finalPoints);
				
				
				//System.out.println("Only ADj");
			

				
//				for(int i = 0; i < n ; i++){
//					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
	//
//				}
				
//				for (int i = 0; i < mP; i++){
//					System.out.println("Edge: " + i);
//					System.out.println("AbsDiffX-> =  " + cplex.getValue(absoluteDiffX[i]) + " AbsDiffY-> =  " + cplex.getValue(absoluteDiffY[i]) );
//					System.out.println("True edge proportion: " + edgeProportion.get(i) + "Final edgeProportion: " + cplex.getValue(finalEdgeProportion[i]) + "Absolut Difference: " + cplex.getValue(absolutEdgeProportionDifference[i]));
//				}
//				for (int i = 0; i < b ; i++){
//					//System.out.println("benDir-> =  " + cplex.getValue(bendDir[i-2]) + " bencost-> =  " + cplex.getValue(bendCost[i-2]) );
//					//System.out.println("gama0-> =  " + cplex.getValue(gama[i-2][0]) + " gama1-> =  " + cplex.getValue(gama[i-2][1])  + " gama2-> =  " + cplex.getValue(gama[i-2][2]));
//					
//					//System.out.println("origTurnDir->= "+ origTurnDir[i -1] + " benDir-> =  " + cplex.getValue(bendDir[i-1]) + " bencost-> =  " + cplex.getValue(bendCost[i-1]) );
//				}
//				
//				for (int i = 0; i <n ; i++){
//					System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//				}
//				for (int i = 0; i <nd ; i++){
//					System.out.println("dx"+ i + " =  " + cplex.getValue(dX[i]) + " dy"+ i + " =  " + cplex.getValue(dY[i]) );
//				}
				
//				for (int i = 0; i < m ; i++){
//					//System.out.println("benDir-> =  " + cplex.getValue(bendDir[i-2]) + " bencost-> =  " + cplex.getValue(bendCost[i-2]) );
//					System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dir-> =  " + cplex.getValue(dir[i][0]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]));
//					
//				}
//				for (int i = 0; i < nA ; i++){
//					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//				}

//				for (int i = nR -1; i < m ; i++){
//					
//					System.out.println("x"+ i+1 + " =  " + cplex.getValue(x[i+1]) + " y"+ i+1 + " =  " + cplex.getValue(y[i+1]) + " z1"+ i+1 + " =  " + cplex.getValue(z1[i+1]) + " z2"+ i+1 + " =  " + cplex.getValue(z2[i+1]));
//				//	System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
	//
//					System.out.println("Route Point: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom() );
//					System.out.println("Adj Point: " +streetNodeMap.get(edgeList.get(i)[1]).getxGeom() );
//					System.out.println("Dist: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom().distance(streetNodeMap.get(edgeList.get(i)[1]).getxGeom()) );
//					System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dir-> =  " + cplex.getValue(dir[i][0]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]));
//					System.out.println();
//				}
				
				
				
//				numInterPoints = intersectionNodeIdList.size();							
//				for (int i = 0; i < numInterPoints; i++) {
//					
	//
//					int vId = intersectionNodeIdList.get(i);
////					System.out.println();
////					System.out.println("Route relevant point n  " + routeNodeIdList.indexOf(vId) + " of id "+vId+"  is intersection and its degree is " + circularOrderList.get(vId).size() );
////					System.out.println("Interpoint"+i+" Intersection Id: " + vId);
//					
//					for(int j = 0; j < beta[i].length ; j ++ ){
//						int indexDirVU1 = 0, indexDirVU2 = 0;
////						System.out.println("--Edge " + j + " and " + (j+1));
//						int u1Id, u2Id;
//						u1Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j).getId();
////						System.out.println("u1ID="+u1Id); 
//						/*se for vertive una orden circular seleciona u2 como o primeiro da lista*/
//						if(j < (beta[i].length -1))
//							u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j + 1).getId();
//						else 
//							u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(0).getId();
////						System.out.println("u2ID="+u2Id);
//						/***identify the edges (v, u1) and (v, u2)**/
//						boolean foundEdgeToU1 = false, foundEdgeToU2 = false;
//						int edgeToU1IsInverted = 0, edgeToU2IsInverted = 0;
//						
//						for(int k = 0; k < edgeList.size(); k++){
	//
//							/*intersectionNOdeid contain only route nodes, the adj node are always edge[1]. Otherwise is necessary to check the inverse*/
//							if(edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u1Id) {
//								indexDirVU1 = k;
//								foundEdgeToU1 = true;
//								edgeToU1IsInverted = 0;
//							}
//							else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u1Id){
//								indexDirVU1 = k;
//								foundEdgeToU1 = true;
//								edgeToU1IsInverted = 1;
//							}
//							if( edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u2Id ){
//								indexDirVU2 = k;
//								foundEdgeToU2 = true;
//								edgeToU2IsInverted = 0;
//							}
//							else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u2Id){
//								indexDirVU2 = k;
//								foundEdgeToU2 = true;
//								edgeToU2IsInverted = 1;
//							}
//							if(foundEdgeToU1 && foundEdgeToU2)
//								break;
//						}
//						
////						System.out.println("EdgeIndex to u1ID= "+ indexDirVU1 + " dir: " + cplex.getValue(dir[indexDirVU1][edgeToU1IsInverted]) + " found: " + foundEdgeToU1  + " is inverted: " + edgeToU1IsInverted); 
////						System.out.println("EdgeIndex to u2ID= "+ indexDirVU2 + " dir: " + cplex.getValue(dir[indexDirVU2][edgeToU2IsInverted]) + " found: " + foundEdgeToU2 + " is inverted: " + edgeToU2IsInverted); 
//				
	//
//				
//					}
////					System.out.println();
//				}
				
				
				//System.out.println("distCost/Normal =  " + (cplex.getValue(sumXDist) + cplex.getValue(sumYDist)) + " / " +cplex.getValue(sumDistCostNormal) + " sumBendCost/Normal = " + cplex.getValue(sumBendCost)+ " / " +cplex.getValue(sumBendCostNormal) + " sumDirCost/Normal = " + cplex.getValue(sumDirCost) + " / " + cplex.getValue(sumDirCostNormal) + " sumEdgeProportionCost/Normal = " + cplex.getValue(sumAbsolutEdgeProportionDifference)+ " / " + cplex.getValue(sumAbsolutEdgeProportionDifferenceNormal) );
				System.out.println("distFactor =  " + distFactor + " bendFactor = " + bendFactor + " dirFactor = " + dirFactor + " proporFactor = " + proportionFactor );
				System.out.println("distCostNormalFactor =  " + cplex.getValue(sumDistCostFactor) + " sumBendCostFactor = " + cplex.getValue(sumBendCostFactor) + " sumDirCostFactor = " + cplex.getValue(sumDirCostFactor) + " sumProportionFactor = " + cplex.getValue(sumEdgeProportionFactor) );

				pathResultReport.setBendSC(new SoftConstraintValues(bendWeight, bendWeightNormalized, cplex.getValue(sumBendCostFactor)));
				pathResultReport.setDistanceWeightSC(new SoftConstraintValues(distWeight, distWeightNormalized, cplex.getValue(sumDistCostFactor)));
				pathResultReport.setDistanceTopoWeightSC(new SoftConstraintValues(distWeightA, distWeightANormalized, cplex.getValue(sumDistCostFactorA)));
				pathResultReport.setOrientationSC(new SoftConstraintValues(dirWeight, dirWeightNormalized, cplex.getValue(sumDirCostFactor)));
				pathResultReport.setProportionWeightSC(new SoftConstraintValues(propWeight, propNormalWeight, cplex.getValue(sumEdgeProportionFactor)));
				pathResultReport.setObjectiveFunctionValue(cplex.getObjValue());
				System.out.println("Final = " + cplex.getObjValue());
				
				cplex.end();
				path.updatePathXNodes2( finalPoints );
				end = System.currentTimeMillis();
				System.out.println("Path Scehmatization- Nodes: " + n+ " Execution Time: "   + (end - start) );
				pathResultReport.setExecutionTime((end - start));
				return topologyViolation;
				
			}
			else{
				
				throw new Exception("Cannot solve MIP Model on time");
				
			}
			
			
			
			
			
		}


		
		public static ArrayList<Point2D> landmarkAbstractOptimizer3DirTopoRelevant1( Path path, ArrayList<Path> pathList, ArrayList<Point2D> transPointList, Map<Integer, StreetNode> nodeMap, StreetNetworkTopological streetNetworkTopological, double bendFactor,  double distFactor, double proportionFactor, double dirFactor, Boolean checkSelfTopology, Boolean checkTopology,double minNonAdjEdgeDist, int executionTimeLimit, PathReport pathResultReport) throws IloException, Exception   {
			//System.out.println("OPTIMIZER Network PATH - > ACTION!!!!!");	

			long start, end;
			start = System.currentTimeMillis(); 
			
			IloCplex cplex = new IloCplex();
			
			
			ArrayList<Point2D> mipLineString =  new ArrayList<Point2D>();
			ArrayList<Point2D> pathRelevantPointsList = new ArrayList<Point2D>();
			ArrayList<Integer> pathRelevantPointsIndex = new ArrayList<Integer>();

			for(int i = 0; i < transPointList.size(); i++) {
				if(path.getNodeList().get(i).isRelevantRouteNode()) {
					pathRelevantPointsList.add(transPointList.get(i));
					pathRelevantPointsIndex.add(i);
				}
			}
//			boolean isClose = true;
//			if(isClose)
//				transPointList.add(transPointList.get(0));
			
			
			
			//System.out.println(lineString);
			int n = pathRelevantPointsList.size();
			int m = n -1; /*number of edges in a paths*/
			int b = m - 1; /*number of bends in a path*/
			
		
			
			
			
			
			
			
			
			
			
			double extendLimit = 0.05;
			ArrayList<Integer[]> topoCheckEdgeList = new ArrayList<Integer[]>();
			ArrayList<Point2D> pointsToBoundingBox = new ArrayList<Point2D>(transPointList);
			Polygon boundingPolygon = (Polygon)GeoConvertionsOperations.Java2DToJTSGeometry( new OctlinearBoundingBox(transPointList, 2*minNonAdjEdgeDist, extendLimit +0.1, 0).getBoundingPolygon(), Geometries.POLYGON);


			
			if(checkTopology)
				for(Path p: pathList) {
	//				if(p.isRoute() ||
	//						p.isRouteAdjEdge())
					if(p.isRoute())		
					if(p.isWasSchematized()) {
						PointsPolar polarPoints = new PointsPolar();
						polarPoints  = GeometricOperation.toPolar(p.asJava2DList(2));
						for(int i = 0; i < p.getNodeList().size() - 1; i++) {
							if(!path.getNodeList().contains(p.getNodeList().get(i)) && !path.getNodeList().contains(p.getNodeList().get(i +1))) {
								
								if (boundingPolygon.contains( p.getNodeList().get(i).getxGeom()  ) || boundingPolygon.contains( p.getNodeList().get(i +1).getxGeom() )){
									int indexU = i;
									int indexV = i+1;
									boolean foundBend = false; 
									while(!foundBend ) {
									
										if(indexV < p.getNodeList().size() -1 && !path.getNodeList().contains(p.getNodeList().get(indexV + 1))
												&& Math.abs(polarPoints.getPoints().get(indexV -1).getTheta() - polarPoints.getPoints().get(indexV).getTheta()) < 0.001
												&& boundingPolygon.contains( p.getNodeList().get(indexV + 1).getxGeom() ) )
											indexV++;
										else {
											Integer[] edge = new Integer[2];
											edge[0] = p.getNodeList().get(indexU).getId();
											edge[1] = p.getNodeList().get(indexV).getId();
											pointsToBoundingBox.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(indexU).getxGeom()));
											pointsToBoundingBox.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p.getNodeList().get(indexV).getxGeom()));
											topoCheckEdgeList.add(edge);
											foundBend = true;
											i = indexV -1;
										}
									}
										
									
									
									
								}
							}
													
						}
					}
				}
			
			
			OctlinearBoundingBox octBox = new OctlinearBoundingBox(pointsToBoundingBox, 2*minNonAdjEdgeDist, extendLimit +0.1, 2);
			double maxAdjDist = octBox.getMaxAdjDist();
			
			
	//
//	        System.out.println("maxD: " + maxD);
//	        System.out.println("N: " + n);
			
			double maxExtend = GeometricOperation.diagonalOf(transPointList);
			double octBoxMaxExted = octBox.getMaxExtend();
			double pathLength = GeometricOperation.length(transPointList);
			double L = pathLength/(n*100); /* Minimal distance between two vertices*/
			System.out.println("Minimal distance between two vertices: " + L);
			System.out.println("Mininal distance from route " + minNonAdjEdgeDist);
			System.out.println("Linestring Legth " + pathLength);
			System.out.println("Max Extend " + maxExtend);
			System.out.println("OctBox Max Extend " + octBoxMaxExted);

			//PathReport report = new PathReport();
			pathResultReport.setNumberOfNodes(n);
			pathResultReport.setNumberOfEdges(m);
			pathResultReport.setPathLenght(pathLength);
			/*********COORDINATES CONSTRAINT*****************/
	       

	        
			IloNumVar[] x  = cplex.numVarArray(n, octBox.getMinX(), octBox.getMaxX());
			IloNumVar[] y  = cplex.numVarArray(n,  octBox.getMinY(), octBox.getMaxY());
			IloNumVar[] z1  = cplex.numVarArray(n, octBox.getMinZ1(), octBox.getMaxZ1());
			IloNumVar[] z2  = cplex.numVarArray(n, octBox.getMinZ2(), octBox.getMaxZ2());
			
			
//			IloNumVar[] x  = cplex.numVarArray(n, -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] y  = cplex.numVarArray(n,  -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] z1  = cplex.numVarArray(n,  -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] z2  = cplex.numVarArray(n, -Double.MAX_VALUE, Double.MAX_VALUE);
			
			IloNumExpr[] z1Constraint = new IloNumExpr[n];
			IloNumExpr[] z2Constraint = new IloNumExpr[n];
			
			for (int i = 0; i < n; i++) {
				/*do we need to multiply by 2?*/
//				z1Constraint[i] = cplex.sum(x[i],y[i]);
//				z2Constraint[i] = cplex.diff(x[i],y[i]);
				
//				/*do we need to multiply by 2?*/
				z1Constraint[i] = cplex.prod(0.5,cplex.sum(x[i],y[i]));
				z2Constraint[i] = cplex.prod(0.5, cplex.diff(x[i],y[i]));
			}
			
			/*Add z constraint to model constranint*/ 
			for (int i = 0; i < n; i++) {
				cplex.addEq(z1[i], z1Constraint[i]);
				cplex.addEq(z2[i], z2Constraint[i]);
							
				
			}
//			/*fix position of first vertex*/
//			cplex.addEq(x[0], lineString.get(0).getX());
//			cplex.addEq(y[0], lineString.get(0).getY());
//			
//			/***fix position of already schematized vertices*/
//			/*I cannot fix position of 2 adjacent edges*/
			
//			for(int i = 0; i < n -1; i++) {
//				
//				
//				if(path.getNodeList().get(i).getxGeom() != null && !path.getNodeList().get(i).isAnchor()) {
//					
//					cplex.addEq(x[i], path.getNodeList().get(i).getxGeom().getX());
//					cplex.addEq(y[i], path.getNodeList().get(i).getxGeom().getY());
//				}
//				
//			}
//////			
////			
			int startEdgeDirection = -1;
			int endEdgeDirection = -1;
			
			if(path.getNodeList().get(0).getId() == path.getNodeList().get(path.getNodeList().size() -1).getId()) {				
				cplex.addEq(x[n-1], x[0]);
				cplex.addEq(y[n-1], y[0]);
			}
			else {
				if(path.getNodeList().get(0).getxGeom() != null ) {				
					cplex.addEq(x[0], path.getNodeList().get(0).getxGeom().getX());
					cplex.addEq(y[0], path.getNodeList().get(0).getxGeom().getY());
					if(path.getNodeList().get(1).getxGeom() != null ) {
					startEdgeDirection = GeometricOperation.sectorOf(
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
									path.getNodeList().get(0).getxGeom().getX(), path.getNodeList().get(0).getxGeom().getY(),
									path.getNodeList().get(1).getxGeom().getX(), path.getNodeList().get(1).getxGeom().getY() 					
									));
					}
				}
				else {
					cplex.addEq(x[0], transPointList.get(0).getX());
					cplex.addEq(y[0], transPointList.get(0).getY());
				}
				if(path.getNodeList().get(path.getNodeList().size() -1).getxGeom() != null ) {				
					cplex.addEq(x[n -1], path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getX());
					cplex.addEq(y[n -1], path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getY());
					if(path.getNodeList().get(path.getNodeList().size() -2).getxGeom() != null ) {
					endEdgeDirection = GeometricOperation.sectorOf(
							GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
									path.getNodeList().get(path.getNodeList().size() -2).getxGeom().getX(), path.getNodeList().get(path.getNodeList().size() -2).getxGeom().getY(),
									path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getX(), path.getNodeList().get(path.getNodeList().size() -1).getxGeom().getY() 					
									));
					}
				}
				else {
					cplex.addEq(x[n -1], transPointList.get(transPointList.size() -1).getX());
					cplex.addEq(y[n -1], transPointList.get(transPointList.size() -1).getY());

				}
			}

			
//			
			
			/*********OCTALINEARITY CONSTRAINT*****************/
			
			/*sec[m][d][(pred,orig,succ)] defines the sector of the octilinear position an edges could lie
			 * m = number of edges
			 * d = direction of the edge 0 is going and 1 is back
			 * (pred,orig,succ) = 0 lies on the previous sector, 1 lies ont the original best sector, and 2 lies on the succecive sector*/ 
			int[][][] sec = new int[m][2][3];		
			/*int[] origBestSec = new int[m];
			int[] succBestSec = new int[m];
			int[] predBestSec = new int[m];*/
			
			for (int i = 0; i < m; i++) {

				if(i == 0 && startEdgeDirection >  -1)
					sec[i][0][1] = startEdgeDirection;

				else if (i == m-1 && endEdgeDirection >  -1)
					sec[i][0][1] = endEdgeDirection;
				else				
					sec[i][0][1] = GeometricOperation.sectorOf(
						GeometricOperation.getAngleBetweenPointsRelativeToAxisX( 
								pathRelevantPointsList.get(i).getX(), pathRelevantPointsList.get(i).getY(),
								pathRelevantPointsList.get(i + 1).getX(), pathRelevantPointsList.get(i + 1).getY()					
								)
						);
				sec[i][0][0] =  Math.floorMod(sec[i][0][1] - 1, 8);
				sec[i][0][2] =  Math.floorMod(sec[i][0][1] + 1, 8);
				
				
				sec[i][1][1] = Math.floorMod(sec[i][0][1] + 4,8);
				sec[i][1][0] = Math.floorMod(sec[i][0][0] + 4,8);
				sec[i][1][2] = Math.floorMod(sec[i][0][2] + 4,8);
			}
		

			
			/*Boolean variable to restrict the orientation of the edge to orig = 1, succ = 2 and prd = 0*/
			IloNumVar[][] alpha = new IloNumVar[m][];
			
			for (int i = 0; i < m; i++) {
				alpha[i] = cplex.boolVarArray(3);
				
			}
			/**force alfa to best direction if start and end edges directions are set **/
			if(startEdgeDirection >  -1){

				cplex.addEq(alpha[0][0], 0);
				cplex.addEq(alpha[0][1], 1);
				cplex.addEq(alpha[0][2], 0);



			}
			if(endEdgeDirection >  -1){

				cplex.addEq(alpha[m-1][0], 0);
				cplex.addEq(alpha[m-1][1], 1);
				cplex.addEq(alpha[m-1][2], 0);



			}
			
			/*IloNumVar[] isOrig = cplex.boolVarArray(m);
			IloNumVar[] isSucc = cplex.boolVarArray(m);
			IloNumVar[] isPred = cplex.boolVarArray(m);*/
			
			IloLinearNumExpr[] constraintEdgeOrientation = new IloLinearNumExpr[m];
			
			for (int i = 0; i < m; i++) {
				constraintEdgeOrientation[i] = cplex.linearNumExpr();
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][0]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][1]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][2]);
				cplex.addEq(constraintEdgeOrientation[i], 1.0);
			}
			
			
			/*Variable dir[i][j] defines the direction of of edege i (j means de drirection of the edege*/
			IloNumVar[][] dir  = new IloNumVar[m][]; /*leave the last dimension to be defined with cplex*/
			
			for (int i = 0; i < m; i++) {
				dir[i] = cplex.intVarArray(2, 0, 7);/* array of size 2 because we need uv an vu*/
				
			}
			
			
			/*For each i ∈ {pred, orig, succ} we have the following set of constraints
			 dir(u, v)−seciu(v) ≤M(1−αi(u, v))
			−dir(u, v)+seciu(v) ≤M(1−αi(u, v))
			dir(v,u)−seciv(u) ≤M(1−αi(u, v))
			−dir(v,u)+seciv(u) ≤M(1−αi(u, v))
			
			∀{u, v} ∈ E,
			
			 *Here, if αi(u, v) = 0, the constraints in (4) are trivially fulfilled and do not influence the left-hand sides. 
			 *On the other hand, if αi(u, v) = 1, the four inequalities are equivalent to dir(u, v) = seciu(v) and
				dir(v,u) = seciv(u) as desired (equality
			 *
			 *
			 *
			 */
			
			double MDir = 8; 
			for (int i = 0; i < m; i++) {
				if(i == 0 && startEdgeDirection >  -1){
					cplex.addEq(dir[i][0],startEdgeDirection);
					cplex.addEq(dir[i][1],Math.floorMod(startEdgeDirection + 4,8));
				}
				else if (i == m-1 && endEdgeDirection >  -1){
					cplex.addEq(dir[i][0],endEdgeDirection);
					cplex.addEq(dir[i][1],Math.floorMod(endEdgeDirection + 4,8));
				}
				else{
					for (int j = 0; j < 3; j++) {
						IloNumExpr rightSide = cplex.prod(MDir, cplex.diff(1, alpha[i][j]));
						/*natural direction of the edge d=0*/
						IloNumExpr leftSide1 = cplex.diff(dir[i][0], sec[i][0][j]);
						IloNumExpr leftSide2 = cplex.diff(sec[i][0][j],  dir[i][0]);
						/*counter direction of the edge d= 1*/
						IloNumExpr leftSide3 = cplex.diff(dir[i][1], sec[i][1][j]);
						IloNumExpr leftSide4 = cplex.diff(sec[i][1][j],  dir[i][1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);
					
					}
				}
				
			
			}
			
			/*contraints to force the correct position of the vertices
			 * if sec original is 2 and alphaoriginal is true then forces x(u) and x(v) to equal
			 * and y(v) > y(u)*/
			double MAdjVertice = 2*maxAdjDist; /* D max distance between two vertices*/
			for (int i = 0; i < m; i++) {
				
				for (int j = 0; j < 3; j++) {
					IloNumExpr rightSide, rightSide2, leftSide1, leftSide2, leftSide3;
					/*do i need to constraint edges for both directions(sec[m][1])?*/
					switch (sec[i][0][j]) {
					case 0:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(y[i], y[i+1]);
						leftSide2 = cplex.diff(y[i+1],  y[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(x[i+1],  x[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
					
						break;
					case 7:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[i], z2[i+1]);
						leftSide2 = cplex.diff(z2[i+1],  z2[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z1[i+1],  z1[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;
						
					case 6:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[i], x[i+1]);
						leftSide2 = cplex.diff(x[i+1],  x[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(y[i+1],  y[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);

						break;
						
					case 5:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[i], z1[i+1]);
						leftSide2 = cplex.diff(z1[i+1],  z1[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z2[i],  z2[i + 1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					case 4:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(y[i], y[i+1]);
						leftSide2 = cplex.diff(y[i+1],  y[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(x[i],  x[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;
					case 3:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[i], z2[i+1]);
						leftSide2 = cplex.diff(z2[i+1],  z2[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z1[i],  z1[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					case 2:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[i], x[i+1]);
						leftSide2 = cplex.diff(x[i+1],  x[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(y[i],  y[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);

						break;	
					case 1:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[i], z1[i+1]);
						leftSide2 = cplex.diff(z1[i+1],  z1[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z2[i+1],  z2[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					default:
						break;
					}
					
				}
				
			}
			
			
//			/*******CHECK SELF PLANARITY*******///
			System.out.println("checkin topology: " + checkSelfTopology);
			if(checkSelfTopology){
				IloNumVar[][][] gama = new IloNumVar[m][m][];
				/*RElation of m edges from the path and r route edges check relative position to 8 orientations ∈{N,S,E,W,NE,NW,SE,SW}*/
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < m; j++)
						gama[i][j] = cplex.boolVarArray(8);
				
			
			
				IloLinearNumExpr[][] constraintEdgeTopology = new IloLinearNumExpr[m][m];
				/*SumOF{ γi(e1, e2) } ≥ 1   
				 * i∈{N,S,E,W,NE,NW,SE,SW} */
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < m; j++){
						//				for (int k = 0; k < 8; k++){
						//					if(k!=0)
						//						cplex.addEq(gama[i][j][k], 0);
						//					else
						//						cplex.addEq(gama[i][j][k], 1);
						//				}


						constraintEdgeTopology[i][j] = cplex.linearNumExpr();
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][0]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][1]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][2]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][3]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][4]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][5]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][6]);
						constraintEdgeTopology[i][j].addTerm(1.0,gama[i][j][7]);
						cplex.addGe(constraintEdgeTopology[i][j], 1.0);

					}


				for (int i = 0; i < m; i++) {
					for(int j = 0; j < m ; j++){

						int edge1UIndex = i ;
						int edge1VIndex = i + 1 ;

						int edge2UIndex = j ;
						int edge2VIndex = j + 1 ;

						/*condition to guarantee that path edege i and j are not adjacents*/
						if( Math.abs(i - j) > 1  && Math.abs(i - j) < m -1 )
							for(int k =0; k < 8; k ++){
								/*whats the value of M(max X ou y dist of points) and dist Min*/ 
								double MTopology = maxExtend*3; /*da pra diminuir*/
								IloNumExpr rightSide,  leftSide1, leftSide2, leftSide3, leftSide4;


								switch (k) {
								case 0: /* E (e2(route edge) ist east of e1 (path edge)) : 0 : x*/

									/*
									 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
									 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
									 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									 * ∀(e1,e2) no incident
									 */
									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(x[edge1UIndex], x[edge2UIndex]);
									leftSide2 = cplex.diff(x[edge1UIndex], x[edge2VIndex]);
									leftSide3 = cplex.diff(x[edge1VIndex], x[edge2UIndex] );
									leftSide4 = cplex.diff(x[edge1VIndex], x[edge2VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 1: /* e2 (route edge) is NE (SE because y is inverted) of e1: 45 (315) : z1 */

									/*Can I use 
									 * route.get(j).getX() + route.get(j).getY() == (route.get(j).getX() / Math.sin(Math.toRadians(45)));
									 */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(z1[edge1UIndex], z1[edge2UIndex]);
									leftSide2 = cplex.diff(z1[edge1UIndex], z1[edge2VIndex]);
									leftSide3 = cplex.diff(z1[edge1VIndex], z1[edge2UIndex]);
									leftSide4 = cplex.diff(z1[edge1VIndex], z1[edge2VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);


									break;
								case 2: /* e2 (route edge) is N (S because y is inverted) of e1: 90 (270) : y */
									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff(y[edge1UIndex], y[edge2UIndex]);
									leftSide2 = cplex.diff(y[edge1UIndex], y[edge2VIndex]);
									leftSide3 = cplex.diff(y[edge1VIndex], y[edge2UIndex]);
									leftSide4 = cplex.diff(y[edge1VIndex], y[edge2VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;	
								case 3: /* e2 (route edge) is NW (SW because y is inverted) of e1: 135 (225) : -z2 */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
									leftSide2 = cplex.diff( z2[edge2VIndex],  z2[edge1UIndex] );
									leftSide3 = cplex.diff( z2[edge2UIndex] , z2[edge1VIndex] );
									leftSide4 = cplex.diff( z2[edge2VIndex],  z2[edge1VIndex] );

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 4: /*  e2 (route edge) is W  of e1:  180:  -x */

									//						/*
									//						 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
									//						 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									//						 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
									//						 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
									//						 * ∀(e1,e2) no incident
									//						 */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff(x[edge2UIndex] , x[edge1UIndex] );
									leftSide2 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );
									leftSide3 = cplex.diff(x[edge2UIndex] , x[edge1VIndex] );
									leftSide4 = cplex.diff(x[edge2VIndex] , x[edge1VIndex] );

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 5: /*  e2 (route edge) is SW (NW because y is inverted) of e1: 225 (135): -z1 */


									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;

									leftSide1 = cplex.diff( z1[edge2UIndex] , z1[edge1UIndex]);
									leftSide2 = cplex.diff( z1[edge2VIndex] , z1[edge1UIndex]);
									leftSide3 = cplex.diff( z1[edge2UIndex] , z1[edge1VIndex]);
									leftSide4 = cplex.diff( z1[edge2VIndex] , z1[edge1VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 6:/*  e2 (route edge) is S (N because y is inverted) of e1: 270 (90)  : -y */

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(  y[edge2UIndex] ,y[edge1UIndex] );
									leftSide2 = cplex.diff(  y[edge2VIndex], y[edge1UIndex]);
									leftSide3 = cplex.diff(  y[edge2UIndex] ,y[edge1VIndex] );
									leftSide4 = cplex.diff(  y[edge2VIndex], y[edge1VIndex]);

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);

									break;
								case 7:/*  e2 (route edge) is SE (NE because y is inverted) of e1: 315 (45)  : z2*/

									rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama[i][j][k])), minNonAdjEdgeDist ) ;
									leftSide1 = cplex.diff(z2[edge1UIndex], z2[edge2UIndex] );
									leftSide2 = cplex.diff(z2[edge1UIndex], z2[edge2VIndex] );
									leftSide3 = cplex.diff(z2[edge1VIndex], z2[edge2UIndex] );
									leftSide4 = cplex.diff(z2[edge1VIndex], z2[edge2VIndex] );

									cplex.addLe(leftSide1, rightSide);
									cplex.addLe(leftSide2, rightSide);
									cplex.addLe(leftSide3, rightSide);
									cplex.addLe(leftSide4, rightSide);


									break;
								default:
									break;
								}

							}

					}

				}

			}
			
			/*********EDGE DISTANCE - EXTRA CROSSING CONSTRAINT - TOPOLOGY*****************/
			
			int e = topoCheckEdgeList.size();
			/*Boolean variable to indicate if path edge is {N,S,E,W,NE,NW,SE,SW} of route edge*/
			IloNumVar[][][] gama2 = new IloNumVar[m][e][];
			
			/*RElation of m edges from the path and r route edges check relative position to 8 orientations ∈{N,S,E,W,NE,NW,SE,SW}*/
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < e; j++)
						gama2[i][j] = cplex.boolVarArray(8);
					
				IloLinearNumExpr[][] constraintEdgeTopology2 = new IloLinearNumExpr[m][e];
				
				
				/*SumOF{ γi(e1, e2) } ≥ 1   
				 * i∈{N,S,E,W,NE,NW,SE,SW} */
				
				for (int i = 0; i < m; i++) 
					for(int j = 0; j < e; j++){
		//				for (int k = 0; k < 8; k++){
		//					if(k!=0)
		//						cplex.addEq(gama[i][j][k], 0);
		//					else
		//						cplex.addEq(gama[i][j][k], 1);
		//				}
						
						
						constraintEdgeTopology2[i][j] = cplex.linearNumExpr();
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][0]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][1]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][2]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][3]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][4]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][5]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][6]);
						constraintEdgeTopology2[i][j].addTerm(1.0,gama2[i][j][7]);
						cplex.addGe(constraintEdgeTopology2[i][j], 1.0);
			
					}
						
				if(checkTopology)
						for (int i = 0; i < m; i++) {
						for(int j = 0; j < e ; j++){
	
							int edge1UIndex = i ;
							int edge1VIndex = i + 1 ;
	
							StreetNode edge2U = nodeMap.get(topoCheckEdgeList.get(j)[0]);
							StreetNode edge2V = nodeMap.get(topoCheckEdgeList.get(j)[1]);
							
							double xedge2U = edge2U.getxGeom().getX();
							double yedge2U = edge2U.getxGeom().getY();
							double z1edge2U = 0.5*(xedge2U + yedge2U);
							double z2edge2U = 0.5*(xedge2U - yedge2U);
							
							double xedge2V = edge2V.getxGeom().getX();
							double yedge2V = edge2V.getxGeom().getY();
							double z1edge2V = 0.5*(xedge2V + yedge2V);
							double z2edge2V = 0.5*(xedge2V - yedge2V);
							
							
	
						
								for(int k =0; k < 8; k ++){
									/*whats the value of M(max X ou y dist of points) and dist Min*/ 
									double MTopology = maxExtend*3; /*da pra diminuir*/
									IloNumExpr rightSide,  leftSide1, leftSide2, leftSide3, leftSide4;
	
	
									switch (k) {
									case 0: /* E (e2(route edge) ist east of e1 (path edge)) : 0 : x*/
	
										/*
										 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
										 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
										 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										 * ∀(e1,e2) no incident
										 */
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(x[edge1UIndex],xedge2U);
										leftSide2 = cplex.diff(x[edge1UIndex], xedge2V);
										leftSide3 = cplex.diff(x[edge1VIndex],xedge2U );
										leftSide4 = cplex.diff(x[edge1VIndex], xedge2V);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 1: /* e2 (route edge) is NE (SE because y is inverted) of e1: 45 (315) : z1 */
	
										/*Can I use 
										 * route.get(j).getX() + route.get(j).getY() == (route.get(j).getX() / Math.sin(Math.toRadians(45)));
										 */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(z1[edge1UIndex], z1edge2U);
										leftSide2 = cplex.diff(z1[edge1UIndex], z1edge2V);
										leftSide3 = cplex.diff(z1[edge1VIndex], z1edge2U);
										leftSide4 = cplex.diff(z1[edge1VIndex], z1edge2V);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
	
										break;
									case 2: /* e2 (route edge) is N (S because y is inverted) of e1: 90 (270) : y */
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff(y[edge1UIndex], yedge2U);
										leftSide2 = cplex.diff(y[edge1UIndex], yedge2V);
										leftSide3 = cplex.diff(y[edge1VIndex], yedge2U);
										leftSide4 = cplex.diff(y[edge1VIndex], yedge2V);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;	
									case 3: /* e2 (route edge) is NW (SW because y is inverted) of e1: 135 (225) : -z2 */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff( z2edge2U , z2[edge1UIndex] );
										leftSide2 = cplex.diff( z2edge2V,  z2[edge1UIndex] );
										leftSide3 = cplex.diff( z2edge2U , z2[edge1VIndex] );
										leftSide4 = cplex.diff( z2edge2V,  z2[edge1VIndex] );
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 4: /*  e2 (route edge) is W  of e1:  180:  -x */
	
										//						/*
										//						 * x(u1)−x(u2) ≤ M(1−γE(e1, e2))−dmin
										//						 * x(u1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										//						 * x(v1)−x(u2) ≤ M(1−γE(e1, e2))−dmin 
										//						 * x(v1)−x(v2) ≤ M(1−γE(e1, e2))−dmin 
										//						 * ∀(e1,e2) no incident
										//						 */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff(xedge2U , x[edge1UIndex] );
										leftSide2 = cplex.diff(xedge2V , x[edge1UIndex] );
										leftSide3 = cplex.diff(xedge2U , x[edge1VIndex] );
										leftSide4 = cplex.diff(xedge2V , x[edge1VIndex] );
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 5: /*  e2 (route edge) is SW (NW because y is inverted) of e1: 225 (135): -z1 */
	
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
	
										leftSide1 = cplex.diff( z1edge2U , z1[edge1UIndex]);
										leftSide2 = cplex.diff( z1edge2V , z1[edge1UIndex]);
										leftSide3 = cplex.diff( z1edge2U , z1[edge1VIndex]);
										leftSide4 = cplex.diff( z1edge2V , z1[edge1VIndex]);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 6:/*  e2 (route edge) is S (N because y is inverted) of e1: 270 (90)  : -y */
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(  yedge2U ,y[edge1UIndex] );
										leftSide2 = cplex.diff(  yedge2V, y[edge1UIndex]);
										leftSide3 = cplex.diff(  yedge2U ,y[edge1VIndex] );
										leftSide4 = cplex.diff(  yedge2V, y[edge1VIndex]);
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
										break;
									case 7:/*  e2 (route edge) is SE (NE because y is inverted) of e1: 315 (45)  : z2*/
	
										rightSide = cplex.diff( cplex.prod(MTopology, cplex.diff(1, gama2[i][j][k])), minNonAdjEdgeDist ) ;
										leftSide1 = cplex.diff(z2[edge1UIndex], z2edge2U );
										leftSide2 = cplex.diff(z2[edge1UIndex], z2edge2V );
										leftSide3 = cplex.diff(z2[edge1VIndex], z2edge2U );
										leftSide4 = cplex.diff(z2[edge1VIndex], z2edge2V );
	
										cplex.addLe(leftSide1, rightSide);
										cplex.addLe(leftSide2, rightSide);
										cplex.addLe(leftSide3, rightSide);
										cplex.addLe(leftSide4, rightSide);
	
	
										break;
									default:
										break;
									}
	
								}
	
						}
				
				}
			
			
			
			/*********BEND MINIMIZATION*****************/

			
			
			
			/*Boolean variable to make condition of bends angle <-5, >-4 e <4, ou >5 measure bends angles (-7 to 7)*/
			/***∆dir(u, v,w) = dir(u, v) − dir(v,w)***/
			/***∆dir(u, v,w) ranges from −7 to 7****/
			
			IloNumVar[][] delta = new IloNumVar[b][];
			
			for (int i = 0; i < b; i++) {
				delta[i] = cplex.boolVarArray(3);
				
			}
			IloLinearNumExpr[] constraintBenMinimizationBooleans = new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				constraintBenMinimizationBooleans[i] = cplex.linearNumExpr();
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][0]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][1]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][2]);
				cplex.addEq(constraintBenMinimizationBooleans[i], 2.0);
			}
			double MBend = 15; /*diference between 2 dir */
			IloNumVar[] bendDir = cplex.intVarArray(b, -7, 7);
			/* bendDir[1] = dir[0] - dir[1]*/

			for (int i = 0; i < b; i++) {
				cplex.addEq(bendDir[i], cplex.diff(dir[i][0], dir[i + 1][0]));
			}
			
			for (int i = 0; i < b; i++) {
				cplex.addLe(bendDir[i], cplex.diff(cplex.prod(MBend, delta[i][0]), 5));
				cplex.addGe(bendDir[i], cplex.diff(5 , cplex.prod(MBend, delta[i][1])));
				cplex.addLe(bendDir[i], cplex.sum(4 , cplex.prod(MBend, delta[i][2])));
				cplex.addGe(bendDir[i], cplex.diff(-4 , cplex.prod(MBend, delta[i][2])));
				
			}
			
			IloNumVar[] bendCost = cplex.intVarArray(b, 0, 3);
			IloLinearNumExpr[] bendCostConstraint =  new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				bendCostConstraint[i] = cplex.linearNumExpr();
				bendCostConstraint[i].addTerm(1, bendDir[i]);
				bendCostConstraint[i].addTerm(-8, delta[i][0]);
				bendCostConstraint[i].addTerm(8, delta[i][1]);
				cplex.addGe(bendCostConstraint[i], cplex.prod(-1, bendCost[i]) );
				cplex.addLe(bendCostConstraint[i], bendCost[i] );
				

				
			}

			IloNumExpr sumBendCost =  cplex.sum(bendCost);

			
			
			/*********Distance Minization Anchors*****************/
			ArrayList<Integer> distNodeIndex = new ArrayList<Integer>();
			for (int i = 0; i < n; i++) {
				int pathIndex = pathRelevantPointsIndex.get(i);
				if( 	//path.getNodeList().get(pathIndex).getDegree() != 2 ||
						path.getNodeList().get(pathIndex).isTopoAnchor()
						) 
					distNodeIndex.add(i);				
			}
			int ndA = distNodeIndex.size();
			IloNumVar[] dXA  = cplex.numVarArray(ndA, 0, Double.MAX_VALUE);
			IloNumVar[] dYA  = cplex.numVarArray(ndA, 0, Double.MAX_VALUE);
			for (int i = 0; i < ndA; i++) {
				int pathPtIndex = distNodeIndex.get(i);
				
				IloNumExpr xLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getX(), x[pathPtIndex]);
				IloNumExpr yLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getY(), y[pathPtIndex]);
				
				cplex.addLe(xLeftSide,dXA[i]);
				cplex.addGe(xLeftSide, cplex.prod(-1, dXA[i]));
				
				
				cplex.addLe(yLeftSide, dYA[i]);
				cplex.addGe(yLeftSide, cplex.prod(-1, dYA[i]));

				
			}
			IloNumExpr sumXDistA =  cplex.sum(dXA);
			IloNumExpr sumYDistA =  cplex.sum(dYA);
			
			
			
			/*********Distance Minization *****************/

//			ArrayList<Integer> distNodeIndex = new ArrayList<Integer>();
//			for (int i = 0; i < n; i++) {
//				int pathIndex = pathRelevantPointsIndex.get(i);
////				if( 	path.getNodeList().get(pathIndex).getDegree() != 2 ||
////						path.getNodeList().get(pathIndex).isTopoAnchor()
////						) 
//					distNodeIndex.add(i);				
//			}
//			int nd = distNodeIndex.size();
//			IloNumVar[] dX  = cplex.numVarArray(nd, 0, Double.MAX_VALUE);
//			IloNumVar[] dY  = cplex.numVarArray(nd, 0, Double.MAX_VALUE);
//			double factor = 1;
//			for (int i = 0; i < nd; i++) {
//				int pathPtIndex = distNodeIndex.get(i);
//				
//				IloNumExpr xLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getX(), x[pathPtIndex]);
//				IloNumExpr yLeftSide = cplex.diff(pathRelevantPointsList.get(pathPtIndex).getY(), y[pathPtIndex]);
//				
//				cplex.addLe(xLeftSide,cplex.prod(factor, dX[i]));
//				cplex.addGe(xLeftSide, cplex.prod(-1*factor, dX[i]));
//				
//				
//				cplex.addLe(yLeftSide, cplex.prod(factor,dY[i]));
//				cplex.addGe(yLeftSide, cplex.prod(-1*factor, dY[i]));
//
//				
//			}
//			IloNumExpr sumXDist =  cplex.sum(dX);
//			IloNumExpr sumYDist =  cplex.sum(dY);
			
			
//			/*distance reduction*/
			
			IloNumVar[] dX  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			IloNumVar[] dY  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			//IloNumVar[] dMax  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			
			for (int i = 0; i < n; i++) {

					
				IloNumExpr xLeftSide = cplex.diff(pathRelevantPointsList.get(i).getX(), x[i]);
				IloNumExpr yLeftSide = cplex.diff(pathRelevantPointsList.get(i).getY(), y[i]);

				cplex.addLe(xLeftSide,dX[i]);
				cplex.addGe(xLeftSide, cplex.prod(-1, dX[i]));


				cplex.addLe(yLeftSide, dY[i]);
				cplex.addGe(yLeftSide, cplex.prod(-1, dY[i]));
				
			}
//			
			IloNumExpr sumXDist =  cplex.sum(dX);
			IloNumExpr sumYDist =  cplex.sum(dY);
			

			
			
			/*************Proportion diference minimization***********/

			/*** lambda: binary variable to determine if (xi - xi+1) is > 0 or < 0**
			 * lambda1 + lambda2 =1 -> if lambda1 = 1 - > lambda2 = 0 ->  (xi - xi+1) >= 0
			 * 							else lambda1 = 0 -> lambda2 = 1 -> (xi - xi+1) < 0
			 * 
			 * we want dx = |xi - xi+1| -> dx >= |xi - xi+1| & dx <= |xi - xi+1|
			 * if (xi - xi+1) >= 0 -> dx >= xi - xi+1 & dx <= xi - xi+1
			 * if (xi - xi+1) < 0  -> dx >= -(xi - xi+1) & dx <= -(xi - xi+1) 
			 * 
			 * so
			 * dx <= (xi - xi+1) + M(1-lambda1) eq1
			 * dx >= (xi - xi+1) - M(1-lambda1) eq2
			 * 
			 * or
			 * dx >= -(xi - xi+1) - M(1-lambda2) eq3
			 * dx <= -(xi - xi+1) + M(1-lambda2) eq4
			 * 
			 * if lambda1 = 1 & lambda2 = 0  => constraint dx = (xi - xi+1) & e3 and eq4 are trivialy fulfiled(makes no diference) 
			 * else lambda1 = 0 & lambda2 = 1  => constraint dx = -(xi - xi+1) & e1 and eq2 are trivialy fulfiled(makes no diference) 
			 * 
			 * 
			 * 
			 * */
			
			int mP = m; /*route edges number: respect order on edge list*/ 
			
			IloNumVar[][] xLambda = new IloNumVar[mP][];
			IloNumVar[][] yLambda = new IloNumVar[mP][];
			
			for (int i = 0; i < mP; i++) {
				xLambda[i] = cplex.boolVarArray(2);
				yLambda[i] = cplex.boolVarArray(2);
				xLambda[i][0].setName("proporMinxLambda0");
				xLambda[i][1].setName("proporMinxLambda1");
				yLambda[i][0].setName("proporMinyLambda0");
				yLambda[i][1].setName("proporMinyLambda1");
				
			}
			IloLinearNumExpr[] xLambdaConstraint = new IloLinearNumExpr[mP];
			IloLinearNumExpr[] yLambdaConstraint = new IloLinearNumExpr[mP];
			for (int i = 0; i < mP; i++) {
				xLambdaConstraint[i] = cplex.linearNumExpr();
				xLambdaConstraint[i].addTerm(1.0,xLambda[i][0]);
				xLambdaConstraint[i].addTerm(1.0,xLambda[i][1]);

				cplex.addEq(xLambdaConstraint[i], 1.0);
				
				
				yLambdaConstraint[i] = cplex.linearNumExpr();
				yLambdaConstraint[i].addTerm(1.0,yLambda[i][0]);
				yLambdaConstraint[i].addTerm(1.0,yLambda[i][1]);

				cplex.addEq(yLambdaConstraint[i], 1.0);
				
			}
			
			 
			/*  maxAdjD distance between two adjacent vertices vertices! it was MaxD*/
			MAdjVertice = pathLength*1.5;
			IloNumVar[] absoluteDiffX  = cplex.numVarArray(mP, 0, 4* maxAdjDist);
			IloNumVar[] absoluteDiffY  = cplex.numVarArray(mP, 0, 4* maxAdjDist);
			
			for (int i = 0; i < mP; i++) {
				
				int v1Index = i;
				int v2Index = i+1;
				
				/***dx = |xi - xi+1|***/
				IloNumExpr diffX = cplex.diff(x[v1Index], x[v2Index]);
				
				cplex.addLe(absoluteDiffX[i], cplex.sum(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffX[i], cplex.diff(diffX, cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffX[i], cplex.diff(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffX[i], cplex.sum(cplex.negative(diffX), cplex.prod(MAdjVertice, cplex.diff(1, xLambda[i][1])))); //eq4.
				
				/***dy = |xi - yi+1|***/
				IloNumExpr diffY = cplex.diff(y[v1Index], y[v2Index]);
				
				cplex.addLe(absoluteDiffY[i], cplex.sum(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq1. 
				cplex.addGe(absoluteDiffY[i], cplex.diff(diffY, cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][0])))); //eq2.
				
				cplex.addGe(absoluteDiffY[i], cplex.diff(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq3. 
				cplex.addLe(absoluteDiffY[i], cplex.sum(cplex.negative(diffY), cplex.prod(MAdjVertice, cplex.diff(1, yLambda[i][1])))); //eq4.

				
				
			}
			/***WE have absDiffX and absDiffY
			 * We wanto to minimize:
			 * Abs((absDiffX[i] + absDiffY[i])L - lengthEdge[i]/L)
			 * 
			 * ****/
			/**
			 * If you are minimizing an increasing function of |x| (or maximizing a decreasing function, of course), you can always have the aboslute value of any quantity x in a lp as a variable absx such as:
				absx >= x -> x<=absx
				absx >= -x -> x >= -absx
				{leftside = x
				dX = absx}
				It works because the value absx will 'tend' to its lower bound, so it will either reach x or -x.
				On the other hand, if you are minimizing a decreasing function of |x|, your problem is not convex and cannot be modelled as a lp.
			 */
			ArrayList<Double> edgeProportion = new ArrayList<Double>();
			for (int i = 0; i < mP; i++) {
	
				//edgeProportion.add(pt1.distance(pt2)/routeLenght);
				edgeProportion.add(pathRelevantPointsList.get(i).distance(pathRelevantPointsList.get(i+1))/pathLength);
			}
			IloNumVar[] absolutEdgeProportionDifference  = cplex.numVarArray(mP, 0, Double.MAX_VALUE);
			IloNumExpr[] finalEdgeProportion = new IloNumExpr[mP];
			for (int i = 0; i < mP; i++) {
				finalEdgeProportion[i] = cplex.prod(1/pathLength,cplex.sum(absoluteDiffX[i],absoluteDiffY[i]));
				/*can I use cplex.Equal direct? yes absX and absY is always positiv*/
				cplex.addLe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), absolutEdgeProportionDifference[i]);
				cplex.addGe(cplex.diff(finalEdgeProportion[i], edgeProportion.get(i)), cplex.prod(-1, absolutEdgeProportionDifference[i]));
			}
			
			IloNumExpr sumAbsolutEdgeProportionDifference =  cplex.sum(absolutEdgeProportionDifference);

			
			/****Edges orientation difference minimization*********/
			IloNumVar[] dirDiff  = cplex.numVarArray(m, -7, 7);
			for (int i = 0; i < m; i++) {
				cplex.addEq(dirDiff[i], cplex.diff(sec[i][0][1] ,dir[i][0]));
			}
			
			IloNumVar[] dirCost = cplex.boolVarArray(m);
			for (int i = 0; i < m; i++) {

				cplex.addLe(dirDiff[i],cplex.prod(8, dirCost[i]));
				cplex.addGe(dirDiff[i],  cplex.prod(-8, dirCost[i]));
			}

//			IloNumVar[][] zeta = new IloNumVar[m][];
//			
//			for (int i = 0; i < m; i++) {
//				zeta[i] = cplex.boolVarArray(3);
//				zeta[i][0].setName("ZETADirMin0");
//				zeta[i][1].setName("ZETADirMin1");
//				zeta[i][2].setName("ZETADirMin2");
//				
//			}
//			IloLinearNumExpr[] constraintDirDiffMinimizationBooleans = new IloLinearNumExpr[m];
//			for (int i = 0; i < m; i++) {
//				constraintDirDiffMinimizationBooleans[i] = cplex.linearNumExpr();
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][0]);
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][1]);
//				constraintDirDiffMinimizationBooleans[i].addTerm(1.0,zeta[i][2]);
//				cplex.addEq(constraintDirDiffMinimizationBooleans[i], 2.0);
//			}
//
//			MDir = 8;
//			for (int i = 0; i < m; i++) {
//				cplex.addLe(dirDiff[i], cplex.diff(cplex.prod(MDir, zeta[i][0]), 5));
//				cplex.addGe(dirDiff[i], cplex.diff(5 , cplex.prod(MDir, zeta[i][1])));
//				cplex.addLe(dirDiff[i], cplex.sum(4 , cplex.prod(MDir, zeta[i][2])));
//				cplex.addGe(dirDiff[i], cplex.diff(-4 , cplex.prod(MDir, zeta[i][2])));
//				
//			}
//			IloNumVar[] dirCost = cplex.intVarArray(m, 0, 4);
//			IloLinearNumExpr[] dirCostConstraint =  new IloLinearNumExpr[m];
//			for (int i = 0; i < m; i++) {
//				dirCostConstraint[i] = cplex.linearNumExpr();
//				dirCostConstraint[i].addTerm(1, dirDiff[i]);
//				dirCostConstraint[i].addTerm(-8, zeta[i][0]);
//				dirCostConstraint[i].addTerm(8, zeta[i][1]);
//				cplex.addGe(dirCostConstraint[i], cplex.prod(-1, dirCost[i]) );
//				cplex.addLe(dirCostConstraint[i], dirCost[i] );
//
//				
//			}
			IloNumExpr sumDirCost =  cplex.sum(dirCost);  
			
			
			
			/*********OBJECTIVE*****************/
			
//			IloNumExpr sumBendCostNormal = cplex.prod(0.5/Math.sqrt(b),sumBendCost);
//			IloNumExpr sumBendCostFactor = cplex.prod(bendFactor,sumBendCostNormal);
			
//			double maxExted = octBox.getMaxExtend();
//			IloNumExpr sumDistCostNormal = cplex.prod((20.0)/(n*maxExted), cplex.sum(sumXDist, sumYDist ));			
//			//IloNumExpr sumDistCostNormal = cplex.prod((50.0)/(n*pahtLength), cplex.sum(sumXDist, sumYDist ));			
//			IloNumExpr sumDistCostFactor = cplex.prod(distFactor, sumDistCostNormal);
			
			
//			IloNumExpr sumDistCostNormalA = cplex.prod((10.0)/(ndA), cplex.sum(sumXDistA, sumYDistA ));
//			IloNumExpr sumDistCostFactorA = cplex.prod(distFactor, sumDistCostNormalA);
			
			
//			IloNumExpr sumAbsolutEdgeProportionDifferenceNormal = cplex.prod(10.0/(mP), sumAbsolutEdgeProportionDifference);			
//			IloNumExpr sumEdgeProportionFactor = cplex.prod(proportionFactor, sumAbsolutEdgeProportionDifferenceNormal);
			
//			IloNumExpr sumDirCostNormal = cplex.prod((5.0/m),sumDirCost);
//			IloNumExpr sumDirCostFactor = cplex.prod((dirFactor),sumDirCostNormal);
			
			
		
//			double bendWeight = bendFactor*0.6* Math.pow((pahtLength*100),0.5);
//			double bendWeightNormalized = bendWeight/b;
//			//IloNumExpr sumBendCostNormal = cplex.prod(0.5/Math.sqrt(b),sumBendCost);
//			IloNumExpr sumBendCostFactor = cplex.prod(bendWeightNormalized,sumBendCost);
////			
////			
//			double distWeight = distFactor*200.0;
//			
//			double distWeightNormalized = distWeight/(n);
//			//IloNumExpr sumDistCostNormal = cplex.prod((20.0)/(n*maxExted), cplex.sum(sumXDist, sumYDist ));			
//
//			//IloNumExpr sumDistCostNormal = cplex.prod((50.0)/(n*pahtLength), cplex.sum(sumXDist, sumYDist ));			
//			IloNumExpr sumDistCostFactor = cplex.prod(distWeightNormalized, cplex.sum(sumXDist, sumYDist ));
////			
////			
//			double distWeightA = distFactor*10.0;
//			double distWeightANormalized = distWeightA/ndA;
////			//IloNumExpr sumDistCostNormalA = cplex.prod((10.0)/(ndA), cplex.sum(sumXDistA, sumYDistA ));
//			IloNumExpr sumDistCostFactorA = cplex.prod(distWeightANormalized, cplex.sum(sumXDistA, sumYDistA ));
////			
////			
//			double propWeight = proportionFactor*10.0;
//			double propNormalWeight = propWeight/mP;
////			//IloNumExpr sumAbsolutEdgeProportionDifferenceNormal = cplex.prod(10.0/(mP), sumAbsolutEdgeProportionDifference);			
//			IloNumExpr sumEdgeProportionFactor = cplex.prod(propNormalWeight, sumAbsolutEdgeProportionDifference);
////			
//			double dirWeight = dirFactor*5.0;
//			double dirWeightNormalized = dirWeight/m;
////			//IloNumExpr sumDirCostNormal = cplex.prod((5.0/m),sumDirCost);
//			IloNumExpr sumDirCostFactor = cplex.prod(dirWeightNormalized,sumDirCost);
			
			
			
			
			double bendWeight = bendFactor*0.5;
			double bendWeightNormalized = bendWeight/Math.sqrt(b);
			//IloNumExpr sumBendCostNormal = cplex.prod(0.5/Math.sqrt(b),sumBendCost);
			IloNumExpr sumBendCostFactor = cplex.prod(bendWeightNormalized,sumBendCost);
//			
//			
			double distWeight = distFactor*20.0;
			
			double distWeightNormalized = distWeight/(n*octBoxMaxExted);
			//IloNumExpr sumDistCostNormal = cplex.prod((20.0)/(n*maxExted), cplex.sum(sumXDist, sumYDist ));			

			//IloNumExpr sumDistCostNormal = cplex.prod((50.0)/(n*pahtLength), cplex.sum(sumXDist, sumYDist ));			
			IloNumExpr sumDistCostFactor = cplex.prod(distWeightNormalized, cplex.sum(sumXDist, sumYDist ));
//			
//			
			double distWeightA = distFactor*10.0;
			double distWeightANormalized = distWeightA/ndA;
//			//IloNumExpr sumDistCostNormalA = cplex.prod((10.0)/(ndA), cplex.sum(sumXDistA, sumYDistA ));
			IloNumExpr sumDistCostFactorA = cplex.prod(distWeightANormalized, cplex.sum(sumXDistA, sumYDistA ));
//			
//			
			double propWeight = proportionFactor*10.0;
			double propNormalWeight = propWeight/mP;
//			//IloNumExpr sumAbsolutEdgeProportionDifferenceNormal = cplex.prod(10.0/(mP), sumAbsolutEdgeProportionDifference);			
			IloNumExpr sumEdgeProportionFactor = cplex.prod(propNormalWeight, sumAbsolutEdgeProportionDifference);
//			
			double dirWeight = dirFactor*5.0;
			double dirWeightNormalized = dirWeight/m;
//			//IloNumExpr sumDirCostNormal = cplex.prod((5.0/m),sumDirCost);
			IloNumExpr sumDirCostFactor = cplex.prod(dirWeightNormalized,sumDirCost);
			
			IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumDistCostFactorA,  sumBendCostFactor,  sumEdgeProportionFactor, sumDirCostFactor ) ;
			//IloNumExpr objective = cplex.sum(  sumDistCostFactor, sumBendCostFactor, sumDirCostFactor) ;

			//IloNumExpr objective = sumBendCostFactor ;
			//IloNumExpr objective = cplex.sum(sumXDist, sumYDist ) ;

			

			
			cplex.addMinimize(objective);
			if(executionTimeLimit > 0 ){
				double limitTimeInSeconds = ((double)(executionTimeLimit))/1000;
				cplex.setParam(IloCplex.DoubleParam.TiLim, limitTimeInSeconds);
			}
			else if(executionTimeLimit <0 ){
				//cplex.setParam(IloCplex.Param.MIP.Limits.Solutions, 1);
				cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, (-(double)executionTimeLimit)/100.0);
				pathResultReport.setGap(-((double)executionTimeLimit)/100.0);
				
				cplex.setParam(IloCplex.DoubleParam.TiLim, 120);
			}
			

			/*control display information*/
			//cplex.setParam(IloCplex.Param.MIP.Display, 1);

			if(cplex.solve()){
	
				System.out.println("obj =  " +cplex.getObjValue());
				for (int i = 0; i < n; i++) {
					Point2D.Double pt = new Point2D.Double(cplex.getValue(x[i]),cplex.getValue(y[i])); 
					mipLineString.add(pt);
				}	
				ArrayList<Point2D> finalPoints = new ArrayList<Point2D>();
				/****USE IT TO REFILL PATH IF YOU NEED TO SIMPLIFY****/
				
				finalPoints.add(mipLineString.get(0));
				for(int i = 1; i < pathRelevantPointsIndex.size()  ; i++){
					int index = pathRelevantPointsIndex.get(i);
					int lastIndexInFinalPoints = finalPoints.size() - 1;
					/*adicional o proximo ponto relevante entre ele e o anterior na linha reta com os ponts complementares*/
					finalPoints.add( mipLineString.get(i) );
					finalPoints = GeometricOperation.fillLine( finalPoints , transPointList,  index - lastIndexInFinalPoints   , lastIndexInFinalPoints );
				}
				
//				ArrayList<Point2D> routePathNormalized =  route.getRoutePath().asJava2DList(1);
//				finalPoints.add(mipLineString.get(0));
//				for(int i = 1; i < route.getRelevantPointIndex().size() ; i++){
//					int index = route.getRelevantPointIndex().get(i);
//					int lastIndexInFinalPoints = finalPoints.size() - 1;
//					/*adicional o proximo ponto relevante entre ele e o anterior na linha reta com os ponts complementares*/
//					finalPoints.add( mipLineString.get(i) );
//					finalPoints = GeometricOperation.fillLine( finalPoints , routePathNormalized,  index - lastIndexInFinalPoints   , lastIndexInFinalPoints );
//				}
//				route.getRoutePath().updatePathXNodes( finalPoints);
				
				
				//System.out.println("Only ADj");
			

				
//				for(int i = 0; i < n ; i++){
//					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
	//
//				}
				
//				for (int i = 0; i < mP; i++){
//					System.out.println("Edge: " + i);
//					System.out.println("AbsDiffX-> =  " + cplex.getValue(absoluteDiffX[i]) + " AbsDiffY-> =  " + cplex.getValue(absoluteDiffY[i]) );
//					System.out.println("True edge proportion: " + edgeProportion.get(i) + "Final edgeProportion: " + cplex.getValue(finalEdgeProportion[i]) + "Absolut Difference: " + cplex.getValue(absolutEdgeProportionDifference[i]));
//				}
//				for (int i = 0; i < b ; i++){
//					//System.out.println("benDir-> =  " + cplex.getValue(bendDir[i-2]) + " bencost-> =  " + cplex.getValue(bendCost[i-2]) );
//					//System.out.println("gama0-> =  " + cplex.getValue(gama[i-2][0]) + " gama1-> =  " + cplex.getValue(gama[i-2][1])  + " gama2-> =  " + cplex.getValue(gama[i-2][2]));
//					
//					//System.out.println("origTurnDir->= "+ origTurnDir[i -1] + " benDir-> =  " + cplex.getValue(bendDir[i-1]) + " bencost-> =  " + cplex.getValue(bendCost[i-1]) );
//				}
//				
//				for (int i = 0; i <n ; i++){
//					System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//				}
//				for (int i = 0; i <nd ; i++){
//					System.out.println("dx"+ i + " =  " + cplex.getValue(dX[i]) + " dy"+ i + " =  " + cplex.getValue(dY[i]) );
//				}
				
//				for (int i = 0; i < m ; i++){
//					//System.out.println("benDir-> =  " + cplex.getValue(bendDir[i-2]) + " bencost-> =  " + cplex.getValue(bendCost[i-2]) );
//					System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dir-> =  " + cplex.getValue(dir[i][0]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]));
//					
//				}
//				for (int i = 0; i < nA ; i++){
//					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
//				}

//				for (int i = nR -1; i < m ; i++){
//					
//					System.out.println("x"+ i+1 + " =  " + cplex.getValue(x[i+1]) + " y"+ i+1 + " =  " + cplex.getValue(y[i+1]) + " z1"+ i+1 + " =  " + cplex.getValue(z1[i+1]) + " z2"+ i+1 + " =  " + cplex.getValue(z2[i+1]));
//				//	System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
	//
//					System.out.println("Route Point: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom() );
//					System.out.println("Adj Point: " +streetNodeMap.get(edgeList.get(i)[1]).getxGeom() );
//					System.out.println("Dist: " +streetNodeMap.get(edgeList.get(i)[0]).getxGeom().distance(streetNodeMap.get(edgeList.get(i)[1]).getxGeom()) );
//					System.out.println("EDGE"+i+" sec =  " + sec[i][0][1] + " dir-> =  " + cplex.getValue(dir[i][0]) +  " dirCost-> =  " + cplex.getValue(dirCost[i]));
//					System.out.println();
//				}
				
				
				
//				numInterPoints = intersectionNodeIdList.size();							
//				for (int i = 0; i < numInterPoints; i++) {
//					
	//
//					int vId = intersectionNodeIdList.get(i);
////					System.out.println();
////					System.out.println("Route relevant point n  " + routeNodeIdList.indexOf(vId) + " of id "+vId+"  is intersection and its degree is " + circularOrderList.get(vId).size() );
////					System.out.println("Interpoint"+i+" Intersection Id: " + vId);
//					
//					for(int j = 0; j < beta[i].length ; j ++ ){
//						int indexDirVU1 = 0, indexDirVU2 = 0;
////						System.out.println("--Edge " + j + " and " + (j+1));
//						int u1Id, u2Id;
//						u1Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j).getId();
////						System.out.println("u1ID="+u1Id); 
//						/*se for vertive una orden circular seleciona u2 como o primeiro da lista*/
//						if(j < (beta[i].length -1))
//							u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(j + 1).getId();
//						else 
//							u2Id = circularOrderList.get(intersectionNodeIdList.get(i)).get(0).getId();
////						System.out.println("u2ID="+u2Id);
//						/***identify the edges (v, u1) and (v, u2)**/
//						boolean foundEdgeToU1 = false, foundEdgeToU2 = false;
//						int edgeToU1IsInverted = 0, edgeToU2IsInverted = 0;
//						
//						for(int k = 0; k < edgeList.size(); k++){
	//
//							/*intersectionNOdeid contain only route nodes, the adj node are always edge[1]. Otherwise is necessary to check the inverse*/
//							if(edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u1Id) {
//								indexDirVU1 = k;
//								foundEdgeToU1 = true;
//								edgeToU1IsInverted = 0;
//							}
//							else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u1Id){
//								indexDirVU1 = k;
//								foundEdgeToU1 = true;
//								edgeToU1IsInverted = 1;
//							}
//							if( edgeList.get(k)[0] == vId && edgeList.get(k)[1] == u2Id ){
//								indexDirVU2 = k;
//								foundEdgeToU2 = true;
//								edgeToU2IsInverted = 0;
//							}
//							else if(edgeList.get(k)[1] == vId && edgeList.get(k)[0] == u2Id){
//								indexDirVU2 = k;
//								foundEdgeToU2 = true;
//								edgeToU2IsInverted = 1;
//							}
//							if(foundEdgeToU1 && foundEdgeToU2)
//								break;
//						}
//						
////						System.out.println("EdgeIndex to u1ID= "+ indexDirVU1 + " dir: " + cplex.getValue(dir[indexDirVU1][edgeToU1IsInverted]) + " found: " + foundEdgeToU1  + " is inverted: " + edgeToU1IsInverted); 
////						System.out.println("EdgeIndex to u2ID= "+ indexDirVU2 + " dir: " + cplex.getValue(dir[indexDirVU2][edgeToU2IsInverted]) + " found: " + foundEdgeToU2 + " is inverted: " + edgeToU2IsInverted); 
//				
	//
//				
//					}
////					System.out.println();
//				}
				
				
				//System.out.println("distCost/Normal =  " + (cplex.getValue(sumXDist) + cplex.getValue(sumYDist)) + " / " +cplex.getValue(sumDistCostNormal) + " sumBendCost/Normal = " + cplex.getValue(sumBendCost)+ " / " +cplex.getValue(sumBendCostNormal) + " sumDirCost/Normal = " + cplex.getValue(sumDirCost) + " / " + cplex.getValue(sumDirCostNormal) + " sumEdgeProportionCost/Normal = " + cplex.getValue(sumAbsolutEdgeProportionDifference)+ " / " + cplex.getValue(sumAbsolutEdgeProportionDifferenceNormal) );
				System.out.println("distFactor =  " + distFactor + " bendFactor = " + bendFactor + " dirFactor = " + dirFactor + " proporFactor = " + proportionFactor );
				System.out.println("distCostNormalFactor =  " + cplex.getValue(sumDistCostFactor) + " sumBendCostFactor = " + cplex.getValue(sumBendCostFactor) + " sumDirCostFactor = " + cplex.getValue(sumDirCostFactor) + " sumProportionFactor = " + cplex.getValue(sumEdgeProportionFactor) );

				pathResultReport.setBendSC(new SoftConstraintValues(bendWeight, bendWeightNormalized, cplex.getValue(sumBendCostFactor)));
				pathResultReport.setDistanceWeightSC(new SoftConstraintValues(distWeight, distWeightNormalized, cplex.getValue(sumDistCostFactor)));
				pathResultReport.setDistanceTopoWeightSC(new SoftConstraintValues(distWeightA, distWeightANormalized, cplex.getValue(sumDistCostFactorA)));
				pathResultReport.setOrientationSC(new SoftConstraintValues(dirWeight, dirWeightNormalized, cplex.getValue(sumDirCostFactor)));
				pathResultReport.setProportionWeightSC(new SoftConstraintValues(propWeight, propNormalWeight, cplex.getValue(sumEdgeProportionFactor)));
				pathResultReport.setObjectiveFunctionValue(cplex.getObjValue());
				System.out.println("Final = " + cplex.getObjValue());
				
				cplex.end();
				path.updatePathXNodes2( finalPoints );
				end = System.currentTimeMillis();
				System.out.println("Path Scehmatization- Nodes: " + n+ " Execution Time: "   + (end - start) );
				pathResultReport.setExecutionTime((end - start));
				return finalPoints;
				
			}
			else{
				
				throw new Exception("Cannot solve MIP Model on time");
				
			}
			
			
			
			
			
		}


		
	
		public static ArrayList<Point2D> adjpathOptimizer2( Path path, ArrayList<Point2D> transPointList ) throws IloException, Exception   {
			//System.out.println("OPTIMIZER Network PATH - > ACTION!!!!!");	
			ArrayList<Point2D> mipLineString =  new ArrayList<Point2D>();
			
			ArrayList<Point2D> lineString = path.asJava2DList(1);
			
			IloCplex cplex = new IloCplex();
			
			for(int i=0; i < transPointList.size(); i++){
				mipLineString.add(transPointList.get(i));
				//Spath.getNodeList().get(i).setxGeom(GeoConvertionsOperations.Java2DPointToJTSGeometry(transPointList.get(i)));
			}
			path.updatePathXNodes2( mipLineString );
			return mipLineString;
			
			
			
		}
		
		
		public static ArrayList<Point2D> networkpathOptimizer2( Path path, ArrayList<Point2D> transPointList ) throws IloException, Exception   {
			//System.out.println("OPTIMIZER Network PATH - > ACTION!!!!!");	
			ArrayList<Point2D> mipLineString =  new ArrayList<Point2D>();
			
			//ArrayList<Point2D> lineString = path.asJava2DList(1);
			
			//IloCplex cplex = new IloCplex();
			
			for(int i=0; i < transPointList.size(); i++){
//				if(path.getNodeList().get(i).getxGeom() != null)
//					mipLineString.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(path.getNodeList().get(i).getxGeom()));
//				else
					mipLineString.add(transPointList.get(i));
				//Spath.getNodeList().get(i).setxGeom(GeoConvertionsOperations.Java2DPointToJTSGeometry(transPointList.get(i)));
			}
			path.updatePathXNodes2( mipLineString );
			return mipLineString;
			
			
			
		}
		
		
		public static ArrayList<Point2D> networkpathOptimizer(Map<Integer, StreetNode> streetNodeMap,
				Map<Integer, ArrayList<StreetNode>> circularOrderList, Path path , int startEdgeDirection,
				int endEdgeDirection, int bendFactor, int dirFactor, int distFactor,
				int executionTimeLimit) throws IloException, Exception   {
			
			System.out.println("OPTIMIZER Network PATH - > ACTION!!!!!");
			ArrayList<Point2D> mipLineString =  new ArrayList<Point2D>();
		
			ArrayList<Point2D> lineString = path.asJava2DList(1);
			
			IloCplex cplex = new IloCplex();
			
			PointsPolar polarPoints = new PointsPolar();
			polarPoints  = GeometricOperation.toPolar(lineString);
			
			System.out.println(lineString);
			int n = lineString.size();
			int m = n -1; /*number of edges in a paths*/
			int b = m - 1; /*number of bends in a path*/
			/***All points to extend?**/
			//ArrayList<Point2D> allpoints = new ArrayList<Point2D>(lineString);
			//allpoints.addAll(route);
			double maxExtend = GeometricOperation.diagonalOf(lineString);
			double stringLenght = GeometricOperation.length(lineString);
			double distMin = Math.min(stringLenght/4, maxExtend/500); /* Mininal distance between non adcent edges from route, stringLength/4 must be used in case the string is to small*/
			double L = stringLenght/(n*100); /* Minimal distance between two vertices*/
			System.out.println("Minimal distance between two vertices: " + L);
			System.out.println("Mininal distance from route " + distMin);
			System.out.println("Linestring Legth " + stringLenght);
			System.out.println("Max Extend " + maxExtend);
			
			
			double maxD, maxX, minX, maxY, minY, maxZ1, minZ1, maxZ2, minZ2;
			maxX = lineString.get(0).getX();
			minX = lineString.get(0).getX();
			maxY = lineString.get(0).getY();
			minY = lineString.get(0).getY();
			maxZ1 = lineString.get(0).getX() + lineString.get(0).getY();
			minZ1 = maxZ1;
			maxZ2 = lineString.get(0).getX() - lineString.get(0).getY();
			minZ2 = maxZ2;
			maxD = lineString.get(0).distance(lineString.get(1));
			for (int i = 1; i < n; i++) {
				
				double x =lineString.get(i).getX();
				double y =lineString.get(i).getY();
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
				double dist = lineString.get(i).distance(lineString.get(i-1));
				maxD = (dist > maxD)? dist : maxD;
			}
//			minZ1 = (minY + minX);
//			maxZ1 = (maxX + maxY);
//			
//			minZ2 = (minX - maxY);
//			maxZ2 = (maxX - minY);
			
			System.out.println("max X: " + maxX + ", min X: " + minX);
	        System.out.println("max Y: " + maxY + ", min Y: " + minY);
	        System.out.println("max Z1: " + maxZ1 + ", min Z1: " + minZ1);
	        System.out.println("max Z2: " + maxZ2 + ", min Z2: " + minZ2);
			
			double xExtend = Math.max(maxX - minX, distMin*10);
			double yExtend = Math.max(maxY - minY, distMin*10);
			double z1Extend = Math.max(maxZ1 - minZ1, distMin*10) ;
			double z2Extend = Math.max(maxZ2 - minZ2, distMin*10);
			
			System.out.println("xExtend: " + xExtend + ", yExtend: " + yExtend);
			System.out.println("z1Extend: " + z1Extend + ", z2Extend: " + z2Extend);
			
			
			
	        

	        System.out.println("maxD: " + maxD);
	        System.out.println("N: " + n);
	        
	        /*********COORDINATES CONSTRAINT*****************/
			/*think in reducing max values*/
	        double extendLimit = 10;

	        
			IloNumVar[] x  = cplex.numVarArray(n, minX - extendLimit*xExtend, maxX + extendLimit*xExtend);
			IloNumVar[] y  = cplex.numVarArray(n,  minY - extendLimit*yExtend, maxY + extendLimit*yExtend);
			IloNumVar[] z1  = cplex.numVarArray(n,  minZ1 - extendLimit*z1Extend, maxZ1 + extendLimit*z1Extend);
			IloNumVar[] z2  = cplex.numVarArray(n, minZ2 - extendLimit*z2Extend, maxZ2 + extendLimit*z2Extend);
			
			
//			IloNumVar[] x  = cplex.numVarArray(n, -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] y  = cplex.numVarArray(n,  -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] z1  = cplex.numVarArray(n,  -Double.MAX_VALUE, Double.MAX_VALUE);
//			IloNumVar[] z2  = cplex.numVarArray(n, -Double.MAX_VALUE, Double.MAX_VALUE);
			
			IloNumExpr[] z1Constraint = new IloNumExpr[n];
			IloNumExpr[] z2Constraint = new IloNumExpr[n];
			
			for (int i = 0; i < n; i++) {
				/*do we need to multiply by 2?*/
				z1Constraint[i] = cplex.sum(x[i],y[i]);
				z2Constraint[i] = cplex.diff(x[i],y[i]);
				
			}
			
			/*Add z constraint to model constranint*/ 
			for (int i = 0; i < n; i++) {
				cplex.addEq(z1[i], z1Constraint[i]);
				cplex.addEq(z2[i], z2Constraint[i]);
				
			}
			System.out.println("Fixing nodes with x positions");
			for( int i = 0; i < path.getNodeList().size(); i ++){
				if(path.getNodeList().get(i).getxGeom()!=null){
					System.out.println("Point" + i + " " +path.getNodeList().get(i).getxGeom());
					cplex.addEq(x[i], path.getNodeList().get(i).getxGeom().getX());
					cplex.addEq(y[i], path.getNodeList().get(i).getxGeom().getY());
				//	i++;
				}
				
			}
//			/*fix position of first vertex*/
//			cplex.addEq(x[0], lineString.get(0).getX());
//			cplex.addEq(y[0], lineString.get(0).getY());
//			
//			/*fix position of first vertex*/
//			cplex.addEq(x[n-1], lineString.get(n-1).getX());
//			cplex.addEq(y[n-1], lineString.get(n-1).getY());
	        
			
			/*********OCTALINEARITY CONSTRAINT*****************/
			
			/*sec[m][d][(pred,orig,succ)] defines the sector of the octilinear position an edges could lie
			 * m = number of edges
			 * d = direction of the edge 0 is going and 1 is back
			 * (pred,orig,succ) = 0 lies on the previous sector, 1 lies ont the original best sector, and 2 lies on the succecive sector*/ 
			int[][][] sec = new int[m][2][3];		

			for (int i = 0; i < m; i++) {
				if(i == 0 && startEdgeDirection >  -1)
					sec[i][0][1] = startEdgeDirection;

				else if (i == m-1 && endEdgeDirection >  -1)
					sec[i][0][1] = endEdgeDirection;
				else
					sec[i][0][1] = GeometricOperation.sectorOf(polarPoints.getPoints().get(i).getTheta());


				sec[i][0][0] =  Math.floorMod(sec[i][0][1] - 1, 8);
				sec[i][0][2] =  Math.floorMod(sec[i][0][1] + 1, 8);


				sec[i][1][1] = Math.floorMod(sec[i][0][1] + 4,8);
				sec[i][1][0] = Math.floorMod(sec[i][0][0] + 4,8);
				sec[i][1][2] = Math.floorMod(sec[i][0][2] + 4,8);

			}
			
			/*Boolean variable to restrict the orientation of the edge to orig = 1, succ = 2 and prd = 0*/
			IloNumVar[][] alpha = new IloNumVar[m][];
			
			for (int i = 0; i < m; i++) {
				alpha[i] = cplex.boolVarArray(3);
				
			}
	/**force alfa to best direction if start and end edges directions are set **/
			if(startEdgeDirection >  -1){

				cplex.addEq(alpha[0][0], 0);
				cplex.addEq(alpha[0][1], 1);
				cplex.addEq(alpha[0][2], 0);



			}
			if(endEdgeDirection >  -1){

				cplex.addEq(alpha[m-1][0], 0);
				cplex.addEq(alpha[m-1][1], 1);
				cplex.addEq(alpha[m-1][2], 0);



			}


			IloLinearNumExpr[] constraintEdgeOrientation = new IloLinearNumExpr[m];

			for (int i = 0; i < m; i++) {
				constraintEdgeOrientation[i] = cplex.linearNumExpr();
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][0]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][1]);
				constraintEdgeOrientation[i].addTerm(1.0,alpha[i][2]);
				cplex.addEq(constraintEdgeOrientation[i], 1.0);
			}
			
			
			/*Variable dir[i][j] defines the direction of of edege i (j means de drirection of the edege*/
			IloNumVar[][] dir  = new IloNumVar[m][]; /*leave the last dimension to be defined with cplex*/
			
			for (int i = 0; i < m; i++) {
				dir[i] = cplex.intVarArray(2, 0, 7);/* array of size 2 because we need uv an vu*/
				
			}
			
			
			/*For each i ∈ {pred, orig, succ} we have the following set of constraints
			 dir(u, v)−seciu(v) ≤M(1−αi(u, v))
			−dir(u, v)+seciu(v) ≤M(1−αi(u, v))
			dir(v,u)−seciv(u) ≤M(1−αi(u, v))
			−dir(v,u)+seciv(u) ≤M(1−αi(u, v))
			
			∀{u, v} ∈ E,
			
			 *Here, if αi(u, v) = 0, the constraints in (4) are trivially fulfilled and do not influence the left-hand sides. 
			 *On the other hand, if αi(u, v) = 1, the four inequalities are equivalent to dir(u, v) = seciu(v) and
				dir(v,u) = seciv(u) as desired (equality
			 *
			 *
			 *
			 */
			
			double MDir = 8; /* |sec - dir| < 8*/
			for (int i = 0; i < m; i++) {
				
				if(i == 0 && startEdgeDirection >  -1){
					cplex.addEq(dir[i][0],startEdgeDirection);
					cplex.addEq(dir[i][1],Math.floorMod(startEdgeDirection + 4,8));
				}
				else if (i == m-1 && endEdgeDirection >  -1){
					cplex.addEq(dir[i][0],endEdgeDirection);
					cplex.addEq(dir[i][1],Math.floorMod(endEdgeDirection + 4,8));
				}
				else{
					for (int j = 0; j < 3; j++) {
						IloNumExpr rightSide = cplex.prod(MDir, cplex.diff(1, alpha[i][j]));
						/*natural direction of the edge d=0*/
						IloNumExpr leftSide1 = cplex.diff(dir[i][0], sec[i][0][j]);
						IloNumExpr leftSide2 = cplex.diff(sec[i][0][j],  dir[i][0]);
						/*counter direction of the edge d= 1*/
						IloNumExpr leftSide3 = cplex.diff(dir[i][1], sec[i][1][j]);
						IloNumExpr leftSide4 = cplex.diff(sec[i][1][j],  dir[i][1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						cplex.addLe(leftSide3, rightSide);
						cplex.addLe(leftSide4, rightSide);
					
					}
				}
				
			
			}
//			if(startEdgeDirection >  -1){
//				cplex.addEq(dir[0][0],startEdgeDirection);
//			}
//			if(endEdgeDirection >  -1){
//				cplex.addEq(dir[m-1][0],endEdgeDirection);
//			}
			/**contraints to force the correct position of the vertices
			 * if sec original is 2 and alphaoriginal is true then forces x(u) and x(v) to equal
			 * and y(v) > y(u)*/
			double MAdjVertice = 2*maxD; /* D max distance between two adjacent vertices vertices*/
			
			for (int i = 0; i < m; i++) {
				
				for (int j = 0; j < 3; j++) {
					IloNumExpr rightSide, rightSide2, leftSide1, leftSide2, leftSide3;
					/*do i need to constraint edges for both directions(sec[m][1])?*/
					switch (sec[i][0][j]) {
					case 0:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(y[i], y[i+1]);
						leftSide2 = cplex.diff(y[i+1],  y[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(x[i+1],  x[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
					
						break;
					case 7:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[i], z2[i+1]);
						leftSide2 = cplex.diff(z2[i+1],  z2[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z1[i+1],  z1[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;
						
					case 6:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[i], x[i+1]);
						leftSide2 = cplex.diff(x[i+1],  x[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(y[i+1],  y[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);

						break;
						
					case 5:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[i], z1[i+1]);
						leftSide2 = cplex.diff(z1[i+1],  z1[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z2[i],  z2[i + 1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					case 4:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(y[i], y[i+1]);
						leftSide2 = cplex.diff(y[i+1],  y[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(x[i],  x[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;
					case 3:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z2[i], z2[i+1]);
						leftSide2 = cplex.diff(z2[i+1],  z2[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z1[i],  z1[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					case 2:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(x[i], x[i+1]);
						leftSide2 = cplex.diff(x[i+1],  x[i]);
						
						rightSide2 = cplex.diff(L, rightSide);
						leftSide3 = cplex.diff(y[i],  y[i+1]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);

						break;	
					case 1:
						rightSide = cplex.prod(MAdjVertice, cplex.diff(1, alpha[i][j]));
						leftSide1 = cplex.diff(z1[i], z1[i+1]);
						leftSide2 = cplex.diff(z1[i+1],  z1[i]);
						
						rightSide2 = cplex.diff(2*L, rightSide);
						leftSide3 = cplex.diff(z2[i+1],  z2[i]);
						
						cplex.addLe(leftSide1, rightSide);
						cplex.addLe(leftSide2, rightSide);
						
						cplex.addGe(leftSide3, rightSide2);
						break;	
					default:
						break;
					}
					
				}
				
			}	
			
			/*********BEND MINIMIZATION*****************/

			
			
			
			/*Boolean variable to make condition of bends angle <-5, >-4 e <4, ou >5 measure bends angles (-7 to 7)*/
			IloNumVar[][] delta = new IloNumVar[b][];
			
			for (int i = 0; i < b; i++) {
				delta[i] = cplex.boolVarArray(3);
				
			}
			IloLinearNumExpr[] constraintBenMinimizationBooleans = new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				constraintBenMinimizationBooleans[i] = cplex.linearNumExpr();
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][0]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][1]);
				constraintBenMinimizationBooleans[i].addTerm(1.0,delta[i][2]);
				cplex.addEq(constraintBenMinimizationBooleans[i], 2.0);
			}
			double MBend = 15; /*diference between 2 dir */
			IloNumVar[] bendDir = cplex.intVarArray(b, -7, 7);
			/* bendDir[1] = dir[0] - dir[1]*/

			for (int i = 0; i < b; i++) {
				cplex.addEq(bendDir[i], cplex.diff(dir[i][0], dir[i + 1][0]));
			}
			
			for (int i = 0; i < b; i++) {
				cplex.addLe(bendDir[i], cplex.diff(cplex.prod(MBend, delta[i][0]), 5));
				cplex.addGe(bendDir[i], cplex.diff(5 , cplex.prod(MBend, delta[i][1])));
				cplex.addLe(bendDir[i], cplex.sum(4 , cplex.prod(MBend, delta[i][2])));
				cplex.addGe(bendDir[i], cplex.diff(-4 , cplex.prod(MBend, delta[i][2])));
				
			}
			
			IloNumVar[] bendCost = cplex.intVarArray(b, 0, 3);
			IloLinearNumExpr[] bendCostConstraint =  new IloLinearNumExpr[b];
			for (int i = 0; i < b; i++) {
				bendCostConstraint[i] = cplex.linearNumExpr();
				bendCostConstraint[i].addTerm(1, bendDir[i]);
				bendCostConstraint[i].addTerm(-8, delta[i][0]);
				bendCostConstraint[i].addTerm(8, delta[i][1]);
				cplex.addGe(bendCostConstraint[i], cplex.prod(-1, bendCost[i]) );
				cplex.addLe(bendCostConstraint[i], bendCost[i] );
				
				//cplex.addLe(bendDir[i], bendCost[i]);
				//cplex.addGe(bendDir[i], cplex.prod(-1,bendCost[i]));
				
			}
			float factor = (1/(float)n);
			//System.out.println("factor= " + factor);
			IloNumExpr sumBendCost =  cplex.sum(bendCost);
			IloNumExpr sumBendCostNormal = cplex.prod((bendFactor),sumBendCost);
			
			/*********Distance Minization*****************/

			/*distance reduction*/
			IloNumVar[] dX  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			IloNumVar[] dY  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			//IloNumVar[] dMax  = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			

			
			for (int i = 0; i < n; i++) {
				IloNumExpr xLeftSide = cplex.diff(lineString.get(i).getX(), x[i]);
				IloNumExpr yLeftSide = cplex.diff(lineString.get(i).getY(), y[i]);
				
				cplex.addLe(xLeftSide, dX[i]);
				cplex.addGe(xLeftSide, cplex.prod(-1, dX[i]));
				
				
				cplex.addLe(yLeftSide, dY[i]);
				cplex.addGe(yLeftSide, cplex.prod(-1, dY[i]));
				
			}
			
			
			
			IloNumExpr sumXDist =  cplex.sum(dX);
			IloNumExpr sumYDist =  cplex.sum(dY);
			
			IloNumExpr sumDistCost = cplex.prod((1000*distFactor)/(stringLenght*n), cplex.sum(sumXDist, sumYDist ));			
			
			
			/*********OBJECTIVE*****************/
			IloNumExpr objective = cplex.sum(  sumDistCost, sumBendCostNormal) ;
			
			
			//IloNumExpr objective = cplex.constant(0);
			
			cplex.addMinimize(objective);
			if(executionTimeLimit > 0 ){
				double limitTimeInSeconds = ((double)(executionTimeLimit))/1000;
				cplex.setParam(IloCplex.DoubleParam.TiLim, limitTimeInSeconds);
			}
			else if(executionTimeLimit <0 ){
				cplex.setParam(IloCplex.Param.MIP.Limits.Solutions, 1);
			}
			
			/*control display information*/
			cplex.setParam(IloCplex.Param.MIP.Display, 1);
			
			if(cplex.solve()){
				System.out.println("obj =  " +cplex.getObjValue());
				System.out.println("Start: " + startEdgeDirection + "  End: " + endEdgeDirection);
				for (int i = 0; i < n; i++) {
					
					//System.out.println("x"+ i + " =  " + cplex.getValue(x[i]) + " y"+ i + " =  " + cplex.getValue(y[i]) + " z1"+ i + " =  " + cplex.getValue(z1[i]) + " z2"+ i + " =  " + cplex.getValue(z2[i]));
					
					//System.out.println("dx"+ i + " =  " + cplex.getValue(dX[i]) + " dy"+ i + " =  " + cplex.getValue(dY[i]));
					if(i > 0){
						//System.out.println("Edge " + (i -1));
						//System.out.println("sec0-> =  " + sec[i-1][0][0] + " sec1-> =  " + sec[i-1][0][1] + " sec2-> =  " + sec[i-1][0][2]);
						//System.out.println("sec0<- =  " + sec[i-1][1][0] + " sec1<- =  " + sec[i-1][1][1] + " sec2-< =  " + sec[i-1][1][2]);
						
						//System.out.println("dir-> =  " + cplex.getValue(dir[i-1][0]) + " dir<- =  " + cplex.getValue(dir[i-1][1]));
						if(i>1){
							//System.out.println("benDir-> =  " + cplex.getValue(bendDir[i-2]) + " bencost-> =  " + cplex.getValue(bendCost[i-2]) );
							//System.out.println("gama0-> =  " + cplex.getValue(gama[i-2][0]) + " gama1-> =  " + cplex.getValue(gama[i-2][1])  + " gama2-> =  " + cplex.getValue(gama[i-2][2]));
						}	
						//System.out.println();
					}
						
					Point2D.Double pt = new Point2D.Double(cplex.getValue(x[i]),cplex.getValue(y[i])); 
					mipLineString.add(pt);
				}
				path.updatePathXNodes( mipLineString);
				
				//System.out.println("sumDx =  " + cplex.getValue(sumXDist) + " sumDy =  " + cplex.getValue(sumYDist));
				System.out.println("DistCost =  " + cplex.getValue(sumDistCost) + " sumBendCost = " + cplex.getValue(sumBendCost) + " sumBendCostNormal = " + cplex.getValue(sumBendCostNormal) + " Final = " + cplex.getObjValue());
				cplex.end();
				return mipLineString;
				
			}
			else{
				
				throw new Exception("Cannot solve MIP Model");
				
			}
			

		}

		public static double getPathProportion(Path path, StreetNetworkTopological streetNetwork, double maxExtend) {
			
			double sumLenghtOriginal = 0;
			double sumLenghtX = 0;
			
			LineString test = path.asLineString(1);
			
			MinimumBoundingCircle pathBoundingCircle = new MinimumBoundingCircle(test);

			Coordinate center = pathBoundingCircle.getCentre();

				
			for(StreetEdge edge: streetNetwork.getEdges().values()){
							
				Point sourceXPoint = edge.getSourcePoint().getxGeom();
				Point targetXPoint = edge.getTargetPoint().getxGeom();
				
				if((sourceXPoint != null && targetXPoint != null) && (!edge.isFakeEdge()) ){
					//System.out.println("Xedge: " + edge.getName());
					Point sourceOrigPoint = edge.getSourcePoint().getProjectGeom();
					Point targetOrigPoint = edge.getTargetPoint().getProjectGeom();
					double dist = (center.distance(sourceOrigPoint.getCoordinate()) + center.distance(targetOrigPoint.getCoordinate()))/2;
					double normalDist = Math.pow(100*dist/maxExtend, 1);
						
					if(normalDist < 30) {
						double distOrig = sourceOrigPoint.distance(targetOrigPoint);
						double distX = sourceXPoint.distance(targetXPoint);
						sumLenghtOriginal += distOrig/normalDist;
						sumLenghtX += distX/normalDist;
						//System.out.println("is inside : ");
					
					}
					
				}
				
				
				
			}
			if(Double.isNaN(sumLenghtX/sumLenghtOriginal))
				return 1;
			else
				return sumLenghtX/sumLenghtOriginal;
		}
		
		
		
		public static double getPathProportion3(Path path, StreetNetworkTopological streetNetwork, double maxExtend) {
			
			double sumLenghtOriginal = 0;
			double sumLenghtX = 0;
			
			LineString pathLineString = path.asLineString(1);
			ArrayList<StreetEdge> xEdges = new ArrayList<StreetEdge>();
			ArrayList<Double> xEdgeDistanceToPath = new ArrayList<Double>();
			ArrayList<Double> edgesWeight = new ArrayList<Double>();

			

				
			for(StreetEdge edge: streetNetwork.getEdges().values()){
				if( !edge.isFakeEdge() ){
					Point sourceXPoint = edge.getSourcePoint().getxGeom();
					Point targetXPoint = edge.getTargetPoint().getxGeom();
					if((sourceXPoint != null && targetXPoint != null) ){
						Point sourceOrigPoint = edge.getSourcePoint().getProjectGeom();
						Point targetOrigPoint = edge.getTargetPoint().getProjectGeom();
						double dist = (sourceOrigPoint.distance(pathLineString) + targetOrigPoint.distance(pathLineString))/2;
						
						if(dist != 0) {
							//System.out.println("dist to path: " + dist);
							xEdges.add(edge);						
							xEdgeDistanceToPath.add(dist);
							if(edge instanceof RouteEdge)
								edgesWeight.add(10.0);
							else if (edge.getIsPolygonEdge()>0)
								edgesWeight.add(5.0);
							else
								edgesWeight.add(1.0);
						}
						else
							System.out.println("dist to path is fucking 0: " + dist);
						
					}
				
//					if(edge instanceof RouteEdge) {
//						
//						
//	
//					}
				}
				
				
				
			}
			
			for(int i=0; i < xEdges.size(); i++) {
				Point sourceOrigPoint = xEdges.get(i).getSourcePoint().getProjectGeom();
				Point targetOrigPoint = xEdges.get(i).getTargetPoint().getProjectGeom();
				Point sourceXPoint = xEdges.get(i).getSourcePoint().getxGeom();
				Point targetXPoint = xEdges.get(i).getTargetPoint().getxGeom();
				
				double normalDist = Math.pow(100*xEdgeDistanceToPath.get(i)/maxExtend, 1.0);
//				if(normalDist < 1)
//					System.out.println("normal dist: " + normalDist);
				if(normalDist < 100) {		
					double distOrig = sourceOrigPoint.distance(targetOrigPoint);
					double distX = sourceXPoint.distance(targetXPoint);
					sumLenghtOriginal += edgesWeight.get(i)*(distOrig/normalDist);
					sumLenghtX += edgesWeight.get(i)*(distX/normalDist);
					//System.out.println("is inside : ");
				}
				
				
			}
			if(Double.isNaN(sumLenghtX/sumLenghtOriginal))
				return 1;
			else
				return sumLenghtX/sumLenghtOriginal;
		}
		
		public static double getPathProportion2(Path path, Route route, double maxExtend) {
			
			double sumLenghtOriginal = 0;
			double sumLenghtX = 0;
			ArrayList<StreetEdge> xEdges = new ArrayList<StreetEdge>();
			
			MinimumBoundingCircle pathBoundingCircle = new MinimumBoundingCircle(path.asLineString(1));
			Coordinate center = pathBoundingCircle.getCentre();
			//Coordinate center = path.getNodeList().get(1).getProjectGeom().getCoordinate();
			for(RouteEdge edge: route.getEdgeList()){
							
				Point sourceXPoint = edge.getSourcePoint().getxGeom();
				Point targetXPoint = edge.getTargetPoint().getxGeom();
				
				if((sourceXPoint != null && targetXPoint != null)  ){
					//System.out.println("Xedge: " + edge.getName());
					xEdges.add(edge);

				}
				
				
				
				
			}
			xEdges.sort(new StreetEdgeDistanceToPointComparator(center));
			for(int i = 0; i < Math.round((double)(xEdges.size())/4); i++) {
				Point sourceOrigPoint = xEdges.get(i).getSourcePoint().getProjectGeom();
				Point targetOrigPoint = xEdges.get(i).getTargetPoint().getProjectGeom();
				Point sourceXPoint = xEdges.get(i).getSourcePoint().getxGeom();
				Point targetXPoint = xEdges.get(i).getTargetPoint().getxGeom();
				
				double dist = (center.distance(sourceOrigPoint.getCoordinate()) + center.distance(targetOrigPoint.getCoordinate()))/2;
				double normalDist = Math.pow(100*dist/maxExtend, 1.0);
				System.out.println("normal dist: " + normalDist);
				
				//if (pathBoundingCircle.getCircle().buffer(pathBoundingCircle.getRadius()).contains(sourceOrigPoint) || pathBoundingCircle.getCircle().buffer(pathBoundingCircle.getRadius()/2).contains(targetOrigPoint) ){
				if(normalDist < 20) {		
					double distOrig = sourceOrigPoint.distance(targetOrigPoint);
					double distX = sourceXPoint.distance(targetXPoint);
					sumLenghtOriginal += distOrig/normalDist;
					sumLenghtX += distX/normalDist;
					//System.out.println("is inside : ");
				}
							
			}
			if(Double.isNaN(sumLenghtX/sumLenghtOriginal))
				return 1;
			else
				return sumLenghtX/sumLenghtOriginal;
		}
		
		/***get the proportion of the route XEdges length compared to the original project route edges length*/
		public static double getPathProportion(Path path, Route route, double maxExtend) {
			
			double sumLenghtOriginal = 0;
			double sumLenghtX = 0;
			
			MinimumBoundingCircle pathBoundingCircle = new MinimumBoundingCircle(path.asLineString(1));
			Coordinate center = pathBoundingCircle.getCentre();
			/*get the proportion of the route XEdges length compared to the original project route edges length*/
			for(RouteEdge edge: route.getEdgeList()){
				
				
				Point sourceXPoint = edge.getSourcePoint().getxGeom();
				Point targetXPoint = edge.getTargetPoint().getxGeom();
				
				if((sourceXPoint != null && targetXPoint != null)  ){
					//System.out.println("Xedge: " + edge.getName());
					Point sourceOrigPoint = edge.getSourcePoint().getProjectGeom();
					Point targetOrigPoint = edge.getTargetPoint().getProjectGeom();
					
					double dist = (center.distance(sourceOrigPoint.getCoordinate()) + center.distance(targetOrigPoint.getCoordinate()))/2;
					double normalDist = Math.pow(100*dist/maxExtend, 1.0);
					//System.out.println("normal dist: " + normalDist);
					
					//if (pathBoundingCircle.getCircle().buffer(pathBoundingCircle.getRadius()).contains(sourceOrigPoint) || pathBoundingCircle.getCircle().buffer(pathBoundingCircle.getRadius()/2).contains(targetOrigPoint) ){
					if(normalDist < 20) {	
						
						double distOrig = sourceOrigPoint.distance(targetOrigPoint);
						double distX = sourceXPoint.distance(targetXPoint);
						sumLenghtOriginal += distOrig/normalDist;
						sumLenghtX += distX/normalDist;
						//System.out.println("is inside : ");
					}
					//}
					
					
				}
				
				
				
			}
			if(Double.isNaN(sumLenghtX/sumLenghtOriginal))
				return 1;
			else
				return sumLenghtX/sumLenghtOriginal;
		}
		
		
		/***get the proportion of the route XEdges length compared to the original project route edges length*/
		public static double getControlEdgeProportion(Path path, Route route, double maxExtend) {
			
			double sumLenghtOriginal = 0;
			double sumLenghtX = 0;
			
			MinimumBoundingCircle pathBoundingCircle = new MinimumBoundingCircle(path.asLineString(1));
			Coordinate center = pathBoundingCircle.getCentre();
			/*get the proportion of the route XEdges length compared to the original project route edges length*/
			for(RouteEdge edge: route.getEdgeList()){
				
				
				Point sourceXPoint = edge.getSourcePoint().getxGeom();
				Point targetXPoint = edge.getTargetPoint().getxGeom();
				
				if((sourceXPoint != null && targetXPoint != null)  ){
					//System.out.println("Xedge: " + edge.getName());
					Point sourceOrigPoint = edge.getSourcePoint().getProjectGeom();
					Point targetOrigPoint = edge.getTargetPoint().getProjectGeom();
					
					double dist = (center.distance(sourceOrigPoint.getCoordinate()) + center.distance(targetOrigPoint.getCoordinate()))/2;
					double normalDist = Math.pow(100*dist/maxExtend, 0.5);
					//System.out.println("normal dist: " + normalDist);
					
					//if (pathBoundingCircle.getCircle().buffer(pathBoundingCircle.getRadius()).contains(sourceOrigPoint) || pathBoundingCircle.getCircle().buffer(pathBoundingCircle.getRadius()/2).contains(targetOrigPoint) ){
					if(normalDist < 80) {	
						
						double distOrig = sourceOrigPoint.distance(targetOrigPoint);
						double distX = sourceXPoint.distance(targetXPoint);
						sumLenghtOriginal += distOrig/normalDist;
						sumLenghtX += distX/normalDist;
						//System.out.println("is inside : ");
					}
					//}
					
					
				}
				
				
				
			}
			if(Double.isNaN(sumLenghtX/sumLenghtOriginal))
				return 1;
			else
				return sumLenghtX/sumLenghtOriginal;
		}
		
		public static double getPathProportion3(Path path, Route route, double maxExtend) {
			
			double sumLenghtOriginal = 0;
			double sumLenghtX = 0;
			
			LineString pathLineString = path.asLineString(1);
			ArrayList<StreetEdge> xEdges = new ArrayList<StreetEdge>();
			ArrayList<Double> xEdgeDistanceToPath = new ArrayList<Double>();

			

				
			for(RouteEdge edge: route.getEdgeList()){

					Point sourceXPoint = edge.getSourcePoint().getxGeom();
					Point targetXPoint = edge.getTargetPoint().getxGeom();
					if((sourceXPoint != null && targetXPoint != null) ){
						Point sourceOrigPoint = edge.getSourcePoint().getProjectGeom();
						Point targetOrigPoint = edge.getTargetPoint().getProjectGeom();
						double dist = (sourceOrigPoint.distance(pathLineString) + targetOrigPoint.distance(pathLineString))/2;
						
						if(dist != 0) {
							//System.out.println("dist to path: " + dist);
							xEdges.add(edge);						
							xEdgeDistanceToPath.add(dist);

						}
						else
							System.out.println("dist to path is fucking 0: " + dist);
						
					}
	
				
			}
			
			for(int i=0; i < xEdges.size(); i++) {
				Point sourceOrigPoint = xEdges.get(i).getSourcePoint().getProjectGeom();
				Point targetOrigPoint = xEdges.get(i).getTargetPoint().getProjectGeom();
				Point sourceXPoint = xEdges.get(i).getSourcePoint().getxGeom();
				Point targetXPoint = xEdges.get(i).getTargetPoint().getxGeom();
				
				double normalDist = Math.pow(100*xEdgeDistanceToPath.get(i)/maxExtend, 1.0);
				if(normalDist < 1)
					System.out.println("normal dist: " + normalDist);
				if(normalDist < 40) {		
					double distOrig = sourceOrigPoint.distance(targetOrigPoint);
					double distX = sourceXPoint.distance(targetXPoint);
					sumLenghtOriginal += (distOrig/normalDist);
					sumLenghtX += (distX/normalDist);
					//System.out.println("is inside : ");
				}
				
				
			}
			if(Double.isNaN(sumLenghtX/sumLenghtOriginal))
				return 1;
			else
				return sumLenghtX/sumLenghtOriginal;
		}
		
		
		


}

