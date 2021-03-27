package com.wayto.controller;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Stack;

import org.codehaus.jettison.json.JSONArray;
import org.codehaus.jettison.json.JSONException;
import org.codehaus.jettison.json.JSONObject;
import org.geotools.data.DataUtilities;
import org.geotools.feature.SchemaException;
import org.geotools.geometry.jts.Geometries;
import org.opengis.feature.simple.SimpleFeatureType;

import com.goebl.simplify.PointExtractor;
import com.goebl.simplify.Simplify;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;

import com.wayto.dao.PointLikeDAO;
import com.wayto.dao.PolygonDAO;
import com.wayto.dao.RouteDAO;
import com.wayto.model.LinearFeature;
import com.wayto.model.Path;
import com.wayto.model.PointFeature;
import com.wayto.model.PolygonalFeature;
import com.wayto.model.RoundAboutNode;
import com.wayto.model.Route;
import com.wayto.model.RouteEdge;
import com.wayto.model.StreetEdge;
import com.wayto.model.StreetNetworkTopological;
import com.wayto.model.StreetNode;
import com.wayto.model.topo.NodeTopoRelation;
import com.wayto.model.topo.PointTopo;
import com.wayto.model.topo.PolygonalTopo;

import com.wayto.operator.FisheyeTransform;
import com.wayto.operator.GeoConvertionsOperations;
import com.wayto.operator.GeometricOperation;
import com.wayto.operator.PointsPolar;

import com.wayto.operator.TopoOperator3;




public class RouteDataController {

	private Map<Integer,StreetNode> streetNodeMap;
	private Map<Integer,RoundAboutNode> rbNodeMap;
	private Map<Integer,ArrayList<StreetNode>> adjacencyList;
	private Map<Integer,ArrayList<StreetNode>> circularOrderList;
	private ArrayList<Path> pathList;
	private ArrayList<Path> streetOnlyPathList;
	
	private ArrayList<Path> routeAdjPathList;
	private ArrayList<Path> coEgedPathList;

	private Route route;
	private LineString rescaledRouteGeom;
	private StreetNetworkTopological streetNetwork;
	private ArrayList<PolygonalFeature> polygonalFeatureList;
	private ArrayList<LinearFeature> linearFeatureList;
	private ArrayList<PointFeature> pointFeatureList;
	  
	//private SimpleFeatureType lineStringType;	

	private Envelope networkEnvelop;
	private double maxRouteProjectedEnvExtention;
	private double routeRescaleLengthProportion;
	
	
	private ArrayList<PolygonalTopo> polygonalTopoList;
	private ArrayList<PointTopo> pointTopoList;
	

	
	Simplify<Point2D> simplifier = new Simplify<Point2D>(new Point2D[0], point2DPointExtractor);
	
	private static PointExtractor<Point2D> point2DPointExtractor = new PointExtractor<Point2D>() {
	    public double getX(Point2D point) {
	        return point.getX();
	    }

	    public double getY(Point2D point) {
	        return point.getY();
	    }
	};

	
	public Boolean loadRouteData(double x1, double y1, double x2, double y2, String startName, String endName, double rescaleFactor, double rescaleRaiseFactor, float simplificationPolygon, float simplificationNetwork, int networkLevel, boolean reduceStubs, int rescaleNodeSelection) {
 
		long start, end;
		
//		try {
//			lineStringType  = DataUtilities.createType("Location",
//			         "linestring:LineString:srid=4326," + // <- the geometry attribute: Point type
//			         "name:String," +   // <- a String attribute
//			         "number:Integer"   // a number attribute
//			 );
//		} catch (SchemaException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} 
		streetNodeMap = new HashMap<Integer, StreetNode>();
		rbNodeMap = new HashMap<Integer, RoundAboutNode>();
		adjacencyList = new HashMap<Integer, ArrayList<StreetNode>>();
		circularOrderList = new HashMap<Integer, ArrayList<StreetNode>>();
		routeAdjPathList = new ArrayList<Path>();
		
		polygonalFeatureList = new ArrayList<PolygonalFeature>();
		pointFeatureList = new ArrayList<PointFeature>();
		linearFeatureList = new ArrayList<LinearFeature>();

		start = System.currentTimeMillis(); 
		
		
		streetNetwork = new StreetNetworkTopological();
		route = RouteDAO.getInstance().getRouteRBCollapse(x1, y1, x2, y2, streetNodeMap, rbNodeMap, adjacencyList, streetNetwork);
		route.setStartName(startName);
		route.setEndName(endName);

		
	
		//route.setGeom(route.getRoutePath().asLineString(0));
		
		
		switch (networkLevel) {
		case 0:
			RouteDAO.getInstance().getStreetNetWork(route, 0.002, streetNodeMap, rbNodeMap, adjacencyList, streetNetwork, false);
			break;
		case 1:
			RouteDAO.getInstance().getStreetNetWork(route, 0.002, streetNodeMap, rbNodeMap, adjacencyList, streetNetwork, true);
			break;
		case 2:
			RouteDAO.getInstance().getStreetNetWork2(route, 0.002, streetNodeMap, rbNodeMap, adjacencyList, streetNetwork, false);
			break;
		case 3:
			RouteDAO.getInstance().getStreetNetWork2(route, 0.002, streetNodeMap, rbNodeMap, adjacencyList, streetNetwork, true);
			break;
		case 4:
			RouteDAO.getInstance().getStreetNetWorkStubsOnly(route, 0.002, streetNodeMap, rbNodeMap, adjacencyList, streetNetwork);
			break;
		case 5:
			RouteDAO.getInstance().getStreetNetWorkSimple(route, 0.002, streetNodeMap, rbNodeMap, adjacencyList, streetNetwork);
			break;	
		case 6:
			RouteDAO.getInstance().getStreetNetWorkPreDefined(route, 0.002, streetNodeMap, rbNodeMap, adjacencyList, streetNetwork, false);
			break;		
		case 7:
			RouteDAO.getInstance().getStreetNetWorkPreDefined2(route, 0.002, streetNodeMap, rbNodeMap, adjacencyList, streetNetwork, false);
			break;	
			
		default:
			break;
		}

		for(int nodeID: adjacencyList.keySet()){
			//adjacencyList.get(nodeID).sort(StreetNode.NodePriorityComparator);
			streetNodeMap.get(nodeID).setDegree(adjacencyList.get(nodeID).size());
			
		}
		
		/*We setstub edges when I load the networks*/
		//setStubEdges();
		/**Refine edges add edges gemotry nodes to the topology (streetnetwork)*/
		TopoOperator3.refineEdges(route, streetNodeMap, adjacencyList, streetNetwork);
		//symplifyPolygons(polygonalFeatureList, 9);
		
		LineString RouteTargetLS = route.getRoutePath().asLineString(0);
		System.out.println("Adj Edges analylis min length: " + RouteTargetLS.getLength()/100);
		double adjEdgesMinLenght = Math.max(4.5601354442756007E-4/2, RouteTargetLS.getLength()/100);
		adjEdgesMinLenght = adjEdgesMinLenght*2;
		System.out.println("Adj Edges analylis min length: " + adjEdgesMinLenght);
		TopoOperator3.breakStubEdges(streetNodeMap, adjacencyList, streetNetwork, adjEdgesMinLenght);
		
		end = System.currentTimeMillis();
		System.out.println("load list time: " + (end - start) );
		
		
		/***Load Landmarks***/
		//polygonalFeatureList = PolygonDAO.getInstance().getBoundaryList(route, 0.003);
		//pointFeatureList = PointLikeDAO.getInstance().getPointLMList(route, 0.04);
		

		
		polygonalTopoList = new ArrayList<PolygonalTopo>();
		pointTopoList = new ArrayList<PointTopo>();
		TopoOperator3.makePolygonsTopo(polygonalFeatureList, polygonalTopoList, streetNodeMap, adjacencyList, streetNetwork);
		TopoOperator3.tellMeMore2(route, streetNodeMap, adjacencyList, streetNetwork, polygonalTopoList );
		
		TopoOperator3.makePointLMTopo(pointFeatureList, pointTopoList, route, streetNodeMap, adjacencyList, streetNetwork);

		
		
		for(int nodeID: adjacencyList.keySet()){
			adjacencyList.get(nodeID).sort(StreetNode.NodePriorityComparator);
			streetNodeMap.get(nodeID).setDegree(adjacencyList.get(nodeID).size());
			
		}
		
		/**Calculate and Update Decision Points*/
		anlyseRoute();
		
		
		pathList = createTrackListLandMarkPreference2();
		//pathList = createTrackList2();
		pathList.addAll(coEgedPathList);
		isolateRoutePath(pathList);

		landmarkMarkPathAanlyses(pathList);
		pathList.get(0).setRoute(true);
		
		//route.setRoutePath(pathList.get(0));
		
		removeDisconnectNodes();
		
	
		
		chunckPahtAnalyses(pathList);
		if(reduceStubs)
			reduceChuckPaths();
		

		/**Add extra vertices to edges of paths (landmark paths too?) adjacent to the route: new stub is edge min lenght is adjEdgesMinLenght**/
		TopoOperator3.adjacentEdgesAnalyses(pathList ,streetNodeMap, adjacencyList, streetNetwork, polygonalTopoList, adjEdgesMinLenght);
		for(int nodeID: adjacencyList.keySet()) {
			//int k = adjacencyList.get(nodeID).size();
			//int snId = streetNodeMap.get(nodeID).getId();
			streetNodeMap.get(nodeID).setDegree(adjacencyList.get(nodeID).size());
		}
		
		/**Break paths starting in the route and create path list only with stubs*/
		//separateStubsFromPaths();
		
		/***This is removing nodes that are pointlike landmark control edges??*/
		/**Remove Residuals (extras nodes and edges from street paths that are chunck) **/
		//chunckPahtResidualRemoval();
		
		
		for(int nodeID: adjacencyList.keySet()) {
			//int k = adjacencyList.get(nodeID).size();
			//int snId = streetNodeMap.get(nodeID).getId();
			streetNodeMap.get(nodeID).setDegree(adjacencyList.get(nodeID).size());
		}
		
		
		//float routeg = 5.5f; /* any number 0-10*/
		float routeg = 7.5f; /* any number 0-10*/
		Envelope routeEnvelpe = route.getRoutePath().asLineString(0).getEnvelopeInternal();
		double diagonal = Math.sqrt( Math.pow(routeEnvelpe.getHeight(), 2)  +  Math.pow(routeEnvelpe.getWidth(), 2));
		double routetolerance = diagonal*(Math.pow((routeg-1)/10, 3))/(80/(routeg));
		//getRouteRelevantPointsOldDelete2();
		setRouteRelevantNodes();

		/**Path Relevante index: Use for street Only**/
		for(int i = 1; i < pathList.size(); i++) {
			if(pathList.get(i).getIsPolygon() > 0 ) {
				float g = simplificationPolygon; /* any number 0-10*/
				//Envelope sEnv = pathList.get(i).asLineString(0).getEnvelopeInternal();
				//double pathLength = pathList.get(i).asLineString(0).getLength();
				//double diagonal = Math.sqrt( Math.pow(sEnv.getHeight(), 2)  +  Math.pow(sEnv.getWidth(), 2));
				double area = 0;
				double perimeter = 0;
				double abstractionLevel = 1;
				for(PolygonalTopo pt: polygonalTopoList) {
					if(pt.getPolygonalFeature().getId() ==  pathList.get(i).getIsPolygon()) {
						Polygon polyGeom = pt.asPolygon(0);
						area = polyGeom.getArea();
						perimeter = polyGeom.getLength();
						abstractionLevel = pt.getPolygonalFeature().getLevelAbstraction();
						break;
					}
				}
				
				//double tolerance = Math.pow(pathLength,0.9)*(Math.pow( (g-1)/10 , 3 ) )/(40/(g));
				
				//double tolerance = Math.pow(perimeter,0.60)* ( Math.pow((g-1)/10,2)/(10/g));
				
				//double tolerance =   (Math.pow(RouteTargetLS.getLength(), 0.2 )/8)*g* Math.pow(perimeter,0.61)/(13);
				
				double tolerance =   (Math.pow(RouteTargetLS.getLength(), 0.2 )/5)*g*(abstractionLevel/3)*Math.pow(perimeter,0.5)/(6);
				
						//* Math.pow((g-1)/10, 3)/20 ;
				
				setPathRelevantPoints(pathList.get(i), tolerance, perimeter/10 , true);
				
//				ArrayList<Integer> revPt = getPathRelevantPoints(pathList.get(i), tolerance, diagonal/6, true);
//				pathList.get(i).setRelevantPointIndex(revPt);
			}
			else {
				float g = simplificationNetwork; /* any number 0-10*/
				//Envelope sEnv = pathList.get(i).asLineString(0).getEnvelopeInternal();
				double pathLength = 0;
				if(pathList.get(i).getNodeList().size() > 1) {
					pathLength = pathList.get(i).asLineString(0).getLength();
				}
				else {
					System.out.println("Strange path with only one node");
				}
				//double diagonal = Math.sqrt( Math.pow(sEnv.getHeight(), 2)  +  Math.pow(sEnv.getWidth(), 2));
//					double tolerance = Math.pow(pathLength,0.8)*(Math.pow((g-1)/10, 3))/(80/(g));
					//tolerance = Math.max(50, tolerance);
					double tolerance = g*15;
					//System.out.println("Tolerance: " + tolerance);
					setPathRelevantPoints(pathList.get(i), g, pathLength/3, true);
					
			}
			
		}
		
		streetOnlyPathList = createTrackListStreetOnly();
		
		/**Normalizing Geometry***/
		Envelope routeTargetEnvelop = RouteTargetLS.getEnvelopeInternal();
		maxRouteProjectedEnvExtention = Math.max(routeTargetEnvelop.getHeight(), routeTargetEnvelop.getWidth());
		for(StreetNode n: streetNodeMap.values()){
			n.setProjectGeom(GeoConvertionsOperations.normalizeJTSPoint2(n.getGeom(), routeTargetEnvelop));
			
		}
		


		networkEnvelop = setNetworkEnvelop();
		circularOrderList.putAll(adjacencyList);
		for(int nodeID: circularOrderList.keySet()){
			
			Collections.sort(circularOrderList.get(nodeID), new NodeCircularOrderComparator(streetNodeMap.get(nodeID)));
		}
		//LineString rescaledRoute ;
		if(rescaleFactor > 5) {
			switch (rescaleNodeSelection) {
			case 0:
				rescaledRouteGeom = GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getDPindex(), rescaleFactor/100, rescaleRaiseFactor/100);
				break;
			case 1:
				rescaledRouteGeom = GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getDPAndTopoAlongIndex2(), rescaleFactor/100, rescaleRaiseFactor/100);
				break;
			case 2:
				rescaledRouteGeom = GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getDPAndTopoAlongCrossIndex2(), rescaleFactor/100, rescaleRaiseFactor/100);
				break;
			case 3:
				rescaledRouteGeom = GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getAllCrossingsIndex2(), rescaleFactor/100, rescaleRaiseFactor/100);	
				break;
			case 4:
				rescaledRouteGeom = GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getDPAndUrbanCrossIndex(), rescaleFactor/100, rescaleRaiseFactor/100);	
				break;	
			case 5:
				rescaledRouteGeom = GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getPredefinedIndex(), rescaleFactor/100, rescaleRaiseFactor/100);	
				break;	
			default:
				rescaledRouteGeom = GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getDPindex(), rescaleFactor/100, rescaleRaiseFactor/100);

				break;
			}
			
			//

			//rescaledRoute = GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getTopoAlongCrossIndex2(), rescaleFactor/100);

			//

			//route.setRescaledGeom(rescaledRoute);
		//ArrayList<Point2D> rescaledRoutePts = GeoConvertionsOperations.JTSGeometryToJavaD2(rescaledRoute);
			//rescaledRouteNodePositions = GeoConvertionsOperations.JTSGeometryToJavaD2(GeoConvertionsOperations.ScaleProcessing(route.getRoutePath().asLineString(1), route.getDPindex(), rescaleFactor/100));
		}else
			rescaledRouteGeom = route.getRoutePath().asLineString(1) ;
		//pathList.sort(Path.PathLenghtComparator);
		//System.out.println(buildGeoJson());
		
		double rescaledRouteLenght = rescaledRouteGeom.getLength();
		
		routeRescaleLengthProportion = rescaledRouteLenght/ route.getRoutePath().asLineString(1).getLength();
		
		//reProjectTO("EPSG:3785");
		//routeEnvelpe = route.getGeom().getEnvelopeInternal(); 

		/**Simulate navigation data!*/
		//TopoOperator3.simulateNavigationData(route.getRoutePath().asLineString(3));
//		System.out.println("Geom size: " + route.getGeom().getNumPoints() + "  " + route.getGeom());
//		System.out.println("Nodelis size " + route.asNodeList().size() + "  " + route.asNodeList());
		
		return true;

	}
	


	private void reduceChuckPaths() {
		
		
		for(int i = 0; i < pathList.size(); i++){
			
			if(pathList.get(i).isChunkPath() && pathList.get(i).getNodeList().size()==2){
			//if(!pathList.get(i).isRoute() && pathList.get(i).getNodeList().size()==2 && pathList.get(i).getIsPolygon() > 0){
				
				StreetEdge stubEdge = streetNetwork.getEdge(pathList.get(i).getNodeList().get(0), pathList.get(i).getNodeList().get(1));
				int stubEdgeClazz = stubEdge.getClazz();
				
				
				/*if chuck edge start at the route*/
				if(stubEdge.getSourcePoint().isRouteNode()){
				
					StreetEdge routeEdge = route.getAnEdgeWithNode(stubEdge.getSourceId());
					if(stubEdgeClazz - 15 > routeEdge.getClazz() ) {
						
						
						adjacencyList.remove(stubEdge.getTargetPoint().getId());
						adjacencyList.get(stubEdge.getSourcePoint().getId()).remove(stubEdge.getTargetPoint());
						
						
						streetNetwork.getEdges().remove(stubEdge.getId() ,stubEdge );
						//streetNodeMap.remove(p.getNodeList().get(0).getId(), p.getNodeList().get(0));
						streetNodeMap.remove(stubEdge.getTargetPoint().getId() , stubEdge.getTargetPoint());
						pathList.remove(i);
						i--;
						
						
					}
				}
				/*if chuck edge ends at the route*/
				else if(stubEdge.getTargetPoint().isRouteNode()){
				
					StreetEdge routeEdge = route.getAnEdgeWithNode(stubEdge.getTargetId());
					if(stubEdgeClazz - 15 > routeEdge.getClazz() ) {
						
						
						adjacencyList.remove(stubEdge.getSourcePoint().getId());
						adjacencyList.get(stubEdge.getTargetPoint().getId()).remove(stubEdge.getSourcePoint());
						
						
						streetNetwork.getEdges().remove(stubEdge.getId() ,stubEdge );
						//streetNodeMap.remove(p.getNodeList().get(0).getId(), p.getNodeList().get(0));
						streetNodeMap.remove(stubEdge.getSourcePoint().getId() , stubEdge.getSourcePoint());
						pathList.remove(i);
						i--;
						
						
					}
				}
				
				
			}

			
			
		}
		
		
	}



	private void landmarkMarkPathAanlyses(ArrayList<Path> pathList2) {
		for(Path p: pathList2){
			int polygonId;
			boolean allPointsPolygon = true;
			polygonId = p.getNodeList().get(0).getIsPolygonNode();
			if (polygonId > 0) {
				for(StreetNode n: p.getNodeList()) {				
					if(n.getIsPolygonNode() != polygonId) {
						allPointsPolygon = false;
						break;
						
					}
				}
				if(allPointsPolygon)
					p.setIsPolygon(polygonId);
			}
			
		}
		/***Divide paths with anchors constrol points***/
		for(int i = 0; i < pathList2.size(); i++) {
		
			if(pathList2.get(i).getIsPolygon() > 0) {
				Path p = pathList2.get(i);
				if(p.getNodeList().get(0).isRouteNode() && p.getNodeList().get(p.getNodeList().size() -1).isRouteNode() ) {
					for(int j =1; j <p.getNodeList().size() -1; j++) {
						for(NodeTopoRelation ntr: p.getNodeList().get(j).getTopoRelations())
						{
							if (ntr.getType() == NodeTopoRelation.ANCHOR) {
								Path p1 = new Path( new ArrayList<StreetNode>(p.getNodeList().subList(0, j +1)) );
								p1.setIsPolygon(p.getIsPolygon());
								Path p2 = new Path( new ArrayList<StreetNode>(p.getNodeList().subList(j, p.getNodeList().size())) );
								p2.setIsPolygon(p.getIsPolygon());
								pathList2.remove(i);							
								pathList2.add(i,p2);
								pathList2.add(i,p1);
								i++;
								
							}
						}
						
						
					}
				}
			}
		}
		
		
	}






	/**Break paths starting in the route and create only with stubs*/
	private void separateStubsFromPaths() {
		routeAdjPathList = new ArrayList<Path>();
	
		for(Path p: pathList){
			
			//if(!p.isRoute() && !p.isChunkPath() && p.getNodeList().size()>1){
			if(!p.isRoute() && p.getNodeList().size()>2){
				
				/*if path start at the route*/
				if(p.getNodeList().get(0).isRouteNode()){
					ArrayList<StreetNode> adjEdge = new ArrayList<StreetNode>(p.getNodeList().subList(0, 2));
					Path adjPath = new Path(adjEdge);
					
					adjPath.setRouteAdjEdge(true);
					adjPath.setIsPolygon(p.getIsPolygon());
					adjPath.setChunkPath(p.isChunkPath());
					routeAdjPathList.add(adjPath);
					if(!(p.getIsPolygon()>0))
						p.getNodeList().subList(0, 1).clear();					
				}
				/*if path start at the route*/
				int pathLenght = p.getNodeList().size();
				if(p.getNodeList().get(pathLenght-1).isRouteNode()){
					ArrayList<StreetNode> adjEdge = new ArrayList<StreetNode>(p.getNodeList().subList(pathLenght-2, pathLenght));
					Path adjPath = new Path(adjEdge);
					adjPath.setRouteAdjEdge(true);
					adjPath.setIsPolygon(p.getIsPolygon());
					adjPath.setChunkPath(p.isChunkPath());
					routeAdjPathList.add(adjPath);
					if(!(p.getIsPolygon()>0))
						p.getNodeList().subList(pathLenght-1, pathLenght).clear();					
				}
				
				
			}
			else if(!p.isRoute() && p.getNodeList().size()==2){
				
				/*if path start at the route*/
				if(p.getNodeList().get(0).isRouteNode()){					
					p.setRouteAdjEdge(true);

				}
				/*if path start at the route*/
				int pathLenght = p.getNodeList().size();
				if(p.getNodeList().get(pathLenght-1).isRouteNode()){
					p.setRouteAdjEdge(true);				
				}
				
				
			}
			
		}
		pathList.addAll(routeAdjPathList);
		
	}







	private Envelope setNetworkEnvelop() {
		double maxX, minX, maxY, minY;
		maxX = Double.MIN_VALUE;
		minX = Double.MAX_VALUE;
		maxY = Double.MIN_VALUE;
		minY = Double.MAX_VALUE;
		for(StreetNode n: streetNodeMap.values()){
			double x = n.getProjectGeom().getX();
			double y = n.getProjectGeom().getY();
			maxX = (x > maxX) ? x : maxX;
			minX = (x < minX) ? x : minX;
			maxY = (y > maxY) ? y : maxY;
			minY = (y < minY) ? y : minY;
		}
		
		return (new Envelope(minX, maxX, minY, maxY));
	}



	private void isolateRoutePath(ArrayList<Path> pathList2) {
		
		ArrayList<StreetNode> routeNodesList = new ArrayList<StreetNode>();
		ArrayList<StreetNode> extraNodesList = new ArrayList<StreetNode>();

		for(int i = 0; i < pathList2.get(0).getNodeList().size(); i++){
			if (pathList2.get(0).getNodeList().get(i).isRouteNode())
				routeNodesList.add(pathList2.get(0).getNodeList().get(i));
			if(!pathList2.get(0).getNodeList().get(i).isRouteNode()){
				if(pathList2.get(0).getNodeList().get(i-1).isRouteNode())
					extraNodesList.add(pathList2.get(0).getNodeList().get(i-1));
				extraNodesList.add(pathList2.get(0).getNodeList().get(i));
			}
		}
		Path routePath = new Path(routeNodesList);
		routePath.setRoute(true);
		Path extraPath = new Path(extraNodesList);
		extraPath.setRoute(false);
		pathList2.remove(0);
		if(extraPath.getNodeList().size() > 1)
			pathList2.add(0, extraPath);
		pathList2.add(0, routePath);
		
		
	}

	/**Calculate and Update Decision Points*/
	private void anlyseRoute() {
		
		Path routePath = new Path(route.asNodeList());
		PointsPolar routeAsPolar = new PointsPolar();
		routeAsPolar  = GeometricOperation.toPolar(routePath.asJava2DList(0));
		/*REmove DP se for grau 2*/
		
		StreetNode lastDP = routePath.getNodeList().get(0);
//		for(StreetNode n: routePath.getNodeList()){
//			 
//			if(n.isDecisionPoint()){
//				System.out.println("DIST: " + lastDP.getGeom().distance(n.getGeom()));
//				if(lastDP.getGeom().distance(n.getGeom()) < 0.00045){
//					lastDP.setRoundAboutStart(true);
//					n.setDecisionPoint(false);
//					n.setRoundAboutEnd(true);
//					
//				}
//				lastDP = n;
//			}
//		}
		int nodeIndex = 0;
		for(StreetNode n: routePath.getNodeList()){
			
			if(n.isDecisionPoint() && n.getDegree() == 2 ){
				double theta1 = routeAsPolar.getPoints().get(nodeIndex-1).getTheta();
				double theta2 = routeAsPolar.getPoints().get(nodeIndex).getTheta();
				double bendAngle =  theta2 - theta1;
				if (bendAngle < 0)
					bendAngle = bendAngle + 2*Math.PI;
				System.out.println("Turn on DP is : " + bendAngle);
				if (bendAngle > (2*Math.PI -0.7)   ||  bendAngle < 0.7)
					n.setDecisionPoint(false);
			}
//			check that
			if(n.isDecisionPoint() && n.getDegree() > 2){
				double theta1 = routeAsPolar.getPoints().get(nodeIndex-1).getTheta();
				double theta2 = routeAsPolar.getPoints().get(nodeIndex).getTheta();
				double bendAngle =  theta2 - theta1;
				if (bendAngle < 0)
					bendAngle = bendAngle + 2*Math.PI;
				System.out.println("Turn on DP is : " + bendAngle);
				if (bendAngle > (2*Math.PI -0.4)   ||  bendAngle < 0.4)
					n.setDecisionPoint(false);
			}
			
			nodeIndex++;
		}
		nodeIndex = 0;
		for(StreetNode n: routePath.getNodeList()){
			
			if(!n.isDecisionPoint() && n.getDegree() > 2 && nodeIndex>1  && nodeIndex< routeAsPolar.getPoints().size()){
				double theta1 = routeAsPolar.getPoints().get(nodeIndex-1).getTheta();
				double theta2 = routeAsPolar.getPoints().get(nodeIndex).getTheta();
				double bendAngle =  theta2 - theta1;
				if (bendAngle < 0)
					bendAngle = bendAngle + 2*Math.PI;
				System.out.println("Turn on non DP is : " + bendAngle);
				if ( !(bendAngle > (2*Math.PI -0.8)   ||  bendAngle < 0.8)) {
					System.out.println(Math.toDegrees(bendAngle));
					System.out.println("Why im i adding this: " + bendAngle);
					n.setDecisionPoint(true);
				}
			}
			if(n.isRoundAbout())
				n.setDecisionPoint(true);
			
			if(!n.isDecisionPoint() && n.getDegree() > 3 && n.getId() != 14620)
				n.setDecisionPoint(true);
			
			nodeIndex++;
		}
		
		//TopoOperator3.addDummyNodesCloseToDP(route, streetNodeMap, adjacencyList, streetNetwork, 60);
		
		System.out.println("Route edges class");
		for(RouteEdge s: route.getEdgeList()){
			if(s.getSourcePoint().isDecisionPoint() && !s.isReversed())
				System.out.println("DP ");
			if(s.getTargetPoint().isDecisionPoint() && s.isReversed())
				System.out.println("DP ");
			System.out.print(s.getClazz() + " ");
			
		}
		

		
		
	}
	/**Function to select route nodes to be schematize:
	 * All DP, intersection, toponodes, roundabout, and exists(need to be included) are select. 
	 * Plus sections(sublist) between those points, DouglasPeucker will tell wich ones need to be selected***/
	private void setRouteRelevantNodesOldNotWorking() {
		Path routePath = new Path(route.asNodeList());
		ArrayList<Point2D> routePoints = routePath.asJava2DList(0);
		//ArrayList<Integer> relevantPoints1 = new ArrayList<Integer>();
		
		
		
		
		//relevantPoints1.add(startIndex);
		
		for(int i = 0; i < routePath.getNodeList().size(); i++) {
			if( i==0 || i == routePath.getNodeList().size() -1 
					|| 	routePath.getNodeList().get(i).isDecisionPoint()
					|| 	routePath.getNodeList().get(i).getDegree() != 2  
					|| 	routePath.getNodeList().get(i).getTopoRelations().size() >=1
					|| routePath.getNodeList().get(i).isRoundAbout()
							
					) 
				routePath.getNodeList().get(i).setRelevantRouteNode(true);
		}
		
		
		//float routeg = 5.5f; /* any number 0-10*/		
		float routeg = 4.5f; /* any number 0-10*/
		Envelope routeEnvelpe = route.getRoutePath().asLineString(0).getEnvelopeInternal();
		double diagonal = Math.sqrt( Math.pow(routeEnvelpe.getHeight(), 2)  +  Math.pow(routeEnvelpe.getWidth(), 2));
		double tolerance = diagonal*(Math.pow((routeg-1)/10, 3))/(80/(routeg));	
		
		int startIndex = 0;
		int endIndex = 0;
		
		routePath.getNodeList().get(startIndex).setRelevantRouteNode(true);
		
		for(int i = 1; i < routePoints.size(); i++ ){
			if(		routePath.getNodeList().get(i).isDecisionPoint()
					||routePath.getNodeList().get(i).getTopoRelations().size() >=1 
					|| routePath.getNodeList().get(i).isRoundAbout()
					|| (routePath.getNodeList().get(i).getDegree() != 2) 
					|| i == routePoints.size() -1 					
					)
			
			
			{
				endIndex = i;
				
				Point2D simplifiedSubList[] = simplifier.simplify(routePoints.subList(startIndex, endIndex+1).toArray(new Point2D.Double[endIndex+1 -startIndex]), tolerance, true);
				int k = 1;
				for ( int j = startIndex +1 ; j < endIndex+1  ; j++){
					
					if ( routePoints.get(j) == simplifiedSubList[k]){
						
						routePath.getNodeList().get(j).setRelevantRouteNode(true);
						//relevantPoints1.add(j);
						k++;
						
					}
				}
				startIndex = endIndex;
				
				
			
			}

			
		}
		/*Add as relevant point closest points to DP, Crossing, non-2degree*/
		/*Add as relevant point closest points to DP, Crossing, non-2degree*/
		for(int i = 0; i < routePath.getNodeList().size(); i++){
			if(routePath.getNodeList().get(i).isRelevantRouteNode()) {
				
			if( routePath.getNodeList().get(i).isDecisionPoint() ||
					(routePath.getNodeList().get(i).getDegree() != 2  ||					
							routePath.getNodeList().get(i).isTopoCrossing()) 					
					){
				if(i != 0 &&	!routePath.getNodeList().get((i -1)).isRelevantRouteNode() ) {
					routePath.getNodeList().get((i -1)).setRelevantRouteNode(true);
					//relevantPoints1.add(i , relevantPoints1.get(i) -1);
					i++;
					
//					double distToPre = routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i -1)));
//					if(distToPre > 0.003){
//						 if (routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i) -1)) < 0.003 ){							 
//							 routePath.getNodeList().get((relevantPoints1.get(i) -1)).setRelevantRouteNode(true);
//							 relevantPoints1.add(i , relevantPoints1.get(i) -1);
//						 }
//
//					}
				}
				if(i < routePoints.size()-1 && !routePath.getNodeList().get((i +1)).isRelevantRouteNode()) {
					routePath.getNodeList().get(i + 1).setRelevantRouteNode(true);
				    //relevantPoints1.add(i + 1 , relevantPoints1.get(i) + 1);
					
//					double distToPos = routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i + 1)));
//					if(distToPos > 0.003){
//						 if (routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i) + 1)) < 0.003 ){	 
//							 routePath.getNodeList().get(relevantPoints1.get(i) + 1).setRelevantRouteNode(true);
//							 relevantPoints1.add(i + 1 , relevantPoints1.get(i) + 1);
//						 }
//					}
				}
				
				
			}
			}
			
		}
//		for(int i = 1; i < routePoints.size(); i++ ){
//			if(i < 4 || i > routePoints.size() -4 )
//				relevantPoints1.add(i);
//		}
		/*gambiarra ultimo ponto as relevant*/
		
			routePath.getNodeList().get(routePath.getNodeList().size() -1).setRelevantRouteNode(true);
			
			System.out.println( "route size: " + route.asNodeList().size() );
			
			System.out.println( "Relevant Index: " + route.getRoutePath().getRelevantPointIndex() );
		
	}
	
	
	/**Function to select route nodes to be schematize:
	 * All DP, intersection, toponodes, roundabout, and exists(need to be included) are select. 
	 * Plus sections(sublist) between those points, DouglasPeucker will tell wich ones need to be selected***/
	private ArrayList<Integer> setRouteRelevantNodes( ) {
		Path routePath = new Path(route.asNodeList());
		ArrayList<Point2D> routePoints = routePath.asJava2DList(0);
		ArrayList<Integer> relevantPoints1 = new ArrayList<Integer>();
		
		
		for(int i = 0; i < routePath.getNodeList().size(); i++) {
			if( i==0 || i == routePath.getNodeList().size() -1 
					|| 	routePath.getNodeList().get(i).isDecisionPoint()
					|| 	routePath.getNodeList().get(i).getDegree() != 2  
					|| 	routePath.getNodeList().get(i).getTopoRelations().size() >=1
					|| routePath.getNodeList().get(i).isRoundAbout()
							
					) 
				routePath.getNodeList().get(i).setRelevantRouteNode(true);
				relevantPoints1.add(i);
		}
		
		float routeg = 4.5f; /* any number 0-10*/
		Envelope routeEnvelpe = route.getRoutePath().asLineString(0).getEnvelopeInternal();
		double diagonal = Math.sqrt( Math.pow(routeEnvelpe.getHeight(), 2)  +  Math.pow(routeEnvelpe.getWidth(), 2));
		double tolerance = diagonal*(Math.pow((routeg-1)/10, 3))/(80/(routeg));	
		int startIndex = 0;
		int endIndex = 0;
		
		routePath.getNodeList().get(startIndex).setRelevantRouteNode(true);
		relevantPoints1.add(startIndex);
		
		for(int i = 1; i < routePoints.size(); i++ ){
			if(
					//routePath.getNodeList().get(i).getTopoRelations().size() >=1 
					routePath.getNodeList().get(i).isDecisionPoint() 
					//||routePath.getNodeList().get(i).isRoundAbout() 
					//||(routePath.getNodeList().get(i).getDegree() > 2 )
					|| i == routePoints.size() -1 					
					)
			
			
			{
				endIndex = i;
				
				Point2D simplifiedSubList[] = simplifier.simplify(routePoints.subList(startIndex, endIndex+1).toArray(new Point2D.Double[endIndex+1 -startIndex]), tolerance, true);
				int k = 1;
				for ( int j = startIndex +1 ; j < endIndex+1  ; j++){
					
					if ( routePoints.get(j) == simplifiedSubList[k]){
						
						routePath.getNodeList().get(j).setRelevantRouteNode(true);
						if(!relevantPoints1.contains(j)){
							
							relevantPoints1.add(j);
						}
						
						k++;
						
					}
				}
				startIndex = endIndex;
				
				
			
			}

			
		}
		/*Add as relevant point closest points to DP, Crossing, non-2degree*/
		for(int i = 0; i < relevantPoints1.size(); i++){
			if( routePath.getNodeList().get(relevantPoints1.get(i)).isDecisionPoint() 
					||routePath.getNodeList().get(relevantPoints1.get(i)).getDegree() != 2  	
					||routePath.getNodeList().get(relevantPoints1.get(i)).isTopoCrossing()
					 					
					){
				if(relevantPoints1.get(i) != 0 &&	!routePath.getNodeList().get((relevantPoints1.get(i) -1)).isRelevantRouteNode() ) {
					routePath.getNodeList().get((relevantPoints1.get(i) -1)).setRelevantRouteNode(true);
					relevantPoints1.add(i , relevantPoints1.get(i) -1);
					i++;
					
//					double distToPre = routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i -1)));
//					if(distToPre > 0.003){
//						 if (routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i) -1)) < 0.003 ){							 
//							 routePath.getNodeList().get((relevantPoints1.get(i) -1)).setRelevantRouteNode(true);
//							 relevantPoints1.add(i , relevantPoints1.get(i) -1);
//						 }
//
//					}
				}
				if(relevantPoints1.get(i) < routePoints.size()-1 && !routePath.getNodeList().get((relevantPoints1.get(i) +1)).isRelevantRouteNode()) {
					routePath.getNodeList().get(relevantPoints1.get(i) + 1).setRelevantRouteNode(true);
				    relevantPoints1.add(i + 1 , relevantPoints1.get(i) + 1);
					
//					double distToPos = routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i + 1)));
//					if(distToPos > 0.003){
//						 if (routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i) + 1)) < 0.003 ){	 
//							 routePath.getNodeList().get(relevantPoints1.get(i) + 1).setRelevantRouteNode(true);
//							 relevantPoints1.add(i + 1 , relevantPoints1.get(i) + 1);
//						 }
//					}
				}
				
				
			}
			
		}
//		for(int i = 1; i < routePoints.size(); i++ ){
//			if(i < 4 || i > routePoints.size() -4 )
//				relevantPoints1.add(i);
//		}
		/*gambiarra ultimo ponto as relevant*/
		if(!relevantPoints1.contains(routePath.getNodeList().size() -1)){
			routePath.getNodeList().get(routePath.getNodeList().size() -1).setRelevantRouteNode(true);
			relevantPoints1.add(routePath.getNodeList().size() -1);
		}
		
		//System.out.println("Tolerance: " +tolerance+  " NumPTsOriginal: "+ originalPoints.size()+ "NumPTRelevant: " + relevantPoints1.size());
		//Collections.sort(relevantPoints1);
		return relevantPoints1;
	}
	
	/**Function to select route nodes to be schematize:
	 * All DP, intersection, toponodes, roundabout, and exists(need to be included) are select. 
	 * Plus sections(sublist) between those points, DouglasPeucker will tell wich ones need to be selected***/
	private ArrayList<Integer> getRouteRelevantPointsOldDelete2( double tolerance, boolean highQuality) {
		Path routePath = new Path(route.asNodeList());
		ArrayList<Point2D> routePoints = routePath.asJava2DList(0);
		ArrayList<Integer> relevantPoints1 = new ArrayList<Integer>();
		
		
		
		int startIndex = 0;
		int endIndex = 0;
		
		routePath.getNodeList().get(startIndex).setRelevantRouteNode(true);
		relevantPoints1.add(startIndex);
		
		for(int i = 1; i < routePoints.size(); i++ ){
			if(routePath.getNodeList().get(i).getTopoRelations().size() >=1 ||
					routePath.getNodeList().get(i).isDecisionPoint() ||
					routePath.getNodeList().get(i).isRoundAbout() ||
					(routePath.getNodeList().get(i).getDegree() != 2 ||
					i == routePoints.size() -1) 					
					)
			
			
			{
				endIndex = i;
				
				Point2D simplifiedSubList[] = simplifier.simplify(routePoints.subList(startIndex, endIndex+1).toArray(new Point2D.Double[endIndex+1 -startIndex]), tolerance, highQuality);
				int k = 1;
				for ( int j = startIndex +1 ; j < endIndex+1  ; j++){
					
					if ( routePoints.get(j) == simplifiedSubList[k]){
						
						routePath.getNodeList().get(j).setRelevantRouteNode(true);
						relevantPoints1.add(j);
						k++;
						
					}
				}
				startIndex = endIndex;
				
				
			
			}

			
		}
		/*Add as relevant point closest points to DP, Crossing, non-2degree*/
		for(int i = 0; i < relevantPoints1.size(); i++){
			if( routePath.getNodeList().get(relevantPoints1.get(i)).isDecisionPoint() ||
					(routePath.getNodeList().get(relevantPoints1.get(i)).getDegree() != 2  ||					
							routePath.getNodeList().get(relevantPoints1.get(i)).isTopoCrossing()) 					
					){
				if(relevantPoints1.get(i) != 0 &&	!routePath.getNodeList().get((relevantPoints1.get(i) -1)).isRelevantRouteNode() ) {
					routePath.getNodeList().get((relevantPoints1.get(i) -1)).setRelevantRouteNode(true);
					relevantPoints1.add(i , relevantPoints1.get(i) -1);
					i++;
					
//					double distToPre = routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i -1)));
//					if(distToPre > 0.003){
//						 if (routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i) -1)) < 0.003 ){							 
//							 routePath.getNodeList().get((relevantPoints1.get(i) -1)).setRelevantRouteNode(true);
//							 relevantPoints1.add(i , relevantPoints1.get(i) -1);
//						 }
//
//					}
				}
				if(relevantPoints1.get(i) < routePoints.size()-1 && !routePath.getNodeList().get((relevantPoints1.get(i) +1)).isRelevantRouteNode()) {
					routePath.getNodeList().get(relevantPoints1.get(i) + 1).setRelevantRouteNode(true);
				    relevantPoints1.add(i + 1 , relevantPoints1.get(i) + 1);
					
//					double distToPos = routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i + 1)));
//					if(distToPos > 0.003){
//						 if (routePoints.get(relevantPoints1.get(i)).distance(routePoints.get(relevantPoints1.get(i) + 1)) < 0.003 ){	 
//							 routePath.getNodeList().get(relevantPoints1.get(i) + 1).setRelevantRouteNode(true);
//							 relevantPoints1.add(i + 1 , relevantPoints1.get(i) + 1);
//						 }
//					}
				}
				
				
			}
			
		}
//		for(int i = 1; i < routePoints.size(); i++ ){
//			if(i < 4 || i > routePoints.size() -4 )
//				relevantPoints1.add(i);
//		}
		/*gambiarra ultimo ponto as relevant*/
		if(!relevantPoints1.contains(routePath.getNodeList().size() -1)){
			routePath.getNodeList().get(routePath.getNodeList().size() -1).setRelevantRouteNode(true);
			relevantPoints1.add(routePath.getNodeList().size() -1);
		}
		
		System.out.println( "route size: " + route.asNodeList().size() );
		
		System.out.println( "Relevant Index: " + route.getRoutePath().getRelevantPointIndex() );
		
		//System.out.println("Tolerance: " +tolerance+  " NumPTsOriginal: "+ originalPoints.size()+ "NumPTRelevant: " + relevantPoints1.size());
		//Collections.sort(relevantPoints1);
		return relevantPoints1;
	}

	
	/**Function to select route nodes to be schematize:
	 * All DP, intersection, toponodes, roundabout, and exists(need to be included) are select. 
	 * Plus sections(sublist) between those points, DouglasPeucker will tell wich ones need to be selected***/
	private void setPathRelevantPoints(Path path, double g, double densifyTolerance, boolean highQuality) {
		ArrayList<Point2D> pathPoints = path.asJava2DList(0);
		ArrayList<Integer> relevantPoints1 = new ArrayList<Integer>();
		
	
		int startIndex = 0;
		int endIndex = 0;
		path.getNodeList().get(startIndex).setRelevantRouteNode(true);
		relevantPoints1.add(startIndex);
		double length = 0;
		for(int i = 1; i < pathPoints.size(); i++ ){
			
			length +=   path.getNodeList().get(i).getGeom().distance(path.getNodeList().get(i -1).getGeom() );
			if(path.getNodeList().get(i).getTopoRelations().size() >=1 ||
					(path.getNodeList().get(i).getDegree() != 2 ) 	||			
					(i == pathPoints.size() -1) /**add this line because it was not anlaysing the last section of the close polygon*/
					){
				endIndex = i;
				
				
				if(endIndex - startIndex < 3){
					
					for ( int j = startIndex +1 ; j < endIndex+1  ; j++){						
											
							path.getNodeList().get(j).setRelevantRouteNode(true);							
							relevantPoints1.add(j);
						
					}
					
					
					
				}
				else {
			
					//System.out.println("long: " + length );
					
					//double tolerance = Math.pow(length,0.8)*(Math.pow((g/4), 3))/(80/(g));
					double tolerance = Math.pow(length/60,0.8)*g;
					//System.out.println("tolerance: " + tolerance);
					//System.out.println("g*15: " + g*10);
					
					tolerance = Math.max(g*10, tolerance);
					//double tolerance = g*15;
					
					//tolerance  = Math.max(length/30, tolerance);
					Point2D simplifiedSubList2[] = simplifier.simplify(pathPoints.subList(startIndex, endIndex+1).toArray(new Point2D.Double[endIndex+1 -startIndex]), tolerance, highQuality);
					
					//System.out.println("orig lenght: " + (endIndex - startIndex) );
					//System.out.println("after first: " + simplifiedSubList2.length);
					double newTolerance = tolerance;
					while(simplifiedSubList2.length < 4 && newTolerance > 10) {
						newTolerance = newTolerance*0.8;
						Point2D	simplifiedSubList3[] = simplifier.simplify(pathPoints.subList(startIndex, endIndex+1).toArray(new Point2D.Double[endIndex+1 -startIndex]), newTolerance, highQuality);
						simplifiedSubList2 = simplifiedSubList3;
						//System.out.println("after first: " + simplifiedSubList2.length);
					}
					//System.out.println("last: " + simplifiedSubList2.length);
					int k = 1;
					for ( int j = startIndex +1 ; j < endIndex+1  ; j++){
						
						if ( pathPoints.get(j) == simplifiedSubList2[k]){
							
							path.getNodeList().get(j).setRelevantRouteNode(true);
							relevantPoints1.add(j);
							k++;
							
						}
//						else if( j == startIndex +1  || j == endIndex ) {
//							path.getNodeList().get(j).setRelevantRouteNode(true);
//							relevantPoints1.add(j);
//						}
					}
					
				
				}
				startIndex = endIndex;
				length = 0;
				
			
			}

			
		}
		/*Add as relevant point closest points to DP, Crossing, non-2degree*/
		for(int i = 0; i < relevantPoints1.size(); i++){
			if(	(path.getNodeList().get(relevantPoints1.get(i)).getDegree() != 2  ) 					
					){
				if( relevantPoints1.get(i) != 0 && !path.getNodeList().get((relevantPoints1.get(i) -1)).isRelevantRouteNode() 
						
						){
					path.getNodeList().get((relevantPoints1.get(i) -1)).setRelevantRouteNode(true);
					relevantPoints1.add(i , relevantPoints1.get(i) -1);
					i++;
					
				}
				if( relevantPoints1.get(i) < path.getNodeList().size()-1 && !path.getNodeList().get((relevantPoints1.get(i) +1)).isRelevantRouteNode()
						
						) {
					path.getNodeList().get(relevantPoints1.get(i) + 1).setRelevantRouteNode(true);
				    relevantPoints1.add(i + 1 , relevantPoints1.get(i) + 1);
					

				}
				
				
			}
			
		}


		
//		for(int i = 0; i < relevantPoints1.size() - 1; i++){
//			if( path.getNodeList().get(relevantPoints1.get(i)).getGeom().distance(path.getNodeList().get(relevantPoints1.get(i +1)).getGeom()) > densifyTolerance*1.2 ) {
//				int k = relevantPoints1.get(i) + 1;
//				double dist = path.getNodeList().get(relevantPoints1.get(i)).getGeom().distance(path.getNodeList().get(k).getGeom());
//				boolean achou = false;
//				while(!achou && k < path.getNodeList().size() -2) {
//					k++;
//					dist = path.getNodeList().get(relevantPoints1.get(i)).getGeom().distance(path.getNodeList().get(k).getGeom());
//					if(dist > densifyTolerance  && k < relevantPoints1.get(i +1)) {
//						relevantPoints1.add(i + 1 , k);
//					    path.getNodeList().get(k).setRelevantRouteNode(true);
//					    achou = true;
//					}
//					else if( k == relevantPoints1.get(i +1))
//						achou = true;
//						
//					
//				}
//			}
//		}
		
		


	}



	/**Indentify paths that are chuck edges*/
	public  void chunckPahtAnalyses(ArrayList<Path> pathList) {
		for(Path p: pathList){
			if(!p.isRoute() && !(p.getIsPolygon() > 0)) {
				
				/*if path start at the route*/
				if(p.getNodeList().get(0).isRouteNode() && !p.getNodeList().get(0).isDecisionPoint() && p.getNodeList().size()  == 2 && p.getNodeList().get(1).getDegree() == 1 && !(p.getNodeList().get(1).getIsPointLMNode() > 0) ){
					p.setChunkPath(true);
					
				}
				if(p.getNodeList().get(0).isRouteNode()  && p.getNodeList().size() < 4 && p.getNodeList().get(p.getNodeList().size() -1).isRouteNode()){
					p.setChunkPath(true);
					
				}
//				if( p.getNodeList().size() == 2 && (p.getNodeList().get(0).isRouteNode()  || p.getNodeList().get(p.getNodeList().size() -1).isRouteNode() ) ) {
//					p.setChunkPath(true);
//					
//				}
			}
		}
	}
	/**Remove Residuals (extras nodes and edges from street paths that are chunck) **/
	private  void chunckPahtResidualRemoval() {
		for(int i = 0; i < pathList.size(); i++){
			Path p = pathList.get(i);
			if(!p.isRoute() && !(p.getIsPolygon() > 0)) {
				
				/*if path start at the route*/
				//if(p.getNodeList().get(0).isRouteNode() && !p.getNodeList().get(0).isDecisionPoint() && p.getNodeList().size()  == 2 && p.getNodeList().get(1).getDegree() ==1){
				if(p.isChunkPath() && !p.getNodeList().get(0).isRouteNode() && p.getNodeList().size()  == 2 && p.getNodeList().get(1).getDegree() ==1 ) {	
					int removeNodeId = p.getNodeList().get(1).getId();
					adjacencyList.remove(p.getNodeList().get(1).getId());
					adjacencyList.get(p.getNodeList().get(0).getId()).remove(p.getNodeList().get(1));
					
					StreetEdge e = streetNetwork.getEdge(p.getNodeList().get(0), p.getNodeList().get(1));
					streetNetwork.getEdges().remove(e.getId() ,e );
					//streetNodeMap.remove(p.getNodeList().get(0).getId(), p.getNodeList().get(0));
					streetNodeMap.remove(p.getNodeList().get(1).getId() , p.getNodeList().get(1));
					pathList.remove(i);
					i--;
				}
				if(p.isChunkPath() && p.getNodeList().size()  == 3 && !p.getNodeList().get(0).isRouteNode()  && !p.getNodeList().get(p.getNodeList().size() -1).isRouteNode()&&  p.getNodeList().get(1).getDegree() ==2 ) {	
					int removeNodeId = p.getNodeList().get(1).getId();
					adjacencyList.remove(p.getNodeList().get(1).getId());
					adjacencyList.get(p.getNodeList().get(0).getId()).remove(p.getNodeList().get(1));
					adjacencyList.get(p.getNodeList().get(p.getNodeList().size() -1).getId()).remove(p.getNodeList().get(1));
					
					StreetEdge e1 = streetNetwork.getEdge(p.getNodeList().get(0), p.getNodeList().get(1));
					streetNetwork.getEdges().remove(e1.getId() ,e1 );
					
					StreetEdge e2 = streetNetwork.getEdge(p.getNodeList().get(p.getNodeList().size() -1), p.getNodeList().get(1));
					streetNetwork.getEdges().remove(e2.getId() ,e2 );
					//streetNodeMap.remove(p.getNodeList().get(0).getId(), p.getNodeList().get(0));
					streetNodeMap.remove(p.getNodeList().get(1).getId() , p.getNodeList().get(1));
					pathList.remove(i);
					i--;
				}
			}
		}
	}



	private void removeDisconnectNodes() {
		for(int i = 0; i < pathList.size(); i++){
			boolean disconnectedPath = false;
			Path p = pathList.get(i);
			if(p.getNodeList().get(0).isDisconnected()) {
				disconnectedPath = true;
				for(int j = 0; j < p.getNodeList().size() - 1; j++) {
					StreetEdge e = streetNetwork.getEdge(p.getNodeList().get(j), p.getNodeList().get(j+1));
					streetNetwork.getEdges().remove(e.getId() ,e );
					adjacencyList.remove(p.getNodeList().get(j).getId());
					streetNodeMap.remove(p.getNodeList().get(j).getId() , p.getNodeList().get(j));
				}
				adjacencyList.remove(p.getNodeList().get(p.getNodeList().size() -1).getId());
				streetNodeMap.remove(p.getNodeList().get(p.getNodeList().size() -1).getId() , p.getNodeList().get(p.getNodeList().size() -1));
				pathList.remove(i);
				i--;
			}
		
			
		}
		
		
	}


	private ArrayList<Path> createTrackListStreetOnly() {
		streetOnlyPathList = new ArrayList<Path>();
		
		Path routePath; 
		
		ArrayList<StreetNode> visitingList = new ArrayList<StreetNode>(); /* Lista de visitação */
		int preOrderIndex, postOrderIndex;
		boolean allPointsVisited = false;
		boolean isDisconnected = false;
		boolean newTrack = false;/* to check if a new track/path is been build */
		StreetNode mainPoint, adjPoint;
		
		ArrayList<StreetNode> track = new ArrayList<StreetNode>();
		
		
		preOrderIndex = 0; /* ordem do empilhamento */
		postOrderIndex = 0; /* ordem do desempilhamento */
		
		for(StreetNode s: streetNodeMap.values()){
			if( s.isStreetNode()) {
				s.setPreOrder(-1);
				s.setPostOrder(-1);					
			}
			else {
				s.setPreOrder(10000);
				s.setPostOrder(10000);		
			}
		}
		for(StreetNode n: route.asNodeList()){
			n.setPreOrder(++preOrderIndex);			
			visitingList.add(n);
			track.add(n);
		}
		routePath = new Path(track);
		track = new ArrayList<StreetNode>();	
		newTrack = true;

		int mainPointId = route.getStart().getId();
		int bestPriority = streetNodeMap.get(mainPointId).getPriority();
		while(!allPointsVisited){
			
			StreetNode node = streetNodeMap.get(mainPointId);
			
				
			node.setPreOrder(++preOrderIndex);
			
			visitingList.add(node);
			
			while (!visitingList.isEmpty()){
				//mainPoint = getBestNode(visitingList);
				/*if is a new track then give preference to start a track connected to the route*/

				mainPoint =	visitingList.get(visitingList.size() -1);

				
				//if(isDisconnected && (mainPoint.getIsPointLMNode() < 1) && (mainPoint.getIsPolygonNode() < 1))
					//mainPoint.setDisconnected(true);
				track.add(mainPoint);
				mainPointId = mainPoint.getId();
				boolean achou = false;
				/*encontra uma parada adjacente ainda não  visitada */
				/*Ordena lista de adjacencia para achar ponto que forma mernor angulo*/
				if(track.size() > 1 && adjacencyList.get(mainPointId).size() > 2 ) {
					putLowestBendOnTop(track.get(track.size() - 2), mainPoint, adjacencyList.get(mainPointId));			
				}
				/*Busca parada adjacent não visitada, e empilha. 
				  Terá preferencia paradas no topo da lista de adjacencia da ultima parara visitada(mainPoint)
				  Interacao interronpida com break se achar parada*/
				
					for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
						adjPoint = adjacencyList.get(mainPointId).get(i);
						if(adjPoint.getPreOrder() < 0 && achou == false){
							
							visitingList.add(adjPoint);
							adjPoint.setPreOrder(++preOrderIndex);
							achou = true;
							newTrack = false;
							break;
							
						}
	
					
					}
				/*Se não achar nenhum adjacente finaliza track, e cria complementar-edge*/
				if(achou == false){
					if(track.size() > 1 && adjacencyList.get(mainPointId).size() > 2 ) {
						bestToClosePathOnTop(track.get(track.size() - 2), mainPoint, adjacencyList.get(mainPointId));

					}		
					/*procura nós que fechem a track, e cria coedges tracks*/
					int j = 0;
					for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
						adjPoint = adjacencyList.get(mainPointId).get(i);
//						if(adjPoint.getPostOrder() < 0 &&							
//								adjPoint != visitingList.get(visitingList.size() -2)){						
						if(adjPoint.getPostOrder() < 0 &&
								 track.size()>1 &&
								adjPoint != track.get(track.size() -2)){
							
							j++;
							/* adiciona primeiro adjacente (ja vsitado) e fecha circulo */
							if(j==1){
								
								track.add(adjPoint);
								streetOnlyPathList.add(new Path(track));
								//coEgedPathList.remove(coEgedPathList.size() - 1);
								
							
							}

//							else{
//								ArrayList<StreetNode> coEdgetrack = new ArrayList<StreetNode>();
//								coEdgetrack.add( mainPoint );
//								coEdgetrack.add( adjPoint );	
//								streetOnlyPathList.add(new Path(coEdgetrack));
//								
//								/*gambi para saber quantas coEdge*/
//							}
							
						}

					}

					/*se não há nó adjacente visitado fecha a track*/
					if(j==0){
						if(track.size()>1)
							streetOnlyPathList.add(new Path(track));
					}

	
					/*desempilha o nó e recomeça a track*/
					streetNodeMap.get(mainPointId).setPostOrder(++postOrderIndex);
					visitingList.remove(mainPoint);
					track = new ArrayList<StreetNode>();	
					newTrack = true;
					
										
				}			
			}
			/* Check if unconnect parts of the graph has been visited */

			bestPriority = 100;
			allPointsVisited = true;
			for(StreetNode n: streetNodeMap.values()){
			
			
				
				preOrderIndex = 0; /* ordem do empilhamento */
				postOrderIndex = 0; /* ordem do desempilhamento */
				/*index primeira parada com menor grau*/
				if (n.getPreOrder() < 0){
					
					isDisconnected = true;
					if(n.getPriority() < bestPriority){
						bestPriority = n.getPriority();
						mainPointId = n.getId();
					}
					allPointsVisited = false;
				   	
				} 	
				
			}
			
		}
		streetOnlyPathList.add(0, routePath);
		return streetOnlyPathList;
		
	}
	
	/**Divides the networt into paths. Similar to deep-first graph search, but gives preference to to paths connected to the route*/ 
	private ArrayList<Path> createTrackListLandMarkPreference() {
		pathList = new ArrayList<Path>();
		coEgedPathList = new ArrayList<Path>();
		Path routePath; 
		
		ArrayList<StreetNode> visitingList = new ArrayList<StreetNode>(); /* Lista de visitação */
		int preOrderIndex, postOrderIndex;
		boolean allPointsVisited = false;
		boolean isDisconnected = false;
		boolean newTrack = false;/* to check if a new track/path is been build */
		StreetNode mainPoint, adjPoint;
		
		
		ArrayList<StreetNode> track = new ArrayList<StreetNode>();
		
		
		preOrderIndex = 0; /* ordem do empilhamento */
		postOrderIndex = 0; /* ordem do desempilhamento */
		
		/* inicializa o preordem e post ordem das paradas como -1* o s
		 * ou seja "não visitado" e "não finalizado"
		 */
		
	
		for(StreetNode s: streetNodeMap.values()){
			s.setPreOrder(-1);
			s.setPostOrder(-1);					
		}
		for(StreetNode n: route.asNodeList()){
			n.setPreOrder(++preOrderIndex);			
			visitingList.add(n);
			track.add(n);
		}
		routePath = new Path(track);
		track = new ArrayList<StreetNode>();	
		newTrack = true;
		for (PolygonalTopo p: this.polygonalTopoList){
			if( p.getType() != PolygonalTopo.SIMPLE_CROSSING &&
					p.getType() != PolygonalTopo.ROUTE_STARTS_AT &&
					p.getType() != PolygonalTopo.ROUTE_ENDS_AT ){
				
				
				p.getEdgeList().get(0).getSourcePoint().setPreOrder(++preOrderIndex);
				visitingList.add(p.getEdgeList().get(0).getSourcePoint());
			}
		}
		boolean foundLandmark = true;
		int mainPointId = route.getStart().getId();
		int bestPriority = streetNodeMap.get(mainPointId).getPriority();
		while(!allPointsVisited){
			
			StreetNode node = streetNodeMap.get(mainPointId);
			
				
			node.setPreOrder(++preOrderIndex);
			
			visitingList.add(node);
			
			while (!visitingList.isEmpty()){
				//mainPoint = getBestNode(visitingList);
				/*if is a new track then give preference to start a track connected to the route*/
				if(!newTrack)
					mainPoint =	visitingList.get(visitingList.size() -1);
				else
					mainPoint = getBestNodeLandmarkPreference(visitingList, foundLandmark); /*select as mainpoint(first node) a node in the route*/
				
				if(isDisconnected && (mainPoint.getIsPointLMNode() < 1) && (mainPoint.getIsPolygonNode() < 1))
					mainPoint.setDisconnected(true);
				track.add(mainPoint);
				mainPointId = mainPoint.getId();
				boolean achou = false;
				boolean endOfPolygonPath = false;
				/*encontra uma parada adjacente ainda não  visitada */
				/*Ordena lista de adjacencia para achar ponto que forma mernor angulo*/
				if(track.size() > 1 && adjacencyList.get(mainPointId).size() > 2 ) {
					putLowestBendOnTop(track.get(track.size() - 2), mainPoint, adjacencyList.get(mainPointId));

					if(mainPoint.getIsPolygonNode() > 0 &&
							mainPoint.getIsPolygonNode() == track.get(track.size() - 2).getIsPolygonNode() &&
							adjacencyList.get(mainPointId).get(0).getIsPolygonNode() != mainPoint.getIsPolygonNode() ){
						endOfPolygonPath = true;
					}
				
				}
				/*Busca parada adjacent não visitada, e empilha. 
				  Terá preferencia paradas no topo da lista de adjacencia da ultima parara visitada(mainPoint)
				  Interacao interronpida com break se achar parada*/
				
				if(!endOfPolygonPath)
					for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
						adjPoint = adjacencyList.get(mainPointId).get(i);
						if(adjPoint.getPreOrder() < 0 && achou == false){
							
							visitingList.add(adjPoint);
							adjPoint.setPreOrder(++preOrderIndex);
							achou = true;
							newTrack = false;
							break;
							
						}
	
					
					}
				/*Se não achar nenhum adjacente finaliza track, e cria complementar-edge*/
				if(achou == false){
					if(track.size() > 1 && adjacencyList.get(mainPointId).size() > 2 ) {
						bestToClosePathOnTop(track.get(track.size() - 2), mainPoint, adjacencyList.get(mainPointId));

					}		
					/*procura nós que fechem a track, e cria coedges tracks*/
					int j = 0;
					for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
						adjPoint = adjacencyList.get(mainPointId).get(i);
//						if(adjPoint.getPostOrder() < 0 &&							
//								adjPoint != visitingList.get(visitingList.size() -2)){						
						if(adjPoint.getPostOrder() < 0 &&
								 track.size()>1 &&
								adjPoint != track.get(track.size() -2)){
							
							j++;
							/* adiciona primeiro adjacente (ja vsitado) e fecha circulo */
							if(j==1){
								
								track.add(adjPoint);
								pathList.add(new Path(track));
								//coEgedPathList.remove(coEgedPathList.size() - 1);
								
							
							}
							/*give up trying to find the co-edges*/
							/* cria coedge para os outros*/
//							else{
//								ArrayList<StreetNode> coEdgetrack = new ArrayList<StreetNode>();
//								coEdgetrack.add( mainPoint );
//								coEdgetrack.add( adjPoint );	
//								coEgedPathList.add(new Path(coEdgetrack));
//								
//								/*gambi para saber quantas coEdge*/
//							}
							
						}

					}

					/*se não há nó adjacente visitado fecha a track*/
					if(j==0){
						if(track.size()>1)
							pathList.add(new Path(track));
					}

	
					/*desempilha o nó e recomeça a track*/
					streetNodeMap.get(mainPointId).setPostOrder(++postOrderIndex);
					visitingList.remove(mainPoint);
					track = new ArrayList<StreetNode>();	
					newTrack = true;
					
										
				}			
			}
			/* Check if unconnect parts of the graph has been visited */

			bestPriority = 100;
			allPointsVisited = true;
			for(StreetNode n: streetNodeMap.values()){
			
			
				
				preOrderIndex = 0; /* ordem do empilhamento */
				postOrderIndex = 0; /* ordem do desempilhamento */
				/*index primeira parada com menor grau*/
				if (n.getPreOrder() < 0){
					
					isDisconnected = true;
					if(n.getPriority() < bestPriority){
						bestPriority = n.getPriority();
						mainPointId = n.getId();
					}
					allPointsVisited = false;
				   	
				} 	
				
			}
			
		}
		pathList.add(0, routePath);
		return pathList;
	}
	
	

	/**Divides the networt into paths. Similar to deep-first graph search, but gives preference to to paths connected to the route*/ 
	private ArrayList<Path> createTrackListLandMarkPreference2() {
		pathList = new ArrayList<Path>();
		coEgedPathList = new ArrayList<Path>();
		Path routePath; 
		
		ArrayList<StreetNode> visitingList = new ArrayList<StreetNode>(); /* Lista de visitação */
		int preOrderIndex, postOrderIndex;
		boolean allPointsVisited = false;
		boolean isDisconnected = false;
		boolean newTrack = false;/* to check if a new track/path is been build */
		StreetNode mainPoint, adjPoint;
		
		
		ArrayList<StreetNode> track = new ArrayList<StreetNode>();
		
		
		preOrderIndex = 0; /* ordem do empilhamento */
		postOrderIndex = 0; /* ordem do desempilhamento */
		
		/* inicializa o preordem e post ordem das paradas como -1* o s
		 * ou seja "não visitado" e "não finalizado"
		 */
		
	
		for(StreetNode s: streetNodeMap.values()){
			s.setPreOrder(-1);
			s.setPostOrder(-1);					
		}
		for(StreetNode n: route.asNodeList()){
			n.setPreOrder(++preOrderIndex);			
			visitingList.add(n);
			track.add(n);
		}
		routePath = new Path(track);
		track = new ArrayList<StreetNode>();	
		newTrack = true;
		for (PolygonalTopo p: this.polygonalTopoList){
			if( p.getType() != PolygonalTopo.SIMPLE_CROSSING &&
					p.getType() != PolygonalTopo.ROUTE_STARTS_AT &&
					p.getType() != PolygonalTopo.ROUTE_ENDS_AT ){
				
				
				p.getEdgeList().get(0).getSourcePoint().setPreOrder(++preOrderIndex);
				visitingList.add(p.getEdgeList().get(0).getSourcePoint());
			}
		}
		boolean foundLandmark = true;
		int mainPointId = route.getStart().getId();
		int bestPriority = streetNodeMap.get(mainPointId).getPriority();
		while(!allPointsVisited){
			
			StreetNode node = streetNodeMap.get(mainPointId);
			
				
			node.setPreOrder(++preOrderIndex);
			
			visitingList.add(node);
			
			while (!visitingList.isEmpty()){
				//mainPoint = getBestNode(visitingList);
				/*if is a new track then give preference to start a track connected to the route*/
				if(!newTrack)
					mainPoint =	visitingList.get(visitingList.size() -1);
				else
					mainPoint = getBestNodeLandmarkPreference(visitingList, foundLandmark); /*select as mainpoint(first node) a node in the route*/
				
				if(isDisconnected && (mainPoint.getIsPointLMNode() < 1) && (mainPoint.getIsPolygonNode() < 1))
					mainPoint.setDisconnected(true);
				track.add(mainPoint);
				mainPointId = mainPoint.getId();
				boolean achou = false;
				boolean endOfPolygonPath = false;
				/*encontra uma parada adjacente ainda não  visitada */
				/*Ordena lista de adjacencia para achar ponto que forma mernor angulo*/
				if(track.size() > 1 && adjacencyList.get(mainPointId).size() > 2 ) {
					putLowestBendOnTop(track.get(track.size() - 2), mainPoint, adjacencyList.get(mainPointId));

					if(mainPoint.getIsPolygonNode() > 0 &&
							mainPoint.getIsPolygonNode() == track.get(track.size() - 2).getIsPolygonNode() &&
							adjacencyList.get(mainPointId).get(0).getIsPolygonNode() != mainPoint.getIsPolygonNode() ){
						endOfPolygonPath = true;
					}
				
				}
				/*Busca parada adjacent não visitada, e empilha. 
				  Terá preferencia paradas no topo da lista de adjacencia da ultima parara visitada(mainPoint)
				  Interacao interronpida com break se achar parada*/
				
				if(!endOfPolygonPath)
					for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
						adjPoint = adjacencyList.get(mainPointId).get(i);
						if(adjPoint.getPreOrder() < 0 && achou == false){
							
							visitingList.add(adjPoint);
							adjPoint.setPreOrder(++preOrderIndex);
							achou = true;
							newTrack = false;
							break;
							
						}
	
					
					}
				/*Se não achar nenhum adjacente finaliza track, e cria complementar-edge*/
				if(achou == false){
					if(track.size() > 1 && adjacencyList.get(mainPointId).size() > 2 ) {
						bestToClosePathOnTop(track.get(track.size() - 2), mainPoint, adjacencyList.get(mainPointId));

					}		
					/*procura nós que fechem a track, e cria coedges tracks*/
					int j = 0;
					for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
						adjPoint = adjacencyList.get(mainPointId).get(i);
//						if(adjPoint.getPostOrder() < 0 &&							
//								adjPoint != visitingList.get(visitingList.size() -2)){						
						if(adjPoint.getPostOrder() < 0 &&
								 track.size()>1 &&
								adjPoint != track.get(track.size() -2)){
							
							j++;
							/* adiciona primeiro adjacente (ja vsitado) e fecha circulo */
							if(j==1){
								
								track.add(adjPoint);
								pathList.add(new Path(track));
								//coEgedPathList.remove(coEgedPathList.size() - 1);
								
							
							}
							/*give up trying to find the co-edges*/
							/* cria coedge para os outros*/
//							else{
//								ArrayList<StreetNode> coEdgetrack = new ArrayList<StreetNode>();
//								coEdgetrack.add( mainPoint );
//								coEdgetrack.add( adjPoint );	
//								coEgedPathList.add(new Path(coEdgetrack));
//								
//								/*gambi para saber quantas coEdge*/
//							}
							
						}						
						else if( adjPoint.getDegree() > 2 && track.size() <2 && adjPoint.getPreOrder() > -1 && adjPoint.getPostOrder() < 0  &&
								Math.abs(adjPoint.getPreOrder() - mainPoint.getPreOrder() ) > 1	&& adjPoint != visitingList.get(visitingList.size() -2)){
							
							System.out.println("mainPreOrder: " + mainPoint.getPreOrder());
							System.out.println("adjPreOrder: " + adjPoint.getPreOrder());
						
							System.out.println("mainPostOrder: " + mainPoint.getPostOrder());
							System.out.println("adjPostOrder: " + adjPoint.getPostOrder());
							
							ArrayList<StreetNode> coEdgetrack = new ArrayList<StreetNode>();
							coEdgetrack.add( mainPoint );
							coEdgetrack.add( adjPoint );	
							coEgedPathList.add(new Path(coEdgetrack));
							
						}
	
					
					}

					

					/*se não há nó adjacente visitado fecha a track*/
					if(j==0){
						if(track.size()>1)
							pathList.add(new Path(track));
					}

	
					/*desempilha o nó e recomeça a track*/
					streetNodeMap.get(mainPointId).setPostOrder(++postOrderIndex);
					visitingList.remove(mainPoint);
					track = new ArrayList<StreetNode>();	
					newTrack = true;
					
										
				}			
			}
			/* Check if unconnect parts of the graph has been visited */

			bestPriority = 100;
			allPointsVisited = true;
			for(StreetNode n: streetNodeMap.values()){
			
			
				
				preOrderIndex = 0; /* ordem do empilhamento */
				postOrderIndex = 0; /* ordem do desempilhamento */
				/*index primeira parada com menor grau*/
				if (n.getPreOrder() < 0){
					
					isDisconnected = true;
					if(n.getPriority() < bestPriority){
						bestPriority = n.getPriority();
						mainPointId = n.getId();
					}
					allPointsVisited = false;
				   	
				} 	
				
			}
			
		}
		pathList.add(0, routePath);
		return pathList;
	}
	
	


	
	/**Divides the networt into paths. Similar to deep-first graph search, but gives preference to to paths connected to the route*/ 
	private ArrayList<Path> createTrackList2() {
		pathList = new ArrayList<Path>();
		coEgedPathList = new ArrayList<Path>();
		ArrayList<StreetNode> visitingList = new ArrayList<StreetNode>(); /* Lista de visitação */
		int preOrderIndex, postOrderIndex;
		boolean allPointsVisited = false;
		boolean newTrack = false;/* to check if a new track/path is been build */
		StreetNode mainPoint, adjPoint;
		
		
		ArrayList<StreetNode> track = new ArrayList<StreetNode>();
		
		
		preOrderIndex = 0; /* ordem do empilhamento */
		postOrderIndex = 0; /* ordem do desempilhamento */
		
		/* inicializa o preordem e post ordem das paradas como -1* o s
		 * ou seja "não visitado" e "não finalizado"
		 */
		
		
		
		for(StreetNode s: streetNodeMap.values()){
			s.setPreOrder(-1);
			s.setPostOrder(-1);					
		}
		int mainPointId = route.getStart().getId();
		int bestPriority = streetNodeMap.get(mainPointId).getPriority();
		while(!allPointsVisited){
			
			StreetNode node = streetNodeMap.get(mainPointId);
			node.setPreOrder(++preOrderIndex);
			
			visitingList.add(node);
			
			while (!visitingList.isEmpty()){
				//mainPoint = getBestNode(visitingList);
				/*if is a new track then give preference to start a track connected to the route*/
				if(!newTrack)
					mainPoint =	visitingList.get(visitingList.size() -1);
				else
					mainPoint = getBestNode(visitingList); /*select as mainpoint(first node) a node in the route*/
				
				track.add(mainPoint);
				mainPointId = mainPoint.getId();
				boolean achou = false;
				/*encontra uma parada adjacente ainda não  visitada */
				/*Ordena lista de adjacencia para achar ponto que forma mernor angulo*/
				if(track.size() > 1 && adjacencyList.get(mainPointId).size() > 2 )
					putLowestBendOnTop(track.get(track.size() - 2), mainPoint, adjacencyList.get(mainPointId));
					

				/*Busca parada adjacent não visitada, e empilha. 
				  Terá preferencia paradas no topo da lista de adjacencia da ultima parara visitada(mainPoint)
				  Interacao interronpida com break se achar parada*/
				
				for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
					adjPoint = adjacencyList.get(mainPointId).get(i);
					if(adjPoint.getPreOrder() < 0 && achou == false){
						
						visitingList.add(adjPoint);
						adjPoint.setPreOrder(++preOrderIndex);
						achou = true;
						newTrack = false;
						break;
						
					}

				
				}
				/*Se não achar nenhum adjacente finaliza track, e cria complementar-edge*/
				if(achou == false){
									
					/*procura nós que fechem a track, e cria coedges tracks*/
					int j = 0;
					for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
						adjPoint = adjacencyList.get(mainPointId).get(i);
//						if(adjPoint.getPostOrder() < 0 &&							
//								adjPoint != visitingList.get(visitingList.size() -2)){						
						if(adjPoint.getPostOrder() < 0 &&
								 track.size()>1 &&
								adjPoint != track.get(track.size() -2)){
							
							j++;
							/* adiciona primeiro adjacente (ja vsitado) e fecha circulo */
							if(j==1){
								
								track.add(adjPoint);
								pathList.add(new Path(track));
								//coEgedPathList.remove(coEgedPathList.size() - 1);
								
							
							}
							/*give up trying to find the co-edges*/
							/* cria coedge para os outros*/
//							else{
//								ArrayList<StreetNode> coEdgetrack = new ArrayList<StreetNode>();
//								coEdgetrack.add( mainPoint );
//								coEdgetrack.add( adjPoint );	
//								coEgedPathList.add(new Path(coEdgetrack));
//								
//								/*gambi para saber quantas coEdge*/
//							}
							
						}

					}

					/*se não há nó adjacente visitado fecha a track*/
					if(j==0){
						if(track.size()>1)
							pathList.add(new Path(track));
					}

	
					/*desempilha o nó e recomeça a track*/
					streetNodeMap.get(mainPointId).setPostOrder(++postOrderIndex);
					visitingList.remove(mainPoint);
					track = new ArrayList<StreetNode>();	
					newTrack = true;
					
										
				}			
			}
			/* Check if unconnect parts of the graph has been visited */

			bestPriority = 100;
			allPointsVisited = true;
			for(StreetNode n: streetNodeMap.values()){
			
			
				
				preOrderIndex = 0; /* ordem do empilhamento */
				postOrderIndex = 0; /* ordem do desempilhamento */
				/*index primeira parada com menor grau*/
				if (n.getPreOrder() < 0){
					
					if(n.getPriority() < bestPriority){
						bestPriority = n.getPriority();
						mainPointId = n.getId();
					}
					allPointsVisited = false;
				   	
				} 	
				
			}
			
		}
		
		return pathList;
	}
	/**check in the visiting List if there is a route node with non-visited adjacent nodes*/
	private StreetNode getBestNode(ArrayList<StreetNode> visitingList) {
		boolean achou = false;
		StreetNode best = visitingList.get(visitingList.size()-1);
		if(!best.isRouteNode()){
			for(int i = visitingList.size()-2 ; i >= 0 ; i--){
				if(visitingList.get(i).isRouteNode()){
					for(int j = 0; j < adjacencyList.get(visitingList.get(i).getId()).size(); j++){
						
						if(adjacencyList.get(visitingList.get(i).getId()).get(j).getPreOrder() < 0){
							best = visitingList.get(i);
							achou = true;
							break;
							
						}
						
					}

				}
					
			}
			
		}
		
		return best;
	}
	
	private StreetNode getBestNodeLandmarkPreference(ArrayList<StreetNode> visitingList, boolean foundLandmark) {

		foundLandmark = false;



		StreetNode best = visitingList.get(visitingList.size() -1);

		for(int i = 0 ; i < visitingList.size() ; i++){
			if(visitingList.get(i).isRouteNode() && visitingList.get(i).getIsPolygonNode()>0) {
				
				foundLandmark = true;
				return visitingList.get(i);
			}
			else if( visitingList.get(i).getIsPolygonNode()>0 && !visitingList.get(i).isTopoCrossing() ) {
				best = visitingList.get(i);
				foundLandmark = true;
			}	
			else if(visitingList.get(i).isRouteNode()   && !foundLandmark )
				best = visitingList.get(i);

		}




		return best;
	}



	private ArrayList<Path> createTrackList() {
		/* ALGORITMO BUSCA ARESTA EM PROFUNDIDADE : INICIO*/
		pathList = new ArrayList<Path>();
		coEgedPathList = new ArrayList<Path>();
		
		
		Stack<StreetNode> S = new Stack<StreetNode>(); /* Pilha de visitação dos pontos */
		int preOrderIndex, postOrderIndex;
		boolean allPointsVisited = false;
		StreetNode mainPoint, adjPoint;

		ArrayList<StreetNode> track = new ArrayList<StreetNode>();
		
		
		preOrderIndex = 0; /* ordem do empilhamento */
		postOrderIndex = 0; /* ordem do desempilhamento */
		
		/* inicializa o preordem e post ordem das paradas como -1* o s
		 * ou seja "não visitado" e "não finalizado"
		 */
		
		int mainPointId = route.getStart().getId();
		int bestPriority = streetNodeMap.get(mainPointId).getPriority();

		for(StreetNode s: streetNodeMap.values()){

			s.setPreOrder(-1);
			s.setPostOrder(-1);
			
			
		}
		
		
		
		while(!allPointsVisited){
		
			/*empilha primeira parada*/
			
			
			StreetNode point = streetNodeMap.get(mainPointId);
			point.setPreOrder(++preOrderIndex);
			S.push(point);
			
			while (!S.isEmpty()){
				mainPoint = S.peek();
				track.add(mainPoint);
				mainPointId = mainPoint.getId();
				boolean achou = false;
				/*encontra uma parada adjacente ainda não  visitada */

				/*Ordena lista de adjacencia para achar ponto que forma mernor angulo*/
				if(track.size() > 1 && adjacencyList.get(mainPointId).size() > 2 )
					putLowestBendOnTop(track.get(track.size() - 2), mainPoint, adjacencyList.get(mainPointId));
					
//					orderAdjacentsStationListByLowestBend(track.get(track.size() - 2), mainPoint, adjacencyList.get(mainPointId), 
//							0, adjacencyList.get(mainPointId).size() - 1 );

				
				/*Busca parada adjacent não visitada, e empilha. 
				  Terá preferencia paradas no topo da lista de adjacencia da ultima parara visitada(mainPoint)
				  Interacao interronpida com break se achar parada*/
				
				for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
					adjPoint = adjacencyList.get(mainPointId).get(i);
					if(adjPoint.getPreOrder() < 0){
						
						S.push(adjPoint);
						adjPoint.setPreOrder(++preOrderIndex);
						achou = true;
						break;
					}
					
				}
				/*Se não achar nenhum adjacente finaliza track, e cria complementar-edge*/
				if(achou == false){
									
					/*procura nós que fechem a track, e cria coedges tracks*/
					int j = 0;
					for(int i = 0; i < adjacencyList.get(mainPointId).size(); i++){
						adjPoint = adjacencyList.get(mainPointId).get(i);
						if(adjPoint.getPostOrder() < 0 &&
								adjPoint != S.get(S.size() -2) ){
							j++;
							/* adiciona primeiro adjacente (ja vsitado) e fecha circulo */
							if(j==1){
								track.add(adjPoint);
								pathList.add(new Path(track));
							
							}
							/* cria coedge para os outros*/
							else{
								ArrayList<StreetNode> coEdgetrack = new ArrayList<StreetNode>();
								
								
								coEdgetrack.add( mainPoint );
								
	
								coEdgetrack.add( adjPoint );
								
								coEgedPathList.add(new Path(coEdgetrack));
								
								/*gambi para saber quantas coEdge*/
							}
							
						}
	
					}
					/*se não há nó adjacente visitado fecha a track*/
					if(j==0){
						if(track.size()>1)
							pathList.add(new Path(track));
					}
	
					/*desempilha o nó e recomeça a track*/
					streetNodeMap.get(mainPointId).setPostOrder(++postOrderIndex);
					point = mainPoint;
					
					track = new ArrayList<StreetNode>();	
					S.pop();
					
					
				}
		
				
			}
		
			/* Check if unconnect parts of the graph has been visited */

			bestPriority = 100;
			allPointsVisited = true;
			for(StreetNode node: streetNodeMap.values()){
			
			
				
				preOrderIndex = 0; /* ordem do empilhamento */
				postOrderIndex = 0; /* ordem do desempilhamento */
				/*index primeira parada com menor grau*/
				if (node.getPreOrder() < 0){
					
					if(node.getPriority() < bestPriority){
						bestPriority = node.getPriority();
						mainPointId = node.getId();
					}
					allPointsVisited = false;
				   	
				} 	
				
			}
			
		}
		/* ALGORITMO BUSCA ARESTA EM PROFUNDIDADE : FINAL*/
		
		
		return pathList;
	}
	
	private void putLowestBendOnTop(StreetNode previousNode, StreetNode mainPoint, ArrayList<StreetNode> adjList) {
		double closestTo180Angle = 0;
		int lowestAngleIndex = 0;
		boolean nonVisitedRouteNode = false;
		
		int i = 0;
		for(StreetNode adjNode: adjList){
			
			if (adjNode.getPreOrder() < 0){
				/*checke if there is non visited route node*/
				if(mainPoint.isRouteNode() && adjNode.isRouteNode()){
					nonVisitedRouteNode = true;
					break;
				}
				/**node can be from more than 1 polygon*/
				else if(adjNode.getIsPolygonNode() > 0 &&
						mainPoint.getIsPolygonNode() == previousNode.getIsPolygonNode() &&
						adjNode.getIsPolygonNode() == mainPoint.getIsPolygonNode()){
					lowestAngleIndex = i;
					break;
				}
				
				else{
					double angle = GeometricOperation.getAngleBetweenVectors(mainPoint.getGeom().getX(), mainPoint.getGeom().getY(),
							previousNode.getGeom().getX(), previousNode.getGeom().getY(),
							adjNode.getGeom().getX(), adjNode.getGeom().getY());
					angle = Math.toDegrees(angle);
					if(Math.abs(angle - 180) < Math.abs(closestTo180Angle - 180)){
						lowestAngleIndex = i;
						closestTo180Angle = angle;
						
					}
				}
			}
			
			i++;
		}
		if(!nonVisitedRouteNode){
			StreetNode bestAngleNode = adjList.get(lowestAngleIndex);
			adjList.remove(lowestAngleIndex);
			adjList.add(0, bestAngleNode);
		}	
		
	}


	private void bestToClosePathOnTop(StreetNode previousNode, StreetNode mainPoint, ArrayList<StreetNode> adjList) {
		double closestTo180Angle = 0;
		int lowestAngleIndex = 0;
		boolean nonVisitedRouteNode = false;
		
		int i = 0;
		for(StreetNode adjNode: adjList){
			
			if (adjNode.getPostOrder() < 0){
				/*checke if there is non visited route node*/
				if(mainPoint.isRouteNode() && adjNode.isRouteNode()){
					nonVisitedRouteNode = true;
					break;
				}
				/**node can be from more than 1 polygon*/
				else if(adjNode.getIsPolygonNode() > 0 &&
						mainPoint.getIsPolygonNode() == previousNode.getIsPolygonNode() &&
						adjNode.getIsPolygonNode() == mainPoint.getIsPolygonNode()){
					lowestAngleIndex = i;
					break;
				}
				
				else{
					double angle = GeometricOperation.getAngleBetweenVectors(mainPoint.getGeom().getX(), mainPoint.getGeom().getY(),
							previousNode.getGeom().getX(), previousNode.getGeom().getY(),
							adjNode.getGeom().getX(), adjNode.getGeom().getY());
					angle = Math.toDegrees(angle);
					if(Math.abs(angle - 180) < Math.abs(closestTo180Angle - 180)){
						lowestAngleIndex = i;
						closestTo180Angle = angle;
						
					}
				}
			}
			
			i++;
		}
		if(!nonVisitedRouteNode){
			StreetNode bestAngleNode = adjList.get(lowestAngleIndex);
			adjList.remove(lowestAngleIndex);
			adjList.add(0, bestAngleNode);
		}	
		
	}

	private void orderAdjacentsStationListByLowestBend(StreetNode previusAdjacentPoint, StreetNode mainPoint,  ArrayList<StreetNode> adjList, int lowIndex, int highIndex) {
		
		if( highIndex > lowIndex){

			double closestTo180Angle = 0;
			int lowestAngleIndex = 0;
			StreetNode auxNode;
			
			for(int i = lowIndex; i <= highIndex; i++){
				double angle = GeometricOperation.getAngleBetweenVectors(mainPoint.getGeom().getX(), mainPoint.getGeom().getY(),
						previusAdjacentPoint.getGeom().getX(), previusAdjacentPoint.getGeom().getY(),
						adjList.get(i).getGeom().getX(), adjList.get(i).getGeom().getY());
				angle = Math.toDegrees(angle);

				if(Math.abs(angle - 180) < Math.abs(closestTo180Angle - 180)){
					lowestAngleIndex = i;
					closestTo180Angle = angle;
					
				}
				
			}
			if( !adjList.get(lowIndex).isRouteNode()  ){
				auxNode = adjList.get(lowestAngleIndex);
				adjList.set(lowestAngleIndex, adjList.get(lowIndex) );
				adjList.set(lowIndex, auxNode);
			}
			orderAdjacentsStationListByLowestBend(previusAdjacentPoint,  mainPoint,  adjList, lowIndex + 1, highIndex );
		}
		else return;
		
		
	}
	
	private void orderAdjacentsStationListByRouteNodeOrLowestBend(StreetNode previusAdjacentPoint, StreetNode mainPoint,  ArrayList<StreetNode> adjList, int lowIndex, int highIndex) {
		
		if( highIndex > lowIndex){

			double closestTo180Angle = 0;
			int lowestAngleIndex = 0;
			StreetNode auxStation;
			
			for(int i = lowIndex; i <= highIndex; i++){
				double angle = GeometricOperation.getAngleBetweenVectors(mainPoint.getGeom().getX(), mainPoint.getGeom().getY(),
						previusAdjacentPoint.getGeom().getX(), previusAdjacentPoint.getGeom().getY(),
						adjList.get(i).getGeom().getX(), adjList.get(i).getGeom().getY());
				angle = (int)Math.toDegrees(angle);

				if(Math.abs(angle - 180) < (int)Math.abs(closestTo180Angle - 180)){
					lowestAngleIndex = i;
					closestTo180Angle = angle;
					
				}
				
			}
			if( !adjList.get(lowIndex).isRouteNode() || adjList.get(lowestAngleIndex).isRouteNode() ){
				auxStation = adjList.get(lowestAngleIndex);
				adjList.set(lowestAngleIndex, adjList.get(lowIndex) );
				adjList.set(lowIndex, auxStation);
			}
			orderAdjacentsStationListByLowestBend(previusAdjacentPoint,  mainPoint,  adjList, lowIndex + 1, highIndex );
		}
		else return;
		
		
	}
	
	
//	public boolean reProjectTO(String CRS){
//		for(StreetNode n: streetNodeMap.values()){
//			n.setGeom((Point)GeoConvertionsOperations.convertProjection(n.getGeom(), CRS));
//			
//		}
//		route.setGeom((LineString)GeoConvertionsOperations.convertProjection(route.getGeom(), CRS));
//		
//		return true;
//	}
	public Map<Integer, StreetNode> getStreetNodeMap() {
		return streetNodeMap;
	}
	public void setStreetNodeMap(Map<Integer, StreetNode> streetNodeMap) {
		this.streetNodeMap = streetNodeMap;
	}
	public Map<Integer, ArrayList<StreetNode>> getAdjacencyList() {
		return adjacencyList;
	}
	public void setAdjacencyList(Map<Integer, ArrayList<StreetNode>> adjacencyList) {
		this.adjacencyList = adjacencyList;
	}
	public Route getRoute() {
		return route;
	}
	public void setRoute(Route route) {
		this.route = route;
	}
	public StreetNetworkTopological getStreetNetwork() {
		return streetNetwork;
	}
	public void setStreetNetwork(StreetNetworkTopological streetNetwork) {
		this.streetNetwork = streetNetwork;
	}
//	public SimpleFeatureType getLineStringType() {
//		return lineStringType;
//	}
//	public void setLineStringType(SimpleFeatureType lineStringType) {
//		this.lineStringType = lineStringType;
//	}

	
	public ArrayList<PolygonalFeature> getPolygonalFeatureList() {
		return polygonalFeatureList;
	}

	public void setPolygonalFeatureList(ArrayList<PolygonalFeature> polygonalFeatureList) {
		this.polygonalFeatureList = polygonalFeatureList;
	}



	public ArrayList<Path> getPathList() {
		return pathList;
	}



	public void setPathList(ArrayList<Path> pathList) {
		this.pathList = pathList;
	}



	public ArrayList<Path> getCoEgedPathList() {
		return coEgedPathList;
	}



//	public ArrayList<Path> getRouteAdjPathList() {
//		return routeAdjPathList;
//	}



	public void setRouteAdjPathList(ArrayList<Path> routeAdjPathList) {
		this.routeAdjPathList = routeAdjPathList;
	}


	public void setCoEgedPathList(ArrayList<Path> coEgedPathList) {
		this.coEgedPathList = coEgedPathList;
	}






	public ArrayList<Path> getStreetOnlyPathList() {
		return streetOnlyPathList;
	}



	public void setStreetOnlyPathList(ArrayList<Path> streetOnlyPathList) {
		this.streetOnlyPathList = streetOnlyPathList;
	}



	public ArrayList<LinearFeature> getLinearFeatureList() {
		return linearFeatureList;
	}



	public void setLinearFeatureList(ArrayList<LinearFeature> linearFeatureList) {
		this.linearFeatureList = linearFeatureList;
	}



	public ArrayList<PointFeature> getPointFeatureList() {
		return pointFeatureList; 
	}



	public void setPointFeatureList(ArrayList<PointFeature> pointFeatureList) {
		this.pointFeatureList = pointFeatureList;
	}



	public ArrayList<PointTopo> getPointTopoList() {
		return pointTopoList;
	}


	public void setPointTopoList(ArrayList<PointTopo> pointTopoList) {
		this.pointTopoList = pointTopoList;
	}


	public ArrayList<PolygonalTopo> getPolygonalTopoList() {
		return polygonalTopoList;
	}



	public void setPolygonalTopoList(ArrayList<PolygonalTopo> polygonalTopoList) {
		this.polygonalTopoList = polygonalTopoList;
	}



	public Map<Integer, ArrayList<StreetNode>> getCircularOrderList() {
		return circularOrderList;
	}



	public void setCircularOrderList(Map<Integer, ArrayList<StreetNode>> circularOrderList) {
		this.circularOrderList = circularOrderList;
	}


	public Envelope getNetworkEnvelop() {
		return networkEnvelop;
	}



	public ArrayList<Path> getRouteAdjPathList() {
		return routeAdjPathList;
	}


	public double getMaxRouteProjectedEnvExtention() {
		return maxRouteProjectedEnvExtention;
	}




	public void setMaxRouteProjectedEnvExtention(double maxRouteProjectedEnvExtention) {
		this.maxRouteProjectedEnvExtention = maxRouteProjectedEnvExtention;
	}




	public void setNetworkEnvelop(Envelope networkEnvelop) {
		this.networkEnvelop = networkEnvelop;
	}




	public double getRouteRescaleLengthProportion() {
		return routeRescaleLengthProportion;
	}




	public void setRouteRescaleLengthProportion(double routeRescaleLengthProportion) {
		this.routeRescaleLengthProportion = routeRescaleLengthProportion;
	}


	public LineString getRescaledRouteGeom() {
		return rescaledRouteGeom;
	}



	public void setRescaledRouteGeom(LineString rescaledRouteGeom) {
		this.rescaledRouteGeom = rescaledRouteGeom;
	}
	
	

}
