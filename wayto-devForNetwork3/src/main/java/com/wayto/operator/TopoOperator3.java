package com.wayto.operator;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import org.codehaus.jettison.json.JSONArray;
import org.codehaus.jettison.json.JSONException;
import org.codehaus.jettison.json.JSONObject;
import org.geotools.geometry.jts.Geometries;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.locationtech.jts.algorithm.ConvexHull;
import org.locationtech.jts.densify.Densifier;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.IntersectionMatrix;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.simplify.DouglasPeuckerSimplifier;

import com.wayto.model.LinearFeature;
import com.wayto.model.Path;
import com.wayto.model.PointFeature;
import com.wayto.model.PolygonalFeature;
import com.wayto.model.Route;
import com.wayto.model.RouteEdge;
import com.wayto.model.StreetEdge;
import com.wayto.model.StreetNetworkTopological;
import com.wayto.model.StreetNode;
import com.wayto.model.topo.NodeTopoRelation;
import com.wayto.model.topo.PointTopo;
import com.wayto.model.topo.PolygonalTopo;

public class TopoOperator3 {



	public static void makePointLMTopo(ArrayList<PointFeature> pointFeatureList, ArrayList<PointTopo> pointTopoList, Route route, 
			Map<Integer, StreetNode> streetNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList,
			StreetNetworkTopological streetNetwork) {
		
		boolean validPointLM = false;

		int newId = Integer.MAX_VALUE;
		
		for(PointFeature p: pointFeatureList) {
			
			boolean idNotFound = true;
			while(idNotFound){
				if(streetNodeMap.containsKey(newId))
					newId--;
				else
					idNotFound = false;
			}
			
			StreetNode n = new StreetNode(newId, p.getGeom(), false, 1, true) ;
			n.setIsPointLMNode(p.getId());
			n.setStreetNode(false);
			streetNodeMap.put(n.getId(), n);
			adjacencyList.put(n.getId(), new ArrayList<StreetNode>());
			
			PointTopo pointFeatureTopological = new PointTopo(p, n);
			
			validPointLM = buildPointLMControlEdge(pointFeatureTopological, route, adjacencyList, streetNodeMap, streetNetwork);
			
			if(validPointLM)
				pointTopoList.add(pointFeatureTopological);
			else {
				System.out.println("Excluded Point LM: " + pointFeatureTopological.getPointFeature().getName() + " dist: " + pointFeatureTopological.getDistToRoute());
			}
		}
		
	}
	
	public static void makePolygonsTopo(ArrayList<PolygonalFeature> polygonalFeatureList,
			ArrayList<PolygonalTopo> polygonalTopoList, Map<Integer, StreetNode> streetNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork) {
		int newEdgeId = Integer.MAX_VALUE;
		int newId = Integer.MAX_VALUE;
		
		for (PolygonalFeature p: polygonalFeatureList) {
			/**simplify polygon geometry**/
			
			
			
//			PolyLineLayer convexHullLayer = new PolyLineLayer("ConvexPath " + i , 8, 2, Color.ORANGE, Color.ORANGE, true, true);	
//			convexHullLayer.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(ch.getConvexHull()));
//			xMap.getMapContent().getLayers().add(convexHullLayer);
//			float g = 0; /* any number 0-10*/
			Envelope sEnv = p.getGeocoordsGeom().getExteriorRing().getEnvelopeInternal();
			double diagonal = Math.sqrt( Math.pow(sEnv.getHeight(), 2)  +  Math.pow(sEnv.getWidth(), 2));
//			double tolerance = diagonal*(Math.pow((g-1)/10, 3))/(80/(g));
//			LineString simplifiedPolygon = (LineString) DouglasPeuckerSimplifier.simplify(p.getGeom().getExteriorRing(), tolerance);
//			simplifiedPolygon = (LineString)Densifier.densify(simplifiedPolygon, diagonal/5);
			
			LineString simplifiedPolygon = p.getGeocoordsGeom().getExteriorRing();		
//			ConvexHull ch = new ConvexHull(simplifiedPolygon);
//			simplifiedPolygon = GeoConvertionsOperations.Java2DToJTSLineString(GeoConvertionsOperations.JTSGeometryToJavaD2(ch.getConvexHull()));
			simplifiedPolygon = (LineString)Densifier.densify(simplifiedPolygon, diagonal/5);
			
			
			//p.setGeom(simplifiedPolygon);
			ArrayList<StreetNode> polyPath = new ArrayList<StreetNode>();
			ArrayList<StreetEdge> polygonEdgeList = new ArrayList<StreetEdge>();

			for(int i = 0; i < simplifiedPolygon.getNumPoints(); i++){
				
				boolean idNotFound = true;
				while(idNotFound){
					if(streetNodeMap.containsKey(newId))
						newId--;
					else
						idNotFound = false;
				}
				
				/**create id, node,  edge and add to the structures*/
				StreetNode n = new StreetNode(newId, simplifiedPolygon.getPointN(i), false, 1, p.getId(), false, true) ;
				polyPath.add(n);
				streetNodeMap.put(n.getId(), n);
				adjacencyList.put(n.getId(), new ArrayList<StreetNode>());
				if(i!=0){
					idNotFound = true;
					while(idNotFound){
						if(streetNetwork.getEdges().containsKey(newEdgeId))
							newEdgeId--;
						else
							idNotFound = false;
					}
					StreetEdge e = new StreetEdge(newEdgeId, p.getName(), polyPath.get(i - 1), n, p.getId());
					polygonEdgeList.add(e);
					streetNetwork.getEdges().put(e.getId(),e);
					adjacencyList.get(polyPath.get(i - 1).getId()).add(n);
					adjacencyList.get(n.getId()).add(polyPath.get(i - 1));
					
				}
				/**treat last point of polygon: connect to the first*/
				/*ATTENTION: IF IS CLOSE POLYLINE IS RIGHT, BUT IF IS OPEN IT NEED TO DISCONSIDER THE LAST EDGE*/
				if(i== simplifiedPolygon.getNumPoints() -1){
					idNotFound = true;
					while(idNotFound){
						if(streetNetwork.getEdges().containsKey(newEdgeId))
							newEdgeId--;
						else
							idNotFound = false;
					}
					StreetEdge e = new StreetEdge(newEdgeId, p.getName(), n , polyPath.get(0), p.getId());
					polygonEdgeList.add(e);
					streetNetwork.getEdges().put(e.getId(),e);
					adjacencyList.get(polyPath.get(0).getId()).add(n);
					adjacencyList.get(n.getId()).add(polyPath.get(0));
										
				}
				
				
				
			
			}
			PolygonalTopo polygonalFeatureTopological = new PolygonalTopo(p, polygonEdgeList);
			polygonalTopoList.add(polygonalFeatureTopological);
		}
		
	}
	
	public static void tellMeMorePointLandmarks(Route route, Map<Integer, StreetNode> streetNodeMap,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork,
			ArrayList<PointTopo> pointTopoList) {
		
		for (PointTopo p: pointTopoList) {
			System.out.println("Building Control Edgeds for : " + p.getPointFeature().getName());
			buildPointLMControlEdge(p, route, adjacencyList, streetNodeMap, streetNetwork);

			
		}
		
		
	}
	


	public static void tellMeMore2(Route route, Map<Integer, StreetNode> streetNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork, ArrayList<PolygonalTopo> polygonalTopoFormatList) {
		
		
		LineString routeLS = route.getRoutePath().asLineString(0);
		
		/*Buffer to identify what is really along*/ 
		
		double bufferSize = 250;
		Polygon routeBuffer = (Polygon)(routeLS.buffer(bufferSize));

		
		for (PolygonalTopo p: polygonalTopoFormatList) {
			
			/**** Route Crossing Planarization ****/
			Polygon polygonGeometry = p.asPolygon(0);
			Geometry intersection = routeLS.intersection(polygonGeometry.getExteriorRing()); 
			//Geometry intersection = route.getGeom().intersection(p.getGeom().getExteriorRing());
			boolean isPointLikeGeometry  = false;
			switch ( Geometries.get(intersection) ) {
			case LINESTRING:
				System.out.println("linestring");
				break;
			case POINT:
				System.out.println("point");
				isPointLikeGeometry = true;
				break;
			case MULTIPOINT:
				System.out.println("multipoint");
				isPointLikeGeometry = true;
				break;	
			default:
				break;


			}
			
			if(isPointLikeGeometry){
				/***POLYGON IS SIMPLE CROSSING****/
				if(intersection.getNumGeometries()%2 == 0){
					p.setType(PolygonalTopo.SIMPLE_CROSSING);
					System.out.println("route crosses " + p.getPolygonalFeature().getName());
				}
				/***route enter or leaves polygon*/			
				else if(intersection.getNumGeometries()%2 == 1){

					/*route leaves this polygon*/
					if(routeLS.getStartPoint().within(polygonGeometry)){
						p.setType(PolygonalTopo.ROUTE_STARTS_AT);
						System.out.println("route starts at polygon " + p.getPolygonalFeature().getName());

					}
					/*route ends in this polygon*/
					else if(routeLS.getEndPoint().within(polygonGeometry)){
						p.setType(PolygonalTopo.ROUTE_ENDS_AT);
						System.out.println("route ends at polygon " + p.getPolygonalFeature().getName());

					}

				}
				
			}
			/***Check ALONG */
			else if(intersection.isEmpty()){
				if( polygonGeometry.contains(routeLS) ){
					System.out.println("route is inside polygon " + p.getPolygonalFeature().getName());
					p.setType(PolygonalTopo.ROUTE_IS_INSIDE);
				}	
				else if(routeBuffer.intersects(polygonGeometry)){
					System.out.println("route goes along " + p.getPolygonalFeature().getName());
					p.setType(PolygonalTopo.PASSING_ALONG_UNKNOW);
				}
				else{
					System.out.println("Global landmark: " + p.getPolygonalFeature().getName());
					p.setType(PolygonalTopo.GLOBAL);
				}
				
			}
			
			
		}
		for (PolygonalTopo p: polygonalTopoFormatList) {
			planarizePolygon(p, route, adjacencyList, streetNodeMap, streetNetwork );
		
		}
		for (PolygonalTopo p: polygonalTopoFormatList) {
			if(p.getType() == PolygonalTopo.PASSING_ALONG_UNKNOW ){
				buildPolyControlEdges(p, route, 100 , adjacencyList, streetNodeMap, streetNetwork);
			}
			if(p.getType() == PolygonalTopo.ROUTE_STARTS_AT || p.getType() == PolygonalTopo.ROUTE_ENDS_AT) {
				buildStartEndPolyControlEdges(p, route,  adjacencyList, streetNodeMap, streetNetwork);
			} 
			if(	p.getType() == PolygonalTopo.GLOBAL || p.getType() == PolygonalTopo.ROUTE_IS_INSIDE){
				buildPolyControlEdges(p, route, 20 , adjacencyList, streetNodeMap, streetNetwork );
			}

			
		}
		//addDummyNodesCloseToDP( route,	streetNodeMap, adjacencyList, streetNetwork, 0.0005);
		//addDummyNodesCloseToDPCrossesAndTopo(route, streetNodeMap, adjacencyList, streetNetwork, 0.0005);
		
	}

	/**Add extra vertices to edges of paths (landmark paths too?) adjacent to the route: new stub is edge min lenght is mindist**/
	public static void breakStubEdges(Map<Integer, StreetNode> streetNodeMap,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork,
			 double minDist) {


		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		int newEdgeId = Integer.MAX_VALUE;
		int newNodeId = Integer.MAX_VALUE;
		/**foir each path**/
		//chunckPahtAnalyses(pathList);


		StreetNetworkTopological tempStreetNetWork = new StreetNetworkTopological();

		try {
			tempStreetNetWork = streetNetwork.clone();
		} catch (CloneNotSupportedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		for(StreetEdge e: streetNetwork.getEdges().values()) {
			

			Point newPoint = null;

			if( e.getSourcePoint().isRouteNode() && !e.getTargetPoint().isRouteNode() && adjacencyList.get(e.getTargetPoint().getId()).size() > 2) {
				double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getGeom().getY(),
						e.getTargetPoint().getGeom().getX(), e.getTargetPoint().getGeom().getY());

				if(e.getSourcePoint().getGeom().distance(e.getTargetPoint().getGeom()) > minDist*1.1){

					double newX = e.getSourcePoint().getGeom().getX() + minDist*Math.cos(angle);
					double newY = e.getSourcePoint().getGeom().getY() - minDist*Math.sin(angle);

					newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
				}
				else{
					double dist = 0.5*e.getSourcePoint().getGeom().distance(e.getTargetPoint().getGeom() );
					double newX = e.getSourcePoint().getGeom().getX() + dist*Math.cos(angle);
					double newY = e.getSourcePoint().getGeom().getY() - dist*Math.sin(angle);


					newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
				}
			}
			else if (e.getTargetPoint().isRouteNode() && !e.getSourcePoint().isRouteNode() &&  adjacencyList.get(e.getSourcePoint().getId()).size() > 2){
				double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(e.getTargetPoint().getGeom().getX(), e.getTargetPoint().getGeom().getY(),
						e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getGeom().getY());
				/*if edge adj. to long split in the minimal distance*/
				if(e.getTargetPoint().getGeom().distance(e.getSourcePoint().getGeom()) > minDist*1.1){

					double newX = e.getTargetPoint().getGeom().getX() + minDist*Math.cos(angle);
					double newY = e.getTargetPoint().getGeom().getY() - minDist*Math.sin(angle);

					newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
				}
				/*if edge adj. is short split in half/third*/
				else{
					double dist = 0.5*e.getTargetPoint().getGeom().distance(e.getSourcePoint().getGeom() );
					double newX = e.getTargetPoint().getGeom().getX() + dist*Math.cos(angle);
					double newY = e.getTargetPoint().getGeom().getY() - dist*Math.sin(angle);
					newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
				}

			}


			if(newPoint != null){
				boolean idNotFound = true;
				while(idNotFound){
					if(streetNodeMap.containsKey(newNodeId))
						newNodeId--;
					else
						idNotFound = false;
				}
				/*new node*/
				StreetNode n;
				if(e.getIsPolygonEdge() > 0)
					n = new StreetNode(newNodeId,newPoint, false, e.getClazz(), e.getIsPolygonEdge(), false, false );
				else
					n = new StreetNode(newNodeId,newPoint, false, e.getClazz(), 0, true, false );
				
				streetNodeMap.put(n.getId(), n);
				StreetEdge e1,e2;
				/*split edges*/
				try {
					e1 = (StreetEdge)e.clone();
					e2 = (StreetEdge)e.clone();
					/*ATENTION still need to se X1 X2 Y1 Y2 edge geom, cost etc 
					 * UPDATE AdjacencyList*/
					e1.setTargetPoint(n);


					e2.setSourcePoint(n);

					tempStreetNetWork.getEdges().remove(e.getId());
					idNotFound = true;			
					while(idNotFound){
						if(tempStreetNetWork.getEdges().containsKey(newEdgeId))
							newEdgeId--;
						else
							idNotFound = false;
					}
					e2.setId(newEdgeId);
					//System.out.println("add to network edge: " + e2.getId());
					tempStreetNetWork.getEdges().put(e2.getId(),e2);
					idNotFound = true;
					while(idNotFound){
						if(tempStreetNetWork.getEdges().containsKey(newEdgeId))
							newEdgeId--;
						else
							idNotFound = false;
					}
					e1.setId(newEdgeId);
					//System.out.println("add to network edge: " + e1.getId());
					tempStreetNetWork.getEdges().put(e1.getId(),e1);


					/*AdjancencyList update*/
					adjacencyList.put(n.getId(), new ArrayList<StreetNode>());

					adjacencyList.get(e.getSourceId()).remove(e.getTargetPoint());
					adjacencyList.get(e.getSourceId()).add(n);
					adjacencyList.get(e.getTargetId()).remove(e.getSourcePoint());
					adjacencyList.get(e.getTargetId()).add(n);

					adjacencyList.get(n.getId()).add(e.getTargetPoint());
					adjacencyList.get(n.getId()).add(e.getSourcePoint());

					/*if it is polygonal edge, update polygonTopo edge lsit*/
//					if (e.getIsPolygonEdge() > 0  ){
//						for(PolygonalTopo pt: polygonalTopoList){
//							if(pt.getPolygonalFeature().getId() == e.getIsPolygonEdge()){
//								System.out.println("edge is polytopo " + pt.getPolygonalFeature().getName());
//								int edgeIndex = pt.getEdgeList().indexOf(e);
//								pt.getEdgeList().remove(edgeIndex);
//								pt.getEdgeList().add(edgeIndex, e2);
//								pt.getEdgeList().add(edgeIndex, e1);
//
//							}
//
//						}
//					}


				} catch (CloneNotSupportedException e3) {
					// TODO Auto-generated catch block
					e3.printStackTrace();
				}
			}


		}
		streetNetwork.setEdges( tempStreetNetWork.getEdges() );




	}


	/**Add extra vertices to edges of paths (landmark paths too?) adjacent to the route: new stub is edge min lenght is mindist**/
	public static void adjacentEdgesAnalyses(ArrayList<Path> pathList, Map<Integer, StreetNode> streetNodeMap,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork,
			ArrayList<PolygonalTopo> polygonalTopoList, double minDist) {

		
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();
		int newEdgeId = Integer.MAX_VALUE;
		int newNodeId = Integer.MAX_VALUE;
		/**foir each path**/
		//chunckPahtAnalyses(pathList);
		
		for(Path p: pathList){
			//if(!p.isRoute() && !p.isChunkPath()){
			if(!p.isRoute() ){
				StreetNode n;
				/*if path start at the route*/
				if(p.getNodeList().get(0).isRouteNode() ){

					StreetEdge e = streetNetwork.getEdge(p.getNodeList().get(0), p.getNodeList().get(1));
					if(!e.isFakeEdge()){
						Point newPoint = null;

						/*if sourcenode of the edge adj. */
						if(e.getSourcePoint().isRouteNode()){
							double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getGeom().getY(),
									e.getTargetPoint().getGeom().getX(), e.getTargetPoint().getGeom().getY());
							if(e.getSourcePoint().getGeom().distance(e.getTargetPoint().getGeom()) > minDist*1.1){

								double newX = e.getSourcePoint().getGeom().getX() + minDist*Math.cos(angle);
								double newY = e.getSourcePoint().getGeom().getY() - minDist*Math.sin(angle);

								newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
							}
							else{
								double dist = 0.5*e.getSourcePoint().getGeom().distance(e.getTargetPoint().getGeom() );
								double newX = e.getSourcePoint().getGeom().getX() + dist*Math.cos(angle);
								double newY = e.getSourcePoint().getGeom().getY() - dist*Math.sin(angle);


								newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
							}
						}
						/*if targetnode of the edge adj. */
						else if (e.getTargetPoint().isRouteNode()){
							double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(e.getTargetPoint().getGeom().getX(), e.getTargetPoint().getGeom().getY(),
									e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getGeom().getY());
							/*if edge adj. to long split in the minimal distance*/
							if(e.getTargetPoint().getGeom().distance(e.getSourcePoint().getGeom()) > minDist*1.1){

								double newX = e.getTargetPoint().getGeom().getX() + minDist*Math.cos(angle);
								double newY = e.getTargetPoint().getGeom().getY() - minDist*Math.sin(angle);

								newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
							}
							/*if edge adj. is short split in half/third*/
							else{
								double dist = 0.5*e.getTargetPoint().getGeom().distance(e.getSourcePoint().getGeom() );
								double newX = e.getTargetPoint().getGeom().getX() + dist*Math.cos(angle);
								double newY = e.getTargetPoint().getGeom().getY() - dist*Math.sin(angle);
								newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
							}


						}
						if(newPoint != null){
							boolean idNotFound = true;
							while(idNotFound){
								if(streetNodeMap.containsKey(newNodeId))
									newNodeId--;
								else
									idNotFound = false;
							}
							/*new node*/
							if(e.getIsPolygonEdge() > 0)
								n = new StreetNode(newNodeId,newPoint, false, e.getClazz(), p.getIsPolygon(), false, false );
							else
								n = new StreetNode(newNodeId,newPoint, false, e.getClazz(), p.getIsPolygon(), true, false );
							p.getNodeList().add(1, n);
							streetNodeMap.put(n.getId(), n);
							StreetEdge e1,e2;
							/*split edges*/
							try {
								e1 = (StreetEdge)e.clone();
								e2 = (StreetEdge)e.clone();
								/*ATENTION still need to se X1 X2 Y1 Y2 edge geom, cost etc 
								 * UPDATE AdjacencyList*/
								e1.setTargetPoint(n);


								e2.setSourcePoint(n);

								streetNetwork.getEdges().remove(e.getId());
								idNotFound = true;			
								while(idNotFound){
									if(streetNetwork.getEdges().containsKey(newEdgeId))
										newEdgeId--;
									else
										idNotFound = false;
								}
								e2.setId(newEdgeId);
								//System.out.println("add to network edge: " + e2.getId());
								streetNetwork.getEdges().put(e2.getId(),e2);
								idNotFound = true;
								while(idNotFound){
									if(streetNetwork.getEdges().containsKey(newEdgeId))
										newEdgeId--;
									else
										idNotFound = false;
								}
								e1.setId(newEdgeId);
								//System.out.println("add to network edge: " + e1.getId());
								streetNetwork.getEdges().put(e1.getId(),e1);


								/*AdjancencyList update*/
								adjacencyList.put(n.getId(), new ArrayList<StreetNode>());

								adjacencyList.get(e.getSourceId()).remove(e.getTargetPoint());
								adjacencyList.get(e.getSourceId()).add(n);
								adjacencyList.get(e.getTargetId()).remove(e.getSourcePoint());
								adjacencyList.get(e.getTargetId()).add(n);

								adjacencyList.get(n.getId()).add(e.getTargetPoint());
								adjacencyList.get(n.getId()).add(e.getSourcePoint());

								/*if it is polygonal edge, update polygonTopo edge lsit*/
								if (e.getIsPolygonEdge() > 0  ){
									for(PolygonalTopo pt: polygonalTopoList){
										if(pt.getPolygonalFeature().getId() == e.getIsPolygonEdge()){
											System.out.println("edge is polytopo" + pt.getPolygonalFeature().getName());
											int edgeIndex = pt.getEdgeList().indexOf(e);
											pt.getEdgeList().remove(edgeIndex);
											pt.getEdgeList().add(edgeIndex, e2);
											pt.getEdgeList().add(edgeIndex, e1);

										}

									}
								}


							} catch (CloneNotSupportedException e3) {
								// TODO Auto-generated catch block
								e3.printStackTrace();
							}
						}
					}


				}
				/*if last node is route node*/
				if(p.getNodeList().get(p.getNodeList().size()-1).isRouteNode() ){

					/*get last edge*/
					StreetEdge e = streetNetwork.getEdge(p.getNodeList().get(p.getNodeList().size()-1), p.getNodeList().get(p.getNodeList().size()-2));
					if(!e.isFakeEdge()){
						Point newPoint = null;
						/*if sourcenode of the edge adj. */
						if(e.getSourcePoint().isRouteNode()){
							double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getGeom().getY(),
									e.getTargetPoint().getGeom().getX(), e.getTargetPoint().getGeom().getY());
							if(e.getSourcePoint().getGeom().distance(e.getTargetPoint().getGeom()) > minDist*1.2){

								double newX = e.getSourcePoint().getGeom().getX() + minDist*Math.cos(angle);
								double newY = e.getSourcePoint().getGeom().getY() - minDist*Math.sin(angle);

								newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
							}
							else{
								double dist = 0.5*e.getSourcePoint().getGeom().distance(e.getTargetPoint().getGeom() );
								double newX = e.getSourcePoint().getGeom().getX() + dist*Math.cos(angle);
								double newY = e.getSourcePoint().getGeom().getY() - dist*Math.sin(angle);


								newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
							}
						}
						/*if targetnode of the edge adj. */
						else if (e.getTargetPoint().isRouteNode()){
							double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(e.getTargetPoint().getGeom().getX(), e.getTargetPoint().getGeom().getY(),
									e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getGeom().getY());
							/*if edge adj. to long split in the minimal distance*/
							if(e.getTargetPoint().getGeom().distance(e.getSourcePoint().getGeom()) > minDist*1.2){

								double newX = e.getTargetPoint().getGeom().getX() + minDist*Math.cos(angle);
								double newY = e.getTargetPoint().getGeom().getY() - minDist*Math.sin(angle);

								newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
							}
							/*if edge adj. is short split in half/third*/
							else {
								double dist = 0.5*e.getTargetPoint().getGeom().distance(e.getSourcePoint().getGeom() );
								double newX = e.getTargetPoint().getGeom().getX() + dist*Math.cos(angle);
								double newY = e.getTargetPoint().getGeom().getY() - dist*Math.sin(angle);
								newPoint = geometryFactory.createPoint(new Coordinate(newX, newY));
							}


						}	
						if(newPoint != null){
							boolean idNotFound = true;
							while(idNotFound){
								if(streetNodeMap.containsKey(newNodeId))
									newNodeId--;
								else
									idNotFound = false;
							}
							/*new node*/
							if(e.getIsPolygonEdge() > 0)
								n = new StreetNode(newNodeId,newPoint, false, e.getClazz(), p.getIsPolygon(), false, false );
							else
								n = new StreetNode(newNodeId,newPoint, false, e.getClazz(), p.getIsPolygon(), true , false);
							p.getNodeList().add(p.getNodeList().size()-1, n);
							streetNodeMap.put(n.getId(), n);

							StreetEdge e1,e2;
							/*split edges*/
							try {
								e1 = (StreetEdge)e.clone();
								e2 = (StreetEdge)e.clone();
								/*ATENTION still need to se X1 X2 Y1 Y2 edge geom, cost etc 
								 * UPDATE AdjacencyList*/
								e1.setTargetPoint(n);


								e2.setSourcePoint(n);

								streetNetwork.getEdges().remove(e.getId());
								idNotFound = true;			
								while(idNotFound){
									if(streetNetwork.getEdges().containsKey(newEdgeId))
										newEdgeId--;
									else
										idNotFound = false;
								}
								e2.setId(newEdgeId);
								//System.out.println("add to network edge: " + e2.getId());
								streetNetwork.getEdges().put(e2.getId(),e2);
								idNotFound = true;
								while(idNotFound){
									if(streetNetwork.getEdges().containsKey(newEdgeId))
										newEdgeId--;
									else
										idNotFound = false;
								}
								e1.setId(newEdgeId);
								//System.out.println("add to network edge: " + e1.getId());
								streetNetwork.getEdges().put(e1.getId(),e1);


								/*AdjancencyList update*/
								adjacencyList.put(n.getId(), new ArrayList<StreetNode>());

								adjacencyList.get(e.getSourceId()).remove(e.getTargetPoint());
								adjacencyList.get(e.getSourceId()).add(n);
								adjacencyList.get(e.getTargetId()).remove(e.getSourcePoint());
								adjacencyList.get(e.getTargetId()).add(n);

								adjacencyList.get(n.getId()).add(e.getTargetPoint());
								adjacencyList.get(n.getId()).add(e.getSourcePoint());

								/*if it is polygonal edge, update polygonTopo edge lsit*/
								if (e.getIsPolygonEdge() > 0){
									for(PolygonalTopo pt: polygonalTopoList){
										if(pt.getPolygonalFeature().getId() == e.getIsPolygonEdge()){
											System.out.println("edge is polytopo" + pt.getPolygonalFeature().getName());
											int edgeIndex = pt.getEdgeList().indexOf(e);
											pt.getEdgeList().remove(edgeIndex);
											pt.getEdgeList().add(edgeIndex, e2);
											pt.getEdgeList().add(edgeIndex, e1);

										}

									}
								}


							} catch (CloneNotSupportedException e3) {
								// TODO Auto-generated catch block
								e3.printStackTrace();
							}
						}
					}
				}


			}
		}	
	}

	/**Add street edge linestring geometry nodes */
	public static void refineEdges(Route route, Map<Integer, StreetNode> streetNodeMap,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork) {

		StreetNetworkTopological tempStreetNetWork = new StreetNetworkTopological();

		try {
			tempStreetNetWork = streetNetwork.clone();
		} catch (CloneNotSupportedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		for(StreetEdge e: streetNetwork.getEdges().values()) {

			System.out.println("Edge id: " + e.getId() + "  num Points: " + e.getGeom().getNumPoints()  );
			if( e.getSourcePoint().getCoordinate().distance(e.getGeom().getPointN(0)) > 0 ) {
				
				
				System.out.println("Edge source with wrong geometry: "+e.getId() + " Dont divide!");
				System.out.println("Source : " + e.getSourcePoint().getCoordinate() + "vs." + e.getGeom().getPointN(0));
				System.out.println("Target : " + e.getTargetPoint().getCoordinate() + "vs." + e.getGeom().getPointN(e.getGeom().getNumPoints()-1));
					if(e.getSourcePoint().getCoordinate().getX() != e.getGeom().getPointN(0).getX() ||
							e.getSourcePoint().getCoordinate().getY() != e.getGeom().getPointN(0).getY() ) {
						e.getSourcePoint().setCoordinate(e.getGeom().getPointN(0));
					}
					
					
						
					

				
			}
			if( e.getTargetPoint().getCoordinate().distance(e.getGeom().getPointN(e.getGeom().getNumPoints()-1)) > 0 ) {
				
				
				System.out.println("Edge target with wrong geometry: "+e.getId() + " Dont divide!");
				System.out.println("Source : " + e.getSourcePoint().getCoordinate() + "vs." + e.getGeom().getPointN(0));
				System.out.println("Target : " + e.getTargetPoint().getCoordinate() + "vs." + e.getGeom().getPointN(e.getGeom().getNumPoints()-1));
					
					if(e.getTargetPoint().getCoordinate().getX() != e.getGeom().getPointN(e.getGeom().getNumPoints()-1).getX() ||
					e.getTargetPoint().getCoordinate().getY() != e.getGeom().getPointN(e.getGeom().getNumPoints()-1).getY() ) {
						e.getTargetPoint().setCoordinate(e.getGeom().getPointN(e.getGeom().getNumPoints()-1)); 
					}
						
					

				
			}
			
			
			
			if( e.getGeom().getNumPoints()  > 2) 					
//				if(e.getSourcePoint().getCoordinate().getX() == e.getGeom().getPointN(0).getX() &&
//					e.getSourcePoint().getCoordinate().getY() == e.getGeom().getPointN(0).getY() &&
//					e.getTargetPoint().getCoordinate().getX() == e.getGeom().getPointN(e.getGeom().getNumPoints()-1).getX() &&
//					e.getTargetPoint().getCoordinate().getY() == e.getGeom().getPointN(e.getGeom().getNumPoints()-1).getY() )
				{
				System.out.println("Linestring start in source and ends in target");

				StreetEdge edgeToBreak =  e;
				StreetNode lastNode;

				if( e instanceof RouteEdge ) {
					
					for(int i = 1; i < edgeToBreak.getGeom().getNumPoints() -1 ; i++) {
						
						boolean idNotFound = true;
						int newNodeId = Integer.MAX_VALUE;
						while(idNotFound){
							if(streetNodeMap.containsKey(newNodeId))
								newNodeId--;
							else
								idNotFound = false;
						}
						StreetNode newRouteNode = new StreetNode(newNodeId, edgeToBreak.getGeom().getPointN(i) , true, 0, true);
						streetNodeMap.put(newRouteNode.getId(), newRouteNode);
						
						addCrossingNodeToRouteEdge(route, (RouteEdge)edgeToBreak, newRouteNode, adjacencyList, tempStreetNetWork);
						
						lastNode = newRouteNode;
						//System.out.println("Braking subedge " + i + ": " + edgeToBreak);
						edgeToBreak = tempStreetNetWork.getEdge(lastNode, edgeToBreak.getTargetPoint());


					}
					


					/**adding point in target coordinate**/
					
					
					

				}
				else if(edgeToBreak.isStubEdge() 
						&& (adjacencyList.get(edgeToBreak.getSourcePoint().getId()).size() == 1 || adjacencyList.get(edgeToBreak.getTargetPoint().getId()).size() == 1 )  ){
					System.out.println(" is stub to break");
					if(edgeToBreak.getSourcePoint().isRouteNode()) {
						lastNode = addNodeToEdge(edgeToBreak.getGeom().getPointN(1), true, edgeToBreak, adjacencyList, streetNodeMap, tempStreetNetWork);
						StreetEdge edgeToRemove = tempStreetNetWork.getEdge(lastNode, edgeToBreak.getTargetPoint());
						StreetNode nodeToRemove = edgeToBreak.getTargetPoint();

						adjacencyList.get(lastNode.getId()).remove(nodeToRemove);
						adjacencyList.get(nodeToRemove.getId()).remove(lastNode);
						tempStreetNetWork.getEdges().remove(edgeToRemove.getId() ,edgeToRemove );
						/*Remove node only if degree is 1*/
						if(adjacencyList.get(nodeToRemove.getId()).isEmpty()){
							streetNodeMap.remove(nodeToRemove.getId() , nodeToRemove);
							adjacencyList.remove(nodeToRemove.getId());
						}


					}
					else if(edgeToBreak.getTargetPoint().isRouteNode()) {
						lastNode = addNodeToEdge(edgeToBreak.getGeom().getPointN(edgeToBreak.getGeom().getNumPoints() -2), true, edgeToBreak, adjacencyList, streetNodeMap, tempStreetNetWork);							
						StreetEdge edgeToRemove = tempStreetNetWork.getEdge(lastNode, edgeToBreak.getSourcePoint());
						StreetNode nodeToRemove = edgeToBreak.getSourcePoint();

						adjacencyList.get(lastNode.getId()).remove(nodeToRemove);
						adjacencyList.get(nodeToRemove.getId()).remove(lastNode);
						tempStreetNetWork.getEdges().remove(edgeToRemove.getId() ,edgeToRemove );	
						/*Remove node only if degree is 1*/
						if(adjacencyList.get(nodeToRemove.getId()).isEmpty()){
							streetNodeMap.remove(nodeToRemove.getId() , nodeToRemove);
							adjacencyList.remove(nodeToRemove.getId());
						}

					}

				}
				/*Rest of the non route edges*/
				else {
					for(int i = 1; i < edgeToBreak.getGeom().getNumPoints() -1 ; i++) {


						lastNode = addNodeToEdge(edgeToBreak.getGeom().getPointN(i), true, edgeToBreak, adjacencyList, streetNodeMap, tempStreetNetWork);
						//System.out.println("Braking subedge " + i + ": " + edgeToBreak);
						edgeToBreak = tempStreetNetWork.getEdge(lastNode, edgeToBreak.getTargetPoint());


					}
				}


			}
			else {
				System.out.println("two node edge");
				
			}

		}

		streetNetwork.setEdges( tempStreetNetWork.getEdges() );



	}
	
	
	public static void addDummyNodesCloseToDPCrossesAndTopo(Route route, Map<Integer, StreetNode> streetNodeMap,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork, double minDist) {

		
		/**Run through all edges and add node after the first node if the first is DP, crossing or topopoint and dist < mindist*/ 
		for(int i = 0; i < route.getEdgeList().size(); i++){
			StreetNode firstNode, secondNode;
			//System.out.println("Edge is reversed: " + !route.getEdgeList().get(i).isReversed() );
			if(!route.getEdgeList().get(i).isReversed()){
				firstNode = route.getEdgeList().get(i).getSourcePoint();
				secondNode = route.getEdgeList().get(i).getTargetPoint();
			}
			else{
				secondNode = route.getEdgeList().get(i).getSourcePoint();
				firstNode = route.getEdgeList().get(i).getTargetPoint();
			}

			if( (firstNode.isDecisionPoint() || (adjacencyList.get(firstNode.getId()).size() > 2) || firstNode.isTopoCrossing())
					&& firstNode.getGeom().distance(secondNode.getGeom()) > minDist){
				boolean idNotFound = true;
				int newId = Integer.MAX_VALUE;
				while(idNotFound){
					if(streetNodeMap.containsKey(newId))
						newId--;
					else
						idNotFound = false;
				}
				double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(firstNode.getGeom().getX(), firstNode.getGeom().getY(), secondNode.getGeom().getX(), secondNode.getGeom().getY());
				double newX = firstNode.getGeom().getX() + 0.9*minDist*Math.cos(angle);
				double newY = firstNode.getGeom().getY() - 0.9*minDist*Math.sin(angle);
				StreetNode newNode = new StreetNode(newId, GeoConvertionsOperations.Java2DPointToJTSGeometry(new Point2D.Double(newX , newY )), true, firstNode.getPriority(), false);
				streetNodeMap.put(newNode.getId(), newNode);
				addCrossingNodeToRouteEdge(route, route.getEdgeList().get(i), newNode, adjacencyList, streetNetwork);
				i++;
			}
//			else if( (secondNode.isDecisionPoint() || (adjacencyList.get(secondNode.getId()).size() > 2) || secondNode.isTopoCrossing()) 
//					&& secondNode.getGeom().distance(firstNode.getGeom()) > minDist){
//				boolean idNotFound = true;
//				int newId = Integer.MAX_VALUE;
//				while(idNotFound){
//					if(streetNodeMap.containsKey(newId))
//						newId--;
//					else
//						idNotFound = false;
//				}
//				double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(secondNode.getGeom().getX(), secondNode.getGeom().getY(), firstNode.getGeom().getX(), firstNode.getGeom().getY());
//				double newX = secondNode.getGeom().getX() + 0.9*minDist*Math.cos(angle);
//				double newY = secondNode.getGeom().getY() - 0.9*minDist*Math.sin(angle);
//				StreetNode newNode = new StreetNode(newId, GeoConvertionsOperations.Java2DPointToJTSGeometry(new Point2D.Double(newX , newY )), true, secondNode.getPriority());
//				streetNodeMap.put(newNode.getId(), newNode);
//				addCrossingNodeToRouteEdge(route, route.getEdgeList().get(i), newNode, adjacencyList, streetNetwork);
//				i++;
//			}




		}
		
		
		/**Run through all edges and add node before the second node if the second is DP, crossing or topopoint and dist < mindist*/ 
		for(int i = 0; i < route.getEdgeList().size(); i++){
			StreetNode firstNode, secondNode;
			//System.out.println("Edge is reversed: " + !route.getEdgeList().get(i).isReversed() );
			if(!route.getEdgeList().get(i).isReversed()){
				firstNode = route.getEdgeList().get(i).getSourcePoint();
				secondNode = route.getEdgeList().get(i).getTargetPoint();
			}
			else{
				secondNode = route.getEdgeList().get(i).getSourcePoint();
				firstNode = route.getEdgeList().get(i).getTargetPoint();
			}

			if( (secondNode.isDecisionPoint() || (adjacencyList.get(secondNode.getId()).size() > 2) || secondNode.isTopoCrossing()) 
					&& secondNode.getGeom().distance(firstNode.getGeom()) > minDist){
				boolean idNotFound = true;
				int newId = Integer.MAX_VALUE;
				while(idNotFound){
					if(streetNodeMap.containsKey(newId))
						newId--;
					else
						idNotFound = false;
				}
				double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(secondNode.getGeom().getX(), secondNode.getGeom().getY(), firstNode.getGeom().getX(), firstNode.getGeom().getY());
				double newX = secondNode.getGeom().getX() + 0.9*minDist*Math.cos(angle);
				double newY = secondNode.getGeom().getY() - 0.9*minDist*Math.sin(angle);
				StreetNode newNode = new StreetNode(newId, GeoConvertionsOperations.Java2DPointToJTSGeometry(new Point2D.Double(newX , newY )), true, secondNode.getPriority(), false);
				streetNodeMap.put(newNode.getId(), newNode);
				addCrossingNodeToRouteEdge(route, route.getEdgeList().get(i), newNode, adjacencyList, streetNetwork);
				i++;
			}




		}




	}
	
	
	public static void addDummyNodesCloseToDP(Route route, Map<Integer, StreetNode> streetNodeMap,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork, double minDist) {

		
		/**Run through all edges and add node after the first node if the first is DP, crossing or topopoint and dist < mindist*/ 
		for(int i = 0; i < route.getEdgeList().size(); i++){
			StreetNode firstNode, secondNode;
			//System.out.println("Edge is reversed: " + !route.getEdgeList().get(i).isReversed() );
			if(!route.getEdgeList().get(i).isReversed()){
				firstNode = route.getEdgeList().get(i).getSourcePoint();
				secondNode = route.getEdgeList().get(i).getTargetPoint();
			}
			else{
				secondNode = route.getEdgeList().get(i).getSourcePoint();
				firstNode = route.getEdgeList().get(i).getTargetPoint();
			}

			if( (firstNode.isDecisionPoint())
					&& firstNode.getGeom().distance(secondNode.getGeom()) > minDist){
				boolean idNotFound = true;
				int newId = Integer.MAX_VALUE;
				while(idNotFound){
					if(streetNodeMap.containsKey(newId))
						newId--;
					else
						idNotFound = false;
				}
				double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(firstNode.getGeom().getX(), firstNode.getGeom().getY(), secondNode.getGeom().getX(), secondNode.getGeom().getY());
				double newX = firstNode.getGeom().getX() + 0.9*minDist*Math.cos(angle);
				double newY = firstNode.getGeom().getY() - 0.9*minDist*Math.sin(angle);
				StreetNode newNode = new StreetNode(newId, GeoConvertionsOperations.Java2DPointToJTSGeometry(new Point2D.Double(newX , newY )), true, firstNode.getPriority(), false);
				streetNodeMap.put(newNode.getId(), newNode);
				addCrossingNodeToRouteEdge(route, route.getEdgeList().get(i), newNode, adjacencyList, streetNetwork);
				i++;
			}
//			else if( (secondNode.isDecisionPoint() || (adjacencyList.get(secondNode.getId()).size() > 2) || secondNode.isTopoCrossing()) 
//					&& secondNode.getGeom().distance(firstNode.getGeom()) > minDist){
//				boolean idNotFound = true;
//				int newId = Integer.MAX_VALUE;
//				while(idNotFound){
//					if(streetNodeMap.containsKey(newId))
//						newId--;
//					else
//						idNotFound = false;
//				}
//				double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(secondNode.getGeom().getX(), secondNode.getGeom().getY(), firstNode.getGeom().getX(), firstNode.getGeom().getY());
//				double newX = secondNode.getGeom().getX() + 0.9*minDist*Math.cos(angle);
//				double newY = secondNode.getGeom().getY() - 0.9*minDist*Math.sin(angle);
//				StreetNode newNode = new StreetNode(newId, GeoConvertionsOperations.Java2DPointToJTSGeometry(new Point2D.Double(newX , newY )), true, secondNode.getPriority());
//				streetNodeMap.put(newNode.getId(), newNode);
//				addCrossingNodeToRouteEdge(route, route.getEdgeList().get(i), newNode, adjacencyList, streetNetwork);
//				i++;
//			}




		}
		
		
		/**Run through all edges and add node before the second node if the second is DP, crossing or topopoint and dist < mindist*/ 
		for(int i = 0; i < route.getEdgeList().size(); i++){
			StreetNode firstNode, secondNode;
			//System.out.println("Edge is reversed: " + !route.getEdgeList().get(i).isReversed() );
			if(!route.getEdgeList().get(i).isReversed()){
				firstNode = route.getEdgeList().get(i).getSourcePoint();
				secondNode = route.getEdgeList().get(i).getTargetPoint();
			}
			else{
				secondNode = route.getEdgeList().get(i).getSourcePoint();
				firstNode = route.getEdgeList().get(i).getTargetPoint();
			}

			if( (secondNode.isDecisionPoint()) 
					&& secondNode.getGeom().distance(firstNode.getGeom()) > minDist){
				boolean idNotFound = true;
				int newId = Integer.MAX_VALUE;
				while(idNotFound){
					if(streetNodeMap.containsKey(newId))
						newId--;
					else
						idNotFound = false;
				}
				double angle = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(secondNode.getGeom().getX(), secondNode.getGeom().getY(), firstNode.getGeom().getX(), firstNode.getGeom().getY());
				double newX = secondNode.getGeom().getX() + 0.9*minDist*Math.cos(angle);
				double newY = secondNode.getGeom().getY() - 0.9*minDist*Math.sin(angle);
				StreetNode newNode = new StreetNode(newId, GeoConvertionsOperations.Java2DPointToJTSGeometry(new Point2D.Double(newX , newY )), true, secondNode.getPriority(), false);
				streetNodeMap.put(newNode.getId(), newNode);
				addCrossingNodeToRouteEdge(route, route.getEdgeList().get(i), newNode, adjacencyList, streetNetwork);
				i++;
			}




		}




	}

	public static void simulateNavigationData(LineString routeLineString) {

		System.out.println("Ssimulating Navigation Data");
		LengthIndexedLine indexdRoute = new LengthIndexedLine(routeLineString);	
		double index = 0;
		double maxIndex = indexdRoute.getEndIndex();
		int i = 0;



		JSONObject feature = new JSONObject();
		try {
			feature.put("type", "Feature");


			JSONArray lineString = new JSONArray();


			while(index < maxIndex) {
				Coordinate position = indexdRoute.extractPoint(index);
//				System.out.println(i + " " + index);
//				System.out.println(position);
				i++;

				JSONArray point = new JSONArray();
				point.put(position.x);
				point.put(position.y);

				lineString.put(point);


				double offset = 0.001*Math.random()*maxIndex;
				index += offset;
			}

			JSONObject geometry = new JSONObject();
			geometry.put("type", "LineString");
			geometry.put("coordinates", lineString);

			feature.put("geometry", geometry);

			JSONObject properties = new JSONObject();
			properties.put("type", "routeGeom");
			feature.put("properties", properties );	
			System.out.println("Navigation Route Linestring.");
			System.out.println(feature);

		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

			
	}


	
	private static void buildStartEndPolyControlEdges(PolygonalTopo p, Route route, 			
			Map<Integer, ArrayList<StreetNode>> adjacencyList, Map<Integer, StreetNode> streetNodeMap,
			StreetNetworkTopological streetNetwork) 
	{
			//ArrayList<Double> polyPointsDistRoute = new ArrayList<Double>();
			ArrayList<Double> polyPointsDistToExtreme = new ArrayList<Double>();
			ArrayList<Double> polyPointsLenghtToCross = new ArrayList<Double>();
			//ArrayList<Double> polyPointsIndexToRoute = new ArrayList<Double>();
			ArrayList<Point> polyPoints = new ArrayList<Point>();
			
			LineString routeLS = route.getRoutePath().asLineString(0);
			
			StreetNode extremeNode;
			
			if(p.getType() ==  PolygonalTopo.ROUTE_STARTS_AT ) {
				extremeNode = route.getStart();
			}
			else {
				extremeNode = route.getEnd();
			}
			
			
			
			LengthIndexedLine indexdRoute = new LengthIndexedLine(routeLS);
			int indexOfCrossig = 0;
			int ptIndex = 0;

			for(StreetEdge e:p.getEdgeList()){

				Point pt = e.getSourcePoint().getGeom();
				polyPoints.add(pt);	
				polyPointsDistToExtreme.add(pt.distance(extremeNode.getGeom()));
				//polyPointsDistRoute.add(routeLS.distance(pt));
				//polyPointsIndexToRoute.add(indexdRoute.indexOf(pt.getCoordinate()));
				if(e.getSourcePoint().isTopoCrossing() && e.getSourcePoint().isRouteNode()) {
					indexOfCrossig = ptIndex;
				}
				ptIndex++;
			}

			/***I`m checking only the distance to the crossing! In instead check the lenght from the crossing!!! */

			double halfPolygonLength =p.asPolygon(0).getLength()/2;
			double sumdist = 0;
			Double[] lengthToCrossArray = new Double[polyPoints.size()];
			lengthToCrossArray[indexOfCrossig] = halfPolygonLength;
			for(int i= indexOfCrossig; i < polyPoints.size() - 1; i++) {
				sumdist += polyPoints.get(i+1).distance(polyPoints.get(i));
				lengthToCrossArray[i+1] = Math.abs( halfPolygonLength - sumdist);

			}
			sumdist = 0;
			for(int i= indexOfCrossig; i > 0; i--) {
				sumdist += polyPoints.get(i-1).distance(polyPoints.get(i));
				lengthToCrossArray[i-1] = Math.abs( halfPolygonLength - sumdist);

			}


			polyPointsLenghtToCross = new ArrayList<Double>(Arrays.asList(lengthToCrossArray));

			System.out.println(p.getPolygonalFeature().getName());
			ArrayList<Double> bestStartEnd = PolygonVertexStats.bestStartEnd(polyPointsDistToExtreme, polyPointsLenghtToCross);


			/***find the index of the best candidate**/
			int bestindex = 0;
			float n;
			double bestValue = bestStartEnd.get(0);
			for(int i = 0; i <  bestStartEnd.size(); i ++){

				if(bestStartEnd.get(i) < bestValue){
					bestValue = bestStartEnd.get(i);
					bestindex = i;
				}


				//			PointFeature polyVertex = new PointFeature();
				//			polyVertex.setGeom(polyPoints.get(i));
				//
				//			//if(polyPointsDistNormalized.get(i) < 0.4)
				//			n = (float) (bestStartEnd.get(i)*100);
				//			//					else 
				//			//						n =100;
				//
				//			int R = Math.round( (255 * n) / 100) ;
				//			int	G = Math.round((255 * (100 - n)) / 100 );
				//			int	B = 0;
				//
				//
				//			polyVertex.setColor(new Color(R, G, B));
				//
				//			polyVertex.setName(p.getPolygonalFeature().getName() + "Vertex" );
				//			polyVertex.setType("polyvertex2");
				//			pointFeatureList.add(polyVertex);

			}

			int newEdgeId = Integer.MAX_VALUE;
			boolean idNotFound;

			/**Control edge first*/
			StreetNode controlEdgeSource = p.getEdgeList().get(bestindex).getSourcePoint();
			controlEdgeSource.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ANCHOR));
			StreetNode controlEdgeTargetnewOnRouteNode = addNodeToTheRoute(controlEdgeSource.getGeom(), route, adjacencyList, streetNodeMap, streetNetwork); 
			controlEdgeTargetnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ANCHOR));
			//extremeNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ANCHOR));

			idNotFound = true;			
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			StreetEdge controEdge = new StreetEdge(newEdgeId, p.getPolygonalFeature().getName()+"ControlEdge",
					controlEdgeSource, controlEdgeTargetnewOnRouteNode, 
					p.getPolygonalFeature().getId());
			controEdge.setFakeEdge(true);
			streetNetwork.getEdges().put(controEdge.getId(),controEdge);

			p.getControlEdges().add(controEdge);

			/*AdjancencyList update*/
			//adjacencyList.get(controEdgeFirst.getSource()).add(controEdgeFirst.getTargetPoint());
			//adjacencyList.get(controEdgeFirst.getTarget()).add(controEdgeFirst.getSourcePoint());





		}



	private static void buildPolyStartEndOld(PolygonalTopo p, Route route, 
		
		Map<Integer, ArrayList<StreetNode>> adjacencyList, Map<Integer, StreetNode> streetNodeMap,
		StreetNetworkTopological streetNetwork) {
		ArrayList<Double> polyPointsDistRoute = new ArrayList<Double>();
		ArrayList<Double> polyPointsLenghtToCross = new ArrayList<Double>();
		ArrayList<Double> polyPointsIndexToRoute = new ArrayList<Double>();
		ArrayList<Point> polyPoints = new ArrayList<Point>();
		
		LineString routeLS = route.getRoutePath().asLineString(0);
		
		LengthIndexedLine indexdRoute = new LengthIndexedLine(routeLS);
		int indexOfCrossig = 0;
		int ptIndex = 0;

		for(StreetEdge e:p.getEdgeList()){

			Point pt = e.getSourcePoint().getGeom();
			polyPoints.add(pt);	
			polyPointsDistRoute.add(routeLS.distance(pt));
			polyPointsIndexToRoute.add(indexdRoute.indexOf(pt.getCoordinate()));
			if(e.getSourcePoint().isTopoCrossing() && e.getSourcePoint().isRouteNode()) {
				indexOfCrossig = ptIndex;
			}
			ptIndex++;
		}

		/***I`m checking only the distance to the crossing! In instead check the lenght from the crossing!!! */

		double halfPolygonLength =p.asPolygon(0).getLength()/2;
		double sumdist = 0;
		Double[] lengthToCrossArray = new Double[polyPoints.size()];
		lengthToCrossArray[indexOfCrossig] = halfPolygonLength;
		for(int i= indexOfCrossig; i < polyPoints.size() - 1; i++) {
			sumdist += polyPoints.get(i+1).distance(polyPoints.get(i));
			lengthToCrossArray[i+1] = Math.abs( halfPolygonLength - sumdist);

		}
		sumdist = 0;
		for(int i= indexOfCrossig; i > 0; i--) {
			sumdist += polyPoints.get(i-1).distance(polyPoints.get(i));
			lengthToCrossArray[i-1] = Math.abs( halfPolygonLength - sumdist);

		}


		polyPointsLenghtToCross = new ArrayList<Double>(Arrays.asList(lengthToCrossArray));

		System.out.println(p.getPolygonalFeature().getName());
		ArrayList<Double> bestStartEnd = PolygonVertexStats.bestStartEnd(polyPointsDistRoute, polyPointsLenghtToCross);


		/***find the index of the best candidate**/
		int bestindex = 0;
		float n;
		double bestValue = bestStartEnd.get(0);
		for(int i = 0; i <  bestStartEnd.size(); i ++){

			if(bestStartEnd.get(i) < bestValue){
				bestValue = bestStartEnd.get(i);
				bestindex = i;
			}


			//			PointFeature polyVertex = new PointFeature();
			//			polyVertex.setGeom(polyPoints.get(i));
			//
			//			//if(polyPointsDistNormalized.get(i) < 0.4)
			//			n = (float) (bestStartEnd.get(i)*100);
			//			//					else 
			//			//						n =100;
			//
			//			int R = Math.round( (255 * n) / 100) ;
			//			int	G = Math.round((255 * (100 - n)) / 100 );
			//			int	B = 0;
			//
			//
			//			polyVertex.setColor(new Color(R, G, B));
			//
			//			polyVertex.setName(p.getPolygonalFeature().getName() + "Vertex" );
			//			polyVertex.setType("polyvertex2");
			//			pointFeatureList.add(polyVertex);

		}

		int newEdgeId = Integer.MAX_VALUE;
		boolean idNotFound;

		/**Control edge first*/
		StreetNode controlEdgeSource = p.getEdgeList().get(bestindex).getSourcePoint();
		controlEdgeSource.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ANCHOR));
		StreetNode controlEdgeTargetnewOnRouteNode = addNodeToTheRoute(controlEdgeSource.getGeom(), route, adjacencyList, streetNodeMap, streetNetwork); 
		controlEdgeTargetnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ANCHOR));

		idNotFound = true;			
		while(idNotFound){
			if(streetNetwork.getEdges().containsKey(newEdgeId))
				newEdgeId--;
			else
				idNotFound = false;
		}
		StreetEdge controEdge = new StreetEdge(newEdgeId, p.getPolygonalFeature().getName()+"ControlEdge",
				controlEdgeSource, controlEdgeTargetnewOnRouteNode, 
				p.getPolygonalFeature().getId());
		controEdge.setFakeEdge(true);
		streetNetwork.getEdges().put(controEdge.getId(),controEdge);

		p.getControlEdges().add(controEdge);

		/*AdjancencyList update*/
		//adjacencyList.get(controEdgeFirst.getSource()).add(controEdgeFirst.getTargetPoint());
		//adjacencyList.get(controEdgeFirst.getTarget()).add(controEdgeFirst.getSourcePoint());





	}

	private static boolean buildPointLMControlEdge(PointTopo pointLMTopo, Route route, 	Map<Integer, ArrayList<StreetNode>> adjacencyList, Map<Integer, StreetNode> streetNodeMap,	StreetNetworkTopological streetNetwork) {

		boolean validPointPLM = true;
		
		
		StreetEdge closestEdge = null;
		StreetEdge closestRouteEdge = null;
		double dist;
		double minDist = Double.MAX_VALUE;
		double minDistRouteEdge = Double.MAX_VALUE;

		Point closestPointToEdge = null;
		Point closestPoint = null;
		int i = 0;
		
		

		/** get closest edge and closest point in the edege*/
		for(StreetEdge e: streetNetwork.getEdges().values()) {
			if(!e.isStubEdge() && !(e.getIsPolygonEdge()>0) && !e.isFakeEdge() ) {

				i++;
				/***Por favor- Conferir se não há edges com tamanho 0. Ou seja, que comecem e terminem no mesmo lugar****/
				//System.out.println("Length of edge " + i + " " + Point2D.distance(e.getX1(), e.getY1(), e.getX2(), e.getY2()) );
				closestPointToEdge = GeoConvertionsOperations.Java2DPointToJTSGeometry(GeometricOperation.getClosestPointOnSegment(e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getGeom().getY(),
						e.getTargetPoint().getGeom().getX(), e.getTargetPoint().getGeom().getY(),
						pointLMTopo.getNode().getGeom().getX(), pointLMTopo.getNode().getGeom().getY() ));

				dist = pointLMTopo.getNode().getGeom().distance(closestPointToEdge);


				
				if(e instanceof RouteEdge){
					 if( dist < minDist) {
						closestEdge = e;
						minDist = dist;
						closestPoint =  closestPointToEdge;
					 }
					 if( dist < minDistRouteEdge) {
							closestRouteEdge = e;
							minDistRouteEdge = dist;
					}
					 
				}
				else if(dist < 70 && dist < minDist) {
					closestEdge = e;
					minDist = dist;
					closestPoint =  closestPointToEdge;
				}


			}

		}
		/**get closest node form closest edge***/
		
		StreetNode controlEdgeLMNode = pointLMTopo.getNode();
		controlEdgeLMNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(), true, NodeTopoRelation.ANCHOR));
		//controlEdgeLMNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(), true, NodeTopoRelation.ANCHOR));
		
		StreetNode closestNode = null;
		if(pointLMTopo.getNode().getGeom().distance(closestEdge.getSourcePoint().getGeom()) < pointLMTopo.getNode().getGeom().distance(closestEdge.getTargetPoint().getGeom()))
			closestNode = closestEdge.getSourcePoint();
		else
			closestNode = closestEdge.getTargetPoint();

		
		StreetNode controlEdgeStreetNode = null;
		
		
		/**if distance to closest node is less then values uses this node fo control edge**/
		if(closestPoint.distance(closestNode.getGeom()) < 10) {
			controlEdgeStreetNode = closestNode;
		}
		else if(  (closestPoint.distance(closestNode.getGeom()) < 40) && closestNode.isDecisionPoint() && !closestNode.isRoundAbout()) {
			controlEdgeStreetNode = closestNode;
		}
		else if(  (closestPoint.distance(closestNode.getGeom()) < 30) && closestNode.getDegree()>2 && !closestNode.isRoundAbout()) {
			controlEdgeStreetNode = closestNode;
		}
		/**else create an extra node in the route/street edge**/
		else if(closestEdge instanceof RouteEdge) {
			controlEdgeStreetNode = addNodeToTheRoute(controlEdgeLMNode.getGeom(), route, adjacencyList, streetNodeMap, streetNetwork); 
			
		}
		else {
			controlEdgeStreetNode = addNodeToEdge(closestPoint, false, closestEdge, adjacencyList, streetNodeMap, streetNetwork); 

		}

		
		/**SET DISTANCE TO ROUTE AND ADD TO ADJ LIST ONLY LM CLOSE TO THE ROUTE***/	
		LineString routeLS = route.getRoutePath().asLineString(0);
		double distToRoute = routeLS.distance(controlEdgeLMNode.getGeom());			
		pointLMTopo.setDistToRoute(distToRoute);
		boolean isRightOfTheRoute = isPointRightToEdge(controlEdgeLMNode.getGeom(), (RouteEdge)closestRouteEdge);
		System.out.println(pointLMTopo.getPointFeature().getName() +" dists " + distToRoute + " from the route. It right of the route: " + isRightOfTheRoute);
		
		
		
		if( distToRoute <= 80 && (pointLMTopo.getPointFeature().getSalience() <= 1 ) && controlEdgeStreetNode.isDecisionPoint() ) {
			if(isRightOfTheRoute)
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.ALONG_RIGHT_PTLM));

			else
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.ALONG_LEFT_PTLM));
			
		}
		else if( distToRoute <= 80 && (pointLMTopo.getPointFeature().getSalience() >= 2 ) ) {
			if(isRightOfTheRoute)
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.ALONG_RIGHT_PTLM));

			else
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.ALONG_LEFT_PTLM));
			
		}
		else if(distToRoute < 300 && pointLMTopo.getPointFeature().getSalience() >= 3 ) {
			if(isRightOfTheRoute)
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.GLOBAL_RIGHT_PTLM));
			else
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.GLOBAL_LEFT_PTLM));
						
		}
		else if(distToRoute < 1500 && pointLMTopo.getPointFeature().getSalience() >=4  ) {
			if(isRightOfTheRoute)
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.GLOBAL_RIGHT_PTLM));
			else
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.GLOBAL_LEFT_PTLM));
			
		}
		else if(pointLMTopo.getPointFeature().getSalience() >= 5 ) {
			if(isRightOfTheRoute)
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.GLOBAL_RIGHT_PTLM));
			else
				controlEdgeStreetNode.getTopoRelations().add(new NodeTopoRelation(pointLMTopo.getPointFeature().getId(),true, NodeTopoRelation.GLOBAL_LEFT_PTLM));
			
		}
		else {
			
			validPointPLM = false;
			
		}
		
		
		
		
	
		
		
		if(validPointPLM) {
		
			/***create control edged*/
			int newEdgeId = Integer.MAX_VALUE;
			boolean idNotFound;
			boolean isRight = false;
			/**Control edge first*/
	
			//controlEdgeTargetFirstnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), NodeTopoRelation.ALONG_UNKNOWN));
	
			idNotFound = true;			
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			StreetEdge controlEdge = new StreetEdge(newEdgeId, pointLMTopo.getPointFeature().getName()+"ControlEdge",
					controlEdgeLMNode, controlEdgeStreetNode);
			
			controlEdge.setFakeEdge(true);
			streetNetwork.getEdges().put(controlEdge.getId(),controlEdge);
	
			pointLMTopo.setControlEdge(controlEdge);
	
			/*Add to AdjancencyList only LM along the route*/
			if(distToRoute <= 80 && controlEdgeStreetNode.isRouteNode()) {
				System.out.println(pointLMTopo.getPointFeature().getName() +" dists " + distToRoute + " from the route. Add to adj list");
				
				adjacencyList.get(controlEdge.getSourceId()).add(controlEdge.getTargetPoint());
				adjacencyList.get(controlEdge.getTargetId()).add(controlEdge.getSourcePoint());
			}
			else {
				pointLMTopo.setType(PointTopo.GLOBAL);
			}
		
		}
		
		return validPointPLM;

	}


	/***DONT USE FOR ROUTE START OR END***/
	/**RouteDistancePreference Tells the preference to chose control nodes with shorter distance */ 
	private static void buildPolyControlEdges(PolygonalTopo p, Route route, double routeDistancePreference,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, Map<Integer, StreetNode> streetNodeMap,
			StreetNetworkTopological streetNetwork) {

		
		LineString rotueLS = route.getRoutePath().asLineString(0);
		ArrayList<Double> polyPointsDist = new ArrayList<Double>();

		/**list of the lenghtindex of all poygon points**/ 
		ArrayList<Double> polyPointsProjectionIndex = new ArrayList<Double>();
		ArrayList<Double> polyPointsIndex = new ArrayList<Double>();
		/**polygon points list**/ 
		ArrayList<Point> polyPoints = new ArrayList<Point>();

		LengthIndexedLine indexdRoute = new LengthIndexedLine(rotueLS);



		for(StreetEdge e:p.getEdgeList()){


			Point pt = e.getSourcePoint().getGeom();
			polyPoints.add(pt);					
			polyPointsDist.add(rotueLS.distance(pt));
			polyPointsProjectionIndex.add(indexdRoute.project(pt.getCoordinate()));
			polyPointsIndex.add(indexdRoute.indexOf(pt.getCoordinate()));


		}
		float n = 100;
		/***evaluate polygon points as posible candidates for first control point choice*/
		ArrayList<Double> bestFirst = PolygonVertexStats.distFirst(polyPointsIndex, polyPointsDist, routeDistancePreference);
		ArrayList<Double> bestLast = PolygonVertexStats.distLast(polyPointsIndex, polyPointsDist , routeDistancePreference);

		/***find the index of the best candidate**/
		int bestFirstindex = 0;
		double bestFirstValue = bestFirst.get(0);
		
		for(int i = 0; i <  bestFirst.size(); i ++){
			if(bestFirst.get(i) < bestFirstValue){
				bestFirstValue = bestFirst.get(i);
				bestFirstindex = i;
			}
		}	
		
		int bestLastindex = 0;
		double bestLastValue = bestLast.get(0);
		for(int i = 0; i <  bestLast.size(); i ++){

			if(bestLast.get(i) < bestLastValue ){
				bestLastValue = bestLast.get(i);
				bestLastindex = i;
			}
		}
		boolean distOfTwo = false;
		boolean swich = true;
		while(!distOfTwo) {
			if(Math.abs(bestLastindex - bestFirstindex) > 1  && Math.abs(bestLastindex - bestFirstindex) < bestLast.size() -2 ) {
				distOfTwo = true;
			}
			else {
				if(swich) 
					bestFirstindex = Math.floorMod((bestFirstindex -1), bestLast.size());
				else
					bestLastindex = Math.floorMod((bestLastindex +1), bestLast.size());

				swich = !swich;
			}
		}
		
		/***Just to create the color point layers**/
//		for(int i = 0; i <  bestFirst.size(); i ++){
//			//System.out.println("pt " + i + " dist to route = " + polyPointsDist.get(i)+ "  index = " + polyPointsProjectionIndex.get(i) + "    projected index = " + polyPointsProjectionIndex.get(i));
//
//			PointFeature polyVertex = new PointFeature();
//			polyVertex.setGeom(polyPoints.get(i));
//
//			//	if(polyPointsDistNormalized.get(i) < 0.2)
//			n = (float) (bestFirst.get(i)*100);
//			//	else 
//			//		n =100;
//
//			int R = Math.round( (255 * n) / 100) ;
//			int	G = Math.round((255 * (100 - n)) / 100 );
//			int	B = 0;
//
//
//			polyVertex.setColor(new Color(R, G, B));
//
//			polyVertex.setName(p.getPolygonalFeature().getName() + "Vertex" );
//			polyVertex.setType("polyvertex1");
//			pointFeatureList.add(polyVertex);
//
//		}
//		/***evaluate polygon points as posible candidates for second control point choice*/
//
//		
//		//ArrayList<Double> bestStartEnde = PolygonVertexStats.distLast(polyPointsIndex, polyPointsDist , routeDistancePreference);
//		/***find the index of the best candidate**/
//		for(int i = 0; i <  bestLast.size(); i ++){
//
//			PointFeature polyVertex = new PointFeature();
//			polyVertex.setGeom(polyPoints.get(i));
//
//			//if(polyPointsDistNormalized.get(i) < 0.4)
//			n = (float) (bestLast.get(i)*100);
//			//					else 
//			//						n =100;
//
//			int R = Math.round( (255 * n) / 100) ;
//			int	G = Math.round((255 * (100 - n)) / 100 );
//			int	B = 0;
//
//
//			polyVertex.setColor(new Color(R, G, B));
//
//			polyVertex.setName(p.getPolygonalFeature().getName() + "Vertex" );
//			polyVertex.setType("polyvertex2");
//			pointFeatureList.add(polyVertex);
//
//		}



		/**for controlEdge: the source connects to the polygon, the target to the route/network***/

		int newEdgeId = Integer.MAX_VALUE;
		boolean idNotFound;
		boolean isRight = false;
		/**Control edge first*/
		
			StreetNode controlEdgeSourceFirst = p.getEdgeList().get(bestFirstindex).getSourcePoint();
			controlEdgeSourceFirst.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ANCHOR));
			StreetNode controlEdgeTargetFirstnewOnRouteNode = addNodeToTheRoute(controlEdgeSourceFirst.getGeom(), route, adjacencyList, streetNodeMap, streetNetwork); 
			//controlEdgeTargetFirstnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), NodeTopoRelation.ALONG_UNKNOWN));
			
			idNotFound = true;			
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			StreetEdge controEdgeFirst = new StreetEdge(newEdgeId, p.getPolygonalFeature().getName()+"ControlEdge",
					controlEdgeSourceFirst, controlEdgeTargetFirstnewOnRouteNode, 
					p.getPolygonalFeature().getId());
			controEdgeFirst.setFakeEdge(true);
			streetNetwork.getEdges().put(controEdgeFirst.getId(),controEdgeFirst);
			if (isPointRightToEdge(controEdgeFirst.getSourcePoint().getGeom(), route.getAnEdgeWithNode(controEdgeFirst.getTargetId()) )){
				isRight = true;
			}
			p.getControlEdges().add(controEdgeFirst);
			
			/*AdjancencyList update*/
			//adjacencyList.get(controEdgeFirst.getSource()).add(controEdgeFirst.getTargetPoint());
			//adjacencyList.get(controEdgeFirst.getTarget()).add(controEdgeFirst.getSourcePoint());
		


		
		/**Control edge Last*/		
			StreetNode controlEdgeSourceLast = p.getEdgeList().get(bestLastindex).getSourcePoint();
			controlEdgeSourceLast.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ANCHOR));
			StreetNode controlEdgeTargetLasttnewOnRouteNode = addNodeToTheRoute(controlEdgeSourceLast.getGeom(), route, adjacencyList, streetNodeMap, streetNetwork);
			//controlEdgeTargetLasttnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), NodeTopoRelation.ALONG_UNKNOWN));

			idNotFound = true;			
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			StreetEdge controEdgeLast = new StreetEdge(newEdgeId, p.getPolygonalFeature().getName()+"ControlEdge",
					controlEdgeSourceLast, controlEdgeTargetLasttnewOnRouteNode, 
					p.getPolygonalFeature().getId());
			controEdgeLast.setFakeEdge(true);
			streetNetwork.getEdges().put(controEdgeLast.getId(),controEdgeLast);
			p.getControlEdges().add(controEdgeLast);
			/*AdjancencyList update*/
			//adjacencyList.get(controEdgeLast.getSource()).add(controEdgeLast.getTargetPoint());
			//adjacencyList.get(controEdgeLast.getTarget()).add(controEdgeLast.getSourcePoint());
		
			switch (p.getType()) {
			case PolygonalTopo.PASSING_ALONG_UNKNOW:
				if (isRight){
					p.setType(PolygonalTopo.PASSING_ALONG_RIGHT);
					System.out.println("The polygon " + p.getPolygonalFeature().getName() + "is right to the route");
					controlEdgeTargetFirstnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ALONG_RIGHT_START));
					controlEdgeTargetLasttnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ALONG_RIGHT_END));

				}
				else{
					p.setType(PolygonalTopo.PASSING_ALONG_LEFT);
					System.out.println("The polygon " + p.getPolygonalFeature().getName() + "is left to the route");
					controlEdgeTargetFirstnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ALONG_LEFT_START));
					controlEdgeTargetLasttnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ALONG_LEFT_END));
		
				}
				
				break;
			case PolygonalTopo.GLOBAL:
				if (isRight){
					//p.setType(PolygonalTopo.PASSING_ALONG_RIGHT);
					System.out.println("The polygon " + p.getPolygonalFeature().getName() + "is right to the route");
					controlEdgeTargetFirstnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.GLOBAL_RIGHT_START));
					controlEdgeTargetLasttnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.GLOBAL_RIGHT_END));

				}
				else{
					//p.setType(PolygonalTopo.PASSING_ALONG_LEFT);
					System.out.println("The polygon " + p.getPolygonalFeature().getName() + "is left to the route");
					controlEdgeTargetFirstnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.GLOBAL_LEFT_START));
					controlEdgeTargetLasttnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.GLOBAL_LEFT_END));
		
				}
				
				break;
			case PolygonalTopo.ROUTE_IS_INSIDE:
				
					//p.setType(PolygonalTopo.PASSING_ALONG_RIGHT);
					System.out.println("The polygon " + p.getPolygonalFeature().getName() + "is right to the route");
					controlEdgeTargetFirstnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ANCHOR));
					controlEdgeTargetLasttnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.ANCHOR));


				
				break;

			default:
				break;
			}
		
		if(p.getType() == PolygonalTopo.PASSING_ALONG_UNKNOW) {
			if (isRight){
				p.setType(PolygonalTopo.PASSING_ALONG_RIGHT);
				System.out.println("The polygon " + p.getPolygonalFeature().getName() + "is right to the route");
			//	controlEdgeTargetFirstnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), NodeTopoRelation.ALONG_RIGHT_START));
			//	controlEdgeTargetLasttnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), NodeTopoRelation.ALONG_RIGHT_END));
	
	
			}
			else{
				p.setType(PolygonalTopo.PASSING_ALONG_LEFT);
				System.out.println("The polygon " + p.getPolygonalFeature().getName() + "is left to the route");
			//	controlEdgeTargetFirstnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), NodeTopoRelation.ALONG_LEFT_START));
			//	controlEdgeTargetLasttnewOnRouteNode.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), NodeTopoRelation.ALONG_LEFT_END));
	
			}
		}






	}

	
	

	private static void planarizePolygon(PolygonalTopo p, Route route, Map<Integer, ArrayList<StreetNode>> adjacencyList,
			Map<Integer, StreetNode> streetNodeMap, StreetNetworkTopological streetNetwork) {
		GeometryFactory geometryFactory = JTSFactoryFinder.getGeometryFactory();


		/**Planarize with route edges**/
		for( int i = 0; i < p.getEdgeList().size(); i++){
			StreetEdge polyEdge = p.getEdgeList().get(i);
			Coordinate[] coords  =
					new Coordinate[] {polyEdge.getSourcePoint().getGeom().getCoordinate(), polyEdge.getTargetPoint().getGeom().getCoordinate()};;

					LineString polyEdgeGeom = geometryFactory.createLineString(coords);
					/*check route edges first: */
					for(RouteEdge re: route.getEdgeList()){
						if(!polyEdge.isAdjacent(re)){
							coords  = new Coordinate[] {re.getSourcePoint().getGeom().getCoordinate(), re.getTargetPoint().getGeom().getCoordinate()};
							LineString edgeGeom = geometryFactory.createLineString(coords);
							if(polyEdgeGeom.intersects(edgeGeom)){
								System.out.println(p.getPolygonalFeature().getName() + " intersect route edge"  + re.getId() + "? "+  "TRUE");
								Geometry intersection = polyEdgeGeom.intersection(edgeGeom);
								System.out.println(Geometries.get(intersection) + " " + intersection);


								/*planarize 2 edges, re is route edge*/
								planarizeEdgesRoute(polyEdge, re, (Point)intersection, route, adjacencyList, streetNodeMap, streetNetwork, p, i);
								i++;
								break;
							}
						}
					}
		}
		/**Planarize with netwrok edges**/
//		for( int i = 0; i < p.getEdgeList().size(); i++){
//			StreetEdge polyEdge = p.getEdgeList().get(i);
//			Coordinate[] coords  =
//					new Coordinate[] {polyEdge.getSourcePoint().getGeom().getCoordinate(), polyEdge.getTargetPoint().getGeom().getCoordinate()};;
//
//					LineString polyEdgeGeom = geometryFactory.createLineString(coords);
//
//					/*check rest of the network */
//					for(StreetEdge e: streetNetwork.getEdges().values()){
//						//		if(e.getIsPolygonEdge() != p.getPolygonalFeature().getId() && !e.isAdjacent(polyEdge) ){
//						if(!(e.getIsPolygonEdge()>0)  && !e.isAdjacent(polyEdge) && !e.isStubEdge()){
//
//							coords  = new Coordinate[] {e.getSourcePoint().getGeom().getCoordinate(), e.getTargetPoint().getGeom().getCoordinate()};
//							LineString edgeGeom = geometryFactory.createLineString(coords);
//							if(polyEdgeGeom.intersects(edgeGeom)){
//								System.out.println(p.getPolygonalFeature().getName() + " intersect "  + e.getId() + "? "+  "TRUE");
//								Geometry intersection = polyEdgeGeom.intersection(edgeGeom);
//								System.out.println(Geometries.get(intersection) + " " + intersection);
//								planarizeEdges(polyEdge, e, (Point)intersection, adjacencyList, streetNodeMap, streetNetwork, p, i);
//								i++;
//								break;
//							}
//						}	
//
//
//					}
//
//
//		}	



	}

	private static void planarizeEdges(StreetEdge polyEdge, StreetEdge e, Point intersection,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, Map<Integer, StreetNode> streetNodeMap,
			StreetNetworkTopological streetNetwork, PolygonalTopo p, int polyEdgeIndex) {
		
		int newEdgeId = Integer.MAX_VALUE;
		int newNodeId = Integer.MAX_VALUE;
		boolean idNotFound = true;
		while(idNotFound){
			if(streetNodeMap.containsKey(newNodeId))
				newNodeId--;
			else
				idNotFound = false;
		}
		/*new node*/
		StreetNode n = new StreetNode(newNodeId,intersection, false, 1, p.getPolygonalFeature().getId(), true , false );
		n.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.CROSSING));
		streetNodeMap.put(n.getId(), n);
		
		/*new street edges*/
		StreetEdge e1, e2;
		try {

			e1 = (StreetEdge)e.clone();
			e2 = (StreetEdge)e.clone();



			/*ATENTION still need to se X1 X2 Y1 Y2 edge geom, cost etc 
			 * UPDATE AdjacencyList*/
			e1.setTargetPoint(n);

			e2.setSourcePoint(n);

			streetNetwork.getEdges().remove(e.getId());
			idNotFound = true;			
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			e2.setId(newEdgeId);
			//System.out.println("add to network edge: " + e2.getId());
			streetNetwork.getEdges().put(e2.getId(),e2);
			idNotFound = true;
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			e1.setId(newEdgeId);
			//System.out.println("add to network edge: " + e1.getId());
			streetNetwork.getEdges().put(e1.getId(),e1);


			/*AdjancencyList update*/
			adjacencyList.put(n.getId(), new ArrayList<StreetNode>());

			adjacencyList.get(e.getSourceId()).remove(e.getTargetPoint());
			adjacencyList.get(e.getSourceId()).add(n);
			adjacencyList.get(e.getTargetId()).remove(e.getSourcePoint());
			adjacencyList.get(e.getTargetId()).add(n);

			adjacencyList.get(n.getId()).add(e.getTargetPoint());
			adjacencyList.get(n.getId()).add(e.getSourcePoint());


			//polyEdgeIndex++;
	


		} catch (CloneNotSupportedException e3) {
			// TODO Auto-generated catch block
			e3.printStackTrace();
		}
		
		/*new polygon edges*/
		try {

			e1 = (StreetEdge)polyEdge.clone();
			e2 = (StreetEdge)polyEdge.clone();



			/*ATENTION still need to se X1 X2 Y1 Y2 edge geom, cost etc 
			 * UPDATE AdjacencyList*/
			e1.setTargetPoint(n);


			e2.setSourcePoint(n);

			streetNetwork.getEdges().remove(polyEdge.getId());
			idNotFound = true;			
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			e2.setId(newEdgeId);
			//System.out.println("add to network edge: " + e2.getId());
			streetNetwork.getEdges().put(e2.getId(),e2);
			idNotFound = true;
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			e1.setId(newEdgeId);
			//System.out.println("add to network edge: " + e1.getId());
			streetNetwork.getEdges().put(e1.getId(),e1);


			/*AdjancencyList update*/
			//adjacencyList.put(n.getId(), new ArrayList<StreetNode>());

			adjacencyList.get(polyEdge.getSourceId()).remove(polyEdge.getTargetPoint());
			adjacencyList.get(polyEdge.getSourceId()).add(n);
			adjacencyList.get(polyEdge.getTargetId()).remove(polyEdge.getSourcePoint());
			adjacencyList.get(polyEdge.getTargetId()).add(n);

			adjacencyList.get(n.getId()).add(polyEdge.getTargetPoint());
			adjacencyList.get(n.getId()).add(polyEdge.getSourcePoint());


			

			p.getEdgeList().remove(polyEdgeIndex);
			
			p.getEdgeList().add(polyEdgeIndex, e2);
			p.getEdgeList().add(polyEdgeIndex, e1);
			//polyEdgeIndex++;
	


		} catch (CloneNotSupportedException e3) {
			// TODO Auto-generated catch block
			e3.printStackTrace();
		}
		
		
		
	}

	private static void planarizeEdgesRoute(StreetEdge polyEdge, RouteEdge re, Point intersection, Route route,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, Map<Integer, StreetNode> streetNodeMap,
			StreetNetworkTopological streetNetwork, PolygonalTopo p, int polyEdgeIndex) {
		
		int newEdgeId = Integer.MAX_VALUE;
		int newNodeId = Integer.MAX_VALUE;
		boolean idNotFound = true;
		while(idNotFound){
			if(streetNodeMap.containsKey(newNodeId))
				newNodeId--;
			else
				idNotFound = false;
		}
		/*new node*/
		StreetNode n = new StreetNode(newNodeId,intersection, true, 1, p.getPolygonalFeature().getId() , true, false);
		n.getTopoRelations().add(new NodeTopoRelation(p.getPolygonalFeature().getId(), false, NodeTopoRelation.CROSSING));
		streetNodeMap.put(n.getId(), n);
		/*new route edges*/
		RouteEdge re1, re2;
		try {

			re1 = (RouteEdge)re.clone();
			re2 = (RouteEdge)re.clone();



			/*ATENTION still need to se X1 X2 Y1 Y2 edge geom, cost etc 
			 * UPDATE AdjacencyList*/
			re1.setTargetPoint(n);


			re2.setSourcePoint(n);

			streetNetwork.getEdges().remove(re.getId());
			idNotFound = true;			
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			re2.setId(newEdgeId);
			//System.out.println("add to network edge: " + e2.getId());
			streetNetwork.getEdges().put(re2.getId(),re2);
			idNotFound = true;
			newEdgeId = Integer.MAX_VALUE;
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			re1.setId(newEdgeId);
			//System.out.println("add to network edge: " + e1.getId());
			streetNetwork.getEdges().put(re1.getId(),re1);


			/*AdjancencyList update*/
			adjacencyList.put(n.getId(), new ArrayList<StreetNode>());

			adjacencyList.get(re.getSourceId()).remove(re.getTargetPoint());
			adjacencyList.get(re.getSourceId()).add(n);
			adjacencyList.get(re.getTargetId()).remove(re.getSourcePoint());
			adjacencyList.get(re.getTargetId()).add(n);

			adjacencyList.get(n.getId()).add(re.getTargetPoint());
			adjacencyList.get(n.getId()).add(re.getSourcePoint());


			int edgeIndex = route.getEdgeList().indexOf(re);

			route.getEdgeList().remove(edgeIndex);
			if(!re.isReversed()){

				route.getEdgeList().add(edgeIndex, re2);
				route.getEdgeList().add(edgeIndex, re1);
			}	
			else{
				//System.out.println("Edge is reversed");
				route.getEdgeList().add(edgeIndex, re1);
				route.getEdgeList().add(edgeIndex, re2);
			}

		} catch (CloneNotSupportedException e3) {
			// TODO Auto-generated catch block
			e3.printStackTrace();
		}
		/*new polygon edges*/
		StreetEdge e1, e2;
		try {

			e1 = (StreetEdge)polyEdge.clone();
			e2 = (StreetEdge)polyEdge.clone();



			/*ATENTION still need to se X1 X2 Y1 Y2 edge geom, cost etc 
			 * UPDATE AdjacencyList*/
			e1.setTargetPoint(n);


			e2.setSourcePoint(n);

			streetNetwork.getEdges().remove(polyEdge.getId());
			idNotFound = true;			
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			e2.setId(newEdgeId);
			//System.out.println("add to network edge: " + e2.getId());
			streetNetwork.getEdges().put(e2.getId(),e2);
			idNotFound = true;
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			e1.setId(newEdgeId);
			//System.out.println("add to network edge: " + e1.getId());
			streetNetwork.getEdges().put(e1.getId(),e1);


			/*AdjancencyList update*/
			//adjacencyList.put(n.getId(), new ArrayList<StreetNode>());

			adjacencyList.get(polyEdge.getSourceId()).remove(polyEdge.getTargetPoint());
			adjacencyList.get(polyEdge.getSourceId()).add(n);
			adjacencyList.get(polyEdge.getTargetId()).remove(polyEdge.getSourcePoint());
			adjacencyList.get(polyEdge.getTargetId()).add(n);

			adjacencyList.get(n.getId()).add(polyEdge.getTargetPoint());
			adjacencyList.get(n.getId()).add(polyEdge.getSourcePoint());


			

			p.getEdgeList().remove(polyEdgeIndex);
			
			p.getEdgeList().add(polyEdgeIndex, e2);
			p.getEdgeList().add(polyEdgeIndex, e1);
			//polyEdgeIndex++;
	


		} catch (CloneNotSupportedException e3) {
			// TODO Auto-generated catch block
			e3.printStackTrace();
		}
		
		
		
	}



	private static StreetNode addNodeToTheRoute(Point point, Route route,
			Map<Integer, ArrayList<StreetNode>> adjacencyList, Map<Integer, StreetNode> streetNodeMap, StreetNetworkTopological streetNetwork) {

		RouteEdge closestEdge = null;
		/*identifica aresta de cruzamento*/

		double dist;
		double minDist = Double.MAX_VALUE;
		StreetNode n;


		Point closestPointToEdge = null;
		Point closestPoint = null;

		int i = 0;
		for(RouteEdge e: route.getEdgeList()){


			/***Por favor- Conferir se não há edges com tamanho 0. Ou seja, que comecem e terminem no mesmo lugar****/
			//System.out.println("Length of edge " + i + " " + Point2D.distance(e.getX1(), e.getY1(), e.getX2(), e.getY2()) );
			i++;
			

			closestPointToEdge = GeoConvertionsOperations.Java2DPointToJTSGeometry(GeometricOperation.getClosestPointOnSegment(e.getSourcePoint().getGeom().getX(), e.getSourcePoint().getGeom().getY(),
					e.getTargetPoint().getGeom().getX(), e.getTargetPoint().getGeom().getY(),
					point.getX(), point.getY() ) );
			
			
			//closestPointToEdgeNewCalc = GeoConvertionsOperations.Java2DPointToJTSGeometry(closestPointToEdge);

			//dist =distanceoInDifferentCRS(point,	closestPointToEdgeNewCalc, "EPSG:3857");
			dist = point.distance(closestPointToEdge);

			if(dist < minDist){
				closestEdge = e;
				minDist = dist;
				closestPoint =  closestPointToEdge;
			}

		}

		/***Check if there is already a crossing/buffercrossing point of the same polygon near to the "closestPoint"
		 * If it is, use this point***/
		//		for(RouteEdge e: route.getEdgeList()){
		//
		//			e.getSourcePoint().isCrossedBy(polygon.getId());
		//
		//
		//			if(e.getSourcePoint().isCrossedBy(polygon.getId()) && e.getSourcePoint().getGeom().distance(GeoConvertionsOperations.Java2DPointToJTSGeometry(closestPoint)) < 0.001){
		//
		//				return e.getSourcePoint();
		//			}		
		//			if(e.getTargetPoint().isCrossedBy(polygon.getId())  && e.getTargetPoint().getGeom().distance(GeoConvertionsOperations.Java2DPointToJTSGeometry(closestPoint)) < 0.001){
		//				return e.getTargetPoint();
		//			}
		//		}




		/***NEED TO CHECK WHY IT DOESNT WORK WITH closestPointToEdgeNewCalc????
		 * Maybe because closest point on different CRS stays out of the edge****/
		/***check if closest point is in the same location of one of the nodes***/
		
//		if(  closestPointNewCalc.getX() == closestEdge.getSourcePoint().getGeom().getX() &&  closestPointNewCalc.getY() == closestEdge.getSourcePoint().getGeom().getY()  ){
//			System.out.println("Add Node to Route: Close node found");
//			n = closestEdge.getSourcePoint();
//			
//		}
//		else if (closestPointNewCalc.getX() == closestEdge.getTargetPoint().getGeom().getX() &&  closestPointNewCalc.getY() == closestEdge.getTargetPoint().getGeom().getY()){
//			System.out.println("Add Node to Route: Close node found");
//			n = closestEdge.getTargetPoint();
//			
//		}
//		
		if(  closestPoint.getX() == closestEdge.getSourcePoint().getGeom().getX() &&  closestPoint.getY() == closestEdge.getSourcePoint().getGeom().getY()  ){
			System.out.println("Add Node to Route: Close node found");
			n = closestEdge.getSourcePoint();
			
		}
		else if (closestPoint.getX() == closestEdge.getTargetPoint().getGeom().getX() &&  closestPoint.getY() == closestEdge.getTargetPoint().getGeom().getY()){
			System.out.println("Add Node to Route: Close node found");
			n = closestEdge.getTargetPoint();
			
		}
		else{
			System.out.println("Add Node to Route: No close node found");
			boolean idNotFound = true;
			int newNodeId = Integer.MAX_VALUE;
			while(idNotFound){
				if(streetNodeMap.containsKey(newNodeId))
					newNodeId--;
				else
					idNotFound = false;
			}

			/**adding point in target coordinate**/
			n = new StreetNode(newNodeId,closestPoint, true, 0, false);
			streetNodeMap.put(n.getId(), n);
			addCrossingNodeToRouteEdge(route, closestEdge, n, adjacencyList, streetNetwork);
		}
		return n;

	}

	private static boolean isPointRightToEdge(Point point, RouteEdge edge) {
		Point edgeBegin, edgeEnd;
		if(!edge.isReversed()){
			edgeBegin = edge.getSourcePoint().getGeom();
			edgeEnd = edge.getTargetPoint().getGeom();



		}
		else{
			edgeEnd = edge.getSourcePoint().getGeom();
			edgeBegin = edge.getTargetPoint().getGeom();
		}
		/****PRODUTO VETORIAL OU CROSS PRODUCT 
		 * (b.X - a.X)*(c.Y - a.Y) - (b.Y - a.Y)*(c.X - a.X);
		 * */

		double value = (edgeEnd.getX() - edgeBegin.getX())*(point.getY() - edgeBegin.getY()) - (edgeEnd.getY() - edgeBegin.getY())*(point.getX() - edgeBegin.getX());

		return (value < 0);
	}

	private static void addCrossingNodeToRouteEdge(Route route, RouteEdge e, StreetNode n, Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork) {

		RouteEdge e1,e2;
		try {

			e1 = (RouteEdge)e.clone();
			e2 = (RouteEdge)e.clone();



			/*ATENTION still need to se X1 X2 Y1 Y2 edge geom, cost etc 
			 * UPDATE AdjacencyList*/
			e1.setTargetPoint(n);


			e2.setSourcePoint(n);

			streetNetwork.getEdges().remove(e.getId());
			boolean idNotFound = true;
			int newEdgeId = Integer.MAX_VALUE;
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			e2.setId(newEdgeId);
			//System.out.println("add to network edge: " + e2.getId());
			streetNetwork.getEdges().put(e2.getId(),e2);
			idNotFound = true;
			newEdgeId = Integer.MAX_VALUE;
			while(idNotFound){
				if(streetNetwork.getEdges().containsKey(newEdgeId))
					newEdgeId--;
				else
					idNotFound = false;
			}
			e1.setId(newEdgeId);
			//System.out.println("add to network edge: " + e1.getId());
			streetNetwork.getEdges().put(e1.getId(),e1);


			/*AdjancencyList update*/
			adjacencyList.put(n.getId(), new ArrayList<StreetNode>());

			adjacencyList.get(e.getSourceId()).remove(e.getTargetPoint());
			adjacencyList.get(e.getSourceId()).add(n);
			adjacencyList.get(e.getTargetId()).remove(e.getSourcePoint());
			adjacencyList.get(e.getTargetId()).add(n);

			adjacencyList.get(n.getId()).add(e.getTargetPoint());
			adjacencyList.get(n.getId()).add(e.getSourcePoint());


			int edgeIndex = route.getEdgeList().indexOf(e);

			route.getEdgeList().remove(edgeIndex);
			if(!e.isReversed()){

				route.getEdgeList().add(edgeIndex, e2);
				route.getEdgeList().add(edgeIndex, e1);
			}	
			else{
				//System.out.println("Edge is reversed");
				route.getEdgeList().add(edgeIndex, e1);
				route.getEdgeList().add(edgeIndex, e2);
			}





		} catch (CloneNotSupportedException e3) {
			// TODO Auto-generated catch block
			e3.printStackTrace();
		}



		// TODO Auto-generated method stub

	}


	/***Create node with newPoint and add to edge e. ATENTION if it is polygonal edge, it doesnt update polygonTopo edge lsit*/
	private static StreetNode addNodeToEdge(Point newPoint, boolean pointIsSourceCRS, StreetEdge e, Map<Integer, ArrayList<StreetNode>> adjacencyList,
			Map<Integer, StreetNode> streetNodeMap, StreetNetworkTopological streetNetwork) {
		
		int newEdgeId = Integer.MAX_VALUE;
		int newNodeId = Integer.MAX_VALUE;
		StreetNode newNode = null;
		if(newPoint != null && !(e.getIsPolygonEdge() > 0)){
			boolean idNotFound = true;
			while(idNotFound){
				if(streetNodeMap.containsKey(newNodeId))
					newNodeId--;
				else
					idNotFound = false;
			}
			/*new node*/
			newNode = new StreetNode(newNodeId,newPoint, false, e.getClazz(), pointIsSourceCRS);
			streetNodeMap.put(newNode.getId(), newNode);
			StreetEdge e1,e2;
			/*split edges*/
			try {
				e1 = (StreetEdge)e.clone();
				e2 = (StreetEdge)e.clone();
				/*ATENTION still need to se X1 X2 Y1 Y2 edge geom, cost etc 
				 * UPDATE AdjacencyList*/
				e1.setTargetPoint(newNode);


				e2.setSourcePoint(newNode);

				streetNetwork.getEdges().remove(e.getId());
				idNotFound = true;			
				while(idNotFound){
					if(streetNetwork.getEdges().containsKey(newEdgeId))
						newEdgeId--;
					else
						idNotFound = false;
				}
				e2.setId(newEdgeId);
				//System.out.println("add to network edge: " + e2.getId());
				streetNetwork.getEdges().put(e2.getId(),e2);
				idNotFound = true;
				while(idNotFound){
					if(streetNetwork.getEdges().containsKey(newEdgeId))
						newEdgeId--;
					else
						idNotFound = false;
				}
				e1.setId(newEdgeId);
				//System.out.println("add to network edge: " + e1.getId());
				streetNetwork.getEdges().put(e1.getId(),e1);


				/*AdjancencyList update*/
				adjacencyList.put(newNode.getId(), new ArrayList<StreetNode>());

				adjacencyList.get(e.getSourceId()).remove(e.getTargetPoint());
				adjacencyList.get(e.getSourceId()).add(newNode);
				adjacencyList.get(e.getTargetId()).remove(e.getSourcePoint());
				adjacencyList.get(e.getTargetId()).add(newNode);

				adjacencyList.get(newNode.getId()).add(e.getTargetPoint());
				adjacencyList.get(newNode.getId()).add(e.getSourcePoint());


			} catch (CloneNotSupportedException e3) {
				// TODO Auto-generated catch block
				e3.printStackTrace();
			}
		}
		return newNode;
	}
	



	private static boolean relate(LineString geom, LineString geom2) {

		IntersectionMatrix matrix = geom.relate( geom2 );
		//matrix.
		System.out.println(geom.intersection(geom2));

		return matrix.isIntersects();

	}


	private static void  intersectionPoints(LineString route, LineString region){
		Geometry intersection = route.intersection(region);
		switch ( Geometries.get(intersection) ) {
		case LINESTRING:
			System.out.println("linestring");
			break;
		case POINT:
			System.out.println("point");
			break;
		case MULTIPOINT:
			System.out.println("multipoint");
			break;	
		default:
			break;

		}
		

	}
		
	




}
