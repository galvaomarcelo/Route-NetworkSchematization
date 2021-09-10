package com.wayto.dao;


import java.awt.geom.Point2D;
import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.PrecisionModel;
import org.locationtech.jts.io.ParseException;
import org.locationtech.jts.io.WKBReader;
import org.locationtech.jts.operation.linemerge.LineMerger;
import com.wayto.factory.ConnectionFactory;
import com.wayto.model.Path;
import com.wayto.model.RoundAboutNode;
import com.wayto.model.Route;
import com.wayto.model.RouteEdge;
import com.wayto.model.StreetEdge;
import com.wayto.model.StreetNetworkTopological;
import com.wayto.model.StreetNode;
import com.wayto.operator.GeoConvertionsOperations;



/**
 * Direct Access to Route on postGis routing
 * @author m_deli02
 *
 */
public class RouteDAO extends ConnectionFactory {

	private static RouteDAO instance;
	private String edgeTable = "mland_2po_4pgr3srb"; 
	private String nodeTable = "mland_2po_4pgr3_vertices_pgr"; 


	public static RouteDAO getInstance() {
		if(instance == null)
			instance = new RouteDAO();
		return instance;
	}


	public Route getRoute(double x1, double y1, double x2, double y2, Map<Integer, StreetNode> streetNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork ){

		int sourceNodeId, targetNodeId;

		sourceNodeId = getNearstNode(x1,y1, true);
		targetNodeId = getNearstNode(x2,y2, false);
		WKBReader wkbReader = new WKBReader();
		GeometryFactory geometryFactory = new GeometryFactory( new PrecisionModel(PrecisionModel.FLOATING), 4326);
		StreetNode nodeTarget, nodeSource;

		Route route = null;


		Connection con = null;
		PreparedStatement pstmt = null;
		ResultSet rs = null;

		con = createConection();

		try {
			//Use this querry for route with multiple paths (check pgplugin for QGIS to see how query is produced) */
			/*String sqlQuery = "(WITH result AS " +
					"(SELECT seq, '(' || start_vid || ',' || end_vid || ')' AS path_name, path_seq AS _path_seq, start_vid AS _start_vid, end_vid AS _end_vid, node AS _node, edge AS _edge, cost AS _cost, lead(agg_cost) over() AS _agg_cost " +
					"FROM pgr_dijkstra(" +
					"' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost " +
					"FROM "+edgeTable+" " +
					"WHERE "+edgeTable+".geom_way && ST_MakeEnvelope(6.167688, 51.595243, 8.475906, 52.423065, 4326) ', array["+sourceNodeId+"]::BIGINT[], array["+targetNodeId+"]::BIGINT[], true))" +
					"SELECT CASE WHEN result._node = "+edgeTable+".source THEN ST_AsGeoJSON("+edgeTable+".geom_way) " +
					"ELSE ST_AsGeoJSON(ST_Reverse("+edgeTable+".geom_way)) END " +
					"AS path_geom, result.*, "+edgeTable+".* " +
					"FROM "+edgeTable+" JOIN result ON "+edgeTable+".id = result._edge " +
					"ORDER BY result.seq)";*/

//			String sqlQuery = "SELECT *, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(geom_way) AS geomjson \r\n" +
//					"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost \r\n" +
//					"FROM "+edgeTable+"', "+sourceNodeId+", "+targetNodeId+", true)\r\n" +
//					"JOIN "+edgeTable+" ON "+edgeTable+".id = edge \r\n" +
//					"ORDER BY seq \r\n";
			
			/***Query for non-direct graph. reverse_cost not taken into consideration ****/
			String sqlQuery = "SELECT *, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(geom_way) AS geomjson \r\n" +
					"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost  \r\n" +
					"FROM "+edgeTable+"', "+sourceNodeId+", "+targetNodeId+", false)\r\n" +
					"JOIN "+edgeTable+" ON "+edgeTable+".id = edge \r\n" +
					"ORDER BY seq \r\n";
			
	

			System.out.println(sqlQuery);

			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();


			route = new Route();

			route.setEdgeList(new ArrayList<RouteEdge>());
			String lastSubPathName = "";
			RouteEdge subPath = null;
			while (rs.next()){
				subPath = new RouteEdge();
				subPath.setGeomAsJson(rs.getString("geomjson"));
				subPath.setSeq(rs.getInt("seq"));
				subPath.setName(rs.getString("osm_name"));
				subPath.setCost(rs.getDouble("cost"));
				subPath.setAggCost(rs.getDouble("agg_cost"));
				subPath.setId(rs.getInt("id"));
				subPath.setClazz(rs.getInt("clazz"));
				subPath.setFlags(rs.getInt("flags"));
				subPath.setSpecial(rs.getInt("special"));
				subPath.setKm(rs.getDouble("km"));
				subPath.setKmh(rs.getInt("kmh"));
				subPath.setCost(rs.getDouble("cost"));
				subPath.setReverseCost(rs.getDouble("reverse_cost"));
				subPath.setGeom((LineString)(wkbReader.read( rs.getBytes("geom") )));
//				subPath.setGeom((LineString)wkbReader.read(rs.getBytes("geom")));


				/*Instaciate source and target nodes add to edge subPath and add to Station map if they dont exist*/
				int sourceId = rs.getInt("source");
				int targetId = rs.getInt("target");


				/*chech if source->target order confirm if route order*/ 
				if (rs.getLong("node")== sourceId) 
					subPath.setReversed(false);
				else subPath.setReversed(true);



				if(streetNodeMap.containsKey(sourceId)){
					nodeSource = streetNodeMap.get(sourceId);
					nodeSource.setRouteNode(true);
					subPath.setSourcePoint(nodeSource);
				}
				else{
					Coordinate coordSource = new Coordinate();
					coordSource.x = rs.getDouble("x1");
					coordSource.y = rs.getDouble("y1");
					nodeSource = new StreetNode(sourceId,geometryFactory.createPoint(coordSource),true, 0, true );
					streetNodeMap.put(sourceId, nodeSource);
					subPath.setSourcePoint(nodeSource);
					adjacencyList.put(sourceId, new ArrayList<StreetNode>());
					
				}


				if(streetNodeMap.containsKey(targetId)){
					nodeTarget = streetNodeMap.get(targetId);
					nodeTarget.setRouteNode(true);
					subPath.setTargetPoint(nodeTarget);
				}
				else{
					Coordinate coordTarget = new Coordinate();
					coordTarget.x = rs.getDouble("x2");
					coordTarget.y = rs.getDouble("y2");
					nodeTarget = new StreetNode(targetId,geometryFactory.createPoint(coordTarget),true, 0 , true );
					streetNodeMap.put(targetId, nodeTarget);
					subPath.setTargetPoint(nodeTarget);
					adjacencyList.put(targetId, new ArrayList<StreetNode>());
					//adjacencyList.get(targetId).add(nodeTarget);

				}

				
				/*Find Decision points in the route (street change name)*/

				if( subPath.getName() != null && !subPath.getName().equals(lastSubPathName)){
					System.out.println(lastSubPathName  + " to " + subPath.getName());
					if(!subPath.isReversed())
						nodeSource.setDecisionPoint(true);
					else nodeTarget.setDecisionPoint(true);
					lastSubPathName = subPath.getName();

				}


//				if (rs.isFirst()){
//
//					if(!subPath.isReversed())
//						route.setStart(nodeSource);
//					else route.setStart(nodeTarget);
//				}
//				if (rs.isLast())
//					if(!subPath.isReversed()){
//						route.setEnd(nodeTarget);
//
//					}	
//					else{
//						route.setEnd(nodeSource);
//					}


				/*Add nodes to Adjacency list*/
				if (!adjacencyList.get(sourceId).contains(nodeTarget))
					adjacencyList.get(sourceId).add(nodeTarget);


				if (!adjacencyList.get(targetId).contains(nodeSource))
					adjacencyList.get(targetId).add(nodeSource);

				route.getEdgeList().add(subPath);
				streetNetwork.getEdges().put(subPath.getId(), subPath);


			}
			rs.close();
			pstmt.close();

			/*
			 * Create Route geom form routeGeom ArrayList.
			 */
//			LineMerger merger = new LineMerger();
//			for (int i = 0; i < route.getEdgeList().size(); i++) {
//				merger.add(route.getEdgeList().get(i).getGeom());
//
//			}
//
//			Collection<LineString> collection = merger.getMergedLineStrings();
//
//			for (LineString l : collection) {
//				route.setGeom(l);
//				//System.out.println(l);
//			}

			/*	

			sqlQuery = "SELECT ST_AsBinary(ST_LineMerge(ST_UNION(geom_way))) AS geom \r\n" +
					"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost FROM "+edgeTable+" ', "+sourceNodeId+", "+targetNodeId+", true) \r\n" +
					"JOIN "+edgeTable+" ON "+edgeTable+".id = edge\r\n";
			System.out.println(sqlQuery);

			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();
			while (rs.next()){
				System.out.println(wkbReader.read(rs.getBytes("geom")));
				Geometry routeGeom = wkbReader.read(rs.getBytes("geom"));
				switch ( Geometries.get(routeGeom) ) {
		        case LINESTRING:
		        	route.setGeom((LineString)routeGeom);

		        	break;
		        case MULTILINESTRING:
		        	MultiLineString lines = (MultiLineString) routeGeom;
		        	route.setGeom( (LineString)lines.getGeometryN(0));      	
		        	break;

				default:
					route.setGeom((LineString)routeGeom);
					break;
				}		

			}*/
			//System.out.println(route.getGeom());


		} catch (Exception e) {
			System.out.println("Fail to list RoutPath" + e);
		}finally{
			closeConnection(con, pstmt, rs);
		}



		return route;
	}
	
	public Route getRouteRBCollapse(double x1, double y1, double x2, double y2, Map<Integer, StreetNode> streetNodeMap, Map<Integer, RoundAboutNode> rbNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork ){

		int sourceNodeId, targetNodeId;

		sourceNodeId = getNearstNode(x1,y1, true);
		targetNodeId = getNearstNode(x2,y2, false);
		WKBReader wkbReader = new WKBReader();
		GeometryFactory geometryFactory = new GeometryFactory( new PrecisionModel(PrecisionModel.FLOATING), 4326);
		

		Route route = null;


		Connection con = null;
		PreparedStatement pstmt = null;
		ResultSet rs = null;

		con = createConection();

		try {
			//Use this querry for route with multiple paths (check pgplugin for QGIS to see how query is produced) */
			/*String sqlQuery = "(WITH result AS " +
					"(SELECT seq, '(' || start_vid || ',' || end_vid || ')' AS path_name, path_seq AS _path_seq, start_vid AS _start_vid, end_vid AS _end_vid, node AS _node, edge AS _edge, cost AS _cost, lead(agg_cost) over() AS _agg_cost " +
					"FROM pgr_dijkstra(" +
					"' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost " +
					"FROM "+edgeTable+" " +
					"WHERE "+edgeTable+".geom_way && ST_MakeEnvelope(6.167688, 51.595243, 8.475906, 52.423065, 4326) ', array["+sourceNodeId+"]::BIGINT[], array["+targetNodeId+"]::BIGINT[], true))" +
					"SELECT CASE WHEN result._node = "+edgeTable+".source THEN ST_AsGeoJSON("+edgeTable+".geom_way) " +
					"ELSE ST_AsGeoJSON(ST_Reverse("+edgeTable+".geom_way)) END " +
					"AS path_geom, result.*, "+edgeTable+".* " +
					"FROM "+edgeTable+" JOIN result ON "+edgeTable+".id = result._edge " +
					"ORDER BY result.seq)";*/

//			String sqlQuery = "SELECT *, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(geom_way) AS geomjson \r\n" +
//					"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost \r\n" +
//					"FROM "+edgeTable+"', "+sourceNodeId+", "+targetNodeId+", true)\r\n" +
//					"JOIN "+edgeTable+" ON "+edgeTable+".id = edge \r\n" +
//					"ORDER BY seq \r\n";
			
			/***Query for non-direct graph. reverse_cost not taken into consideration ****/
			String sqlQuery = "SELECT * , ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom,  ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(geom_way) AS geomjson \r\n" +
					"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost  \r\n" +
					"FROM "+edgeTable+"', "+sourceNodeId+", "+targetNodeId+", false)\r\n" +
					"JOIN "+edgeTable+" ON "+edgeTable+".id = edge \r\n" +
					"ORDER BY seq \r\n";
			
	

			System.out.println(sqlQuery);

			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();


			route = new Route();

			route.setEdgeList(new ArrayList<RouteEdge>());
			String lastSubPathName = "";
			RouteEdge subPath = null;
			StreetNode lastNode = null;
			
		    LineMerger merger = new LineMerger();



			
			while (rs.next()){
				//System.out.println("Edge id= " + rs.getInt("id"));
				/*if is not a round about edge*/
				if(rs.getInt("special")!=1){
					subPath = new RouteEdge();
					subPath.setGeomAsJson(rs.getString("geomjson"));
					subPath.setSeq(rs.getInt("seq"));
					subPath.setName(rs.getString("osm_name"));
					subPath.setCost(rs.getDouble("cost"));
					subPath.setAggCost(rs.getDouble("agg_cost"));
					subPath.setId(rs.getInt("id"));
					subPath.setClazz(rs.getInt("clazz"));
					subPath.setFlags(rs.getInt("flags"));
					subPath.setSpecial(rs.getInt("special"));
					subPath.setKm(rs.getDouble("km"));
					subPath.setKmh(rs.getInt("kmh"));
					
					subPath.setGeom((LineString)(wkbReader.read( rs.getBytes("geom") )));
					
					
					

					subPath.setReverseCost(rs.getDouble("reverse_cost"));
//					subPath.setGeom((LineString)wkbReader.read(rs.getBytes("geom")));


					/*Instaciate source and target nodes add to edge subPath and add to Station map if they dont exist*/

					/*chech if source->target order confirm if route order*/ 
					if (rs.getLong("node")== rs.getInt("source")) 
						subPath.setReversed(false);
					else subPath.setReversed(true);
					
					/*reads edge linestring to create route original geometry*/
					LineString edgeGeom = (LineString)(wkbReader.read( rs.getBytes("geom") ));
					if(!subPath.isReversed())
						merger.add(edgeGeom);
					else
						merger.add(edgeGeom.reverse());
					
					
					int sourceId = rs.getInt("source");
					int targetId = rs.getInt("target");


					int rbsourceId = rs.getInt("rbsourceid");
					int rbtargetId = rs.getInt("rbtargetid");

					

					if( rbsourceId !=0 ){					
						if(rbNodeMap.containsKey(rbsourceId)){
							RoundAboutNode rbNode = rbNodeMap.get(rbsourceId);
							rbNode.setRouteNode(true);				
							subPath.setSourcePoint(rbNode);
							sourceId = rbNode.getId();
						}
						else{
							Point rbsPoint = (Point)wkbReader.read(rs.getBytes("rbsgeom"));
							RoundAboutNode rbNode = new RoundAboutNode(rbsourceId, sourceId, rbsPoint, true, 0);
							rbNode.setDecisionPoint(true);
							rbNodeMap.put(rbsourceId, rbNode);
							streetNodeMap.put(sourceId, rbNode);
							subPath.setSourcePoint(rbNode);
							adjacencyList.put(sourceId, new ArrayList<StreetNode>());						
						}
					}
					else{
						if(streetNodeMap.containsKey(sourceId)){
							StreetNode nodeSource = streetNodeMap.get(sourceId);
							nodeSource.setRouteNode(true);
							subPath.setSourcePoint(nodeSource);
						}
						else{
							Coordinate coordSource =  subPath.getGeom().getPointN(0).getCoordinate();
//							Coordinate coordSource = new Coordinate();
//							coordSource.x = rs.getDouble("x1");
//							coordSource.y = rs.getDouble("y1");
							StreetNode nodeSource = new StreetNode(sourceId,geometryFactory.createPoint(coordSource),true, 0 , true );
							streetNodeMap.put(sourceId, nodeSource);
							subPath.setSourcePoint(nodeSource);
							adjacencyList.put(sourceId, new ArrayList<StreetNode>());

						}

					}


					if( rbtargetId !=0 ){					
						if(rbNodeMap.containsKey(rbtargetId)){
							RoundAboutNode rbNode = rbNodeMap.get(rbtargetId);
							rbNode.setRouteNode(true);				
							subPath.setTargetPoint(rbNode);
							targetId = rbNode.getId();
						}
						else{
							Point rbsPoint = (Point)wkbReader.read(rs.getBytes("rbtgeom"));
							RoundAboutNode rbNode = new RoundAboutNode(rbtargetId, targetId, rbsPoint, true, 0);
							rbNode.setDecisionPoint(true);
							rbNodeMap.put(rbtargetId, rbNode);
							streetNodeMap.put(targetId, rbNode);
							subPath.setTargetPoint(rbNode);
							adjacencyList.put(targetId, new ArrayList<StreetNode>());						
						}
					}
					else{

						if(streetNodeMap.containsKey(targetId)){
							 
							StreetNode nodeTarget = streetNodeMap.get(targetId);
							nodeTarget.setRouteNode(true);
							subPath.setTargetPoint(nodeTarget);
						}
						else{
							Coordinate coordTarget =  subPath.getGeom().getPointN(subPath.getGeom().getNumPoints() -1).getCoordinate(); 
//							Coordinate coordTarget = new Coordinate();
//							coordTarget.x = rs.getDouble("x2");
//							coordTarget.y = rs.getDouble("y2");
							StreetNode nodeTarget = new StreetNode(targetId,geometryFactory.createPoint(coordTarget),true, 0 , true);
							streetNodeMap.put(targetId, nodeTarget);
							subPath.setTargetPoint(nodeTarget);
							adjacencyList.put(targetId, new ArrayList<StreetNode>());
							//adjacencyList.get(targetId).add(nodeTarget);

						}
					}


					if(!subPath.isReversed()){
						lastNode = subPath.getTargetPoint();
					}
					else{

						lastNode = subPath.getSourcePoint();
					}

					/*Find Decision points in the route (street change name)*/
					if( subPath.getName() != null && !subPath.getName().equals(lastSubPathName) && !rs.isFirst()){
						System.out.println(lastSubPathName  + " to " + subPath.getName());
						if(!subPath.isReversed())
							subPath.getSourcePoint().setDecisionPoint(true);
						else subPath.getTargetPoint().setDecisionPoint(true);
						

					}
					lastSubPathName = subPath.getName();
					

					/*Add nodes to Adjacency list*/
					if (!adjacencyList.get(subPath.getSourceId()).contains(subPath.getTargetPoint()))
						adjacencyList.get(subPath.getSourceId()).add(subPath.getTargetPoint());


					if (!adjacencyList.get(subPath.getTargetId()).contains(subPath.getSourcePoint()))
						adjacencyList.get(subPath.getTargetId()).add(subPath.getSourcePoint());

					route.getEdgeList().add(subPath);
					streetNetwork.getEdges().put(subPath.getId(), subPath);
				}
				else{
					System.out.println("Round about edge found, id= " + rs.getInt("id"));
				}

			}
			
			rs.close();
			pstmt.close();
			
			
		    Collection<LineString> collection = merger.getMergedLineStrings();
		    ArrayList<LineString> lineStrings = new ArrayList<>(collection);		    
//		    for (LineString l : lineStrings) {
//		      System.out.println(l);
//		    }
		    route.setGeom(lineStrings.get(0));

		} catch (Exception e) {
			System.out.println("Fail to list RoutPath" + e);
		}finally{
			closeConnection(con, pstmt, rs);
		}



		return route;
	}


	public boolean getStreetNetWork(Route route, double buffer, Map<Integer, StreetNode> streetNodeMap, Map<Integer, RoundAboutNode> rbNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork, boolean includeStubs  ){


		  /**
		Highway1 11
		Highway link 12
		Highway2(trunk) 13
		Highway2(trunk) link 14
		Primary 15
		Primary link 16
		Secondary 21
		Secondary link 22
		Tertiary 31
		Tertiary link 32
		Residential 41
		Unknow 42
		Not classified/Serivice/Rural  43
		Lowpriority(not street)  100
		*/


		Connection con = null;
		PreparedStatement pstmt = null;
		ResultSet rs = null;

		con = createConection();

	
		System.out.println(route.getRoutePath().getNodeList());
		LineString routeLS = route.getRoutePath().asLineString(4);
		double extend = routeLS.getEnvelopeInternal().maxExtent();
		String sqlQuery = "";
		String routeSelection = null;
		System.out.println("extend: " + extend);
		try {

			/*	sqlQuery = "SELECT e.*, ST_AsGeoJSON(e.geom_way) AS geom \r\n" +
					"FROM "+edgeTable+" AS e \r\n" +
					"JOIN (SELECT ST_BUFFER(ST_LineMerge(ST_UNION(geom_way)),0.002) AS route_geom \r\n" +
					"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost FROM "+edgeTable+" ', "+sourceNodeId+", "+targetNodeId+", true) \r\n" +
					"JOIN "+edgeTable+" ON "+edgeTable+".id = edge) route\r\n" +
					"ON ST_Intersects(e.geom_way, route.route_geom)\r\n";*/
			if(includeStubs) {
			/***Select Roads very close to the route STUBS***/
			
			
				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), 0.0000000009))\r\n " +
						//"AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 42 OR e.clazz = 43) \r\n";	
				        "AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) \r\n";
				//"AND (e.clazz <> 11 AND e.clazz <> 13 AND e.clazz <> 15 AND e.clazz <> 21) \r\n";
	
				System.out.println(sqlQuery);
	
				pstmt = con.prepareStatement(sqlQuery);
				rs = pstmt.executeQuery();
	
				loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, true );
				rs.close();
				pstmt.close();
			}
			
			/***Select Roads close to the route***/
			if(extend < 0.04)
				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21  OR e.clazz = 31 ) ";
			else if (extend < 0.1)	
				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 ) ";
			else	
				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 ) ";
			
				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), 0.027))\r\n " +
						"AND "+routeSelection+" \r\n";	
			
			

			System.out.println(sqlQuery);

			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();

			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );
			rs.close();
			pstmt.close();


//
			/*******Select Main Roads in MinimunBounding Circle *******/

//			if(extend > 0.10)
//				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 )";
//			else
//				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )";
//
//			sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
//					"FROM "+edgeTable+" AS e \r\n" +
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_MinimumBoundingCircle(ST_GeomFromText('"+routeLS+"',4326)), "+extend/100+"))\r\n" +
//					"AND "+routeSelection+" \r\n";
//
//			System.out.println(sqlQuery);
//
//			pstmt = con.prepareStatement(sqlQuery);
//			rs = pstmt.executeQuery();
//
//			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );
//			rs.close();
//		   pstmt.close();

//			/*******Select Roads around Origin *******/

			if(extend > 0.10)
				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )";
			else
				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31)";

			sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
					"FROM "+edgeTable+" AS e \r\n" +
					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getStart().getCoordinate()+"',4326), "+extend/7+"))\r\n " +
					"OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getEnd().getCoordinate()+"',4326), "+extend/7+")) ) \r\n " +
					"AND "+routeSelection+" \r\n";	

			
			System.out.println(sqlQuery);

			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();

			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );

			rs.close();
			pstmt.close();


		} catch (Exception e) {
			System.out.println("Fail to list RoutPath" + e);
			return false;
		}finally{
			closeConnection(con, pstmt, rs);
		}

		return true;
	}

	
	public boolean getStreetNetWork2(Route route, double buffer, Map<Integer, StreetNode> streetNodeMap, Map<Integer, RoundAboutNode> rbNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork, boolean includeStubs  ){


		  /**
		Highway1 11
		Highway link 12
		Highway2(trunk) 13
		Highway2(trunk) link 14
		Primary 15
		Primary link 16
		Secondary 21
		Secondary link 22
		Tertiary 31
		Tertiary link 32
		Residential 41
		Unknow 42
		Not classified/Serivice/Rural  43
		Lowpriority(not street)  100
		*/


		Connection con = null;
		PreparedStatement pstmt = null;
		ResultSet rs = null;

		con = createConection();

		LineString routeLS = route.getRoutePath().asLineString(4);
		double extend = routeLS.getEnvelopeInternal().maxExtent();
		String sqlQuery = "";
		String routeSelection = null;
		System.out.println("extend: " + extend);
		try {

			/*	sqlQuery = "SELECT e.*, ST_AsGeoJSON(e.geom_way) AS geom \r\n" +
					"FROM "+edgeTable+" AS e \r\n" +
					"JOIN (SELECT ST_BUFFER(ST_LineMerge(ST_UNION(geom_way)),0.002) AS route_geom \r\n" +
					"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost FROM "+edgeTable+" ', "+sourceNodeId+", "+targetNodeId+", true) \r\n" +
					"JOIN "+edgeTable+" ON "+edgeTable+".id = edge) route\r\n" +
					"ON ST_Intersects(e.geom_way, route.route_geom)\r\n";*/

			
			if(includeStubs) {
			/***Select Roads very close to the route STUBS***/
			
			
				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), 0.0000000009))\r\n " +
						//"AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 42 OR e.clazz = 43) \r\n";	
				        "AND (e.clazz = 31 OR e.clazz = 41) \r\n";
				//"AND (e.clazz <> 11 AND e.clazz <> 13 AND e.clazz <> 15 AND e.clazz <> 21) \r\n";
	
				System.out.println(sqlQuery);
	
				pstmt = con.prepareStatement(sqlQuery);
				rs = pstmt.executeQuery();
	
				loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, true );
				rs.close();
				pstmt.close();
			}
			
//			/***Select Roads very close to the route***/
//			
//			sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
//					"FROM "+edgeTable+" AS e \r\n" +
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getGeom()+"',4326), 0.0000009))\r\n " +
//					"AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 42 OR e.clazz = 43) \r\n";	
//			//"AND (e.clazz <> 11 AND e.clazz <> 13 AND e.clazz <> 15 AND e.clazz <> 21) \r\n";
//
//			System.out.println(sqlQuery);
//
//			pstmt = con.prepareStatement(sqlQuery);
//			rs = pstmt.executeQuery();
//
//			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList );
//			rs.close();
//			pstmt.close();
			
			/***Select Roads close to the route***/
			if(extend < 0.04)
				routeSelection =  "(e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 ) ";
			else if (extend < 0.1)	
				routeSelection =  "(e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 ) ";
			else	
				routeSelection =  "(e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 ) ";
			
				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), 0.002))\r\n " +
						"AND "+routeSelection+" \r\n";	
			
			

			System.out.println(sqlQuery);

			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();

			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );
			rs.close();
			pstmt.close();



			/*******Select Main Roads in MinimunBounding Circle *******/

			if(extend > 0.10)
				routeSelection =  "(e.clazz = 11 )";
			else
				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )";

			sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
					"FROM "+edgeTable+" AS e \r\n" +
					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_MinimumBoundingCircle(ST_GeomFromText('"+routeLS+"',4326)), "+extend/2+"))\r\n" +
					"AND "+routeSelection+" \r\n";

			System.out.println(sqlQuery);

			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();

			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false);


			/*******Select Roads around Origin *******/
				rs.close();
			pstmt.close();
			if(extend > 0.10)
				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )";
			else
				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )";

			sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
					"FROM "+edgeTable+" AS e \r\n" +
					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getStart().getGeom()+"',4326), "+extend/8+"))\r\n " +
					"OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getEnd().getGeom()+"',4326), "+extend/8+")) ) \r\n " +
					"AND "+routeSelection+" \r\n";	

			System.out.println(sqlQuery);

			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();

			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );

			rs.close();
			pstmt.close();


		} catch (Exception e) {
			System.out.println("Fail to list RoutPath" + e);
			return false;
		}finally{
			closeConnection(con, pstmt, rs);
		}



		return true;
	}

	
	
	
	/*Used Only to produce maps for the experiment Route+Polygon*/
	public boolean getStreetNetWorkSimple(Route route, double buffer, Map<Integer, StreetNode> streetNodeMap, Map<Integer, RoundAboutNode> rbNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork  ){


		  /**
		Highway1 11
		Highway link 12
		Highway2(trunk) 13
		Highway2(trunk) link 14
		Primary 15
		Primary link 16
		Secondary 21
		Secondary link 22
		Tertiary 31
		Tertiary link 32
		Residential 41
		Unknow 42
		Not classified/Serivice/Rural  43
		Lowpriority(not street)  100
		*/


		Connection con = null;
		PreparedStatement pstmt = null;
		ResultSet rs = null;

		con = createConection();


		LineString routeLS = route.getRoutePath().asLineString(4);
		double extend = routeLS.getEnvelopeInternal().maxExtent();
		String sqlQuery = "";
		String routeSelection = null;
		System.out.println("extend: " + extend);
		try {

			/*	sqlQuery = "SELECT e.*, ST_AsGeoJSON(e.geom_way) AS geom \r\n" +
					"FROM "+edgeTable+" AS e \r\n" +
					"JOIN (SELECT ST_BUFFER(ST_LineMerge(ST_UNION(geom_way)),0.002) AS route_geom \r\n" +
					"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost FROM "+edgeTable+" ', "+sourceNodeId+", "+targetNodeId+", true) \r\n" +
					"JOIN "+edgeTable+" ON "+edgeTable+".id = edge) route\r\n" +
					"ON ST_Intersects(e.geom_way, route.route_geom)\r\n";*/

			/***Select Roads very close to the route***/
			
//			sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
//					"FROM "+edgeTable+" AS e \r\n" +
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getGeom()+"',4326), 0.0000009))\r\n " +
//					"AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 42 OR e.clazz = 43) \r\n";	
//			//"AND (e.clazz <> 11 AND e.clazz <> 13 AND e.clazz <> 15 AND e.clazz <> 21) \r\n";
//
//			System.out.println(sqlQuery);
//
//			pstmt = con.prepareStatement(sqlQuery);
//			rs = pstmt.executeQuery();
//
//			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList );
//			rs.close();
//			pstmt.close();
			
			/***Select Roads close to the route***/
//			if(extend < 0.04)
//				routeSelection =  "(e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21   ) ";
//			else if (extend < 0.1)	
//				routeSelection =  "(e.clazz = 13 OR e.clazz = 15  ) ";
//			else	
//				routeSelection =  "(e.clazz = 13 OR e.clazz = 15 ) ";
//			
//				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
//						"FROM "+edgeTable+" AS e \r\n" +
//						"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getGeom()+"',4326), 0.0002))\r\n " +
//						"AND "+routeSelection+" \r\n";	
//			
//			
//
//			System.out.println(sqlQuery);
//
//			pstmt = con.prepareStatement(sqlQuery);
//			rs = pstmt.executeQuery();
//
//			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList );
//			rs.close();
//			pstmt.close();



			/*******Select Main Roads in MinimunBounding Circle *******/

//			if(extend > 0.10)
//				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 )";
//			else
//				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 )";
//
//			sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
//					"FROM "+edgeTable+" AS e \r\n" +
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_MinimumBoundingCircle(ST_GeomFromText('"+route.getGeom()+"',4326)), "+extend/15+"))\r\n" +
//					"AND "+routeSelection+" \r\n";
//
//			System.out.println(sqlQuery);
//
//			pstmt = con.prepareStatement(sqlQuery);
//			rs = pstmt.executeQuery();
//
//			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList );
//			rs.close();
//		pstmt.close();

			/*******Select Roads around Origin *******/

//			if(extend > 0.10)
//				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )";
//			else
//				routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31)";
//
//			sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
//					"FROM "+edgeTable+" AS e \r\n" +
//					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getStart().getGeom()+"',4326), "+extend/5+"))\r\n " +
//					"OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getEnd().getGeom()+"',4326), "+extend/5+")) ) \r\n " +
//					"AND "+routeSelection+" \r\n";	
//
//			System.out.println(sqlQuery);
//
//			pstmt = con.prepareStatement(sqlQuery);
//			rs = pstmt.executeQuery();
//
//			loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList );
//
//			rs.close();
//			pstmt.close();


		} catch (Exception e) {
			System.out.println("Fail to list RoutPath" + e);
			return false;
		}finally{
			closeConnection(con, pstmt, rs);
		}



		return true;
	}


	
	

	/***Private Method that get a resultset and builds the Roads Data Structures (StreetNetWork, AdjacencyList, and List of Node )
	 *it need to be called inside a TryCatch statement because it throws Exceptions. 
	 * @param rbNodeMap 
	 * ***/
	private void loadRoadsToDataStructure(ResultSet rs, Map<Integer, StreetNode> streetNodeMap,
			Map<Integer, RoundAboutNode> rbNodeMap, StreetNetworkTopological streetNetWork, Map<Integer, ArrayList<StreetNode>> adjacencyList, boolean stubsOnly) throws SQLException, ParseException {

		StreetNode nodeTarget, nodeSource;
		StreetEdge subPath = null;
		WKBReader wkbReader = new WKBReader();
		GeometryFactory geometryFactory = new GeometryFactory( new PrecisionModel(PrecisionModel.FLOATING), 4326);


		while (rs.next()){
			int edgeId = rs.getInt("id");
			
			if(rs.getInt("special")!=1){
				if(!streetNetWork.getEdges().containsKey(rs.getInt("id"))){


					subPath = new StreetEdge();
					subPath.setGeomAsJson(rs.getString("geomjson"));
					subPath.setId(rs.getInt("id"));
					subPath.setName(rs.getString("osm_name"));
					subPath.setClazz(rs.getInt("clazz"));
					subPath.setFlags(rs.getInt("flags"));
					subPath.setSpecial(rs.getInt("special"));
					subPath.setKm(rs.getDouble("km"));
					subPath.setKmh(rs.getInt("kmh"));
					subPath.setCost(rs.getDouble("cost"));
					subPath.setReverseCost(rs.getDouble("reverse_cost"));
					subPath.setGeom((LineString)(wkbReader.read( rs.getBytes("geom") )));
					//				subPath.setGeom((LineString)wkbReader.read(rs.getBytes("geom")));

					/*Instaciate source and target nodes add to edge subPath and add to Station map if they dont exist*/
					int sourceId = rs.getInt("source");
					int targetId = rs.getInt("target");



					int rbsourceId = rs.getInt("rbsourceid");
					int rbtargetId = rs.getInt("rbtargetid");



					/***Treat Edge source node: check if is round abound, and if already exist***/
					if( rbsourceId !=0 ){					
						if(rbNodeMap.containsKey(rbsourceId)){
							RoundAboutNode rbNode = rbNodeMap.get(rbsourceId);	
							if(rbNode.getPriority() > subPath.getClazz())
								rbNode.setPriority( subPath.getClazz() );
							subPath.setSourcePoint(rbNode);
							sourceId = rbNode.getId();
						}
						else{
							Point rbsPoint = (Point)wkbReader.read(rs.getBytes("rbsgeom"));
							RoundAboutNode rbNode = new RoundAboutNode(rbsourceId, sourceId, rbsPoint, false, 0);
							rbNodeMap.put(rbsourceId, rbNode);
							streetNodeMap.put(sourceId, rbNode);
							subPath.setSourcePoint(rbNode);
							adjacencyList.put(sourceId, new ArrayList<StreetNode>());						
						}
					}
					else{
						if(streetNodeMap.containsKey(sourceId)){
							if(streetNodeMap.get(sourceId).getPriority() > subPath.getClazz())
								streetNodeMap.get(sourceId).setPriority( subPath.getClazz() );
							nodeSource = streetNodeMap.get(sourceId);
							subPath.setSourcePoint(nodeSource);
						}
						else{
							Coordinate coordSource =  subPath.getGeom().getPointN(0).getCoordinate();
							//coordSource.x = rs.getDouble("x1");
							//coordSource.y = rs.getDouble("y1");
							
							nodeSource = new StreetNode(sourceId,geometryFactory.createPoint(coordSource),false, subPath.getClazz(), true);
							streetNodeMap.put(sourceId, nodeSource);
							subPath.setSourcePoint(nodeSource);
							adjacencyList.put(sourceId, new ArrayList<StreetNode>());

						}

					}
					/***Treat Edge target node: check if is round abound, and if already exist***/
					if( rbtargetId !=0 ){					
						if(rbNodeMap.containsKey(rbtargetId)){
							RoundAboutNode rbNode = rbNodeMap.get(rbtargetId);	
							if(rbNode.getPriority() > subPath.getClazz())
								rbNode.setPriority( subPath.getClazz() );
							subPath.setTargetPoint(rbNode);
							targetId = rbNode.getId();
						}
						else{
							Point rbsPoint = (Point)wkbReader.read(rs.getBytes("rbtgeom"));
							RoundAboutNode rbNode = new RoundAboutNode(rbtargetId, targetId, rbsPoint, false, 0);
							rbNodeMap.put(rbtargetId, rbNode);
							streetNodeMap.put(targetId, rbNode);
							subPath.setTargetPoint(rbNode);
							adjacencyList.put(targetId, new ArrayList<StreetNode>());						
						}
					}
					else{

						if(streetNodeMap.containsKey(targetId)){

							if(streetNodeMap.get(targetId).getPriority() > subPath.getClazz())
								streetNodeMap.get(targetId).setPriority( subPath.getClazz() );
							nodeTarget = streetNodeMap.get(targetId);
							subPath.setTargetPoint(nodeTarget);
						}
						else{
							Coordinate coordTarget =  subPath.getGeom().getPointN(subPath.getGeom().getNumPoints() -1).getCoordinate(); 
							//coordTarget.x = rs.getDouble("x2");
							//coordTarget.y = rs.getDouble("y2");
							nodeTarget = new StreetNode(targetId,geometryFactory.createPoint(coordTarget),false, subPath.getClazz() , true );
							streetNodeMap.put(targetId, nodeTarget);
							subPath.setTargetPoint(nodeTarget);
							adjacencyList.put(targetId, new ArrayList<StreetNode>());
							//adjacencyList.get(targetId).add(nodeTarget);

						}
					}



					/*Add nodes to Adjacency list*/
					if (!adjacencyList.get(sourceId).contains(subPath.getTargetPoint()))
						adjacencyList.get(sourceId).add(subPath.getTargetPoint());


					if (!adjacencyList.get(targetId).contains(subPath.getSourcePoint()))
						adjacencyList.get(targetId).add(subPath.getSourcePoint());

					if(stubsOnly)
						subPath.setStubEdge(true);
					streetNetWork.getEdges().put(subPath.getId(), subPath);

				}
				else if( !(streetNetWork.getEdges().get(edgeId) instanceof RouteEdge)  &&  stubsOnly){
					
						streetNetWork.getEdges().get(edgeId).setStubEdge(true);
					
				}
				
			}
			else {
				System.out.println("Load network. Round about edge found, id= " + rs.getInt("id"));
			}

		}

	}
	
	
	private void loadRoadsToDataStructure2(ResultSet rs, Map<Integer, StreetNode> streetNodeMap,
			Map<Integer, RoundAboutNode> rbNodeMap, StreetNetworkTopological streetNetWork, Map<Integer, ArrayList<StreetNode>> adjacencyList,  boolean stubsOnly) throws SQLException, ParseException {

		StreetNode nodeTarget, nodeSource;
		StreetEdge subPath = null;
		WKBReader wkbReader = new WKBReader();
		GeometryFactory geometryFactory = new GeometryFactory( new PrecisionModel(PrecisionModel.FLOATING), 4326);


		while (rs.next()){
			if(!streetNetWork.getEdges().containsKey(rs.getInt("id"))){


				subPath = new StreetEdge();
				subPath.setGeomAsJson(rs.getString("geomjson"));
				subPath.setId(rs.getInt("id"));
				subPath.setName(rs.getString("osm_name"));
				subPath.setClazz(rs.getInt("clazz"));
				subPath.setFlags(rs.getInt("flags"));
				subPath.setSpecial(rs.getInt("special"));

				subPath.setKm(rs.getDouble("km"));
				subPath.setKmh(rs.getInt("kmh"));
				subPath.setCost(rs.getDouble("cost"));
				subPath.setReverseCost(rs.getDouble("reverse_cost"));
//				subPath.setGeom((LineString)wkbReader.read(rs.getBytes("geom")));

				/*Instaciate source and target nodes add to edge subPath and add to Station map if they dont exist*/
				int sourceId = rs.getInt("source");
				int targetId = rs.getInt("target");
				
				

				
				int rbsourceId = rs.getInt("rbsourceid");
				int rbtargetId = rs.getInt("rbtargetid");

				
				
				/***Treat Edge source node: check if is round abound, and if already exist***/
				if( rbsourceId !=0 ){					
					if(rbNodeMap.containsKey(rbsourceId)){
						RoundAboutNode rbNode = rbNodeMap.get(rbsourceId);	
						if(rbNode.getPriority() > subPath.getClazz())
							rbNode.setPriority( subPath.getClazz() );
						subPath.setSourcePoint(rbNode);

						sourceId = rbNode.getId();
					}
					else{
						Point rbsPoint = (Point)wkbReader.read(rs.getBytes("rbsgeom"));
						RoundAboutNode rbNode = new RoundAboutNode(rbsourceId, sourceId, rbsPoint, false, 0);
						rbNodeMap.put(rbsourceId, rbNode);
						streetNodeMap.put(sourceId, rbNode);
						subPath.setSourcePoint(rbNode);
						adjacencyList.put(sourceId, new ArrayList<StreetNode>());						
					}
				}
				else{
					if(streetNodeMap.containsKey(sourceId)){
						if(streetNodeMap.get(sourceId).getPriority() > subPath.getClazz())
							streetNodeMap.get(sourceId).setPriority( subPath.getClazz() );
						nodeSource = streetNodeMap.get(sourceId);
						subPath.setSourcePoint(nodeSource);
					}
					else{
						Coordinate coordSource = new Coordinate();
						coordSource.x = rs.getDouble("x1");
						coordSource.y = rs.getDouble("y1");
						nodeSource = new StreetNode(sourceId,geometryFactory.createPoint(coordSource),false, subPath.getClazz(), true);
						streetNodeMap.put(sourceId, nodeSource);
						subPath.setSourcePoint(nodeSource);
						adjacencyList.put(sourceId, new ArrayList<StreetNode>());

					}

				}
				/***Treat Edge target node: check if is round abound, and if already exist***/
				if( rbtargetId !=0 ){					
					if(rbNodeMap.containsKey(rbtargetId)){
						RoundAboutNode rbNode = rbNodeMap.get(rbtargetId);	
						if(rbNode.getPriority() > subPath.getClazz())
							rbNode.setPriority( subPath.getClazz() );
						subPath.setTargetPoint(rbNode);
						targetId = rbNode.getId();
					}
					else{
						Point rbsPoint = (Point)wkbReader.read(rs.getBytes("rbtgeom"));
						RoundAboutNode rbNode = new RoundAboutNode(rbtargetId, targetId, rbsPoint, false, 0);
						rbNodeMap.put(rbtargetId, rbNode);
						streetNodeMap.put(targetId, rbNode);
						subPath.setTargetPoint(rbNode);
						adjacencyList.put(targetId, new ArrayList<StreetNode>());						
					}
				}
				else{

					if(streetNodeMap.containsKey(targetId)){
						 
						if(streetNodeMap.get(targetId).getPriority() > subPath.getClazz())
							streetNodeMap.get(targetId).setPriority( subPath.getClazz() );
						nodeTarget = streetNodeMap.get(targetId);
						subPath.setTargetPoint(nodeTarget);
					}
					else{
						Coordinate coordTarget = new Coordinate();
						coordTarget.x = rs.getDouble("x2");
						coordTarget.y = rs.getDouble("y2");
						nodeTarget = new StreetNode(targetId,geometryFactory.createPoint(coordTarget),false, subPath.getClazz() , true );
						streetNodeMap.put(targetId, nodeTarget);
						subPath.setTargetPoint(nodeTarget);
						adjacencyList.put(targetId, new ArrayList<StreetNode>());
						//adjacencyList.get(targetId).add(nodeTarget);

					}
				}
				


				/*Add nodes to Adjacency list*/
				if (!adjacencyList.get(sourceId).contains(subPath.getTargetPoint()))
					adjacencyList.get(sourceId).add(subPath.getTargetPoint());


				if (!adjacencyList.get(targetId).contains(subPath.getSourcePoint()))
					adjacencyList.get(targetId).add(subPath.getSourcePoint());


				streetNetWork.getEdges().put(subPath.getId(), subPath);

			}

		}

	}




	private int getNearstNode(double x1, double y1, boolean isSource) {
		int nodeId = 0;

		Connection con = null;
		PreparedStatement pstmt = null;
		ResultSet rs = null;
		con = createConection();

		try {

			String sqlQuery = "SELECT * FROM "+nodeTable+"" + 
					" ORDER BY the_geom <-> ST_GeometryFromText('POINT("+x1+" "+y1+")',4326)"+ 
					" LIMIT 1";
			
			System.out.println(sqlQuery);

			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();

			while (rs.next()){
				
					nodeId = rs.getInt("id");


			}


		} catch (Exception e) {
			System.out.println("Fail to list Nearst Node: " + e);
		}finally{
			closeConnection(con, pstmt, rs);
		}


		return nodeId;
	}



		public boolean getStreetNetWorkStubsOnly(Route route, double buffer, Map<Integer, StreetNode> streetNodeMap, Map<Integer, RoundAboutNode> rbNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList, StreetNetworkTopological streetNetwork  ){


			  /**
			Highway1 11
			Highway link 12
			Highway2(trunk) 13
			Highway2(trunk) link 14
			Primary 15
			Primary link 16
			Secondary 21
			Secondary link 22
			Tertiary 31
			Tertiary link 32
			Residential 41
			Unknow 42
			Not classified/Serivice/Rural  43
			Lowpriority(not street)  100
			*/


			Connection con = null;
			PreparedStatement pstmt = null;
			ResultSet rs = null;

			con = createConection();

		

			LineString routeLS = route.getRoutePath().asLineString(4);
			double extend = routeLS.getEnvelopeInternal().maxExtent();
			String sqlQuery = "";
			String routeSelection = null;
			System.out.println("extend: " + extend);
			try {

				/*	sqlQuery = "SELECT e.*, ST_AsGeoJSON(e.geom_way) AS geom \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"JOIN (SELECT ST_BUFFER(ST_LineMerge(ST_UNION(geom_way)),0.002) AS route_geom \r\n" +
						"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost FROM "+edgeTable+" ', "+sourceNodeId+", "+targetNodeId+", true) \r\n" +
						"JOIN "+edgeTable+" ON "+edgeTable+".id = edge) route\r\n" +
						"ON ST_Intersects(e.geom_way, route.route_geom)\r\n";*/

				/***Select Roads very close to the route***/
				
				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), 0.0005))\r\n " +
						//"AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 42 OR e.clazz = 43) \r\n";	
				        "AND (e.clazz = 11  OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 OR e.clazz = 41) \r\n";
				//"AND (e.clazz <> 11 AND e.clazz <> 13 AND e.clazz <> 15 AND e.clazz <> 21) \r\n";

				System.out.println(sqlQuery);

				pstmt = con.prepareStatement(sqlQuery);
				rs = pstmt.executeQuery();

				loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, true );
				rs.close();
				pstmt.close();
				






			} catch (Exception e) {
				System.out.println("Fail to list RoutPath" + e);
				return false;
			}finally{
				closeConnection(con, pstmt, rs);
			}



			return true;
		}


		public boolean getStreetNetWorkPreDefined(Route route, double d, Map<Integer, StreetNode> streetNodeMap,
				Map<Integer, RoundAboutNode> rbNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList,
				StreetNetworkTopological streetNetwork, boolean includeStubs) {
			

			  /**
			Highway1 11
			Highway link 12
			Highway2(trunk) 13
			Highway2(trunk) link 14
			Primary 15
			Primary link 16
			Secondary 21
			Secondary link 22
			Tertiary 31
			Tertiary link 32
			Residential 41
			Unknow 42
			Not classified/Serivice/Rural  43
			Lowpriority(not street)  100
			*/


			Connection con = null;
			PreparedStatement pstmt = null;
			ResultSet rs = null;

			con = createConection();

		
			System.out.println(route.getRoutePath().getNodeList());
			LineString routeLS = route.getRoutePath().asLineString(4);
			double extend = routeLS.getEnvelopeInternal().maxExtent();
			String sqlQuery = "";
			String routeSelection = null;
			System.out.println("extend: " + extend);
			try {

				/*	sqlQuery = "SELECT e.*, ST_AsGeoJSON(e.geom_way) AS geom \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"JOIN (SELECT ST_BUFFER(ST_LineMerge(ST_UNION(geom_way)),0.002) AS route_geom \r\n" +
						"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost FROM "+edgeTable+" ', "+sourceNodeId+", "+targetNodeId+", true) \r\n" +
						"JOIN "+edgeTable+" ON "+edgeTable+".id = edge) route\r\n" +
						"ON ST_Intersects(e.geom_way, route.route_geom)\r\n";*/
				
				/***Select Roads very close to the route STUBS***/
				
				
//					sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
//							"FROM "+edgeTable+" AS e \r\n" +
//							"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), 0.0000000009))\r\n " +
//							//"AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 42 OR e.clazz = 43) \r\n";	
//					        "AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) \r\n";
//					//"AND (e.clazz <> 11 AND e.clazz <> 13 AND e.clazz <> 15 AND e.clazz <> 21) \r\n";
//		
//					System.out.println(sqlQuery);
//		
//					pstmt = con.prepareStatement(sqlQuery);
//					rs = pstmt.executeQuery();
//		
//					loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, true );
//					rs.close();
//					pstmt.close();
				
				
				/***Select Roads close to the route***/

					routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 ) ";
				
					sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
							"FROM "+edgeTable+" AS e \r\n" +
							"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), 0.025))\r\n " +
							"AND "+routeSelection+" \r\n";	
				
				

				System.out.println(sqlQuery);

				pstmt = con.prepareStatement(sqlQuery);
				rs = pstmt.executeQuery();

				loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );
				rs.close();
				pstmt.close();
				
				
	
				


	//
				/*******Select Main Roads in MinimunBounding Circle *******/

//				if(extend > 0.10)
//					routeSelection =  "(e.clazz = 11 OR e.clazz = 13 )";
//				else
//					routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )";
	//
//				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
//						"FROM "+edgeTable+" AS e \r\n" +
//						"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_MinimumBoundingCircle(ST_GeomFromText('"+routeLS+"',4326)), "+extend/100+"))\r\n" +
//						"AND "+routeSelection+" \r\n";
	//
//				System.out.println(sqlQuery);
	//
//				pstmt = con.prepareStatement(sqlQuery);
//				rs = pstmt.executeQuery();
	//
//				loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );
//				rs.close();
//			   pstmt.close();

				
				/*******Select Main Roads in BoundindgBox *******/

//				if(extend > 0.10)
//					routeSelection =  "(e.clazz = 11 OR e.clazz = 13 )";
//				else
				routeSelection =  "(e.clazz = 11  OR e.clazz = 13 OR e.clazz = 15)";
	//
				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_MinimumBoundingCircle(ST_GeomFromText('"+routeLS+"',4326)), "+extend/1000+"))\r\n" +
						"AND "+routeSelection+" \r\n";
	
				System.out.println(sqlQuery);
	
				pstmt = con.prepareStatement(sqlQuery);
				rs = pstmt.executeQuery();
	
				loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );
				rs.close();
			   pstmt.close();

//				/*******Select Roads around Origin *******/		
				
				
				/*******Select Roads around Origin *******/

				
					routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43)";

				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getStart().getCoordinate()+"',4326), 0.002))\r\n " +						
						"OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getEnd().getCoordinate()+"',4326), 0.005)) ) \r\n " +
						"AND "+routeSelection+" \r\n";	

				
				System.out.println(sqlQuery);

				pstmt = con.prepareStatement(sqlQuery);
				rs = pstmt.executeQuery();

				loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );

				rs.close();
				pstmt.close();


			} catch (Exception e) {
				System.out.println("Fail to list RoutPath" + e);
				return false;
			}finally{
				closeConnection(con, pstmt, rs);
			}

			return true;
			
			
		}
		
		public boolean getStreetNetWorkPreDefined2(Route route, double d, Map<Integer, StreetNode> streetNodeMap,
				Map<Integer, RoundAboutNode> rbNodeMap, Map<Integer, ArrayList<StreetNode>> adjacencyList,
				StreetNetworkTopological streetNetwork, boolean includeStubs) {
			

			  /**
			Highway1 11
			Highway link 12
			Highway2(trunk) 13
			Highway2(trunk) link 14
			Primary 15
			Primary link 16
			Secondary 21
			Secondary link 22
			Tertiary 31
			Tertiary link 32
			Residential 41
			Unknow 42
			Not classified/Serivice/Rural  43
			Lowpriority(not street)  100
			*/

			
			ArrayList<String> queries = new ArrayList<String>();
		
			/***ROUTE 1 Ahlen 2.5 to Beckum 2.5***/
			
//			/**Major Streets*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.9014356 51.7582985, 7.9013973 51.7586567, 7.9021901 51.7587209, 7.9020684 51.7595251, 7.9020389 51.7597925, 7.9022087 51.7608913, 7.9039855 51.7610159, 7.9045226 51.7610784, 7.9051709 51.7611287, 7.907048 51.7613225, 7.9096113 51.761855, 7.9108649 51.7621297, 7.9115985 51.7622852, 7.9106908 51.7640451, 7.9105804 51.7645768, 7.9120181 51.7647436, 7.9129834 51.7648073, 7.9132079 51.7648225, 7.9144081 51.7649026, 7.917529 51.7652719, 7.9181451 51.7653877, 7.9190393 51.7656233, 7.9198518 51.7658719, 7.9202922 51.766013, 7.9206719 51.7661418, 7.9241607 51.7673152, 7.9253054 51.7677282, 7.9266413 51.7681796, 7.9289726 51.7686429, 7.9297355 51.768678, 7.9304559 51.7686668, 7.9311113 51.7686412, 7.938005 51.7676272, 7.9413715 51.766909, 7.9421813 51.7667149, 7.9437663 51.7663666, 7.95182 51.7656585, 7.9653383 51.7662464, 7.9685866 51.7666496, 7.9762877 51.7682887, 7.9789178 51.7685443, 7.979665698610883 51.7686761363438, 7.9888102 51.7622911, 7.9977334 51.7598313, 8.0011497 51.7601255, 8.001754559751475 51.76017743266081, 8.0023342 51.7602272, 8.0041192 51.7603851, 8.0096446 51.760394, 8.017678 51.758453, 8.0206933 51.7580647, 8.0216207 51.7580138, 8.0219031 51.7580228, 8.0246054 51.758336, 8.0259517 51.7582145, 8.0265729 51.7580638, 8.0294201 51.7570947, 8.0307993 51.7566065, 8.0321189 51.756074, 8.0336396 51.7554147, 8.0337623 51.7551582, 8.034817 51.7552928, 8.0354944 51.7560833, 8.0379216 51.7573278, 8.0382867 51.7574517, 8.0394169 51.7577856, 8.0396654 51.7578356, 8.0408829 51.7580759, 8.0415353 51.7580553, 8.043068 51.7578119, 8.0434301 51.757702, 8.042949 51.7571286, 8.0423002 51.7563317, 8.0422792 51.7563008, 8.0419805 51.7559434, 8.043115 51.7555773, 8.042559875841512 51.75487391448508)',4326), 0.027))\r\n" + 
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21  OR e.clazz = 31 )  ");
//				
//			/**Around origins*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.9014356 51.7582985)',4326), 0.003))\r\n" + 
//					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (8.042559875841512 51.75487391448508)',4326), 0.003)) ) \r\n" + 
//					" AND ( e.clazz = 41) ");
//			
//			
//			/**Around origins 2*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.9014356 51.7582985)',4326), 0.022))\r\n" + 
//					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (8.042559875841512 51.75487391448508)',4326), 0.0065)) ) \r\n" + 
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31) ");
			
			
			/***ROUTE 2 Outa Ahlen to Ahlen 2***/
			
//			/**Major Streets*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.911602 51.7909821, 7.9042328 51.7921036, 7.9035291 51.7930497, 7.9028039 51.7905586, 7.9023506 51.7889987, 7.9013052 51.7838984, 7.9010817 51.783275, 7.8989337 51.7794703, 7.8987466 51.7790204, 7.8984172 51.7782088, 7.8979552 51.7770818, 7.8975352 51.7760604, 7.896952 51.774648, 7.8968269 51.7743435, 7.896587 51.7737564, 7.8965646 51.7737016, 7.8963227 51.7731194, 7.896157 51.7727287, 7.8961154 51.7726221, 7.8960387 51.7724603, 7.8960197 51.7724138, 7.8955651 51.7713582, 7.8953854 51.770515, 7.8953285 51.7702372, 7.89516 51.7695408, 7.8951052 51.7693091, 7.8948785 51.7683085, 7.8948413 51.768142, 7.8938868 51.7682272, 7.8936297 51.7671374, 7.8946839 51.7670497, 7.894554 51.7664781, 7.8943042 51.7662287, 7.8941221 51.766088, 7.8948702 51.7640088, 7.8949066 51.7630408, 7.8947686 51.7627342, 7.8949397787013975 51.76167026134774, 7.8931805 51.7607533, 7.8926926 51.7605574, 7.892179486822644 51.760476763605176, 7.8917601 51.7606545, 7.8911238 51.7612488, 7.8903895 51.7612918, 7.8895793 51.7610693, 7.88884 51.7609824, 7.887654 51.7603363, 7.887392 51.760236, 7.8869951 51.760094, 7.8858652 51.7597374, 7.8849134 51.7594317, 7.8842919 51.7592446, 7.8840042 51.7591965, 7.8819316 51.7588534, 7.880269 51.7589243, 7.8802134 51.7584696, 7.8801974 51.7583315, 7.8801736 51.7581328, 7.8800072 51.7568044, 7.879614 51.7536595, 7.8795835 51.7534257, 7.8795023 51.7525701, 7.8801383 51.7496323, 7.880391 51.7491142, 7.8805076 51.7489021, 7.880615 51.7487016, 7.8806977 51.7485636, 7.8811536 51.7477333, 7.881183865592416 51.747671095842726, 7.8812133 51.7476106, 7.8814076 51.7471611, 7.881577 51.7465604, 7.8816821 51.745266, 7.8817372 51.7416796, 7.8817486 51.7415921, 7.880257 51.7415824, 7.87940915578302 51.7415417281943, 7.8793011 51.7412176, 7.8792905 51.7411307, 7.8791865 51.7407234, 7.8790144 51.7401121, 7.8789454 51.739892, 7.878040415785093 51.73988000474934)',4326), 0.020))\r\n" + 
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 )  ");
//			
//			
//			/**Major Streets 2*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.911602 51.7909821, 7.9042328 51.7921036, 7.9035291 51.7930497, 7.9028039 51.7905586, 7.9023506 51.7889987, 7.9013052 51.7838984, 7.9010817 51.783275, 7.8989337 51.7794703, 7.8987466 51.7790204, 7.8984172 51.7782088, 7.8979552 51.7770818, 7.8975352 51.7760604, 7.896952 51.774648, 7.8968269 51.7743435, 7.896587 51.7737564, 7.8965646 51.7737016, 7.8963227 51.7731194, 7.896157 51.7727287, 7.8961154 51.7726221, 7.8960387 51.7724603, 7.8960197 51.7724138, 7.8955651 51.7713582, 7.8953854 51.770515, 7.8953285 51.7702372, 7.89516 51.7695408, 7.8951052 51.7693091, 7.8948785 51.7683085, 7.8948413 51.768142, 7.8938868 51.7682272, 7.8936297 51.7671374, 7.8946839 51.7670497, 7.894554 51.7664781, 7.8943042 51.7662287, 7.8941221 51.766088, 7.8948702 51.7640088, 7.8949066 51.7630408, 7.8947686 51.7627342, 7.8949397787013975 51.76167026134774, 7.8931805 51.7607533, 7.8926926 51.7605574, 7.892179486822644 51.760476763605176, 7.8917601 51.7606545, 7.8911238 51.7612488, 7.8903895 51.7612918, 7.8895793 51.7610693, 7.88884 51.7609824, 7.887654 51.7603363, 7.887392 51.760236, 7.8869951 51.760094, 7.8858652 51.7597374, 7.8849134 51.7594317, 7.8842919 51.7592446, 7.8840042 51.7591965, 7.8819316 51.7588534, 7.880269 51.7589243, 7.8802134 51.7584696, 7.8801974 51.7583315, 7.8801736 51.7581328, 7.8800072 51.7568044, 7.879614 51.7536595, 7.8795835 51.7534257, 7.8795023 51.7525701, 7.8801383 51.7496323, 7.880391 51.7491142, 7.8805076 51.7489021, 7.880615 51.7487016, 7.8806977 51.7485636, 7.8811536 51.7477333, 7.881183865592416 51.747671095842726, 7.8812133 51.7476106, 7.8814076 51.7471611, 7.881577 51.7465604, 7.8816821 51.745266, 7.8817372 51.7416796, 7.8817486 51.7415921, 7.880257 51.7415824, 7.87940915578302 51.7415417281943, 7.8793011 51.7412176, 7.8792905 51.7411307, 7.8791865 51.7407234, 7.8790144 51.7401121, 7.8789454 51.739892, 7.878040415785093 51.73988000474934)',4326), 0.0055))\r\n" + 
//					" AND ( e.clazz = 43 )   ");
//			
//			
////			/**Around origins*/
////			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
////					"FROM mland_2po_4pgr3srb AS e \r\n" + 
////					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.911602 51.7909821)',4326), 0.007595670750094184))\r\n" + 
////					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.878040415785093 51.73988000474934)',4326), 0.007595670750094184)) ) \r\n" + 
////					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 OR e.clazz = 43) ");
////			
//			//7.8948413 51.768142
//			
//			/**Residential around some DP*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.893093 51.7683)',4326), 0.0015))\r\n" + 
//					//" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.878040415785093 51.73988000474934)',4326), 0.007595670750094184))"
//					 " ) \r\n" + 
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) ");		
//														
//			
//			/**Stubs*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.911602 51.7909821, 7.9042328 51.7921036, 7.9035291 51.7930497, 7.9028039 51.7905586, 7.9023506 51.7889987, 7.9013052 51.7838984, 7.9010817 51.783275, 7.8989337 51.7794703, 7.8987466 51.7790204, 7.8984172 51.7782088, 7.8979552 51.7770818, 7.8975352 51.7760604, 7.896952 51.774648, 7.8968269 51.7743435, 7.896587 51.7737564, 7.8965646 51.7737016, 7.8963227 51.7731194, 7.896157 51.7727287, 7.8961154 51.7726221, 7.8960387 51.7724603, 7.8960197 51.7724138, 7.8955651 51.7713582, 7.8953854 51.770515, 7.8953285 51.7702372, 7.89516 51.7695408, 7.8951052 51.7693091, 7.8948785 51.7683085, 7.8948413 51.768142, 7.8938868 51.7682272, 7.8936297 51.7671374, 7.8946839 51.7670497, 7.894554 51.7664781, 7.8943042 51.7662287, 7.8941221 51.766088, 7.8948702 51.7640088, 7.8949066 51.7630408, 7.8947686 51.7627342, 7.8949397787013975 51.76167026134774, 7.8931805 51.7607533, 7.8926926 51.7605574, 7.892179486822644 51.760476763605176, 7.8917601 51.7606545, 7.8911238 51.7612488, 7.8903895 51.7612918, 7.8895793 51.7610693, 7.88884 51.7609824, 7.887654 51.7603363, 7.887392 51.760236, 7.8869951 51.760094, 7.8858652 51.7597374, 7.8849134 51.7594317, 7.8842919 51.7592446, 7.8840042 51.7591965, 7.8819316 51.7588534, 7.880269 51.7589243, 7.8802134 51.7584696, 7.8801974 51.7583315, 7.8801736 51.7581328, 7.8800072 51.7568044, 7.879614 51.7536595, 7.8795835 51.7534257, 7.8795023 51.7525701, 7.8801383 51.7496323, 7.880391 51.7491142, 7.8805076 51.7489021, 7.880615 51.7487016, 7.8806977 51.7485636, 7.8811536 51.7477333, 7.881183865592416 51.747671095842726, 7.8812133 51.7476106, 7.8814076 51.7471611, 7.881577 51.7465604, 7.8816821 51.745266, 7.8817372 51.7416796, 7.8817486 51.7415921, 7.880257 51.7415824, 7.87940915578302 51.7415417281943, 7.8793011 51.7412176, 7.8792905 51.7411307, 7.8791865 51.7407234, 7.8790144 51.7401121, 7.8789454 51.739892, 7.878040415785093 51.73988000474934)',4326), 0.0000000009))\r\n" + 
//					" AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) ");
//		
			
			

			/***Route 3 Bahnhof to Geo1***/
			
//			/**Major Streets*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.6304796 51.9552406, 7.6285224 51.9559208, 7.6276395 51.9557967, 7.626462681067475 51.95581996121084, 7.6265523 51.9564757, 7.6266136 51.9569419, 7.6266211 51.956992, 7.6248528 51.9572708, 7.622685 51.9578183, 7.6213206 51.958292, 7.6195581 51.958701, 7.6194908 51.9587968, 7.619403 51.9588972, 7.6184961 51.9596266, 7.6175928 51.9619003, 7.6175867 51.9622961, 7.6178769 51.9634681, 7.6176023 51.9652499, 7.6173504 51.9658736, 7.6172375 51.9660877, 7.6170081 51.9663873, 7.6166988 51.9665757, 7.616647 51.9665973, 7.6160696 51.9668184, 7.6156853 51.9669839, 7.6150772 51.9673023, 7.6141558 51.9672417, 7.6137971 51.967226, 7.6122424 51.9672762, 7.6102351 51.9675858, 7.6101176 51.9677546, 7.6084744 51.9681641, 7.6083772 51.9681823, 7.6075881 51.9682714, 7.6075276 51.9682767, 7.6061603 51.9683576, 7.6047278 51.9685507, 7.6041139 51.968638, 7.603058350554436 51.968877150432455, 7.6028155885004765 51.96892678507723, 7.602708877299727 51.9689470436484, 7.602592558529045 51.968969126167174, 7.602216683252761 51.96904021221962, 7.602028560019637 51.96907715495743, 7.597530885176496 51.969813977531636, 7.597262348242379 51.9698398046183, 7.5966393175681315 51.9699027995222, 7.59662740125859 51.969854455134396, 7.596618820081265 51.96982165042027, 7.596540865454117 51.969519302196154, 7.596449496570837 51.968785383499615, 7.5944284 51.9690601)',4326), 0.027))\r\n" + 
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )  ");
//			
//			/**Major Streets*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.6304796 51.9552406, 7.6285224 51.9559208, 7.6276395 51.9557967, 7.626462681067475 51.95581996121084, 7.6265523 51.9564757, 7.6266136 51.9569419, 7.6266211 51.956992, 7.6248528 51.9572708, 7.622685 51.9578183, 7.6213206 51.958292, 7.6195581 51.958701, 7.6194908 51.9587968, 7.619403 51.9588972, 7.6184961 51.9596266, 7.6175928 51.9619003, 7.6175867 51.9622961, 7.6178769 51.9634681, 7.6176023 51.9652499, 7.6173504 51.9658736, 7.6172375 51.9660877, 7.6170081 51.9663873, 7.6166988 51.9665757, 7.616647 51.9665973, 7.6160696 51.9668184, 7.6156853 51.9669839, 7.6150772 51.9673023, 7.6141558 51.9672417, 7.6137971 51.967226, 7.6122424 51.9672762, 7.6102351 51.9675858, 7.6101176 51.9677546, 7.6084744 51.9681641, 7.6083772 51.9681823, 7.6075881 51.9682714, 7.6075276 51.9682767, 7.6061603 51.9683576, 7.6047278 51.9685507, 7.6041139 51.968638, 7.603058350554436 51.968877150432455, 7.6028155885004765 51.96892678507723, 7.602708877299727 51.9689470436484, 7.602592558529045 51.968969126167174, 7.602216683252761 51.96904021221962, 7.602028560019637 51.96907715495743, 7.597530885176496 51.969813977531636, 7.597262348242379 51.9698398046183, 7.5966393175681315 51.9699027995222, 7.59662740125859 51.969854455134396, 7.596618820081265 51.96982165042027, 7.596540865454117 51.969519302196154, 7.596449496570837 51.968785383499615, 7.5944284 51.9690601)',4326), 0.018))\r\n" + 
//					" AND ( e.clazz = 31)  ");
//			
//			/**Around origins*/
////			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
////					"FROM mland_2po_4pgr3srb AS e \r\n" + 
////					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT ( 7.6328255 51.9557183)',4326), 0.002))\r\n" + 
////					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.596449496570837 51.968785383499615)',4326), 0.0055)) ) \r\n" + 
////					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43)  ");
//			
//			
////			/**Around origins 2*/
////			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
////					"FROM mland_2po_4pgr3srb AS e \r\n" + 
////					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.9014356 51.7582985)',4326), 0.022))\r\n" + 
////					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (8.042559875841512 51.75487391448508)',4326), 0.0065)) ) \r\n" + 
////					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31) ");
//			
//			
////			/**Stubs*/
////			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
////					"FROM mland_2po_4pgr3srb AS e \r\n" + 
////					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.6304796 51.9552406, 7.6285224 51.9559208, 7.6276395 51.9557967, 7.626462681067475 51.95581996121084, 7.6265523 51.9564757, 7.6266136 51.9569419, 7.6266211 51.956992, 7.6248528 51.9572708, 7.622685 51.9578183, 7.6213206 51.958292, 7.6195581 51.958701, 7.6194908 51.9587968, 7.619403 51.9588972, 7.6184961 51.9596266, 7.6175928 51.9619003, 7.6175867 51.9622961, 7.6178769 51.9634681, 7.6176023 51.9652499, 7.6173504 51.9658736, 7.6172375 51.9660877, 7.6170081 51.9663873, 7.6166988 51.9665757, 7.616647 51.9665973, 7.6160696 51.9668184, 7.6156853 51.9669839, 7.6150772 51.9673023, 7.6141558 51.9672417, 7.6137971 51.967226, 7.6122424 51.9672762, 7.6102351 51.9675858, 7.6101176 51.9677546, 7.6084744 51.9681641, 7.6083772 51.9681823, 7.6075881 51.9682714, 7.6075276 51.9682767, 7.6061603 51.9683576, 7.6047278 51.9685507, 7.6041139 51.968638, 7.603058350554436 51.968877150432455, 7.6028155885004765 51.96892678507723, 7.602708877299727 51.9689470436484, 7.602592558529045 51.968969126167174, 7.602216683252761 51.96904021221962, 7.602028560019637 51.96907715495743, 7.597530885176496 51.969813977531636, 7.597262348242379 51.9698398046183, 7.5966393175681315 51.9699027995222, 7.59662740125859 51.969854455134396, 7.596618820081265 51.96982165042027, 7.596540865454117 51.969519302196154, 7.596449496570837 51.968785383499615, 7.5944284 51.9690601)',4326), 0.0000000009))\r\n" + 
////					" AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) ");
			
			
			
			
			/***Route 3 Ahlen2 to Leudingenhausen***/
			
			/**Major Streets*/
			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
					"FROM mland_2po_4pgr3srb AS e \r\n" + 
					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.8975214 51.7672006, 7.8975739 51.7664462, 7.896832902196226 51.766469547801464, 7.8968417 51.7659074, 7.8968657 51.7653688, 7.89742 51.7640657, 7.8974875 51.7632154, 7.8972228 51.7629551, 7.8956731 51.7617706, 7.8949397787013975 51.76167026134774, 7.8931805 51.7607533, 7.8926926 51.7605574, 7.892179486822644 51.760476763605176, 7.8917601 51.7606545, 7.8911238 51.7612488, 7.8903895 51.7612918, 7.8895793 51.7610693, 7.88884 51.7609824, 7.887654 51.7603363, 7.887392 51.760236, 7.8869951 51.760094, 7.8858652 51.7597374, 7.8849134 51.7594317, 7.8842919 51.7592446, 7.8840042 51.7591965, 7.8819316 51.7588534, 7.880269 51.7589243, 7.8805896 51.7614571, 7.8806555 51.761972, 7.880710955669097 51.762830314178736, 7.8786264 51.7645893, 7.8784537 51.7653791, 7.8779719 51.765957, 7.877172 51.7663449, 7.8761237 51.7668502, 7.8749166 51.7674949, 7.8744746 51.7677449, 7.8726432 51.7687191, 7.8706032 51.771307, 7.8704125 51.7714958, 7.8697821 51.7719507, 7.8660296 51.7726606, 7.8629261 51.7721815, 7.8626729 51.7721345, 7.8613859 51.7719331, 7.8518118 51.7727119, 7.8471301 51.7749397, 7.8470013 51.7750117, 7.846081 51.775522, 7.844702 51.7761796, 7.8439904 51.7764648, 7.839574 51.7776889, 7.8339008 51.7783397, 7.819388615719476 51.77839325468037, 7.803481 51.7835391, 7.7989088 51.785766, 7.7928827 51.7875745, 7.7867737 51.7889324, 7.7825778 51.7895901, 7.7766946 51.7898038, 7.7720913 51.7904176, 7.7633185 51.7931346, 7.7603938 51.7938416, 7.7598822 51.7939668, 7.7589686 51.7941169, 7.757522551982287 51.79406607731579, 7.757398237781499 51.794113297383966, 7.7565892 51.7949754, 7.7563224 51.7953124, 7.7558992 51.795834, 7.7535052 51.798402, 7.753347597463348 51.798552742842574, 7.7531353 51.7987558, 7.7498324 51.8010873, 7.749455 51.8012644, 7.7488108 51.8015463, 7.7483875 51.8017216, 7.7471879 51.802173, 7.7465688 51.8023657, 7.7460033 51.8025369, 7.7448129 51.8029155, 7.7443312 51.8030547, 7.7423969 51.8035276, 7.7396376 51.8040219, 7.7385454 51.8041467, 7.732138 51.8041562, 7.731901535590515 51.80413050935738, 7.73171 51.8041097, 7.7294042 51.8038641, 7.7286919 51.8037991, 7.7250012 51.8036369, 7.7241117 51.8036615, 7.7218613 51.8037798, 7.7215511 51.8038031, 7.7208407 51.8038501, 7.7196908 51.8039302, 7.7146647 51.8044798, 7.714354 51.8045156, 7.7098095 51.805074, 7.7074751 51.8052341, 7.7071958 51.805246, 7.705023 51.8052903, 7.6981482 51.8051542, 7.6886121 51.8053439, 7.6861773 51.8055844, 7.6854108 51.8056456, 7.6847312 51.8057036, 7.682369 51.8059462, 7.6803074 51.8060951, 7.6717112 51.8055064, 7.6660899 51.8043869, 7.6618697 51.803559, 7.6600804 51.8031868, 7.6590194 51.802963, 7.6514434 51.8010252, 7.6498379 51.8005015, 7.6482671 51.7999289, 7.6445155 51.7985146, 7.6431477 51.7980196, 7.642624836763316 51.79785666978184, 7.639551407258339 51.796951710827756, 7.637444030055882 51.796487327896145, 7.636684149222736 51.79635833642669, 7.636652285868522 51.796353446097484, 7.636086372384139 51.796271876703926, 7.634803278446335 51.79611952250431, 7.634052191495651 51.796043085219395, 7.633142940560769 51.795974601699484, 7.632284556122827 51.79591954204776, 7.6285096 51.7958546, 7.6280506 51.7958613, 7.627762643333938 51.79586661249059, 7.6274652 51.7958721, 7.6215845 51.7959383, 7.6212337 51.7959205, 7.618537 51.7955562, 7.6179056 51.79542, 7.6169121 51.7951469, 7.6163865 51.7949705, 7.6161893 51.7949093, 7.6158176 51.7947801, 7.6120472 51.7936382, 7.6018294 51.7923185, 7.599446 51.7925443, 7.5947864 51.7933971, 7.5810601 51.7952921, 7.5651317 51.7945051, 7.5578922 51.7922494, 7.546694 51.7875522, 7.5437097 51.7870075, 7.5416799 51.7867809, 7.539509 51.7866455, 7.5359608 51.7866045, 7.5107192 51.7837418, 7.4862012 51.7803336, 7.4743709 51.7750724, 7.4721407 51.7731345, 7.4706364 51.7734119, 7.4684736 51.7728487, 7.465298 51.7725158, 7.4633884 51.7723419, 7.462588 51.7722568, 7.4602079 51.7720163, 7.4604806 51.7708992)',4326), 0.022))\r\n" + 
					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )   ");
			
//			/**Major Streets*/
			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
					"FROM mland_2po_4pgr3srb AS e \r\n" + 
					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.8975214 51.7672006, 7.8975739 51.7664462, 7.896832902196226 51.766469547801464, 7.8968417 51.7659074, 7.8968657 51.7653688, 7.89742 51.7640657, 7.8974875 51.7632154, 7.8972228 51.7629551, 7.8956731 51.7617706, 7.8949397787013975 51.76167026134774, 7.8931805 51.7607533, 7.8926926 51.7605574, 7.892179486822644 51.760476763605176, 7.8917601 51.7606545, 7.8911238 51.7612488, 7.8903895 51.7612918, 7.8895793 51.7610693, 7.88884 51.7609824, 7.887654 51.7603363, 7.887392 51.760236, 7.8869951 51.760094, 7.8858652 51.7597374, 7.8849134 51.7594317, 7.8842919 51.7592446, 7.8840042 51.7591965, 7.8819316 51.7588534, 7.880269 51.7589243, 7.8805896 51.7614571, 7.8806555 51.761972, 7.880710955669097 51.762830314178736, 7.8786264 51.7645893, 7.8784537 51.7653791, 7.8779719 51.765957, 7.877172 51.7663449, 7.8761237 51.7668502, 7.8749166 51.7674949, 7.8744746 51.7677449, 7.8726432 51.7687191, 7.8706032 51.771307, 7.8704125 51.7714958, 7.8697821 51.7719507, 7.8660296 51.7726606, 7.8629261 51.7721815, 7.8626729 51.7721345, 7.8613859 51.7719331, 7.8518118 51.7727119, 7.8471301 51.7749397, 7.8470013 51.7750117, 7.846081 51.775522, 7.844702 51.7761796, 7.8439904 51.7764648, 7.839574 51.7776889, 7.8339008 51.7783397, 7.819388615719476 51.77839325468037, 7.803481 51.7835391, 7.7989088 51.785766, 7.7928827 51.7875745, 7.7867737 51.7889324, 7.7825778 51.7895901, 7.7766946 51.7898038, 7.7720913 51.7904176, 7.7633185 51.7931346, 7.7603938 51.7938416, 7.7598822 51.7939668, 7.7589686 51.7941169, 7.757522551982287 51.79406607731579, 7.757398237781499 51.794113297383966, 7.7565892 51.7949754, 7.7563224 51.7953124, 7.7558992 51.795834, 7.7535052 51.798402, 7.753347597463348 51.798552742842574, 7.7531353 51.7987558, 7.7498324 51.8010873, 7.749455 51.8012644, 7.7488108 51.8015463, 7.7483875 51.8017216, 7.7471879 51.802173, 7.7465688 51.8023657, 7.7460033 51.8025369, 7.7448129 51.8029155, 7.7443312 51.8030547, 7.7423969 51.8035276, 7.7396376 51.8040219, 7.7385454 51.8041467, 7.732138 51.8041562, 7.731901535590515 51.80413050935738, 7.73171 51.8041097, 7.7294042 51.8038641, 7.7286919 51.8037991, 7.7250012 51.8036369, 7.7241117 51.8036615, 7.7218613 51.8037798, 7.7215511 51.8038031, 7.7208407 51.8038501, 7.7196908 51.8039302, 7.7146647 51.8044798, 7.714354 51.8045156, 7.7098095 51.805074, 7.7074751 51.8052341, 7.7071958 51.805246, 7.705023 51.8052903, 7.6981482 51.8051542, 7.6886121 51.8053439, 7.6861773 51.8055844, 7.6854108 51.8056456, 7.6847312 51.8057036, 7.682369 51.8059462, 7.6803074 51.8060951, 7.6717112 51.8055064, 7.6660899 51.8043869, 7.6618697 51.803559, 7.6600804 51.8031868, 7.6590194 51.802963, 7.6514434 51.8010252, 7.6498379 51.8005015, 7.6482671 51.7999289, 7.6445155 51.7985146, 7.6431477 51.7980196, 7.642624836763316 51.79785666978184, 7.639551407258339 51.796951710827756, 7.637444030055882 51.796487327896145, 7.636684149222736 51.79635833642669, 7.636652285868522 51.796353446097484, 7.636086372384139 51.796271876703926, 7.634803278446335 51.79611952250431, 7.634052191495651 51.796043085219395, 7.633142940560769 51.795974601699484, 7.632284556122827 51.79591954204776, 7.6285096 51.7958546, 7.6280506 51.7958613, 7.627762643333938 51.79586661249059, 7.6274652 51.7958721, 7.6215845 51.7959383, 7.6212337 51.7959205, 7.618537 51.7955562, 7.6179056 51.79542, 7.6169121 51.7951469, 7.6163865 51.7949705, 7.6161893 51.7949093, 7.6158176 51.7947801, 7.6120472 51.7936382, 7.6018294 51.7923185, 7.599446 51.7925443, 7.5947864 51.7933971, 7.5810601 51.7952921, 7.5651317 51.7945051, 7.5578922 51.7922494, 7.546694 51.7875522, 7.5437097 51.7870075, 7.5416799 51.7867809, 7.539509 51.7866455, 7.5359608 51.7866045, 7.5107192 51.7837418, 7.4862012 51.7803336, 7.4743709 51.7750724, 7.4721407 51.7731345, 7.4706364 51.7734119, 7.4684736 51.7728487, 7.465298 51.7725158, 7.4633884 51.7723419, 7.462588 51.7722568, 7.4602079 51.7720163, 7.4604806 51.7708992)',4326), 0.012))\r\n" + 
					" AND (e.clazz = 31 )   ");
			
			/**Around origins*/
			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
					"FROM mland_2po_4pgr3srb AS e \r\n" + 
					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.8975214 51.7672006)',4326), 0.002	))\r\n" + 
					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.460796 51.7699831)',4326), 0.0028)) ) \r\n" + 
					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) ");
			
			
			
			/**Around origins*/
			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
					"FROM mland_2po_4pgr3srb AS e \r\n" + 
					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.8975214 51.7672006)',4326), 0.017))\r\n" + 
					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.4594445 51.769718)',4326), 0.012)) ) \r\n" + 
					" AND ( e.clazz = 31 ) ");

			
			/**Stubs*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.8975214 51.7672006, 7.8975739 51.7664462, 7.896832902196226 51.766469547801464, 7.8968417 51.7659074, 7.8968657 51.7653688, 7.89742 51.7640657, 7.8974875 51.7632154, 7.8972228 51.7629551, 7.8956731 51.7617706, 7.8949397787013975 51.76167026134774, 7.8931805 51.7607533, 7.8926926 51.7605574, 7.892179486822644 51.760476763605176, 7.8917601 51.7606545, 7.8911238 51.7612488, 7.8903895 51.7612918, 7.8895793 51.7610693, 7.88884 51.7609824, 7.887654 51.7603363, 7.887392 51.760236, 7.8869951 51.760094, 7.8858652 51.7597374, 7.8849134 51.7594317, 7.8842919 51.7592446, 7.8840042 51.7591965, 7.8819316 51.7588534, 7.880269 51.7589243, 7.8805896 51.7614571, 7.8806555 51.761972, 7.880710955669097 51.762830314178736, 7.8786264 51.7645893, 7.8784537 51.7653791, 7.8779719 51.765957, 7.877172 51.7663449, 7.8761237 51.7668502, 7.8749166 51.7674949, 7.8744746 51.7677449, 7.8726432 51.7687191, 7.8706032 51.771307, 7.8704125 51.7714958, 7.8697821 51.7719507, 7.8660296 51.7726606, 7.8629261 51.7721815, 7.8626729 51.7721345, 7.8613859 51.7719331, 7.8518118 51.7727119, 7.8471301 51.7749397, 7.8470013 51.7750117, 7.846081 51.775522, 7.844702 51.7761796, 7.8439904 51.7764648, 7.839574 51.7776889, 7.8339008 51.7783397, 7.819388615719476 51.77839325468037, 7.803481 51.7835391, 7.7989088 51.785766, 7.7928827 51.7875745, 7.7867737 51.7889324, 7.7825778 51.7895901, 7.7766946 51.7898038, 7.7720913 51.7904176, 7.7633185 51.7931346, 7.7603938 51.7938416, 7.7598822 51.7939668, 7.7589686 51.7941169, 7.757522551982287 51.79406607731579, 7.757398237781499 51.794113297383966, 7.7565892 51.7949754, 7.7563224 51.7953124, 7.7558992 51.795834, 7.7535052 51.798402, 7.753347597463348 51.798552742842574, 7.7531353 51.7987558, 7.7498324 51.8010873, 7.749455 51.8012644, 7.7488108 51.8015463, 7.7483875 51.8017216, 7.7471879 51.802173, 7.7465688 51.8023657, 7.7460033 51.8025369, 7.7448129 51.8029155, 7.7443312 51.8030547, 7.7423969 51.8035276, 7.7396376 51.8040219, 7.7385454 51.8041467, 7.732138 51.8041562, 7.731901535590515 51.80413050935738, 7.73171 51.8041097, 7.7294042 51.8038641, 7.7286919 51.8037991, 7.7250012 51.8036369, 7.7241117 51.8036615, 7.7218613 51.8037798, 7.7215511 51.8038031, 7.7208407 51.8038501, 7.7196908 51.8039302, 7.7146647 51.8044798, 7.714354 51.8045156, 7.7098095 51.805074, 7.7074751 51.8052341, 7.7071958 51.805246, 7.705023 51.8052903, 7.6981482 51.8051542, 7.6886121 51.8053439, 7.6861773 51.8055844, 7.6854108 51.8056456, 7.6847312 51.8057036, 7.682369 51.8059462, 7.6803074 51.8060951, 7.6717112 51.8055064, 7.6660899 51.8043869, 7.6618697 51.803559, 7.6600804 51.8031868, 7.6590194 51.802963, 7.6514434 51.8010252, 7.6498379 51.8005015, 7.6482671 51.7999289, 7.6445155 51.7985146, 7.6431477 51.7980196, 7.642624836763316 51.79785666978184, 7.639551407258339 51.796951710827756, 7.637444030055882 51.796487327896145, 7.636684149222736 51.79635833642669, 7.636652285868522 51.796353446097484, 7.636086372384139 51.796271876703926, 7.634803278446335 51.79611952250431, 7.634052191495651 51.796043085219395, 7.633142940560769 51.795974601699484, 7.632284556122827 51.79591954204776, 7.6285096 51.7958546, 7.6280506 51.7958613, 7.627762643333938 51.79586661249059, 7.6274652 51.7958721, 7.6215845 51.7959383, 7.6212337 51.7959205, 7.618537 51.7955562, 7.6179056 51.79542, 7.6169121 51.7951469, 7.6163865 51.7949705, 7.6161893 51.7949093, 7.6158176 51.7947801, 7.6120472 51.7936382, 7.6018294 51.7923185, 7.599446 51.7925443, 7.5947864 51.7933971, 7.5810601 51.7952921, 7.5651317 51.7945051, 7.5578922 51.7922494, 7.546694 51.7875522, 7.5437097 51.7870075, 7.5416799 51.7867809, 7.539509 51.7866455, 7.5359608 51.7866045, 7.5107192 51.7837418, 7.4862012 51.7803336, 7.4743709 51.7750724, 7.4721407 51.7731345, 7.4706364 51.7734119, 7.4684736 51.7728487, 7.465298 51.7725158, 7.4633884 51.7723419, 7.462588 51.7722568, 7.4602079 51.7720163, 7.4604806 51.7708992)',4326), 0.0000000009))\r\n" + 
//					" AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) ");
			
			
			
			
			/***Route 3 Muenster to Leudingenhausen***/
			
//			/**Major Streets*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.6239771 51.9680345, 7.6229841 51.9679161, 7.6233502 51.9668186, 7.6236614 51.9663472, 7.6237187 51.9662925, 7.6237368 51.9659441, 7.6232776 51.966133, 7.6230846 51.9661975, 7.6202086 51.9666831, 7.6179853 51.9665983, 7.6178842 51.9665865, 7.6175783 51.9665359, 7.6170081 51.9663873, 7.6172375 51.9660877, 7.6173504 51.9658736, 7.6176023 51.9652499, 7.6178769 51.9634681, 7.6175867 51.9622961, 7.6175928 51.9619003, 7.6184961 51.9596266, 7.619403 51.9588972, 7.6194908 51.9587968, 7.6195581 51.958701, 7.6197766 51.957631, 7.6196718 51.9573417, 7.6193865 51.9565142, 7.6194104 51.9561918, 7.6195794 51.9558018, 7.6197204 51.9554536, 7.6198318 51.95512, 7.6198437 51.9549772, 7.619856325046852 51.95468890079192, 7.619796116458885 51.95392662931681, 7.619776958028891 51.953847093430475, 7.619692922344601 51.953590669310216, 7.619119378093791 51.952651397720416, 7.619051820014261 51.952563059504655, 7.6178456327607345 51.951064904309774, 7.616952067760621 51.949942072450455, 7.616888991237225 51.94987361014158, 7.616565099134549 51.94949884741608, 7.614281349689054 51.94676944993304, 7.611995784570723 51.943975322778286, 7.609368707817535 51.940810732580786, 7.607670770001325 51.939044073451576, 7.605626900213225 51.93782796938331, 7.604254376623104 51.93699226081671, 7.60360270815554 51.93652540135513, 7.602705697717013 51.935842112390944, 7.60261419395956 51.935768476731226, 7.602088938538454 51.935324751866204, 7.602065809571792 51.93530469051646, 7.601933756485331 51.93518808816516, 7.6019246629100525 51.93517995278232, 7.601752023445378 51.93502707083298, 7.601715446025839 51.934992960727655, 7.601433782851107 51.93473152387356, 7.601250429856183 51.93455494315048, 7.5988230973881095 51.9316743105905, 7.597640010796699 51.92993316448388, 7.597172390132532 51.92916961989348, 7.596296288209425 51.92763749166338, 7.570985661289419 51.91578824155866, 7.569344520351417 51.91574646547042, 7.5688111589143405 51.915734401350775, 7.560754587579307 51.91545506796437, 7.560429502974787 51.91543764913106, 7.560237847443203 51.91542730885466, 7.560085941265419 51.91541885408143, 7.5582828808897835 51.915309115890075, 7.557376382211006 51.91525102288408, 7.552649241977363 51.9148638310389, 7.5497987932336095 51.91457942061035, 7.540569797024833 51.91337566043593, 7.529188591446412 51.91112314850887, 7.508952961334629 51.90499601599612, 7.495876641090609 51.90191391912159, 7.492218512047133 51.90124751286379, 7.492127193773733 51.90123194900587, 7.4921423 51.9010473, 7.492195794128601 51.90021863598911, 7.492238 51.8997163, 7.4923779 51.8984762, 7.492461 51.8979727, 7.492823 51.8964813, 7.4947592 51.890627, 7.4959856 51.8874712, 7.4973563 51.8838654, 7.4976452 51.8823741, 7.4959668 51.8783032, 7.492883 51.8734344, 7.4927451 51.8694342, 7.4927306 51.8677245, 7.4927306 51.8675556, 7.492720760388603 51.8635897376383, 7.4925717 51.8602343, 7.4930447 51.8584027, 7.4929014 51.8569979, 7.4917356 51.8533435, 7.4913887 51.8525178, 7.4911194 51.8518451, 7.4909376 51.8514222, 7.4907935 51.8511161, 7.4905106 51.8507056, 7.4892231 51.8495665, 7.4889698 51.8492832, 7.4886998 51.8488338, 7.4884912 51.8470493, 7.4884379 51.8459643, 7.489586 51.8380129, 7.4910139 51.8311499, 7.4912241 51.8302448, 7.4919261 51.8279166, 7.4931532 51.8198652, 7.4931666 51.8193345, 7.4928913 51.8179769, 7.4916675 51.8126854, 7.4910264 51.810654, 7.489061 51.8083098, 7.4870129 51.8068535, 7.4839544 51.8050401, 7.4635107 51.788918, 7.4615542 51.7877455, 7.4592925 51.7863382, 7.451619 51.7829889, 7.4497 51.7821099, 7.4491519 51.7816878, 7.4486477 51.7811875, 7.4484084 51.7808371, 7.4483582 51.7777595, 7.4476322 51.775602, 7.447609 51.7752039, 7.4479891 51.7744774, 7.4497676128055 51.773180150427336, 7.4508284 51.7721255, 7.4513004 51.771549, 7.4513789 51.7714293, 7.4544498 51.7717307, 7.456265 51.771765, 7.4566758 51.7717575, 7.4571851 51.7717481, 7.457661201270908 51.771770967421645, 7.4582968 51.7706334, 7.4589995 51.770756, 7.4594445 51.769718)',4326), 0.025))\r\n" + 
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )   ");
//			
//			
//			/**Around origins*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.624503 51.968063)',4326), 0.0018))\r\n" + 
//					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.460796 51.7699831)',4326), 0.0028)) ) \r\n" + 
//					" AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) ");
//			
//			
//			
//			/**Around origins*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.6264109 51.9690991)',4326), 0.008))\r\n" + 
//					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.4594445 51.769718)',4326), 0.012)) ) \r\n" + 
//					" AND (e.clazz = 31 ) ");
			
			
			
			/***Route 5 Warendorf- Beckum 3***/
			
//			/**Major Streets*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_MinimumBoundingCircle(ST_GeomFromText('LINESTRING (7.7873084 51.9782515, 7.7891353 51.9781302, 7.7888905 51.9771534, 7.7875174 51.9772055, 7.786476 51.976992, 7.7883445 51.9745776, 7.789086966044599 51.973341037127305, 7.7894495 51.9711984, 7.789558990728939 51.97004618750708, 7.7897635 51.9676972, 7.7897638 51.9676331, 7.7898997 51.9654503, 7.7902431 51.9611457, 7.790292 51.9420461, 7.7912274 51.9409555, 7.7928035 51.9394711, 7.7970749 51.9385623, 7.8036451 51.9297242, 7.8039595 51.9279634, 7.8087853 51.9282206, 7.8155802 51.9288451, 7.815796681277088 51.928865653646056, 7.8160015 51.9288851, 7.8278821 51.9290866, 7.8307932 51.9291361, 7.8309818 51.929141, 7.8315788 51.9291768, 7.833084004859872 51.9292773877162, 7.8342515 51.9293677, 7.844776084046062 51.93092210610068, 7.8448555 51.9307417, 7.8450453 51.9304003, 7.845279 51.9298513, 7.8454619 51.9291157, 7.8464233 51.9271838, 7.8478898 51.9272189, 7.8491567 51.9271211, 7.8482409 51.9254551, 7.848586 51.9246349, 7.8490252 51.9237909, 7.8489671 51.9227524, 7.8489257 51.9226921, 7.8481892 51.921441, 7.8515933 51.9203724, 7.8532321 51.9198483, 7.8533464 51.9198244, 7.8535973 51.9197083, 7.8542288 51.919169, 7.8554382 51.9177066, 7.8556269 51.917455, 7.8600821 51.912599, 7.8649953 51.9113395, 7.8686693 51.9104479, 7.8703203 51.9100094, 7.8720721 51.9095114, 7.8759725 51.9079091, 7.877606 51.9062352, 7.8829798 51.899833, 7.8861042 51.8961257, 7.8864858 51.8956832, 7.8883154 51.8935007, 7.8899099 51.8900605, 7.8901459 51.8892891, 7.8911233 51.8860615, 7.8996456 51.8776527, 7.9009442 51.8767307, 7.9010758 51.8766366, 7.9030557 51.8752844, 7.9035873 51.8750251, 7.9060921 51.8744976, 7.9062294 51.8744623, 7.9071752 51.8737588, 7.907519 51.8726759, 7.9087856 51.8723068, 7.9086683 51.8719131, 7.9084087 51.8713847, 7.9087574 51.8705739, 7.9087908 51.8704197, 7.908761 51.8699753, 7.9086844 51.869604, 7.9083941 51.8685969, 7.9082078 51.867687, 7.9081817 51.8675964, 7.9081361 51.865278, 7.9083074 51.8650301, 7.9111848 51.861245, 7.9123857 51.8606254, 7.9288007 51.8544756, 7.9294683 51.8517443, 7.9300584 51.8463239, 7.9265117 51.83855, 7.9257668 51.836428, 7.9250667 51.8329508, 7.9248779 51.8318865, 7.924868 51.8318271, 7.9247707 51.8312244, 7.924740099171212 51.831030694198475, 7.9252366 51.8309159, 7.9259992 51.8307412, 7.9356846 51.8300766, 7.9374382 51.8299745, 7.9384947 51.8298957, 7.9386021 51.829888, 7.9389618 51.8298623, 7.9437423 51.8295541, 7.9459428 51.8297649, 7.9462208 51.828456, 7.9497451 51.8262135, 7.950254 51.8250635, 7.9505301 51.8245765, 7.9506746 51.8244505, 7.9513903 51.8238943, 7.9516621 51.8234952, 7.9527124 51.8211596, 7.9530905 51.8200716, 7.9531702 51.8200784, 7.956618 51.8201627, 7.9726651 51.8137286, 7.9843451 51.8113583, 7.9882609 51.8105225, 7.9916152 51.8098487, 7.996991 51.8087833, 8.000212 51.8081211, 8.0069459 51.8072864, 8.0079903 51.8072355, 8.008458459434756 51.80722815111431, 8.0088178 51.8018335, 8.0091705 51.8007097, 8.0092428 51.8004957, 8.0098694 51.799075, 8.0104017 51.7982367, 8.0114064 51.7969765, 8.011573107886672 51.79683529068249, 8.0117856 51.7966553, 8.0123723 51.7961982, 8.0129602 51.7958324, 8.022719533035149 51.79301591570051, 8.0289785 51.7862284, 8.0299371 51.7858604, 8.0303465 51.7856516, 8.0310938 51.785142, 8.0314113 51.7848598, 8.0320518 51.7839349, 8.0324785 51.7810857, 8.0325409 51.780582, 8.03265071233384 51.780164480759, 8.032870040096455 51.77937083521365, 8.0329267 51.7791685, 8.0330444 51.7787314, 8.033074 51.7786218, 8.033221192965657 51.778212237254344, 8.0335306 51.7771368, 8.0336569 51.7767457, 8.0341083 51.7756326, 8.0348826 51.7742828, 8.0354434 51.7736538, 8.0355109 51.7735795, 8.0368124 51.7725058, 8.042306 51.7653357, 8.0425623 51.7649738, 8.042782 51.7646317, 8.043182 51.7635386, 8.042415739702186 51.76009371608758, 8.0422968 51.7599026, 8.0421394 51.7597212, 8.04204 51.7595364, 8.0419351 51.7592795, 8.0418749 51.7590934, 8.041837 51.7589916, 8.0416817 51.7585187, 8.0415353 51.7580553, 8.043068 51.7578119, 8.0434301 51.757702, 8.0446581 51.7572234, 8.0440603 51.7566115, 8.0447757 51.7553337, 8.0445711 51.7546673, 8.0437078 51.7547599, 8.0431217 51.7544928, 8.0410718 51.7540204, 8.0408379 51.7540372, 8.0405532 51.7540577, 8.0404549 51.7536157, 8.0404278 51.7535335, 8.0404102 51.7534683, 8.0402974 51.7529792)',4326)), 2.5E-4))\r\n" + 
//					"AND (e.clazz = 11  OR e.clazz = 13 OR e.clazz = 15)  ");
////			
////			
//			/**Major Streets*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.7873084 51.9782515, 7.7891353 51.9781302, 7.7888905 51.9771534, 7.7875174 51.9772055, 7.786476 51.976992, 7.7883445 51.9745776, 7.789086966044599 51.973341037127305, 7.7894495 51.9711984, 7.789558990728939 51.97004618750708, 7.7897635 51.9676972, 7.7897638 51.9676331, 7.7898997 51.9654503, 7.7902431 51.9611457, 7.790292 51.9420461, 7.7912274 51.9409555, 7.7928035 51.9394711, 7.7970749 51.9385623, 7.8036451 51.9297242, 7.8039595 51.9279634, 7.8087853 51.9282206, 7.8155802 51.9288451, 7.815796681277088 51.928865653646056, 7.8160015 51.9288851, 7.8278821 51.9290866, 7.8307932 51.9291361, 7.8309818 51.929141, 7.8315788 51.9291768, 7.833084004859872 51.9292773877162, 7.8342515 51.9293677, 7.844776084046062 51.93092210610068, 7.8448555 51.9307417, 7.8450453 51.9304003, 7.845279 51.9298513, 7.8454619 51.9291157, 7.8464233 51.9271838, 7.8478898 51.9272189, 7.8491567 51.9271211, 7.8482409 51.9254551, 7.848586 51.9246349, 7.8490252 51.9237909, 7.8489671 51.9227524, 7.8489257 51.9226921, 7.8481892 51.921441, 7.8515933 51.9203724, 7.8532321 51.9198483, 7.8533464 51.9198244, 7.8535973 51.9197083, 7.8542288 51.919169, 7.8554382 51.9177066, 7.8556269 51.917455, 7.8600821 51.912599, 7.8649953 51.9113395, 7.8686693 51.9104479, 7.8703203 51.9100094, 7.8720721 51.9095114, 7.8759725 51.9079091, 7.877606 51.9062352, 7.8829798 51.899833, 7.8861042 51.8961257, 7.8864858 51.8956832, 7.8883154 51.8935007, 7.8899099 51.8900605, 7.8901459 51.8892891, 7.8911233 51.8860615, 7.8996456 51.8776527, 7.9009442 51.8767307, 7.9010758 51.8766366, 7.9030557 51.8752844, 7.9035873 51.8750251, 7.9060921 51.8744976, 7.9062294 51.8744623, 7.9071752 51.8737588, 7.907519 51.8726759, 7.9087856 51.8723068, 7.9086683 51.8719131, 7.9084087 51.8713847, 7.9087574 51.8705739, 7.9087908 51.8704197, 7.908761 51.8699753, 7.9086844 51.869604, 7.9083941 51.8685969, 7.9082078 51.867687, 7.9081817 51.8675964, 7.9081361 51.865278, 7.9083074 51.8650301, 7.9111848 51.861245, 7.9123857 51.8606254, 7.9288007 51.8544756, 7.9294683 51.8517443, 7.9300584 51.8463239, 7.9265117 51.83855, 7.9257668 51.836428, 7.9250667 51.8329508, 7.9248779 51.8318865, 7.924868 51.8318271, 7.9247707 51.8312244, 7.924740099171212 51.831030694198475, 7.9252366 51.8309159, 7.9259992 51.8307412, 7.9356846 51.8300766, 7.9374382 51.8299745, 7.9384947 51.8298957, 7.9386021 51.829888, 7.9389618 51.8298623, 7.9437423 51.8295541, 7.9459428 51.8297649, 7.9462208 51.828456, 7.9497451 51.8262135, 7.950254 51.8250635, 7.9505301 51.8245765, 7.9506746 51.8244505, 7.9513903 51.8238943, 7.9516621 51.8234952, 7.9527124 51.8211596, 7.9530905 51.8200716, 7.9531702 51.8200784, 7.956618 51.8201627, 7.9726651 51.8137286, 7.9843451 51.8113583, 7.9882609 51.8105225, 7.9916152 51.8098487, 7.996991 51.8087833, 8.000212 51.8081211, 8.0069459 51.8072864, 8.0079903 51.8072355, 8.008458459434756 51.80722815111431, 8.0088178 51.8018335, 8.0091705 51.8007097, 8.0092428 51.8004957, 8.0098694 51.799075, 8.0104017 51.7982367, 8.0114064 51.7969765, 8.011573107886672 51.79683529068249, 8.0117856 51.7966553, 8.0123723 51.7961982, 8.0129602 51.7958324, 8.022719533035149 51.79301591570051, 8.0289785 51.7862284, 8.0299371 51.7858604, 8.0303465 51.7856516, 8.0310938 51.785142, 8.0314113 51.7848598, 8.0320518 51.7839349, 8.0324785 51.7810857, 8.0325409 51.780582, 8.03265071233384 51.780164480759, 8.032870040096455 51.77937083521365, 8.0329267 51.7791685, 8.0330444 51.7787314, 8.033074 51.7786218, 8.033221192965657 51.778212237254344, 8.0335306 51.7771368, 8.0336569 51.7767457, 8.0341083 51.7756326, 8.0348826 51.7742828, 8.0354434 51.7736538, 8.0355109 51.7735795, 8.0368124 51.7725058, 8.042306 51.7653357, 8.0425623 51.7649738, 8.042782 51.7646317, 8.043182 51.7635386, 8.042415739702186 51.76009371608758, 8.0422968 51.7599026, 8.0421394 51.7597212, 8.04204 51.7595364, 8.0419351 51.7592795, 8.0418749 51.7590934, 8.041837 51.7589916, 8.0416817 51.7585187, 8.0415353 51.7580553, 8.043068 51.7578119, 8.0434301 51.757702, 8.0446581 51.7572234, 8.0440603 51.7566115, 8.0447757 51.7553337, 8.0445711 51.7546673, 8.0437078 51.7547599, 8.0431217 51.7544928, 8.0410718 51.7540204, 8.0408379 51.7540372, 8.0405532 51.7540577, 8.0404549 51.7536157, 8.0404278 51.7535335, 8.0404102 51.7534683, 8.0402974 51.7529792)',4326), 0.025))\r\n" + 
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 )   ");
//			
//			
//			/**Select Route*/
//			queries.add("SELECT * , ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom,  ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(geom_way) AS geomjson \r\n" + 
//					"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost  \r\n" + 
//					"FROM mland_2po_4pgr3srb', 16506, 14676, false)\r\n" + 
//					"JOIN mland_2po_4pgr3srb ON mland_2po_4pgr3srb.id = edge \r\n" + 
//					"ORDER BY seq  ");
//			
//			
////	
////			
//			/**Around origins 1*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.7873084 51.9782515)',4326), 0.038))\r\n" + 
//					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (8.013014686099806 51.75706318461605 )',4326), 0.045)) ) \r\n" + 
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 ) ");			
//			
//			/**Around origins*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.7873084 51.9782515)',4326), 0.008))\r\n" + 
//					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.84832982619 51.92563337)',4326), 0.009))  \r\n" +
//					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (8.0402974 51.7529792)',4326), 0.007)) ) \r\n" + 
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 ) ");
//			
//			
//			/**Around origins*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.78935725075845 51.97706492825796)',4326), 0.0018))\r\n" + 
//					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (8.041801102356974 51.75597648203497)',4326), 0.0015)))  \r\n" + 
//					
//					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) ");
			
			
			
		

			Connection con = null;
			PreparedStatement pstmt = null;
			ResultSet rs = null;

			con = createConection();

			String sqlQuery = "";

		
			try {

				/*	sqlQuery = "SELECT e.*, ST_AsGeoJSON(e.geom_way) AS geom \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"JOIN (SELECT ST_BUFFER(ST_LineMerge(ST_UNION(geom_way)),0.002) AS route_geom \r\n" +
						"FROM pgr_dijkstra(' SELECT id AS id, source AS source, target AS target, cost AS cost , reverse_cost::float8 AS reverse_cost FROM "+edgeTable+" ', "+sourceNodeId+", "+targetNodeId+", true) \r\n" +
						"JOIN "+edgeTable+" ON "+edgeTable+".id = edge) route\r\n" +
						"ON ST_Intersects(e.geom_way, route.route_geom)\r\n";*/
				
				
				for(String query: queries) {
					
					sqlQuery = query;
					
					System.out.println(sqlQuery);
					
					pstmt = con.prepareStatement(sqlQuery);
					rs = pstmt.executeQuery();
		
					loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, true );
					rs.close();
					pstmt.close();
					
					
					
				}
				



			} catch (Exception e) {
				System.out.println("Fail to list RoutPath" + e);
				return false;
			}finally{
				closeConnection(con, pstmt, rs);
			}

			return true;
			
			
		}
		
		

		

}
