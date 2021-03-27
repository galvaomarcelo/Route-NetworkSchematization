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
					routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 ) ";
				
					sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
							"FROM "+edgeTable+" AS e \r\n" +
							"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), 0.020))\r\n " +
							"AND "+routeSelection+" \r\n";	
				
				

				System.out.println(sqlQuery);

				pstmt = con.prepareStatement(sqlQuery);
				rs = pstmt.executeQuery();

				loadRoadsToDataStructure(rs, streetNodeMap, rbNodeMap, streetNetwork, adjacencyList, false );
				rs.close();
				pstmt.close();
				
				
				
				/***Select Roads close to the route***/
				
					routeSelection =  "(e.clazz = 311 OR e.clazz = 43 ) ";
				
					sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
							"FROM "+edgeTable+" AS e \r\n" +
							"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), 0.0045))\r\n " +
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

//				/*******Select Roads around Origin *******/

				if(extend > 0.10)
					routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 OR e.clazz = 41)";
				else
					routeSelection =  "(e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31 OR e.clazz = 41)";

				sqlQuery = "SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" +
						"FROM "+edgeTable+" AS e \r\n" +
						"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getStart().getCoordinate()+"',4326), 0.003))\r\n " +
						"OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('"+route.getEnd().getCoordinate()+"',4326), 0.003)) ) \r\n " +
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
			
			/**Major Streets*/
			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
					"FROM mland_2po_4pgr3srb AS e \r\n" + 
					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.9014356 51.7582985, 7.9013973 51.7586567, 7.9021901 51.7587209, 7.9020684 51.7595251, 7.9020389 51.7597925, 7.9022087 51.7608913, 7.9039855 51.7610159, 7.9045226 51.7610784, 7.9051709 51.7611287, 7.907048 51.7613225, 7.9096113 51.761855, 7.9108649 51.7621297, 7.9115985 51.7622852, 7.9106908 51.7640451, 7.9105804 51.7645768, 7.9120181 51.7647436, 7.9129834 51.7648073, 7.9132079 51.7648225, 7.9144081 51.7649026, 7.917529 51.7652719, 7.9181451 51.7653877, 7.9190393 51.7656233, 7.9198518 51.7658719, 7.9202922 51.766013, 7.9206719 51.7661418, 7.9241607 51.7673152, 7.9253054 51.7677282, 7.9266413 51.7681796, 7.9289726 51.7686429, 7.9297355 51.768678, 7.9304559 51.7686668, 7.9311113 51.7686412, 7.938005 51.7676272, 7.9413715 51.766909, 7.9421813 51.7667149, 7.9437663 51.7663666, 7.95182 51.7656585, 7.9653383 51.7662464, 7.9685866 51.7666496, 7.9762877 51.7682887, 7.9789178 51.7685443, 7.979665698610883 51.7686761363438, 7.9888102 51.7622911, 7.9977334 51.7598313, 8.0011497 51.7601255, 8.001754559751475 51.76017743266081, 8.0023342 51.7602272, 8.0041192 51.7603851, 8.0096446 51.760394, 8.017678 51.758453, 8.0206933 51.7580647, 8.0216207 51.7580138, 8.0219031 51.7580228, 8.0246054 51.758336, 8.0259517 51.7582145, 8.0265729 51.7580638, 8.0294201 51.7570947, 8.0307993 51.7566065, 8.0321189 51.756074, 8.0336396 51.7554147, 8.0337623 51.7551582, 8.034817 51.7552928, 8.0354944 51.7560833, 8.0379216 51.7573278, 8.0382867 51.7574517, 8.0394169 51.7577856, 8.0396654 51.7578356, 8.0408829 51.7580759, 8.0415353 51.7580553, 8.043068 51.7578119, 8.0434301 51.757702, 8.042949 51.7571286, 8.0423002 51.7563317, 8.0422792 51.7563008, 8.0419805 51.7559434, 8.043115 51.7555773, 8.042559875841512 51.75487391448508)',4326), 0.027))\r\n" + 
					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21  OR e.clazz = 31 )  ");
			
			/**Around origins*/
			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
					"FROM mland_2po_4pgr3srb AS e \r\n" + 
					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.9014356 51.7582985)',4326), 0.003))\r\n" + 
					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (8.042559875841512 51.75487391448508)',4326), 0.003)) ) \r\n" + 
					" AND ( e.clazz = 41) ");
			
			
			/**Around origins 2*/
			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
					"FROM mland_2po_4pgr3srb AS e \r\n" + 
					"WHERE (ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (7.9014356 51.7582985)',4326), 0.022))\r\n" + 
					" OR ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('POINT (8.042559875841512 51.75487391448508)',4326), 0.0065)) ) \r\n" + 
					" AND (e.clazz = 11 OR e.clazz = 13 OR e.clazz = 15 OR e.clazz = 21 OR e.clazz = 31) ");
			
			
			/**Stubs*/
//			queries.add("SELECT e.*, ST_AsBinary(rbsourcegeom) as rbsgeom, ST_AsBinary(rbtargetgeom) as rbtgeom, ST_AsBinary(geom_way) AS geom, ST_AsGeoJSON(e.geom_way) AS geomjson \r\n" + 
//					"FROM mland_2po_4pgr3srb AS e \r\n" + 
//					"WHERE ST_Intersects(e.geom_way, ST_BUFFER(ST_GeomFromText('LINESTRING (7.9014356 51.7582985, 7.9013973 51.7586567, 7.9021901 51.7587209, 7.9020684 51.7595251, 7.9020389 51.7597925, 7.9022087 51.7608913, 7.9039855 51.7610159, 7.9045226 51.7610784, 7.9051709 51.7611287, 7.907048 51.7613225, 7.9096113 51.761855, 7.9108649 51.7621297, 7.9115985 51.7622852, 7.9106908 51.7640451, 7.9105804 51.7645768, 7.9120181 51.7647436, 7.9129834 51.7648073, 7.9132079 51.7648225, 7.9144081 51.7649026, 7.917529 51.7652719, 7.9181451 51.7653877, 7.9190393 51.7656233, 7.9198518 51.7658719, 7.9202922 51.766013, 7.9206719 51.7661418, 7.9241607 51.7673152, 7.9253054 51.7677282, 7.9266413 51.7681796, 7.9289726 51.7686429, 7.9297355 51.768678, 7.9304559 51.7686668, 7.9311113 51.7686412, 7.938005 51.7676272, 7.9413715 51.766909, 7.9421813 51.7667149, 7.9437663 51.7663666, 7.95182 51.7656585, 7.9653383 51.7662464, 7.9685866 51.7666496, 7.9762877 51.7682887, 7.9789178 51.7685443, 7.979665698610883 51.7686761363438, 7.9888102 51.7622911, 7.9977334 51.7598313, 8.0011497 51.7601255, 8.001754559751475 51.76017743266081, 8.0023342 51.7602272, 8.0041192 51.7603851, 8.0096446 51.760394, 8.017678 51.758453, 8.0206933 51.7580647, 8.0216207 51.7580138, 8.0219031 51.7580228, 8.0246054 51.758336, 8.0259517 51.7582145, 8.0265729 51.7580638, 8.0294201 51.7570947, 8.0307993 51.7566065, 8.0321189 51.756074, 8.0336396 51.7554147, 8.0337623 51.7551582, 8.034817 51.7552928, 8.0354944 51.7560833, 8.0379216 51.7573278, 8.0382867 51.7574517, 8.0394169 51.7577856, 8.0396654 51.7578356, 8.0408829 51.7580759, 8.0415353 51.7580553, 8.043068 51.7578119, 8.0434301 51.757702, 8.042949 51.7571286, 8.0423002 51.7563317, 8.0422792 51.7563008, 8.0419805 51.7559434, 8.043115 51.7555773, 8.042559875841512 51.75487391448508)',4326), 0.0000000009))\r\n" + 
//					" AND (e.clazz = 31 OR e.clazz = 41 OR e.clazz = 43) ");
			

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
