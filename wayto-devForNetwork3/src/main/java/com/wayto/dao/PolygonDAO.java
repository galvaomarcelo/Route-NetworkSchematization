package com.wayto.dao;

import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.util.ArrayList;

import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.io.WKBReader;
import com.wayto.factory.ConnectionFactory;
import com.wayto.model.PolygonalFeature;
import com.wayto.model.Route;
 
public class PolygonDAO extends ConnectionFactory{

private static PolygonDAO instance;
	
	public static PolygonDAO getInstance(){
		if(instance == null)
			instance = new PolygonDAO();
		return instance;
	}
	
	
	public ArrayList<PolygonalFeature> getBoundaryList(Route route, double buffer){
		ArrayList<PolygonalFeature> polygonalFeatureList = null;
		
		WKBReader wkbReader = new WKBReader();
		
		Connection con = null;
		PreparedStatement pstmt = null;
		ResultSet rs = null;
		
		con = createConection();
		
		try {
			LineString routeLS = route.getRoutePath().asLineString(4);
			
			String sqlQuery = "SELECT f.id, f.type, f.name, f.boundary, f.salience, f.level_abstraction, ST_AsBinary(ST_ExteriorRing(ST_GeometryN(f.geom, 1))) AS geom, ST_AsGeoJSON(ST_ExteriorRing(ST_GeometryN(f.geom, 1))) AS geomjson \r\n" +
					"FROM nrw_my_polygons AS f \r\n" +
					"WHERE   ST_Intersects(f.geom, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), "+buffer+"))\r\n";
			
			
			
			System.out.println(sqlQuery);
			
			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();
			
			
			polygonalFeatureList = new ArrayList<PolygonalFeature>();
			
			PolygonalFeature feature = null;

			while (rs.next()){
				feature = new PolygonalFeature();
				feature.setId(rs.getInt("id"));
				feature.setName(rs.getString("name"));
				feature.setType(rs.getString("type"));
				feature.setLevelAbstraction(rs.getInt("level_abstraction"));
				feature.setSalience(rs.getInt("salience"));
				feature.setGeomAsJSON(rs.getString("geomjson"));
				feature.setGeocoordsGeom((LineString)wkbReader.read(rs.getBytes("geom")));				
				
				polygonalFeatureList.add(feature);
			
			}
			
			rs.close();
			pstmt.close();
			
			
			sqlQuery = "SELECT f.id, f.type, f.name, f.boundary, f.salience, f.level_abstraction, ST_AsBinary(ST_ExteriorRing(ST_GeometryN(f.geom, 1))) AS geom, ST_AsGeoJSON(ST_ExteriorRing(ST_GeometryN(f.geom, 1))) AS geomjson \r\n" +
					"FROM nrw_my_polygons AS f \r\n" +
					"WHERE  f.salience > 2 AND ST_Intersects(f.geom, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), "+10*buffer+"))\r\n";
			
			
			
			System.out.println(sqlQuery);
			
			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();
			
			
			//polygonalFeatureList = new ArrayList<PolygonalFeature>();
			
			feature = null;

			boolean alreadyExists = false;
			while (rs.next()){
				
				alreadyExists = false;
				for(PolygonalFeature p: polygonalFeatureList) {
					if(p.getId() == rs.getInt("id")) {
						alreadyExists = true;
						break;
					}
						
				}
				if(!alreadyExists) {
					feature = new PolygonalFeature();
					feature.setId(rs.getInt("id"));
					feature.setName(rs.getString("name"));
					feature.setType(rs.getString("type"));
					feature.setGeomAsJSON(rs.getString("geomjson"));
					feature.setLevelAbstraction(rs.getInt("level_abstraction"));
					feature.setSalience(rs.getInt("salience"));
					feature.setGeocoordsGeom((LineString)wkbReader.read(rs.getBytes("geom")));				
					
					polygonalFeatureList.add(feature);
				}
			}
			
			rs.close();
			pstmt.close();
			
			
		} catch (Exception e) {
			System.out.println("Fail to list RoutPath" + e);
		}finally{
			closeConnection(con, pstmt, rs);
		}
		
		
		
		return polygonalFeatureList;
		
	}
	
}
