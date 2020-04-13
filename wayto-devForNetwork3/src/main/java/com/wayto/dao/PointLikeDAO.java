package com.wayto.dao;

import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.util.ArrayList;

import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.io.WKBReader;
import com.wayto.factory.ConnectionFactory;
import com.wayto.model.PointFeature;
import com.wayto.model.PolygonalFeature;
import com.wayto.model.Route;
 
public class PointLikeDAO extends ConnectionFactory{

private static PointLikeDAO instance;
	
	public static PointLikeDAO getInstance(){
		if(instance == null)
			instance = new PointLikeDAO();
		return instance;
	}
	
	
	public ArrayList<PointFeature> getPointLMList(Route route, double buffer){
		ArrayList<PointFeature> pointFeatureList = null;
		
		WKBReader wkbReader = new WKBReader();
		
		Connection con = null;
		PreparedStatement pstmt = null;
		ResultSet rs = null;
		
		con = createConection();
		
		try {
			
			LineString routeLS = route.getRoutePath().asLineString(4);
			String sqlQuery = "SELECT f.id, f.type, f.name, f.icon, f.salience, f.street_level, f.color, ST_AsBinary(f.geom) AS geom, ST_AsGeoJSON(f.geom, 1) AS geomjson \r\n" +
					"FROM nrw_my_points AS f \r\n" +
					"WHERE   ST_Intersects(f.geom, ST_BUFFER(ST_GeomFromText('"+routeLS+"',4326), "+buffer+"))\r\n";
			
			
			
			System.out.println(sqlQuery);
			
			pstmt = con.prepareStatement(sqlQuery);
			rs = pstmt.executeQuery();
			
			
			pointFeatureList = new ArrayList<PointFeature>();
			
			PointFeature feature = null;

			while (rs.next()){
				feature = new PointFeature();
				feature.setId(rs.getInt("id"));
				feature.setName(rs.getString("name"));
				feature.setType(rs.getString("type"));
				feature.setSalience(rs.getInt("salience"));
				feature.setStreetLevel(rs.getInt("street_level"));
				feature.setIcon(rs.getString("icon"));
				feature.setGeomAsJSON(rs.getString("geomjson"));
				feature.setGeom((Point)wkbReader.read(rs.getBytes("geom")));				
				
				pointFeatureList.add(feature);
			
			}
			
			
		} catch (Exception e) {
			System.out.println("Fail to list Point Landmarks" + e);
		}finally{
			closeConnection(con, pstmt, rs);
		}
		
		
		
		return pointFeatureList;
		
	}
	
}
