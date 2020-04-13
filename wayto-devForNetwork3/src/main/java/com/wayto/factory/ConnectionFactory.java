package com.wayto.factory;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;





/**
 * Create and close database
 *  * @author Marcelo
 *
 */
public class ConnectionFactory {
	
	private static final String DRIVER = "org.postgresql.Driver";
	private static final String URL = "jdbc:postgresql://localhost:5432/wayto2";
	private static final String USER = "postgres";
	private static final String PASS = "postgres";

	
	public Connection createConection(){
		
		Connection connection = null;
		
		try {
			Class.forName(DRIVER);
			connection = DriverManager.getConnection(URL, USER, PASS);
			
		} catch (Exception e) {
			System.out.println("Fail to create connection" + URL);
			e.printStackTrace();
		}
		
		return connection;
		
	}
	
	public void closeConnection (Connection con, PreparedStatement pstmt, ResultSet rset ){
		
	
		try {
			if(con != null ){
				con.close();
			}
			if(pstmt != null){
				pstmt.close();
			}
			if(rset != null){
				rset.close();
			}
		} catch (SQLException e) {
			System.out.println("Faio to close database:" + URL);
			e.printStackTrace();
		}
	
	
	}
}
