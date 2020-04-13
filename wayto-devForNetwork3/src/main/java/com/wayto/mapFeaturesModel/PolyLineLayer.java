package com.wayto.mapFeaturesModel;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.locationtech.jts.geom.Envelope;
import com.wayto.model.StreetNode;

public class PolyLineLayer implements Layer {
	
	private String name;
	private double layerDepth = 100;
	private float stroke =4;
	private Color color = Color.BLACK;
	private Color fillColor = Color.WHITE;
	private boolean isClosed = false;
	private boolean isVisible = true;
	private ArrayList<ArrayList<Point2D>> lines;
	private int type = -1;

	
	
	
	public PolyLineLayer(String name) {
		this.name = name;
		this.lines = new ArrayList<ArrayList<Point2D>>();
		// TODO Auto-generated constructor stub
	}
	
	
	public PolyLineLayer(String name, double layerDepth, float stroke, Color color, Color fillColor, boolean isClosed,
			boolean isVisible) {
		super();
		this.name = name;
		this.layerDepth = layerDepth;
		this.stroke = stroke;
		this.color = color;
		this.fillColor = fillColor;
		this.isClosed = isClosed;
		this.isVisible = isVisible;
		this.lines = new ArrayList<ArrayList<Point2D>>();
	}


	public ArrayList<ArrayList<Point2D>> getLines() {
		return lines;
	}

	public void setLines(ArrayList<ArrayList<Point2D>> lines) {
		this.lines = lines;
	}

	public float getStroke() {
		return stroke;
	}

	public void setStroke(float stroke) {
		this.stroke = stroke;
	}

	public Color getColor() {
		return color;
	}

	public void setColor(Color color) {
		this.color = color;
	}

	public boolean isClosed() {
		return isClosed;
	}

	public void setClosed(boolean inClosed) {
		this.isClosed = inClosed;
	}

	public Color getFillColor() {
		return fillColor;
	}

	public void setFillColor(Color fillColor) {
		this.fillColor = fillColor;
	}

	public int getType() {
		// TODO Auto-generated method stub
		return this.type;
	}

	public void setType(int type) {
		this.type  = type;
		
	}
	public String getName() {
		// TODO Auto-generated method stub
		return this.name;
	}
	
	

	public void setName(String name) {
		this.name = name;
	}


	public boolean isVisible() {
		return isVisible;
	}

	public void setVisible(boolean isVisible) {
		this.isVisible = isVisible;
	} 

	public double getLayerDepth() {
		return layerDepth;
	}

	public void setLayerDepth(int layerDepth) {
		this.layerDepth = layerDepth;
	}

	public static Layer toLayer(ArrayList<StreetNode> path, String name, int number, Envelope env) {
		PolyLineLayer layer = new PolyLineLayer(name + " " + number);
		ArrayList<Point2D> polyline = new ArrayList<Point2D>();
		for (StreetNode point: path){
			
			polyline.add( new Point2D.Double(point.getGeom().getX() - env.getMinX(),
					env.getMaxY() - point.getGeom().getY() ));
			
			
			
			
		}
		layer.getLines().add(polyline);
		
		return layer;

	}
	
	public static ArrayList<Point2D> pathToLayerLine(ArrayList<StreetNode> path,  Envelope env) {
		
		ArrayList<Point2D> polyline = new ArrayList<Point2D>();
		for (StreetNode point: path){
			
			polyline.add( new Point2D.Double(point.getGeom().getX() - env.getMinX(),
					env.getMaxY() - point.getGeom().getY() ));
						
		}
		
		
		return polyline;

	}
	
	public static ArrayList<Point2D> xPathToLayerLine(ArrayList<StreetNode> path) {
		
		ArrayList<Point2D> polyline = new ArrayList<Point2D>();
		for (StreetNode point: path){
			
			polyline.add( new Point2D.Double(point.getxGeom().getX() ,
					 point.getxGeom().getY() ));
						
		}
		
		
		return polyline;

	}
	
	
	public static Layer toXLayer(ArrayList<StreetNode> path, String name, int number, Envelope env) {
		PolyLineLayer layer = new PolyLineLayer(name + " " + number);
		ArrayList<Point2D> polyline = new ArrayList<Point2D>();
		for (StreetNode point: path){
			
			try {
				polyline.add( new Point2D.Double(point.getxGeom().getX() - env.getMinX(),
						env.getMaxY() - point.getxGeom().getY() ));
			} catch (Exception e) {
				// TODO: handle exception
			}
				
			
			
			
			
		}
		layer.getLines().add(polyline);
		
		return layer;

	}







	
	
}
