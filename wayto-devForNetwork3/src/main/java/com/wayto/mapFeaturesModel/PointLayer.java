package com.wayto.mapFeaturesModel;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class PointLayer implements Layer{


	private String name;
	private double layerDepth = 100;
	private float stroke =2;
	private float size = 4;
	private Color color = Color.BLACK;
	private Color fillColor = Color.WHITE;
	private boolean isVisible = true;
	


	private ArrayList<Point2D> points;
	private int type = -1;

	

	
	public PointLayer(String name) {
		this.name = name;
		this.points = new ArrayList<Point2D>();
	}
	
	
	
	public PointLayer(String name, double layerDepth, float stroke, float size, Color color, Color fillColor,
			boolean isVisible) {
		super();
		this.name = name;
		this.layerDepth = layerDepth;
		this.stroke = stroke;
		this.size = size;
		this.color = color;
		this.fillColor = fillColor;
		this.isVisible = isVisible;
		this.points = new ArrayList<Point2D>();
	}



	public ArrayList<Point2D> getPoints() {
		return points;
	}
	public void setPoints(ArrayList<Point2D> points) {
		this.points = points;
	}
	public float getStroke() {
		return stroke;
	}
	public void setStroke(float stroke) {
		this.stroke = stroke;
	}
	public float getSize() {
		return size;
	}
	public void setSize(float size) {
		this.size = size;
	}
	public Color getColor() {
		return color;
	}
	public void setColor(Color color) {
		this.color = color;
	}
	public Color getFillColor() {
		return fillColor;
	}
	public void setFillColor(Color fillColor) {
		this.fillColor = fillColor;
	}

	public int getType() {
		return this.type;
		
	}
	public void setType(int type) {
		this.type   = type;
		
	}

	public String getName() {
		return this.name;
		
	}

	public boolean isVisible() {
		return isVisible;
	}

	public void setVisible(boolean isVisible) {
		this.isVisible = isVisible;
	}



	public void setName(String name) {
		this.name = name;
	}

	public double getLayerDepth() {
		return layerDepth;
	}

	public void setLayerDepth(double layerDepth) {
		this.layerDepth = layerDepth;
	}
	
	
	

}
