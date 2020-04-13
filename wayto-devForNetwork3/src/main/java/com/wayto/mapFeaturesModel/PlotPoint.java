package com.wayto.mapFeaturesModel;

import java.awt.Color;
import java.awt.geom.Point2D;

public class PlotPoint {
	private Point2D position;
	private float stroke =2;
	private float size = 4;
	private Color color = Color.BLACK; 
	private Color fillColor = Color.WHITE;
	private float transparency = 1.0f;
	
	
	
	public PlotPoint(Point2D position, Color fillColor) {
		super();
		this.position = position;
		this.fillColor = fillColor;
	}
	public Point2D getPosition() {
		return position;
	}
	public void setPosition(Point2D position) {
		this.position = position;
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
	public float getTransparency() {
		return transparency;
	}
	public void setTransparency(float transparency) {
		this.transparency = transparency;
	}
	
	

}
