package com.wayto.mapFeaturesModel;

import java.awt.Color;
import java.util.ArrayList;

public class ClassyPointLayer implements Layer {

	private int type = 2;
	private int layerDepth = 100;
	private String name;
	private ArrayList<PlotPoint> points;
	private float stroke =2;
	private float size = 4;
	private Color color = Color.BLACK; 
	private Color fillColor = Color.WHITE;
	private boolean isVisible = true;
	
	public ClassyPointLayer(String name) {
		this.name = name;
		this.points = new ArrayList<PlotPoint>();
	}
	
	public ArrayList<PlotPoint> getPoints() {
		return points;
	}
	public void setPoints(ArrayList<PlotPoint> points) {
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

	public String getName() {
		return this.name;
		
	}

	public boolean isVisible() {
		return isVisible;
	}

	public void setVisible(boolean isVisible) {
		this.isVisible = isVisible;
	}

	public void setType(int type) {
		this.type = type;
	}

	public void setName(String name) {
		this.name = name;
	}

	public double getLayerDepth() {
		return layerDepth;
	}

	public void setLayerDepth(int layerDepth) {
		this.layerDepth = layerDepth;
	}

	

}
