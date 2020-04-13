package com.wayto.operator;

import java.awt.geom.Point2D;
import java.util.ArrayList;




public class Gene implements Comparable< Gene >{

	/*Cromossoma Cartesian*/
	private ArrayList<Point2D> schematicPoints;
	private double value;
	
	
	
	public Gene(ArrayList<Point2D> schematicPoints) {
		super();
		this.schematicPoints = schematicPoints;
		this.value = 0;
	}
	
	public Gene() {
		super();
		this.schematicPoints = new ArrayList<Point2D>();
		this.value = 0;
	}
	
	public int compareTo(Gene anotherGene) {
		
		return Double.valueOf( this.value ).compareTo( anotherGene.getValue() );
	}
	
	public ArrayList<Point2D> getSchematicPoints() {
		return schematicPoints;
	}
	
	public void setSchematicPoints(ArrayList<Point2D> schematicPoints) {
		this.schematicPoints = schematicPoints;
	}
	
	public double getValue() {
		return value;
	}
	
	public void setValue(double value) {
		this.value = value;
	}
	
}
