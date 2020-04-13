package com.wayto.operator;

import java.awt.geom.Point2D;
import java.util.ArrayList;
 
/*Cromossoma*/
public class PointsPolar {

	private Point2D origem;
	private ArrayList< RelativPolarPoint > points;
	
	public PointsPolar(Point2D  origem) {
		super();
		this.origem = origem;
		points = new ArrayList<RelativPolarPoint>();
	}

	
	public PointsPolar() {
		super();
		points = new ArrayList<RelativPolarPoint>();
	}


	public Point2D getOrigem() {
		return origem;
	}

	public void setOrigem(Point2D origem) {
		this.origem = origem;
	}


	public ArrayList<RelativPolarPoint> getPoints() {
		return points;
	}


	public void add(RelativPolarPoint ralativPolarPoint) {
		points.add( ralativPolarPoint );
		
	}
}
