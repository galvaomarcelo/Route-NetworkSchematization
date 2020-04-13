package com.wayto.operator;



public class RelativPolarPoint {
/*Gene*/
	private double r;
	private double theta; /*angle in radians*/
	
	public RelativPolarPoint(double r, double theta) {
		super();
		this.r = r;
		this.theta = theta;
	}
	
	public RelativPolarPoint() {
		// TODO Auto-generated constructor stub
	}

	public double getR() {
		return r;
	}
	public void setR(double r) {
		this.r = r;
	}
	public double getTheta() {
		return theta;
	}
	public void setTheta(double theta) {
		this.theta = theta;
	}
}
