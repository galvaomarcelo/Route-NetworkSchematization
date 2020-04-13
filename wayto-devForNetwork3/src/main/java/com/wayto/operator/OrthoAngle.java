package com.wayto.operator;

import java.util.Comparator;



public class OrthoAngle {
	
	@Override
	public String toString() {
		return "OrthoAngle [vertice=" + vertice + ", orthogonalAngle=" + orthogonalAngle + "]";
	}


	private int vertice;
	private double orthogonalAngle;
	
	
	public OrthoAngle(int vertice, double orthogonalAngle) {
		super();
		this.vertice = vertice;
		this.orthogonalAngle = orthogonalAngle;
	}


	public int getVertice() {
		return vertice;
	}


	public void setVertice(int vertice) {
		this.vertice = vertice;
	}


	public double getOrthogonalAngle() {
		return orthogonalAngle;
	}


	public void setOrthogonalAngle(double orthogonalAngle) {
		this.orthogonalAngle = orthogonalAngle;
	}
	
	
	public static Comparator<OrthoAngle> AngleComparator = new Comparator<OrthoAngle>() {

		public int compare(OrthoAngle s1, OrthoAngle s2) {
			Double angle1 = s1.getOrthogonalAngle();
			Double angle2 = s2.getOrthogonalAngle();

			//ascending order
			return angle1.compareTo(angle2);

			//descending order
			//return StudentName2.compareTo(StudentName1);
		}

	};
	



}
