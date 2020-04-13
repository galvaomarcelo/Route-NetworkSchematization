package com.wayto.operator;

import java.awt.geom.Point2D;

public class FisheyeTransform {
	/* https://www.fernuni-hagen.de/imperia/md/content/fakultaetfuermathematikundinformatik/forschung/berichte/bericht_271.pdf */
	
	
	private Point2D focus;
	private double distortionParameter;
	private double intensity; /* normalized distortion parameter [0..1]*/
	
	
	public FisheyeTransform(Point2D focus, double intensity, Point2D topLeft, Point2D bottomRight) {
		super();
		
		this.intensity = intensity;
		this.topLeft = topLeft;
		this.bottomRight = bottomRight;
		
		if( this.intensity >= 0 && this.intensity < 1)
			this.distortionParameter = ( this.intensity )/ (1 - this.intensity);
		else {
			System.out.println("Intensity of the distorison must be >= 0 and < 1");
			this.distortionParameter = 0;
		}
		
		if ( boundsContains(focus) )
			this.focus = focus;
		else{
			System.out.println("Focus must be in bounds of Point2D topLeft and Point2D bottomRight, otherwise focus will be center of the bounds");
			this.focus = new Point2D.Double( (bottomRight.getX() - topLeft.getX())/2, (bottomRight.getY() - topLeft.getY())/2 );
		}
	}


	private Point2D topLeft;
	private Point2D bottomRight;
	
	
	public Point2D transform(Point2D pt){
		/* https://www.fernuni-hagen.de/imperia/md/content/fakultaetfuermathematikundinformatik/forschung/berichte/bericht_271.pdf  image 2(a)*/
		if (boundsContains(pt)) {
			Point2D ptFinal = new Point2D.Double();
			/* ix: x of bounds, nx: normalized ptx - focalx, pnx: projecttion of nx in the distortion function */ 
			double ix, nx, pnx;
			double iy, ny, pny;
			
			
			/* find new x */
			if( pt.getX() > this.focus.getX() )
				ix = this.bottomRight.getX();
			else ix = this.topLeft.getX();
				
				
			nx = (pt.getX() - this.focus.getX())/(ix - this.focus.getX());
			pnx = h2(nx);
			
			double newX = pnx*(ix - this.focus.getX()) + this.focus.getX();
			
			/* find new y */
			if( pt.getY() > this.focus.getY() )
				iy = this.bottomRight.getY();
			else iy = this.topLeft.getY();
			
			ny = (pt.getY() - this.focus.getY())/(iy - this.focus.getY());
			pny = h2(ny);
			
			double newY = pny*(iy - this.focus.getY()) + this.focus.getY();
			
			
			
			ptFinal.setLocation(newX, newY);
			
			
			return ptFinal;		
		}
		else return pt;
		
	}


	private double h(double n) {
		
		double value =  (this.distortionParameter + 1)*n/(this.distortionParameter*n + 1);
		
		System.out.println("n = "  + n + " value = " + value);
		return value;
	}
	
	private double h2(double n) {
		double value =  4*Math.pow(n, 3) - 6*Math.pow(n, 2) + 3*n;
		System.out.println("n = "  + n + " value = " + value);
		return value;
	}
	
	/* check if focus is in the bounds */
	private boolean boundsContains(Point2D pt) {

		if( pt.getX() < this.bottomRight.getX() && pt.getX() > this.topLeft.getX() 
				&& pt.getY() < this.bottomRight.getY() && pt.getY() > topLeft.getY()  )
			return true;
		else return false;
		
	}
	

}
