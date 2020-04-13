package com.wayto.model.topo;


import org.locationtech.jts.geom.Point;
import com.wayto.model.StreetNode;
import com.wayto.operator.GeoConvertionsOperations;
import com.wayto.operator.GeometricOperation;
import com.wayto.operator.RelativPolarPoint;

public class ControlPoint {
	
	private StreetNode node;
	private Point originalPosition;
	private RelativPolarPoint relativeToNodePosition;
	
	/***if controlPoint is relative it means that the original position of this point in the polygon is not coincident in the note route
	 * it means it is required to store a relative position to the node  */
	private boolean isRelative = false;
	private boolean isBufferCrossing = false;
	
	public ControlPoint(StreetNode node, Point originalPosition) {
		super();
		this.node = node;
		this.originalPosition = originalPosition;
		relativeToNodePosition = new RelativPolarPoint(0, 0);
		this.isRelative =  false;
	}
	
	public ControlPoint(StreetNode node, Point originalPosition, boolean isRelative) {
		super();
		this.node = node;
		this.originalPosition = originalPosition;
		this.isRelative = isRelative;
		if(isRelative){
//			relativeToNodePosition = GeometricOperation.toPolar(GeoConvertionsOperations.PointJTSGeometryToJava2D(node.getGeom()), 
//				GeoConvertionsOperations.PointJTSGeometryToJava2D(originalPosition));
			
			double r = node.getGeom().distance( originalPosition );
			double theta = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(node.getGeom().getX(), node.getGeom().getY(),
					originalPosition.getX(), originalPosition.getY());
			relativeToNodePosition = new RelativPolarPoint( 1*r , theta );
			
		}	
		else 
			relativeToNodePosition = new RelativPolarPoint(0, 0);
	}

	public StreetNode getNode() {
		return node;
	}

	public void setNode(StreetNode node) {
		this.node = node;
	}

	public Point getOriginalPosition() {
		return originalPosition;
	}

	public void setOriginalPosition(Point originalPosition) {
		this.originalPosition = originalPosition;
	}
	

	public RelativPolarPoint getRelativeToNodePosition() {
		return relativeToNodePosition;
	}

	public void setRelativeToNodePosition(RelativPolarPoint relativeToNodePosition) {
		this.relativeToNodePosition = relativeToNodePosition;
	}

	public boolean isRelative() {
		return isRelative;
	}

	public void setRelative(boolean isRelative) {
		this.isRelative = isRelative;
	}

	public boolean isBufferCrossing() {
		return isBufferCrossing;
	}

	public void setBufferCrossing(boolean isBufferCrossing) {
		this.isBufferCrossing = isBufferCrossing;
	}

	@Override
	public String toString() {
		return "ControlPoint [node=" + node + ", originalPosition=" + originalPosition + "]";
	}

	
}
