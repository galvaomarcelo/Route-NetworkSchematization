package com.wayto.controller;

import java.util.Comparator;

import com.wayto.model.StreetNode;
import com.wayto.operator.GeometricOperation;

public class NodeCircularOrderComparator implements Comparator<StreetNode>
{
	StreetNode origin;
	NodeCircularOrderComparator(StreetNode origin){
        this.origin= origin;
    }
    public int compare(StreetNode n1, StreetNode n2) {
    	Double angleN1 = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(
				origin.getProjectGeom().getX(), origin.getProjectGeom().getY(),
				n1.getProjectGeom().getX(), n1.getProjectGeom().getY() ); 
    	Double angleN2 = GeometricOperation.getAngleBetweenPointsRelativeToAxisX(
				origin.getProjectGeom().getX(), origin.getProjectGeom().getY(),
				n2.getProjectGeom().getX(), n2.getProjectGeom().getY() ); 
        return Double.compare(angleN1, angleN2);
    }
}