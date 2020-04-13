package com.wayto.model;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.locationtech.jts.geom.Envelope;
import com.wayto.mapFeaturesModel.Layer;
import com.wayto.mapFeaturesModel.PolyLineLayer;
 
public class StreetNetworkTopological implements Cloneable{
	
	private Map<Integer,StreetEdge> edges;


	public StreetNetworkTopological() {
		super();
		this.edges =  new HashMap<Integer, StreetEdge>();
	}

	public Map<Integer,StreetEdge> getEdges() {
		return edges;
	}

	public void setEdges(Map<Integer,StreetEdge> edges) {
		this.edges = edges;
	}



	public StreetEdge getEdge(StreetNode streetNode1, StreetNode streetNode2) {
		StreetEdge theEdge = null;
		for(StreetEdge e: this.edges.values()){
			if( (e.getSourcePoint() == streetNode1 && e.getTargetPoint() == streetNode2) ||
					(e.getSourcePoint() == streetNode2 && e.getTargetPoint() == streetNode1)	){
				theEdge = e;
				break;
			}
		}
		if(theEdge == null)
			System.out.println("Edge with nodes not found " + streetNode1.getId() + ", " + streetNode2.getId());
		return theEdge;
	}
	
	
	
	@Override
	public StreetNetworkTopological clone() throws CloneNotSupportedException {
		
		StreetNetworkTopological clonedStreetNetwork = new StreetNetworkTopological();
		for(StreetEdge e: edges.values()) {
			StreetEdge clonedEdge = (StreetEdge)e.clone();
			clonedStreetNetwork.getEdges().put(clonedEdge.getId(), clonedEdge);
			
		}
		
		
		return clonedStreetNetwork;
	}

}
