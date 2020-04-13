package com.wayto.model;

import java.util.ArrayList;
import java.util.Comparator;
 
import org.locationtech.jts.geom.Point;
import com.wayto.model.topo.NodeTopoRelation;
import com.wayto.operator.GeoConvertionsOperations;


public class StreetNode {
	 
	
	

	private int id;
	private Point coordinate; /*source geographc coordinate*/
	private Point geom; /*target CRS projected geom*/
	private Point projectGeom; /*Normalized CRS projected geom*/
	private Point rescaledGeom = null;
	private Point xGeom; /*Schematized CRS projected geom*/
	private String geomAsJson;
	private boolean isRouteNode = false;
	private boolean isDecisionPoint = false; /*Street change name "Decision Point*/
	private boolean isRoundAbout = false;
	private boolean isRelevantRouteNode = false;
	private boolean isDisconnected = false;
	private boolean isFakeNode = false;
	private boolean isStreetNode = true;
	private int isPolygonNode = 0; /*polygon id or 0 if false*/
	private int isPointLMNode = 0; /*point id or 0 if false*/
	private ArrayList<NodeTopoRelation> topoRelations; 
	//private int crossingRegion = -1;
	/*priority = lowstes incident edge clazz and 0 is is route node, and 1 if is landmark */
	private int priority;
	private int degree;
	private byte edges = 0;
	
	int preOrder = -1;
	int postOrder = -1;
	
	private String sourceCRS = "EPSG:4326";
	private String targetCRS = "EPSG:3857";
	
	
	public StreetNode(int id, Point coordinate, boolean isRoute, int priority, boolean isSourceCRS) {
		super();
		this.id = id;		
		if(isSourceCRS) {
			this.coordinate = coordinate;
			this.geom = ((Point)GeoConvertionsOperations.convertProjection(coordinate,sourceCRS, targetCRS));
		}
		else {
			this.geom = coordinate;
			this.coordinate = ((Point)GeoConvertionsOperations.convertProjection(coordinate,targetCRS, sourceCRS));
		}
		this.isRouteNode = isRoute;
		this.priority = priority;
		this.isDecisionPoint = false;
		this.topoRelations = new ArrayList<NodeTopoRelation>();
	}
	
	public StreetNode(int id, Point coordinate, boolean isRoute, int priority, int isPolygonNode, boolean isStreetNode, boolean isSourceCRS) {
		super();
		this.id = id;
		if(isSourceCRS) {
			this.coordinate = coordinate;
			this.geom = ((Point)GeoConvertionsOperations.convertProjection(coordinate,sourceCRS, targetCRS));
		}
		else {
			this.geom = coordinate;
			this.coordinate = ((Point)GeoConvertionsOperations.convertProjection(coordinate,targetCRS, sourceCRS));
		}
		this.isRouteNode = isRoute;
		this.priority = priority;
		this.isPolygonNode = isPolygonNode;
		this.topoRelations = new ArrayList<NodeTopoRelation>();
		this.isStreetNode = isStreetNode;
	}
	

	public Point getGeomByType(int coordinateType) {
		switch (coordinateType) {
		case 0:
			return geom;
		case 1:					
			return projectGeom;
		case 2:
			return xGeom;
		case 3:
			return coordinate;
		default:
			return geom;
		}
		
	}
	
	public int getId() {
		return id;
	}
	public void setId(int id) {
		this.id = id;
	}
	
	
	
	public Point getCoordinate() {
		return coordinate;
	}

	public void setCoordinate(Point coordinate) {
		this.coordinate = coordinate;
	}

	public Point getGeom() {
		return geom;
	}
	public void setGeom(Point geom) {
		this.geom = geom;
	}
	public Point getProjectGeom() {
		return projectGeom;
	}
	public void setProjectGeom(Point projectGeom) {
		this.projectGeom = projectGeom;
	}
	public Point getxGeom() {
		return xGeom;
	}
	public void setxGeom(Point xGeom) {
		this.xGeom = xGeom;
	}
	
	public Point getRescaledGeom() {
		return rescaledGeom;
	}


	public void setRescaledGeom(Point rescaledGeom) {
		this.rescaledGeom = rescaledGeom;
	}


	public String getGeomAsJson() {
		return geomAsJson;
	}
	public void setGeomAsJson(String geomAsJson) {
		this.geomAsJson = geomAsJson;
	}
	
	
	public boolean isRouteNode() {
		return isRouteNode;
	}
	public void setRouteNode(boolean isRouteNode) {
		this.isRouteNode = isRouteNode;
	}
	public int getPriority() {
		return priority;
	}
	public void setPriority(int priority) {
		this.priority = priority;
	}
	
	
	public int getPreOrder() {
		return preOrder;
	}
	public void setPreOrder(int preOrder) {
		this.preOrder = preOrder;
	}
	public int getPostOrder() {
		return postOrder;
	}
	public void setPostOrder(int postOrder) {
		this.postOrder = postOrder;
	}
	
	
	public int getDegree() {
		return degree;
	}
	public void setDegree(int degree) {
		this.degree = degree;
	}
	
	public byte getEdges() {
		return edges;
	}
	public void setEdges(byte edges) {
		this.edges = edges;
	}
	
	
	public boolean isDecisionPoint() {
		return isDecisionPoint;
	}
	public void setDecisionPoint(boolean isDecisionPoint) {
		this.isDecisionPoint = isDecisionPoint;
	}
	
	
	
//	public int getCrossingRegion() {
//		return crossingRegion;
//	}
//
//
//	public void setCrossingRegion(int crossingRegion) {
//		this.crossingRegion = crossingRegion;
//	}


	public int getIsPolygonNode() {
		return isPolygonNode;
	}

	public void setIsPolygonNode(int isPolygonNode) {
		this.isPolygonNode = isPolygonNode;
	}

	
	public int getIsPointLMNode() {
		return isPointLMNode;
	}

	public void setIsPointLMNode(int isPointLMNode) {
		this.isPointLMNode = isPointLMNode;
	}

	public boolean isFakeNode() {
		return isFakeNode;
	}


	public void setFakeNode(boolean isFakeNode) {
		this.isFakeNode = isFakeNode;
	}


	public boolean isRoundAbout() {
		return isRoundAbout;
	}


	public void setRoundAbout(boolean isRoundAbout) {
		this.isRoundAbout = isRoundAbout;
	}


	public ArrayList<NodeTopoRelation> getTopoRelations() {
		return topoRelations;
	}


	public void setTopoRelations(ArrayList<NodeTopoRelation> topoRelations) {
		this.topoRelations = topoRelations;
	}

	

	public boolean isStreetNode() {
		return isStreetNode;
	}

	public void setStreetNode(boolean isStreetNode) {
		this.isStreetNode = isStreetNode;
	}
	

	public boolean isDisconnected() {
		return isDisconnected;
	}

	public void setDisconnected(boolean isDisconnected) {
		this.isDisconnected = isDisconnected;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((geom == null) ? 0 : geom.hashCode());
		result = prime * result + id;
		return result;
	}
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		StreetNode other = (StreetNode) obj;
		if (geom == null) {
			if (other.geom != null)
				return false;
		} else if (!geom.equals(other.geom))
			return false;
		if (id != other.id)
			return false;
		return true;
	}
	@Override
	public String toString() {
		return "StreetNode [id=" + id + ", coordinate=" + coordinate + "]";
	}
	
	public static Comparator<StreetNode> NodePriorityComparator = new Comparator<StreetNode>() {

		public int compare(StreetNode s1, StreetNode s2) {
			
			Integer nodePritorty1, nodePritorty2;
			if(s1.isRouteNode)
				nodePritorty1 = 0;
			else 
				nodePritorty1 = s1.getPriority();
			
			if(s2.isRouteNode)
				nodePritorty2 = 0;
			else
				nodePritorty2 = s2.getPriority();

			//ascending order
			return nodePritorty1.compareTo(nodePritorty2);

			//descending order
			//return StudentName2.compareTo(StudentName1);
		}

	};
	
	



	public boolean isCrossedBy(int id2) {
		for(NodeTopoRelation r: topoRelations){
			if(r.getFeatureId() == id2 && r.getType() == NodeTopoRelation.CROSSING)
				return true;
			
						
		}
		return false;
		
	}


	public boolean isTopoCrossing() {
		for(NodeTopoRelation r: topoRelations){
			if(r.getType() == NodeTopoRelation.CROSSING)
				return true;
			
						
		}
		return false;
	}
	
	public boolean isTopoAnchor() {
		for(NodeTopoRelation r: topoRelations){
			if(r.getType() == NodeTopoRelation.ANCHOR)
				return true;
			
						
		}
		return false;
	}
	
	public boolean isTopoAlong() {
		for(NodeTopoRelation r: topoRelations){
			if(r.getType() == NodeTopoRelation.ALONG_LEFT_END ||
					r.getType() == NodeTopoRelation.ALONG_LEFT_START ||
					r.getType() == NodeTopoRelation.ALONG_RIGHT_END ||
					r.getType() == NodeTopoRelation.ALONG_RIGHT_START ||
					r.getType() == NodeTopoRelation.ALONG_UNKNOWN)
				return true;
			
						
		}
		return false;
	}


	public boolean isRelevantRouteNode() {
		return isRelevantRouteNode;
	}


	public void setRelevantRouteNode(boolean isRelevantRouteNode) {
		this.isRelevantRouteNode = isRelevantRouteNode;
	}

	public boolean isAnchor() {
		for(NodeTopoRelation r: topoRelations){
			if(r.getType() == NodeTopoRelation.ANCHOR)
				return true;
			
						
		}
		return false;
	}






	

	
	

}
