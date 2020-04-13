package com.wayto.mapFeaturesModel;

import java.util.Comparator;

import com.wayto.model.StreetNode;

public interface Layer {
	
	/*type: 0 is point, 1 array of points(linestring)*/

	public static final int ORIGINAL_MAIN = 10;
	public static final int SCHEMATIC_MAIN = 11;
	
	public boolean isVisible();
	
	public void setVisible(boolean isVisible);

	public int getType() ;
	public void setType(int type) ;

	public String getName();
	
	
	/**
	 * 8 - route point landmarks
	 * 9 - route point elements
	 * 10 - route 
	 * 11 - network
	 * 11 - point land mark
	 * 12 - line landmark
	 * 13 - region landmark
	 * 14- venecular areas
	 * 
	 */
	public double getLayerDepth();
	

	public static Comparator<Layer> LayerDepthComparator = new Comparator<Layer>() {

		public int compare(Layer l1, Layer l2) {
			
			Double depth1 = l1.getLayerDepth();
			Double depth2 = l2.getLayerDepth();

			//ascending order
			return depth2.compareTo(depth1);

			//descending order
			//return StudentName2.compareTo(StudentName1);
		}

	};
	

}
