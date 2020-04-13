package com.wayto.mapFeaturesModel;

import java.util.ArrayList;

public class MapContent {

	private ArrayList<Layer> layers = null;

	public MapContent() {
		layers = new ArrayList<Layer>();
	}

	public void addLayer(Layer l){
		layers.add(l);
		
	}
	public ArrayList<Layer> getLayers() {
		return layers;
	}

	public void setLayers(ArrayList<Layer> layers) {
		this.layers = layers;
	}

	public void toogleVisibility(int r) {
		if(layers.get(r).isVisible())
			layers.get(r).setVisible(false);
		else
			layers.get(r).setVisible(true);
		
	}

	public void clearLayers() {
		layers = new ArrayList<Layer>();
		
	}
	
	
	
}
