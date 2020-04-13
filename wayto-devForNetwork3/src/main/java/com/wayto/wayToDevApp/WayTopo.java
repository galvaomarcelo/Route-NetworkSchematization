package com.wayto.wayToDevApp;



import java.awt.Frame;

import javax.swing.JFrame;

import com.wayto.controller.MainController;
 



/**
 * Prompts the user for a shapefile and displays the contents on the screen in a map frame.
 * <p>
 * This is the GeoTools Quickstart application used in documentationa and tutorials. *
 */
public class WayTopo {
 
    /**
     * GeoTools Quickstart demo application. Prompts the user for a shapefile and displays its
     * contents on the screen in a map frame
     */
    public static void main(String[] args) throws Exception {
/*        // display a data store file chooser dialog for shapefiles
        File file = JFileDataStoreChooser.showOpenFile("shp", null);
        if (file == null) {
            return;
        }

        FileDataStore store = FileDataStoreFinder.getDataStore(file);
        SimpleFeatureSource featureSource = store.getFeatureSource();
        
        // Create a map context and add our shapefile to it
        MapContext map = new DefaultMapContext();
        map.setTitle("Quickstart");
        map.addLayer(featureSource, null);

        // Now display the map
        JMapFrame.showMap(map);*/
    	System.out.println("test");
    	System.setProperty("org.geotools.referencing.forceXY", "true");
	    JFrame frame = new JFrame("WAYTO For NetWork 2!");
	    frame.setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE);
	    frame.setExtendedState(frame.getExtendedState()|Frame.MAXIMIZED_BOTH);
	    //PanelController app = new PanelController();
	    MainController app = new MainController();	    

	    frame.add( app.getMainFrame() );
	    frame.setVisible(true); 

    	
    	
    }

}