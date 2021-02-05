package com.wayto.controller;

import static org.junit.Assert.assertNotNull;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.Point2D;
import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import org.codehaus.jettison.json.JSONArray;
import org.codehaus.jettison.json.JSONException;
import org.codehaus.jettison.json.JSONObject;
import org.geotools.geometry.jts.Geometries;
import org.locationtech.jts.algorithm.ConvexHull;
import org.locationtech.jts.densify.Densifier;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.simplify.DouglasPeuckerSimplifier;
import com.wayto.mapFeaturesModel.ClassyPointLayer;
import com.wayto.mapFeaturesModel.Layer;
import com.wayto.mapFeaturesModel.PlotPoint;
import com.wayto.mapFeaturesModel.PointLayer;
import com.wayto.mapFeaturesModel.PolyLineLayer;
import com.wayto.model.LinearFeature;
import com.wayto.model.Path;
import com.wayto.model.PointFeature;
import com.wayto.model.PolygonalFeature;
import com.wayto.model.RouteEdge;
import com.wayto.model.StreetEdge;
import com.wayto.model.StreetNode;
import com.wayto.model.topo.PointTopo;
import com.wayto.model.topo.PolygonalTopo;
import com.wayto.model.topo.Section;
import com.wayto.operator.GeoConvertionsOperations;
import com.wayto.operator.GeometricOperation;
import com.wayto.operator.LinearTransformation2;
import com.wayto.operator.OctlinearBoundingBox;

import com.wayto.operator.OptimizerOperator2;
import com.wayto.operator.PointsPolar;
import com.wayto.operator.Smoother;
import com.wayto.resultReport.PathReport;
import com.wayto.resultReport.ResultReport;
import com.wayto.view.MainFrame;
import com.wayto.view.XMap;

import ilog.concert.IloException;

public class MainController {
	 
	/* views controladas*/ 


	private MainFrame mainFrame = null;
	private XMap xMap = null;
	
	private float featureEnhance = 0.65f;
	final String exportFolder = "C:\\Users\\m_deli02\\Documents\\AfterErasmus\\Wayto\\JOURNALPAPER\\data\\";
	
	
	private LayerTableModel layerTableModel;

	/* estruturas de dados de controle */

	private MainFrameOperationListener operationHandler;
	private LayerTableListener tableListener;


	private RouteDataController dataController;
	
	private ResultReport resultReport;
	
	
	
	public MainController(){


		mainFrame = new MainFrame();
		operationHandler = new MainFrameOperationListener();
		mainFrame.addListener( operationHandler );

		//xMap.setRenderer(new StreamingRenderer());
		xMap = new XMap();
		layerTableModel = new LayerTableModel(xMap.getMapContent());
		tableListener = new LayerTableListener();

		mainFrame.getLayerTable().addMouseListener(tableListener);
		mainFrame.getLayerTable().setModel(layerTableModel);

		dataController = new RouteDataController();


		
		
		
		mainFrame.addTab("Schematic", xMap);


	}
	
	
	class MainFrameOperationListener implements ActionListener {  
		
		public void actionPerformed(ActionEvent e) {
			Object source = e.getSource();
			/**Load Data **/
			if (source == mainFrame.getbLoadRoute()) {
				long start, end;
				
				mainFrame.getTpCenter().setSelectedIndex(1);
				xMap.getMapContent().clearLayers();

				Double x1, y1, x2, y2;
				String originName, destinationName;
				if( mainFrame.getTextFieldSource().getText().isEmpty() ) {
					y1 = Double.valueOf( ((String)mainFrame.getCbSource().getSelectedItem()).split(",")[0] );
					x1 = Double.valueOf( ((String)mainFrame.getCbSource().getSelectedItem()).split(",")[1] );
					originName = ((String)mainFrame.getCbSource().getSelectedItem()).split(",")[2];
					
				}
				else {
					y1 = Double.valueOf( mainFrame.getTextFieldSource().getText().split(",")[0] );
					x1 = Double.valueOf( mainFrame.getTextFieldSource().getText().split(",")[1] );
					originName = "My origin";
					
				}
				
				if( mainFrame.getTextFieldDestination().getText().isEmpty() ) {
					y2 = Double.valueOf( ((String)mainFrame.getCbDestination().getSelectedItem()).split(",")[0] );
					x2 = Double.valueOf( ((String)mainFrame.getCbDestination().getSelectedItem()).split(",")[1] );
					destinationName = ((String)mainFrame.getCbDestination().getSelectedItem()).split(",")[2];
					
				}
				else {
					y2 = Double.valueOf( mainFrame.getTextFieldDestination().getText().split(",")[0] );
					x2 = Double.valueOf( mainFrame.getTextFieldDestination().getText().split(",")[1] );
					destinationName = "My destination";
					
				}
				
				/**Read Orign and Destination **/
				
					
//				y1 = Double.valueOf( ((String)mainFrame.getCbSource().getSelectedItem()).split(",")[0] );
//				x1 = Double.valueOf( ((String)mainFrame.getCbSource().getSelectedItem()).split(",")[1] );
//				y2 = Double.valueOf( ((String)mainFrame.getCbDestination().getSelectedItem()).split(",")[0] );
//				x2 = Double.valueOf( ((String)mainFrame.getCbDestination().getSelectedItem()).split(",")[1] );
//				String destino = ((String)mainFrame.getCbDestination().getSelectedItem()).split(",")[2];
//				String origem = ((String)mainFrame.getCbSource().getSelectedItem()).split(",")[2];
				
				int networkLevel = mainFrame.getCbStreetNetwork().getSelectedIndex();
				boolean reduceStubs = mainFrame.getCbReduceStubs().isSelected();
				double rescaleFactor = (double)mainFrame.getSliderScaleVariation().getValue();
				double rescaleRaiseFactor = (double)mainFrame.getSliderScaleVariationRaiseFactor().getValue();
				
				int rescaleNodeSelection = mainFrame.getCbRescalePoints().getSelectedIndex();
				start = System.currentTimeMillis(); 
				/**Call data controller an load data **/
				dataController.loadRouteData(x1, y1, x2, y2, originName	, destinationName, rescaleFactor, rescaleRaiseFactor, mainFrame.getSliderTranfsSimplification().getValue(), mainFrame.getSliderTranfsSimplificationNetwork().getValue(), networkLevel, reduceStubs, rescaleNodeSelection);
				
				end = System.currentTimeMillis();
				System.out.println("Load and prepare data time: " + (end - start) );
				
				Envelope env = dataController.getRoute().getRoutePath().asLineString(1).getEnvelopeInternal();

				/**Set map scale a and tranlation*/
				xMap.resetTransformations();

				double scale = 0.9/Math.max( (env.getWidth())/xMap.getWidth() , (env.getHeight())/xMap.getHeight());

				double tX = -env.getMinX()*scale + (xMap.getSize().getWidth()/2)  - env.getWidth()*scale/2;	
				double tY = -env.getMinY()*scale + (xMap.getSize().getHeight()/2) - env.getHeight()*scale/2;

				xMap.getTransformScale().scale(scale, scale);
				xMap.getTransformTranslate().translate(tX,tY);

				this.addOriginalMainLayers();
				this.addOriginalExtraLayers();

				layerTableModel.fireTableStructureChanged();

				xMap.repaint();
				
				
				
			}
			/*** Button Open Shape File ****/
			if (source == mainFrame.getbOpenShapeFile()) {

			}
			/**** Button Schematize** */
			if (source == mainFrame.getbSchematize()) {
				long start, end;
				start = System.currentTimeMillis(); 
				mainFrame.getTpCenter().setSelectedIndex(1);
				mainFrame.getTaPosicao().setText("Schematizing Route");
				System.out.println("SCHEMATIZING!!!!!!!!!!!!!!!!!!!");
				
				
				
//				double routeLenght = dataController.getRoute().getRescaledGeom().getLength();
//				double adjVerticesMinDist = Math.min(routeLenght/500, 1.5E-4) ;
//				//adjVerticesMinDist = 1.5E-4;
//				double adjToRouteEdgesMinLength = adjVerticesMinDist*2;
//				//adjToRouteEdgesMinLength = 1.5E-4;
//				double minNonAdjEdgeDist = adjVerticesMinDist;
				

				//double adjVerticesMinDist = routeLenght/3200 ;
				//double adjVerticesMinDist = 4.3521025465784055E-4;
				double minDistInput = Double.parseDouble(mainFrame.getTextFieldMinAdjDist().getText());
				
				double adjVerticesMinDist = minDistInput*(dataController.getRouteRescaleLengthProportion()/dataController.getMaxRouteProjectedEnvExtention());
				
				double minNonAdjEdgeDist = adjVerticesMinDist*Double.parseDouble(mainFrame.getTextFieldMinNonAdjDist().getText());
				
				double minDistToRoute = adjVerticesMinDist*Double.parseDouble(mainFrame.getTextFieldMinDistToRoute().getText());
				
				double adjToRouteEdgesFixLenght = adjVerticesMinDist*Double.parseDouble(mainFrame.getTextFieldStubFixDist().getText());
				
				double localLMFixDist = adjVerticesMinDist*Double.parseDouble(mainFrame.getTextFieldLocalLMFixDist().getText());
				
				double levelOfOrthogonality = Double.parseDouble(mainFrame.getTextFieldOrthoLevel().getText());
				double turnLevelOfOrthogonality = Double.parseDouble(mainFrame.getTextFieldTurnOrthoLevel().getText());
				
				//double adjToRouteEdgesMinLength = adjVerticesMinDist*2;

				//adjToRouteEdgesMinLength = 1.5E-4;
				
				
				
				System.out.println("adjVerticesMinDist: " + adjVerticesMinDist + "  minNonAdjEdgeDist: " + minNonAdjEdgeDist + "  adjToRouteEdgesFixLenght: " + adjToRouteEdgesFixLenght + "  localLMFixDist: " + localLMFixDist );

//				Envelope rescaledEnv = dataController.getRoute().getRescaledGeom().getEnvelopeInternal();
//
//				/**Set map scale a and tranlation*/
//				double scale = 0.9/Math.max( (rescaledEnv.getMaxX() - rescaledEnv.getMinX())/xMap.getWidth() , (rescaledEnv.getMaxY() - rescaledEnv.getMinY())/xMap.getHeight());
//
//				//xMap.resetTransformations();
//				xMap.transformScale.scale(scale, scale);
//				double tX = ((xMap.getWidth() - scale*(rescaledEnv.getMaxX() - rescaledEnv.getMinX()))/2);
//				double tY =  ((xMap.getHeight() - scale*(rescaledEnv.getMaxY() - rescaledEnv.getMinY()))/2);
//				xMap.transformTranslate.translate(tX,tY);
				
				
				for(Layer l: xMap.getMapContent().getLayers()){
					l.setVisible(false);
				}

				resultReport = new ResultReport(dataController.getPointTopoList().size() , dataController.getPolygonalTopoList().size());
				resultReport.setLenghtKm(dataController.getRoute().getRoutePath().asLineString(0).getLength());
				/***SCHEMATIZE THE ROUTE***/
				try {
					
					ArrayList<Point2D> MIPPath = 	OptimizerOperator2.routeOptimizerLazyTopologyCheck(
							dataController.getStreetNodeMap(),							
							dataController.getCircularOrderList(),
							dataController.getRoute(),
							dataController.getRescaledRouteGeom(),
							mainFrame.getSliderBendFactor().getValue(),/*bend*/
							mainFrame.getSliderCrossBendFactor().getValue(),/*bend*/
							mainFrame.getSliderEdgeOrientation().getValue(), /*dir*/
							mainFrame.getSliderDistFactor().getValue(), /*dist*/
							mainFrame.getSliderProportionDP().getValue(), /*proportion DP*/
							mainFrame.getSliderProportionAllPts().getValue(), /*proportion DP*/
							mainFrame.getSliderScaleVariation().getValue(), /*scale*/
							mainFrame.getCbDirectionModel().getSelectedIndex(), /*0:none , 1:bestdir, 2:trad, 3:klippel*/
							adjVerticesMinDist,
							minNonAdjEdgeDist,
							adjToRouteEdgesFixLenght,
							localLMFixDist,
							levelOfOrthogonality,
							turnLevelOfOrthogonality,
							mainFrame.getCbCheckRouteTopology().isSelected(),
							Integer.parseInt(mainFrame.getTextFieldExecutionTimeLimit().getText()), 
							resultReport
							);
							
							
							dataController.getPathList().get(0).setWasSchematized(true);

							
							
//							PolyLineLayer transfPathLayer = new PolyLineLayer("MIPRESuLT ", 8, 2, Color.LIGHT_GRAY, Color.LIGHT_GRAY, false, false);	
//							transfPathLayer.getLines().add(MIPPath);
//							xMap.getMapContent().getLayers().add(transfPathLayer);

					
							
//							ArrayList<Point2D> MIPPath = 	OptimizerOperator2.routeOptimizer2(
//									dataController.getStreetNodeMap(),							
//									dataController.getCircularOrderList(),
//									dataController.getRoute(),
//									mainFrame.getSliderBendFactor().getValue(),/*bend*/
//									mainFrame.getSliderEdgeOrientation().getValue(), /*dir*/
//									mainFrame.getSliderDistFactor().getValue(), /*dist*/
//									mainFrame.getCbDirectionModel().getSelectedIndex(), /*0:none , 1:bestdir, 2:trad, 3:klippel*/
//									Integer.parseInt(mainFrame.getTextFieldExecutionTimeLimit().getText()));
//									
//									dataController.getRoute().uptadeXGeom();
//									dataController.getPathList().get(0).setWasSchematized(true);
//									dataController.getPathList().get(0).setxGeom(dataController.getRoute().getxGeom());


				} catch (IloException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				} catch (Exception e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				
				/***NETWORK SCHEMATIZATION***/
				LinearTransformation2 lt = new LinearTransformation2();
				/***POLYGONS CONTROL EDGES*/
				for(PolygonalTopo pt: dataController.getPolygonalTopoList()){
					for(StreetEdge ce: pt.getControlEdges()) {
						
						ArrayList<StreetNode> auxNodeList = new ArrayList<StreetNode>();
						auxNodeList.add(ce.getTargetPoint());
						auxNodeList.add(ce.getSourcePoint());
						Path auxPAth = new Path(auxNodeList);
						double proportion = OptimizerOperator2.getControlEdgeProportion(auxPAth ,dataController.getRoute(), dataController.getNetworkEnvelop().maxExtent());
						//double proportion = 1;
//						System.out.println("PROPORTION Control Edege: " + proportion);
						if(pt.getType() == PolygonalTopo.PASSING_ALONG_LEFT ||
								pt.getType() == PolygonalTopo.PASSING_ALONG_RIGHT ||
								pt.getType() == PolygonalTopo.PASSING_ALONG_UNKNOW )
							proportion = proportion*((100.0 - mainFrame.getSliderAlongness().getValue())/100);
						ArrayList<Point2D> transPointList = lt.transformControlEdge(auxPAth , proportion);
//						System.out.println();
//						PolyLineLayer transfPathLayer = new PolyLineLayer("ControlEdge "  , 8, 2, Color.GREEN, Color.GREEN, false, true);	
//						transfPathLayer.getLines().add(transPointList);
//						xMap.getMapContent().getLayers().add(transfPathLayer);
						try {
							ArrayList<Point2D> MIPPath = OptimizerOperator2.networkpathOptimizer2(auxPAth,transPointList);
						}  catch (IloException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						} catch (Exception e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
						
					}
					
					
				}

				/***ADJUST ROUTE ADJ EDGES LENGHT (ALREADY SCHEMATIZED WITH THE ROUTE***/
				for(int i = 1; i < dataController.getPathList().size(); i++){
					Path adjPath = dataController.getPathList().get(i);
					/** if is adj edge but not LM  control edge**/
					if(dataController.getPathList().get(i).isRouteAdjEdge() ) {

//						ArrayList<Point2D> transPointList = null;
//						/**IF IT IS LM CONTROL EDGE (ALREADY SCHEMATIZED WITH THE ROUTE*/
//						if (adjPath.getNodeList().get(0).getIsPointLMNode() > 0 || adjPath.getNodeList().get(1).getIsPointLMNode() > 0)
//							transPointList = lt.transformRouteAdjEdgeFixDist(adjPath , 27*adjToRouteEdgesMinLength);
//						else
//							transPointList = lt.transformRouteAdjEdgeFixDist(adjPath , 27*adjToRouteEdgesMinLength);
						
						ArrayList<Point2D> transPointList = adjPath.asJava2DList(2);
						System.out.println();
//						PolyLineLayer transfPathLayer = new PolyLineLayer("TransAdjEdje " + i , 8, 2, Color.GREEN, Color.GREEN, false, true);	
//						transfPathLayer.getLines().add(transPointList);
//						xMap.getMapContent().getLayers().add(transfPathLayer);
						try {
							ArrayList<Point2D> MIPPath = OptimizerOperator2.networkpathOptimizer2(dataController.getPathList().get(i),transPointList);

							dataController.getPathList().get(i).setWasSchematized(true);
							
						}  catch (IloException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						} catch (Exception e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
					}
				}

				/***ADJUST CHUCK EDGES LENGHT (ALREADY SCHEMATIZED WITH THE ROUTE***/
				for(int i = 1; i < dataController.getPathList().size(); i++){
					if(dataController.getPathList().get(i).isChunkPath()) {
//						System.out.println("Adjusting ADjPath " + i + " size: " + dataController.getPathList().get(i).getNodeList().size() + " length: " + dataController.getPathList().get(i).getGeom().getLength() );
	
						Path adjPath = dataController.getPathList().get(i);
						if( adjPath.getNodeList().get(0).getIsPointLMNode() > 0 || adjPath.getNodeList().get(1).getIsPointLMNode() > 0) {
							System.out.println("this is LM control edge");
						}
						//ArrayList<Point2D> transPointList = lt.transformRouteAdjEdgeFixDist(adjPath , 27*adjToRouteEdgesMinLength);
						ArrayList<Point2D> transPointList = adjPath.asJava2DList(2);
//						System.out.println();
////						PolyLineLayer transfPathLayer = new PolyLineLayer("TransAdjEdje " + i , 8, 2, Color.GREEN, Color.GREEN, false, true);	
////						transfPathLayer.getLines().add(transPointList);
////						xMap.getMapContent().getLayers().add(transfPathLayer);
						try {
							ArrayList<Point2D> MIPPath = OptimizerOperator2.networkpathOptimizer2(dataController.getPathList().get(i),transPointList);

							dataController.getPathList().get(i).setWasSchematized(true);
							
						}  catch (IloException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						} catch (Exception e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
					}
				}
				/***NETWORK PATH SCHEMATIZATION***/
				for(int i = 1; i < dataController.getPathList().size(); i++){
					//					System.out.println("Schematizing path " + i + " size: " + dataController.getPathList().get(i).getNodeList().size() + " length: " + dataController.getPathList().get(i).getGeom().getLength() );

					if(!dataController.getPathList().get(i).getNodeList().get(0).isFakeNode()
							&& !dataController.getPathList().get(i).getNodeList().get(dataController.getPathList().get(i).getNodeList().size() -1).isFakeNode()
							&& !dataController.getPathList().get(i).isRouteAdjEdge()){

						Path p = dataController.getPathList().get(i);


						/****POLYGONA PATHS***/

						if(dataController.getPathList().get(i).getIsPolygon()>0) {
							PolygonalTopo pt = null;
							for(int j = 0;  j < dataController.getPolygonalTopoList().size(); j++) {


								/**ConvexHullText*/
								//	ConvexHull ch = new ConvexHull(dataController.getPathList().get(i).asLineString(1));

								//	PolyLineLayer convexHullLayer = new PolyLineLayer("ConvexPath " + i , 8, 2, Color.ORANGE, Color.ORANGE, true, true);	
								//	convexHullLayer.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(ch.getConvexHull()));
								//	xMap.getMapContent().getLayers().add(convexHullLayer);



								if (dataController.getPolygonalTopoList().get(j).getPolygonalFeature().getId() == dataController.getPathList().get(i).getIsPolygon()){
									pt = dataController.getPolygonalTopoList().get(j);
									break;
								}
							}
							System.out.println(" \n \nSchematizing Landmark " + pt.getPolygonalFeature().getName() + " size: " + dataController.getPathList().get(i).getNodeList().size() + " length: " + dataController.getPathList().get(i).asLineString(0).getLength() );

							
							PathReport resultPathReport = new PathReport();
							resultPathReport.setPathName(pt.getPolygonalFeature().getName());
							resultPathReport.setPathId(String.valueOf(i));
							
							double extendLimit = 0.05;
							/****DISCONNECTED POLYGONAL PATHS***/
							if(	pt.getType() == PolygonalTopo.PASSING_ALONG_LEFT || 
									pt.getType() == PolygonalTopo.PASSING_ALONG_RIGHT ||
									pt.getType() == PolygonalTopo.PASSING_ALONG_UNKNOW  
									) {

								//								double proportion = OptimizerOperator2.getPathProportion(dataController.getPathList().get(i),dataController.getStreetNetwork(), dataController.getNetworkEnvelop().maxExtent());
								//								System.out.println("PROPORTION: " + proportion);
								
								double proportion = OptimizerOperator2.getPathProportion3(dataController.getPathList().get(i),dataController.getStreetNetwork(), dataController.getNetworkEnvelop().maxExtent());

								ArrayList<Point2D> transPointList = lt.transformFullPolygon(dataController.getPathList().get(i) ,pt.getType(), proportion );
								
								
								PolyLineLayer transfPathLayer = new PolyLineLayer("TransPath " + i , 8, 3, Color.DARK_GRAY, Color.DARK_GRAY, false, false);	
								transfPathLayer.getLines().add(transPointList);
								xMap.getMapContent().getLayers().add(transfPathLayer);
								
								
								OctlinearBoundingBox octBox = new OctlinearBoundingBox(transPointList, 2*minNonAdjEdgeDist, extendLimit +0.1, 0);
								Polygon boundingPolygon = (Polygon)GeoConvertionsOperations.Java2DToJTSGeometry( octBox.getBoundingPolygon(), Geometries.POLYGON);
								

								PolyLineLayer OBBOXLayer = new PolyLineLayer("OBBOXPath " + i , 8, 2, Color.DARK_GRAY, Color.DARK_GRAY, false, false);	
								OBBOXLayer.getLines().add(octBox.getBoundingPolygon());
								xMap.getMapContent().getLayers().add(OBBOXLayer);
								
								
								double originalLenght = GeometricOperation.length(dataController.getPathList().get(i).asJava2DList(1));
								double transformedLendth = GeometricOperation.length(transPointList);
								double bestProportion = ((transformedLendth + originalLenght*proportion)/2)/transformedLendth;
							    //bestProportion = 1;
								//double bestProportion2 = 1/2 + (originalLenght*proportion)/2*transformedLendth;
								
								System.out.println("Original Length: " + GeometricOperation.length(dataController.getPathList().get(i).asJava2DList(1)));
								System.out.println("TransLegnt Length: " + GeometricOperation.length(transPointList));								
								System.out.println("PROPORTION: " + proportion);
								System.out.println("PROPORTION Best: " + bestProportion);
								
								
								if(mainFrame.getCbXIncorrects().isSelected() || mainFrame.getCbXall().isSelected()) {
									try {
										ArrayList<Point2D> MIPPath = 	OptimizerOperator2.landmarkAbstractOptimizerLazy(dataController.getPathList().get(i), pt , dataController.getPathList(), transPointList, 1, boundingPolygon,
												dataController.getStreetNodeMap(), dataController.getStreetNetwork(), 
												mainFrame.getSliderBendFactorRegion().getValue(),
												mainFrame.getSliderDistFactorRegion().getValue(),
												mainFrame.getSliderProportionRegion().getValue(),
												mainFrame.getSliderEdgeOrientationRegion().getValue(),
												mainFrame.getCbCheckSelfTopology().isSelected(),
												mainFrame.getCbCheckTopology().isSelected(),
												minDistToRoute,
												Integer.parseInt(mainFrame.getTextFieldExecutionTimeLimitRegion().getText()),
												resultPathReport);

										dataController.getPathList().get(i).setWasSchematized( true );

									}  catch (IloException e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									} catch (Exception e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									}

								}
								else {

									try {
										ArrayList<Point2D> MIPPath = 	OptimizerOperator2.networkpathOptimizer2(dataController.getPathList().get(i),transPointList);
									}  catch (IloException e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									} catch (Exception e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									}
								}


							}
							/****GLOBAL POLYGONAL PATHS***/
							else if(pt.getType() == PolygonalTopo.ROUTE_IS_INSIDE||
									pt.getType() == PolygonalTopo.GLOBAL 
									) {

								//								double proportion = OptimizerOperator2.getPathProportion(dataController.getPathList().get(i),dataController.getStreetNetwork(), dataController.getNetworkEnvelop().maxExtent());
								//								System.out.println("PROPORTION: " + proportion);
								
								
								double proportion = OptimizerOperator2.getPathProportion3(dataController.getPathList().get(i),dataController.getStreetNetwork(), dataController.getNetworkEnvelop().maxExtent());
								
								
								
								ArrayList<Point2D> transPointList = lt.transformFullPolygon(dataController.getPathList().get(i) , pt.getType(), proportion );

								PolyLineLayer transfPathLayer = new PolyLineLayer("TransPath " + i , 8, 3, Color.DARK_GRAY, Color.DARK_GRAY, false, false);	
								transfPathLayer.getLines().add(transPointList);
								xMap.getMapContent().getLayers().add(transfPathLayer);


								OctlinearBoundingBox octBox = new OctlinearBoundingBox(transPointList, 2*minNonAdjEdgeDist, extendLimit +0.1, 0);
								Polygon boundingPolygon = (Polygon)GeoConvertionsOperations.Java2DToJTSGeometry( octBox.getBoundingPolygon(), Geometries.POLYGON);
								

								PolyLineLayer OBBOXLayer = new PolyLineLayer("OBBOXPath " + i , 8, 2, Color.DARK_GRAY, Color.DARK_GRAY, false, false);
								OBBOXLayer.getLines().add(octBox.getBoundingPolygon());
								xMap.getMapContent().getLayers().add(OBBOXLayer);
								//dataController.getRoute().getxGeom().intersects(GeoConvertionsOperations.Java2DToJTSLineString(transPointList));

								
								
								
								if(
										mainFrame.getCbXall().isSelected() ||  
										(mainFrame.getCbXIncorrects().isSelected() &&
												dataController.getRoute().getRoutePath().asLineString(2).intersects(GeoConvertionsOperations.Java2DToJTSLineString(transPointList))) ) {

									try {
										ArrayList<Point2D> MIPPath = 	OptimizerOperator2.landmarkAbstractOptimizerLazy(dataController.getPathList().get(i), pt ,dataController.getPathList(), transPointList, 1, boundingPolygon,
												dataController.getStreetNodeMap(), dataController.getStreetNetwork(), 
												mainFrame.getSliderBendFactorRegion().getValue(),
												mainFrame.getSliderDistFactorRegion().getValue(),
												mainFrame.getSliderProportionRegion().getValue(),
												mainFrame.getSliderEdgeOrientationRegion().getValue(),
												mainFrame.getCbCheckSelfTopology().isSelected(),
												mainFrame.getCbCheckTopology().isSelected(),
												adjVerticesMinDist,
												Integer.parseInt(mainFrame.getTextFieldExecutionTimeLimitRegion().getText()),
												resultPathReport);

										dataController.getPathList().get(i).setWasSchematized( true );

									}  catch (IloException e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									} catch (Exception e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									}


								}
								else {


									try {
										ArrayList<Point2D> MIPPath = 	OptimizerOperator2.networkpathOptimizer2(dataController.getPathList().get(i),transPointList);
									}  catch (IloException e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									} catch (Exception e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									}


								}

							}
							/****CROSSED POLYGONAL PATHS***/
							else if(pt.getType() == PolygonalTopo.SIMPLE_CROSSING ||
									pt.getType() ==  PolygonalTopo.ROUTE_ENDS_AT ||
									pt.getType() ==  PolygonalTopo.ROUTE_STARTS_AT
									) {

								double proportion = OptimizerOperator2.getPathProportion3(dataController.getPathList().get(i),dataController.getStreetNetwork(), dataController.getNetworkEnvelop().maxExtent());
//								System.out.println("PROPORTION: " + proportion);
								ArrayList<Point2D> transPointList = lt.transformNetworkPath(dataController.getPathList().get(i) , proportion);
								/*put layer trans path*/
								PolyLineLayer transfPathLayer = new PolyLineLayer("TransPath " + i , 8, 3, Color.DARK_GRAY, Color.DARK_GRAY, false, false);	
								transfPathLayer.getLines().add(transPointList);
								xMap.getMapContent().getLayers().add(transfPathLayer);
																

								OctlinearBoundingBox octBox = new OctlinearBoundingBox(transPointList, 2*minNonAdjEdgeDist, extendLimit +0.1, 0);
								Polygon boundingPolygon = (Polygon)GeoConvertionsOperations.Java2DToJTSGeometry( octBox.getBoundingPolygon(), Geometries.POLYGON);

								
								
								PolyLineLayer topocheckLayer = new PolyLineLayer("TopoCheckLayerPath " + i , 8, 5, Color.RED, Color.RED, false, false);	
								PointLayer edgesNodesTopochecker = new PointLayer("PointTopoCheckLayerPath " + i , 8.1, 4,7, Color.RED, Color.RED, false);	
								
	
								PolyLineLayer OBBOXLayer = new PolyLineLayer("OBBOXPath " + i , 8, 2, Color.DARK_GRAY, Color.DARK_GRAY, false, false);	
								OBBOXLayer.getLines().add(octBox.getBoundingPolygon());
								xMap.getMapContent().getLayers().add(OBBOXLayer);

								for(Path p2: dataController.getPathList()) {
									if(p2.isWasSchematized()) {
										PointsPolar polarPoints = new PointsPolar();
										polarPoints  = GeometricOperation.toPolar(p2.asJava2DList(2));
										for(int k = 0; k < p2.getNodeList().size() - 1; k++) {
											if(!dataController.getPathList().get(i).getNodeList().contains(p2.getNodeList().get(k)) && !dataController.getPathList().get(i).getNodeList().contains(p2.getNodeList().get(k +1))) {
												
												if (boundingPolygon.contains( p2.getNodeList().get(k).getxGeom()  ) || boundingPolygon.contains( p2.getNodeList().get(k +1).getxGeom() )){
													int indexU = k;
													int indexV = k+1;
													boolean foundBend = false; 
													while(!foundBend ) {
														if(indexV < p2.getNodeList().size() -1 && !dataController.getPathList().get(i).getNodeList().contains(p2.getNodeList().get(indexV + 1))
																&& Math.abs(polarPoints.getPoints().get(indexV -1).getTheta() - polarPoints.getPoints().get(indexV).getTheta()) < 0.001
																&& boundingPolygon.contains( p2.getNodeList().get(indexV + 1).getxGeom()  ))
															indexV++;
														else {
															
															ArrayList<Point2D> edgeTopo = new ArrayList<Point2D>();
															edgeTopo.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p2.getNodeList().get(indexU).getxGeom()));
															edgeTopo.add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p2.getNodeList().get(indexV).getxGeom()));
															topocheckLayer.getLines().add(edgeTopo);
															edgesNodesTopochecker.getPoints().add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p2.getNodeList().get(indexU).getxGeom()));
															edgesNodesTopochecker.getPoints().add(GeoConvertionsOperations.PointJTSGeometryToJava2D(p2.getNodeList().get(indexV).getxGeom()));
															foundBend = true;
															k = indexV -1;
														}
													}
														
													
													
													
												}
											}
																	
										}
									}
								}
								
								xMap.getMapContent().getLayers().add(topocheckLayer);
								xMap.getMapContent().getLayers().add(edgesNodesTopochecker);
								
								
								if( mainFrame.getCbXall().isSelected() ||
										(mainFrame.getCbXIncorrects().isSelected() && 
										dataController.getRoute().getRoutePath().asLineString(2).crosses(GeoConvertionsOperations.Java2DToJTSLineString(new ArrayList<Point2D>(transPointList.subList(1, transPointList.size()-1)))))) {
//									if(dataController.getPathList().get(i).getNodeList().size() < 10) {
									try {
										ArrayList<Point2D> MIPPath = 	OptimizerOperator2.landmarkAbstractOptimizerLazy(dataController.getPathList().get(i), pt , dataController.getPathList(), transPointList, 1, boundingPolygon,
												dataController.getStreetNodeMap(), dataController.getStreetNetwork(), 
												mainFrame.getSliderBendFactorRegion().getValue(),
												mainFrame.getSliderDistFactorRegion().getValue(),
												mainFrame.getSliderProportionRegion().getValue(),
												mainFrame.getSliderEdgeOrientationRegion().getValue(),
												mainFrame.getCbCheckSelfTopology().isSelected(),
												mainFrame.getCbCheckTopology().isSelected(),
												adjVerticesMinDist,
												Integer.parseInt(mainFrame.getTextFieldExecutionTimeLimitRegion().getText()),
												resultPathReport);
										
					
										dataController.getPathList().get(i).setWasSchematized( true );
										
									}  catch (IloException e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									} catch (Exception e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									}
									
//									try {
//										ArrayList<Point2D> MIPPath = 	OptimizerOperator2.optimizeCrossSection(dataController.getPathList().get(i), dataController.getPathList(), transPointList, dataController.getStreetNodeMap(), dataController.getStreetNetwork(), 
//												mainFrame.getSliderBendFactorRegion().getValue(),
//												mainFrame.getSliderDistFactorRegion().getValue(),
//												mainFrame.getSliderProportionRegion().getValue(),
//												mainFrame.getSliderEdgeOrientationRegion().getValue(),
//												mainFrame.getCbCheckSelfTopology().isSelected(),
//												mainFrame.getCbCheckTopology().isSelected(),
//												minNonAdjEdgeDist,
//												Integer.parseInt(mainFrame.getTextFieldExecutionTimeLimitRegion().getText()));
//										dataController.getPathList().get(i).updatePathXNodes(MIPPath);
//										dataController.getPathList().get(i).uptadeXGeom();
//										dataController.getPathList().get(i).setWasSchematized( true );
//										
//									}  catch (IloException e1) {
//										// TODO Auto-generated catch block
//										e1.printStackTrace();
//									} catch (Exception e1) {
//										// TODO Auto-generated catch block
//										e1.printStackTrace();
//									}
//									
									
									
									

								}
								else {

									try {
										ArrayList<Point2D> MIPPath = 	OptimizerOperator2.networkpathOptimizer2(dataController.getPathList().get(i),transPointList);
										
									}  catch (IloException e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									} catch (Exception e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									}
								}

							}
							/****CONNECTED POLYGONAL PATHS***/
							else {

								double proportion = OptimizerOperator2.getPathProportion3(dataController.getPathList().get(i),dataController.getStreetNetwork(), dataController.getNetworkEnvelop().maxExtent());
//								System.out.println("PROPORTION: " + proportion);
								ArrayList<Point2D> transPointList = lt.transformNetworkPath(dataController.getPathList().get(i) , proportion);
								/*put layer trans path*/
								PolyLineLayer transfPathLayer = new PolyLineLayer("TransPath " + i , 8, 2, Color.LIGHT_GRAY, Color.LIGHT_GRAY, false, false);	
								transfPathLayer.getLines().add(transPointList);
								xMap.getMapContent().getLayers().add(transfPathLayer);
								
								

								OctlinearBoundingBox octBox = new OctlinearBoundingBox(transPointList, 2*minNonAdjEdgeDist, extendLimit +0.1, 0);
								Polygon boundingPolygon = (Polygon)GeoConvertionsOperations.Java2DToJTSGeometry( octBox.getBoundingPolygon(), Geometries.POLYGON);
								

								PolyLineLayer OBBOXLayer = new PolyLineLayer("OBBOXPath " + i , 8, 2, Color.DARK_GRAY, Color.DARK_GRAY, false, false);
								transfPathLayer.getLines().add(octBox.getBoundingPolygon());
								xMap.getMapContent().getLayers().add(OBBOXLayer);
								
								if( mainFrame.getCbXall().isSelected() ||
										(mainFrame.getCbXIncorrects().isSelected() &&
										dataController.getRoute().getRoutePath().asLineString(2).crosses(GeoConvertionsOperations.Java2DToJTSLineString(new ArrayList<Point2D>(transPointList.subList(1, transPointList.size()-1)))))) {
									try {
										ArrayList<Point2D> MIPPath = 	OptimizerOperator2.landmarkAbstractOptimizerLazy(dataController.getPathList().get(i), pt ,dataController.getPathList(), transPointList, 1, boundingPolygon,
												dataController.getStreetNodeMap(), dataController.getStreetNetwork(), 
												mainFrame.getSliderBendFactorRegion().getValue(),
												mainFrame.getSliderDistFactorRegion().getValue(),
												mainFrame.getSliderProportionRegion().getValue(),
												mainFrame.getSliderEdgeOrientationRegion().getValue(),
												mainFrame.getCbCheckSelfTopology().isSelected(),
												mainFrame.getCbCheckTopology().isSelected(),
												adjVerticesMinDist,
												Integer.parseInt(mainFrame.getTextFieldExecutionTimeLimitRegion().getText()),
												resultPathReport);
										
										dataController.getPathList().get(i).setWasSchematized( true );
										
									}  catch (IloException e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									} catch (Exception e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									}
									

								}
								
								else {
									try {
										ArrayList<Point2D> MIPPath = 	OptimizerOperator2.networkpathOptimizer2(dataController.getPathList().get(i),transPointList);
									}  catch (IloException e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									} catch (Exception e1) {
										// TODO Auto-generated catch block
										e1.printStackTrace();
									}
								}

							}

							if(resultPathReport.getObjectiveFunctionValue() > 0)
								resultReport.getPathReportList().add(resultPathReport);

							

						}
						/**END POLYGONAL PATHS**/
						/****STREET PATHS***/
						else {
							System.out.println("Street path: " + i + " nodes: " + dataController.getPathList().get(i).getNodeList().size());
							/***Stop here. I was using getPathProportion() of getPathProportion3();**/
							double proportion = OptimizerOperator2.getPathProportion3(dataController.getPathList().get(i),dataController.getStreetNetwork(), dataController.getNetworkEnvelop().maxExtent());
							System.out.println("PROPORTION: " + proportion);
							ArrayList<Point2D> transPointList = lt.transformNetworkPath(dataController.getPathList().get(i) , proportion);
							//							PolyLineLayer transfPathLayer = new PolyLineLayer("TransPath " + i , 8, 2, Color.LIGHT_GRAY, Color.LIGHT_GRAY, false, true);	
							//							transfPathLayer.getLines().add(transPointList);
							//							xMap.getMapContent().getLayers().add(transfPathLayer);
							
							if( dataController.getPathList().get(i).getNodeList().size() > 1 && (mainFrame.getCbXallNetwork().isSelected() ||
									(mainFrame.getCbXIncorrectsNetwork().isSelected() &&
									dataController.getRoute().getRoutePath().asLineString(2).crosses(GeoConvertionsOperations.Java2DToJTSLineString(new ArrayList<Point2D>(transPointList.subList(1, transPointList.size()-1))))))) {
								try {
									ArrayList<Point2D> MIPPath = 	OptimizerOperator2.streetPathOptimizer3DirTopoRelevant(dataController.getPathList().get(i), dataController.getPathList(), transPointList, dataController.getStreetNodeMap(), dataController.getStreetNetwork(), 
											mainFrame.getSliderBendFactorNetwork().getValue(),
											mainFrame.getSliderDistFactorNetwork().getValue(),
											mainFrame.getSliderProportionNetwork().getValue(),
											mainFrame.getSliderEdgeOrientationNetwork().getValue(),
											mainFrame.getCbCheckSelfTopologyNetwork().isSelected(),
											mainFrame.getCbCheckTopologyNetwork().isSelected(),
											adjVerticesMinDist,
											Integer.parseInt(mainFrame.getTextFieldExecutionTimeLimitRegion().getText()));
									
									dataController.getPathList().get(i).setWasSchematized( true );
									
								}  catch (IloException e1) {
									// TODO Auto-generated catch block
									e1.printStackTrace();
								} catch (Exception e1) {
									// TODO Auto-generated catch block
									e1.printStackTrace();
								}
								

							}
							
							else {
								try {
									ArrayList<Point2D> MIPPath = 	OptimizerOperator2.networkpathOptimizer2(dataController.getPathList().get(i),transPointList);
								}  catch (IloException e1) {
									// TODO Auto-generated catch block
									e1.printStackTrace();
								} catch (Exception e1) {
									// TODO Auto-generated catch block
									e1.printStackTrace();
								}
							}

						

//
//
//							
//							
//							
//							try {
//								ArrayList<Point2D> MIPPath = 	OptimizerOperator2.networkpathOptimizer2(dataController.getPathList().get(i),transPointList);
//							}  catch (IloException e1) {
//								// TODO Auto-generated catch block
//								e1.printStackTrace();
//							} catch (Exception e1) {
//								// TODO Auto-generated catch block
//								e1.printStackTrace();
//							}

						}




					}

				}
				/**END NETWORK SHCEMATIZATION**/
				
				
				/**POINT LIKE LANDMARK CONTROL EDGES THAT WHERE NOT SCHEMATIZED WITH THE ROUTE (GLOBALS)
				 * control edges of landmarks that are to far away from the route were not add to the adjacent list and are in no path so not schematized with the route**/
				for(PointTopo pt: dataController.getPointTopoList()){
					if(pt.getNode().getxGeom() == null ) {
						//System.out.println(pt.getPointFeature().getName());
						StreetEdge ce = pt.getControlEdge();
						ArrayList<StreetNode> auxNodeList = new ArrayList<StreetNode>();
						auxNodeList.add(ce.getTargetPoint());
						auxNodeList.add(ce.getSourcePoint());
						Path auxPAth = new Path(auxNodeList);
						double proportion = OptimizerOperator2.getControlEdgeProportion(auxPAth ,dataController.getRoute(), dataController.getNetworkEnvelop().maxExtent());
						//double proportion = OptimizerOperator2.getPathProportion(auxPAth ,dataController.getStreetNetwork(), dataController.getNetworkEnvelop().maxExtent());
						ArrayList<Point2D> transPointList = lt.transformControlEdge(auxPAth,proportion);
						/**check this**/
						//ArrayList<Point2D> transPointList = lt.transformControlEdgeFixDist(auxPAth , 35*adjToRouteEdgesMinLength);
						
						try {
							ArrayList<Point2D> MIPPath = OptimizerOperator2.networkpathOptimizer2(auxPAth,transPointList);
						}  catch (IloException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						} catch (Exception e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
					
					}

					
				}
				
				end = System.currentTimeMillis();
				System.out.println( resultReport );
				System.out.println( resultReport.toCSV() );
				System.out.println("Total Schematization time: " + (end - start) );
				this.addOriginalXRouteLayers();
				//this.addXDisconectedPolygonLayers();
				layerTableModel.fireTableStructureChanged();
				xMap.repaint();
				
				
				//				for( Path p: dataController.getPathList()){
//					
//					
//				}
				
//				int pathIndex = 0;
//				for( Path p: dataController.getPathList()){
//					if(pathIndex == 1){
//						try {
//							System.out.println("Schematizing path " + pathIndex);
//							ArrayList<Point2D> MIPPath = 	OptimizerOperator2.networkpathOptimizer(
//									dataController.getStreetNodeMap(),							
//									dataController.getCircularOrderList(),
//									p,
//									-1, 
//									-1,
//									mainFrame.getSliderBendFactor().getValue(),/*bend*/
//									mainFrame.getSliderEdgeOrientation().getValue(), /*dir*/
//									mainFrame.getSliderDistFactor().getValue(), /*dist*/		
//									Integer.parseInt(mainFrame.getTextFieldExecutionTimeLimit().getText()));
//						}  catch (IloException e1) {
//							// TODO Auto-generated catch block
//							e1.printStackTrace();
//						} catch (Exception e1) {
//							// TODO Auto-generated catch block
//							e1.printStackTrace();
//						}
//						
//					}
//					pathIndex++;
//				}
//				this.addXNetworkLayers();
//				layerTableModel.fireTableStructureChanged();
//				xMap.repaint();
				


			}
			/* Button Show All */
			if (source == mainFrame.getCbShowAllLayers()) {
				if(mainFrame.getCbShowAllLayers().isSelected() == false ){
					for(Layer l: xMap.getMapContent().getLayers()){
						l.setVisible(false);
					}

				}
				else{
					for(Layer l: xMap.getMapContent().getLayers()){
						l.setVisible(true);
					}
				}
				layerTableModel.fireTableStructureChanged();


				xMap.repaint();

			}
			/* Button Show All */
			if (source == mainFrame.getCbToggleXOLayers()) {
				if(mainFrame.getCbToggleXOLayers().isSelected() == false ){
					for(Layer l: xMap.getMapContent().getLayers()){
						if(l.getType() == Layer.ORIGINAL_MAIN)
							l.setVisible(true);
						else
							l.setVisible(false);
					}

				}
				else{
					for(Layer l: xMap.getMapContent().getLayers()){
						if(l.getType() == Layer.SCHEMATIC_MAIN)
							l.setVisible(true);
						else
							l.setVisible(false);
					}
				}
				layerTableModel.fireTableStructureChanged();


				xMap.repaint();

			}
			
			/*** Button Export Geojson File ****/
			if (source == mainFrame.getbExportGeoJson()) {
				String geoJsonText;
				LocalDateTime date = LocalDateTime.now();
				DateTimeFormatter formatter = DateTimeFormatter.ofPattern("ddMMyyHHmm");
				String timestamp = date.format(formatter);
				String fileName = "Route" + dataController.getRoute().getStartName() + dataController.getRoute().getEndName() + timestamp;
				if(mainFrame.getCbToggleXOLayers().isSelected() == false ) {
					geoJsonText = buildGeoJson(3, 1);
					//geoJsonText = builMultibleRescaledRouteGeoJson();
					//fileName = fileName.concat("MultpleRescale");
				}
				else {	
					geoJsonText = buildGeoJsonX(100);
					fileName = fileName.concat("X");
				}
				
				try {
					fileName = fileName.concat(".json");
					writeFile(geoJsonText, fileName, exportFolder  );
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}

			}
			
			if (source == mainFrame.getbReportFiles()) {
				String generalReport;
				String routeReport;
				String pathsReport;
				
				generalReport  = resultReport.generalCSV();
				routeReport = resultReport.routeCSV();
				pathsReport = resultReport.pathsCSV();
				LocalDateTime date = LocalDateTime.now();
				DateTimeFormatter formatter = DateTimeFormatter.ofPattern("ddMMyyHHmm");
				String timestamp = date.format(formatter);
				
				
				
				try {
					writeFile(generalReport, dataController.getRoute().getStartName() + dataController.getRoute().getEndName() + timestamp+ "General.csv" , exportFolder);
					writeFile(routeReport, dataController.getRoute().getStartName() + dataController.getRoute().getEndName() + timestamp+"Route.csv" , exportFolder);
					writeFile(pathsReport, dataController.getRoute().getStartName() + dataController.getRoute().getEndName() + timestamp+"Paths.csv" , exportFolder);
					
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}

			}
			
		}
		private  void writeFile(String content, String fileName, String location) throws IOException
		{
		    
		     System.out.println("Wrinting Files: " + fileName);
		    BufferedWriter writer = new BufferedWriter(new FileWriter(location+fileName));
		    writer.write(content);
		    writer.close();
		}

		/**This method was used only generate figure for the paper**/
		private String builMultibleRescaledRouteGeoJson(){
			
			
			JSONObject featureCollection = new JSONObject();
			

			ArrayList<Integer> dpIndexList = dataController.getRoute().getDPindex();
			if(dpIndexList.get(0)!=0)
				dpIndexList.add(0,0);
			if(dpIndexList.get(dpIndexList.size() -1)!= dataController.getRoute().asNodeList().size()-1)
				dpIndexList.add(dataController.getRoute().asNodeList().size()-1);
			
			
		    try {
		        featureCollection.put("type", "FeatureCollection");
		        JSONArray featureList = new JSONArray();
		        // iterate through your list
		        

		
				 JSONObject feature;
				 JSONArray lineString;
				 JSONObject geometry ;
				 JSONObject properties;
				 
			
				 ArrayList<Point2D> routePtList  = GeoConvertionsOperations.JTSGeometryToJavaD2( dataController.getRoute().getRoutePath().asLineString(1));
				 
				 feature = new JSONObject();
				 feature.put("type", "Feature");
				 lineString = new JSONArray();
				 for(Point2D pt: routePtList){
					 JSONArray point = new JSONArray();
					 point.put(pt.getX());
					 point.put(pt.getY());

					 lineString.put(point);
				 }

				 geometry = new JSONObject();
				 geometry.put("type", "LineString");
				 geometry.put("coordinates", lineString);

				 feature.put("geometry", geometry);

				 properties = new JSONObject();
				 properties.put("type", "route0");
				 properties.put("name", "route0");
				 //properties.put("color", arearef.getColor());


				 feature.put("properties", properties );		
				 featureList.put(feature);

				 
				 int dpcount = 0;
				 for(Integer dpIndex: dpIndexList) {
				 	feature = new JSONObject();
	            	feature.put("type", "Feature");
	            	
	            	JSONArray point = new JSONArray();/*coordinate point1*/
					
					point.put(routePtList.get(dpIndex).getX());
	            	point.put(routePtList.get(dpIndex).getY());
	            	
	            	geometry = new JSONObject();
	            	geometry.put("type", "Point");
	            	geometry.put("coordinates", point);
	            	feature.put("geometry", geometry);
	            	
	            	properties = new JSONObject();
		        	properties.put("type", "dps0");
		        	properties.put("name", "dp0" + dpcount);
		        	dpcount++;

		        	
		        	feature.put("properties", properties );		
		        	featureList.put(feature);
				 
				 }
				 ArrayList<Integer> dpIndexListEMpty = dataController.getRoute().getDPindex();
				 for(int i = 1;  i < 11; i++) {
					 float rescaleFactor = 0.1f*i;
					 LineString rescaledRouteLS = GeoConvertionsOperations.ScaleProcessing(dataController.getRoute().getRoutePath().asLineString(1), dpIndexListEMpty, rescaleFactor, 0.2);
					 routePtList  = GeoConvertionsOperations.JTSGeometryToJavaD2( rescaledRouteLS );
					 
					 feature = new JSONObject();
					 feature.put("type", "Feature");
					 lineString = new JSONArray();
					 for(Point2D pt: routePtList){
						 JSONArray point = new JSONArray();
						 point.put(pt.getX());
						 point.put(pt.getY());

						 lineString.put(point);
					 }

					 geometry = new JSONObject();
					 geometry.put("type", "LineString");
					 geometry.put("coordinates", lineString);

					 feature.put("geometry", geometry);

					 properties = new JSONObject();
					 properties.put("type", "route" + i);
					 properties.put("name", "route" + i);
					 properties.put("rescaleFactor", rescaleFactor);
					 //properties.put("color", arearef.getColor());


					 feature.put("properties", properties );		
					 featureList.put(feature);

					 
					 dpcount = 0;
					 for(Integer dpIndex: dpIndexList) {
					 	feature = new JSONObject();
		            	feature.put("type", "Feature");
		            	
		            	JSONArray point = new JSONArray();/*coordinate point1*/
						
						point.put(routePtList.get(dpIndex).getX());
		            	point.put(routePtList.get(dpIndex).getY());
		            	
		            	geometry = new JSONObject();
		            	geometry.put("type", "Point");
		            	geometry.put("coordinates", point);
		            	feature.put("geometry", geometry);
		            	
		            	properties = new JSONObject();
			        	properties.put("type", "dps" + i);
			        	properties.put("name", "dp" + String.valueOf(i) + String.valueOf(dpcount));
			        	dpcount++;

			        	
			        	feature.put("properties", properties );		
			        	featureList.put(feature);
					 
					 }
					 
					 
					 
				 }
				 
   
	            featureCollection.put("features", featureList);


		    } catch (JSONException e) {
		    	System.out.println("can't save json object: "+e.toString());
		    }
			
			
		   // System.out.println(featureCollection.toString());
			return featureCollection.toString();
		}
		
		/**coordinateType 
		 * 0 projection
		 * 1 normalized projection
		 * 2 schematic
		 * 3 geographic
		 * 4 default(projection)***/
		private String buildGeoJson(int coordinateType, double scale){
			
			
			long start, end;
			JSONObject featureCollection = new JSONObject();
			
			start = System.currentTimeMillis();
		    try {
		        featureCollection.put("type", "FeatureCollection");
		        JSONArray featureList = new JSONArray();
		        // iterate through your list
		        
		       int localPointLmCount = 0;
		       double sumControlEdgeLength = 0;
		       for(PointTopo pointLm :dataController.getPointTopoList()) {
		    	   /*if not global*/
		    	   if(pointLm.getType() != PointTopo.GLOBAL) {
		    		   sumControlEdgeLength += pointLm.getControlEdge().getLength(coordinateType);
		    		   localPointLmCount++;
		    		   System.out.println("found local " + pointLm.getPointFeature().getName());
		    		   System.out.println("length: " + pointLm.getControlEdge().getLength(coordinateType));
		    	   }
		       }
		       double meanCELenght = scale*sumControlEdgeLength/localPointLmCount;
		       System.out.println("Poin Like Landmar CE Length: " + meanCELenght);
		       	
		        /**PATHS***/
		        
				int pathIndex = 0;
				for (Path path: dataController.getStreetOnlyPathList()){
					/*OnlyRoute Condition*/

					
					if(   !(path.getIsPolygon()>0)){
						ArrayList<Path> pathClazzSubdivision = path.divideByClazzChange(dataController.getStreetNetwork());
						for(Path subPath: pathClazzSubdivision) {
							
							
							ArrayList<Point2D> pointList = null;
							
							
							ArrayList<Point2D> relevantPtList = new ArrayList<Point2D>();
							ArrayList<Integer> keyPtIndex = new ArrayList<Integer>();
							subPath.setSmothData(relevantPtList,keyPtIndex, coordinateType);
							if(mainFrame.getCbSmoothNetwork().isSelected() && relevantPtList.size() > 1 ) {					
								LineString smoothedPath = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList), keyPtIndex, 0.15, 8), 0.00005) ;
								//PolyLineLayer smoothedSectionPolygon = new PolyLineLayer(p.getPolygonalFeature().getName() + "Smoothes" ,  12, 3, Color.GRAY, Color.GRAY, true, true);
								//smoothedSectionPolygon.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon));
								
								//smoothedPath = Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.15, 8) ;
				
								pointList = GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPath);
							}
							else {
								pointList = relevantPtList;
							}

							JSONObject feature = new JSONObject();
							feature.put("type", "Feature");
							JSONArray lineString = new JSONArray();

							for(Point2D pt: pointList){
								JSONArray point = new JSONArray();
								point.put(scale*pt.getX());
								point.put(scale*pt.getY());

								lineString.put(point);
							}


							JSONObject geometry = new JSONObject();
							geometry.put("type", "LineString");
							geometry.put("coordinates", lineString);

							feature.put("geometry", geometry);

							JSONObject properties = new JSONObject();
							properties.put("type", "street-path");
							properties.put("name", "street-path");
							properties.put("clazz", subPath.getClazz());
							//properties.put("color", arearef.getColor());



							feature.put("properties", properties );		
							featureList.put(feature);




						}
//						
//						networkLayer.getLines().add(path.asJava2DList(2));
	//
//						xMap.getMapContent().getLayers().add(
//								networkLayer
//								);
					}
					pathIndex++;
					

				}
		        
		        /**ROUTE ADJ EDGES***/
					for (Path path: dataController.getRouteAdjPathList()){
						if(path.getIsPolygon() == 0) {
							
							JSONObject feature = new JSONObject();
							feature.put("type", "Feature");
							JSONArray lineString = new JSONArray();

							for(Point2D pt: path.asJava2DList(coordinateType)){
								JSONArray point = new JSONArray();
								point.put(scale*pt.getX());
								point.put(scale*pt.getY());

								lineString.put(point);
							}


							JSONObject geometry = new JSONObject();
							geometry.put("type", "LineString");
							geometry.put("coordinates", lineString);

							feature.put("geometry", geometry);

							JSONObject properties = new JSONObject();
							properties.put("type", "stub-path");
							properties.put("name", "stub-path");
							properties.put("clazz", path.getClazz());
							//properties.put("color", arearef.getColor());
							feature.put("properties", properties );		
							featureList.put(feature);
						}
						
						
					}
				
		        /**ROUTE ***/
		        LineString smotthedLine = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(dataController.getRoute().getRoutePath().asLineString(coordinateType), dataController.getRoute().getDPAnd2DegreeIndex(), 0.15, 8) , 0.00005);
		    
		        ArrayList<Point2D> smothLinePointList = GeoConvertionsOperations.JTSGeometryToJavaD2(smotthedLine);
		        JSONObject feature = new JSONObject();
            	feature.put("type", "Feature");
            	JSONArray lineString = new JSONArray();

            	for(Point2D pt: smothLinePointList){
            		JSONArray point = new JSONArray();
            		point.put(scale*pt.getX());
                	point.put(scale*pt.getY());
            		
                	lineString.put(point);
            	}
            	
            	
            	JSONObject geometry = new JSONObject();
            	geometry.put("type", "LineString");
            	geometry.put("coordinates", lineString);
            	
            	feature.put("geometry", geometry);
            	
            	JSONObject properties = new JSONObject();
            	properties.put("type", "route");
            	properties.put("name", "route");
            	properties.put("projectedScale", 1/dataController.getMaxRouteProjectedEnvExtention());
            	properties.put("pointLMDist", meanCELenght);
	        	//properties.put("color", arearef.getColor());
	        	feature.put("properties", properties );		
	        	featureList.put(feature);
		        
	        	/**ROUTE NODES**/
	        	int indexOfnode = 0;
	        	for(StreetNode n : dataController.getRoute().getRoutePath().getNodeList())
	        	
					if(n.isRouteNode()){
						
						feature = new JSONObject();
		            	feature.put("type", "Feature");
		            	
		            	JSONArray point = new JSONArray();/*coordinate point1*/
						
		            	point.put(scale*n.getGeomByType(coordinateType).getX());
		            	point.put(scale*n.getGeomByType(coordinateType).getY());
		            	
		            	geometry = new JSONObject();
		            	geometry.put("type", "Point");
		            	geometry.put("coordinates", point);
		            	feature.put("geometry", geometry);
		            	
		            	properties = new JSONObject();
			        	properties.put("type", "route-node");
			        	properties.put("name", "route-node");
			        	properties.put("order", indexOfnode);
			        	properties.put("lat", n.getCoordinate().getY());
			        	properties.put("lon", n.getCoordinate().getX());
			        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
			        	
			        	feature.put("properties", properties );		
			        	featureList.put(feature);
			        	
			        	indexOfnode++;
				}
	     
		        
		        
		        /**STREET NETWORK***/
	        	for(StreetEdge edge: dataController.getStreetNetwork().getEdges().values()){
	        		if(!edge.getSourcePoint().isDisconnected() && !edge.getTargetPoint().isDisconnected()) {
	        			/*exist schamatic geometry?*/


	        				/*if it is adjedge*/
	        				//						if(!edge.isFakeEdge() && 
	        				//								((edge.getSourcePoint().isRouteNode() && !edge.getTargetPoint().isRouteNode()) ||
	        				//								(!edge.getSourcePoint().isRouteNode() && edge.getTargetPoint().isRouteNode()) ))
	        				//							adjEdgesLayerX.getLines().add( edge.asPointList(2));

	        				/*if it is strictly street edge? if its not polygon edge?*/
//	        				if(!edge.isFakeEdge() && !(edge.getIsPolygonEdge()>0) ) {
//	        					if(edge.getSpecial() == 5)
//	        						System.out.println("this is brighe edge");
//	        					featureList.put(edge.toGeoJSONFeature(coordinateType));
//
//
//
//	        				}
	        				
	        				if(edge.isFakeEdge() )  {
	        									
								feature = new JSONObject();
								feature.put("type", "Feature");
								lineString = new JSONArray();

								JSONArray point1 = new JSONArray();
								JSONArray point2 = new JSONArray();
								
								lineString = new JSONArray();
								point1.put(edge.getSourcePoint().getGeomByType(coordinateType).getX());
								point1.put(edge.getSourcePoint().getGeomByType(coordinateType).getY());
								point2.put(edge.getTargetPoint().getGeomByType(coordinateType).getX());
								point2.put(edge.getTargetPoint().getGeomByType(coordinateType).getY());
								lineString.put(point1);
								lineString.put(point2);
	
								geometry = new JSONObject();
								geometry.put("type", "LineString");
								geometry.put("coordinates", lineString);

								properties = new JSONObject();
								properties.put("type", "control-edge");
								properties.put("name", "control-edge");
								
								feature.put("geometry", geometry);
								feature.put("properties", properties );		
								featureList.put(feature);



	        				}

	        			
	        		}

	        	}
		        
		        /**POLYGONAL FEATURES**/
				/****Polygonal Features ******/
				for(PolygonalTopo p:dataController.getPolygonalTopoList()){
					
//					p.setKeyPoints();
//					ArrayList<Point2D> relevantPtList = new ArrayList<Point2D>();
//					ArrayList<Point2D> finalPolygonPtList = new ArrayList<Point2D>();
//					ArrayList<Integer> keyPtIndex = new ArrayList<Integer>();
//					//p.setSmothData(relevantPtList,keyPtIndex);
//					if(mainFrame.getCbSmoothLandmarks().isSelected()) {
//						p.getPolygonalFeature().
//						LineString smoothedPolygon = Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList), keyPtIndex, 0.15, 8) ;
//						//PolyLineLayer smoothedSectionPolygon = new PolyLineLayer(p.getPolygonalFeature().getName() + "Smoothes" ,  12, 3, Color.GRAY, Color.GRAY, true, true);
//						//smoothedSectionPolygon.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon));
//						
//						if(p.getPolygonalFeature().getType().equals( "water" ))
//							smoothedPolygon = Smoother.smoothSpline2(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.2);
//						else
//							smoothedPolygon = Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.15, 8) ;
//		
//						finalPolygonPtList = GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon);
//						
//						
//					}
//					else {
//						finalPolygonPtList = relevantPtList;
//					}
					
						Point latLonCentroid  = p.getPolygonalFeature().getGeocoordsGeom().getCentroid();

				        feature = new JSONObject();
		            	feature.put("type", "Feature");
		            	JSONArray polygon = new JSONArray();
		            	JSONArray linearRing = new JSONArray();
		            	for(Point2D pt: p.asJava2DList(coordinateType)){
		            		JSONArray point = new JSONArray();
		            		point.put(scale*pt.getX());
		                	point.put(scale*pt.getY());
		            		
		                	linearRing.put(point);
		            	}
		            	polygon.put(linearRing);
		            	
		            	geometry = new JSONObject();
		            	geometry.put("type", "Polygon");
		            	geometry.put("coordinates", polygon);
		            	
		            	feature.put("geometry", geometry);
		            	System.out.println(p.getPolygonalFeature().getName() + "," + latLonCentroid.getY() + "," + latLonCentroid.getX());
		            	properties = new JSONObject();
		            	properties.put("type", p.getPolygonalFeature().getType());
		            	properties.put("name", p.getPolygonalFeature().getName());
		            	properties.put("lat", latLonCentroid.getY());
		            	properties.put("lon", latLonCentroid.getX());
		            	if(p.getType() == 1001)
		            		properties.put("topology", "global");
		            	else
		            		properties.put("topology", "local");
		            	if(p.getPolygonalFeature().getType().equals("urban")) {
		            		properties.put("labelpos", "1");
		            		properties.put("labelanchor", "middle");
		            	}
		            	else{
		            		properties.put("labelpos", "0");
		            		properties.put("labelanchor", "middle");
		            	}
			        	
			        	feature.put("properties", properties );		
			        	featureList.put(feature);
		            	
		            	
		           
				

				}
				
				/**ROUTE START***/
				
				feature = new JSONObject();
            	feature.put("type", "Feature");
            	
            	JSONArray startPoint = new JSONArray();/*coordinate point1*/
				
            	startPoint.put(scale*dataController.getRoute().getStart().getGeomByType(coordinateType).getX());
            	startPoint.put(scale*dataController.getRoute().getStart().getGeomByType(coordinateType).getY());
            	
            	geometry = new JSONObject();
            	geometry.put("type", "Point");
            	geometry.put("coordinates", startPoint);
            	feature.put("geometry", geometry);
            	
            	properties = new JSONObject();
	        	properties.put("type", "route-start");
	        	properties.put("name", "route-start");
	        	properties.put("lat", dataController.getRoute().getStart().getCoordinate().getX());
	        	properties.put("lon", dataController.getRoute().getStart().getCoordinate().getY());
	        	properties.put("edges", String.format("%8s", Integer.toBinaryString((dataController.getRoute().getStart().getEdges() + 256) % 256)).replace(' ', '0') );
	        	
	        	feature.put("properties", properties );		
	        	featureList.put(feature);
	        	
	        	
				feature = new JSONObject();
            	feature.put("type", "Feature");
            	
            	/**ROUTE END***/
            	JSONArray endPoint = new JSONArray();/*coordinate point1*/
				
            	endPoint.put(scale*dataController.getRoute().getEnd().getGeomByType(coordinateType).getX());
            	endPoint.put(scale*dataController.getRoute().getEnd().getGeomByType(coordinateType).getY());
            	
            	geometry = new JSONObject();
            	geometry.put("type", "Point");
            	geometry.put("coordinates", endPoint);
            	feature.put("geometry", geometry);
            	
            	properties = new JSONObject();
	        	properties.put("type", "route-end");
	        	properties.put("name", "route-end");
	        	properties.put("lat", dataController.getRoute().getEnd().getCoordinate().getX());
	        	properties.put("lon", dataController.getRoute().getEnd().getCoordinate().getY());
	        	properties.put("edges", String.format("%8s", Integer.toBinaryString((dataController.getRoute().getEnd().getEdges() + 256) % 256)).replace(' ', '0') );
	        	
	        	feature.put("properties", properties );		
	        	featureList.put(feature);
				
				
				/**NODE FEATURES***/
				
				for(StreetNode n: dataController.getStreetNodeMap().values()){
					

						if(n.isDecisionPoint()) {
							
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
							point.put(scale*n.getGeomByType(coordinateType).getX());
			            	point.put(scale*n.getGeomByType(coordinateType).getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "decision-point");
				        	properties.put("name", "decision-point");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						
						
						}
		
						if(n.isRoundAbout()  && n.isRouteNode()){
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
							point.put(scale*n.getGeomByType(coordinateType).getX());
			            	point.put(scale*n.getGeomByType(coordinateType).getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "round-about");
				        	properties.put("name", "round-about");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						}
						
						
						if(n.getTopoRelations().size() > 0){
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
			            	point.put(scale*n.getGeomByType(coordinateType).getX());
			            	point.put(scale*n.getGeomByType(coordinateType).getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "topo-point");
				        	properties.put("name", "topo-point");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						}
						

					


				}
		        
				/**POINT LIKE LANDMARKS**/
				for(PointTopo p:dataController.getPointTopoList()) {
					feature = new JSONObject();
	            	feature.put("type", "Feature");
	            	
	            	System.out.println(p.getPointFeature().getName() + "," + p.getNode().getCoordinate().getY() + "," + p.getNode().getCoordinate().getX());
	            	JSONArray point = new JSONArray();/*coordinate point1*/
					
					point.put(scale*p.getNode().getGeomByType(coordinateType).getX());
	            	point.put(scale*p.getNode().getGeomByType(coordinateType).getY());
	            	
	            	geometry = new JSONObject();
	            	geometry.put("type", "Point");
	            	geometry.put("coordinates", point);
	            	feature.put("geometry", geometry);
	            	
	            	properties = new JSONObject();
		        	properties.put("type", "pointlike-landmark");
		        	properties.put("subtype", p.getPointFeature().getType());
		        	properties.put("name", p.getPointFeature().getName());
		        	properties.put("salience", p.getPointFeature().getSalience());
		        	properties.put("icon", p.getPointFeature().getIcon());
		        	properties.put("lat",  p.getNode().getCoordinate().getY());
		        	properties.put("lon", p.getNode().getCoordinate().getX());
		        	properties.put("edges", String.format("%8s", Integer.toBinaryString((p.getNode().getEdges() + 256) % 256)).replace(' ', '0') );
		        	properties.put("labelpos", getLabelPosition(p.getNode().getGeomByType(coordinateType), p.getPointFeature().getName(), coordinateType));
		        	if(p.getType() == 1001)
	            		properties.put("topology", "global");
	            	else
	            		properties.put("topology", "local");
		        	
		        	feature.put("properties", properties );		
		        	featureList.put(feature);
				}  
		        
	            featureCollection.put("features", featureList);
	            
	            end = System.currentTimeMillis();
	    		System.out.println("json built time: " + (end - start) );

		    } catch (JSONException e) {
		    	System.out.println("can't save json object: "+e.toString());
		    }
			
			
		   // System.out.println(featureCollection.toString());
			return featureCollection.toString();
		}

		
		private String buildGeoJsonX(double scale){
			
			
			long start, end;
			JSONObject featureCollection = new JSONObject();
			
			start = System.currentTimeMillis();
		    try {
		        featureCollection.put("type", "FeatureCollection");
		        JSONArray featureList = new JSONArray();
		        // iterate through your list
		        
		        int localPointLmCount = 0;
			       double sumControlEdgeLength = 0;
			       for(PointTopo pointLm :dataController.getPointTopoList()) {
			    	   /*if not global*/
			    	   if(pointLm.getType() != PointTopo.GLOBAL) {
			    		   System.out.println("found local " + pointLm.getPointFeature().getName());
			    		   System.out.println("length: " + pointLm.getControlEdge().getLength(2));
			    		   sumControlEdgeLength += pointLm.getControlEdge().getLength(2);
			    		   localPointLmCount++;
			    	   }
			       }
			       double meanCELenght = scale*sumControlEdgeLength/localPointLmCount;
			       System.out.println("Poin Like Landmar CE Length: " + meanCELenght);
	
		        
		        /**PATHS***/
		        
				int pathIndex = 0;
				for (Path path: dataController.getStreetOnlyPathList()){
					
					if(path.isRoute()){

						

					}
					
					if(   !(path.getIsPolygon()>0)){
						ArrayList<Path> pathClazzSubdivision = path.divideByClazzChange(dataController.getStreetNetwork());
						for(Path subPath: pathClazzSubdivision) {
							
							
							ArrayList<Point2D> pointList = null;
							
							
							ArrayList<Point2D> relevantPtList = new ArrayList<Point2D>();
							ArrayList<Integer> keyPtIndex = new ArrayList<Integer>();
							subPath.setSmothData(relevantPtList,keyPtIndex,2);
							if(mainFrame.getCbSmoothNetwork().isSelected() && relevantPtList.size() > 1 ) {					
								LineString smoothedPath = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList), keyPtIndex, 0.15, 8), 0.00005) ;
								//PolyLineLayer smoothedSectionPolygon = new PolyLineLayer(p.getPolygonalFeature().getName() + "Smoothes" ,  12, 3, Color.GRAY, Color.GRAY, true, true);
								//smoothedSectionPolygon.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon));
								
								//smoothedPath = Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.15, 8) ;
				
								pointList = GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPath);
							}
							else {
								pointList = relevantPtList;
							}

							JSONObject feature = new JSONObject();
							feature.put("type", "Feature");
							JSONArray lineString = new JSONArray();

							for(Point2D pt: pointList){
								JSONArray point = new JSONArray();
								point.put(scale*pt.getX());
								point.put(scale*pt.getY());

								lineString.put(point);
							}


							JSONObject geometry = new JSONObject();
							geometry.put("type", "LineString");
							geometry.put("coordinates", lineString);

							feature.put("geometry", geometry);

							JSONObject properties = new JSONObject();
							properties.put("type", "street-path");
							properties.put("name", "street-path");
							properties.put("clazz", subPath.getClazz());
							//properties.put("color", arearef.getColor());



							feature.put("properties", properties );		
							featureList.put(feature);




						}
//						
//						networkLayer.getLines().add(path.asJava2DList(2));
	//
//						xMap.getMapContent().getLayers().add(
//								networkLayer
//								);
					}
					pathIndex++;
					

				}
				
		        /**ROUTE ADJ EDGES***/
					for (Path path: dataController.getRouteAdjPathList()){
						if(path.getIsPolygon() == 0) {
							
							JSONObject feature = new JSONObject();
							feature.put("type", "Feature");
							JSONArray lineString = new JSONArray();

							for(Point2D pt: path.asJava2DList(2)){
								JSONArray point = new JSONArray();
								point.put(scale*pt.getX());
								point.put(scale*pt.getY());

								lineString.put(point);
							}


							JSONObject geometry = new JSONObject();
							geometry.put("type", "LineString");
							geometry.put("coordinates", lineString);

							feature.put("geometry", geometry);

							JSONObject properties = new JSONObject();
							properties.put("type", "stub-path");
							properties.put("name", "stub-path");
							properties.put("clazz", path.getClazz());
							//properties.put("color", arearef.getColor());
							feature.put("properties", properties );		
							featureList.put(feature);
						}
						
						
					}
				
				
				 JSONObject feature;
				 JSONArray lineString;
				 JSONObject geometry ;
				 JSONObject properties;
		        
		        /**ROUTE ***/
				 
					Path routePath = dataController.getRoute().getRoutePath();
					
					/**Divide the route path between DP**/
//					ArrayList<Path> routePathDPSubdivision = routePath.divideByDP(dataController.getStreetNetwork());
//					for(Path subPath: routePathDPSubdivision) {
//						
//						
//						ArrayList<Point2D> relevantPtList = new ArrayList<Point2D>();
//						ArrayList<Integer> keyPtIndex = new ArrayList<Integer>();
//						subPath.setSmothData(relevantPtList,keyPtIndex,2);
//								
//						LineString smoothedSubPath = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList), keyPtIndex, 0.15, 8), 0.00005) ;
//
//						double lengthInMeters = subPath.getLength(0);	
//						double lengthInPixels = subPath.getLength(2);
//						double numberOfParts;
//						
//						ArrayList<LineString> routeSectionSubdivision;
//						if(lengthInMeters<250) {
//							routeSectionSubdivision = new ArrayList<LineString>();
//							routeSectionSubdivision.add(smoothedSubPath);
//						}							
//						else {
//							numberOfParts = lengthInMeters/250;
//							double sectionLengthInPixel = lengthInPixels/numberOfParts;
//							routeSectionSubdivision = GeoConvertionsOperations.splitLineStringIntoParts(smoothedSubPath, sectionLengthInPixel);
//						}
//						
//						ArrayList<Point2D> pointList = null;
//						for(LineString routeSection: routeSectionSubdivision) {
//							pointList = GeoConvertionsOperations.JTSGeometryToJavaD2(routeSection);
//					        feature = new JSONObject();
//			            	feature.put("type", "Feature");
//			            	lineString = new JSONArray();
//							for(Point2D pt: pointList){
//			            		JSONArray point = new JSONArray();
//			            		point.put(scale*pt.getX());
//			                	point.put(scale*pt.getY());
//			            		
//			                	lineString.put(point);
//			            	}
//			            	
//			            	
//			            	geometry = new JSONObject();
//			            	geometry.put("type", "LineString");
//			            	geometry.put("coordinates", lineString);
//			            	
//			            	feature.put("geometry", geometry);
//			            	
//			            	properties = new JSONObject();
//			            	properties.put("type", "route");
//			            	properties.put("name", "route");
//			            	properties.put("projectedScale", 1/(dataController.getMaxRouteProjectedEnvExtention()));
//				        	//properties.put("color", arearef.getColor());
//				        	
//				        	
//
//				        	feature.put("properties", properties );		
//				        	featureList.put(feature);
//							
//							
//						}
//						
//						
//						
//					}
					
					/**Single route path**/	
		        LineString smotthedLine = Smoother.geometrySmoothCut(routePath.asLineString(2), dataController.getRoute().getDPAnd2DegreeIndex(), 0.15, 8) ;
		    
		        
		        ArrayList<Point2D> smothLinePointList = GeoConvertionsOperations.JTSGeometryToJavaD2(smotthedLine);
		        
		        //ArrayList<LineString> = GeoConvertionsOperations.splitLineString(smotthedLine, )
		        feature = new JSONObject();
            	feature.put("type", "Feature");
            	lineString = new JSONArray();

            	for(Point2D pt: smothLinePointList){
            		JSONArray point = new JSONArray();
            		point.put(scale*pt.getX());
                	point.put(scale*pt.getY());
            		
                	lineString.put(point);
            	}
            	
            	
            	geometry = new JSONObject();
            	geometry.put("type", "LineString");
            	geometry.put("coordinates", lineString);
            	
            	feature.put("geometry", geometry);
            	
            	properties = new JSONObject();
            	properties.put("type", "route");
            	properties.put("name", "route");
            	properties.put("projectedScale", 1/(dataController.getMaxRouteProjectedEnvExtention()));
            	properties.put("pointLMDist", meanCELenght);
	        	//properties.put("color", arearef.getColor());
	        	
	        	

	        	feature.put("properties", properties );		
	        	featureList.put(feature);
		        
	        	
	        	/**ROUTE NODES**/
	        	int indexOfnode = 0;
	        	for(StreetNode n : dataController.getRoute().getRoutePath().getNodeList())
	        	
					if(n.isRouteNode()){
						
						feature = new JSONObject();
		            	feature.put("type", "Feature");
		            	
		            	JSONArray point = new JSONArray();/*coordinate point1*/
						
		            	point.put(scale*n.getxGeom().getX());
		            	point.put(scale*n.getxGeom().getY());
		            	
		            	geometry = new JSONObject();
		            	geometry.put("type", "Point");
		            	geometry.put("coordinates", point);
		            	feature.put("geometry", geometry);
		            	
		            	properties = new JSONObject();
			        	properties.put("type", "route-node");
			        	properties.put("name", "route-node");
			        	properties.put("order", indexOfnode);
			        	properties.put("lat", n.getCoordinate().getY());
			        	properties.put("lon", n.getCoordinate().getX());
			        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
			        	
			        	feature.put("properties", properties );		
			        	featureList.put(feature);
			        	
			        	indexOfnode++;
				}
		        
		        
		        /**STREET NETWORK***/
	        	for(StreetEdge edge: dataController.getStreetNetwork().getEdges().values()){
	        		if(!edge.getSourcePoint().isDisconnected() && !edge.getTargetPoint().isDisconnected()) {
	        			/*exist schamatic geometry?*/
	        			if(edge.getSourcePoint().getxGeom()!= null && edge.getTargetPoint().getxGeom()!= null){

	        				/*if it is adjedge*/
	        				//						if(!edge.isFakeEdge() && 
	        				//								((edge.getSourcePoint().isRouteNode() && !edge.getTargetPoint().isRouteNode()) ||
	        				//								(!edge.getSourcePoint().isRouteNode() && edge.getTargetPoint().isRouteNode()) ))
	        				//							adjEdgesLayerX.getLines().add( edge.asPointList(2));

	        				/*if it is strictly street edge? if its not polygon edge?*/
//	        				if(!edge.isFakeEdge() && !(edge.getIsPolygonEdge()>0) ) {
//	        					if(edge.getSpecial() == 5)
//	        						System.out.println("this is brighe edge");
//	        					featureList.put(edge.toGeoJSONFeature(2));
//
//
//
//	        				}
	        				if(edge.isFakeEdge() )  {
								
								feature = new JSONObject();
								feature.put("type", "Feature");
								lineString = new JSONArray();

								JSONArray point1 = new JSONArray();
								JSONArray point2 = new JSONArray();
								
								lineString = new JSONArray();
								point1.put(edge.getSourcePoint().getxGeom().getX());
								point1.put(edge.getSourcePoint().getxGeom().getY());
								point2.put(edge.getTargetPoint().getxGeom().getX());
								point2.put(edge.getTargetPoint().getxGeom().getY());
								lineString.put(point1);
								lineString.put(point2);
	
								geometry = new JSONObject();
								geometry.put("type", "LineString");
								geometry.put("coordinates", lineString);

								properties = new JSONObject();
								properties.put("type", "control-edge");
								properties.put("name", "control-edge");
								
								feature.put("geometry", geometry);
								feature.put("properties", properties );		
								featureList.put(feature);



	        				}

	        			}
	        		}

	        	}
		        
		        /**POLYGONAL FEATURES**/
				/****Polygonal Features ******/
				for(PolygonalTopo p:dataController.getPolygonalTopoList()){
					
					p.setKeyPoints();
					ArrayList<Point2D> relevantPtList = new ArrayList<Point2D>();
					ArrayList<Point2D> finalPolygonPtList = new ArrayList<Point2D>();
					ArrayList<Integer> keyPtIndex = new ArrayList<Integer>();
					p.setSmothData(relevantPtList,keyPtIndex);
					if(mainFrame.getCbSmoothLandmarks().isSelected()) {
						
						LineString smoothedPolygon = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList), keyPtIndex, 0.15, 8), 0.00005) ;
						//PolyLineLayer smoothedSectionPolygon = new PolyLineLayer(p.getPolygonalFeature().getName() + "Smoothes" ,  12, 3, Color.GRAY, Color.GRAY, true, true);
						//smoothedSectionPolygon.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon));
						
//						if(p.getPolygonalFeature().getType().equals( "water" ))
//							smoothedPolygon = Smoother.smoothSpline2(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.2);
//						else
//							smoothedPolygon = Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.15, 8) ;
		
						finalPolygonPtList = GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon);
						
						
					}
					else {
						finalPolygonPtList = relevantPtList;
					}
					
					

				        feature = new JSONObject();
		            	feature.put("type", "Feature");
		            	JSONArray polygon = new JSONArray();
		            	JSONArray linearRing = new JSONArray();
		            	for(Point2D pt: finalPolygonPtList){
		            		JSONArray point = new JSONArray();
		            		if(!Double.isNaN(pt.getX()) &&  !Double.isNaN(pt.getX()) ) {
		            			point.put(scale*pt.getX());
			                	point.put(scale*pt.getY());
		            		}
		            		else
		            			System.out.println("Something strange in polygon smoothed: " +  pt);
		            		
		            		
		            		
		                	linearRing.put(point);
		            	}
		            	polygon.put(linearRing);
		            	
		            	geometry = new JSONObject();
		            	geometry.put("type", "Polygon");
		            	geometry.put("coordinates", polygon);
		            	
		            	feature.put("geometry", geometry);
		            	
		            	properties = new JSONObject();
		            	properties.put("type", p.getPolygonalFeature().getType());
		            	properties.put("name", p.getPolygonalFeature().getName());
		            	if(p.getType() == 1001)
		            		properties.put("topology", "global");
		            	else
		            		properties.put("topology", "local");
		            	if(p.getPolygonalFeature().getType().equals("urban")) {
		            		properties.put("labelpos", "1");
		            		properties.put("labelanchor", "middle");
		            	}
		            	else{
		            		properties.put("labelpos", "0");
		            		properties.put("labelanchor", "middle");
		            	}
			        	
			        	feature.put("properties", properties );		
			        	featureList.put(feature);
		            	
		            	
		           
				

				}
				
				/**ROUTE START***/
				
				feature = new JSONObject();
            	feature.put("type", "Feature");
            	
            	JSONArray startPoint = new JSONArray();/*coordinate point1*/
				
            	startPoint.put(scale*dataController.getRoute().getStart().getxGeom().getX());
            	startPoint.put(scale*dataController.getRoute().getStart().getxGeom().getY());
            	
            	geometry = new JSONObject();
            	geometry.put("type", "Point");
            	geometry.put("coordinates", startPoint);
            	feature.put("geometry", geometry);
            	
            	properties = new JSONObject();
	        	properties.put("type", "route-start");
	        	properties.put("name", "route-start");
	        	properties.put("lat", dataController.getRoute().getStart().getCoordinate().getX());
	        	properties.put("lon", dataController.getRoute().getStart().getCoordinate().getY());
	        	properties.put("edges", String.format("%8s", Integer.toBinaryString((dataController.getRoute().getStart().getEdges() + 256) % 256)).replace(' ', '0') );
	        	
	        	feature.put("properties", properties );		
	        	featureList.put(feature);
	        	
	        	
				feature = new JSONObject();
            	feature.put("type", "Feature");
            	
            	/**ROUTE END***/
            	JSONArray endPoint = new JSONArray();/*coordinate point1*/
				
            	endPoint.put(scale*dataController.getRoute().getEnd().getxGeom().getX());
            	endPoint.put(scale*dataController.getRoute().getEnd().getxGeom().getY());
            	
            	geometry = new JSONObject();
            	geometry.put("type", "Point");
            	geometry.put("coordinates", endPoint);
            	feature.put("geometry", geometry);
            	
            	properties = new JSONObject();
	        	properties.put("type", "route-end");
	        	properties.put("name", "route-end");
	        	properties.put("lat", dataController.getRoute().getEnd().getCoordinate().getX());
	        	properties.put("lon", dataController.getRoute().getEnd().getCoordinate().getY());
	        	properties.put("edges", String.format("%8s", Integer.toBinaryString((dataController.getRoute().getEnd().getEdges() + 256) % 256)).replace(' ', '0') );
	        	
	        	feature.put("properties", properties );		
	        	featureList.put(feature);
				
				/**NODE FEATURES***/
				
				for(StreetNode n: dataController.getStreetNodeMap().values()){
					
					
					if(n.getxGeom()!= null) {
						
						
						if(n.isDecisionPoint()) {
							
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
							point.put(scale*n.getxGeom().getX());
			            	point.put(scale*n.getxGeom().getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "decision-point");
				        	properties.put("name", "decision-point");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						
						
						}
		
						if(n.isRoundAbout()  && n.isRouteNode()){
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
							point.put(scale*n.getxGeom().getX());
			            	point.put(scale*n.getxGeom().getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "round-about");
				        	properties.put("name", "round-about");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						}
						
						
						if(n.getTopoRelations().size() > 0){
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
							point.put(scale*n.getxGeom().getX());
			            	point.put(scale*n.getxGeom().getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "topo-point");
				        	properties.put("name", "topo-point");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						}
						

						
						
						
					}


				}
		        
				/**POINT LIKE LANDMARKS**/
				for(PointTopo p:dataController.getPointTopoList()) {
					feature = new JSONObject();
	            	feature.put("type", "Feature");
	            	
	            	JSONArray point = new JSONArray();/*coordinate point1*/
					
					point.put(scale*p.getNode().getxGeom().getX());
	            	point.put(scale*p.getNode().getxGeom().getY());
	            	
	            	geometry = new JSONObject();
	            	geometry.put("type", "Point");
	            	geometry.put("coordinates", point);
	            	feature.put("geometry", geometry);
	            	
	            	properties = new JSONObject();
		        	properties.put("type", "pointlike-landmark");
		        	properties.put("subtype", p.getPointFeature().getType());
		        	properties.put("name", p.getPointFeature().getName());
		        	properties.put("salience", p.getPointFeature().getSalience());
		        	properties.put("icon", p.getPointFeature().getIcon());
		        	properties.put("lat",  p.getNode().getCoordinate().getY());
		        	properties.put("lon", p.getNode().getCoordinate().getX());
		        	properties.put("edges", String.format("%8s", Integer.toBinaryString((p.getNode().getEdges() + 256) % 256)).replace(' ', '0') );
		        	properties.put("labelpos", getLabelPosition(p.getNode().getxGeom(), p.getPointFeature().getName(), 2));
		        	if(p.getType() == 1001)
	            		properties.put("topology", "global");
	            	else
	            		properties.put("topology", "local");
		        	feature.put("properties", properties );		
		        	featureList.put(feature);
				}  
				
				
				/****EXTRAS THINGS****/
				
				/***Rescaled Route***/
//				LineString rescaledRoute = dataController.getRescaledRouteGeom();
//				
//
//				ArrayList<Point2D> rescaledRoutePointList  = GeoConvertionsOperations.JTSGeometryToJavaD2(rescaledRoute);
//		        feature = new JSONObject();
//            	feature.put("type", "Feature");
//            	lineString = new JSONArray();
//				for(Point2D pt: rescaledRoutePointList){
//            		JSONArray point = new JSONArray();
//            		point.put(scale*pt.getX());
//                	point.put(scale*pt.getY());
//            		
//                	lineString.put(point);
//            	}
//
//            	geometry = new JSONObject();
//            	geometry.put("type", "LineString");
//            	geometry.put("coordinates", lineString);
//            	
//            	feature.put("geometry", geometry);
//            	
//            	properties = new JSONObject();
//            	properties.put("type", "rescaled-route");
//            	properties.put("name", "rescaled-route");
//            	properties.put("projectedScale", 1/(dataController.getMaxRouteProjectedEnvExtention()));
//	        	//properties.put("color", arearef.getColor());
//	        	feature.put("properties", properties );		
//	        	featureList.put(feature);
//	        	
//	        	/***Transposed Polygons Route***/
//	        	for( Layer l : xMap.getMapContent().getLayers() ) {
//	        		if(  l instanceof PolyLineLayer ) {
//	        			PolyLineLayer pl = (PolyLineLayer)l;
//	        			if(pl.getName().contains("TransPath")) {
//	        				
//	        				
//	        				feature = new JSONObject();
//	                    	feature.put("type", "Feature");
//	                    	lineString = new JSONArray();
//	        				for(Point2D pt: pl.getLines().get(0)){
//	                    		JSONArray point = new JSONArray();
//	                    		point.put(scale*pt.getX());
//	                        	point.put(scale*pt.getY());
//	                    		
//	                        	lineString.put(point);
//	                    	}
//
//	                    	geometry = new JSONObject();
//	                    	geometry.put("type", "LineString");
//	                    	geometry.put("coordinates", lineString);
//	                    	
//	                    	feature.put("geometry", geometry);
//	                    	
//	                    	properties = new JSONObject();
//	                    	properties.put("type", "transpath");
//	                    	properties.put("name", pl.getName());
//	                    	properties.put("projectedScale", 1/(dataController.getMaxRouteProjectedEnvExtention()));
//	        	        	//properties.put("color", arearef.getColor());
//	        	        	feature.put("properties", properties );		
//	        	        	featureList.put(feature);
//	        				
//	        				
//	        			}
//	        			
//	        			
//	        		}
//	        		if(  l instanceof PointLayer ) {
//	        			PointLayer pl = (PointLayer)l;
//	        			
//	        			
//	        		}
//	        	}
				
				
		        
	            featureCollection.put("features", featureList);
	            
	            end = System.currentTimeMillis();
	    		System.out.println("json built time: " + (end - start) );

		    } catch (JSONException e) {
		    	System.out.println("can't save json object: "+e.toString());
		    }
			
			
		   // System.out.println(featureCollection.toString());
			return featureCollection.toString();
		}

		private String buildGeoJsonXSimple(){
			
			
			long start, end;
			JSONObject featureCollection = new JSONObject();
			
			start = System.currentTimeMillis();
		    try {
		        featureCollection.put("type", "FeatureCollection");
		        JSONArray featureList = new JSONArray();
		        // iterate through your list
		        

		
				 JSONObject feature;
				 JSONArray lineString;
				 JSONObject geometry ;
				 JSONObject properties;
		        
		        /**ROUTE ***/
				 
					Path routePath = dataController.getRoute().getRoutePath();
					
					LineString smoothedRoute = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(dataController.getRoute().getRoutePath().asLineString(2), dataController.getRoute().getDPindex(), 0.15, 8), 0.00005) ;
					ArrayList<Point2D> smoothedRoutePointList  = GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedRoute);
			        feature = new JSONObject();
	            	feature.put("type", "Feature");
	            	lineString = new JSONArray();
					for(Point2D pt: smoothedRoutePointList){
	            		JSONArray point = new JSONArray();
	            		point.put(pt.getX());
	                	point.put(pt.getY());
	            		
	                	lineString.put(point);
	            	}

	            	geometry = new JSONObject();
	            	geometry.put("type", "LineString");
	            	geometry.put("coordinates", lineString);
	            	
	            	feature.put("geometry", geometry);
	            	
	            	properties = new JSONObject();
	            	properties.put("type", "route");
	            	properties.put("name", "route");
	            	properties.put("projectedScale", 1/(dataController.getMaxRouteProjectedEnvExtention()));
		        	//properties.put("color", arearef.getColor());
		        	
		        	

		        	feature.put("properties", properties );		
		        	featureList.put(feature);
					
								
		        
		        /**POLYGONAL FEATURES**/
				/****Polygonal Features ******/
				for(PolygonalTopo p:dataController.getPolygonalTopoList()){
					
					p.setKeyPoints();
					ArrayList<Point2D> relevantPtList = new ArrayList<Point2D>();
					ArrayList<Point2D> finalPolygonPtList = new ArrayList<Point2D>();
					ArrayList<Integer> keyPtIndex = new ArrayList<Integer>();
					p.setSmothData(relevantPtList,keyPtIndex);
					if(mainFrame.getCbSmoothLandmarks().isSelected()) {
						
						LineString smoothedPolygon = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList), keyPtIndex, 0.15, 8), 0.00005) ;
						//PolyLineLayer smoothedSectionPolygon = new PolyLineLayer(p.getPolygonalFeature().getName() + "Smoothes" ,  12, 3, Color.GRAY, Color.GRAY, true, true);
						//smoothedSectionPolygon.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon));
						
						//if(p.getPolygonalFeature().getType().equals( "water" ))
						//	smoothedPolygon = Smoother.smoothSpline2(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.2);
						//else
							//smoothedPolygon = Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.15, 8) ;
		
						finalPolygonPtList = GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon);
						
						
					}
					else {
						finalPolygonPtList = relevantPtList;
					}
					
					

				        feature = new JSONObject();
		            	feature.put("type", "Feature");
		            	JSONArray polygon = new JSONArray();
		            	JSONArray linearRing = new JSONArray();
		            	for(Point2D pt: finalPolygonPtList){
		            		JSONArray point = new JSONArray();
		            		if(!Double.isNaN(pt.getX()) &&  !Double.isNaN(pt.getX()) ) {
		            			point.put(pt.getX());
			                	point.put(pt.getY());
		            		}
		            		else
		            			System.out.println("Something strange in polygon smoothed: " +  pt);
		            		
		            		
		            		
		                	linearRing.put(point);
		            	}
		            	polygon.put(linearRing);
		            	
		            	geometry = new JSONObject();
		            	geometry.put("type", "Polygon");
		            	geometry.put("coordinates", polygon);
		            	
		            	feature.put("geometry", geometry);
		            	
		            	properties = new JSONObject();
		            	properties.put("type", p.getPolygonalFeature().getType());
		            	properties.put("name", p.getPolygonalFeature().getName());
			        	
			        	feature.put("properties", properties );		
			        	featureList.put(feature);
		            	
		            	
		           
				

				}
				
				/**ROUTE START***/
				
				feature = new JSONObject();
            	feature.put("type", "Feature");
            	
            	JSONArray startPoint = new JSONArray();/*coordinate point1*/
				
            	startPoint.put(dataController.getRoute().getStart().getxGeom().getX());
            	startPoint.put(dataController.getRoute().getStart().getxGeom().getY());
            	
            	geometry = new JSONObject();
            	geometry.put("type", "Point");
            	geometry.put("coordinates", startPoint);
            	feature.put("geometry", geometry);
            	
            	properties = new JSONObject();
	        	properties.put("type", "route-start");
	        	properties.put("name", "route-start");
	        	properties.put("lat", dataController.getRoute().getStart().getCoordinate().getX());
	        	properties.put("lon", dataController.getRoute().getStart().getCoordinate().getY());
	        	properties.put("edges", String.format("%8s", Integer.toBinaryString((dataController.getRoute().getStart().getEdges() + 256) % 256)).replace(' ', '0') );
	        	
	        	feature.put("properties", properties );		
	        	featureList.put(feature);
	        	
	        	
				feature = new JSONObject();
            	feature.put("type", "Feature");
            	
            	/**ROUTE END***/
            	JSONArray endPoint = new JSONArray();/*coordinate point1*/
				
            	endPoint.put(dataController.getRoute().getEnd().getxGeom().getX());
            	endPoint.put(dataController.getRoute().getEnd().getxGeom().getY());
            	
            	geometry = new JSONObject();
            	geometry.put("type", "Point");
            	geometry.put("coordinates", endPoint);
            	feature.put("geometry", geometry);
            	
            	properties = new JSONObject();
	        	properties.put("type", "route-end");
	        	properties.put("name", "route-end");
	        	properties.put("lat", dataController.getRoute().getEnd().getCoordinate().getX());
	        	properties.put("lon", dataController.getRoute().getEnd().getCoordinate().getY());
	        	properties.put("edges", String.format("%8s", Integer.toBinaryString((dataController.getRoute().getEnd().getEdges() + 256) % 256)).replace(' ', '0') );
	        	
	        	feature.put("properties", properties );		
	        	featureList.put(feature);
				
				/**NODE FEATURES***/
				
				for(StreetNode n: dataController.getStreetNodeMap().values()){
					
					
					if(n.getxGeom()!= null) {
						
						
						if(n.isDecisionPoint()) {
							
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
							point.put(n.getxGeom().getX());
			            	point.put(n.getxGeom().getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "decision-point");
				        	properties.put("name", "decision-point");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						
						
						}
		
						if(n.isRoundAbout()  && n.isRouteNode()){
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
							point.put(n.getxGeom().getX());
			            	point.put(n.getxGeom().getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "round-about");
				        	properties.put("name", "round-about");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						}
					}


				}
		        

		        
	            featureCollection.put("features", featureList);
	            
	            end = System.currentTimeMillis();
	    		System.out.println("json built time: " + (end - start) );

		    } catch (JSONException e) {
		    	System.out.println("can't save json object: "+e.toString());
		    }
			
			
		   // System.out.println(featureCollection.toString());
			return featureCollection.toString();
		}

		
		
		private String buildGeoJsonSimple(){
			
			
			long start, end;
			JSONObject featureCollection = new JSONObject();
			
			start = System.currentTimeMillis();
		    try {
		        featureCollection.put("type", "FeatureCollection");
		        JSONArray featureList = new JSONArray();
		        // iterate through your list
		        

		
				 JSONObject feature;
				 JSONArray lineString;
				 JSONObject geometry ;
				 JSONObject properties;
		        
		        /**ROUTE ***/
				 
					Path routePath = dataController.getRoute().getRoutePath();
					
					LineString smoothedRoute = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(dataController.getRoute().getRoutePath().asLineString(1), dataController.getRoute().getDPindex(), 0.15, 8), 0.00005) ;
					ArrayList<Point2D> smoothedRoutePointList  = GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedRoute);
			        feature = new JSONObject();
	            	feature.put("type", "Feature");
	            	lineString = new JSONArray();
					for(Point2D pt: smoothedRoutePointList){
	            		JSONArray point = new JSONArray();
	            		point.put(pt.getX());
	                	point.put(pt.getY());
	            		
	                	lineString.put(point);
	            	}

	            	geometry = new JSONObject();
	            	geometry.put("type", "LineString");
	            	geometry.put("coordinates", lineString);
	            	
	            	feature.put("geometry", geometry);
	            	
	            	properties = new JSONObject();
	            	properties.put("type", "route");
	            	properties.put("name", "route");
	            	properties.put("projectedScale", 1/(dataController.getMaxRouteProjectedEnvExtention()));
		        	//properties.put("color", arearef.getColor());
		        	
		        	

		        	feature.put("properties", properties );		
		        	featureList.put(feature);
					
								
		        
		        /**POLYGONAL FEATURES**/
				/****Polygonal Features ******/
				for(PolygonalTopo p:dataController.getPolygonalTopoList()){
					
			        feature = new JSONObject();
	            	feature.put("type", "Feature");
	            	JSONArray polygon = new JSONArray();
	            	JSONArray linearRing = new JSONArray();
	            	for(Point2D pt: p.asJava2DList(1)){
	            		JSONArray point = new JSONArray();
	            		point.put(pt.getX());
	                	point.put(pt.getY());
	            		
	                	linearRing.put(point);
	            	}
	            	polygon.put(linearRing);
	            	
	            	geometry = new JSONObject();
	            	geometry.put("type", "Polygon");
	            	geometry.put("coordinates", polygon);
	            	
	            	feature.put("geometry", geometry);
	            	
	            	properties = new JSONObject();
	            	properties.put("type", p.getPolygonalFeature().getType());
	            	properties.put("name", p.getPolygonalFeature().getName());
		        	
		        	feature.put("properties", properties );		
		        	featureList.put(feature);
		            	
		            	
		           
				

				}
				
				/**ROUTE START***/
				
				feature = new JSONObject();
            	feature.put("type", "Feature");
            	
            	JSONArray startPoint = new JSONArray();/*coordinate point1*/
				
            	startPoint.put(dataController.getRoute().getStart().getProjectGeom().getX());
            	startPoint.put(dataController.getRoute().getStart().getProjectGeom().getY());
            	
            	geometry = new JSONObject();
            	geometry.put("type", "Point");
            	geometry.put("coordinates", startPoint);
            	feature.put("geometry", geometry);
            	
            	properties = new JSONObject();
	        	properties.put("type", "route-start");
	        	properties.put("name", "route-start");
	        	properties.put("lat", dataController.getRoute().getStart().getCoordinate().getX());
	        	properties.put("lon", dataController.getRoute().getStart().getCoordinate().getY());
	        	properties.put("edges", String.format("%8s", Integer.toBinaryString((dataController.getRoute().getStart().getEdges() + 256) % 256)).replace(' ', '0') );
	        	
	        	feature.put("properties", properties );		
	        	featureList.put(feature);
	        	
	        	
				feature = new JSONObject();
            	feature.put("type", "Feature");
            	
            	/**ROUTE END***/
            	JSONArray endPoint = new JSONArray();/*coordinate point1*/
				
            	endPoint.put(dataController.getRoute().getEnd().getProjectGeom().getX());
            	endPoint.put(dataController.getRoute().getEnd().getProjectGeom().getY());
            	
            	geometry = new JSONObject();
            	geometry.put("type", "Point");
            	geometry.put("coordinates", endPoint);
            	feature.put("geometry", geometry);
            	
            	properties = new JSONObject();
	        	properties.put("type", "route-end");
	        	properties.put("name", "route-end");
	        	properties.put("lat", dataController.getRoute().getEnd().getCoordinate().getX());
	        	properties.put("lon", dataController.getRoute().getEnd().getCoordinate().getY());
	        	properties.put("edges", String.format("%8s", Integer.toBinaryString((dataController.getRoute().getEnd().getEdges() + 256) % 256)).replace(' ', '0') );
	        	
	        	feature.put("properties", properties );		
	        	featureList.put(feature);
				
				/**NODE FEATURES***/
				
				for(StreetNode n: dataController.getStreetNodeMap().values()){
					
					
					if(n.getxGeom()!= null) {
						
						
						if(n.isDecisionPoint()) {
							
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
							point.put(n.getProjectGeom().getX());
			            	point.put(n.getProjectGeom().getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "decision-point");
				        	properties.put("name", "decision-point");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						
						
						}
		
						if(n.isRoundAbout()  && n.isRouteNode()){
							feature = new JSONObject();
			            	feature.put("type", "Feature");
			            	
			            	JSONArray point = new JSONArray();/*coordinate point1*/
							
							point.put(n.getProjectGeom().getX());
			            	point.put(n.getProjectGeom().getY());
			            	
			            	geometry = new JSONObject();
			            	geometry.put("type", "Point");
			            	geometry.put("coordinates", point);
			            	feature.put("geometry", geometry);
			            	
			            	properties = new JSONObject();
				        	properties.put("type", "round-about");
				        	properties.put("name", "round-about");
				        	properties.put("lat", n.getCoordinate().getY());
				        	properties.put("lon", n.getCoordinate().getX());
				        	properties.put("edges", String.format("%8s", Integer.toBinaryString((n.getEdges() + 256) % 256)).replace(' ', '0') );
				        	
				        	feature.put("properties", properties );		
				        	featureList.put(feature);
						}
					}


				}
		        

		        
	            featureCollection.put("features", featureList);
	            
	            end = System.currentTimeMillis();
	    		System.out.println("json built time: " + (end - start) );

		    } catch (JSONException e) {
		    	System.out.println("can't save json object: "+e.toString());
		    }
			
			
		   // System.out.println(featureCollection.toString());
			return featureCollection.toString();
		}

		
		
		private int getLabelPosition(Point pt, String text, int coordinateType) {
			

			ArrayList<Double> labelBBGradesList = new ArrayList<Double>();
			double labelHeight = 35/dataController.getMaxRouteProjectedEnvExtention();
			double labelWidght = text.length()*labelHeight*0.5;
//			double labelHeight = 0.01;
//			double labelWidght = 0.08;
			
			//System.out.println(text);
//			System.out.println("l height: " + labelHeight);
//			System.out.println("l widht: " + labelWidght);
			for (int i = 0; i < 8; i++) {
				double labelGrade = 0;
				Geometry labelBB = GeoConvertionsOperations.getLabelBB(pt, labelHeight, labelWidght, i );
				
				for(int k = 0; k < dataController.getPathList().size(); k++){
					Path path = dataController.getPathList().get(k);
					/** if is adj edge but not LM  control edge**/
					if(path.isRoute()) {
						if(labelBB.intersects(path.asLineString(coordinateType)))
								labelGrade +=100;
					}	
					else if(path.isRouteAdjEdge() || path.isChunkPath()) {
						if(labelBB.intersects(path.asLineString(coordinateType)))
								labelGrade +=10;
					}	
					else if(path.getIsPolygon() < 1)  {
						if(labelBB.intersects(path.asLineString(coordinateType)))
									labelGrade +=1;
					}
				
				}
				if(labelGrade < 1) {
					//System.out.println("0 Found: " + i);
					return i;
					
				
				}
				//System.out.println("Grade " + i + ": " + labelGrade);
				labelBBGradesList.add(labelGrade);
					
			}
			
			int minGradeIndex = labelBBGradesList.indexOf(Collections.min(labelBBGradesList));
			//System.out.println("Min Grade " + minGradeIndex );
			//System.out.println(" ");
			
			return minGradeIndex;
		}


		private void addOriginalMainLayers() {
			
			
			/** LAyer Deths (double): 1 is top
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

			Path routePath = null;
			int pathIndex = 0;
			
			
			PolyLineLayer routeLayer = new PolyLineLayer( "ROUTE");

			routeLayer.setColor(Color.BLUE);
			routeLayer.setStroke(featureEnhance*3);
			routeLayer.setClosed(false);
			routeLayer.setVisible(true);
			routeLayer.setType(Layer.ORIGINAL_MAIN);
			routeLayer.setLayerDepth(10);
			routeLayer.getLines().add( dataController.getRoute().getRoutePath().asJava2DList(1));
			xMap.getMapContent().getLayers().add(routeLayer);
			


			/*** Add Co-edges ***/  
			int i = 0;
//			for (Path path: dataController.getCoEgedPathList()){
//				i++;
//				xMap.getMapContent().getLayers().add(
//						PolyLineLayer.toLayer(path.getNodeList(),"coPath",i, env)
//						);
//
//			}


			PointLayer routeExtremes = new PointLayer("RouteExtremes");
			routeExtremes.setColor(Color.RED);
			routeExtremes.setFillColor(Color.RED);
			routeExtremes.setSize(featureEnhance*8);
			routeExtremes.setStroke(featureEnhance*1);
			routeExtremes.setVisible(true);
			routeExtremes.setType(Layer.ORIGINAL_MAIN);
			routeExtremes.setLayerDepth(9);
			routeExtremes.getPoints().add(
					new Point2D.Double(dataController.getRoute().getStart().getProjectGeom().getX(),
							dataController.getRoute().getStart().getProjectGeom().getY()));
			routeExtremes.getPoints().add(
					new Point2D.Double(dataController.getRoute().getEnd().getProjectGeom().getX() ,
							 dataController.getRoute().getEnd().getProjectGeom().getY()));

			xMap.getMapContent().getLayers().add(
					routeExtremes
					);

			/**
			 * Add network
			 */
			PolyLineLayer networkEdgesLayer = new PolyLineLayer( "All Network");
			networkEdgesLayer.setColor(Color.GRAY);
			networkEdgesLayer.setStroke(featureEnhance*4);
			networkEdgesLayer.setClosed(false);
			networkEdgesLayer.setVisible(false);
			networkEdgesLayer.setLayerDepth(11);
			
			PolyLineLayer streetNetworkEdges1Layer = new PolyLineLayer( "Street Network1");
			streetNetworkEdges1Layer.setColor(Color.ORANGE);
			streetNetworkEdges1Layer.setStroke(featureEnhance*6);
			streetNetworkEdges1Layer.setClosed(false);
			streetNetworkEdges1Layer.setVisible(false);
			streetNetworkEdges1Layer.setLayerDepth(11);
			//streetNetworkEdges1Layer.setType(Layer.ORIGINAL_MAIN);
			
			
			PolyLineLayer streetNetworkEdges2Layer = new PolyLineLayer( "Street Network2");
			streetNetworkEdges2Layer.setColor(Color.ORANGE);
			streetNetworkEdges2Layer.setStroke(featureEnhance*4);
			streetNetworkEdges2Layer.setClosed(false);
			streetNetworkEdges2Layer.setVisible(false);
			streetNetworkEdges2Layer.setLayerDepth(11);
			//streetNetworkEdges2Layer.setType(Layer.ORIGINAL_MAIN);
			
			PolyLineLayer streetNetworkEdges3Layer = new PolyLineLayer( "Street Network3");
			streetNetworkEdges3Layer.setColor(Color.ORANGE);
			streetNetworkEdges3Layer.setStroke(featureEnhance*3);
			streetNetworkEdges3Layer.setClosed(false);
			streetNetworkEdges3Layer.setVisible(false);
			streetNetworkEdges3Layer.setLayerDepth(11);
			//streetNetworkEdges3Layer.setType(Layer.ORIGINAL_MAIN);
			
			
			PolyLineLayer brigdesLayer = new PolyLineLayer( "Bridge Edges");
			brigdesLayer.setColor(Color.BLACK);
			brigdesLayer.setStroke(featureEnhance*10);
			brigdesLayer.setClosed(false);
			brigdesLayer.setLayerDepth(11);
			brigdesLayer.setVisible(false);
			brigdesLayer.setType(Layer.ORIGINAL_MAIN);
			
			
			
			
			PolyLineLayer linkEdgesLayer = new PolyLineLayer( "Control Edges");
			linkEdgesLayer.setColor(Color.GREEN);
			linkEdgesLayer.setStroke(featureEnhance*6);
			linkEdgesLayer.setClosed(false);
			linkEdgesLayer.setVisible(false);
			linkEdgesLayer.setLayerDepth(11);
			
			
			
			PolyLineLayer adjEdgesLayer = new PolyLineLayer( "Adj. Edges");
			adjEdgesLayer.setColor(Color.GRAY);
			adjEdgesLayer.setStroke(featureEnhance*4);
			adjEdgesLayer.setLayerDepth(11);
			adjEdgesLayer.setClosed(false);
			adjEdgesLayer.setVisible(true);
			adjEdgesLayer.setType(Layer.ORIGINAL_MAIN);
			
//			PolyLineLayer adjEdgesLayer2 = new PolyLineLayer( "Adj. Edges");
//			adjEdgesLayer2.setColor(Color.RED);
//			adjEdgesLayer2.setStroke(featureEnhance*5);
//			adjEdgesLayer2.setLayerDepth(11);
//			adjEdgesLayer2.setClosed(false);
//
//			for(Path p: dataController.getPathList()) {
//				if(p.isRouteAdjEdge())
//					adjEdgesLayer2.getLines().add(PolyLineLayer.pathToLayerLine(p.getNodeList(), env));
//			}
//			xMap.getMapContent().getLayers().add(adjEdgesLayer2);

			
			/**
			Highway1 11
			Highway link 12
			Highway2(trunk) 13
			Highway2(trunk) link 14
			Primary 15
			Primary link 16
			Secondary 21
			Secondary link 22
			Tertiary 31
			Tertiary link 32
			Residential 41
			Unknow 42
			Not classified/Serivice/Rural  43
			Lowpriority(not street)  100
			*/
			
			for(StreetEdge edge: dataController.getStreetNetwork().getEdges().values()){
				if(!edge.getSourcePoint().isDisconnected() && !edge.getTargetPoint().isDisconnected()) {

					if(!edge.isFakeEdge() && 
							((edge.getSourcePoint().isRouteNode() && !edge.getTargetPoint().isRouteNode()) ||
									(!edge.getSourcePoint().isRouteNode() && edge.getTargetPoint().isRouteNode()) ))
						adjEdgesLayer.getLines().add( edge.asPointList(1));
					if(!edge.isFakeEdge() ) {
						if(!(edge.getIsPolygonEdge() > 0)) { 
							switch (edge.getClazz()) {
							case 11:
								streetNetworkEdges1Layer.getLines().add( edge.asPointList(1));
								break;
							case 13:
								streetNetworkEdges1Layer.getLines().add( edge.asPointList(1));
								break;							
							case 15:
								streetNetworkEdges2Layer.getLines().add( edge.asPointList(1));
								break;
							case 21:
								streetNetworkEdges2Layer.getLines().add( edge.asPointList(1));
								break;
							case 31:
								streetNetworkEdges3Layer.getLines().add( edge.asPointList(1));
								break;
							case 41:
								streetNetworkEdges3Layer.getLines().add( edge.asPointList(1));
								break;
							case 43:
								streetNetworkEdges3Layer.getLines().add( edge.asPointList(1));
								break;
							default:
								streetNetworkEdges3Layer.getLines().add( edge.asPointList(1));
								break;
							}


						}
						
						if(edge.getSpecial() == 5)
							brigdesLayer.getLines().add(edge.asPointList(1));
						
						if(edge.getId() == 118749 ) {
							networkEdgesLayer.getLines().add( edge.asPointList(1));
							System.out.println("Do you exists3 new1: " + dataController.getStreetNetwork().getEdges().get(2147483647) );  
							System.out.println("Do you exists3 new2: " + dataController.getStreetNetwork().getEdges().get(2147483646) );
							System.out.println("Do you exists3 old: " + dataController.getStreetNetwork().getEdges().get(118749) );
							
						}	
					}
					else if(edge.isFakeEdge())	
						linkEdgesLayer.getLines().add( edge.asPointList(1));
				}
			}
			xMap.getMapContent().getLayers().add(networkEdgesLayer);
			
			xMap.getMapContent().getLayers().add(streetNetworkEdges1Layer);
			xMap.getMapContent().getLayers().add(streetNetworkEdges2Layer);
			xMap.getMapContent().getLayers().add(streetNetworkEdges3Layer);
			xMap.getMapContent().getLayers().add(brigdesLayer);
			xMap.getMapContent().getLayers().add(adjEdgesLayer);
			xMap.getMapContent().getLayers().add(linkEdgesLayer);

			/******Pointlike Feature*****/
			PointLayer pointLandMarksLayer = new PointLayer("Point Landmarks");
			pointLandMarksLayer.setColor(Color.BLACK);
			routeExtremes.setColor(Color.RED);
			pointLandMarksLayer.setFillColor(Color.BLACK);
			pointLandMarksLayer.setSize(featureEnhance*8);
			pointLandMarksLayer.setStroke(featureEnhance*1);
			pointLandMarksLayer.setVisible(true);
			pointLandMarksLayer.setType(Layer.ORIGINAL_MAIN);
			pointLandMarksLayer.setLayerDepth(9);
			
			
			PolyLineLayer labelPos = new PolyLineLayer( "Label Pos");
			labelPos.setColor(Color.GREEN);
			labelPos.setStroke(featureEnhance*6);
			labelPos.setClosed(false);
			labelPos.setVisible(false);
			labelPos.setLayerDepth(11);
			
			
			for(PointTopo p:dataController.getPointTopoList()) {
				pointLandMarksLayer.getPoints().add(
						new Point2D.Double(p.getNode().getProjectGeom().getX(),
								p.getNode().getProjectGeom().getY()));
				double labelHeight = 35/dataController.getMaxRouteProjectedEnvExtention();
				double labelWidght = p.getPointFeature().getName().length()*labelHeight*0.5;
//				double labelHeight = 0.01;
//				double labelWidght = 0.08;
				
//				System.out.println(p.getPointFeature().getName());
//				System.out.println("l height: " + labelHeight);
//				System.out.println("l widht: " + labelWidght);
				Geometry test;
				for(int m = 0; m<8; m++) {
					test = GeoConvertionsOperations.getLabelBB(p.getNode().getProjectGeom(), labelHeight, labelWidght, m );
					//GeoConvertionsOperations.JTSGeometryToJavaD2(test)
					labelPos.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(test));
					
				}
				
			}
			xMap.getMapContent().getLayers().add(labelPos);
			xMap.getMapContent().getLayers().add(pointLandMarksLayer);
			
			/****Polygonal Features ******/
			for(PolygonalTopo p:dataController.getPolygonalTopoList()){
				p.setKeyPoints();
				PolyLineLayer boundaries = new PolyLineLayer(p.getPolygonalFeature().getName());
				boundaries.setType(Layer.ORIGINAL_MAIN);
				//boundaries.setClosed(true);
				if(p.getPolygonalFeature().getType().equals("urban")){
					boundaries.setColor(Color.GRAY);
					boundaries.setStroke(featureEnhance*1);
					boundaries.setFillColor(new Color(170, 170, 170, 80));
					boundaries.setLayerDepth(14);
				}	
				else if(p.getPolygonalFeature().getType().equals( "park" )){
					boundaries.setColor(Color.GREEN);
					boundaries.setStroke(featureEnhance*2);
					boundaries.setFillColor(new Color(183, 255, 187, 255));
					boundaries.setLayerDepth(13);
				}
				else if(p.getPolygonalFeature().getType().equals( "water" )){
					boundaries.setColor(new Color(149, 188, 216, 255));
					boundaries.setStroke(featureEnhance*2);
					boundaries.setFillColor(new Color(183, 225, 255, 255));
					boundaries.setLayerDepth(13);
				}
				else {
					boundaries.setColor(Color.GRAY);
					boundaries.setStroke(featureEnhance*2);
					boundaries.setFillColor(new Color(170, 170, 170, 50));
					boundaries.setVisible(false);
					boundaries.setLayerDepth(14);
				}
				boundaries.setClosed(true);
				boundaries.getLines().add( p.asJava2DList(1) );
				xMap.getMapContent().getLayers().add(boundaries);

			

			}




			PointLayer routeNodes = new PointLayer("RouteNodes");
			routeNodes.setColor(Color.WHITE);
			routeNodes.setFillColor(Color.WHITE);
			routeNodes.setSize(featureEnhance*5);
			routeNodes.setStroke(featureEnhance*1);
			routeNodes.setLayerDepth(9.4);
			routeNodes.setVisible(false);
			

			PointLayer allStreetNodes = new PointLayer("StreetNodes");
			allStreetNodes.setColor(Color.BLACK);
			allStreetNodes.setFillColor(Color.BLACK);
			allStreetNodes.setSize(featureEnhance*3.5f);
			allStreetNodes.setStroke(featureEnhance*1);
			allStreetNodes.setLayerDepth(9.1);
			allStreetNodes.setVisible(false);
			
			PointLayer relevantNodes = new PointLayer("allRelevantNodes");
			relevantNodes.setColor(Color.MAGENTA);
			relevantNodes.setFillColor(Color.MAGENTA);
			relevantNodes.setSize(featureEnhance*5f);
			relevantNodes.setStroke(featureEnhance*1);
			relevantNodes.setLayerDepth(9.2);
			relevantNodes.setVisible(false);

			PointLayer decisionPoints = new PointLayer("DecisionPoints");
			decisionPoints.setColor(Color.GREEN);
			decisionPoints.setFillColor(Color.GREEN);
			decisionPoints.setSize(featureEnhance*7);
			decisionPoints.setStroke(featureEnhance*1);
			decisionPoints.setLayerDepth(9.3);
			decisionPoints.setType(Layer.ORIGINAL_MAIN);
			decisionPoints.setVisible(true);


			PointLayer crossingPoints = new PointLayer("TopoPoints");
			crossingPoints.setColor(Color.YELLOW);
			crossingPoints.setFillColor(Color.YELLOW);
			crossingPoints.setSize(featureEnhance*8);
			crossingPoints.setStroke(featureEnhance*1);
			crossingPoints.setLayerDepth(9.2);
			crossingPoints.setVisible(false);


			PointLayer non2DegreesPoints = new PointLayer("Non2Degrees");
			non2DegreesPoints.setColor(Color.BLACK);
			non2DegreesPoints.setFillColor(Color.BLACK);
			non2DegreesPoints.setSize(featureEnhance*6);
			non2DegreesPoints.setStroke(featureEnhance*1);
			non2DegreesPoints.setLayerDepth(9.4);
			non2DegreesPoints.setVisible(false);


			PointLayer relevantRotuePoints = new PointLayer("relevantRotuePoints");
			relevantRotuePoints.setColor(Color.MAGENTA);
			relevantRotuePoints.setFillColor(Color.MAGENTA);
			relevantRotuePoints.setSize(featureEnhance*7);
			relevantRotuePoints.setStroke(featureEnhance*1);
			relevantRotuePoints.setLayerDepth(9.3);
			relevantRotuePoints.setVisible(false);
			
			PointLayer roundAbouts = new PointLayer("round abouts");
			roundAbouts.setColor(Color.BLUE);
			roundAbouts.setFillColor(Color.GREEN);
			roundAbouts.setSize(featureEnhance*8);
			roundAbouts.setStroke(featureEnhance*11);
			roundAbouts.setLayerDepth(9.1);
			roundAbouts.setVisible(false);
			roundAbouts.setType(Layer.ORIGINAL_MAIN);




			for(StreetNode n: dataController.getStreetNodeMap().values()){

				if(n.isDecisionPoint())
					decisionPoints.getPoints().add(
							new Point2D.Double(n.getProjectGeom().getX() ,
									 n.getProjectGeom().getY()));

				if(n.getTopoRelations().size() > 0)
					crossingPoints.getPoints().add(
							new Point2D.Double(n.getProjectGeom().getX() ,
									 n.getProjectGeom().getY()));
				if(n.isRouteNode()){
					routeNodes.getPoints().add(
							new Point2D.Double(n.getProjectGeom().getX() ,
									n.getProjectGeom().getY()));
				}

				if(n.getDegree() != 2 ){
					non2DegreesPoints.getPoints().add(
							new Point2D.Double(n.getProjectGeom().getX() ,
									n.getProjectGeom().getY()));
				}
				if(( n.isRoundAbout())){
					if(n.isRouteNode()){
						roundAbouts.setColor(Color.BLUE);
						roundAbouts.setFillColor(Color.GREEN);
					}
					else {
						roundAbouts.setColor(Color.ORANGE);
						roundAbouts.setFillColor(Color.GRAY);
					}
					roundAbouts.getPoints().add(
							new Point2D.Double(n.getProjectGeom().getX() ,
									n.getProjectGeom().getY()));
				}
				if(n.isRelevantRouteNode() ){
					relevantNodes.getPoints().add(
							new Point2D.Double(n.getProjectGeom().getX() ,
									n.getProjectGeom().getY()));
				}

				allStreetNodes.getPoints().add(
						new Point2D.Double(n.getProjectGeom().getX(), n.getProjectGeom().getY()));


			}
			int index = 0;
			for(StreetNode n: dataController.getRoute().asNodeList()){
				if(n.isRelevantRouteNode() ){
					relevantRotuePoints.getPoints().add(
							new Point2D.Double(n.getProjectGeom().getX() ,
									n.getProjectGeom().getY()));
				}
				index++;

			}
			xMap.getMapContent().getLayers().add(allStreetNodes);
			xMap.getMapContent().getLayers().add(relevantRotuePoints);
			xMap.getMapContent().getLayers().add(relevantNodes);
			xMap.getMapContent().getLayers().add(roundAbouts);
			xMap.getMapContent().getLayers().add(decisionPoints);
			xMap.getMapContent().getLayers().add(crossingPoints);
			xMap.getMapContent().getLayers().add(non2DegreesPoints);
			xMap.getMapContent().getLayers().add(routeNodes);
			
			
			
			
			for (Path path: dataController.getPathList()){
				/*OnlyRoute Condition*/
				if(path.isRoute()){
					System.out.println("ROUTE: "  + dataController.getRescaledRouteGeom());

//					OctlinearBoundingBox octBox = new OctlinearBoundingBox(path.asJava2DList(1), 0, 0, 1);
					OctlinearBoundingBox octBox2 = new OctlinearBoundingBox(GeoConvertionsOperations.JTSGeometryToJavaD2(dataController.getRescaledRouteGeom()), 0, 0.05, 2 );
					OctlinearBoundingBox octBox3 = new OctlinearBoundingBox(GeoConvertionsOperations.JTSGeometryToJavaD2(dataController.getRescaledRouteGeom()), 0, 0.05, 0 );
//					PolyLineLayer OBBOXLayer = new PolyLineLayer("RouteOBB" , 8, 2, Color.GREEN, Color.GREEN, false, false);	
//					OBBOXLayer.getLines().add(octBox.getBoundingPolygon());
//					xMap.getMapContent().getLayers().add(OBBOXLayer);
//					
					PolyLineLayer OBBOXLayer2 = new PolyLineLayer("RouteOBBDivBy2 " , 8, 2, Color.BLUE, Color.BLUE, false, false);	
					OBBOXLayer2.getLines().add(octBox2.getBoundingPolygon2());
					xMap.getMapContent().getLayers().add(OBBOXLayer2);
//					
					PolyLineLayer OBBOXLayer3 = new PolyLineLayer("OBBRoute " , 8, 2, Color.RED, Color.RED, false, false);	
					OBBOXLayer3.getLines().add(octBox3.getBoundingPolygon());
					xMap.getMapContent().getLayers().add(OBBOXLayer3);
					routePath = path;

				}
				
				if( !path.isRoute() && !path.isRouteAdjEdge()){
				
					PolyLineLayer networkLayer = new PolyLineLayer("Path" + pathIndex);
					
					if(path.getIsPolygon() > 0)
						networkLayer.setName("PolyPath" + pathIndex);
					else if(path.isChunkPath())
						networkLayer.setColor(Color.RED);
					else {
						networkLayer.setType(Layer.ORIGINAL_MAIN);
						networkLayer.setColor(Color.GRAY);
						
					}
					networkLayer.setStroke(featureEnhance*4);
					networkLayer.setLayerDepth(10);
					if(path.getIsPolygon() > 0)
						networkLayer.setVisible(false);
					else
						networkLayer.setVisible(true);
					networkLayer.getLines().add(path.asJava2DList(1));

					xMap.getMapContent().getLayers().add(
							networkLayer
							);
				}
				if( !path.isRoute() && path.isRouteAdjEdge()){
					
//					PolyLineLayer networkLayer = new PolyLineLayer("StubPath" + pathIndex);
//					
//					if(path.getIsPolygon() > 0)
//						networkLayer.setName("PolyStubPath" + pathIndex);
//					else if(path.isChunkPath())
//						networkLayer.setColor(Color.RED);
//					else {
//						networkLayer.setType(Layer.ORIGINAL_MAIN);
//						networkLayer.setColor(Color.GRAY);
//						
//					}
//					networkLayer.setStroke(featureEnhance*4);
//					networkLayer.setLayerDepth(10);
//					if(path.getIsPolygon() > 0)
//						networkLayer.setVisible(false);
//					else
//						networkLayer.setVisible(true);
//					networkLayer.getLines().add(path.asJava2DList(1));
//
//					xMap.getMapContent().getLayers().add(
//							networkLayer
//							);
				}
				
				pathIndex++;
				

			}
			
			
		}
		
		
		private void addOriginalExtraLayers() {
			
		//	Envelope env = dataController.getRouteEnvelpe();
//			for(PolygonalFeature p:dataController.getPolygonalFeatureList()){
//				if(p.getType() == "buffer"){
//					PolyLineLayer buffer = new PolyLineLayer(p.getName());
//					buffer.setColor(Color.GRAY);
//					buffer.setStroke(featureEnhance*0);
//					buffer.setFillColor(new Color(170, 170, 170, 50));
//					buffer.setClosed(true);
//					buffer.getLines().add( GeoConvertionsOperations.JTSGeometryToJavaD2Normalized(p.getGeom(), env));
//					buffer.setVisible(false);
//					xMap.getMapContent().getLayers().add(buffer);
//				}	
//			}
			
			/*Rescaled Route*/
			
			LineString rescaledRoute = dataController.getRescaledRouteGeom();
			ArrayList<Point2D> rescaledRoutePts = GeoConvertionsOperations.JTSGeometryToJavaD2(rescaledRoute);
			System.out.println("num original de pontos: " + dataController.getRoute().getRoutePath().asLineString(1).getNumPoints() + "  num pontos depoins de rescala: " + rescaledRoute.getNumPoints());
			PolyLineLayer rescaledRouteLayer = new PolyLineLayer("Rescaled Route", 10, 4, Color.RED, Color.RED, false, false);
			rescaledRouteLayer.getLines().add(rescaledRoutePts);
			xMap.getMapContent().getLayers().add( rescaledRouteLayer  );			
			PointLayer decisionPoints = new PointLayer("ReScaledDecisionPoints", 9.3, 1, 7,Color.GREEN , Color.GREEN, false );
			int index = 0;
			for(StreetNode n: dataController.getRoute().getRoutePath().getNodeList()){
				if(n.isDecisionPoint() ){
					decisionPoints.getPoints().add(
							new Point2D.Double(rescaledRoutePts.get(index).getX(),
									rescaledRoutePts.get(index).getY()));
				}
				index++;

			}

			
//			
//			/****Points:  intersections ******/
//			PointLayer ptbufferInterLayer = new  PointLayer("Control Point Anchor");
//			ptbufferInterLayer.setSize(featureEnhance*8);
//			ptbufferInterLayer.setStroke(featureEnhance*1);
//			ptbufferInterLayer.setFillColor(Color.PINK);
//			ptbufferInterLayer.setVisible(false);
//
//			PointLayer ptanchorLayer = new  PointLayer("Anchors");
//			ptanchorLayer.setSize(featureEnhance*8);
//			ptanchorLayer.setStroke(featureEnhance*1);
//			ptanchorLayer.setFillColor(Color.PINK);
//			ptanchorLayer.setVisible(false);
//
//
//			PointLayer ptPolycross = new  PointLayer("Route Polygon Crosses");
//			ptPolycross.setSize(featureEnhance*8);
//			ptPolycross.setStroke(featureEnhance*1);
//			ptPolycross.setFillColor(Color.ORANGE);
//			ptPolycross.setVisible(false);
//
//
//			PointLayer ptPolySelectedcross = new  PointLayer("SelectedRoute Polygon Crosses");
//			ptPolySelectedcross.setSize(featureEnhance*8);
//			ptPolySelectedcross.setStroke(featureEnhance*1);
//			ptPolySelectedcross.setFillColor(Color.MAGENTA);
//			ptPolySelectedcross.setVisible(false);
//
//
//			ClassyPointLayer ptPolyVertices = new  ClassyPointLayer("Polygons vertices start class.");
//			ptPolyVertices.setSize(featureEnhance*4);
//			ptPolyVertices.setStroke(featureEnhance*2);
//			ptPolyVertices.setFillColor(Color.MAGENTA);
//			ptPolyVertices.setVisible(false);
//
//
//			ClassyPointLayer ptPolyVertices2 = new  ClassyPointLayer("Polygons vertices end class.");
//			ptPolyVertices2.setSize(featureEnhance*4);
//			ptPolyVertices2.setStroke(featureEnhance*2);
//			ptPolyVertices2.setFillColor(Color.MAGENTA);
//			ptPolyVertices2.setVisible(false);
//
//
//			for(PointFeature pt:dataController.getPointFeatureList()){
//
//				if(pt.getType() == "ptInter")
//					ptbufferInterLayer.getPoints().add(
//							new Point2D.Double(pt.getGeom().getX() - env.getMinX(),
//									env.getMaxY() - pt.getGeom().getY()));
//				else if(pt.getType() == "anchor")
//					ptanchorLayer.getPoints().add(
//							new Point2D.Double(pt.getGeom().getX() - env.getMinX(),
//									env.getMaxY() - pt.getGeom().getY()));
//				else if(pt.getType() == "cross")
//					ptPolycross.getPoints().add(
//							new Point2D.Double(pt.getGeom().getX() - env.getMinX(),
//									env.getMaxY() - pt.getGeom().getY()));
//				else if(pt.getType() == "selectedcross")
//					ptPolySelectedcross.getPoints().add(
//							new Point2D.Double(pt.getGeom().getX() - env.getMinX(),
//									env.getMaxY() - pt.getGeom().getY()));
//				else if(pt.getType() == "polyvertex1")
//					ptPolyVertices.getPoints().add( new PlotPoint(
//							new Point2D.Double(pt.getGeom().getX() - env.getMinX(),	env.getMaxY() - pt.getGeom().getY()),
//							pt.getColor()
//							)
//
//							);
//				else if(pt.getType() == "polyvertex2")
//					ptPolyVertices2.getPoints().add( new PlotPoint(
//							new Point2D.Double(pt.getGeom().getX() - env.getMinX(),	env.getMaxY() - pt.getGeom().getY()),
//							pt.getColor()
//							)
//
//							);
//
//
//			}
//			xMap.getMapContent().getLayers().add(ptbufferInterLayer);
//			xMap.getMapContent().getLayers().add(ptanchorLayer);
//			xMap.getMapContent().getLayers().add(ptPolycross);
//			xMap.getMapContent().getLayers().add(ptPolySelectedcross);
//			xMap.getMapContent().getLayers().add(ptPolyVertices);
//			xMap.getMapContent().getLayers().add(ptPolyVertices2);
//			xMap.getMapContent().getLayers().add(decisionPoints);
			
			
		}
		
private void addOriginalXRouteLayers() {
			
			PolyLineLayer xRouteLayer = new PolyLineLayer( "XROUTE");

			xRouteLayer.setColor(Color.BLUE);
			xRouteLayer.setStroke(featureEnhance*4);
			xRouteLayer.setClosed(false);
			xRouteLayer.setVisible(false);
			xRouteLayer.setLayerDepth(10);
			xRouteLayer.getLines().add( dataController.getPathList().get(0).asJava2DList(2));


			PolyLineLayer xNetworkEdgesLayer = new PolyLineLayer( "XWholeNetwork");
			xNetworkEdgesLayer.setColor(Color.GRAY);
			xNetworkEdgesLayer.setStroke(featureEnhance*4);
			xNetworkEdgesLayer.setClosed(false);
			xNetworkEdgesLayer.setLayerDepth(11);
			xNetworkEdgesLayer.setVisible(false);
			
			
			PolyLineLayer xStreetNetworkEdges1Layer = new PolyLineLayer( "XStreetNetwork1");
			xStreetNetworkEdges1Layer.setColor(Color.GRAY);
			xStreetNetworkEdges1Layer.setStroke(featureEnhance*6);
			xStreetNetworkEdges1Layer.setClosed(false);
			xStreetNetworkEdges1Layer.setLayerDepth(11);
			xStreetNetworkEdges1Layer.setType(Layer.SCHEMATIC_MAIN);
			xStreetNetworkEdges1Layer.setVisible(false);
			
			PolyLineLayer xStreetNetworkEdges2Layer = new PolyLineLayer( "XStreetNetwork2");
			xStreetNetworkEdges2Layer.setColor(Color.GRAY);
			xStreetNetworkEdges2Layer.setStroke(featureEnhance*4);
			xStreetNetworkEdges2Layer.setClosed(false);
			xStreetNetworkEdges2Layer.setLayerDepth(11);
			xStreetNetworkEdges2Layer.setType(Layer.SCHEMATIC_MAIN);
			xStreetNetworkEdges2Layer.setVisible(false);
			
			PolyLineLayer xStreetNetworkEdges3Layer = new PolyLineLayer( "XStreetNetwork3");
			xStreetNetworkEdges3Layer.setColor(Color.GRAY);
			xStreetNetworkEdges3Layer.setStroke(featureEnhance*3);
			xStreetNetworkEdges3Layer.setClosed(false);
			xStreetNetworkEdges3Layer.setLayerDepth(11);
			xStreetNetworkEdges3Layer.setType(Layer.SCHEMATIC_MAIN);
			xStreetNetworkEdges3Layer.setVisible(false);


			PolyLineLayer adjEdgesLayerX = new PolyLineLayer( "XAllAdjcentEdges");
			adjEdgesLayerX.setColor(Color.GRAY);
			adjEdgesLayerX.setStroke(featureEnhance*4);
			adjEdgesLayerX.setLayerDepth(11);
			adjEdgesLayerX.setClosed(false);
			adjEdgesLayerX.setVisible(false);

			PolyLineLayer fakeAdjEdgesLayerX = new PolyLineLayer( "XControlEdges");
			fakeAdjEdgesLayerX.setColor(Color.MAGENTA);
			fakeAdjEdgesLayerX.setStroke(featureEnhance*5);
			fakeAdjEdgesLayerX.setLayerDepth(11);
			fakeAdjEdgesLayerX.setClosed(false);
			fakeAdjEdgesLayerX.setVisible(false);


			for(StreetEdge edge: dataController.getStreetNetwork().getEdges().values()){
				if(!edge.getSourcePoint().isDisconnected() && !edge.getTargetPoint().isDisconnected()) {
					if(edge.getSourcePoint().getxGeom()!= null && edge.getTargetPoint().getxGeom()!= null){
						if(!edge.isFakeEdge() && 
								((edge.getSourcePoint().isRouteNode() && !edge.getTargetPoint().isRouteNode()) ||
										(!edge.getSourcePoint().isRouteNode() && edge.getTargetPoint().isRouteNode()) ))
							adjEdgesLayerX.getLines().add( edge.asPointList(2));
						if(!edge.isFakeEdge() && !(edge.getSourcePoint().isRouteNode() && edge.getTargetPoint().isRouteNode())) {
							if(!(edge.getIsPolygonEdge()>0)){ 
								switch (edge.getClazz()) {
								case 11:
									xStreetNetworkEdges1Layer.getLines().add( edge.asPointList(2));
									break;
								case 13:
									xStreetNetworkEdges1Layer.getLines().add( edge.asPointList(2));
									break;							
								case 15:
									xStreetNetworkEdges2Layer.getLines().add( edge.asPointList(2));
									break;
								case 21:
									xStreetNetworkEdges2Layer.getLines().add( edge.asPointList(2));
									break;
								case 31:
									xStreetNetworkEdges3Layer.getLines().add( edge.asPointList(2));
									break;
								case 41:
									xStreetNetworkEdges3Layer.getLines().add( edge.asPointList(2));
									break;
								case 43:
									xStreetNetworkEdges3Layer.getLines().add( edge.asPointList(2));
									break;
								default:
									xStreetNetworkEdges3Layer.getLines().add( edge.asPointList(2));
									break;
								}


							}					
							xNetworkEdgesLayer.getLines().add( edge.asPointList(2));	
						}
						else if(edge.isFakeEdge() )
							fakeAdjEdgesLayerX.getLines().add( edge.asPointList(2));
					}
				}
			}
			
			PolyLineLayer adjEdgesLayerX2 = new PolyLineLayer( "XStreetAdjcentEdges");
			adjEdgesLayerX2.setColor(Color.GRAY);
			adjEdgesLayerX2.setStroke(featureEnhance*4);
			adjEdgesLayerX2.setLayerDepth(11);
			adjEdgesLayerX2.setClosed(false);
			adjEdgesLayerX2.setVisible(true);
			adjEdgesLayerX2.setType(Layer.SCHEMATIC_MAIN);
			for (Path path: dataController.getRouteAdjPathList()){
				if(path.getIsPolygon() == 0)
				adjEdgesLayerX2.getLines().add(path.asJava2DList(2));
				
			}
			

			/***Smooth***/
			//					LineString smotthedLine = Smoother.smoothSpline2(dataController.getRoute().getxGeom(), dataController.getRoute().getDPindex(), 0.3);
			//			    	PolyLineLayer routeLineString = new PolyLineLayer("LineString Route");
			//			    	routeLineString.setColor(Color.RED);
			//			    	routeLineString.setStroke(featureEnhance*2);
			//		    		routeLineString.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(smotthedLine));
			//		    		xMap.getMapContent().getLayers().add( routeLineString  );

			
			//LineString simplifiedPolygon = (LineString) DouglasPeuckerSimplifier.simplify(p.getGeom().getExteriorRing(), tolerance);

			LineString smotthedLine = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(dataController.getRoute().getRoutePath().asLineString(2), dataController.getRoute().getDPAnd2DegreeIndex(), 0.15, 8),0.00005) ;
			PolyLineLayer xSmothhedRouteLayer = new PolyLineLayer("XSmoothedRoute");
			xSmothhedRouteLayer.setColor(Color.BLUE);
			xSmothhedRouteLayer.setStroke(featureEnhance*3);
			xSmothhedRouteLayer.setLayerDepth(9);
			xSmothhedRouteLayer.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(smotthedLine));
			xSmothhedRouteLayer.setType(Layer.SCHEMATIC_MAIN);
			xMap.getMapContent().getLayers().add( xSmothhedRouteLayer  );




			xMap.getMapContent().getLayers().add(adjEdgesLayerX);
			xMap.getMapContent().getLayers().add(adjEdgesLayerX2);
			xMap.getMapContent().getLayers().add(fakeAdjEdgesLayerX);
			xMap.getMapContent().getLayers().add(xRouteLayer);
			xMap.getMapContent().getLayers().add(xNetworkEdgesLayer);
			xMap.getMapContent().getLayers().add(xStreetNetworkEdges1Layer);
			xMap.getMapContent().getLayers().add(xStreetNetworkEdges2Layer);
			xMap.getMapContent().getLayers().add(xStreetNetworkEdges3Layer);
			
			
			
			PointLayer routeExtremes = new PointLayer("XRouteExtremes");
			routeExtremes.setColor(Color.RED);
			routeExtremes.setFillColor(Color.RED);
			routeExtremes.setSize(featureEnhance*8);
			routeExtremes.setStroke(featureEnhance*1);
			routeExtremes.setVisible(true);
			routeExtremes.setType(Layer.SCHEMATIC_MAIN);
			routeExtremes.setLayerDepth(9);
			routeExtremes.getPoints().add(
					new Point2D.Double(dataController.getRoute().getStart().getxGeom().getX() ,
							 dataController.getRoute().getStart().getxGeom().getY()));
			routeExtremes.getPoints().add(
					new Point2D.Double(dataController.getRoute().getEnd().getxGeom().getX(),
							 dataController.getRoute().getEnd().getxGeom().getY()));

			xMap.getMapContent().getLayers().add(
					routeExtremes
					);
			
			
			
			PointLayer routeNodes = new PointLayer("XRouteNodes");
			routeNodes.setColor(Color.WHITE);
			routeNodes.setFillColor(Color.WHITE);
			routeNodes.setSize(featureEnhance*5);
			routeNodes.setStroke(featureEnhance*1);
			routeNodes.setLayerDepth(9);
			routeNodes.setVisible(false);

			PointLayer decisionPoints = new PointLayer("XDecisionPoints");
			decisionPoints.setColor(Color.GREEN);
			decisionPoints.setFillColor(Color.GREEN);
			decisionPoints.setSize(featureEnhance*8);
			decisionPoints.setStroke(featureEnhance*1);
			decisionPoints.setLayerDepth(9);
			decisionPoints.setVisible(true);
			decisionPoints.setType(Layer.SCHEMATIC_MAIN);

			PointLayer non2DegreesPoints = new PointLayer("XNon2Degrees");
			non2DegreesPoints.setColor(Color.BLUE);
			non2DegreesPoints.setFillColor(Color.BLUE);
			non2DegreesPoints.setSize(featureEnhance*7);
			non2DegreesPoints.setStroke(featureEnhance*1);
			non2DegreesPoints.setLayerDepth(9.1);
			non2DegreesPoints.setVisible(false);

			PointLayer crossingPoints = new PointLayer("XTopoPoints");
			crossingPoints.setColor(Color.YELLOW);
			crossingPoints.setFillColor(Color.YELLOW);
			crossingPoints.setSize(featureEnhance*9);
			crossingPoints.setStroke(featureEnhance*1);
			crossingPoints.setLayerDepth(9);
			crossingPoints.setVisible(false);

			PointLayer relevantRotuePoints = new PointLayer("XrelevantRotuePoints");
			relevantRotuePoints.setColor(Color.MAGENTA);
			relevantRotuePoints.setFillColor(Color.MAGENTA);
			relevantRotuePoints.setSize(featureEnhance*6);
			relevantRotuePoints.setStroke(featureEnhance*1);
			relevantRotuePoints.setLayerDepth(9);
			relevantRotuePoints.setVisible(false);
			
			
			PointLayer allStreetNodes = new PointLayer("XAllNodes");
			allStreetNodes.setColor(Color.BLACK);
			allStreetNodes.setFillColor(Color.BLACK);
			allStreetNodes.setSize(featureEnhance*4);
			allStreetNodes.setStroke(featureEnhance*1);
			allStreetNodes.setLayerDepth(9.1);
			allStreetNodes.setVisible(false);
			
			PointLayer roundAbouts = new PointLayer("XRoundAbouts");
			roundAbouts.setColor(Color.BLUE);
			roundAbouts.setFillColor(Color.GREEN);
			roundAbouts.setSize(featureEnhance*8);
			roundAbouts.setStroke(featureEnhance*11);
			roundAbouts.setLayerDepth(9.1);
			roundAbouts.setVisible(false);
			roundAbouts.setType(Layer.SCHEMATIC_MAIN);


			for(StreetNode n: dataController.getStreetNodeMap().values()){

				if(n.getxGeom()!= null) {
					allStreetNodes.getPoints().add(
							new Point2D.Double(n.getxGeom().getX(),
									n.getxGeom().getY()));
					
					if(n.isDecisionPoint())
						decisionPoints.getPoints().add(
								new Point2D.Double(n.getxGeom().getX(),
										n.getxGeom().getY()));
	
					if(n.getTopoRelations().size() > 0 )
						crossingPoints.getPoints().add(
								new Point2D.Double(n.getxGeom().getX(),
										n.getxGeom().getY()));
					if(n.isRouteNode()){
						routeNodes.getPoints().add(
								new Point2D.Double(n.getxGeom().getX() ,
										n.getxGeom().getY()));
					}
	
					if(n.getDegree() != 2 ){
						non2DegreesPoints.getPoints().add(
								new Point2D.Double(n.getxGeom().getX() ,
										n.getxGeom().getY()));
					}
					if(n.isRelevantRouteNode()){
						relevantRotuePoints.getPoints().add(
								new Point2D.Double(n.getxGeom().getX() ,
										n.getxGeom().getY()));
					}
					if(n.isRoundAbout() ){
						
						if(n.isRouteNode()){
							roundAbouts.setColor(Color.BLUE);
							roundAbouts.setFillColor(Color.GREEN);
						}
						else {
							roundAbouts.setColor(Color.ORANGE);
							roundAbouts.setFillColor(Color.GRAY);
						}
						roundAbouts.getPoints().add(
								new Point2D.Double(n.getxGeom().getX(),
										n.getxGeom().getY()));
					}
				}


			}
			xMap.getMapContent().getLayers().add(relevantRotuePoints);
			/* addint to schematize route*/
			xMap.getMapContent().getLayers().add(crossingPoints);
			xMap.getMapContent().getLayers().add(decisionPoints);
			xMap.getMapContent().getLayers().add(non2DegreesPoints);
			xMap.getMapContent().getLayers().add(roundAbouts);
			xMap.getMapContent().getLayers().add(routeNodes);
			xMap.getMapContent().getLayers().add(allStreetNodes);
			
			/******Pointlike Feature*****/
			PointLayer pointLandMarksLayer = new PointLayer("XPointLandmarks");
			pointLandMarksLayer.setColor(Color.BLACK);
			pointLandMarksLayer.setFillColor(Color.BLACK);
			pointLandMarksLayer.setSize(featureEnhance*8);
			pointLandMarksLayer.setStroke(featureEnhance*2);
			pointLandMarksLayer.setVisible(true);
			pointLandMarksLayer.setType(Layer.SCHEMATIC_MAIN);
			pointLandMarksLayer.setLayerDepth(9);
			
			for(PointTopo p:dataController.getPointTopoList()) {
				pointLandMarksLayer.getPoints().add(
						new Point2D.Double(p.getNode().getxGeom().getX(),
								p.getNode().getxGeom().getY()));
				
			}
			xMap.getMapContent().getLayers().add(pointLandMarksLayer);
			
			
			/**Add smoothed network paths*/
			int pathIndex = 0;
			for (Path path: dataController.getStreetOnlyPathList()){
				/*OnlyRoute Condition*/
//				if(path.isRoute()){
////					routePath = path;
////					PolyLineLayer routeLayer = new PolyLineLayer("Route");
////					routeLayer.setColor(Color.BLUE);
////					routeLayer.setStroke(featureEnhance*3);
////					routeLayer.setLayerDepth(10);
////					routeLayer.getLines().add(PolyLineLayer.pathToLayerLine(routePath.getNodeList(), env));
////
////					xMap.getMapContent().getLayers().add(
////							routeLayer
////							);
//				}
				
				if(   !(path.getIsPolygon()>0)){
					ArrayList<Path> pathClazzSubdivision = path.divideByClazzChange(dataController.getStreetNetwork());
					for(Path subPath: pathClazzSubdivision) {
						PolyLineLayer networkLayer = new PolyLineLayer("XPath" + pathIndex);
						
						if( subPath.getClazz() == 11 || subPath.getClazz() == 13  )
							networkLayer.setStroke(featureEnhance*8);
						else if( subPath.getClazz() == 15 || subPath.getClazz() == 21  )
							networkLayer.setStroke(featureEnhance*6);
						else
							networkLayer.setStroke(featureEnhance*3);
						
						
						networkLayer.setColor(Color.GRAY);
						//networkLayer.setStroke(featureEnhance*4);
						networkLayer.setType(Layer.SCHEMATIC_MAIN);
						networkLayer.setLayerDepth(10);
						networkLayer.setVisible(true);
						
						
						ArrayList<Point2D> relevantPtList = new ArrayList<Point2D>();
						ArrayList<Integer> keyPtIndex = new ArrayList<Integer>();
						subPath.setSmothData(relevantPtList,keyPtIndex,2);
						if(mainFrame.getCbSmoothNetwork().isSelected() && relevantPtList.size() > 1 ) {					
							LineString smoothedPath = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList), keyPtIndex, 0.15, 8), 0.00005) ;
							//PolyLineLayer smoothedSectionPolygon = new PolyLineLayer(p.getPolygonalFeature().getName() + "Smoothes" ,  12, 3, Color.GRAY, Color.GRAY, true, true);
							//smoothedSectionPolygon.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon));
							
							//smoothedPath = Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.15, 8) ;
			
							networkLayer.getLines().add( GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPath));
						}
						else {
							networkLayer.getLines().add( relevantPtList );
						}
						xMap.getMapContent().getLayers().add(networkLayer);
					
					}
//					
//					networkLayer.getLines().add(path.asJava2DList(2));
//
//					xMap.getMapContent().getLayers().add(
//							networkLayer
//							);
				}
				pathIndex++;
				

			}
			
			
			/****Polygonal Features ******/
			for(PolygonalTopo p:dataController.getPolygonalTopoList()){
				p.setKeyPoints();
				PolyLineLayer boundaries = new PolyLineLayer("X"+p.getPolygonalFeature().getName());
				boundaries.setType(Layer.SCHEMATIC_MAIN);
				//boundaries.setClosed(true);
				if(p.getPolygonalFeature().getType().equals("urban")){
					boundaries.setColor(Color.GRAY);
					boundaries.setStroke(featureEnhance*1);
					boundaries.setFillColor(new Color(170, 170, 170, 80));
					boundaries.setLayerDepth(14);
				}	
				else if(p.getPolygonalFeature().getType().equals( "park" )){
					boundaries.setColor(Color.GREEN);
					boundaries.setStroke(featureEnhance*2);
					boundaries.setFillColor(new Color(183, 255, 187, 255));
					boundaries.setLayerDepth(13);
				}
				else if(p.getPolygonalFeature().getType().equals( "water" )){
					boundaries.setColor(new Color(149, 188, 216, 255));
					boundaries.setStroke(featureEnhance*2);
					boundaries.setFillColor(new Color(183, 225, 255, 255));
					boundaries.setLayerDepth(13);
				}
				else {
					boundaries.setColor(Color.GRAY);
					boundaries.setStroke(featureEnhance*2);
					boundaries.setFillColor(new Color(170, 170, 170, 50));
					boundaries.setVisible(false);
					boundaries.setLayerDepth(14);
				}
				boundaries.setClosed(true);
				
				ArrayList<Point2D> relevantPtList = new ArrayList<Point2D>();
				ArrayList<Integer> keyPtIndex = new ArrayList<Integer>();
				p.setSmothData(relevantPtList,keyPtIndex);
				if(mainFrame.getCbSmoothLandmarks().isSelected()) {					
					LineString smoothedPolygon = (LineString) DouglasPeuckerSimplifier.simplify(Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList), keyPtIndex, 0.15, 8), 0.00005) ;
					//PolyLineLayer smoothedSectionPolygon = new PolyLineLayer(p.getPolygonalFeature().getName() + "Smoothes" ,  12, 3, Color.GRAY, Color.GRAY, true, true);
					//smoothedSectionPolygon.getLines().add(GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon));
					
					//if(p.getPolygonalFeature().getType().equals( "water" ))
					//smoothedPolygon = Smoother.smoothSpline2(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.2);
					//else
					//smoothedPolygon = Smoother.geometrySmoothCut(GeoConvertionsOperations.Java2DToJTSLineString(relevantPtList),	keyPtIndex, 0.15, 8) ;
	
					boundaries.getLines().add( GeoConvertionsOperations.JTSGeometryToJavaD2(smoothedPolygon));
				}
				else {
					boundaries.getLines().add( p.asJava2DList(2) );
				}
				xMap.getMapContent().getLayers().add(boundaries);

			

			}
			
			
		}

		
	}
	
	
	
	
	
	class LayerTableListener implements MouseListener{


		public void mouseClicked(MouseEvent e) {
			int r = mainFrame.getLayerTable().rowAtPoint(e.getPoint());
			int c = mainFrame.getLayerTable().columnAtPoint(e.getPoint());
			if(c==1){
				xMap.getMapContent().toogleVisibility(r);
				layerTableModel.fireTableCellUpdated(r, c);
				xMap.repaint();
			}

		}

		public void mousePressed(MouseEvent e) {
			// TODO Auto-generated method stub

		}

		public void mouseReleased(MouseEvent e) {
			// TODO Auto-generated method stub

		}

		public void mouseEntered(MouseEvent e) {
			// TODO Auto-generated method stub

		}

		public void mouseExited(MouseEvent e) {
			// TODO Auto-generated method stub

		}

	}
	
	public MainFrame getMainFrame() {
		return mainFrame;
	}
	

}
