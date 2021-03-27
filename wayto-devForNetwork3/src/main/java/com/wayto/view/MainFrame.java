package com.wayto.view;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.GridLayout;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFormattedTextField;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.JTabbedPane;
import javax.swing.JTable;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.plaf.metal.MetalBorders;










public class MainFrame extends JPanel{


	
	private JTabbedPane tpCenter; /*Painel de Abas*/
	
	/*routeControls*/
	private JSlider sliderBendFactor, sliderCrossBendFactor, sliderDistFactor, sliderEdgeOrientation, sliderProportionDP, sliderProportionAllPts, sliderScaleVariation, sliderScaleVariationRaiseFactor;
	private JFormattedTextField textFieldExecutionTimeLimit, textFieldMinAdjDist, textFieldMinNonAdjDist, textFieldStubFixDist, textFieldLocalLMFixDist, textFieldOrthoLevel, textFieldTurnOrthoLevel;
	
	
	private JComboBox<String> cbDirectionModel, cbRescalePoints;
	private JCheckBox  cbCheckRouteTopology;
	
	/*regionControls*/
	private JSlider sliderBendFactorRegion, sliderDistFactorRegion, sliderEdgeOrientationRegion, sliderProportionRegion, sliderAlongness, sliderTranfsSimplification;
	
	private JFormattedTextField textFieldExecutionTimeLimitRegion, textFieldMinDistToRoute;
	private JCheckBox cbXIncorrects, cbXall;
	private JCheckBox cbCheckSelfTopology, cbCheckTopology, cbSmoothLandmarks;
	
	
	/*StreetNetworkControls*/
	private JComboBox<String> cbStreetNetwork;
	private JSlider sliderBendFactorNetwork, sliderDistFactorNetwork, sliderEdgeOrientationNetwork, sliderProportionNetwork,  sliderTranfsSimplificationNetwork;
	private JFormattedTextField textFieldExecutionTimeLimitNetwork;
	private JCheckBox cbReduceStubs,cbXIncorrectsNetwork, cbXallNetwork;
	private JCheckBox cbCheckSelfTopologyNetwork, cbCheckTopologyNetwork, cbSmoothNetwork;


	/* Componentes de intera��o */

	/*Load from databse*/
	private JButton bLoadRoute;

	private JComboBox<String> cbSource, cbDestination;
	
	private JFormattedTextField textFieldSource, textFieldDestination;
	
	

	/*load from shapefile*/
	private JButton bValidateGeometry;
	private JButton bOpenShapeFile;

	
	/* Componentes de apresenta��o da informa��o */
	
	private JTable layerTable ;

	JTextArea taPosicao;
	private JCheckBox cbShowAllLayers;
	private JCheckBox cbToggleXOLayers;
	


	
	private JButton bExportShape;
	private JButton bExportGeoJson;
	private JButton bReportFiles;
	private JButton bSchematize;
	
	
	public MainFrame() {

//		super("Mapa Esquematico");
//		this.setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE);
//		this.setSize(900, 600);
//		this.setExtendedState(this.getExtendedState()|JFrame.MAXIMIZED_BOTH);
		
		/* Main panels*/
		JPanel pSouth, pNorth, pCenter, pWest, pWest1, pWest2, pWest3, pWest4, pEast, pNorth1, pNorth2, pNorth3;
		
		
		/*Controls Panels*/
		JPanel  pControl, pControlGeneral, pControlRoute, pControlNetwork, pControlRegion;
		
		
		
		/* Adiciona componetes ao painel pNorth1 */


		pNorth1 = new JPanel();
		pNorth1.setLayout(new FlowLayout());
		pNorth1.setBorder(new MetalBorders.Flush3DBorder());
		/* Adiciona componetes ao painel pNorth2 */



		pNorth2 = new JPanel();
		pNorth2.setLayout(new FlowLayout());
		pNorth2.setBorder(new MetalBorders.Flush3DBorder());



		/* Adiciona componetes ao painel pNorth */

		pNorth = new JPanel();
		pNorth.setLayout( new BorderLayout() );
		pNorth.setBorder( new MetalBorders.Flush3DBorder() );
		
		//pNorth.add(pNorth1);
		//pNorth.add(pNorth2);
		//pNorth.add(pNorth3);

		/* Adiciona componetes ao pWest */
		
		/* Adiciona componetes ao painel pWest1 */
		

		/*Coesfeld, Geo1, Telgte, institute of sociology, Greven */
		String[] lacations = { "51.756351, 8.039380, Beckum",  "51.759886, 7.907797, Ahlen", "51.77414307, 7.17605781, Haltern Nord", "51.706887, 7.193698, Haltern Sud", "51.74346956249209, 7.185658400022311, Haltern Center", "51.767347, 7.897526, Ahlen2", "51.758348, 7.901508, Ahlen2.5", "51.790968, 7.911588, Out Ahlen", "51.739854, 7.877066, Ahlen3","51.756001, 7.885407, Ahlen3.1",  "51.770922, 8.039127, Beckum2", "51.7550790, 8.0427605, Beckum2.5", "51.746769, 8.045381, Beckum2.6", "51.753091, 8.040308, Beckum3", "51.770913, 7.460567, Luedingenhausen","51.969514, 7.670296, test3", "51.973454, 7.693132, test4", "51.945615, 7.169686, Coesfeld", "51.969748, 7.591509, GEO2", "51.968832, 7.595635, GEO1", "51.766295916, 7.96566661, testa", "51.7623111279, 7.9888497714, testb", "51.971783, 7.618972, testc", "51.969719, 7.622258, testd", "51.780154, 7.486016, texte", "51.786821, 7.541262, testf", "51.985403, 7.780783, Telgte", "51.950937, 7.609571, AaseeViertel" , "52.092090, 7.611286, Greven", "51.921248, 7.620748, Angela" , "51.968063, 7.624503, Home",  "51.954893, 7.630217, HauptBahnhof" };
	  //String[] lacations = {"51.732573, 7.415138, exp5",  "51.741365, 7.246723, exp4", "51.871761, 7.492952, exp3","51.787335, 7.616613, exp1", "51.652766, 7.538049, exp2", "51.756351, 8.039380, Beckum",  "51.759886, 7.907797, Ahlen", "51.767347, 7.897526, Ahlen2", "51.739854, 7.877066, Ahlen3", "51.969514, 7.670296, test3", "51.973454, 7.693132, test4", "51.945615, 7.169686, Coesfeld", "51.968832, 7.595635, GEO1", "51.909491, 7.399813, test", "51.980797, 7.631839, test2", "51.999059, 7.569633, test3", "51.985403, 7.780783, Telgte", "51.950937, 7.609571, AaseeViertel" , "52.092090, 7.611286, Greven", "51.921248, 7.620748, Angela" , "51.968063, 7.624503, Home",  "51.954893, 7.630217, HauptBahnhof" };

		
			

		
		cbSource = new JComboBox<String>(lacations);
		cbSource.setSelectedIndex(7);
		cbDestination = new JComboBox<String>(lacations);
		cbDestination.setSelectedIndex(8);
		
		
		textFieldSource = new JFormattedTextField();
		textFieldSource.setText("");
		textFieldSource.setColumns(20);
		
		textFieldDestination = new JFormattedTextField();
		textFieldDestination.setText("");
		textFieldDestination.setColumns(20);
		
		bLoadRoute= new JButton("Load Route");

		
		
		pWest1 = new JPanel();
		pWest1.setLayout(new FlowLayout(FlowLayout.CENTER));
		pWest1.setBorder(new MetalBorders.Flush3DBorder());
		pWest1.add( cbSource );
		pWest1.add( textFieldSource );
		pWest1.add( cbDestination );
		pWest1.add( textFieldDestination );
		pWest1.add( bLoadRoute );

		
		
		
		bValidateGeometry = new JButton("Validade Geometry");
		bExportShape = new JButton("Export Shape File");
		bSchematize = new JButton("Esquematizar");
		bOpenShapeFile = new JButton("Abrir Shape");
		
		pWest2 = new JPanel();
		pWest2.setLayout(new FlowLayout(FlowLayout.CENTER));
		pWest2.setBorder(new MetalBorders.Flush3DBorder());
		pWest2.add( bOpenShapeFile );
		pWest2.add( bValidateGeometry );

		
		

		/* Adiciona componetes ao painel pWest2 */
		/* rotina para acesso a banco de dados */
		

		
		layerTable = new JTable();
		layerTable.setPreferredScrollableViewportSize(new Dimension(250, 70));		
		JScrollPane tableScrollPane = new JScrollPane(layerTable);
		
		
		pWest3 = new JPanel();
		pWest3.setLayout( new BorderLayout() );
		pWest3.setBorder(new MetalBorders.Flush3DBorder());
		pWest3.add( tableScrollPane , BorderLayout.CENTER );
		
		
		bValidateGeometry = new JButton("Validade Geometry");
		bExportShape = new JButton("Export Shape File");
		bExportGeoJson = new JButton("Export GeoJson");
		bSchematize = new JButton("Esquematizar");
		cbShowAllLayers = new JCheckBox("Show all", true);
		cbToggleXOLayers = new JCheckBox("Toggle", true);
		bReportFiles = new JButton("Report");
		pWest4 = new JPanel();
		pWest4.setLayout(new FlowLayout(FlowLayout.CENTER));
		pWest4.setBorder(new MetalBorders.Flush3DBorder());
		pWest4.add( bSchematize );
		pWest4.add( bExportShape );
		pWest4.add( bExportGeoJson );
		pWest4.add( cbToggleXOLayers );
		pWest4.add( cbShowAllLayers );
		pWest4.add( bReportFiles );

		
		
		
		

		pWest = new JPanel();
		pWest.setPreferredSize( new Dimension(300, 2000));
		pWest.setLayout( new GridLayout(4,1) );
		pWest.add( pWest1 );
		pWest.add( pWest2 );
		pWest.add( pWest3 );
		pWest.add( pWest4 );
		
		/*Adiciona componentes ao pCenter*/
		pControl = new JPanel();
		pControl.setLayout( new GridLayout(1,3) );

		
		pControlGeneral = new JPanel();
		pControlGeneral.setLayout(new FlowLayout(FlowLayout.CENTER));
		pControlGeneral.setBorder(new MetalBorders.Flush3DBorder());
		
		
		/****CONTROL ROUTE*****/
		pControlRoute = new JPanel();
		pControlRoute.setLayout(new GridLayout(0,1));
		pControlRoute.setBorder(new MetalBorders.Flush3DBorder());
		
		JPanel routeTitelPanel = new JPanel(new FlowLayout(FlowLayout.CENTER));
		JLabel textAreaRouteTitel = new JLabel("Route Parameters");
		routeTitelPanel.add(textAreaRouteTitel);
		
		JPanel routeGeneralPanel2 = new JPanel(new FlowLayout(FlowLayout.LEFT));
		cbCheckRouteTopology = new JCheckBox("Check Topology", true);
		routeGeneralPanel2.add(cbCheckRouteTopology);
		
		JPanel routeGeneralPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		textFieldMinAdjDist = new JFormattedTextField();
		textFieldMinAdjDist.setText("1");
		JLabel textAreaMinAdjDistl = new JLabel(" Min.dist(adj):");
		routeGeneralPanel.add(textAreaMinAdjDistl);
		routeGeneralPanel.add(textFieldMinAdjDist);
		
		textFieldMinNonAdjDist = new JFormattedTextField();
		textFieldMinNonAdjDist.setText("3");
		JLabel textMinNonAdjDist = new JLabel(" Min.dist(non adj):");
		routeGeneralPanel.add(textMinNonAdjDist);
		routeGeneralPanel.add(textFieldMinNonAdjDist);
		
		textFieldStubFixDist = new JFormattedTextField();
		textFieldStubFixDist.setText("80");
		JLabel textStubFixDist = new JLabel(" Fix dist(stubs):");
		routeGeneralPanel.add(textStubFixDist);
		routeGeneralPanel.add(textFieldStubFixDist);
		
		textFieldLocalLMFixDist = new JFormattedTextField();
		textFieldLocalLMFixDist.setText("75");
		JLabel textLocalLMFixDist = new JLabel(" Fix dist(LM):");
		routeGeneralPanel.add(textLocalLMFixDist);
		routeGeneralPanel.add(textFieldLocalLMFixDist);
		
		textFieldOrthoLevel = new JFormattedTextField();
		textFieldOrthoLevel.setText("0.0");
		JLabel textOrthoLevel = new JLabel("Level Ortho:");
		routeGeneralPanel.add(textOrthoLevel);
		routeGeneralPanel.add(textFieldOrthoLevel);
		
		
		JPanel bendPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaBend = new JLabel("Bend Factor:");
		sliderBendFactor = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderBendFactor.setBackground( new Color(1,true));
     	sliderBendFactor.setOpaque(false);
		sliderBendFactor.setMajorTickSpacing(10);
		sliderBendFactor.setMinorTickSpacing(5);
		sliderBendFactor.setPaintTicks(true);
		sliderBendFactor.setPaintLabels(true);
		bendPanel.add(textAreaBend);
		bendPanel.add(sliderBendFactor);
		
		JPanel crossingsBendPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaCrossBend = new JLabel("Cross. bend factor:");
		sliderCrossBendFactor = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderCrossBendFactor.setBackground( new Color(1,true));
		sliderCrossBendFactor.setOpaque(false);
		sliderCrossBendFactor.setMajorTickSpacing(10);
		sliderCrossBendFactor.setMinorTickSpacing(5);
		sliderCrossBendFactor.setPaintTicks(true);
		sliderCrossBendFactor.setPaintLabels(true);
		crossingsBendPanel.add(textAreaCrossBend);
		crossingsBendPanel.add(sliderCrossBendFactor);
		
		
		JPanel distPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaDist = new JLabel("Dist Factor:");
		sliderDistFactor = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderDistFactor.setBackground( new Color(1,true));
		sliderDistFactor.setOpaque(false);
		sliderDistFactor.setMajorTickSpacing(10);
		sliderDistFactor.setMinorTickSpacing(5);
		sliderDistFactor.setPaintTicks(true);
		sliderDistFactor.setPaintLabels(true);
		distPanel.add(textAreaDist);
		distPanel.add(sliderDistFactor);
		
		
		JPanel edgeOrientationPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaEdgeOrientation = new JLabel("Edge Orientation:");
		sliderEdgeOrientation = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderEdgeOrientation.setBackground( new Color(1,true));
		sliderEdgeOrientation.setOpaque(false);
		sliderEdgeOrientation.setMajorTickSpacing(10);
		sliderEdgeOrientation.setMinorTickSpacing(5);
		sliderEdgeOrientation.setPaintTicks(true);
		sliderEdgeOrientation.setPaintLabels(true);
		edgeOrientationPanel.add(textAreaEdgeOrientation);
		edgeOrientationPanel.add(sliderEdgeOrientation);
		
		
		JPanel proportionDPPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaProportionDP = new JLabel("Proportion DP:");
		sliderProportionDP = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderProportionDP.setBackground( new Color(1,true));
		sliderProportionDP.setOpaque(false);
		sliderProportionDP.setMajorTickSpacing(10);
		sliderProportionDP.setMinorTickSpacing(5);
		sliderProportionDP.setPaintTicks(true);
		sliderProportionDP.setPaintLabels(true);
		proportionDPPanel.add(textAreaProportionDP);
		proportionDPPanel.add(sliderProportionDP);
		
		JPanel proportionAllPtsPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaProportionAllPts = new JLabel("Proportion All Pts:");
		sliderProportionAllPts = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderProportionAllPts.setBackground( new Color(1,true));
		sliderProportionAllPts.setOpaque(false);
		sliderProportionAllPts.setMajorTickSpacing(10);
		sliderProportionAllPts.setMinorTickSpacing(5);
		sliderProportionAllPts.setPaintTicks(true);
		sliderProportionAllPts.setPaintLabels(true);
		proportionAllPtsPanel.add(textAreaProportionAllPts);
		proportionAllPtsPanel.add(sliderProportionAllPts);
		
		
		

		JPanel scaleVariationPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaScaleVariation = new JLabel("Scale Variation:");
		sliderScaleVariation = new JSlider(JSlider.HORIZONTAL, 1, 100, 1);
		sliderScaleVariation.setBackground( new Color(1,true));
		sliderScaleVariation.setOpaque(false);
		sliderScaleVariation.setMajorTickSpacing(10);
		sliderScaleVariation.setMinorTickSpacing(5);
		sliderScaleVariation.setPaintTicks(true);
		sliderScaleVariation.setPaintLabels(true);
		scaleVariationPanel.add(textAreaScaleVariation);
		scaleVariationPanel.add(sliderScaleVariation);
		JLabel textAreaScaleVariationRaiseFactor = new JLabel("Raise Factor:");
		sliderScaleVariationRaiseFactor = new JSlider(JSlider.HORIZONTAL, 1, 100, 20);
		sliderScaleVariationRaiseFactor.setBackground( new Color(1,true));
		sliderScaleVariationRaiseFactor.setOpaque(false);
		sliderScaleVariationRaiseFactor.setMajorTickSpacing(10);
		sliderScaleVariationRaiseFactor.setMinorTickSpacing(5);
		sliderScaleVariationRaiseFactor.setPaintTicks(true);
		sliderScaleVariationRaiseFactor.setPaintLabels(true);
		scaleVariationPanel.add(textAreaScaleVariationRaiseFactor);
		scaleVariationPanel.add(sliderScaleVariationRaiseFactor);
		
		
		JPanel rescalePointsPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaRescalePoints = new JLabel("Rescale Nodes:");
		String[] rescaleNodes = {"DP Only", "DP + Along CN", "DP + Along CN + Cross CN" , "All Intersection", "DP & LM Cross", "Pre-defined" };	
		cbRescalePoints = new JComboBox<String>(rescaleNodes);
		cbRescalePoints.setSelectedIndex(2);
		rescalePointsPanel.add(textAreaRescalePoints);
		rescalePointsPanel.add(cbRescalePoints);
		
		
		JPanel directionModelPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaDirectionModel = new JLabel("Direction Model at DP:");
		String[] directionModels = {"None","Best in-out edges orientation", "Traditional Best Bend", "Klippel Best Bend" };	
		cbDirectionModel = new JComboBox<String>(directionModels);
		cbDirectionModel.setSelectedIndex(1);
		directionModelPanel.add(textAreaDirectionModel);
		directionModelPanel.add(cbDirectionModel);
		textFieldTurnOrthoLevel = new JFormattedTextField();
		textFieldTurnOrthoLevel.setText("0.0");
		JLabel textTurnOrthoLevel = new JLabel("Turn ortho:");
		directionModelPanel.add(textTurnOrthoLevel);
		directionModelPanel.add(textFieldTurnOrthoLevel);
		
		
		
		JPanel executionTimePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		textFieldExecutionTimeLimit = new JFormattedTextField();
		textFieldExecutionTimeLimit.setText("-10");
		JLabel textAreaTimeLabel = new JLabel(" Execution Time Limit:");
		executionTimePanel.add(textAreaTimeLabel);
		executionTimePanel.add(textFieldExecutionTimeLimit);
		
		pControlRoute.add(routeTitelPanel);
		pControlRoute.add(routeGeneralPanel2);
		pControlRoute.add(routeGeneralPanel);
		pControlRoute.add(bendPanel);
		pControlRoute.add(crossingsBendPanel);
		pControlRoute.add(distPanel);
		pControlRoute.add(edgeOrientationPanel);
		pControlRoute.add(proportionDPPanel);
		pControlRoute.add(proportionAllPtsPanel);
		pControlRoute.add(scaleVariationPanel);
		pControlRoute.add(rescalePointsPanel);
		pControlRoute.add(directionModelPanel);
		pControlRoute.add(executionTimePanel);

		/****CONTROL NETWORK*****/
		pControlNetwork = new JPanel();
		pControlNetwork.setLayout(new GridLayout(0,1));
		pControlNetwork.setBorder(new MetalBorders.Flush3DBorder());
		
		JPanel networkTitelPanel = new JPanel(new FlowLayout(FlowLayout.CENTER));
		JLabel textAreaNetworkTitel = new JLabel("Network Parameters");
		networkTitelPanel.add(textAreaNetworkTitel);
		
		
		
		JPanel networkActivatePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		String[] networklevel = {"Network1", "Network1+Stubs", "Network2", "Network2+Stubs", "StubsOnly" , "No Network" , "Pre-defined" , "Pre-defined2"}; 
		cbStreetNetwork = new JComboBox<String>(networklevel);
		cbStreetNetwork.setSelectedIndex(6);
		cbReduceStubs = new JCheckBox("Reduce Stubs", false);
		cbSmoothNetwork = new JCheckBox("Smooth Geometries", true);	
		
		
		networkActivatePanel.add(cbStreetNetwork);
		networkActivatePanel.add(cbReduceStubs);
		networkActivatePanel.add(cbSmoothNetwork);
		
		
		
		JPanel networkGeneralPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		
		cbXIncorrectsNetwork = new JCheckBox("Schematize Incorrects", false);
		cbXallNetwork = new JCheckBox("Schematize All", true);
		
		cbCheckSelfTopologyNetwork = new JCheckBox("Check Self Topology", false);
		cbCheckTopologyNetwork = new JCheckBox("Check Topology", false);
	
		cbCheckSelfTopology = new JCheckBox("Check Self Topology", true);
		cbCheckTopology = new JCheckBox("Check Topology", true);
		textFieldMinDistToRoute = new JFormattedTextField();
		textFieldMinDistToRoute.setText("20");
		JLabel textMinDistToRoute = new JLabel(" min dist.:");
		
		networkGeneralPanel.add(cbXIncorrectsNetwork);
		networkGeneralPanel.add(cbXallNetwork);
		networkGeneralPanel.add(cbCheckTopologyNetwork);
		networkGeneralPanel.add(cbCheckSelfTopologyNetwork);
		networkGeneralPanel.add(textMinDistToRoute);
		networkGeneralPanel.add(textFieldMinDistToRoute);

		
		
		JPanel networkBendPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaNetworkBend = new JLabel("Bend Factor:");
		sliderBendFactorNetwork = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderBendFactorNetwork.setBackground( new Color(1,true));
		sliderBendFactorNetwork.setOpaque(false);
		sliderBendFactorNetwork.setMajorTickSpacing(10);
		sliderBendFactorNetwork.setMinorTickSpacing(5);
		sliderBendFactorNetwork.setPaintTicks(true);
		sliderBendFactorNetwork.setPaintLabels(true);
		networkBendPanel.add(textAreaNetworkBend);
		networkBendPanel.add(sliderBendFactorNetwork);
		
		
		JPanel networkDistPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaNetworkDist = new JLabel("Dist Factor:");
		sliderDistFactorNetwork = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderDistFactorNetwork.setBackground( new Color(1,true));
		sliderDistFactorNetwork.setOpaque(false);
		sliderDistFactorNetwork.setMajorTickSpacing(10);
		sliderDistFactorNetwork.setMinorTickSpacing(5);
		sliderDistFactorNetwork.setPaintTicks(true);
		sliderDistFactorNetwork.setPaintLabels(true);
		networkDistPanel.add(textAreaNetworkDist);
		networkDistPanel.add(sliderDistFactorNetwork);
		
		JPanel networkProportionPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaNetworkProportion = new JLabel("Proportion:");
		sliderProportionNetwork = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderProportionNetwork.setBackground( new Color(1,true));
		sliderProportionNetwork.setOpaque(false);
		sliderProportionNetwork.setMajorTickSpacing(10);
		sliderProportionNetwork.setMinorTickSpacing(5);
		sliderProportionNetwork.setPaintTicks(true);
		sliderProportionNetwork.setPaintLabels(true);
		sliderProportionNetwork.setEnabled(true);
		networkProportionPanel.add(textAreaNetworkProportion);
		networkProportionPanel.add(sliderProportionNetwork);
		
		
		JPanel networkEdgeOrientationPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaNetworkEdgeOrientation = new JLabel("Edge Orientation:");
		sliderEdgeOrientationNetwork = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderEdgeOrientationNetwork.setBackground( new Color(1,true));
		sliderEdgeOrientationNetwork.setOpaque(false);
		sliderEdgeOrientationNetwork.setMajorTickSpacing(10);
		sliderEdgeOrientationNetwork.setMinorTickSpacing(5);
		sliderEdgeOrientationNetwork.setPaintTicks(true);
		sliderEdgeOrientationNetwork.setPaintLabels(true);
		sliderEdgeOrientationNetwork.setEnabled(true);
		networkEdgeOrientationPanel.add(textAreaNetworkEdgeOrientation);
		networkEdgeOrientationPanel.add(sliderEdgeOrientationNetwork);
		

		JPanel networkTransSimpPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaNetworkTransSimpPanel = new JLabel("Simplification tolerance ");
		sliderTranfsSimplificationNetwork = new JSlider(JSlider.HORIZONTAL, 1, 10, 5);
		sliderTranfsSimplificationNetwork.setBackground( new Color(1,true));
		sliderTranfsSimplificationNetwork.setOpaque(false);
		sliderTranfsSimplificationNetwork.setMajorTickSpacing(2);
		sliderTranfsSimplificationNetwork.setMinorTickSpacing(1);
		sliderTranfsSimplificationNetwork.setPaintTicks(true);
		sliderTranfsSimplificationNetwork.setPaintLabels(true);
		networkTransSimpPanel.add(textAreaNetworkTransSimpPanel);
		networkTransSimpPanel.add(sliderTranfsSimplificationNetwork);
		

		
		JPanel networkExecutionTimePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		textFieldExecutionTimeLimitNetwork = new JFormattedTextField();
		textFieldExecutionTimeLimitNetwork.setText("-22");
		JLabel textAreaNetworkTimeLabel = new JLabel(" Execution Time Limit:");
		networkExecutionTimePanel.add(textAreaNetworkTimeLabel);
		networkExecutionTimePanel.add(textFieldExecutionTimeLimitNetwork);
		
		pControlNetwork.add(networkTitelPanel);
		pControlNetwork.add(networkActivatePanel);
		pControlNetwork.add(networkGeneralPanel);
		pControlNetwork.add(networkBendPanel);
		pControlNetwork.add(networkDistPanel);
		pControlNetwork.add(networkProportionPanel);
		pControlNetwork.add(networkEdgeOrientationPanel);

		pControlNetwork.add(networkExecutionTimePanel);
		pControlNetwork.add(networkTransSimpPanel);

		/****CONTROL REGIONS*****/
		pControlRegion = new JPanel();
		pControlRegion.setLayout(new GridLayout(0,1));
		pControlRegion.setBorder(new MetalBorders.Flush3DBorder());
		
		JPanel regionTitelPanel = new JPanel(new FlowLayout(FlowLayout.CENTER));
		JLabel textAreaRegionTitel = new JLabel("Region Parameters");
		regionTitelPanel.add(textAreaRegionTitel);
		
		JPanel regionActivatePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		cbXIncorrects = new JCheckBox("Schematize Incorrects", false);
		cbXall = new JCheckBox("Schematize All", true);
		cbSmoothLandmarks = new JCheckBox("Smooth Geometries", true);
		regionActivatePanel.add(cbXIncorrects);
		regionActivatePanel.add(cbXall);
		regionActivatePanel.add(cbSmoothLandmarks);
		
		JPanel regionGeneralPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		
		
		
		
		regionGeneralPanel.add(cbCheckTopology);
		regionGeneralPanel.add(cbCheckSelfTopology);
		
		
		
		JPanel regionBendPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaRegionBend = new JLabel("Bend Factor:");
		sliderBendFactorRegion = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderBendFactorRegion.setBackground( new Color(1,true));
		sliderBendFactorRegion.setOpaque(false);
		sliderBendFactorRegion.setMajorTickSpacing(10);
		sliderBendFactorRegion.setMinorTickSpacing(5);
		sliderBendFactorRegion.setPaintTicks(true);
		sliderBendFactorRegion.setPaintLabels(true);
		regionBendPanel.add(textAreaRegionBend);
		regionBendPanel.add(sliderBendFactorRegion);
		
		
		JPanel regionDistPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaRegionDist = new JLabel("Dist Factor:");
		sliderDistFactorRegion = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderDistFactorRegion.setBackground( new Color(1,true));
		sliderDistFactorRegion.setOpaque(false);
		sliderDistFactorRegion.setMajorTickSpacing(10);
		sliderDistFactorRegion.setMinorTickSpacing(5);
		sliderDistFactorRegion.setPaintTicks(true);
		sliderDistFactorRegion.setPaintLabels(true);
		regionDistPanel.add(textAreaRegionDist);
		regionDistPanel.add(sliderDistFactorRegion);
		
		JPanel regionProportionPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaRegionProportion = new JLabel("Proportion:");
		sliderProportionRegion = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderProportionRegion.setBackground( new Color(1,true));
		sliderProportionRegion.setOpaque(false);
		sliderProportionRegion.setMajorTickSpacing(10);
		sliderProportionRegion.setMinorTickSpacing(5);
		sliderProportionRegion.setPaintTicks(true);
		sliderProportionRegion.setPaintLabels(true);
		sliderProportionRegion.setEnabled(true);
		regionProportionPanel.add(textAreaRegionProportion);
		regionProportionPanel.add(sliderProportionRegion);
		
		
		JPanel regionEdgeOrientationPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaRegionEdgeOrientation = new JLabel("Edge Orientation:");
		sliderEdgeOrientationRegion = new JSlider(JSlider.HORIZONTAL, 1, 100, 50);
		sliderEdgeOrientationRegion.setBackground( new Color(1,true));
		sliderEdgeOrientationRegion.setOpaque(false);
		sliderEdgeOrientationRegion.setMajorTickSpacing(10);
		sliderEdgeOrientationRegion.setMinorTickSpacing(5);
		sliderEdgeOrientationRegion.setPaintTicks(true);
		sliderEdgeOrientationRegion.setPaintLabels(true);
		sliderEdgeOrientationRegion.setEnabled(true);
		regionEdgeOrientationPanel.add(textAreaRegionEdgeOrientation);
		regionEdgeOrientationPanel.add(sliderEdgeOrientationRegion);
		
		
		JPanel alongessPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaAlongness = new JLabel("Alongness:");
		sliderAlongness = new JSlider(JSlider.HORIZONTAL, 0, 100, 0);
		sliderAlongness.setBackground( new Color(1,true));
		sliderAlongness.setOpaque(false);
		sliderAlongness.setMajorTickSpacing(10);
		sliderAlongness.setMinorTickSpacing(5);
		sliderAlongness.setPaintTicks(true);
		sliderAlongness.setPaintLabels(true);
		sliderAlongness.setEnabled(true);
		alongessPanel.add(textAreaAlongness);
		alongessPanel.add(sliderAlongness);
		
		
		JPanel regionTransSimpPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JLabel textAreaRegionTransSimpPanel = new JLabel("Simplification tolerance ");
		sliderTranfsSimplification = new JSlider(JSlider.HORIZONTAL, 1, 10, 5);
		sliderTranfsSimplification.setBackground( new Color(1,true));
		sliderTranfsSimplification.setOpaque(false);
		sliderTranfsSimplification.setMajorTickSpacing(2);
		sliderTranfsSimplification.setMinorTickSpacing(1);
		sliderTranfsSimplification.setPaintTicks(true);
		sliderTranfsSimplification.setPaintLabels(true);
		regionTransSimpPanel.add(textAreaRegionTransSimpPanel);
		regionTransSimpPanel.add(sliderTranfsSimplification);
		

		
		JPanel regionExecutionTimePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		textFieldExecutionTimeLimitRegion = new JFormattedTextField();
		textFieldExecutionTimeLimitRegion.setText("-10");
		JLabel textAreaRegionTimeLabel = new JLabel(" Execution Time Limit:");
		regionExecutionTimePanel.add(textAreaRegionTimeLabel);
		regionExecutionTimePanel.add(textFieldExecutionTimeLimitRegion);
		
		pControlRegion.add(regionTitelPanel);
		pControlRegion.add(regionActivatePanel);
		pControlRegion.add(regionGeneralPanel);
		pControlRegion.add(regionBendPanel);
		pControlRegion.add(regionDistPanel);
		pControlRegion.add(regionProportionPanel);
		pControlRegion.add(regionEdgeOrientationPanel);
		pControlRegion.add(alongessPanel);
		pControlRegion.add(regionExecutionTimePanel);
		pControlRegion.add(regionTransSimpPanel);
		
		
		
		
		//pControl.add(pControlGeneral);
		pControl.add(pControlRoute);
		pControl.add(pControlNetwork);
		pControl.add(pControlRegion);
		

		
		

		tpCenter = new JTabbedPane();	
		tpCenter.add( "Controls" , pControl );


		pCenter = new JPanel();
		pCenter.setLayout( new BorderLayout() );
		pCenter.add( tpCenter );

		/*Adiciona componentes ao pSouth*/
		taPosicao = new JTextArea("Pos");
		taPosicao.setEditable(true);
		JLabel  lOpInfo = new JLabel( "Informações Operacionais" );
		lOpInfo.setForeground( Color.GRAY );
		pSouth = new JPanel();
		pSouth.setBorder( new MetalBorders.Flush3DBorder() );
		pSouth.add( lOpInfo );
		pSouth.add( taPosicao );



		pEast = new JPanel();

		
		this.setLayout(new BorderLayout());
		this.add(BorderLayout.NORTH, pNorth);
		this.add(BorderLayout.WEST, pWest);		
		this.add(BorderLayout.SOUTH, pSouth);
		this.add(BorderLayout.CENTER, pCenter);
		

		/* Eventos dos Bot�es*/
		
		




		

		
		this.setVisible(true);
	}



	public void addListener (ActionListener listener) {  
		    this.bLoadRoute.addActionListener(listener);
		    this.bSchematize.addActionListener(listener);
		    this.bOpenShapeFile.addActionListener(listener);
		    this.bValidateGeometry.addActionListener(listener);
		    this.bExportShape.addActionListener(listener);
		    this.bExportGeoJson.addActionListener(listener);
		    this.bReportFiles.addActionListener(listener);
		    

		    this.cbShowAllLayers.addActionListener(listener);
		    this.cbToggleXOLayers.addActionListener(listener);
	    } 

	/* adicona ao painel de abas*/
	public void addTab(String name,Component panel) {
		tpCenter.add( name , panel );
		
		
	}



	public JTabbedPane getTpCenter() {
		return tpCenter;
	}




	public JSlider getSliderBendFactor() {
		return sliderBendFactor;
	}



	public JSlider getSliderCrossBendFactor() {
		return sliderCrossBendFactor;
	}



	public JSlider getSliderDistFactor() {
		return sliderDistFactor;
	}



	public JSlider getSliderEdgeOrientation() {
		return sliderEdgeOrientation;
	}
	
	

	public JSlider getSliderProportionDP() {
		return sliderProportionDP;
	}



	public JSlider getSliderProportionAllPts() {
		return sliderProportionAllPts;
	}



	public JSlider getSliderScaleVariation() {
		return sliderScaleVariation;
	}

	
	

	public JSlider getSliderScaleVariationRaiseFactor() {
		return sliderScaleVariationRaiseFactor;
	}



	public JFormattedTextField getTextFieldExecutionTimeLimit() {
		return textFieldExecutionTimeLimit;
	}



	public JComboBox<String> getCbRescalePoints() {
		return cbRescalePoints;
	}



	public JComboBox<String> getCbDirectionModel() {
		return cbDirectionModel;
	}




	public JSlider getSliderBendFactorRegion() {
		return sliderBendFactorRegion;
	}




	public JSlider getSliderDistFactorRegion() {
		return sliderDistFactorRegion;
	}


	public JSlider getSliderProportionRegion() {
		return sliderProportionRegion;
	}



	public JSlider getSliderEdgeOrientationRegion() {
		return sliderEdgeOrientationRegion;
	}

	

	public JSlider getSliderAlongness() {
		return sliderAlongness;
	}



	public JSlider getSliderTranfsSimplification() {
		return sliderTranfsSimplification;
	}



	public JFormattedTextField getTextFieldExecutionTimeLimitRegion() {
		return textFieldExecutionTimeLimitRegion;
	}







	public JSlider getSliderBendFactorNetwork() {
		return sliderBendFactorNetwork;
	}



	public JSlider getSliderDistFactorNetwork() {
		return sliderDistFactorNetwork;
	}



	public JSlider getSliderEdgeOrientationNetwork() {
		return sliderEdgeOrientationNetwork;
	}



	public JSlider getSliderProportionNetwork() {
		return sliderProportionNetwork;
	}



	public JSlider getSliderTranfsSimplificationNetwork() {
		return sliderTranfsSimplificationNetwork;
	}



	public JFormattedTextField getTextFieldExecutionTimeLimitNetwork() {
		return textFieldExecutionTimeLimitNetwork;
	}



	public JCheckBox getCbXIncorrectsNetwork() {
		return cbXIncorrectsNetwork;
	}

	

	public JCheckBox getCbReduceStubs() {
		return cbReduceStubs;
	}



	public JCheckBox getCbXallNetwork() {
		return cbXallNetwork;
	}



	public JCheckBox getCbCheckSelfTopologyNetwork() {
		return cbCheckSelfTopologyNetwork;
	}



	public JCheckBox getCbCheckTopologyNetwork() {
		return cbCheckTopologyNetwork;
	}



	public JCheckBox getCbSmoothNetwork() {
		return cbSmoothNetwork;
	}



	public JCheckBox getCbXIncorrects() {
		return cbXIncorrects;
	}



	public JCheckBox getCbXall() {
		return cbXall;
	}



	public JCheckBox getCbCheckSelfTopology() {
		return cbCheckSelfTopology;
	}



	public JCheckBox getCbCheckTopology() {
		return cbCheckTopology;
	}



	public JButton getbLoadRoute() {
		return bLoadRoute;
	}



	public JComboBox<String> getCbSource() {
		return cbSource;
	}


	public JComboBox<String> getCbDestination() {
		return cbDestination;
	}


	

	public JFormattedTextField getTextFieldSource() {
		return textFieldSource;
	}



	public JFormattedTextField getTextFieldDestination() {
		return textFieldDestination;
	}



	public JComboBox<String> getCbStreetNetwork() {
		return cbStreetNetwork;
	}



	public JButton getbValidateGeometry() {
		return bValidateGeometry;
	}



	public JButton getbOpenShapeFile() {
		return bOpenShapeFile;
	}


	public JTextArea getTaPosicao() {
		return taPosicao;
	}


	public JCheckBox getCbShowAllLayers() {
		return cbShowAllLayers;
	}

	public JCheckBox getCbToggleXOLayers() {
		return cbToggleXOLayers;
	}



	public JButton getbExportShape() {
		return bExportShape;
	}



	public JButton getbExportGeoJson() {
		return bExportGeoJson;
	}

	

	public JButton getbReportFiles() {
		return bReportFiles;
	}



	public JButton getbSchematize() {
		return bSchematize;
	}


	public JTable getLayerTable() {
		return layerTable;
	}



	public JCheckBox getCbCheckRouteTopology() {
		return cbCheckRouteTopology;
	}




	public JCheckBox getCbSmoothLandmarks() {
		return cbSmoothLandmarks;
	}



	public JFormattedTextField getTextFieldMinAdjDist() {
		return textFieldMinAdjDist;
	}



	public JFormattedTextField getTextFieldMinNonAdjDist() {
		return textFieldMinNonAdjDist;
	}



	public JFormattedTextField getTextFieldStubFixDist() {
		return textFieldStubFixDist;
	}



	public JFormattedTextField getTextFieldLocalLMFixDist() {
		return textFieldLocalLMFixDist;
	}



	
	public JFormattedTextField getTextFieldOrthoLevel() {
		return textFieldOrthoLevel;
	}
	
	



	public JFormattedTextField getTextFieldTurnOrthoLevel() {
		return textFieldTurnOrthoLevel;
	}



	public JFormattedTextField getTextFieldMinDistToRoute() {
		return textFieldMinDistToRoute;
	}


	


}