package com.wayto.view;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import javax.swing.JCheckBox;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeListener;

import com.wayto.mapFeaturesModel.ClassyPointLayer;
import com.wayto.mapFeaturesModel.Layer;
import com.wayto.mapFeaturesModel.MapContent;
import com.wayto.mapFeaturesModel.PlotPoint;
import com.wayto.mapFeaturesModel.PointLayer;
import com.wayto.mapFeaturesModel.PolyLineLayer;




public class XMap extends JPanel{
	

	private MapContent mapContent;


	/* painel de controle*/
	private JCheckBox addLabels;
	private JSlider sliderAngleCoe;
	private JSlider sliderDistorcionCoe;
	
	/* atributos de transforma��o */	
	private AffineTransform transformTranslate;
	private AffineTransform transformRotate;
	private AffineTransform transformScale;
	
	private AffineTransform transformCenterDrawing;
	private Rectangle2D canvas;
	
	
	//private Dimension panelDimension;
	public Point2D.Double ptPivo ;
	public int xTranslation; 
	public int yTranslation;
	
	public int xMousePressedPos;
	public int yMousePressedPos;
	public double angle = 0;
	public int xMousePressedPos2;
	public int yMousePressedPos2;
	public Dimension planeDimension;
	private int stokeWith = 3;
	
	public XMap(){	
		
		
		/*instacia painel de controle*/
		
		this.addLabels = new JCheckBox("Adicionar rotulos" ,false);
		addLabels.setBackground( new Color(1,true));
		addLabels.setOpaque(false);
		
		
		sliderAngleCoe = new JSlider(SwingConstants.HORIZONTAL, 1, 100, 1);
		sliderAngleCoe.setBackground( new Color(1,true));
     	sliderAngleCoe.setOpaque(false);
		sliderAngleCoe.setMajorTickSpacing(10);
		sliderAngleCoe.setMinorTickSpacing(5);
		sliderAngleCoe.setPaintTicks(true);
		sliderAngleCoe.setPaintLabels(true);
		
		
		transformTranslate = new AffineTransform();		
		transformScale = new AffineTransform();
		transformRotate = new AffineTransform();
		transformCenterDrawing = new AffineTransform();
		
		
		
		
		ptPivo = new Point2D.Double();
		mapContent = new MapContent();
		
		
		SchematicMapMouseHandler handler = new SchematicMapMouseHandler();
		this.addMouseListener(handler);
		this.addMouseMotionListener(handler);
		this.addMouseWheelListener(handler);
			
	}
	
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		
		
		this.setBackground(Color.WHITE);
		

		/* configura scala */
		
		planeDimension = this.getSize();
		ptPivo.setLocation(planeDimension.getWidth()/2 , planeDimension.getHeight()/2);

		/* Altera Desnhador gr�fico */
		Graphics2D g2d = ( Graphics2D )g.create();
		g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
				RenderingHints.VALUE_ANTIALIAS_ON);
		g2d.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
				RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
		g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g2d.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
		
		
		
		this.add(sliderAngleCoe).setBounds(20, (int)planeDimension.getHeight() - 135, 200, 40);
		this.add(addLabels).setBounds(20, (int)planeDimension.getHeight() - 100, 200, 25);
		
		//drawScales(g2d, 6 , planeDimension );
		

		g2d.transform(transformTranslate );		
		g2d.transform(transformScale );
		g2d.transform(transformRotate );
		
		/*test shape transformations*/
//		canvas = new Rectangle.Double(230.0, 90.0, planeDimension.getWidth()/16, planeDimension.getHeight()/6);
//		g2d.setStroke(new BasicStroke(5, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL));
//		g2d.setColor( Color.BLACK);    
//		g2d.draw(canvas);
//		g2d.fill(canvas);
//		double scale = 0.9/Math.max( (canvas.getWidth())/planeDimension.getWidth() , (canvas.getHeight())/planeDimension.getHeight());
//		double tX = -canvas.getMinX() + (planeDimension.getWidth()/2)/scale  - canvas.getWidth()/2;
//		double tY = -canvas.getMinY() + (planeDimension.getHeight()/2)/scale  - canvas.getHeight()/2;
//		g2d.transform(transformCenterDrawing);
//		g2d.scale(scale, scale);
//		g2d.translate( tX , tY);
//		g2d.setStroke(new BasicStroke(5, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL));
//		g2d.setColor( Color.BLACK);    
//		g2d.draw(canvas);
//		g2d.fill(canvas);
		
		
//		g2d.scale(transformCenterDrawing.getScaleX(), transformCenterDrawing.getScaleY());
//		
//		g2d.translate(transformCenterDrawing.getTranslateX(), transformCenterDrawing.getTranslateY());
		ArrayList<Layer> auxLayerList = new ArrayList<Layer>();
		auxLayerList.addAll(mapContent.getLayers());
		auxLayerList.sort(Layer.LayerDepthComparator);

		
		for(Layer layer:auxLayerList){
			if(layer.isVisible()){
				float stroke;
				float size;
				if(layer instanceof PointLayer) {
					
					PointLayer pLayer = (PointLayer)layer;
					
					stroke = pLayer.getStroke()/(float)transformScale.getScaleX();
					size = pLayer.getSize()/(float)transformScale.getScaleX();
					g2d.setStroke(new BasicStroke(stroke, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL));
	
					for(Point2D p:pLayer.getPoints()){
	
						Ellipse2D.Double shape = new Ellipse2D.Double(p.getX() - size/2,p.getY() - size/2, size, size);
						g2d.setColor( pLayer.getColor());    
						g2d.draw(shape);
						g2d.setColor( pLayer.getFillColor());
						    g2d.fill(shape);
								
					}
				}
				if(layer instanceof ClassyPointLayer) {	
					ClassyPointLayer cpLayer = (ClassyPointLayer)layer;
					
					stroke = cpLayer.getStroke()/(float)transformScale.getScaleX();
					//size = cpLayer.getSize()/(float)transformScale.getScaleX();
					g2d.setStroke(new BasicStroke(stroke, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL));

					for(PlotPoint p:cpLayer.getPoints()){
						size = p.getSize()/(float)transformScale.getScaleX();
						Ellipse2D.Double shape = new Ellipse2D.Double(p.getPosition().getX() - size/2,p.getPosition().getY() - size/2, size, size);
						g2d.setColor( cpLayer.getColor());    
						g2d.draw(shape);
						g2d.setColor( p.getFillColor());
						g2d.fill(shape);
								
					}
					
				}
				if(layer instanceof PolyLineLayer) {
					PolyLineLayer plLayer = (PolyLineLayer)layer;
					stroke = plLayer.getStroke()/(float)transformScale.getScaleX();
					
					g2d.setColor( plLayer.getColor());
					g2d.setStroke(new BasicStroke(stroke, BasicStroke.CAP_ROUND, BasicStroke.JOIN_BEVEL));
					
					for( ArrayList<Point2D> polyline: plLayer.getLines()){
						
						
						
		
						
						Path2D.Double path = new Path2D.Double();
						path.moveTo(polyline.get(0).getX(), polyline.get(0).getY());
		
						for(Point2D p:polyline){
	
							path.lineTo(p.getX(),p.getY());			
						}
						
						g2d.draw(path);
						if(plLayer.isClosed()){
							//System.out.println("CLOSED POLYLINE " + plLayer.getName()  + " : " + polyline.get(0) + " " + polyline.get(polyline.size() -1));
							g2d.setColor( plLayer.getFillColor());
							g2d.fill(path);
						}
					}
					/*Draw vertices */
					/*double diameter = 2*stroke;
					g2d.setColor( Color.WHITE);
					for( ArrayList<Point2D> polyline: plLayer.getLines()){
						
		
						
		

		
						for(Point2D p:polyline){
							
							Ellipse2D.Double shape = new Ellipse2D.Double(p.getX() - diameter/2,p.getY() - diameter/2, diameter, diameter);
							    g2d.fill(shape);
									
						}
						
						
					}*/
				}
					
			}
		}
			
					
	
			
		
		
	}
	
	
	private Polygon toPolygon(ArrayList<Point2D> track) {
		Polygon poly = new Polygon();
		for( Point2D pt: track){
			
			poly.addPoint( (int)pt.getX() , (int)pt.getY());
		}
		
		return poly;
	}
	
	private void drawScales(Graphics2D g , int units , Dimension planeDimension){
		
		int dif = (int)((planeDimension.getWidth() )/units);		
		g.setPaint(Color.LIGHT_GRAY);
		g.setStroke(new BasicStroke(0.5f));
		for (int i = 0 ; i <= units; i ++){
			g.drawLine(dif*i , 0 , 
					dif*i , (int)planeDimension.getHeight());
		}
		
		dif = (int)(planeDimension.getHeight()/units);

		g.setPaint(Color.LIGHT_GRAY);
		g.setStroke(new BasicStroke(0.5f));
		for (int i = 0 ; i <= units; i ++){
			if (i != 0)
				g.drawLine(0, dif*i  , 
						(int)planeDimension.getWidth(), dif*i );
		}

	}
	
	private class CompositeStroke implements Stroke {
		private Stroke stroke1, stroke2;

		public CompositeStroke( Stroke stroke1, Stroke stroke2 ) {
			this.stroke1 = stroke1;
			this.stroke2 = stroke2;
		}

		public Shape createStrokedShape( Shape shape ) {
			return stroke2.createStrokedShape( stroke1.createStrokedShape( shape ) );
		}
	}
	
	public void incrementAngle(double rot) {
		
		double aux = (rot + angle)%(2*Math.PI);
		if(aux < 0)
			aux = (Math.PI*2) + aux;
		angle = aux;
	}
	
	public void setStokeWith(int stokeWith) {
		this.stokeWith = stokeWith;
	}



	public JSlider getSliderAngleCoe() {
		return sliderAngleCoe;
	}
	
	
	
	public MapContent getMapContent() {
		return mapContent;
	}

	public void setMapContent(MapContent layers) {
		this.mapContent = layers;
	}

	public void addListener (ChangeListener listener) {  
	    this.sliderAngleCoe.addChangeListener(listener);

    } 
	
	private class SchematicMapMouseHandler implements MouseListener , MouseMotionListener, MouseWheelListener{

		/* variaveis auxiliares */
		private int oldxTranslation = 0;
		private int oldyTranslation = 0;	
		private double angleAux = 0;
		private double oldAngle = 0;
		
		private boolean rightClick = false;
		private Point2D.Double ptPivoAux = new Point2D.Double();

		
		public void mouseClicked( MouseEvent event ){
			xMousePressedPos = event.getX();
			yMousePressedPos = event.getY();
			System.out.println("X: " + xMousePressedPos + " Y: " + yMousePressedPos);
			Point2D.Double pt  = new Point2D.Double(xMousePressedPos, yMousePressedPos);
			transformScale.transform(pt, pt);
			transformTranslate.transform(pt, pt);
			System.out.println(pt);
			
			
		}	

		public void mousePressed( MouseEvent event ){
		
			xMousePressedPos = event.getX();
			yMousePressedPos = event.getY();
			if ( event.getButton() == MouseEvent.BUTTON1 ){
				System.out.println("x: " + xMousePressedPos + "y: " + yMousePressedPos);
				rightClick  = false;
			}
			else if ( event.getButton() == MouseEvent.BUTTON3 )
				rightClick = true;
			

			try {
				transformTranslate.inverseTransform(ptPivo, ptPivoAux);
				transformScale.inverseTransform(ptPivoAux, ptPivoAux);
				transformRotate.inverseTransform(ptPivoAux, ptPivoAux);
			} catch (NoninvertibleTransformException e) {

				e.printStackTrace();
			}
		}

		public void mouseReleased( MouseEvent event ){

			if(rightClick){
				
				repaint();
				
			}
			xTranslation = 0;
			yTranslation = 0;
			oldxTranslation = 0;
			oldyTranslation = 0;
			oldAngle = 0;
			//schematicMap.angle = 0;
			
			
			

		}

		public void mouseEntered( MouseEvent event ){

		}

		public void mouseExited( MouseEvent event ){

		} 

		public void mouseDragged( MouseEvent event ){
			
			if(!rightClick){
				xTranslation = (event.getX() - xMousePressedPos) ;
				yTranslation = (event.getY() - yMousePressedPos) ;
				transformTranslate.translate(xTranslation - oldxTranslation, yTranslation - oldyTranslation);
				oldxTranslation = xTranslation ;
				oldyTranslation = yTranslation ;
			}
			else{
				
				xMousePressedPos2 = event.getX();
				yMousePressedPos2 = event.getY();
				
				angleAux = getAngle(planeDimension.getWidth()/ 2 , planeDimension.getHeight()/2 ,
						xMousePressedPos , yMousePressedPos ,
						xMousePressedPos2 , yMousePressedPos2) ;
				


				transformRotate.rotate( angleAux - oldAngle , ptPivoAux.getX() , ptPivoAux.getY());
				incrementAngle( angleAux - oldAngle ) ;
				oldAngle = angleAux;				
				
				
			}
			repaint();

		}
		public void mouseMoved( MouseEvent event ){


		}

		public void mouseWheelMoved(MouseWheelEvent e) {
	

			if ( e.getWheelRotation() > 0){
				transformScale.scale(0.9, 0.9);
			}
			else{
				transformScale.scale(1.1, 1.1);
			}
			
			transformTranslate.translate( (planeDimension.getWidth()/2 - e.getX() )/2 , 
					(planeDimension.getHeight()/2 - e.getY() )/2 );
			repaint();
		}
		
		private double getAngle(double origemX, double origemY, double v1x,
				double v1y, double v2x, double v2y) {
			
			return( (Math.atan2(v2y - origemY, v2x - origemX) ) - ( Math.atan2(v1y - origemY, v1x - origemX) ) );
			
			
		}
	}

	public void resetTransformations() {
		transformTranslate = new AffineTransform();		
		transformScale = new AffineTransform();
		transformRotate = new AffineTransform();
		transformCenterDrawing = new AffineTransform();
	}

//	public AffineTransform getTransformCenterDrawing() {
//		return transformCenterDrawing;
//	}
//
//	public void setTransformCenterDrawing(AffineTransform transformCenterDrawing) {
//		this.transformCenterDrawing = transformCenterDrawing;
//	}



	public void setCanvas(double  x, double y, double width, double height) {
		this.canvas = new Rectangle2D.Double(x, y, width, height);
	}

	public AffineTransform getTransformTranslate() {
		return transformTranslate;
	}

	public void setTransformTranslate(AffineTransform transformTranslate) {
		this.transformTranslate = transformTranslate;
	}

	public AffineTransform getTransformRotate() {
		return transformRotate;
	}

	public void setTransformRotate(AffineTransform transformRotate) {
		this.transformRotate = transformRotate;
	}

	public AffineTransform getTransformScale() {
		return transformScale;
	}

	public void setTransformScale(AffineTransform transformScale) {
		this.transformScale = transformScale;
	}

	

	

	

}
