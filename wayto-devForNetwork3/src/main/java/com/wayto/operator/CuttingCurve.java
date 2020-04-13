package com.wayto.operator;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class CuttingCurve {
	private  List<Point2D.Double> points;
	public CuttingCurve() {
		points = new LinkedList<Point2D.Double>();
	}
	public void addPoint( Point2D.Double point) {
		points.add(point);
	}
	public List<Point2D.Double> smoothByChaikin(double tension, int nrOfIteration) 
	{
		
		//double cutdist = 0.05 + (tension * 0.4);
		double cutdist = tension;
		//make a copy of pointList and iterate it 
		List<Point2D.Double> nl = new ArrayList<Point2D.Double>();
		List<Point2D.Double> nl2 = new ArrayList<Point2D.Double>();
		for(int i =0; i< points.size();i++)
		{
			nl.add(points.get(i));
		}
		
		for(int i=0;i< nrOfIteration;i++)
		{
			nl2 = getSmootherChaikin(nl,cutdist);
			nl = null;
			nl = nl2;
			nl2 =null;
		}
		return nl;
	}
	public List<Point2D.Double> getSmootherChaikin(List<Point2D.Double> nl, double cuttingDist)
	{
		List<Point2D.Double> nl2 = new ArrayList<Point2D.Double>();
		nl2.add(nl.get(0));
		
		for(int i =0; i< nl.size()-1;i++)
		{
			Point2D.Double q = new Point2D.Double();
			Point2D.Double r = new Point2D.Double();
			q.x = cuttingDist*(nl.get(i+1).x-nl.get(i).x) + nl.get(i).x;
			q.y = cuttingDist*(nl.get(i+1).y-nl.get(i).y) + nl.get(i).y;
			r.x = (1-cuttingDist)*(nl.get(i+1).x-nl.get(i).x) + nl.get(i).x;
			r.y = (1-cuttingDist)*(nl.get(i+1).y-nl.get(i).y) + nl.get(i).y;
			nl2.add(q);
			nl2.add(r);
		}
		nl2.add(nl.get(nl.size()-1));
		return nl2;
	}
}
