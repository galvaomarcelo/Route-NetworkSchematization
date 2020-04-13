package com.wayto.operator;

import java.util.ArrayList;

public final class PolygonVertexStats {

	public static ArrayList<Double> distFirstLast(ArrayList<Double> polyPointsIndex) {
		
		ArrayList<Double> distFirstLast = new ArrayList<Double>();
		double min = polyPointsIndex.get(0);
		double max = polyPointsIndex.get(0);
		
		for(int i = 0; i < polyPointsIndex.size(); i ++){
			min = polyPointsIndex.get(i)< min ?  polyPointsIndex.get(i): min;
			max = polyPointsIndex.get(i) > max ? polyPointsIndex.get(i):max;
			
		}
		for(int i = 0; i < polyPointsIndex.size(); i ++){
			double dist = polyPointsIndex.get(i) - min < max - polyPointsIndex.get(i)? polyPointsIndex.get(i) - min : max - polyPointsIndex.get(i) ;
			distFirstLast.add(dist); 
			
		}
		
		
		
		
		
		return normalize(distFirstLast);
	}
	/*use route dist preference 1-100*/
	public static ArrayList<Double> distFirst(ArrayList<Double> polyPointsIndex, ArrayList<Double> polyPointsDist, double routeDistPreference) {
		ArrayList<Double> distFirst = new ArrayList<Double>();
		ArrayList<Double> bestFirst = new ArrayList<Double>();
		double min = polyPointsIndex.get(0);
		
		
		for(int i = 0; i < polyPointsIndex.size(); i ++){
			min = polyPointsIndex.get(i)< min ?  polyPointsIndex.get(i): min;
		
		}
		for(int i = 0; i < polyPointsIndex.size(); i ++){
			double distToFirst = polyPointsIndex.get(i) - min ;
			distFirst.add(distToFirst); 
			
			
		}
		distFirst = normalize(distFirst);
		ArrayList<Double> distNormalized = normalize(polyPointsDist);
		for(int i = 0; i < polyPointsIndex.size(); i ++){
			bestFirst.add( distFirst.get(i)*100 + Math.pow(distNormalized.get(i)*routeDistPreference, 1.3));
		}
		
		return normalize(bestFirst);
		
	}
	/*use route dist preference 1-100*/
	public static ArrayList<Double> distLast(ArrayList<Double> polyPointsIndex, ArrayList<Double> polyPointsDist, double routeDistPreference) {
		ArrayList<Double> distLast = new ArrayList<Double>();
		ArrayList<Double> bestLast = new ArrayList<Double>();
		double max = polyPointsIndex.get(0);
		
		
		for(int i = 0; i < polyPointsIndex.size(); i ++){
			max = polyPointsIndex.get(i) > max ? polyPointsIndex.get(i):max;
		
		}
		for(int i = 0; i < polyPointsIndex.size(); i ++){
			double distToLast = max - polyPointsIndex.get(i) ;
			distLast.add(distToLast); 	
		}
		distLast = normalize(distLast);
		ArrayList<Double> distNormalized = normalize(polyPointsDist);
		
		for(int i = 0; i < polyPointsIndex.size(); i ++){
			bestLast.add( distLast.get(i)*100 + Math.pow(distNormalized.get(i)*routeDistPreference, 1.3));
		}
		
		
		return normalize(bestLast);
	}


	public static ArrayList<Double> bestStartEnd(ArrayList<Double> polyPointsDistRoute,
			ArrayList<Double> polyPointsLenghtToCross) {
		ArrayList<Double> startEndGrades = new ArrayList<Double>();
		
		ArrayList<Double> polyPointsDistRouteNormalized = normalize(polyPointsDistRoute);
		ArrayList<Double> polyPointsLenghtToCrossNormalized = normalize(polyPointsLenghtToCross);
		System.out.println("dist to route norm:" + polyPointsDistRouteNormalized);
		System.out.println("leght to cros:" + polyPointsLenghtToCross);
		System.out.println("leght to cross norm:" + polyPointsLenghtToCrossNormalized);

		for(int i = 0; i < polyPointsDistRoute.size(); i++) {
			startEndGrades.add((1.8f)*polyPointsLenghtToCrossNormalized.get(i) + polyPointsDistRouteNormalized.get(i) );
			
		}
		System.out.println("start end grades:" + startEndGrades);
		return startEndGrades;
	}

	public static ArrayList<Double> normalize(ArrayList<Double> list) {
		
		ArrayList<Double> normalizedList = new ArrayList<Double>();
		double min = list.get(0);
		double max = list.get(0);
		
		for(int i = 0; i < list.size(); i ++){
			min = list.get(i)< min ?  list.get(i): min;
			max = list.get(i) > max ? list.get(i):max;
			
		}
		
		for(int i = 0; i < list.size(); i ++){
			double value = (list.get(i) - min)/(max - min);
			normalizedList.add(value);
			
		}
		
		
		return normalizedList;
		
	}

	public static double min(ArrayList<Double> list) {
		double min = list.get(0);
		for(int i = 0; i < list.size(); i ++){
			min = list.get(i)< min ?  list.get(i): min;
		
		}
		return min;
	}




	
	
}
