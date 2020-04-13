package com.wayto.resultReport;

import java.text.DecimalFormat;

public class RouteSchematizationReport {
	
	private int numberOfNodes;
	private int nubmerOfRouteNodes;
	private int numberOfEdges;
	private int numberOfDP;
	private int nubmerOfPLLanmarks;
	private int numberOfStubs;
	private double pathLength;
	

	private SoftConstraintValues orientationSC;
	private SoftConstraintValues distanceSC;
	private SoftConstraintValues proportionSC;
	private SoftConstraintValues proportionDPSC;
	private SoftConstraintValues bendSC;

	private double scaleVariation;
	private double objectiveFunctionValue;
	private long executionTime; 
	private double gap;
	
	private int executions;
	private int fixedExtraCrossings;
	
	
	
	
	
	
	public RouteSchematizationReport() {
		super();
		
	}
	public int getNumberOfNodes() {
		return numberOfNodes;
	}
	public void setNumberOfNodes(int numberOfNodes) {
		this.numberOfNodes = numberOfNodes;
	}
	
	public int getNubmerOfRouteNodes() {
		return nubmerOfRouteNodes;
	}
	public void setNubmerOfRouteNodes(int nubmerOfRouteNodes) {
		this.nubmerOfRouteNodes = nubmerOfRouteNodes;
	}
	public int getNumberOfEdges() {
		return numberOfEdges;
	}
	public void setNumberOfEdges(int numberOfEdges) {
		this.numberOfEdges = numberOfEdges;
	}
	public int getNumberOfDP() {
		return numberOfDP;
	}
	public void setNumberOfDP(int numberOfDP) {
		this.numberOfDP = numberOfDP;
	}
	public int getNubmerOfPLLanmarks() {
		return nubmerOfPLLanmarks;
	}
	public void setNubmerOfPLLanmarks(int nubmerOfPLLanmarks) {
		this.nubmerOfPLLanmarks = nubmerOfPLLanmarks;
	}
	public int getNumberOfStubs() {
		return numberOfStubs;
	}
	public void setNumberOfStubs(int numberOfStubs) {
		this.numberOfStubs = numberOfStubs;
	}
	
	public long getExecutionTime() {
		return executionTime;
	}
	public void setExecutionTime(long executionTime) {
		this.executionTime = executionTime;
	}
	public double getGap() {
		return gap;
	}
	public void setGap(double gap) {
		this.gap = gap;
	}

	
	public double getScaleVariation() {
		return scaleVariation;
	}
	public void setScaleVariation(double scaleVariation) {
		this.scaleVariation = scaleVariation;
	}
	public SoftConstraintValues getOrientationSC() {
		return orientationSC;
	}
	public void setOrientationSC(SoftConstraintValues orientationSC) {
		this.orientationSC = orientationSC;
	}
	public SoftConstraintValues getDistanceSC() {
		return distanceSC;
	}
	public void setDistanceSC(SoftConstraintValues distanceSC) {
		this.distanceSC = distanceSC;
	}
	public SoftConstraintValues getProportionSC() {
		return proportionSC;
	}
	public void setProportionSC(SoftConstraintValues proportionSC) {
		this.proportionSC = proportionSC;
	}
	public SoftConstraintValues getProportionDPSC() {
		return proportionDPSC;
	}
	public void setProportionDPSC(SoftConstraintValues proportionDPSC) {
		this.proportionDPSC = proportionDPSC;
	}
	public SoftConstraintValues getBendSC() {
		return bendSC;
	}
	public void setBendSC(SoftConstraintValues bendSC) {
		this.bendSC = bendSC;
	}
	public int getExecutions() {
		return executions;
	}
	public void setExecutions(int executions) {
		this.executions = executions;
	}
	public int getFixedExtraCrossings() {
		return fixedExtraCrossings;
	}
	public void setFixedExtraCrossings(int fixedExtraCrossings) {
		this.fixedExtraCrossings = fixedExtraCrossings;
	}
	

	public double getPathLength() {
		return pathLength;
	}
	public void setPathLength(double pathLength) {
		this.pathLength = pathLength;
	}
	
	
	public double getObjectiveFunctionValue() {
		return objectiveFunctionValue;
	}
	public void setObjectiveFunctionValue(double objectiveFunctionValue) {
		this.objectiveFunctionValue = objectiveFunctionValue;
	}
	@Override
	public String toString() {
		return "RouteSchematizationReport [numberOfNodes=" + numberOfNodes + ", nubmerOfRouteNodes="+ nubmerOfRouteNodes + ", numberOfEdges=" + numberOfEdges + ", numberOfDP=" + numberOfDP
				+ ", nubmerOfPLLanmarks=" + nubmerOfPLLanmarks + ", numberOfStubs=" + numberOfStubs + ", scaleVariation="
				+ scaleVariation + ", pathLength=" + pathLength 
				+ "\n Bend SC=" + bendSC
				+ "\n Edge Dir SC=" + orientationSC 
				+ "\n Distance SC=" + distanceSC 
				+ "\n Proportion SC=" + proportionSC 
				+ "\n Proportion DP SC=" + proportionDPSC   
				+ "\n executionTime=" + executionTime + ", gap=" + gap +", obj. value =" + objectiveFunctionValue + ", executions=" + executions + ", fixedExtraCrossings=" + fixedExtraCrossings + "]";
	}
	public String toCSV() {
		
		DecimalFormat resultdf = new DecimalFormat("###.###");
		DecimalFormat weightdf = new DecimalFormat("####.#");
		String text = " n , m , n(r) , DPs, stubs ,  lenght , fixed cross , ILP exec. , time , W(bend) , R(bend), W(dir) , R(dir), W(dist) , R(dist), W(prop) , R(prop), W(propDP) , R(propDP)\n ";
		text = text.concat( numberOfNodes + " , " +  numberOfEdges +  " , " + nubmerOfRouteNodes  +  " , " +  numberOfDP  +  " , " + numberOfStubs  +  " , " +  resultdf.format(pathLength)  +  " , " + fixedExtraCrossings  +  " , " + executions  +  " , " + executionTime  +  " , " 
		+ weightdf.format(bendSC.getWeight()) + "/b"  +  " , " +  resultdf.format(bendSC.getResult()) +  " , " 
		+ weightdf.format(orientationSC.getWeight()) + "/m"  +  " , " + resultdf.format(orientationSC.getResult())  +  " , " 
		+ weightdf.format(distanceSC.getWeight())  + "/n" +  " , " + resultdf.format(distanceSC.getResult())  +  " , " 
		+  weightdf.format(proportionSC.getWeight()) + "/m" +  " , " +  resultdf.format(proportionSC.getResult()) +  " , "  
		+  weightdf.format(proportionDPSC.getWeight()) + "/DP"+  " , " +  resultdf.format(proportionDPSC.getResult()) +  " \n "  );
		return text;
	}
	
	

}
