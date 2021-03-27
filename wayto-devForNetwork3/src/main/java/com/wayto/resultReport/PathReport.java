package com.wayto.resultReport;

import java.text.DecimalFormat;

public class PathReport {

	
	private String pathId;
	private String pathName;
	private String pathType;
	private int numberOfNodes;
	private int numberOfEdges;	
	private double pathLenght;
	private double proportion;
	
	private SoftConstraintValues orientationSC;
	private SoftConstraintValues distanceWeightSC;
	private SoftConstraintValues distanceTopoWeightSC;
	private SoftConstraintValues proportionWeightSC;
	private SoftConstraintValues bendSC;
	private double objectiveFunctionValue;
	
	private int executions;
	private int fixedExtraCrossings;

	private long executionTime; 
	private double gap;
	
			
	public String getPathId() {
		return pathId;
	}
	public void setPathId(String pathId) {
		this.pathId = pathId;
	}
	public String getPathName() {
		return pathName;
	}
	public void setPathName(String name) {
		String reportName;
		if(name.length() > 5) {
			reportName = name.substring(0, 5);
			reportName = reportName.concat(".");
		}	
		else
			reportName = name;
		this.pathName = reportName;
	}
	public double getPathLenght() {
		return pathLenght;
	}
	public void setPathLenght(double pathLenght) {
		this.pathLenght = pathLenght;
	}
	
	public double getProportion() {
		return proportion;
	}
	public void setProportion(double proportion) {
		this.proportion = proportion;
	}
	public SoftConstraintValues getOrientationSC() {
		return orientationSC;
	}
	public void setOrientationSC(SoftConstraintValues orientationSC) {
		this.orientationSC = orientationSC;
	}
	public SoftConstraintValues getDistanceWeightSC() {
		return distanceWeightSC;
	}
	public void setDistanceWeightSC(SoftConstraintValues distanceWeightSC) {
		this.distanceWeightSC = distanceWeightSC;
	}
	public SoftConstraintValues getDistanceTopoWeightSC() {
		return distanceTopoWeightSC;
	}
	public void setDistanceTopoWeightSC(SoftConstraintValues distanceTopoWeightSC) {
		this.distanceTopoWeightSC = distanceTopoWeightSC;
	}
	public SoftConstraintValues getProportionWeightSC() {
		return proportionWeightSC;
	}
	public void setProportionWeightSC(SoftConstraintValues proportionWeightSC) {
		this.proportionWeightSC = proportionWeightSC;
	}
	public SoftConstraintValues getBendSC() {
		return bendSC;
	}
	public void setBendSC(SoftConstraintValues bendSC) {
		this.bendSC = bendSC;
	}
	public int getNumberOfNodes() {
		return numberOfNodes;
	}
	public void setNumberOfNodes(int numberOfNodes) {
		this.numberOfNodes = numberOfNodes;
	}
	public int getNumberOfEdges() {
		return numberOfEdges;
	}
	public void setNumberOfEdges(int numberOfEdges) {
		this.numberOfEdges = numberOfEdges;
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
	public double getObjectiveFunctionValue() {
		return objectiveFunctionValue;
	}
	public void setObjectiveFunctionValue(double objectiveFunctionValue) {
		this.objectiveFunctionValue = objectiveFunctionValue;
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
	
	
	public String getPathType() {
		return pathType;
	}
	public void setPathType(int pathType) {
		
		String typeName;
		
		switch (pathType) {
		case 01:
			typeName = "cross";
			break;
		case 10:
			typeName = "along";
			break;
		case 11:
			typeName = "along";
			break;
		case 100:
			typeName = "along";
			break;
		case 101:
			typeName = "along";
			break;
		case 110:
			typeName = "starts";
			break;
		case 111:
			typeName = "ends";
			break;
		case 1000:
			typeName = "global";
			break;
		case 1001:
			typeName = "global";
			break;
		case 1011:
			typeName = "along";
			break;

		default:
			typeName = "unknown";
			break;
		}
		
		this.pathType = typeName;
	}
	@Override
	public String toString() {
		return " PathId=" + pathId + ", pathName=" + pathName + ", type=" + pathType + ", numberOfNodes=" + numberOfNodes + ", numberOfEdges=" + numberOfEdges + ", pathLenght=" + pathLenght + ", proportion=" + proportion 
				+ "\n Bend SC=" + bendSC 
				+ "\n DIR Edge SC = " + orientationSC
				+ "\n Dist. SC = " + distanceWeightSC 
				+ "\n Dist. TOPO SC = " + distanceTopoWeightSC
				+ "\n Proportion SC = " + proportionWeightSC 
				+ "\n Obj. Funciton ="+ objectiveFunctionValue + ", execution time=" + executionTime + ", gap=" + gap 
				+ "\n exexutions="+ executions + ", fixedExtraCrossings=" + fixedExtraCrossings +"\n\n";
	}
	public String asCSV() {
		DecimalFormat resultdf = new DecimalFormat("###.###");
		DecimalFormat weightdf = new DecimalFormat("####.#");
		DecimalFormat propdf = new DecimalFormat("####.##");
		
		/* 		String pathText  = "id, n , m , proportion , type ,  name , length , fixed X, ILP exec. , time , W(bend) , R(bend), W(dir) , R(dir), W(dist) , R(dist), W(distCN) , R(distCN), W(prop) , R(prop)\n ";*/
		String text = new String();
		text = text.concat("path " + pathId + " , " + numberOfNodes + " , " +  numberOfEdges +  " , " + propdf.format(proportion)  +  " , " +  pathType  +  " , " + pathName  +  " , " +  resultdf.format(pathLenght)  +  " , " + fixedExtraCrossings  +  " , " + executions  +  " , " + executionTime  +  " , " 
		//+ weightdf.format(bendSC.getWeight()) + "/b"  +  " , " +  weightdf.format(bendSC.getNormalizedWeight()) +" , " +  resultdf.format(bendSC.getResult()) +  " , " 
		+ weightdf.format(orientationSC.getWeight()) + "/m"  +  " , " + weightdf.format(orientationSC.getNormalizedWeight()) +" , " + resultdf.format(orientationSC.getResult())  +  " , " 
		//+ weightdf.format(distanceWeightSC.getWeight())  + "/n" +  " , " + weightdf.format(distanceWeightSC.getNormalizedWeight()) +" , " + resultdf.format(distanceWeightSC.getResult())  +  " , " 
		//+  weightdf.format(distanceTopoWeightSC.getWeight())  +  " , " +  weightdf.format(distanceTopoWeightSC.getNormalizedWeight()) +" , " + resultdf.format(distanceTopoWeightSC.getResult()) +  " , "  
		+  weightdf.format(proportionWeightSC.getWeight()) + "/m"+  " , " + weightdf.format(proportionWeightSC.getNormalizedWeight()) +" , " + resultdf.format(proportionWeightSC.getResult()) +  " \n "  );
		
		
		return text;
	}
	
	
	
}
