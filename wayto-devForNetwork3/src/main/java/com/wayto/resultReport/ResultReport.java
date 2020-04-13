package com.wayto.resultReport;

import java.text.DecimalFormat;
import java.util.ArrayList;

public class ResultReport {

	private RouteSchematizationReport routeReport;
	private ArrayList<PathReport> pathReportList;
	private int nubmerOfPointLikeLM = 0;
	private int numberOfPolygonalLM = 0;
	private double lenghtKm; 
	public ResultReport(int pointLM, int polyLM) {
		super();
		routeReport = new RouteSchematizationReport();
		pathReportList = new ArrayList<PathReport>();
		numberOfPolygonalLM = polyLM;
		nubmerOfPointLikeLM = pointLM;
	}
	public RouteSchematizationReport getRouteReport() {
		return routeReport;
	}
	public void setRouteReport(RouteSchematizationReport routeReport) {
		this.routeReport = routeReport;
	}
	public ArrayList<PathReport> getPathReportList() {
		return pathReportList;
	}
	public void setPathReportList(ArrayList<PathReport> pathReportList) {
		this.pathReportList = pathReportList;
	}
	
	public double getLenghtKm() {
		return lenghtKm;
	}
	public void setLenghtKm(double lenghtKm) {
		this.lenghtKm = lenghtKm;
	}
	@Override
	public String toString() {
		long totalExecutionTime = routeReport.getExecutionTime();
		int totalNumberOfNodes = routeReport.getNumberOfNodes();
		int totaNumberOfEdges = routeReport.getNumberOfEdges();
		int totalNumberOfPaths = 0;
		int totalOfExtraCrossFixed = routeReport.getFixedExtraCrossings();
		int totalILPExecutions = routeReport.getExecutions();
		for(PathReport pr: pathReportList) {
			totalExecutionTime += pr.getExecutionTime();
			totalNumberOfNodes += pr.getNumberOfNodes();
			totaNumberOfEdges += pr.getNumberOfEdges();
			totalNumberOfPaths++;
			totalOfExtraCrossFixed += pr.getFixedExtraCrossings();
			totalILPExecutions += pr.getExecutions();
			
		}
		
		return "RESULT REPORT: \n"
			+ "Total Exec. Time = " + totalExecutionTime + " Polygonal LM = " + numberOfPolygonalLM + " Point LM = " + nubmerOfPointLikeLM + " Total Nodes = " + totalNumberOfNodes + " Total Edges = " + totaNumberOfEdges + " Total Paths = " + totalNumberOfPaths	 + " Total Edges Cross Fixed = " + totalOfExtraCrossFixed + " Total ILP execution = " + totalILPExecutions			
			+ "\n \n Rute Report: " + routeReport 
			+ "\n \n Path Report List:\n \\n" 
			+ pathReportList ;
	}
	
	public String toCSV() {
		
		String pathText  = "id, n , m , proportion , type ,  name , length , fixed X, ILP exec. , time , W(bend) , R(bend), W(dir) , R(dir), W(dist) , R(dist), W(distCN) , R(distCN), W(prop) , R(prop)\n ";
		long totalExecutionTime = routeReport.getExecutionTime();
		int totalNumberOfNodes = routeReport.getNumberOfNodes();
		int totaNumberOfEdges = routeReport.getNumberOfEdges();
		int totalNumberOfPaths = 0;
		int totalOfExtraCrossFixed = routeReport.getFixedExtraCrossings();
		int totalILPExecutions = routeReport.getExecutions();
		for(PathReport pr: pathReportList) {
			totalExecutionTime += pr.getExecutionTime();
			totalNumberOfNodes += pr.getNumberOfNodes();
			totaNumberOfEdges += pr.getNumberOfEdges();
			totalNumberOfPaths++;
			totalOfExtraCrossFixed += pr.getFixedExtraCrossings();
			totalILPExecutions += pr.getExecutions();
			pathText = pathText.concat(pr.asCSV() );
		}
		DecimalFormat lenghtdf = new DecimalFormat("####.#");
		String text = "n , e , length(km) , rescale , paths , poly. LM , point LM , fixed X , ILP exec. , time \n";
		text = text.concat(totalNumberOfNodes + " , " + totaNumberOfEdges  + " , " + lenghtdf.format(lenghtKm/1000)  + " , " + routeReport.getScaleVariation()  + " , " + totalNumberOfPaths + " , " + numberOfPolygonalLM + " , " + nubmerOfPointLikeLM + " , " + totalOfExtraCrossFixed +  " , " + totalILPExecutions + " , " + totalExecutionTime  + "\n\n\n");
		text = text.concat(routeReport.toCSV() + "\n\n");		
		text = text.concat(pathText );
		
		return text;
	}
	public String generalCSV() {
		long totalExecutionTime = routeReport.getExecutionTime();
		int totalNumberOfNodes = routeReport.getNumberOfNodes();
		int totaNumberOfEdges = routeReport.getNumberOfEdges();
		int totalNumberOfPaths = 0;
		int totalOfExtraCrossFixed = routeReport.getFixedExtraCrossings();
		int totalILPExecutions = routeReport.getExecutions();
		for(PathReport pr: pathReportList) {
			totalExecutionTime += pr.getExecutionTime();
			totalNumberOfNodes += pr.getNumberOfNodes();
			totaNumberOfEdges += pr.getNumberOfEdges();
			totalNumberOfPaths++;
			totalOfExtraCrossFixed += pr.getFixedExtraCrossings();
			totalILPExecutions += pr.getExecutions();
			
		}
		DecimalFormat lenghtdf = new DecimalFormat("####.#");
		String text = "n , m , length(km) , rescale , paths , poly. LM , point LM , fixed X , ILP exec. , time \n";
		text = text.concat(totalNumberOfNodes + " , " + totaNumberOfEdges  + " , " + lenghtdf.format(lenghtKm/1000)  + " , " + routeReport.getScaleVariation()  + " , " + totalNumberOfPaths + " , " + numberOfPolygonalLM + " , " + nubmerOfPointLikeLM + " , " + totalOfExtraCrossFixed +  " , " + totalILPExecutions + " , " + totalExecutionTime  + "\n\n\n");

		return text;
	}
	public String routeCSV() {
		String text = routeReport.toCSV();
		return text;
	}
	public String pathsCSV() {
		String pathText  = "id, n , m , prop. , type ,  name , length , fixed X, ILP exec. , time , W(bend) , Wn(bend) , R(bend), W(dir), Wn(dir) , R(dir), W(dist), Wn(dist) , R(dist), W(distCN) , Wn(distCN) , R(distCN), W(prop) , Wn(prop) , R(prop)\n ";
		for(PathReport pr: pathReportList) {
			pathText = pathText.concat(pr.asCSV() );
		}
		return pathText;
	}
	
	
	
	
	
	
}
