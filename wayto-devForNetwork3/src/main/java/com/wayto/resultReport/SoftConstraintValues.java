package com.wayto.resultReport;

public class SoftConstraintValues {
	
	private double weight;
	private double normalizedWeight;
	private double result;
	
	
	
	public SoftConstraintValues(double weight, double normalizedWeight, double result) {
		super();
		
		this.weight = Double.valueOf(weight).isNaN() ? 0 : weight;
		this.normalizedWeight = Double.valueOf(normalizedWeight).isNaN() ? 0 : normalizedWeight;
		this.result = Double.valueOf(result).isNaN() ? 0 : result;
	}
	public double getWeight() {
		return weight;
	}
	public void setWeight(double weight) {
		this.weight = weight;
	}
	public double getNormalizedWeight() {
		return normalizedWeight;
	}
	public void setNormalizedWeight(double normalizedWeight) {
		this.normalizedWeight = normalizedWeight;
	}
	public double getResult() {
		return result;
	}
	public void setResult(double result) {
		this.result = result;
	}
	@Override
	public String toString() {
		return "SoftConstraintValues [weight=" + weight + ", normalizedWeight=" + normalizedWeight + ", result="
				+ result + "]";
	}
	

}

