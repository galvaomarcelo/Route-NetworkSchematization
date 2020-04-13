package com.wayto.model.topo;

public class NodeTopoRelation {
	
	
	private boolean isPointLikeLM = false;
	private int featureId;
	private int type;
	
	public static final int ANCHOR = 100;
	
	public static final int CROSSING = 200;
	
	public static final int ALONG_UNKNOWN = 300;
	public static final int ALONG_RIGHT_START = 310;
	public static final int ALONG_RIGHT_END = 311;
	public static final int ALONG_LEFT_START = 320;
	public static final int ALONG_LEFT_END = 321;


	public static final int GLOBAL_UNKNOWN = 400;
	public static final int GLOBAL_RIGHT_START = 410;
	public static final int GLOBAL_RIGHT_END = 411;
	public static final int GLOBAL_LEFT_START = 420;
	public static final int GLOBAL_LEFT_END = 421;
	
	public static final int ALONG_UNKNOWN_PTLM = 500;
	public static final int ALONG_RIGHT_PTLM = 510;
	public static final int ALONG_LEFT_PTLM = 520;
	
	public static final int GLOBAL_UNKNOWN_PTLM = 600;
	public static final int GLOBAL_RIGHT_PTLM = 610;
	public static final int GLOBAL_LEFT_PTLM = 620;
	
	public NodeTopoRelation(int featureId, boolean isPointLikeLM, int type) {
		super();
		this.featureId = featureId;
		this.type = type;
		this.isPointLikeLM = isPointLikeLM;
	}

	public int getFeatureId() {
		return featureId;
	}

	public void setFeatureId(int featureId) {
		this.featureId = featureId;
	}

	public int getType() {
		return type;
	}

	public void setType(int type) {
		this.type = type;
	}

	public boolean isPointLikeLM() {
		return isPointLikeLM;
	}

	public void setPointLikeLM(boolean isPointLikeLM) {
		this.isPointLikeLM = isPointLikeLM;
	}
	
	
	

	
	

}
