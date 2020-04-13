package com.wayto.controller;


import javax.swing.table.AbstractTableModel;

import com.wayto.mapFeaturesModel.MapContent;

public class LayerTableModel extends AbstractTableModel{


	private MapContent content;
	
	public LayerTableModel(MapContent content) {
		super();
		this.content = content;
	}

	@Override
	public String getColumnName(int col) {
        
		switch (col) {
		case 0:
			return "Layer";
		case 1:
			return "Visible";

		default:
			return " ";

		}
		
    }
	
	@Override
	public Class<? extends Object> getColumnClass(int c) {
	        return getValueAt(0, c).getClass();
	    }
	
	public int getRowCount() {
		// TODO Auto-generated method stub
		return content.getLayers().size();
	}

	public int getColumnCount() {
		// TODO Auto-generated method stub
		return 2;
	}

	public Object getValueAt(int rowIndex, int columnIndex) {
		if(columnIndex == 0)
			return content.getLayers().get(rowIndex).getName();
		else if(columnIndex == 1)
			return content.getLayers().get(rowIndex).isVisible();
		return null;
	}
	
	@Override
	public boolean isCellEditable(int row, int col) {
        //Note that the data/cell address is constant,
        //no matter where the cell appears onscreen.
        if (col == 1) {
            return true;
        } else {
            return true;
        }
        
    }

	


}
