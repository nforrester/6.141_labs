package MotorControl;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;

/**
 * Created on Jan 28, 2005
 *
 * @author David Blau
 *
 * TODO To change the template for this generated type comment go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
public class GraphPanel extends JPanel {
	
	private int labelOffsetX = 20;
	private int labelOffsetY = 20;
	private int graphSizeX;
	private int graphSizeY;
	private int originX;
	private int originY;
	private double xScale;
	private double yScale;
	
	private double[] data;
	private double maxY;
	private double minY;
	
	public boolean drawAxes = true;
	public boolean drawPoints = false;
	
	public GraphPanel(int w, int h, int numPoints){
		super.setPreferredSize(new Dimension(w,h));
		graphSizeX = w - labelOffsetX;
		graphSizeY = h - labelOffsetY;
		
		originX = labelOffsetX;
		originY = graphSizeY/2;
		
		maxY=10.0d;
		minY=-10.0d;
		
		data = new double[numPoints];
		xScale = (double)graphSizeX/numPoints;
		yScale = graphSizeY/(maxY-minY);
		
		clearGraph();
	}
	
	public void clearGraph(){
		for(int i=0;i<data.length;i++)
			data[i]=0;
	}
	
	public void log(double datum){
		for(int i=0;i<data.length-1;i++)
			data[i]=data[i+1];
		data[data.length-1]=datum;
		
		/*if(datum>maxY){
			maxY=datum;
			yScale=graphSizeY/(maxY-minY);
		}
		if(datum<minY){
			minY=datum;
			yScale=graphSizeY/(maxY-minY);
		}
		originY =(int) ((maxY/(maxY-minY))*graphSizeY);*/ 
		this.repaint();
	}
	
	public void paintComponent(Graphics g){
		g.setColor(Color.white);
		//g.fillRect(0, 0, labelOffsetX + graphSizeX,labelOffsetY + graphSizeY);
		g.fillRect(0,0,this.getWidth(),this.getHeight());
		g.setColor(Color.black);
		
		if(drawAxes){
			g.drawLine(labelOffsetX, graphSizeY, labelOffsetX, 0);//y axis
			g.drawLine(originX, originY, labelOffsetX + graphSizeX, originY);//x axis
			g.drawString("t",labelOffsetX+graphSizeX/2,labelOffsetY/2+graphSizeY);
			g.drawString("v",0,graphSizeY/2);
			g.drawString("0",labelOffsetX-10,originY+5);
			g.drawString(" "+(int)maxY,labelOffsetX-20,5);
			g.drawString(""+(int)minY,labelOffsetX-20,graphSizeY+5);
			g.setColor(Color.gray);
			for(int i = -8;i<=8;i=i+2){
				if(i!=0){
					g.drawString(""+i,labelOffsetX-10,originY + 5 - (int)(i*yScale));
					g.drawLine(originX, originY - (int)(i*yScale),
							labelOffsetX + graphSizeX, originY - (int)(i*yScale));//y = 2
				}
			}
			
		}
		g.setColor(Color.red);
		for(int i=0;i<data.length-1;i++){
			if(drawPoints) g.drawLine(originX + (int)(i * xScale), originY - (int)(data[i] * yScale),
					originY + (int)(i * xScale), originY - (int)(data[i] * yScale));
			else
				g.drawLine(originX + (int)(i * xScale), originY - (int)(data[i] * yScale),
						originX + (int)((i+1) * xScale), originY - (int)(data[i+1] * yScale));
		}
	}
	
	/*
	public static void main(String[] args){
		JFrame f = new JFrame();
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		final GraphPanel gp = new GraphPanel(300,200,100);
		f.setContentPane(gp);
		new Timer(100,new ActionListener(){
			Random r = new Random();
			int ctr =0;
			public void actionPerformed(ActionEvent e){
				gp.log(r.nextDouble()*20.0d-10.0d);
				ctr++;
			}
		}).start();
		f.pack();
		f.setVisible(true);
	}
     **/
}
