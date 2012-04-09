package Grasping;

/*
 * ArmPoseGUI.java
 *
 * @author edsinger
 * @author prentice
 */

import java.awt.FlowLayout;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.border.*;
import javax.swing.event.*;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyListener;
import java.awt.event.KeyEvent;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;

import org.ros.message.rss_msgs.ArmMsg;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.namespace.GraphName;

import java.text.*;

/**
 *
 */
public class ArmPoseGUI extends JFrame implements NodeMain{

	/**
	 *
	 */
	private Container cp;

	/**
	 *
	 */
	ChannelPanel channel0;

	/**
	 *
	 */
	ChannelPanel channel1;

	/**
	 *
	 */
	ChannelPanel channel2;

	/**
	 *
	 */
	ChannelPanel channel3;

	/**
	 *
	 */
	JFrame frame;

	/**
	 *
	 */
	long[] armTheta=new long[]{0,0,0,0,0,0,0,0};

	/**
	 * 
	 */
	ArmVisualizerPanel avPanel;

	private Publisher<ArmMsg> armPub;


	/**
	 * Constructor for ArmPoseGUI
	 */
	public ArmPoseGUI() {
		channel0 = new ChannelPanel("Shoulder",0);
		channel1 = new ChannelPanel("Wrist",1);
		channel2 = new ChannelPanel("Gripper",2);
		channel3 = new ChannelPanel("N/A",3);
		avPanel = new ArmVisualizerPanel();

		/** Adding components to container */
		cp = getContentPane();
		BoxLayout bLayout = new BoxLayout(cp, BoxLayout.Y_AXIS);
		cp.setLayout(new BorderLayout());
		Box top = Box.createHorizontalBox();
		top.add(channel0);
		top.add(Box.createHorizontalStrut(5));
		top.add(channel1);
		top.add(Box.createHorizontalStrut(5));
		top.add(channel2);
		top.add(Box.createHorizontalStrut(5));
		top.add(channel3);
		top.add(Box.createVerticalStrut(10));
		top.add(avPanel);
		cp.add(top, BorderLayout.CENTER);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		pack();
		setTitle( "ArmPose" );
		setSize(1200,700);
		setVisible( true );
	}

	/**
	 * ChannelPanel
	 */
	class ChannelPanel extends JPanel{
		private JLabel desiredLabel;
		private JSlider desiredSlider;
		private JTextField desiredValue;
		private Box displayBox;
		private int channel;
		private ChannelPanelListener cpListener;

		/**
		 *
		 */
		public ChannelPanel(String title, int channel){
			super(new BorderLayout());
			this.channel = channel;
			cpListener = new ChannelPanelListener(this);
			displayBox = Box.createVerticalBox();
			displayBox.setPreferredSize(new java.awt.Dimension(500,20));
			displayBox.add(Box.createVerticalStrut(10));
			addSliderPanel();
			setBorder(new TitledBorder(new EtchedBorder(),title));
			add(displayBox, BorderLayout.CENTER );
		}

		/**
		 *
		 */
		private void addSliderPanel(){
			JPanel sliderPanel = new JPanel(new FlowLayout(FlowLayout.LEADING));

			desiredSlider = new JSlider(0,3000,1500);
			desiredSlider.setMajorTickSpacing(100);
			desiredSlider.setMinorTickSpacing(10);

			desiredSlider.setPaintTicks(false);//display tick marks
			desiredSlider.setPaintLabels(false);//display numbers
			desiredValue = new JTextField("0", 5);////<<---add a listener
			desiredValue.addActionListener(new ChannelPanelListener(this));
			desiredSlider.addChangeListener( new DesiredSliderListener(channel,desiredValue) );
			desiredValue.addActionListener(cpListener);
			desiredLabel = new JLabel("RawPwm");
			sliderPanel.add(desiredLabel);
			sliderPanel.add(desiredSlider);
			sliderPanel.add(desiredValue);
			displayBox.add(sliderPanel);
			displayBox.add(Box.createVerticalStrut(5));
		}

		/**
		 *
		 */
		class ChannelPanelListener implements ActionListener{
			private ChannelPanel cp;
			public ChannelPanelListener(ChannelPanel cp){
				this.cp=cp;
			}
			public void actionPerformed(ActionEvent e){
				desiredSlider.setValue(Integer.parseInt(desiredValue.getText()));
			}
		}

		/**
		 *
		 */
		class DesiredSliderListener implements ChangeListener{
			private int channel;
			private JTextField label;
			public DesiredSliderListener(int c, JTextField j){
				channel=c;
				label =j;
			}
			public void stateChanged(ChangeEvent e) {
				JSlider source = (JSlider)e.getSource();
				label.setText(Integer.toString(source.getValue()));
				long val = source.getValue();

				System.out.println("Channel: " + channel + " PWM: " + val);

				armTheta[channel]=val;

				// send pose to Orc
				ArmMsg msg = new ArmMsg();
				msg.pwms = armTheta;
				armPub.publish(msg);
			}
		}
	}
	//end ChannelPanel

	private void addArmVisualizerPanel(){
		avPanel = new ArmVisualizerPanel();

	}

    @Override public void onShutdownComplete(Node node) {
    }
    
    @Override public GraphName getDefaultNodeName() {
	return new GraphName("rss/armposegui");
    }

	@Override
	public void onShutdown(Node node) {
	}

	@Override
	public void onStart(Node node) {
		armPub = node.newPublisher("command/Arm", "rss_msgs/ArmMsg");
	}


} //end ArmPoseGui




