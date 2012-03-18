package LocalNavigation;

import java.awt.Color;

import org.ros.message.MessageListener;
import org.ros.message.lab5_msgs.GUISegmentMsg;

public class SegmentMessageListener implements MessageListener<GUISegmentMsg> {

	private SonarGUI gui;

	public SegmentMessageListener(SonarGUI sonarGUI) {
		this.gui = sonarGUI;
	}

	@Override
	public void onNewMessage(GUISegmentMsg msg) {
		int r = (int) msg.color.r;
		int g = (int) msg.color.g;
		int b = (int) msg.color.b;
		if (r<0 || g < 0 || b < 0){
			gui.addSegment(msg.startX, msg.startY, msg.endX, msg.endY);
		} else{
			Color c = new Color(r, g, b);
			gui.addSegment(msg.startX, msg.startY, msg.endX, msg.endY, c);
		}
	}

}
