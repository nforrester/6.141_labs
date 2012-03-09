package LocalNavigation;

import java.awt.Color;

import org.ros.message.MessageListener;
import org.ros.message.lab5_msgs.GUIPointMsg;

public class PointMessageListener implements MessageListener<GUIPointMsg> {

	private SonarGUI gui;

	public PointMessageListener(SonarGUI sonarGUI) {
		this.gui = sonarGUI;
	}

	@Override
	public void onNewMessage(GUIPointMsg msg) {
		int r = (int) msg.color.r;
		int g = (int) msg.color.g;
		int b = (int) msg.color.b;

		if (r<0 || g < 0 || b < 0){
			gui.addPoint(msg.x, msg.y, (int) msg.shape);
		} else{
			Color c = new Color(r, g, b);
			gui.addPoint(msg.x, msg.y, (int) msg.shape, c);
		}
	}

}
