package LocalNavigation;

import java.awt.Color;

import org.ros.message.MessageListener;
import org.ros.message.lab5_msgs.GUILineMsg;

public class LineMessageListener implements MessageListener<GUILineMsg> {

	private SonarGUI gui;

	public LineMessageListener(SonarGUI sonarGUI) {
		this.gui = sonarGUI;
	}

	@Override
	public void onNewMessage(GUILineMsg msg) {
		int r = (int) msg.color.r;
		int g = (int) msg.color.g;
		int b = (int) msg.color.b;
		if (r<0 || g < 0 || b < 0){
			gui.setLine(msg.lineA, msg.lineB, msg.lineC);
		} else{
			Color c = new Color(r, g, b);
			gui.setLine(msg.lineA, msg.lineB, msg.lineC, c);
		}
	}

}
