package GlobalNavigation;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;

public class OdometryListener implements MessageListener<OdometryMsg> {

	private GlobalNavigation nav;

	public OdometryListener(GlobalNavigation globalNavigation) {
		// TODO Auto-generated constructor stub
		nav = globalNavigation;
	}

	@Override
	public void onNewMessage(OdometryMsg arg0) {
		// TODO Auto-generated method stub
		nav.handle(arg0);
	}

}
