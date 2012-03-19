package GlobalNavigation;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.BumpMsg;

public class BumpListener implements MessageListener<BumpMsg> {
	
	GlobalNavigation nav;
	
	public BumpListener(GlobalNavigation gn){
		nav = gn;
	}

	@Override
	public void onNewMessage(BumpMsg arg0) {
		// TODO Auto-generated method stub
		nav.handle(arg0);
	}

}
