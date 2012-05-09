package Challenge;

import LocalNavigation.Mat;
import java.util.Random;

/**
 * <p>Rapidly Exploring Random Tree</p>
 *
 * @author nforrest
 *
 **/
public class RapRandTree {
	static Random rand = new Random();

	public ArrayList<Waypoint> findWaypoints(CSpace cspace, Mat start, Mat end)
		assert cspace.pointInCSpace(start);

		double delta = 0.1;
		ArrayList<Mat> nodes = new ArrayList<Mat>();
		ArrayList<int> parents = new ArrayList<int>();
		int nnodes = 0;
		nodes.add(start);
		parents.add(-1);
		nnodes++;

		Mat nextTarget;
		int closeNode;
		Mat segIncrement;
		Mat point;
		double dist, minDist;
		int i;
		boolean lineSegBroken;
		boolean seekingGoal;
		while (true) {
			if (rand.nextFloat() < 0.9) {
				nextTarget = randomPoint()
				seekingGoal = false;
			} else {
				nextTarget = goal;
				seekingGoal = true;
			}
			minDist = xMax - xMin + yMax - yMin;
			for (Mat node: nodes) {
				dist = Mat.dist(node, nextTarget);
				if (dist < minDist) {
					nodes.get(closeNode) = node;
					minDist = dist;
				}
			}

			segIncrement = Mat.mul(delta / minDist, Mat.sub(nextTarget, nodes.get(closeNode)));
			lineSegBroken = false;
			for (i = 0; i < minDist / delta; i++) {
				point = Mat.add(nodes.get(closeNode), Mat.mul(i, segIncrement))
				if (cspace.lineSegInCSpace(nodes.get(closeNode), point)) {
					nodes.add(point);
					parents.add(closeNode);
					closeNode = nnodes;
					nnodes++;
				} else {
					lineSegBroken = true;
					break;
				}
			}
			if (!lineSegBroken && seekingGoal) {
				if (cspace.lineSegInCSpace(nodes.get(closeNode), nextTarget)) {
					nodes.add(nextTarget);
					parents.add(closeNode);
					closeNode = nnodes;
					nnodes++;
					break;
				}
			}
		}

		int thisNode = nnodes - 1;
		ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
		while (thisNode != 0) {
			double pt[] = Mat.decodePoint(nodes.get(thisNode));
			waypoints.add(0, new Waypoint(pt[0], pt[1]));
			thisNode = parents.get(thisNode);
		}
	}

	private Mat randomPoint() {
		Mat point;
		double x;
		double y;
		do {
			x = rand.nextFloat() * (cspace.xMax - cspace.xMin) + cspace.xMin;
			y = rand.nextFloat() * (cspace.yMax - cspace.yMin) + cspace.yMin;
			point = Mat.encodePoint(x, y);
		} while (!cspace.pointInCSpace(point));
		return point;
	}
}
