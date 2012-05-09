package Challenge;

import LocalNavigation.Mat;
import java.util.Random;
import java.util.ArrayList;

/**
 * <p>Rapidly Exploring Random Tree</p>
 *
 * @author nforrest
 *
 **/
public class RapRandTree {
	static Random rand = new Random();

	public static ArrayList<Waypoint> findWaypoints(CSpace cspace, Mat start, Mat end) throws Exception {
		assert cspace.pointInCSpace(start);
		assert cspace.pointInCSpace(end);

		System.err.println("Starting to make Rapidly-exploring Random Tree");
		System.err.println("start = " + pts(start));
		System.err.println("end   = " + pts(end));

		double delta = 0.1;
		ArrayList<Mat> nodes = new ArrayList<Mat>();
		ArrayList<Integer> parents = new ArrayList<Integer>();
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
			if (nnodes > 1000) {
				System.err.println("Goal not accessible!");
				throw new Exception("Goal not accessible!");
			}

			if (rand.nextFloat() < 0.9) {
				nextTarget = randomPoint(cspace);
				seekingGoal = false;
				//System.err.println("not seeking goal.");
			} else {
				nextTarget = end;
				seekingGoal = true;
				//System.err.println("SEEKING GOAL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			}

			//System.err.println("nextTarget = " + pts(nextTarget));

			minDist = cspace.xMax - cspace.xMin + cspace.yMax - cspace.yMin;
			closeNode = 0;
			for (i = 0; i < nodes.size(); i++) {
				dist = Mat.dist(nodes.get(i), nextTarget);
				if (dist < minDist) {
					closeNode = i;
					minDist = dist;
				}
			}
			//System.err.println("closeNode = " + closeNode + ", " + pts(nodes.get(closeNode)));

			segIncrement = Mat.mul(delta / minDist, Mat.sub(nextTarget, nodes.get(closeNode)));
			for (i = 0; i < minDist / delta; i++) {
				point = Mat.add(nodes.get(closeNode), Mat.mul(i, segIncrement));
				if (cspace.lineSegInCSpace(nodes.get(closeNode), point)) {
					//System.err.println("Adding node " + nnodes + ": " + pts(point));
					nodes.add(point);
					parents.add(closeNode);
					nnodes++;
				}
			}
			if (seekingGoal) {
				//System.err.println("Goal not plausibly in reach.");
				if (cspace.lineSegInCSpace(nodes.get(closeNode), nextTarget)) {
					//System.err.println("Goal in reach!");
					nodes.add(nextTarget);
					parents.add(closeNode);
					nnodes++;
					break;
				}
			} else {
				//System.err.println("Goal not plausibly in reach.");
			}
		}

		int thisNode = nnodes - 1;
		ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
		while (thisNode != 0) {
			double pt[] = Mat.decodePoint(nodes.get(thisNode));
			waypoints.add(0, new Waypoint(pt[0], pt[1], (short) 1));
			thisNode = parents.get(thisNode);
		}
		System.err.println("Rapidly-exploring Random Tree is done! nnodes = " + nnodes);
		return waypoints;
	}

	private static Mat randomPoint(CSpace cspace) {
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

	private static String pts(Mat point) {
		double[] pt = Mat.decodePoint(point);
		return "(" + pt[0] + " " + pt[1] + ")";
	}
}
