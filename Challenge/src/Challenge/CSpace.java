package Challenge;

import java.awt.geom.*;
import java.util.*;

import LocalNavigation.Mat;
import GlobalNavigation.PolygonObstacle;
import GlobalNavigation.GeomUtils;

/**
 * <p>Configuration space</p>
 *
 * @author nforrest
 *
 **/
public class CSpace {
	public Polygon reflectedRobot;

	public ArrayList<Polygon> obstacles = new ArrayList<Polygon>();
	public ArrayList<Polygon> csObstacles = new ArrayList<Polygon>();

	public double xMin, yMin, xMax, yMax;

	// robot reference point is the origin
	public CSpace(Polygon robot, GrandChallengeMap map) {
		Rectangle2D.Double worldRect = map.getWorldRect();
		constructor(robot, worldRect.getMinX(), worldRect.getMinY(), worldRect.getMaxX(), worldRect.getMaxY());
		for (PolygonObstacle obstacle : map.getPolygonObstacles()) {
			addObstacle(obstacle);
		}
	}

	public CSpace(Polygon robot, Rectangle2D.Double worldRect) {
		constructor(robot, worldRect.getMinX(), worldRect.getMinY(), worldRect.getMaxX(), worldRect.getMaxY());
	}

	public CSpace(Polygon robot, double boundaryXMin, double boundaryYMin, double boundaryXMax, double boundaryYMax) {
		constructor(robot, boundaryXMin, boundaryYMin, boundaryXMax, boundaryYMax);
	}

	private void constructor(Polygon robot, double boundaryXMin, double boundaryYMin, double boundaryXMax, double boundaryYMax) {

		xMin = boundaryXMin;
		yMin = boundaryYMin;
		xMax = boundaryXMax;
		yMax = boundaryYMax;

		reflectedRobot = Polygon.mul(Mat.mul(-1, Mat.eye(4)), robot);
		ArrayList<List<Mat>> boundaries = new ArrayList<List<Mat>>();

		boundaries.add(Arrays.asList(Mat.encodePoint(xMin - 0, yMin - 1),
		                             Mat.encodePoint(xMin - 0, yMax + 1),
		                             Mat.encodePoint(xMin - 1, yMax + 1),
		                             Mat.encodePoint(xMin - 1, yMin - 1)));

		boundaries.add(Arrays.asList(Mat.encodePoint(xMax + 0, yMin - 1),
		                             Mat.encodePoint(xMax + 0, yMax + 1),
		                             Mat.encodePoint(xMax + 1, yMax + 1),
		                             Mat.encodePoint(xMax + 1, yMin - 1)));

		boundaries.add(Arrays.asList(Mat.encodePoint(xMin - 1, yMin - 0),
		                             Mat.encodePoint(xMax + 1, yMin - 0),
		                             Mat.encodePoint(xMax + 1, yMin - 1),
		                             Mat.encodePoint(xMin - 1, yMin - 1)));

		boundaries.add(Arrays.asList(Mat.encodePoint(xMin - 1, yMax + 0),
		                             Mat.encodePoint(xMax + 1, yMax + 0),
		                             Mat.encodePoint(xMax + 1, yMax + 1),
		                             Mat.encodePoint(xMin - 1, yMax + 1)));

		for (List<Mat> boundary : boundaries) {
			addObstacle(new Polygon(boundary));
		}

	}

	public void addObstacle(PolygonObstacle obstacle) {
		ArrayList<Mat> vertices = new ArrayList<Mat>();
		for (Point2D.Double vert : obstacle.getVertices()) {
			vertices.add(Mat.encodePoint(vert.x, vert.y));
		}
		addObstacle(new Polygon(vertices));
	}

	public void addObstacle(Polygon obstacle) {
		obstacles.add(obstacle);
		csObstacles.add(Polygon.minkowskiSumSimple(obstacle, reflectedRobot));
		System.err.print("(obstacle");
		for (Mat vertex: obstacle.vertices) {
			double[] pt = Mat.decodePoint(vertex);
			System.err.print("\n    (vertex " + pt[0] + " " + pt[1] + ")");
		}
		System.err.println(")");
	}

	public boolean pointInCSpace(Mat point) {
		for (Polygon obstacle: csObstacles) {
			if (Polygon.pointInPolygon(obstacle, point)) {
				return false;
			}
		}
		return true;
	}

	public boolean lineSegInCSpace(Mat start, Mat end) {
		double delta = 0.01;
		double dist = Mat.dist(start, end);
		Mat segIncrement = Mat.mul(delta / dist, Mat.sub(end, start));
		for (int i = 0; i < dist / delta; i++) {
			if (!pointInCSpace(Mat.add(start, Mat.mul(i, segIncrement)))) {
				return false;
			}
		}
		return true;
	}

	// occupancy grid is indexed from (xMin, yMin)
	public boolean[][] getOccupancyGrid(int nCellsLinear) {
		System.err.println("GOT HERE YO!");
		int i, j;
		double xLow, xHigh, yLow, yHigh;
		double maxDimension = Math.max(xMax - xMin, yMax - yMin);
		double resolutionLinear = maxDimension / nCellsLinear;
		Polygon resolutionCellSpace;
		resolutionCellSpace = new Polygon(Arrays.asList(Mat.encodePoint(-1 * resolutionLinear / 2, -1 * resolutionLinear / 2),
								Mat.encodePoint(-1 * resolutionLinear / 2,      resolutionLinear / 2),
								Mat.encodePoint(     resolutionLinear / 2,      resolutionLinear / 2),
								Mat.encodePoint(     resolutionLinear / 2, -1 * resolutionLinear / 2)));
		reflectedRobot = Polygon.minkowskiSumSimple(reflectedRobot, resolutionCellSpace);
		System.err.println("GOT HERE as well.");
		boolean [][] occupancyGrid = new boolean[nCellsLinear][nCellsLinear];
		for (i = 0; i < nCellsLinear; i++) {
			xLow = xMin + resolutionLinear * i;
			xHigh = xLow + resolutionLinear;
			for (j = 0; j < nCellsLinear; j++) {
				yLow = yMin + resolutionLinear * j;
				yHigh = yLow + resolutionLinear;

				if (i == 9 && j == 6) {
					System.err.print("(" + i + ", " + j + ")     ");
					System.err.println("(" + ((xLow + xHigh) / 2) + ", " + ((yLow + yHigh) / 2) + ")");
				}

				occupancyGrid[i][j] = true;
				for (Polygon obstacle : csObstacles) {
					if (Polygon.pointInPolygon(obstacle, Mat.encodePoint((xLow + xHigh) / 2, (yLow + yHigh) / 2))) {
						occupancyGrid[i][j] = false;
						break;
					}
				}
				if (occupancyGrid[i][j]) {
					System.err.print(".");
				} else {
					System.err.print("#");
				}
			}
			System.err.println("");
		}

		System.err.println("GOT HERE TOO!");
		return occupancyGrid;
	}

	public static class Polygon {
		private static final double DEFAULT_TOLERANCE = 0.0001;
		public ArrayList<Mat> vertices;

		public Polygon(List<Mat> verts) {
			vertices = new ArrayList<Mat>(verts);
		}

		public static boolean pointInPolygon(Polygon poly, Mat point) {
			ArrayList<Point2D.Double> points = new ArrayList<Point2D.Double>();
			for (Mat v : poly.vertices) {
				points.add(new Point2D.Double(v.data[0][0], v.data[1][0]));
			}
			PolygonObstacle po = GeomUtils.convexHull(points);
			return po.contains(point.data[0][0], point.data[1][0]);
		}

		// Transform a polygon by a matrix
		public static Polygon mul(Mat matrix, Polygon poly) {
			ArrayList<Mat> newVertices = new ArrayList<Mat>();
			for (Mat vertex : poly.vertices) {
				newVertices.add(Mat.mul(matrix, vertex));
			}
			return new Polygon(newVertices);
		}

		// assumes that there will be no enclosed empty spaces in the result, and that the result will be contiguous, and that everything is convex
		public static Polygon minkowskiSumSimple(Polygon poly1, Polygon poly2) {
			ArrayList<Mat> verts = new ArrayList<Mat>();
			for (Mat v1 : poly1.vertices) {
				for (Mat v2 : poly2.vertices) {
					verts.add(Mat.add(v1, v2));
				}
			}
			ArrayList<Point2D.Double> points = new ArrayList<Point2D.Double>();
			for (Mat v : verts) {
				points.add(new Point2D.Double(v.data[0][0], v.data[1][0]));
			}
			PolygonObstacle po = GeomUtils.convexHull(points);
			verts = new ArrayList<Mat>();
			for (Point2D.Double vert : po.getVertices()) {
				verts.add(Mat.encodePoint(vert.x, vert.y));
			}
			return new Polygon(verts);
		}
	}
}
