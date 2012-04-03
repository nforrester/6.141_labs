package GlobalNavigation;

import java.awt.geom.*;
import java.util.*;

import LocalNavigation.Mat;

/**
 * <p>Configuration space</p>
 *
 * @author nforrest
 *
 **/
public class CSpace {
	public Polygon reflectedRobot;

	public ArrayList<Polygon> obstacles = new ArrayList<Polygon>();
	public DoubleMap<ArrayList<Polygon>> cSpaceObstacles = new DoubleMap<ArrayList<Polygon>>();

	public double xMin, yMin, xMax, yMax;

	// robot reference point is the origin
	public CSpace(Polygon robot, PolygonMap map) {
		Rectangle2D.Double worldRect = map.getWorldRect();
		constructor(robot, worldRect.getMinX(), worldRect.getMinY(), worldRect.getMaxX(), worldRect.getMaxY());
		for (PolygonObstacle obstacle : map.getObstacles()) {
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
		cSpaceObstacles.setWrap(-1 * Math.PI, Math.PI);

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
	}

	public ArrayList<Polygon> getThetaObstacles(double theta, double thetaTolerance) {
		Maybe<DoubleMap<ArrayList<Polygon>>.Pair> mp = cSpaceObstacles.get(theta, thetaTolerance);
		ArrayList<Polygon> thetaObstacles;
		Polygon strokedRobot;
		if (mp.just) {
			thetaObstacles = mp.value.v;
		} else {
			strokedRobot = Polygon.strokeRot(theta - thetaTolerance, theta + thetaTolerance, reflectedRobot);
			thetaObstacles = new ArrayList<Polygon>();
			for (Polygon obstacle : obstacles) {
				thetaObstacles.add(Polygon.minkowskiSum(obstacle, strokedRobot));
			}
			cSpaceObstacles.put(theta, thetaObstacles, thetaTolerance);
		}
		return thetaObstacles;
	}

	// occupancy grid is indexed from (xMin, yMin, 0)
	public boolean[][][] getOccupancyGrid(int nCellsLinear, int nCellsAngular) {
		int i, j, k;
		double xLow, xHigh, yLow, yHigh, thetaLow, thetaHigh;
		double maxDimension = Math.max(xMax - xMin, yMax - yMin);
		double resolutionLinear = maxDimension / nCellsLinear;
		double resolutionAngular = 2 * Math.PI / nCellsAngular;
		Polygon strokedRobot;
		ArrayList<Polygon> thetaObstacles;
		Polygon resolutionCellSpace;
		boolean [][][] occupancyGrid = new boolean[nCellsLinear][nCellsLinear][nCellsAngular];
		for (i = 0; i < nCellsLinear; i++) {
			xLow = xMin + resolutionLinear * i;
			xHigh = xLow + resolutionLinear;
			for (j = 0; j < nCellsLinear; j++) {
				yLow = yMin + resolutionLinear * j;
				yHigh = yLow + resolutionLinear;
				for (k = 0; k < nCellsAngular; k++) {
					thetaLow = resolutionAngular * k;
					thetaHigh = thetaLow + resolutionAngular;

					resolutionCellSpace = new Polygon(Arrays.asList(Mat.encodePoint(xLow, yLow),
					                                                Mat.encodePoint(xLow, yHigh),
					                                                Mat.encodePoint(xHigh, yHigh),
					                                                Mat.encodePoint(xHigh, yLow)));

					strokedRobot = Polygon.strokeRot(thetaLow, thetaHigh, reflectedRobot);
					occupancyGrid[i][j][k] = false;
					for (Polygon obstacle : obstacles) {
						if (Polygon.polygonsIntersect(resolutionCellSpace, Polygon.minkowskiSum(obstacle, strokedRobot))) {
							occupancyGrid[i][j][k] = true;
							break;
						}
					}

				}
			}
		}

		return occupancyGrid;
	}

	public static class Polygon {
		private static final double DEFAULT_TOLERANCE = 0.0000001;
		public ArrayList<Mat> vertices;

		public Polygon(List<Mat> verts) {
			vertices = new ArrayList<Mat>(verts);
		}

		public static boolean polygonsIntersect(Polygon poly1, Polygon poly2) {
			// if any vertices from one polygon are inside the other polygon
			for (Mat vertex : poly1.vertices) {
				if (pointInPolygon(poly2, vertex)) {
					return true;
				}
			}
			for (Mat vertex : poly2.vertices) {
				if (pointInPolygon(poly1, vertex)) {
					return true;
				}
			}

			// if any edges hit eachother
			ArrayList<Mat> poly1EdgesA = poly1.vertices;
			ArrayList<Mat> poly1EdgesB = new ArrayList<Mat>(poly1EdgesA);
			poly1EdgesB.add(poly1EdgesB.get(0));
			poly1EdgesB.remove(0);

			ArrayList<Mat> poly2EdgesA = poly2.vertices;
			ArrayList<Mat> poly2EdgesB = new ArrayList<Mat>(poly2EdgesA);
			poly2EdgesB.add(poly2EdgesB.get(0));
			poly2EdgesB.remove(0);

			for (int e1 = 0; e1 < poly1EdgesA.size(); e1++) {
				for (int e2 = 0; e2 < poly2EdgesA.size(); e2++) {
					if (lineSegIntersect(poly1EdgesA.get(e1), poly1EdgesB.get(e1), poly2EdgesA.get(e2), poly2EdgesB.get(e2))) {
						return true;
					}
				}
			}

			// No? ok, cool.
			return false;
		}

		public static boolean pointInPolygon(Polygon poly, Mat point) {
			Mat farPoint = Mat.encodePoint(0, 0);
			double maxDist = -1;
			double dist;

			for (Mat vertex : poly.vertices) {
				dist = Mat.l2(Mat.add(point, Mat.mul(-1, vertex)));
				if (maxDist < dist) {
					maxDist = dist;
					farPoint = vertex;
				}
			}

			farPoint = Mat.add(point, Mat.mul(2, Mat.add(farPoint, Mat.mul(-1, point))));

			int size = poly.vertices.size();
			int intersections = 0;
			for (int i = 0; i < size; i++) {
				if (lineSegIntersect(point, farPoint, poly.vertices.get(i), poly.vertices.get((i + 1) % size))) {
					intersections++;
				}
			}

			return ((intersections % 2) == 1);
		}

		// Transform a polygon by a matrix
		public static Polygon mul(Mat matrix, Polygon poly) {
			ArrayList<Mat> newVertices = new ArrayList<Mat>();
			for (Mat vertex : poly.vertices) {
				newVertices.add(Mat.mul(matrix, vertex));
			}
			return new Polygon(newVertices);
		}

		// assumes that there will be no enclosed empty spaces in the result, and that the result will be contiguous.
		public static Polygon minkowskiSum(Polygon poly1, Polygon poly2) {
			Mat prevVertex = poly1.vertices.get(poly1.vertices.size());
			Polygon sum = poly1;
			for (Mat vertex : poly1.vertices) {
				sum = combine(sum, mul(Mat.translation(vertex.data[0][0], vertex.data[1][0]), stroke(prevVertex.data[0][0] - vertex.data[0][0], prevVertex.data[1][0] - vertex.data[1][0], poly2)));
				prevVertex = vertex;
			}
			return sum;
		}

		// Generates a new polygon by dragging a polygon through an angle, and taking all the points it touched (and a few extras for overestimation)
		// Only valid for convex polygons, and small rotations (20 degrees or less)
		// stroke from theta1 to theta2
		public static Polygon strokeRot(double theta1, double theta2, Polygon poly) {
			Polygon poly1 = mul(Mat.rotation(theta1), poly);
			Polygon poly2 = mul(Mat.rotation(theta2), poly);
			Polygon stroked = combine(poly1, poly2);
			ArrayList<Mat> cornerVerts;
			Polygon corner;
			int thisVertex, prevVertex, nextVertex;
			Mat origin = Mat.encodePoint(0, 0);
			Mat cornerVert1a, cornerVert1b;
			Mat cornerVert2a, cornerVert2b;
			double actualDist, desiredDist;
			Mat cornerTransform;
			for (thisVertex = 0; thisVertex < poly.vertices.size(); thisVertex++) {
				if (thisVertex == 0) {
					prevVertex = poly.vertices.size() - 1;
				} else {
					prevVertex = thisVertex - 1;
				}

				if (thisVertex == poly.vertices.size() - 1) {
					nextVertex = 0;
				} else {
					nextVertex = thisVertex + 1;
				}

				cornerVerts = new ArrayList<Mat>();
				cornerVerts.add(poly2.vertices.get(nextVertex));
				cornerVerts.add(origin);
				cornerVerts.add(poly1.vertices.get(prevVertex));

				cornerVert1a = poly1.vertices.get(thisVertex);
				cornerVert2a = poly2.vertices.get(thisVertex);

				actualDist = ptSegDistance(cornerVert1a, cornerVert2a, origin);
				desiredDist = Mat.l2(cornerVert1a);

				cornerTransform = Mat.mul(desiredDist / actualDist, Mat.eye(4));
				cornerVert1b = Mat.mul(cornerTransform, cornerVert1a);
				cornerVert2b = Mat.mul(cornerTransform, cornerVert2a);

				cornerVert1b = lineSegIntersection(cornerVert1b, cornerVert2b, poly1.vertices.get(prevVertex), cornerVert1a);
				cornerVert2b = lineSegIntersection(cornerVert1b, cornerVert2b, poly2.vertices.get(nextVertex), cornerVert2a);

				cornerVerts.add(cornerVert1b);
				cornerVerts.add(cornerVert2b);

				corner = new Polygon(cornerVerts);
				stroked = combine(stroked, corner);
			}

			return stroked;
		}

		// Generates a new polygon by dragging a polygon by a vector and taking all the points it touched.
		public static Polygon stroke(double x, double y, Polygon poly) {
			Polygon polyTrans = mul(Mat.translation(x, y), poly);
			ArrayList<Mat> vertices = new ArrayList<Mat>();
			ArrayList<TreeSet<Integer>> edgesTo = new ArrayList<TreeSet<Integer>>();
			int size = poly.vertices.size();

			// The list of possible vertices of the extrusion
			vertices.addAll(poly.vertices);
			vertices.addAll(polyTrans.vertices);

			// pre-compute some information that will make it easy to find the actual perimeter of the extrusion
			for (int i = 0; i < 2 * size; i++) {
				edgesTo.add(new TreeSet<Integer>());
			}

			for (int i = 0; i < size; i++) {
				edgesTo.get(i).add(new Integer((i + 1) % size));
				edgesTo.get((i + 1) % size).add(new Integer(i));

				edgesTo.get(i + size).add(new Integer(((i + 1) % size) + size));
				edgesTo.get(((i + 1) % size) + size).add(new Integer(i + size));

				edgesTo.get(i).add(new Integer(i + size));
				edgesTo.get(i + size).add(new Integer(i));
			}

			return perimeter(vertices, edgesTo);
		}

		// take the union of two polygons, assuming no enclosed empty spaces, and that the result will be contiguous.
		public static Polygon combine(Polygon poly1, Polygon poly2) {
			ArrayList<Mat> vertices = new ArrayList<Mat>();
			ArrayList<TreeSet<Integer>> edgesTo = new ArrayList<TreeSet<Integer>>();
			int sizePoly1 = poly1.vertices.size();
			int sizePoly2 = poly2.vertices.size();
			int size = sizePoly1 + sizePoly2;
			boolean done;

			// add all the vertices in both polygons
			vertices.addAll(poly1.vertices);
			vertices.addAll(poly2.vertices);

			// add all the edges in both polygons
			for (int i = 0; i < size; i++) {
				edgesTo.get(i).add(new Integer((i + 1) % sizePoly1));
				edgesTo.get((i + 1) % sizePoly1).add(new Integer(i));

				edgesTo.get(i + sizePoly1).add(new Integer(((i + 1) % sizePoly2) + sizePoly1));
				edgesTo.get(((i + 1) % sizePoly2) + sizePoly1).add(new Integer(i + sizePoly1));
			}

			// find and merge colocated points
			done = false;
			while (!done) {
				done = true;
				for (int p0 = 0; p0 < size; p0++){
					for (int p1 = 0; p1 < size; p1++){
						if (p0 != p1) {
							if (ptsEqual(vertices.get(p0), vertices.get(p1))) {
								edgesTo.get(p0).addAll(edgesTo.get(p1));
								vertices.remove(p1);
								edgesTo.remove(p1);
								size--;

								for (int e0 = 0; e0 < size; e0++) {
									if (edgesTo.get(e0).contains(new Integer(p1))) {
										edgesTo.get(e0).remove(new Integer(p1));
										edgesTo.get(e0).add(new Integer(p0));
									}
									SortedSet<Integer> head = edgesTo.get(e0).headSet(new Integer(p1));
									SortedSet<Integer> tail = edgesTo.get(e0).tailSet(new Integer(p1));
									SortedSet<Integer> newTail = new TreeSet<Integer>();
									for (Integer e1 : tail) {
										newTail.add(e1 - 1);
									}
									edgesTo.get(e0).clear();
									edgesTo.get(e0).addAll(head);
									edgesTo.get(e0).addAll(newTail);
								}

								done = false;
							}
						}
					}
				}
			}

			// find and split edges bisected by points
			done = false;
			while (!done) {
				done = true;
				for (int e0 = 0; e0 < size; e0++){
					for (int e1 : edgesTo.get(e0)) {
						for (int p = 0; p < size; p++){
							if (e0 != p && e1 != p) {
								if (ptSegIntersect(vertices.get(e0), vertices.get(e1), vertices.get(p))) {
									edgesTo.get(p).add(new Integer(e0));
									edgesTo.get(p).add(new Integer(e1));

									edgesTo.get(e0).remove(new Integer(e1));
									edgesTo.get(e0).add(new Integer(p));

									edgesTo.get(e1).remove(new Integer(e0));
									edgesTo.get(e1).add(new Integer(p));

									size++;
									done = false;
								}
							}
						}
					}
				}
			}

			// find and split intersecting edges
			done = false;
			while (!done) {
				done = true;
				for (int e00 = 0; e00 < size; e00++){
					for (int e10 = 0; e10 < size; e10++){
						if (e00 != e10) {
							for (int e01 : edgesTo.get(e00)) {
								if (e01 != e10) {
									for (int e11 : edgesTo.get(e10)) {
										if (e11 != e00 && e11 != e01) {
											if (lineSegIntersect(vertices.get(e00), vertices.get(e01), vertices.get(e10), vertices.get(e11))) {
												vertices.add(lineSegIntersection(vertices.get(e00), vertices.get(e01), vertices.get(e10), vertices.get(e11)));

												edgesTo.add(new TreeSet<Integer>());

												edgesTo.get(size).add(new Integer(e00));
												edgesTo.get(size).add(new Integer(e01));
												edgesTo.get(size).add(new Integer(e10));
												edgesTo.get(size).add(new Integer(e11));

												edgesTo.get(e00).remove(new Integer(e01));
												edgesTo.get(e00).add(new Integer(size));

												edgesTo.get(e01).remove(new Integer(e00));
												edgesTo.get(e01).add(new Integer(size));

												edgesTo.get(e10).remove(new Integer(e11));
												edgesTo.get(e10).add(new Integer(size));

												edgesTo.get(e11).remove(new Integer(e10));
												edgesTo.get(e11).add(new Integer(size));

												size++;
												done = false;
											}
										}
									}
								}
							}
						}
					}
				}
			}

			return perimeter(vertices, edgesTo);
		}

		/*
		 * Take a collection of vertices and edges between them,
		 * and return the minimum area polygon containing all of them.
		 *
		 * ArrayList<Mat> vertices = the list of vertices
		 *
		 * ArrayList<TreeSet<Integer>> edgesTo = To find the edges connecting to a point indexed i in vertices,
		 *                                       look in edgesTo.get(i). There you will find a list of indicies
		 *                                       into vertices, which are the endpoints of the edges. This data
		 *                                       structure is slow to write, but fast to read.
		 */
		public static Polygon perimeter(ArrayList<Mat> vertices, ArrayList<TreeSet<Integer>> edgesTo) {
			// Find a point guaranteed to be on the perimeter of the polygon, so we can start tracing it out
			double dist;
			double maxDist = -1;
			int farPoint = -1;
			for (int i = 0; i < vertices.size(); i++) {
				dist = Math.pow(Math.pow(vertices.get(i).data[0][0], 2) + Math.pow(vertices.get(i).data[1][0], 2), 0.5);
				if (dist > maxDist) {
					maxDist = dist;
					farPoint = i;
				}
			}

			// Trace out the perimeter of the polygon
			int vertexStart = farPoint;
			int vertex = vertexStart;
			int prevVertex = -1;
			double prevAngle = Math.atan2(vertices.get(vertex).data[0][0], vertices.get(vertex).data[1][0]);
			double edgeAngle;
			double angleDiff;
			double minAngleDiff;
			int nextVertex = -1;
			double nextAngle = -1;
			Mat e0, e1;
			ArrayList<Mat> perimeter = new ArrayList<Mat>();
			while (vertex != vertexStart || prevVertex == -1) {
				minAngleDiff = 3 * Math.PI;
				e0 = vertices.get(vertex);
				perimeter.add(e0);
				for (int edgeVertex : edgesTo.get(vertex)) {
					e1 = vertices.get(edgeVertex);
					edgeAngle = Math.atan2(e1.data[0][0] - e0.data[0][0], e1.data[1][0] - e0.data[1][0]);
					angleDiff = edgeAngle - prevAngle;
					if (angleDiff < 0) {
						angleDiff += 2 * Math.PI;
					}
					if (angleDiff < minAngleDiff) {
						minAngleDiff = angleDiff;
						nextVertex = edgeVertex;
						nextAngle = edgeAngle;
					}
				}

				if (nextAngle > 0) {
					prevAngle = nextAngle - Math.PI;
				} else {
					prevAngle = nextAngle + Math.PI;
				}
				prevVertex = vertex;
				vertex = nextVertex;
			}

			return new Polygon(perimeter);
		}

		public static boolean ptsEqual(Mat p0, Mat p1) {
			return ptsEqual(p0, p1, DEFAULT_TOLERANCE);
		}

		public static boolean ptsEqual(Mat p0, Mat p1, double tolerance) {
			return tolerance > Math.pow(Math.pow(p0.data[0][0] - p1.data[0][0], 2) + Math.pow(p0.data[1][0] - p1.data[1][0], 2), 0.5);
		}

		public static boolean ptSegIntersect(Mat e0, Mat e1, Mat p) {
			return ptSegIntersect(e0, e1, p, DEFAULT_TOLERANCE);
		}

		public static boolean ptSegIntersect(Mat e0, Mat e1, Mat p, double tolerance) {
			return tolerance > ptSegDistance(e0, e1, p);
		}

		public static double ptSegDistance(Mat e0, Mat e1, Mat p) {
			return Line2D.ptSegDist(e0.data[0][0], e0.data[1][0], e1.data[0][0], e1.data[1][0], p.data[0][0], p.data[1][0]);
		}

		public static boolean lineSegIntersect(Mat e00, Mat e01, Mat e10, Mat e11) {
			return Line2D.linesIntersect(e00.data[0][0], e00.data[1][0], e01.data[0][0], e01.data[1][0], e10.data[0][0], e10.data[1][0], e11.data[0][0], e11.data[1][0]);
		}

		public static Mat lineSegIntersection(Mat e00, Mat e01, Mat e10, Mat e11) {
			double x0 = e00.data[0][0];
			double y0 = e00.data[1][0];
			double x1 = e10.data[0][0];
			double y1 = e10.data[1][0];

			double x, y;
			double m0, m1;
			if (x0 == e01.data[0][0]) {
				m1 = (y1 - e11.data[1][0]) / (x1 - e11.data[0][0]);

				x = x0;
				y = m1 * (x - x1) + y1;
			} else if (x1 == e11.data[0][0]) {
				m0 = (y0 - e01.data[1][0]) / (x0 - e01.data[0][0]);

				x = x1;
				y = m0 * (x - x0) + y0;
			} else {
				m0 = (y0 - e01.data[1][0]) / (x0 - e01.data[0][0]);
				m1 = (y1 - e11.data[1][0]) / (x1 - e11.data[0][0]);

				x = (m0 * x0 - m1 * x1 - y0 + y1) / (m0 - m1);
				y = m1 * (x - x1) + y1;
			}
			return Mat.encodePoint(x, y);
		}
	}
}
