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

	public ArrayList<Polygon> cSpaceObstacles = new ArrayList<Polygon>();

	double xMin, yMin, xMax, yMax;

	// robot reference point is the origin
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
		addObstacle(new Polygon(obstacle.getVertices()));
	}

	public void addObstacle(Polygon obstacle) {
		cSpaceObstacles.add(Polygon.minkowskiSum(obstacle, reflectedRobot));
	}

	public static class Polygon {
		private static final double DEFAULT_TOLERANCE = 0.0000001;
		public ArrayList<Mat> vertices;

		public Polygon(List<Point2D.Double> verts) {
			vertices = new ArrayList<Mat>();
			for (Point2D.Double vert : verts) {
				vertices.add(Mat.encodePoint(vert.x, vert.y));
			}
		}

		public Polygon(List<Mat> verts) {
			vertices = new ArrayList<Mat>(verts);
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

		// Generates a new polygon by dragging a polygon by a vector and taking all the points it touched.
		public static Polygon stroke(double x, double y, Polygon poly) {
			Polygon polyTrans = mul(Mat.translation(x, y), poly);
			ArrayList<Mat> vertices = new ArrayList<Mat>();
			ArrayList<ArrayList<Integer>> edgesTo = new ArrayList<ArrayList<Integer>>();
			int size = poly.vertices.size();

			// The list of possible vertices of the extrusion
			vertices.addAll(poly.vertices);
			vertices.addAll(polyTrans.vertices);

			// pre-compute some information that will make it easy to find the actual perimeter of the extrusion
			for (int i = 0; i < 2 * size; i++) {
				edgesTo.add(new ArrayList<Integer>());
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
			ArrayList<ArrayList<Integer>> edgesTo = new ArrayList<ArrayList<Integer>>();
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
								vertices.remove(p1);
								edgesTo.remove(p1);
								size--;

								for (int e0 = 0; e0 < size; e0++) {
									for (int i = 0; i < edgesTo.get(e0).size(); i++) {
										if (edgesTo.get(e0).get(i) == p1) {
											edgesTo.get(e0).set(i, p0);
										} else if (edgesTo.get(e0).get(i) > p1) {
											edgesTo.get(e0).set(i, edgesTo.get(e0).get(i) - 1);
										}
									}
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

												edgesTo.add(new ArrayList<Integer>());

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
		 * ArrayList<ArrayList<Integer>> edgesTo = To find the edges connecting to a point indexed i in vertices,
		 *                                     look in edgesTo.get(i). There you will find a list of indicies
		 *                                     into vertices, which are the endpoints of the edges. This data
		 *                                     structure is slow to write, but fast to read.
		 */
		public static Polygon perimeter(ArrayList<Mat> vertices, ArrayList<ArrayList<Integer>> edgesTo) {
			// Find a point guaranteed to be on the perimeter of the polygon, so we can start tracing it out
			double dist;
			double maxDist = -1;
			int farPoint;
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
			int nextVertex;
			double nextAngle;
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
			return tolerance > Line2D.ptSegDist(e0.data[0][0], e0.data[1][0], e1.data[0][0], e1.data[1][0], p.data[0][0], p.data[1][0]);
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
