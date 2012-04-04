package GlobalNavigation;

import java.awt.geom.*;
import java.util.*;

import LocalNavigation.Mat;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab5_msgs.ColorMsg;

/**
 * <p>Configuration space</p>
 *
 * @author nforrest
 *
 **/
public class CSpace {
	public Polygon reflectedRobot;

	public static Node node;

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
		Polygon.tests();
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

	public ArrayList<Polygon> getCSObstacles() {
		ArrayList<Polygon> csObstacles = new ArrayList<Polygon>();
		for (Polygon obstacle : obstacles) {
			csObstacles.add(Polygon.minkowskiSumSimple(obstacle, reflectedRobot));
		}
		return csObstacles;
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
		ArrayList<Polygon> csObstacles = getCSObstacles();
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
			}
		}

		System.err.println("GOT HERE TOO!");
		return occupancyGrid;
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
					occupancyGrid[i][j][k] = true;
					for (Polygon obstacle : obstacles) {
						if (Polygon.polygonsIntersect(resolutionCellSpace, Polygon.minkowskiSum(obstacle, strokedRobot))) {
							occupancyGrid[i][j][k] = false;
							break;
						}
					}

				}
			}
		}

		return occupancyGrid;
	}

	public static class Polygon {
		private static final double DEFAULT_TOLERANCE = 0.0001;
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
			ArrayList<Point2D.Double> points = new ArrayList<Point2D.Double>();
			for (Mat v : poly.vertices) {
				points.add(new Point2D.Double(v.data[0][0], v.data[1][0]));
			}
			PolygonObstacle po = GeomUtils.convexHull(points);
			return po.contains(point.data[0][0], point.data[1][0]);

			/*Mat farPoint = Mat.encodePoint(0, 0);
			double maxDist = -1;
			double dist;

			for (Mat vertex : poly.vertices) {
				dist = Mat.dist(point, vertex);
				if (maxDist < dist) {
					maxDist = dist;
				}
			}

			farPoint = Mat.add(point, Mat.mul(maxDist + 1, Mat.encodePoint(1, 0)));

			int size = poly.vertices.size();
			int intersections = 0;
			for (int i = 0; i < size; i++) {
				if (lineSegIntersect(point, farPoint, poly.vertices.get(i), poly.vertices.get((i + 1) % size))) {
					intersections++;
				}
			}

			return ((intersections % 2) == 1);*/
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

		// assumes that there will be no enclosed empty spaces in the result, and that the result will be contiguous.
		public static Polygon minkowskiSum(Polygon poly1, Polygon poly2) {
			Mat prevVertex = poly1.vertices.get(poly1.vertices.size() - 1);
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
				desiredDist = Mat.dist(cornerVert1a, Mat.encodePoint(0, 0));

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

			Publisher<GUISegmentMsg> segmentPub = node.newPublisher("/gui/Segment", "lab5_msgs/GUISegmentMsg");
			Publisher<GUIEraseMsg> erasePub = node.newPublisher("/gui/Erase", "lab5_msgs/GUIEraseMsg");
			//erasePub.publish(new GUIEraseMsg());
			GUISegmentMsg segmentPlot = new GUISegmentMsg();
			ColorMsg segmentPlotColor = new ColorMsg();

			// add all the vertices in both polygons
			vertices.addAll(poly1.vertices);
			vertices.addAll(poly2.vertices);

			for (int i = 0; i < size; i++) {
				edgesTo.add(new TreeSet<Integer>());
			}

			// add all the edges in both polygons
			for (int i = 0; i < sizePoly1; i++) {
				edgesTo.get(i).add(new Integer((i + 1) % sizePoly1));
				edgesTo.get((i + 1) % sizePoly1).add(new Integer(i));
			}
			for (int i = 0; i < sizePoly2; i++) {
				edgesTo.get(i + sizePoly1).add(new Integer(((i + 1) % sizePoly2) + sizePoly1));
				edgesTo.get(((i + 1) % sizePoly2) + sizePoly1).add(new Integer(i + sizePoly1));
			}

			System.err.println(vertices);
			System.err.println(edgesTo);

			segmentPlotColor.r = 255;
			segmentPlotColor.g = 0;
			segmentPlotColor.b = 0;
			segmentPlot.color = segmentPlotColor;
			for (int e0 = 0; e0 < size; e0++){
				for (int e1 : edgesTo.get(e0)) {
					double[] xyStart = Mat.decodePoint(vertices.get(e0));
					double[] xyEnd   = Mat.decodePoint(vertices.get(e1));
					segmentPlot.startX = xyStart[0];
					segmentPlot.startY = xyStart[1];
					segmentPlot.endX = xyEnd[0];
					segmentPlot.endY = xyEnd[1];
					segmentPub.publish(segmentPlot);
				}
			}

			// find and merge colocated points
			done = false;
			while (!done) {
				done = true;
				checkFMCP: {
					for (int p0 = 0; p0 < size; p0++){
						for (int p1 = 0; p1 < size; p1++){
							if (p0 != p1) {
								if (ptsEqual(vertices.get(p0), vertices.get(p1))) {
			//						System.err.println("found two colocated: " + p0 + " " + p1);
			//System.err.println(edgesTo);
									edgesTo.get(p0).addAll(edgesTo.get(p1));
									edgesTo.get(p0).remove(p0);
									vertices.remove(p1);
									edgesTo.remove(p1);
									size--;

									for (int e0 = 0; e0 < size; e0++) {
										if (edgesTo.get(e0).contains(new Integer(p1))) {
											edgesTo.get(e0).remove(new Integer(p1));
											edgesTo.get(e0).add(new Integer(p0));
										}
			//System.err.println("e0: " + e0);
			//System.err.println(edgesTo.get(e0));
										TreeSet<Integer> head = new TreeSet(edgesTo.get(e0).headSet(new Integer(p1)));
			//System.err.println(head);
										for (Integer e1 : edgesTo.get(e0).tailSet(new Integer(p1))) {
											head.add(e1 - 1);
										}
										head.remove(e0);
			//System.err.println(head);
										edgesTo.set(e0, head);
			//System.err.println(edgesTo.get(e0));
									}

									done = false;
			//System.err.println(edgesTo);
									break checkFMCP;
								}
							}
						}
					}
				}
			}
			System.err.println("merged points");
			System.err.println(edgesTo);

			/*segmentPlotColor.r = 0;
			segmentPlotColor.g = 0;
			segmentPlotColor.b = 255;
			segmentPlot.color = segmentPlotColor;
			for (int e0 = 0; e0 < size; e0++){
				for (int e1 : edgesTo.get(e0)) {
					double[] xyStart = Mat.decodePoint(vertices.get(e0));
					double[] xyEnd   = Mat.decodePoint(vertices.get(e1));
					segmentPlot.startX = xyStart[0];
					segmentPlot.startY = xyStart[1];
					segmentPlot.endX = xyEnd[0];
					segmentPlot.endY = xyEnd[1];
					segmentPub.publish(segmentPlot);
				}
			}*/

			// find and split edges bisected by points
			done = false;
			while (!done) {
				done = true;
				checkFSEBP: {
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
										break checkFSEBP;
									}
								}
							}
						}
					}
				}
			}
			System.err.println("split edges on points");
			System.err.println(edgesTo);

			System.err.println("GOT HERE!");

			int iters = 0;
			done = false;
			while (!done) {
				// find and split intersecting edges
				System.err.println("size: " + size);
				done = true;
				checkFSIE: {
					for (int e00 = 0; e00 < size; e00++){
						for (int e10 = 0; e10 < size; e10++){
							if (e00 != e10) {
								for (int e01 : new TreeSet<Integer>(edgesTo.get(e00))) {
									if (e01 != e10 && e01 != e00) {
										for (int e11 : new TreeSet<Integer>(edgesTo.get(e10))) {
											if (e11 != e00 && e11 != e01 && e11 != e10) {
												if (lineSegIntersect(vertices.get(e00), vertices.get(e01), vertices.get(e10), vertices.get(e11))) {
													//System.err.println("intersectors for iter " + iters);
													//System.err.println(e00);
													//System.err.println(edgesTo.get(e00));
													//Mat.print(System.err, vertices.get(e00));
													//System.err.println(e01);
													//System.err.println(edgesTo.get(e01));
													//Mat.print(System.err, vertices.get(e01));
													//System.err.println(e10);
													//System.err.println(edgesTo.get(e10));
													//Mat.print(System.err, vertices.get(e10));
													//System.err.println(e11);
													//System.err.println(edgesTo.get(e11));
													//Mat.print(System.err, vertices.get(e11));
						if (iters > 10000) {
							// FUCK!
							try{
								throw new Exception();
							}catch (Exception e){
								e.printStackTrace();
								System.err.println("hey there");
								System.exit(1);
							}
						}
						iters++;

													Mat newVertex = lineSegIntersection(vertices.get(e00), vertices.get(e01), vertices.get(e10), vertices.get(e11));

													if (ptsEqual(newVertex, vertices.get(e00))) {
															edgesTo.get(e10).remove(new Integer(e11));
															edgesTo.get(e10).add(new Integer(e00));
															edgesTo.get(e00).add(new Integer(e10));

															edgesTo.get(e11).remove(new Integer(e10));
															edgesTo.get(e11).add(new Integer(e00));
															edgesTo.get(e00).add(new Integer(e11));
													} else if (ptsEqual(newVertex, vertices.get(e01))) {
															edgesTo.get(e10).remove(new Integer(e11));
															edgesTo.get(e10).add(new Integer(e01));
															edgesTo.get(e01).add(new Integer(e10));

															edgesTo.get(e11).remove(new Integer(e10));
															edgesTo.get(e11).add(new Integer(e01));
															edgesTo.get(e01).add(new Integer(e11));
													} else if (ptsEqual(newVertex, vertices.get(e10))) {
															edgesTo.get(e00).remove(new Integer(e01));
															edgesTo.get(e00).add(new Integer(e10));
															edgesTo.get(e10).add(new Integer(e00));

															edgesTo.get(e01).remove(new Integer(e00));
															edgesTo.get(e01).add(new Integer(e10));
															edgesTo.get(e10).add(new Integer(e01));
													} else if (ptsEqual(newVertex, vertices.get(e11))) {
															edgesTo.get(e00).remove(new Integer(e01));
															edgesTo.get(e00).add(new Integer(e11));
															edgesTo.get(e11).add(new Integer(e00));

															edgesTo.get(e01).remove(new Integer(e00));
															edgesTo.get(e01).add(new Integer(e11));
															edgesTo.get(e11).add(new Integer(e01));
													} else {
															vertices.add(newVertex);

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
													}
													done = false;
													break checkFSIE;
												}
											}
										}
									}
								}
							}
						}
					}
				}
				System.err.println("split edges on edges");
				System.err.println(edgesTo);
			}

			System.err.println("GOT HERE TOO!");

			System.err.println("begin vertices");
			for (Mat vertex : vertices) {
				Mat.print(System.err, vertex);
			}
			System.err.println("end vertices");
			System.err.println(edgesTo);

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
				System.err.println("v:"+vertex);
				e0 = vertices.get(vertex);
			//	System.err.println(edgesTo.get(vertex));
			//	Mat.print(System.err, e0);
				perimeter.add(e0);

				if (perimeter.size() > 100) {
					// FUCK!
					try{
						throw new Exception();
					}catch (Exception e){
						e.printStackTrace();
						System.err.println("hey there");
						System.exit(1);
					}
				}

				for (int edgeVertex : edgesTo.get(vertex)) {
					e1 = vertices.get(edgeVertex);
					edgeAngle = Math.atan2(e1.data[0][0] - e0.data[0][0], e1.data[1][0] - e0.data[1][0]);
					angleDiff = edgeAngle - prevAngle;
					if (angleDiff < 0) {
						angleDiff += 2 * Math.PI;
					}
					if (angleDiff > 2 * Math.PI) {
						angleDiff -= 2 * Math.PI;
					}
					if (angleDiff < minAngleDiff && edgeVertex != prevVertex) {
						minAngleDiff = angleDiff;
						nextVertex = edgeVertex;
						nextAngle = edgeAngle;
					}
				}

				System.err.println("PERIMETER ITERATION:");
				System.err.println("vertexStart = " + vertexStart);
				System.err.println("prevVertex = " + prevVertex);
				System.err.println("vertex = " + vertex);
				System.err.println("nextVertex = " + nextVertex);
				System.err.println("");

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
			return tolerance > Mat.dist(p0, p1);
		}

		public static boolean ptSegIntersect(Mat e0, Mat e1, Mat p) {
			return ptSegIntersect(e0, e1, p, DEFAULT_TOLERANCE);
		}

		public static boolean ptSegIntersect(Mat e0, Mat e1, Mat p, double tolerance) {
			return tolerance > ptSegDistance(e0, e1, p);
		}

		public static double ptSegDistance(Mat e0, Mat e1, Mat p) {
			double x0 = e0.data[0][0];
			double y0 = e0.data[1][0];
			double x1 = e1.data[0][0];
			double y1 = e1.data[1][0];
			double xp = p.data[0][0];
			double yp = p.data[1][0];

			e0 = new Mat(2, 1);
			e0.data[0][0] = x0;
			e0.data[1][0] = y0;
			e1 = new Mat(2, 1);
			e1.data[0][0] = x1;
			e1.data[1][0] = y1;
			p = new Mat(2, 1);
			p.data[0][0] = xp;
			p.data[1][0] = yp;

			e1 = Mat.sub(e1, e0);
			p  = Mat.sub(p,  e0);
			e0.data[0][0] = 0;
			e0.data[1][0] = 0;

			Mat e0b = Mat.sub(e0, e1);
			Mat pb  = Mat.sub(p,  e1);
			Mat e1b = new Mat(2, 1);
			e1b.data[0][0] = 0;
			e1b.data[1][0] = 0;

			if (Mat.dot(e1, p) > 0) {
				if (Mat.dot(e0b, pb) > 0) {
					Mat e1u = Mat.mul(1 / Mat.l2(e1), e1);
					return Mat.dist(p, Mat.mul(Mat.dot(e1u, p), e1u));
				} else {
					return Mat.dist(p, e1);
				}
			} else {
				return Mat.dist(p, e0);
			}
		}

		public static boolean lineSegIntersect(Mat e00, Mat e01, Mat e10, Mat e11) {
			double x00 = e00.data[0][0];
			double y00 = e00.data[1][0];
			double x01 = e01.data[0][0];
			double y01 = e01.data[1][0];
			double x10 = e10.data[0][0];
			double y10 = e10.data[1][0];
			double x11 = e11.data[0][0];
			double y11 = e11.data[1][0];

			e00 = new Mat(2, 1);
			e00.data[0][0] = x00;
			e00.data[1][0] = y00;
			e01 = new Mat(2, 1);
			e01.data[0][0] = x01;
			e01.data[1][0] = y01;
			e10 = new Mat(2, 1);
			e10.data[0][0] = x10;
			e10.data[1][0] = y10;
			e11 = new Mat(2, 1);
			e11.data[0][0] = x11;
			e11.data[1][0] = y11;

			double x, y;
			double m0, m1;

			if (x00 == x01) {
				if (x10 == x11) {
					return false;
				}
				m1 = (y10 - e11.data[1][0]) / (x10 - e11.data[0][0]);

				x = x00;
				y = m1 * (x - x10) + y10;

				if (((y > y00 && y < y01) || (y < y00 && y > y01)) &&
				    ((y > y10 && y < y11) || (y < y10 && y > y11) || (y10 == y11))) {
					return true;
				} else {
					return false;
				}
			} else if (x10 == x11) {
				m0 = (y00 - e01.data[1][0]) / (x00 - e01.data[0][0]);

				x = x10;
				y = m0 * (x - x00) + y00;

				if (((y > y00 && y < y01) || (y < y00 && y > y01) || (y00 == y01)) &&
				    ((y > y10 && y < y11) || (y < y10 && y > y11))) {
					return true;
				} else {
					return false;
				}
			} else {
				m0 = (y00 - e01.data[1][0]) / (x00 - e01.data[0][0]);
				m1 = (y10 - e11.data[1][0]) / (x10 - e11.data[0][0]);

				x = (m0 * x00 - m1 * x10 - y00 + y10) / (m0 - m1);
				y = m1 * (x - x10) + y10;

				if (((x > x00 && x < x01) || (x < x00 && x > x01)) &&
				    ((x > x10 && x < x11) || (x < x10 && x > x11))) {
					return true;
				} else {
					return false;
				}
			}
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

				//System.err.println("m0 = " + m0);
				//System.err.println("m1 = " + m1);

				x = (m0 * x0 - m1 * x1 - y0 + y1) / (m0 - m1);
				y = m1 * (x - x1) + y1;

				//System.err.println("x = " + x);
				//System.err.println("y = " + y);
			}
			return Mat.encodePoint(x, y);
		}

		public static void tests() {
			System.err.println("HEY!");
			Mat p0 = Mat.encodePoint(0, 0);
			Mat p1 = Mat.encodePoint(1, .1);
			Mat p2 = Mat.encodePoint(.1, 1);
			Mat p3 = Mat.encodePoint(-1, -.1);
			Mat p4 = Mat.encodePoint(-.1, -1);
			Mat p5 = Mat.encodePoint(0.0001, 0);
			Mat p6 = Mat.encodePoint(-0.0001, 0);
			if (!lineSegIntersect(p1, p3, p2, p4)) {
					try{
						throw new Exception();
					}catch (Exception e){
						e.printStackTrace();
						System.err.println("hey there");
						System.exit(1);
					}
			}
			if (lineSegIntersect(p1, p4, p2, p3)) {
					try{
						throw new Exception();
					}catch (Exception e){
						e.printStackTrace();
						System.err.println("hey there");
						System.exit(1);
					}
			}
			if (lineSegIntersect(p1, p5, p2, p4)) {
					try{
						throw new Exception();
					}catch (Exception e){
						e.printStackTrace();
						System.err.println("hey there");
						System.exit(1);
					}
			}
			if (!lineSegIntersect(p1, p6, p2, p4)) {
					try{
						throw new Exception();
					}catch (Exception e){
						e.printStackTrace();
						System.err.println("hey there");
						System.exit(1);
					}
			}
			Mat inter = lineSegIntersection(p1, p3, p2, p4);
			if (inter.data[0][0] != 0 || inter.data[1][0] != 0) {
					try{
						throw new Exception();
					}catch (Exception e){
						e.printStackTrace();
						System.err.println("hey there");
						System.exit(1);
					}
			}
		}
	}
}
