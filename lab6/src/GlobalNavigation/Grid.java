package GlobalNavigation;

import java.awt.geom.*;
import java.util.*;

/**
 * <p>Grid for path planning.</p>
 *
 * @author vona
 **/
public class Grid implements Iterable<Grid.Cell> {

  public class Cell {

    /**
     * <p>This cell's row.</p>
     **/ 
    int r;

    /**
     * <p>This cell's column.</p>
     **/
    int c;

    /**
     * <p>The bounds of this cell in world coords.</p>
     **/
    Rectangle2D.Double rect = new Rectangle2D.Double();

    /**
     * <p>Distance to goal along the shortest path (m), iff {@link
     * Grid#computeShortestPaths} has been run.</p>.
     **/
    double minDistanceToGoal = Double.POSITIVE_INFINITY;

    /**
     * <p>Whether this cell is in free space.</p>
     **/
    boolean free = true;

    /**
     * <p>The next cell along the shortest path to the goal, if any, iff {@link
     * Grid#computeShortestPaths} has been run.</p>
     *
     * <p>Null if this cell contains the goal.</p>
     **/
    Cell toGoalNext = null;

    /**
     * <p>Create a new cell.</p>
     **/
    Cell(int r, int c, double x, double y, double width, double height) {
      this.r = r;
      this.c = c;
      rect.x = x;
      rect.y = y;
      rect.width = width;
      rect.height = height;
    }

    /**
     * <p>Get the x coordinate of the center of this cell.</p>
     *
     * @return the x coordinate of the center of this cell
     **/
    public double getCenterX() {
      return rect.x + rect.width/2.0;
    }

    /**
     * <p>Get the y coordinate of the center of this cell.</p>
     *
     * @return the y coordinate of the center of this cell
     **/
    public double getCenterY() {
      return rect.y + rect.height/2.0;
    }

    /**
     * <p>Return the center point of this cell.</p>
     *
     * @return the center point of this cell as a new point
     **/
    public Point2D.Double makeCenterPoint() {
      return new Point2D.Double(getCenterX(), getCenterY());
    }

    /**
     * <p>Check whether this cell has a path to the goal.</p>
     *
     * <p>Returns true even if this cell contains the goal (i.e. {@link
     * #toGoalNext} is null).</p>
     *
     * @return true iff this cell has a path to the goal
     **/
    public boolean hasPathToGoal() {
      return free && !Double.isInfinite(minDistanceToGoal);
    }

    /**
     * <p>Return a human-readable string representation of this cell.</p>
     **/
    public String toString() {
      return "(" + r + ", " + c + ") x=" +
        rect.x + ", y=" + rect.y +
        ", width=" + rect.width + ", height=" + rect.height +
        ", dist=" + minDistanceToGoal;
    }
  }

  /**
   * <p>World boundary.</p>
   **/
  protected Rectangle2D.Double worldBounds;

  /**
   * <p>Grid resolution.</p>
   **/
  protected double resolution;

  /**
   * <p>Number of rows in the grid.</p>
   **/
  protected int rows;

  /**
   * <p>Number of cols in the grid.</p>
   **/
  protected int cols;

  /**
   * <p>The grid cells.</p>
   **/
  protected Cell[][] cell;

  /**
   * <p>Create a new grid.</p>
   *
   * @param worldBounds the world boundary
   * @param resolution the grid resolution
   **/
  public Grid(Rectangle2D.Double worldBounds, double resolution) {

    this.worldBounds = worldBounds;
    this.resolution = resolution;
    
    rows = (int) Math.ceil(worldBounds.height/resolution);
    cols = (int) Math.ceil(worldBounds.width/resolution);

    cell = new Cell[rows][cols];

    for (int r = 0; r < rows; r++)
      for (int c = 0; c < cols; c++)
        cell[r][c] =
          new Cell(r, c,
                   worldBounds.x + c*resolution, worldBounds.y + r*resolution,
                   resolution, resolution);
  }

  /**
   * <p>Mark all the {@link Grid.Cell} that {@link PolygonObstacle#intersects}
   * <code>obstacle</code> not {@link Grid.Cell#free}.</p>
   *
   * @param obstacle the obstacle
   *
   * @return the number of cells marked
   **/
  public int markObstacle(PolygonObstacle obstacle) {

    int n = 0;

    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {
        if (obstacle.intersects(cell[r][c].rect)) {
          cell[r][c].free = false;
          n++;
        }
      }
    }

    return n;
  }

  /**
   * <p>Compute and cache the shortest (Manhattan) paths (BFS) from each cell
   * in the grid to the cell containing <code>goalPoint</code>.</p>
   *
   * <p>Only {@link Cell#free} cells are traversed.</p>
   *
   * @param goalPoint the goal point
   *
   * @return the maximum path distance from any cell which can reach the goal
   * to the goal, or Double.POSITIVE_INFINITY if <code>goalPoint</code> is out
   * of bounds
   **/
  public double computeShortestPaths(Point2D.Double goalPoint) {

    Cell goalCell = getCell(goalPoint);

    if (goalCell == null)
      return Double.POSITIVE_INFINITY;
   
    goalCell.minDistanceToGoal = 0.0;

    double maxDist = 0.0;

    LinkedList<Cell> queue = new LinkedList<Cell>();
    LinkedList<Cell> neighbors = new LinkedList<Cell>();

    queue.add(goalCell);

    while (!queue.isEmpty()) {
      Cell cell = queue.removeFirst();
      neighbors.clear();
      for (Cell neighbor : collectNeighbors(cell, neighbors)) {
        if (neighbor.free && Double.isInfinite(neighbor.minDistanceToGoal)) {
          neighbor.minDistanceToGoal = cell.minDistanceToGoal + resolution;
          neighbor.toGoalNext = cell;
          queue.add(neighbor);
          if (neighbor.minDistanceToGoal > maxDist)
            maxDist = neighbor.minDistanceToGoal;
        }
      }
    }

    return maxDist;
  }

  /**
   * <p>Collect all the (Manhattan) neighbors of <code>cell</code>.</p>
   *
   * @param c the cell
   * @param neighbors the neighbors are collected here
   * @return a ref to <code>neighbors</code>
   **/ 
  protected List<Cell> collectNeighbors(Cell c, List<Cell> neighbors) {

    //below
    if (c.r > 0)
      neighbors.add(cell[c.r-1][c.c]);

    //above
    if (c.r < rows-1)
      neighbors.add(cell[c.r+1][c.c]);

    //left
    if (c.c > 0)
      neighbors.add(cell[c.r][c.c-1]);

    //right

    if (c.c < cols-1)
      neighbors.add(cell[c.r][c.c+1]);

    return neighbors;
  }

  /**
   * <p>Get the cell containing <code>point</code>, or null if out of
   * bounds.</p>
   *
   * @param point the query point
   *
   * @return the cell containing <code>point</code>, or null if out of bounds
   **/
  public Cell getCell(Point2D.Double point) {

    if (!worldBounds.contains(point.x, point.y))
      return null;

    int r = (int) Math.floor((point.y-worldBounds.y)/resolution);
    int c = (int) Math.floor((point.x-worldBounds.x)/resolution);

    return cell[r][c];
  }

  /**
   * <p>Get a {@link Grid.GridIterator} over {@link #cell}.</p>
   **/
  public Iterator<Cell> iterator() {
    return new GridIterator();
  }

  /**
   * <p>Get the number of cells in this Grid.</p>
   *
   * @return the number of cells in this Grid
   **/
  public int numCells() {
    return rows*cols;
  }

  /**
   * <p>An iterator over {@link Grid#cell}.</p>
   *
   * <p>Iterates in a row raster from left to right starting at the lower
   * left.</p>
   **/
  protected class GridIterator implements Iterator<Cell> {

    int r = 0;
    int c = 0;

    public Cell next() {

      Cell ret = cell[r][c];

      c++;
      if (c == cols) {
        c = 0;
        r++;
      }

      return ret;
    }

    public boolean hasNext() {
      return (r < rows);
    }

    public void remove() {
      throw new UnsupportedOperationException();
    }
  }
}
