package disl.ml.lbs.trajectory;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import edu.gatech.lbs.core.vector.CartesianVector;
import edu.gatech.lbs.core.vector.IVector;
import edu.gatech.lbs.core.world.roadnet.RoadJunction;
import edu.gatech.lbs.core.world.roadnet.RoadMap;
import edu.gatech.lbs.core.world.roadnet.RoadSegment;

public class Trajectory {
  protected int id;
  protected int clusId;
  protected LinkedList<MoPoint> Points;
  protected LinkedList<PointsOnSeg> Route;// cached list of t-fragments
  protected HashMap<Integer, List<CartesianVector>> cellIdtoPoints;
  protected List<Integer> roadSegmentIds;


  public Trajectory(int id) {
    this.id = id;
    this.Points = new LinkedList<MoPoint>();
    this.Route = new LinkedList<PointsOnSeg>();
    // generateRoute();
    this.cellIdtoPoints = new HashMap<Integer, List<CartesianVector>>();
    this.roadSegmentIds = new ArrayList<Integer>();
  }

  public HashMap<Integer, List<CartesianVector>> getCellIdtoPoints() {
    return this.cellIdtoPoints;
  }

  public String printTrajectoryCellSequence() {
    StringBuffer str = new StringBuffer();
    for (int cellId : cellIdtoPoints.keySet()) {
      str.append(cellId);
      str.append(" ");
    }
    return str.toString();
  }

  public String printSegmentIDSequence() {
    StringBuffer str = new StringBuffer();
    for (int segId : this.roadSegmentIds) {
      str.append(segId);
      str.append(" ");
    }
    return str.toString();
  }

  public int getId() {
    return id;
  }

  public void setId(int id) {
    this.id = id;
  }

  public int getClusId() {
    return clusId;
  }

  public void setClusId(int clusId) {
    this.clusId = clusId;
  }

  public LinkedList<MoPoint> getPoints() {
    return Points;
  }

  public void setPoints(LinkedList<MoPoint> points) {
    Points = points;
  }

  public LinkedList<PointsOnSeg> getRoute() {
    return Route;
  }

  public void setRoute(LinkedList<PointsOnSeg> route) {
    Route = route;
  }

  public void generateRoute() {
    PointsOnSeg ps0 = new PointsOnSeg(Points.get(0).getSegid());

    ps0.getV().add(Points.get(0).getV());
    Route.add(ps0);
    roadSegmentIds.add(ps0.getSegid());

    for (int i = 1; i < Points.size(); i++) {
      if (Points.get(i).getSegid() != Points.get(i - 1).getSegid()) {
        PointsOnSeg ps1 = new PointsOnSeg(Points.get(i).getSegid());
        // ps1.setSegid(Points.get(i).getSegid());
        ps1.getV().add(Points.get(i).getV());
        Route.add(ps1);
        roadSegmentIds.add(ps1.getSegid());
      } else {
        if ((i < Points.size() - 1) && (Points.get(i).getSegid() != Points.get(i + 1).getSegid())) Route.getLast()
            .getV().add(Points.get(i).getV());
        if (i == Points.size() - 1) Route.getLast().getV().add(Points.get(i).getV());
      }
    }
    // this.roadSegmentIds=generateSegIds();
  }

  public RoadJunction getStartJunction(RoadMap roadmap) {
    PointsOnSeg startPs = this.getRoute().getFirst();
    RoadSegment startSeg = startPs.getRoadSeg(roadmap);
    return startSeg.getOtherJunction(startPs.getSharedJunction(roadmap, this.getRoute().get(1)));
  }

  public RoadJunction getEndJunction(RoadMap roadmap) {
    PointsOnSeg endPs = this.getRoute().getLast();
    RoadSegment endSeg = endPs.getRoadSeg(roadmap);
    return endSeg.getOtherJunction(endPs.getSharedJunction(roadmap, this.getRoute().get(this.getRoute().size() - 2)));
  }

  public FlowCluster createSegCluster(RoadMap roadmap) {
    FlowCluster sc = new FlowCluster();
    sc.setSegments(this.Route);
    sc.setStartJunc(getStartJunction(roadmap));
    sc.setEndJunc(getEndJunction(roadmap));
    return sc;
  }

  public double getLength(RoadMap rm) {
    List<Integer> segIds = generateSegIds();
    double length = 0;
    for (int segId : segIds) {
      length = length + rm.getRoadSegment(segId).getLength();
    }
    return length;
  }

  public List<Integer> generateSegIds() {
    List<Integer> segIds = new ArrayList<Integer>();
    for (PointsOnSeg ps : Route) {
      if (!segIds.contains(ps.getSegid())) // eliminate duplicate segments
      segIds.add(ps.getSegid());
    }
    return segIds;
  }

  public void printRoute() {
    System.out.print(getId() + " ");
    List<Integer> segIds = generateSegIds();
    for (int i = 0; i < segIds.size(); i++) {
      System.out.print(segIds + " ");
    }
    System.out.println();
  }

  public List<CartesianVector> getGeometryPoints(RoadMap rm) {
    List<Integer> segIds = generateSegIds();
    List<CartesianVector> geoPoints = new LinkedList<CartesianVector>();
    for (Integer segId : segIds) {
      RoadSegment seg = rm.getRoadSegment(segId);

      for (CartesianVector v : seg.getGeometry().getPoints()) {
        geoPoints.add(v);
      }
    }
    return geoPoints;
  }

  public List<CartesianVector> toCartesianVectors() {
    List<CartesianVector> cv = new LinkedList<CartesianVector>();
    for (MoPoint p : this.getPoints()) {
      cv.add(p.getV());
    }
    return cv;
  }

  public List<IVector> toIVectors() {
    List<IVector> cv = new LinkedList<IVector>();
    for (MoPoint p : this.getPoints()) {
      cv.add(p.getV());
    }
    return cv;
  }

  public double distToOtherTrajHausdorff(Trajectory otherTraj, RoadMap roadmap) {
    FlowCluster mySc = createSegCluster(roadmap);
    return mySc.distToOtherFlowCluster(roadmap, otherTraj.createSegCluster(roadmap));
  }
}
