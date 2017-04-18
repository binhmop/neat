package disl.ml.lbs.trajectory;


import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import org.apache.commons.collections4.CollectionUtils;
import edu.gatech.lbs.core.vector.IVector;
import edu.gatech.lbs.core.world.roadnet.RoadJunction;
import edu.gatech.lbs.core.world.roadnet.RoadMap;
import edu.gatech.lbs.core.world.roadnet.RoadSegment;

public class PointsOnSeg {
  protected int segid;

  protected List<IVector> v;
  protected List<Integer> trajIdList;


  public PointsOnSeg(int id) {
    segid = id;
    v = new LinkedList<IVector>();
    trajIdList = new ArrayList<Integer>();
  }

  public List<Integer> getTrajIdList() {
    return trajIdList;
  }

  public void setTrajIdList(List<Integer> trajIdList) {
    this.trajIdList = trajIdList;
  }

  public int getSegid() {
    return segid;
  }

  public void setSegid(int segid) {
    this.segid = segid;
  }

  public List<IVector> getV() {
    return v;
  }

  public void setV(List<IVector> v) {
    this.v = v;
  }

  public int numPoints() {
    return v.size();
  }

  public int numTrajs() {
    return trajIdList.size();
  }

  public float getLength(RoadMap rm) {
    return this.getRoadSeg(rm).getLength();
  }

  public float getSpeedLimit(RoadMap rm) {
    return this.getRoadSeg(rm).getSpeedLimit();
  }

  public RoadSegment getRoadSeg(RoadMap roadmap) {
    return roadmap.getRoadSegment(segid);
  }

  public boolean isNeighbor(RoadMap roadmap, PointsOnSeg otherSeg) {
    RoadSegment rs1 = this.getRoadSeg(roadmap);
    RoadSegment rs2 = otherSeg.getRoadSeg(roadmap);
    if ((rs1.getJunctionIndex(rs2.getSourceJunction()) != -1) || (rs1.getJunctionIndex(rs2.getTargetJunction()) != -1)) return true;
    else return false;
  }

  public IVector getSharedEndPoint(RoadMap roadmap, PointsOnSeg neighborSeg) {
    RoadSegment rs1 = this.getRoadSeg(roadmap);
    RoadSegment rs2 = neighborSeg.getRoadSeg(roadmap);
    if (rs1.getJunctionIndex(rs2.getSourceJunction()) != -1) return rs2.getSourceLocation();
    else return rs2.getTargetLocation();
  }

  public RoadJunction getSharedJunction(RoadMap roadmap, PointsOnSeg otherPs) {
    RoadSegment rs1 = this.getRoadSeg(roadmap);
    RoadSegment rs2 = otherPs.getRoadSeg(roadmap);
    if ((rs1.getJunctionIndex(rs2.getSourceJunction()) == -1) || (rs1.getJunctionIndex(rs2.getTargetJunction()) == -1)) return null;
    if (rs1.getJunctionIndex(rs2.getSourceJunction()) != -1) return rs2.getSourceJunction();
    else return rs2.getTargetJunction();
  }

  public boolean isEndPoint(RoadMap rm, RoadJunction junc) {
    return (this.getRoadSeg(rm).getJunctionIndex(junc) != -1);

  }

  public void add(PointsOnSeg pos) {
    if (this.getSegid() == pos.getSegid()) {
      for (IVector coor : pos.getV()) {
        this.getV().add(coor);
      }
    }
  }

  public Collection<Integer> getIntersectTrajs(PointsOnSeg pos) {
    Collection<Integer> intersection = CollectionUtils.intersection(this.trajIdList, pos.getTrajIdList());
    return intersection;
  }

  public int getNumOfSharedTrajs(PointsOnSeg pos) {
    return getIntersectTrajs(pos).size();
  }

  public List<PointsOnSeg> getNeighbors(RoadMap rm, Collection<PointsOnSeg> posList, RoadJunction endPoint) {
    List<PointsOnSeg> cand = new LinkedList<PointsOnSeg>();
    for (PointsOnSeg pos : posList) {
      if (this.equals(pos)) continue;
      if (pos.isEndPoint(rm, endPoint)) {
        if (this.getNumOfSharedTrajs(pos) > 0) cand.add(pos);
      }
    }
    return cand;
  }

  public int getMaxNumOfSharedTrajs(RoadMap rm, Collection<PointsOnSeg> posList, RoadJunction endPoint) {
    int maxFlow = 0;
    for (PointsOnSeg pos : posList) {
      if (this.equals(pos)) continue;
      if (pos.isEndPoint(rm, endPoint)) {
        if (this.getNumOfSharedTrajs(pos) > maxFlow) maxFlow = this.getNumOfSharedTrajs(pos);
      }
    }
    return maxFlow;
  }

  public double distToOtherPos(RoadMap rm, PointsOnSeg otherPos) {
    RoadSegment rs1 = this.getRoadSeg(rm);
    RoadSegment rs2 = otherPos.getRoadSeg(rm);
    double dss = rm.getShortestRoute(rs1.getSourceJunction().getRoadnetLocation(),
        rs2.getSourceJunction().getRoadnetLocation()).getLength();
    double dse = rm.getShortestRoute(rs1.getSourceJunction().getRoadnetLocation(),
        rs2.getTargetJunction().getRoadnetLocation()).getLength();
    double des = rm.getShortestRoute(rs1.getTargetJunction().getRoadnetLocation(),
        rs2.getSourceJunction().getRoadnetLocation()).getLength();
    double dee = rm.getShortestRoute(rs1.getTargetJunction().getRoadnetLocation(),
        rs2.getTargetJunction().getRoadnetLocation()).getLength();
    // double max1 = Math.max(Math.min(dss, dse), Math.min(des, dee));
    // double max2 = Math.max(Math.min(dss, des), Math.min(dse, dee));
    return Math.min(Math.min(dss, dse), Math.min(des, dee));
    // return Math.max(Math.min(dss, dse), Math.min(des, dee));
  }

}
