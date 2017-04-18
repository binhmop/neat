package disl.ml.lbs.trajectory;


import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import edu.gatech.lbs.core.vector.IVector;
import edu.gatech.lbs.core.world.roadnet.RoadJunction;
import edu.gatech.lbs.core.world.roadnet.RoadMap;

public class FlowCluster {
  protected LinkedList<PointsOnSeg> flowClus;// forms a route - flow cluster
  protected RoadJunction startJunc;
  protected RoadJunction endJunc;
  protected boolean extendStart;
  protected boolean extendEnd;
  protected Set<Integer> trajIdSet;
  protected int flowComputeCount;

  public FlowCluster() {
    this.flowClus = new LinkedList<PointsOnSeg>();
    this.trajIdSet = new HashSet<Integer>();
    this.extendEnd = true;
    this.extendStart = true;
    this.flowComputeCount = 0;
  }

  public int getSize() {
    return flowClus.size();
  }

  public int getFlowComputeCount() {
    return this.flowComputeCount;
  }

  
  public int getTrajCount(){ 
    return trajIdSet.size();
  }
   
  public Set<Integer> getTrajIdSet() {

    return trajIdSet;
  }

  public void generateTrajIds() {
    // this.TrajIdList = new ArrayList<Integer>();
    for (int i = 0; i < flowClus.size(); i++) {
      List<Integer> tl = flowClus.get(i).getTrajIdList();
      for (int trajId : tl) {
        if (!trajIdSet.contains(trajId)) {
          trajIdSet.add(trajId);
        }
      }
    }
  }

  public LinkedList<PointsOnSeg> getFlowClus() {
    return flowClus;
  }

  public void setSegments(LinkedList<PointsOnSeg> segments) {
    this.flowClus = segments;
  }

  public RoadJunction getStartJunc() {
    return startJunc;
  }

  public void setStartJunc(RoadJunction startJunc) {
    this.startJunc = startJunc;
  }

  public RoadJunction getEndJunc() {
    return endJunc;
  }

  public void setEndJunc(RoadJunction endJunc) {
    this.endJunc = endJunc;
  }

  public void addSeg(PointsOnSeg pos) {
    this.flowClus.add(pos);
  }

  public List<PointsOnSeg> candidatesToAdd(RoadJunction junc, RoadMap rm,
      Collection<PointsOnSeg> posList) {
    List<PointsOnSeg> cands = new ArrayList<PointsOnSeg>();
    for (PointsOnSeg pos : posList) {
      if (pos.isEndPoint(rm, startJunc)) {
        cands.add(pos);
      }
    }
    return cands;

  }

  /*
   * return the densest base cluster among the f-neighbors to begin joining
   */
  public PointsOnSeg maxDense(List<PointsOnSeg> posList) {
    int maxDense = 0;
    int iDensest = 0;
    for (int i = 0; i < posList.size(); i++) {
      if (posList.get(i).numTrajs() > maxDense) {
        iDensest = i;
        maxDense = posList.get(i).numTrajs();
      }
    }
    return posList.get(iDensest);
  }

  // when wq=1, wk=wv=0 i.e. only maximum flows are considered and use the look
  // backward approach
  public PointsOnSeg chooseMergeNeighbor(List<PointsOnSeg> fNeighborhood) {
    int chosenIndex = 0;
    int maxBackwardFlow = 0;
    for (int i = 0; i < fNeighborhood.size(); i++) {
      int curFlow = this.netFlow(fNeighborhood.get(i));
      if (curFlow > maxBackwardFlow) {
        maxBackwardFlow = curFlow;
        chosenIndex = i;
      }
    }
    return fNeighborhood.get(chosenIndex);
  }

  public PointsOnSeg chooseMergeNeighbor(RoadMap rm, PointsOnSeg endRouteSeg,
      List<PointsOnSeg> fNeighborhood, double wq, double wk, double wv) {
    //if (wq==1) return chooseMaxFlowNeighbor(rm, endRouteSeg, fNeighborhood, Collection<PointsOnSeg> posList, 1);//new for look for/backward
    double maxSelectivity = 0;
    int chosenIndex = 0;
    int fNeighborhoodSize = fNeighborhood.size();
    int[] denominators = { 0, 0, 0 };
    int[] zeroDe = { 1, 1, 1 };
    int[][] neighborAbsWeights = new int[fNeighborhoodSize][3];// store the 3
                                                               // absolute
                                                               // weights q, k,
                                                               // v of each
                                                               // f-neighbor
    double[] neighborRelWeights = new double[fNeighborhoodSize];// store the
                                                                // relative
                                                                // weights
    double[] maxFactors = {0,0,0};
    for (int i = 0; i < fNeighborhoodSize; i++) {
      neighborAbsWeights[i][0] = endRouteSeg.getNumOfSharedTrajs(fNeighborhood
          .get(i));
      neighborAbsWeights[i][1] = fNeighborhood.get(i).numPoints();
      neighborAbsWeights[i][2] = (int) fNeighborhood.get(i).getSpeedLimit(rm);
      for (int j = 0; j < 3; j++) {
        denominators[j] += neighborAbsWeights[i][j];
        if (maxFactors[j]<neighborAbsWeights[i][j]) maxFactors[j]= neighborAbsWeights[i][j];//for adaptive weights 
      }
    }
    for (int j = 0; j < 3; j++) {
      if (denominators[j] == 0) {
        zeroDe[j] = 0;
        System.out.println("One of the denominators is zero" + j);
      }
    }

    if (wq * wk * wv < 0) {// when using adaptive weights, assign wq=wk=wv=-1
//      double weights[] = getAdaptiveWeights(maxFactors[0] / denominators[0],
//          maxFactors[1] / denominators[1], maxFactors[2] / denominators[2]);
      double weights[] = getAdaptiveWeights(maxFactors[0] / denominators[0],
          maxFactors[1] / denominators[1], 0);
      wq = weights[0];
      wk = weights[1];
      wv = weights[2];
    }
    if (zeroDe[0] + zeroDe[1] + zeroDe[2] < 3) {
      // System.out.println("One of the denominators is zero");
      return null;
    } else {
      for (int i = 0; i < fNeighborhoodSize; i++) {
        neighborRelWeights[i] = wq * neighborAbsWeights[i][0] / denominators[0]
            + wk * neighborAbsWeights[i][1] / denominators[1] + wv
            * neighborAbsWeights[i][2] / denominators[2];
        if (maxSelectivity < neighborRelWeights[i]) {
          maxSelectivity = neighborRelWeights[i];
          chosenIndex = i;
        }
      }
    }
    return fNeighborhood.get(chosenIndex);
  }
  
  public double[] adaptiveWeights(double q, double k, double v){
    double[] weights = new double[3];
    double[] factors = {q,k,v};
    double minFactor= Math.min(Math.min(q,k),Math.min(k,v));
    int minIndex=0;
    for (int i=0; i<3; i++){
      if (factors[i]== minFactor) {
        minIndex=i;
        weights[i]=0;
      }             
      break;
    }
    for (int i=0; i<3;i++){
      if(i==minIndex) continue;
      weights[i]= factors[i]/(q+k+v-minFactor);
    }
    return weights;
  }
  
  public double[] getAdaptiveWeights(double q, double k, double v){
    if ((q+k+v)==0) return new double[]{0,0,0};
    double[] weights = {q,k,v};
    double alpha = 1/(q+k+v);
    for (int i=0; i<3;i++){
      weights[i]= weights[i]*alpha;
    }
    return weights;
  }
  // when wq=1, wk=wv=0 i.e. only maximum flows are considered and use the look
  // forward k hops approach
  public PointsOnSeg chooseMaxFlowNeighbor(RoadMap rm, PointsOnSeg endRouteSeg,
      List<PointsOnSeg> fNeighborhood, Collection<PointsOnSeg> posList,
      int kHops) {
    // int chosenIndex=0;
    int maxBackwardFlow = 0;

    List<PointsOnSeg> maxFlowNeighborList = new LinkedList<PointsOnSeg>();
    for (int i = 0; i < fNeighborhood.size(); i++) {
      int curFlow = endRouteSeg.getNumOfSharedTrajs(fNeighborhood.get(i));
      if (curFlow > maxBackwardFlow) {
        maxBackwardFlow = curFlow;
      }
    }
    for (int i = 0; i < fNeighborhood.size(); i++) {
      int curFlow = endRouteSeg.getNumOfSharedTrajs(fNeighborhood.get(i));
      if (curFlow == maxBackwardFlow)
        maxFlowNeighborList.add(fNeighborhood.get(i));
    }
    if (maxFlowNeighborList.size() > 1) {
      if (kHops == 0)
        return chooseMergeNeighbor(maxFlowNeighborList);// look backward
      else {
        int maxForwardFlow = 0;// look forward
        PointsOnSeg chosenPs = maxFlowNeighborList.get(0);
        for (PointsOnSeg pos : maxFlowNeighborList) {
          RoadJunction endPoint = (pos.getRoadSeg(rm).getSourceJunction() == this.startJunc) ? pos
              .getRoadSeg(rm).getTargetJunction() : pos.getRoadSeg(rm)
              .getSourceJunction();
          int curMax = pos.getMaxNumOfSharedTrajs(rm, posList, endPoint);
          if (curMax > maxForwardFlow) {
            maxForwardFlow = curMax;
            chosenPs = pos;
          }
        }
        return chosenPs;
      }
    } else {
      return maxFlowNeighborList.get(0);
    }
  }
  
  public PointsOnSeg[] concatenateJuncs(RoadMap rm,
      Collection<PointsOnSeg> posList, double wq, double wk, double wv, int khops) {
    int countTrajs0 = 0;
    int countTrajs1 = 0;
    PointsOnSeg[] ps = new PointsOnSeg[2];
    List<PointsOnSeg> candFront = new LinkedList<PointsOnSeg>();
    List<PointsOnSeg> candEnd = new LinkedList<PointsOnSeg>();
    for (PointsOnSeg pos : posList) {
      if (extendStart) {
        if (pos.isEndPoint(rm, startJunc)) {

          /*
           * if (pos.numTrajs()>countTrajs0){//chosen by road density
           * countTrajs0 = pos.numTrajs(); ps[0] = pos; }
           */
          int netflow = flowClus.getFirst().getNumOfSharedTrajs(pos);
          this.flowComputeCount++;

          if (netflow > 0)
            candFront.add(pos);// pos is an f-neighbor of startJunc
          if (netflow > countTrajs0) {// calculate maximum flow
            countTrajs0 = netflow;
            ps[0] = pos;
          }
          // if (pos.getSpeedLimit(rm)>countTrajs0){//chosen by road speed limit
          // countTrajs0 = (int)pos.getSpeedLimit(rm);
          // ps[0] = pos;
          // }
        }
      }

      if (extendEnd) {
        if (pos.isEndPoint(rm, endJunc)) {

          /*
           * if (pos.numTrajs()>countTrajs1){ countTrajs1 = pos.numTrajs();
           * ps[1] = pos; }
           */
          int flow = flowClus.getLast().getNumOfSharedTrajs(pos);
          this.flowComputeCount++;

          if (flow > 0)
            candEnd.add(pos);// pos is an f-neighbor of endJunc
          if (flow > countTrajs1) {
            countTrajs1 = flow;
            ps[1] = pos;
          }

          // if (pos.getSpeedLimit(rm)>countTrajs1){
          // countTrajs1 = (int)pos.getSpeedLimit(rm);
          // ps[1] = pos;
          // }
        }
      }
    }

    // use factors

    if (candFront.size() != 0){
      if (wq==1 && khops>=0) ps[0]= chooseMaxFlowNeighbor(rm, flowClus.getFirst(), candFront, posList, khops);//new for look for/backward
      else ps[0] = chooseMergeNeighbor(rm, flowClus.getFirst(), candFront, wq, wk, wv);
      
    }
    if (candEnd.size() != 0){
      if (wq==1 && khops>=0) ps[1]= chooseMaxFlowNeighbor(rm, flowClus.getLast(), candEnd, posList, khops);//new for look for/backward
      else ps[1] = chooseMergeNeighbor(rm, flowClus.getLast(), candEnd, wq, wk, wv);
      
    }  
    if (ps[0] == null)
      extendStart = false;
    else {
      this.flowClus.addFirst(ps[0]);
      this.startJunc = (ps[0].getRoadSeg(rm).getSourceJunction() == this.startJunc) ? ps[0]
          .getRoadSeg(rm).getTargetJunction() : ps[0].getRoadSeg(rm)
          .getSourceJunction();
    }
    if (ps[1] == null)
      extendEnd = false;
    else {
      this.flowClus.addLast(ps[1]);
      this.endJunc = (ps[1].getRoadSeg(rm).getSourceJunction() == this.endJunc) ? ps[1]
          .getRoadSeg(rm).getTargetJunction() : ps[1].getRoadSeg(rm)
          .getSourceJunction();
    }
    return ps;
  }

  // returns the list of points on the segments of the routes from first to last
  public List<IVector> representativePoints(RoadMap rm) {
    List<IVector> repPoints = new LinkedList<IVector>();
    for (int i = 0; i < flowClus.getFirst().getV().size(); i++) {
      repPoints.add(flowClus.getFirst().getV().get(i));
    }

    for (int i = 0; i < flowClus.size() - 1; i++) {
      repPoints.add(flowClus.get(i).getSharedEndPoint(rm, flowClus.get(i + 1)));
    }
    for (int i = 0; i < flowClus.getLast().getV().size(); i++) {
      repPoints.add(flowClus.getLast().getV().get(i));
    }
    return repPoints;
  }

  public boolean isExtensible() {
    return (extendStart || extendEnd);
  }

  public boolean isExtendStart() {
    return extendStart;
  }

  public void setExtendStart(boolean extendStart) {
    this.extendStart = extendStart;
  }

  public boolean isExtendEnd() {
    return extendEnd;
  }

  public void setExtendEnd(boolean extendEnd) {
    this.extendEnd = extendEnd;
  }

  public int netFlow(FlowCluster otherFlowCluster) {// netflow(flow,flow)
    int num = 0;
    for (int trajId : this.trajIdSet) {
      if (otherFlowCluster.getTrajIdSet().contains(trajId))
        num++;
    }
    return num;
  }

  public int netFlow(PointsOnSeg ps) {// netflow(flow,t-fragment)
    int num = 0;
    for (int trajId : this.trajIdSet) {
      if (ps.getTrajIdList().contains(trajId))
        num++;
    }
    return num;
  }

  public double distToOtherFlowCluster(RoadMap rm, FlowCluster otherFlow) {
    double dss = rm.getShortestRoute(this.getStartJunc().getRoadnetLocation(),
        otherFlow.getStartJunc().getRoadnetLocation()).getLength();
    double dse = rm.getShortestRoute(this.getStartJunc().getRoadnetLocation(),
        otherFlow.getEndJunc().getRoadnetLocation()).getLength();
    double des = rm.getShortestRoute(this.getEndJunc().getRoadnetLocation(),
        otherFlow.getStartJunc().getRoadnetLocation()).getLength();
    double dee = rm.getShortestRoute(this.getEndJunc().getRoadnetLocation(),
        otherFlow.getEndJunc().getRoadnetLocation()).getLength();
    // return Math.max(Math.min(dss, dse), Math.min(des, dee));
    double max1 = Math.max(Math.min(dss, dse), Math.min(des, dee));
    double max2 = Math.max(Math.min(dss, des), Math.min(dse, dee));
    return Math.max(max1, max2);
  }

  public double euclideanDistToOtherFlowCluster(FlowCluster otherFlow) {
    MoPoint curStartJunc = new MoPoint(-1,this.getStartJunc().getCartesianLocation());
    MoPoint curEndJunc = new MoPoint(-1,this.getEndJunc().getCartesianLocation());
    MoPoint otherStartJunc = new MoPoint(-1,otherFlow.getStartJunc().getCartesianLocation());
    MoPoint otherEndJunc = new MoPoint(-1,otherFlow.getEndJunc().getCartesianLocation());
    double dss = curStartJunc.distToPoint(otherStartJunc);
    double dse = curStartJunc.distToPoint(otherEndJunc);
    double des = curEndJunc.distToPoint(otherStartJunc);
    double dee = curEndJunc.distToPoint(otherEndJunc);
    
    // double max1 = Math.max(Math.min(dss, dse), Math.min(des, dee));
    // double max2 = Math.max(Math.min(dss, des), Math.min(dse, dee));
    return Math.min(Math.min(dss, dse), Math.min(des, dee));
  }

  public double getLength(RoadMap rm) {
    double length = 0;
    for (int i = 0; i < this.flowClus.size(); i++) {
      length += this.flowClus.get(i).getRoadSeg(rm).getLength();

    }
    return length;
  }
  public double[] getStreamStats(RoadMap rm){
    double stat[] = {0,0,0};
    if (flowClus.size()==1) {
      stat[0] = 0;
      stat[1] = flowClus.get(0).numPoints();
      stat[2] = flowClus.get(0).getSpeedLimit(rm);
      return stat;
    }
    for (int i=0; i<this.flowClus.size()-1; i++){
      stat[0] += flowClus.get(i).getNumOfSharedTrajs(flowClus.get(i+1)); 
      stat[1] += flowClus.get(i).numPoints();
      stat[2] += flowClus.get(i).getSpeedLimit(rm);
      if (i==this.flowClus.size()-1){
        stat[1] += flowClus.get(i+1).numPoints();
        stat[2] += flowClus.get(i+1).getSpeedLimit(rm);
      }
    }
//    for (int j=0; j<3;j++){
//      if(j==0) 
//        stat[j] = stat[j]/(flowClus.size()-1);
//      else
//        stat[j] = stat[j]/flowClus.size();
//    }
    return stat;
  }
}

