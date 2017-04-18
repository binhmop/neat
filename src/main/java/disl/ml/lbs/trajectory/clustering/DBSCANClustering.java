package disl.ml.lbs.trajectory.clustering;


import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import disl.ml.lbs.trajectory.PointsOnSeg;
import edu.gatech.lbs.core.world.roadnet.RoadMap;

/**
 * Density-based clustering of t-fragments
 */
public class DBSCANClustering {
  protected double eps;
  protected RoadMap roadmap;
  protected Collection<PointsOnSeg> tfragments;
  protected List<LinkedList<PointsOnSeg>> clusters;


  public DBSCANClustering(double eps, RoadMap roadmap, Collection<PointsOnSeg> tfragments) {
    this.eps = eps;
    this.roadmap = roadmap;
    this.tfragments = tfragments;
    this.clusters = new ArrayList<LinkedList<PointsOnSeg>>();
  }

  public void setEps(double eps) {
    this.eps = eps;
  }

  public List<LinkedList<PointsOnSeg>> getClusters() {
    return clusters;
  }

  public void dbscanClusterSegments() { // dbscan-like merging of t-fragments
    ArrayList<PointsOnSeg> tfragmentList = new ArrayList<PointsOnSeg>(tfragments);
    ArrayList<PointsOnSeg> cloneSegments = (ArrayList<PointsOnSeg>) tfragmentList.clone();
    while (!cloneSegments.isEmpty()) {
      LinkedList<PointsOnSeg> curSegClus = new LinkedList<PointsOnSeg>();
      // PointsOnSeg ps0 = getDensestSeg(cloneSegments);
      PointsOnSeg ps0 = cloneSegments.get(cloneSegments.size() - 1);
      curSegClus.add(ps0);
      cloneSegments.remove(cloneSegments.size() - 1);
      for (int i = 0; i < curSegClus.size(); i++) {
        PointsOnSeg myPos = curSegClus.get(i);
        for (int j = cloneSegments.size() - 1; j > 0; j--) {
          PointsOnSeg otherPos = cloneSegments.get(j);
          if (myPos.distToOtherPos(roadmap, otherPos) < eps) {
            curSegClus.add(otherPos);
            cloneSegments.remove(j);
          }
        }
      }
      clusters.add(curSegClus);
    }

  }

  public void dbscanClusterSegments(HashMap<Integer, ArrayList<Double>> distList) { //DBSCAN clustering given pre-computed distances
    // HashMap<Integer,ArrayList<Double>> distList = calculateDistList(tfragments);
    ArrayList<PointsOnSeg> tfragmentList = new ArrayList<PointsOnSeg>(tfragments);

    int n = tfragmentList.size();
    int[] classified = new int[n];
    for (int i = 0; i < n; i++)
      classified[i] = 0;// initially all the t-fragments are unclassified
    for (int i = 0; i < n; i++) {
      if (classified[i] == 0) {
        ArrayList<Integer> queue = new ArrayList<Integer>();// queue to expand the cluster
        LinkedList<PointsOnSeg> mySegClus = new LinkedList<PointsOnSeg>();
        mySegClus.add(tfragmentList.get(i));
        classified[i] = 1;
        List<Integer> epsNeighbors = getEpsNeighbors(i, eps, distList);

        for (Integer eNeighbor : epsNeighbors) {
          queue.add(eNeighbor);
        }
        while (!queue.isEmpty()) {
          int first = queue.get(0);
          if (classified[first] == 0) {
            mySegClus.add(tfragmentList.get(first));
            classified[first] = 1;
            queue.remove(0);
            List<Integer> eN = getEpsNeighbors(first, eps, distList);
            if (!eN.isEmpty()) {
              for (Integer eNeighbor : eN) {
                queue.add(eNeighbor);
              }
            }
          } else queue.remove(0);

        }
        clusters.add(mySegClus);
      }

    }

  }

  public List<Integer> getEpsNeighbors(int iCheck, double eps, HashMap<Integer, ArrayList<Double>> distList) {
    int n = distList.size();
    if (iCheck >= n) return null;
    List<Integer> neighborIndex = new ArrayList<Integer>();
    if (iCheck < (n - 1)) {
      for (int k = 0; k < distList.get(iCheck).size(); k++) {// check entry of iCheck in the distList
        if (distList.get(iCheck).get(k) < eps) {
          neighborIndex.add(iCheck + k + 1);
        }
      }
    }
    for (int j = iCheck - 1; j > 0; j--) {// check other entry that has distance to iCheck
      if (distList.get(j).get(iCheck - j - 1) < eps) {
        neighborIndex.add(j);
      }
    }
    return neighborIndex;
  }

  public double calculateEps() {
    double sum = 0;
    double minDist = Double.MAX_VALUE;
    double maxDist = 0;
    ArrayList<PointsOnSeg> tfragmentList = new ArrayList<PointsOnSeg>(tfragments);
    for (int i = 0; i < this.tfragments.size() - 1; i++) {
      for (int j = i + 1; j < this.tfragments.size(); j++) {
        double dist = tfragmentList.get(i).distToOtherPos(this.roadmap, tfragmentList.get(j));

        if (dist < minDist) minDist = dist;
        if (dist > maxDist) maxDist = dist;
        sum += dist;
      }
    }
    double avgDist = sum / ((tfragmentList.size() - 1) * (tfragmentList.size() - 2) / 2);
    System.out.println(String.format("Pairwise t-fragment distance: avg = %.3f, min = %.3f, max = %.3f", avgDist, minDist, maxDist));
    return avgDist;
  }

  // return the adjacent list of distance between each pair of t-fragments and calculate Eps threshold
  public HashMap<Integer, ArrayList<Double>> calculateDistList(List<PointsOnSeg> posList) {
    HashMap<Integer, ArrayList<Double>> distanceList = new HashMap<Integer, ArrayList<Double>>();

    double sum = 0;
    double minDist = Double.MAX_VALUE;
    double maxDist = 0;

    int n = posList.size();
    for (int i = 0; i < n - 1; i++) {
      PointsOnSeg curPos = posList.get(i);
      ArrayList<Double> tmpList = new ArrayList<Double>();
      for (int j = i + 1; j < n; j++) {
        double dist = curPos.distToOtherPos(roadmap, posList.get(j));
        tmpList.add(dist);

        if (dist < minDist) minDist = dist;
        if (dist > maxDist) maxDist = dist;
        sum += dist;

      }
      distanceList.put(i, tmpList);
    }
    double avgDist = sum / ((n - 1) * (n - 2) / 2);
    System.out.println(String.format("Pairwise t-fragment distance: avg = %.3f, min = %.3f, max = %.3f", avgDist, minDist, maxDist));
    setEps(Math.round(avgDist / 2));
    return distanceList;
  }

}
