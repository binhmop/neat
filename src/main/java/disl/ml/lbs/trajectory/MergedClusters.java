package disl.ml.lbs.trajectory;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import org.apache.commons.collections4.CollectionUtils;
import edu.gatech.lbs.core.world.roadnet.RoadMap;

/**
 * Density-based merging of flow clusters
 */
public class MergedClusters {
  protected List<FlowCluster> flows;// input - flow clusters
  protected HashMap<Integer, List<FlowCluster>> mClusters;// output - density based opt
  protected RoadMap roadmap;
  protected HashMap<Integer, List<Integer>> trajIdList;
  protected double eps; // distance threshold (mm)
  
  public MergedClusters(List<FlowCluster> flows, RoadMap rm, double distThres) {
    this.flows = flows;
    this.roadmap = rm;
    this.trajIdList = new HashMap<Integer, List<Integer>>();
    this.mClusters = new HashMap<Integer, List<FlowCluster>>();
    this.eps = distThres;
    // this.eps = calculateEps();
    merge(eps);
  }

  public double getEps() {
    return eps;
  }

  // return the adjacent list of distance between each pair of flow clusters
  public HashMap<Integer, ArrayList<Double>> calculateDistList(List<FlowCluster> segClus) {
    HashMap<Integer, ArrayList<Double>> distanceList = new HashMap<Integer, ArrayList<Double>>();
    int n = segClus.size();
    double sum = 0;
    for (int i = 0; i < n - 1; i++) {
      FlowCluster mySc = segClus.get(i);
      ArrayList<Double> tmpList = new ArrayList<Double>();
      for (int j = i + 1; j < n; j++) {
        double tmpDist = mySc.distToOtherFlowCluster(this.roadmap, segClus.get(j));
        tmpList.add(tmpDist);
        sum += tmpDist;
      }
      distanceList.put(i, tmpList);
    }
    double avgDist = sum / ((n - 1) * (n - 2) / 2);
    System.out.println("The average distance is: " + avgDist);
    this.eps = avgDist;

    return distanceList;
  }

  public double calculateEps() {
    double sum = 0;

    for (int i = 0; i < flows.size() - 1; i++) {

      for (int j = i + 1; j < flows.size(); j++) {
        double dist = this.flows.get(i).distToOtherFlowCluster(this.roadmap, this.flows.get(j));
        sum += dist;
      }
    }
    double avgDist = sum / ((flows.size() - 1) * (flows.size() - 2) / 2);
    return avgDist;
  }

/*  public double[][] distMatrix generateDistMatrix(){
    double sum = 0;
    double[][] distMatrix = new double[flows.size()][flows.size]
     for (int i=0; i<flows.size(); i++)
         {
       distMatrix[i][i] = 0;
             for (int j = i+1; j<flows.size(); j++)
             {
               distMatrix[i][j] = this.flows.get(i).distToOtherFlowCluster(this.roadmap, this.flows.get(j));
               distMatrix[j][i] = distMatrix[i][j];
               sum += distMatrix[i][j];
             }
         }
     this.eps = (double)sum/((segClus.size()-1)*(segClus.size()-2)/2);
     return distMatrix;
  }*/
  
  public HashMap<Integer, List<FlowCluster>> getClusters() {
    return mClusters;
  }

  public void setEps(double eps) {
    this.eps = eps;
  }
  
  public void merge(double eps) {
    setEps(eps);
    int count = 0;
    while (flows.size() > 0) {
      List<FlowCluster> newClus = new ArrayList<FlowCluster>();
      newClus.add(flows.get(flows.size() - 1));
      flows.remove(flows.size() - 1);
      for (int i = 0; i < newClus.size(); i++) {
        FlowCluster mySc = newClus.get(i);
        for (int j = flows.size() - 1; j >= 0; j--) {
          FlowCluster otherSc = flows.get(j);
          if ((mySc.euclideanDistToOtherFlowCluster(otherSc)) < this.eps) {
            if ((mySc.distToOtherFlowCluster(roadmap, otherSc)) < this.eps) {
              newClus.add(otherSc);
              this.flows.remove(j);
            }
          }
        }
      }
      this.mClusters.put(count, newClus);
      count++;
    }


  }

  public double getLengthOf(List<FlowCluster> scList) {
    double length = 0;
    for (int i = 0; i < scList.size(); i++) {
      length += scList.get(i).getLength(this.roadmap);
    }
    return length;
  }

  public List<Integer> getTrajListOf(List<FlowCluster> fcList) {
    List<Integer> myIdList = new ArrayList<Integer>();

    myIdList.addAll(fcList.get(0).getTrajIdSet());
    for (int j = 1; j < fcList.size(); j++) {
      Set<Integer> tmp = fcList.get(j).getTrajIdSet();
      for (int trajId : tmp) {
        if (!myIdList.contains(trajId)) myIdList.add(trajId);
      }
    }
    return myIdList;
  }

  public double getTotalLength() {
    double total = 0;
    for (List<FlowCluster> sc : this.mClusters.values()) {
      total = total + getLengthOf(sc);
    }
    return total;
  }

  // dist = max_i(clus1){min_j(clus2){dist(fc_i(clus1),fc_j(clus2))}}
  public double distOfMergedClusters(List<FlowCluster> scList1, List<FlowCluster> scList2) {
    double maxij = 0;
    for (int i = 0; i < scList1.size(); i++) {
      double min = scList1.get(i).distToOtherFlowCluster(this.roadmap, scList2.get(0));
      for (int j = 1; j < scList2.size(); j++) {
        if (min > scList1.get(i).distToOtherFlowCluster(this.roadmap, scList2.get(j))) min = scList1.get(i)
            .distToOtherFlowCluster(this.roadmap, scList2.get(j));
      }
      if (min > maxij) maxij = min;
    }
    return maxij;
  }

  public void generatetrajIdList() {

    for (int i = 0; i < this.mClusters.size(); i++) {
      this.trajIdList.put(i, getTrajListOf(this.mClusters.get(i)));
    }
  }

  public int sharedElements(List<Integer> l1, List<Integer> l2) {
    return CollectionUtils.intersection(l1, l2).size();
  }


  public double[] clusterValid() {
    double[] valid = new double[3];
    int n = this.mClusters.size();
    int[] tuples = new int[n];// store index of the closest cluster to cluster [i]
    double[] minDists = new double[n];// store the distance from a cluster to its closest cluster

    generatetrajIdList();

    // calculate distances between clusters
    HashMap<Integer, ArrayList<Double>> distanceList = new HashMap<Integer, ArrayList<Double>>();
    for (int i = 0; i < n - 1; i++) {
      List<FlowCluster> mySC = this.mClusters.get(i);
      ArrayList<Double> tmpList = new ArrayList<Double>();
      for (int j = i + 1; j < n; j++) {
        tmpList.add(distOfMergedClusters(mySC, this.mClusters.get(j)));
      }
      distanceList.put(i, tmpList);
    }

    // find the closest cluster for each cluster
    for (int i = 0; i < n; i++) {
      if (i < n - 1) {
        minDists[i] = distanceList.get(i).get(0);
        tuples[i] = i + 1;
        for (int j = 0; j < distanceList.get(i).size(); j++) {
          if (distanceList.get(i).get(j) < minDists[i]) {
            minDists[i] = distanceList.get(i).get(j);
            tuples[i] = i + j + 1;
          }
        }
      } else {
        minDists[i] = distanceList.get(n - 2).get(0);
        tuples[i] = n - 2;
      }
      for (int j = i - 1; j > 0; j--) {

        if (distanceList.get(j).get(i - j - 1) < minDists[i]) {
          minDists[i] = distanceList.get(j).get(i - j - 1);
          tuples[i] = j;
        }
      }
    }

    // trajSharedBased
    double trajSharedBased = 0;
    for (int i = 0; i < n; i++) {
      double tmp0;
      int s = sharedElements(trajIdList.get(i), trajIdList.get(tuples[i]));
      tmp0 = (double) s / (trajIdList.get(i).size() + trajIdList.get(tuples[i]).size() - s);
      trajSharedBased += tmp0;
    }
    valid[2] = trajSharedBased / n;
    double sum = 0;
    for (int i = 0; i < n; i++) {
      sum += minDists[i];
    }
    valid[0] = sum / getTotalLength();
    // lengthBased
    double lengthBased = 0;
    for (int i = 0; i < n; i++) {
      double tmp1;
      tmp1 = 2 * minDists[i]
          / (getLengthOf(this.mClusters.get(i)) + getLengthOf(this.mClusters.get(tuples[i])));
      lengthBased += tmp1;
    }
    valid[1] = lengthBased / n;
    return valid;
  }


}
