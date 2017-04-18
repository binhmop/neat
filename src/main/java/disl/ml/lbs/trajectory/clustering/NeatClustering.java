package disl.ml.lbs.trajectory.clustering;


import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;
import disl.ml.lbs.trajectory.FlowCluster;
import disl.ml.lbs.trajectory.FlowClusterComparator;
import disl.ml.lbs.trajectory.MergedClusters;
import disl.ml.lbs.trajectory.PointsOnSeg;
import disl.ml.lbs.trajectory.Trajectory;
import disl.ml.lbs.trajectory.gui.BaseClusterDrawer;
import disl.ml.lbs.trajectory.gui.FlowClusterDrawer;
import disl.ml.lbs.trajectory.gui.MergedClusterDrawer;
import disl.ml.lbs.trajectory.gui.TrajectoryClusterDrawer;
import disl.ml.lbs.util.Const;
import disl.ml.lbs.util.DataImporter;
import edu.gatech.lbs.core.FileHelper;
import edu.gatech.lbs.core.logging.Logz;
import edu.gatech.lbs.core.vector.IVector;
import edu.gatech.lbs.core.world.IWorld;
import edu.gatech.lbs.core.world.roadnet.RoadMap;
import edu.gatech.lbs.sim.config.XmlWorldConfigInterpreter;
import edu.gatech.lbs.sim.gui.SimPanel;
import edu.gatech.lbs.sim.gui.drawer.IDrawer;
import edu.gatech.lbs.sim.gui.drawer.RoadMapDrawer;
import edu.gatech.lbs.sim.gui.drawer.TrajectoriesDrawer;

/**
 * Road-Network Aware Trajectory Clustering: Integrating Locality, Flow, and Density (NEAT)
 * 
 * @author Binh Han
 *
 */
public class NeatClustering {
  static final String frameCaption = "Road Network Aware Trajectory Clustering";
  protected Collection<Trajectory> trajs;
  protected HashMap<Integer, PointsOnSeg> tfragments; // list of set of t-fragments PointsOnSegments (base clusters)
  protected List<FlowCluster> flowClusters; // flow clusters
  protected RoadMap roadmap;
  protected Collection<List<FlowCluster>> mClus; // final clusters
  protected static int segCountThres = 25;
  protected static int trajCountThres = 25;
  protected long startTime;
  protected int concateCount;// debug


  public NeatClustering(String configFile) {
    startTime = System.nanoTime();
    DataImporter im = new DataImporter();
    this.trajs = im.loadTrajectories(Const.traceFile);
    for (Trajectory myTraj : trajs) {
      myTraj.generateRoute();
      // myTraj.printRoute();
      // myTraj.generateSegIds();
    }
    this.roadmap = (RoadMap) loadRoadmap(configFile);
    this.flowClusters = new LinkedList<FlowCluster>();
    this.mClus = new ArrayList<List<FlowCluster>>();
    this.concateCount = 0;
  }

  public IWorld loadRoadmap(String configFile) {
    IWorld world = null;
    try {
      InputStream in = FileHelper.openFileOrUrl(configFile);
      String configText = FileHelper.getContentsFromInputStream(in);
      in.close();

      Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder()
          .parse(new InputSource(new StringReader(configText)));
      Element rootNode = doc.getDocumentElement();
      XmlWorldConfigInterpreter interpreter = new XmlWorldConfigInterpreter();
      world = interpreter.initFromXmlElement(rootNode);
    } catch (IOException e) {
      Logz.println("IOException");
      System.exit(-1);
    } catch (SAXParseException e) {
      Logz.println("Parsing error on line " + e.getLineNumber());
      Logz.println(" " + e.getMessage());
      System.exit(-1);
    } catch (SAXException e) {
      Logz.println("SAXException");
      Exception x = e.getException();
      ((x == null) ? e : x).printStackTrace();
      System.exit(-1);
    } catch (ParserConfigurationException e) {
      Logz.println("ParserConfigurationException");
      System.exit(-1);
    }
    Logz.println("Configuration loaded.\n");
    return world;
  }

  public void generateSimTrajs(Collection<Trajectory> trajs) {
    Collection<List<IVector>> traces = new ArrayList<List<IVector>>();
    for (Trajectory myTraj : trajs) {
      List<IVector> trace = new LinkedList<IVector>();
      for (int i = 0; i < myTraj.getPoints().size(); i++) {
        trace.add(myTraj.getPoints().get(i).getV());
      }
      traces.add(trace);
    }
    drawTrajectories(traces);
  }

  public Collection<Trajectory> getTrajs() {
    return trajs;
  }

  public void drawTrajectories(Collection<List<IVector>> traces) {
    try {
      SimPanel panel = SimPanel.makeGui(this.roadmap, "Road Network Trajectories");
      List<IDrawer> drawers = new ArrayList<IDrawer>();
      drawers.add(new RoadMapDrawer(this.roadmap, panel));
      drawers.add(new TrajectoriesDrawer(traces, panel));
      panel.setDrawers(drawers);
      panel.redrawSim();
      panel.repaint();
    } catch (Exception e) {
      Logz.println("No GUI.");
    }
  }

  public void drawBaseClusters() {
    try {
      SimPanel panel = SimPanel.makeGui(this.roadmap, frameCaption);
      List<IDrawer> drawers = new ArrayList<IDrawer>();
      drawers.add(new RoadMapDrawer(this.roadmap, panel));
      drawers.add(new BaseClusterDrawer(this, panel));
      panel.setDrawers(drawers);
      panel.redrawSim();
      panel.repaint();
    } catch (Exception e) {
      System.out.println("No GUI.");
    }
  }

  public void drawFlowClusters() {
    try {
      SimPanel panel = SimPanel.makeGui(this.roadmap, frameCaption);
      List<IDrawer> drawers = new ArrayList<IDrawer>();
      drawers.add(new RoadMapDrawer(this.roadmap, panel));
      drawers.add(new FlowClusterDrawer(this, panel));
      panel.setDrawers(drawers);
      panel.redrawSim();
      panel.repaint();
    } catch (Exception e) {
      System.out.println("No GUI.");
    }

  }

  public void drawNeatClusters() {
    try {
      SimPanel panel = SimPanel.makeGui(this.roadmap, frameCaption);
      List<IDrawer> drawers = new ArrayList<IDrawer>();
      drawers.add(new RoadMapDrawer(this.roadmap, panel));
      drawers.add(new MergedClusterDrawer(this, panel));
      panel.setDrawers(drawers);
      panel.redrawSim();
      panel.repaint();
    } catch (Exception e) {
      System.out.println("No GUI.");
    }

  }

  public void drawTrajectoryClusters(HashMap<Integer, List<Trajectory>> clusters) {
    try {
      SimPanel panel = SimPanel.makeGui(this.roadmap, frameCaption);
      List<IDrawer> drawers = new ArrayList<IDrawer>();
      drawers.add(new RoadMapDrawer(this.roadmap, panel));
      drawers.add(new TrajectoryClusterDrawer(panel, clusters));
      panel.setDrawers(drawers);
      panel.redrawSim();
      panel.repaint();
    } catch (Exception e) {
      System.out.println("No GUI.");
    }
  }

  public long getStartTime() {
    return startTime;
  }

  public RoadMap getRoadmap() {
    return roadmap;
  }

  public void setTrajs(Collection<Trajectory> trajs) {
    this.trajs = trajs;
  }

  public Collection<List<FlowCluster>> getmClus() {
    return mClus;
  }

  public List<FlowCluster> getFlowClusters() {
    return flowClusters;
  }

  public Collection<PointsOnSeg> getTfragments() {
    return tfragments.values();
  }

  public void formBaseClusters() {

    tfragments = new HashMap<Integer, PointsOnSeg>();
    for (Trajectory myTraj : trajs) {
      for (PointsOnSeg pos : myTraj.getRoute()) {
        if (!tfragments.containsKey(pos.getSegid())) {
          tfragments.put(pos.getSegid(), pos);
        } else {

          tfragments.get(pos.getSegid()).add(pos);// add another t-frag to the base cluster
        }
        if (!tfragments.get(pos.getSegid()).getTrajIdList().contains(myTraj.getId())) {
          tfragments.get(pos.getSegid()).getTrajIdList().add(myTraj.getId());
        }
      }

    }

    int objPerBase = 0;
    int objPerBaseMin = Integer.MAX_VALUE;
    int objPerBaseMax = 0;
    for (PointsOnSeg pos : tfragments.values()) {
      int curTrajCount = pos.getTrajIdList().size();
      objPerBase += curTrajCount;
      if (objPerBaseMin > curTrajCount) objPerBaseMin = pos.getTrajIdList().size();
      if (objPerBaseMax < curTrajCount) objPerBaseMax = pos.getTrajIdList().size();
    }

    objPerBase = (int) objPerBase / tfragments.size();
    System.out.println("Number of base clusters: " + this.getTfragments().size());
    System.out.println(String.format("Number of mobile objects per base cluster: avg = %d, min = %d, max = %d \n",
        objPerBase, objPerBaseMin, objPerBaseMax));
    // drawBaseClusters();
  }

  public PointsOnSeg getDensestSeg(Collection<PointsOnSeg> posList) {
    int startSeg = 0;
    int maxDense = 0;
    for (PointsOnSeg pos : posList) {
      /*
       * if (pos.numTrajs()>maxDense){ startSeg = pos.getSegid(); maxDense = pos.numTrajs(); }
       */
      if (pos.numPoints() > maxDense) {
        startSeg = pos.getSegid();
        maxDense = pos.numPoints();
      }
      /*
       * if (Math.round(pos.numPoints()/pos.getRoadSeg(roadmap). getLength())>maxDense){ startSeg = pos.getSegid();
       * maxDense = Math.round(pos.numPoints()/pos.getRoadSeg(roadmap). getLength()); }
       */

    }
    return tfragments.get(startSeg);
  }

  public void computeFlows() {
    while (!this.tfragments.isEmpty()) {
      mergeTfragments(-1, -1, -1, -1);// flow-based clustering
    }

    System.out.println("Number of flow clusters before filtering: " + this.flowClusters.size());
    int avgTrajCount = 0;
    int minTrajCount = Integer.MAX_VALUE;
    int maxTrajCount = 0;
    for (int i = this.flowClusters.size() - 1; i >= 0; i--) {
      flowClusters.get(i).generateTrajIds();
      int curTrajCount = flowClusters.get(i).getTrajCount();
      avgTrajCount += curTrajCount;
      if (minTrajCount > curTrajCount) minTrajCount = curTrajCount;
      if (maxTrajCount < curTrajCount) maxTrajCount = curTrajCount;
    }

    avgTrajCount = (int) avgTrajCount / this.flowClusters.size();

    System.out.println(String.format("Number of mobile objects per flow cluster: avg = %d, min = %d, max = %d",
        avgTrajCount, minTrajCount, maxTrajCount));

    // Filter clusters which has too few objects moving on, use avgTrajCount instead of trajCountThres
    for (int i = this.flowClusters.size() - 1; i >= 0; i--) {
      if (flowClusters.get(i).getTrajCount() <= avgTrajCount) flowClusters.remove(i);
    }
    System.out.println("\nNumber of flow clusters after filtering: " + this.flowClusters.size());
    // sort flow clusters by length in ascending order
    Collections.sort(this.flowClusters, new FlowClusterComparator(roadmap));
  }

  public void mergeTfragments(double wq, double wk, double wv, int khops) {
    // start with the densest segment which has the maximum number of trajectories

    FlowCluster flow = new FlowCluster();
    PointsOnSeg ps0 = getDensestSeg(getTfragments());
    flow.addSeg(ps0);

    flow.setStartJunc(ps0.getRoadSeg(roadmap).getSourceJunction());
    flow.setEndJunc(ps0.getRoadSeg(roadmap).getTargetJunction());
    tfragments.remove(ps0.getSegid());

    // when using adaptive weights, assign wq=wk=wv=-1
    while (flow.isExtensible()) {
      // khops: 0=lookback 1=look ahead, -1=randomly choose
      PointsOnSeg[] ps = flow.concatenateJuncs(roadmap, tfragments.values(), wq, wk, wv, khops);
      this.concateCount++;
      if (ps[0] != null) tfragments.remove(ps[0].getSegid());
      if (ps[1] != null) tfragments.remove(ps[1].getSegid());
    }
    flowClusters.add(flow);
  }

  public void showFlowStats() {
    double stat[] = { 0, 0, 0 };
    int count = 0;
    for (int i = 0; i < flowClusters.size(); i++) {
      double[] curStat = flowClusters.get(i).getStreamStats(roadmap);
      for (int j = 0; j < 3; j++) {
        stat[j] += curStat[j];
      }
      count += flowClusters.get(i).getSize();
    }
    for (int j = 0; j < 3; j++) {
      if (j == 0) {
        stat[j] = stat[j] / (count - flowClusters.size());
        break;
      }
      stat[j] = stat[j] / count;
    }
    // System.out.println(String.format("Flow stats: avgNetflow = %d, avgDensity = %d, avgSpeed = %d(m/s)",
    // (long) stat[0], (long) stat[1], (long) (stat[2] / 1000)));
    // statistics of representative length
    // double flowCompute = 0;
    double avgLen = 0;
    int nFlows = this.flowClusters.size();
    for (int i = 0; i < this.flowClusters.size(); i++) {
      // flowCompute += this.flowClusters.get(i).getFlowComputeCount();
      avgLen += this.flowClusters.get(i).getLength(roadmap);
    }
    avgLen = avgLen / nFlows;
    double minLen = this.flowClusters.get(0).getLength(roadmap);
    double maxLen = this.flowClusters.get(nFlows - 1).getLength(roadmap);
    System.out.println(String.format("Length of flow representatives (m): avg = %.3f, min = %.3f, max = %.3f",
        avgLen / 1000.0, minLen / 1000.0, maxLen / 1000.0));
  }

  public void optClusteringResult(double optEps) {
    if (optEps < 0) {
      for (int i = 0; i < this.flowClusters.size(); i++) {
        List<FlowCluster> newClus = new ArrayList<FlowCluster>();
        newClus.add(flowClusters.get(i));
        this.mClus.add(newClus);
      }
      System.out.println("\nNo further merging.");
    } else {
      // Density-based merging
      System.out.print("\nDensity-based merging flow clusters...");
      this.mClus = new MergedClusters(this.flowClusters, roadmap, optEps).getClusters().values();
      System.out.println("Done");
    }

    System.out.println("\nFinal number of clusters: " + this.mClus.size());
  }

  // return the routes in a group after the density-based opt
  public Collection<List<IVector>> representativeRoutes(List<FlowCluster> segClus, RoadMap rm) {
    Collection<List<IVector>> repRoutes = new LinkedList<List<IVector>>();
    for (int i = 0; i < segClus.size(); i++) {

      repRoutes.add(segClus.get(i).representativePoints(rm));
    }
    return repRoutes;
  }

  public List<LinkedList<PointsOnSeg>> getDbscanClusters(double eps) {
    DBSCANClustering dbs = new DBSCANClustering(eps, roadmap, tfragments.values());
    dbs.dbscanClusterSegments();
    return dbs.getClusters();
  }

  public HashMap<Integer, List<Integer>> toIntList(HashMap<Integer, List<Trajectory>> clusters) {
    HashMap<Integer, List<Integer>> l = new HashMap<Integer, List<Integer>>();
    for (int clusId : clusters.keySet()) {
      List<Integer> li = new ArrayList<Integer>();
      for (Trajectory t : clusters.get(clusId)) {
        li.add(t.getId());
      }
      l.put(clusId, li);
    }
    return l;
  }

  public HashMap<Integer, Trajectory> toHashMapData(Collection<Trajectory> trajs) {
    HashMap<Integer, Trajectory> dataSet = new HashMap<Integer, Trajectory>();
    for (Trajectory t : trajs) {
      dataSet.put(t.getId(), t);
    }
    return dataSet;
  }

  /*
   * param clusters: hashmap(clusterId -> list of trajectory Ids in the cluster) toTrajectoryList creates a hashmap
   * clusterId -> list of trajectories in the cluster it also sets the clusId for each trajectory
   */
  public HashMap<Integer, List<Trajectory>> toTrajectoryList(HashMap<Integer, List<Integer>> clusters) {
    HashMap<Integer, List<Trajectory>> tl = new HashMap<Integer, List<Trajectory>>();
    HashMap<Integer, Trajectory> dataSet = toHashMapData(this.trajs);
    for (int clusId : clusters.keySet()) {
      List<Trajectory> trajList = new LinkedList<Trajectory>();
      for (int trajId : clusters.get(clusId)) {
        dataSet.get(trajId).setClusId(clusId);
        trajList.add(dataSet.get(trajId));
      }
      tl.put(clusId, trajList);
    }
    return tl;
  }

  public void saveSegmentSequences(String outputFilePath) throws IOException {
    BufferedWriter output = new BufferedWriter(new FileWriter(new File(outputFilePath)));
    for (Trajectory t : this.trajs) {
      output.write(t.printSegmentIDSequence());
      output.newLine();
      System.out.print(outputFilePath);
    }
    output.close();
  }

  public void runNeat() {
    // 1. Base cluster formation
    formBaseClusters();

    // 2. Flow-based clustering
    computeFlows();
    showFlowStats();
    drawFlowClusters();

    // 3. Final phase
    // optClusteringResult(450000); // eps: distance threshold (mm)
    optClusteringResult(-1);// eps < 0 : no further merging 

    drawNeatClusters();
  }

  public static void main(String[] args) throws IOException {
    NeatClustering rc = new NeatClustering(Const.configFile);
    System.out.println(String.format("Running NEAT clustering on %d trajectories...", rc.getTrajs().size()));
    rc.runNeat();
  }
}
