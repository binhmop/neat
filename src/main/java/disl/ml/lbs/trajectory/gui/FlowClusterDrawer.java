package disl.ml.lbs.trajectory.gui;


import java.awt.Graphics;
import java.util.List;
import java.util.Random;
import disl.ml.lbs.trajectory.FlowCluster;
import disl.ml.lbs.trajectory.PointsOnSeg;
import disl.ml.lbs.trajectory.clustering.NeatClustering;
import edu.gatech.lbs.core.world.roadnet.RoadSegment;
import edu.gatech.lbs.sim.gui.SimPanel;
import edu.gatech.lbs.sim.gui.drawer.IDrawer;
import edu.gatech.lbs.sim.gui.drawer.SegmentDrawer;
import edu.gatech.lbs.sim.gui.drawer.TrajectoryDrawer;


public class FlowClusterDrawer implements IDrawer {
  private NeatClustering neat;
  private SimPanel panel;


  public FlowClusterDrawer(NeatClustering nc, SimPanel panel) {
    this.neat = nc;
    this.panel = panel;
  }

  public void draw(Graphics g) {
    Random numGen = new Random();
    List<FlowCluster> flows = neat.getFlowClusters();

    for (FlowCluster fc : flows) {
      g.setColor(TrajectoryDrawer.getRandomColor(numGen));
      for (PointsOnSeg pos : fc.getFlowClus()) {
        RoadSegment seg = pos.getRoadSeg(neat.getRoadmap());
        // List<IVector> traj = pos.getV();
        // IDrawer trajDrawer = new TrajectoryDrawer(panel,traj,g.getColor());
        // trajDrawer.draw(g);
        IDrawer segDrawer = new SegmentDrawer(panel, seg, g.getColor());
        segDrawer.draw(g);
      }
    }
  }

}
