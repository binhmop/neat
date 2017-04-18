package disl.ml.lbs.trajectory.gui;


import java.awt.Graphics;
import java.util.ArrayList;
import java.util.Random;
import disl.ml.lbs.trajectory.PointsOnSeg;
import disl.ml.lbs.trajectory.clustering.NeatClustering;
import edu.gatech.lbs.core.world.roadnet.RoadSegment;
import edu.gatech.lbs.sim.gui.SimPanel;
import edu.gatech.lbs.sim.gui.drawer.IDrawer;
import edu.gatech.lbs.sim.gui.drawer.SegmentDrawer;
import edu.gatech.lbs.sim.gui.drawer.TrajectoryDrawer;

public class BaseClusterDrawer implements IDrawer {

  private NeatClustering neat;
  private SimPanel panel;


  public BaseClusterDrawer(NeatClustering nc, SimPanel panel) {
    this.neat = nc;
    this.panel = panel;
  }

  public void draw(Graphics g) {
    Random numGen = new Random();
    ArrayList<PointsOnSeg> segList = new ArrayList<PointsOnSeg>(neat.getTfragments());
    
    for (PointsOnSeg pos : segList) {
      g.setColor(TrajectoryDrawer.getRandomColor(numGen));
      RoadSegment seg = pos.getRoadSeg(neat.getRoadmap());
      // List<IVector> traj = pos.getV();
      // IDrawer trajDrawer = new TrajectoryDrawer(panel,traj,g.getColor());
      // trajDrawer.draw(g);
      IDrawer segDrawer = new SegmentDrawer(panel, seg, g.getColor());
      segDrawer.draw(g);
    }

  }

}
