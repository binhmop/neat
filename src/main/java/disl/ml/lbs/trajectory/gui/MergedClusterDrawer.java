package disl.ml.lbs.trajectory.gui;


import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import disl.ml.lbs.trajectory.FlowCluster;
import disl.ml.lbs.trajectory.MoPoint;
import disl.ml.lbs.trajectory.Trajectory;
import disl.ml.lbs.trajectory.clustering.NeatClustering;
import edu.gatech.lbs.core.vector.IVector;
import edu.gatech.lbs.sim.gui.SimPanel;
import edu.gatech.lbs.sim.gui.drawer.IDrawer;
import edu.gatech.lbs.sim.gui.drawer.TrajectoryDrawer;

public class MergedClusterDrawer implements IDrawer {

  private NeatClustering neat;
  private SimPanel panel;


  public MergedClusterDrawer(NeatClustering nc, SimPanel panel) {
    this.neat = nc;
    this.panel = panel;
  }

  public void draw(Graphics g) {
    Random numGen = new Random();

    ArrayList<Color> colorSet = new ArrayList<Color>();
    colorSet.add(Color.gray);
    colorSet.add(Color.DARK_GRAY);
    colorSet.add(Color.LIGHT_GRAY);
    // g.setColor(Color.green);
    drawInputPoints(g);
    Collection<List<FlowCluster>> mClusters = this.neat.getmClus();
    for (List<FlowCluster> mClus : mClusters) {

      do {
        g.setColor(TrajectoryDrawer.getRandomColor(numGen));
      } while (colorSet.contains(g.getColor())); // make sure new color is different from the already used ones
      colorSet.add(g.getColor()); // add each selected color to the end of the list
      Collection<List<IVector>> trajs = neat.representativeRoutes(mClus, neat.getRoadmap());
      for (List<IVector> traj : trajs) {

        IDrawer trajDrawer = new TrajectoryDrawer(panel, traj, colorSet.get(colorSet.size() - 1));
        trajDrawer.draw(g);
      }
    }
  }

  public void drawInputPoints(Graphics g) {
    g.setColor(Color.DARK_GRAY);
    Collection<Trajectory> trajs = this.neat.getTrajs();
    for (Trajectory myTraj : trajs) {
      LinkedList<MoPoint> moPointLs = myTraj.getPoints();
      for (int i = 0; i < moPointLs.size() - 1; i++) {
        Point p0 = panel.getPixel(moPointLs.get(i).getV());
        g.fillOval(p0.x - 1, p0.y - 1, 3, 3);
      }
    }
  }

}
