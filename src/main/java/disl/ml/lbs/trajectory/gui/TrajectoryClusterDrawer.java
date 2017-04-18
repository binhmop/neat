package disl.ml.lbs.trajectory.gui;


import java.awt.Graphics;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import disl.ml.lbs.trajectory.Trajectory;
import edu.gatech.lbs.core.vector.IVector;
import edu.gatech.lbs.sim.gui.SimPanel;
import edu.gatech.lbs.sim.gui.drawer.IDrawer;
import edu.gatech.lbs.sim.gui.drawer.TrajectoryDrawer;

public class TrajectoryClusterDrawer implements IDrawer {
  private SimPanel panel;
  private HashMap<Integer, List<Trajectory>> clusters;


  public TrajectoryClusterDrawer(SimPanel panel, HashMap<Integer, List<Trajectory>> clusters) {
    this.panel = panel;
    this.clusters = clusters;
  }

  public void draw(Graphics g) {
    // g.setColor(Color.green);
    int id = -1;
    Random numGen = new Random();

    for (List<Trajectory> clus : clusters.values()) {
      id++;
      if (id != 4) continue;
      g.setColor(TrajectoryDrawer.getRandomColor(numGen));
      // g.setColor(Color.blue);
      for (Trajectory curTraj : clus) {
        List<IVector> traj = curTraj.toIVectors();
        IDrawer trajDrawer = new TrajectoryDrawer(panel, traj, g.getColor());
        trajDrawer.draw(g);
      }
    }

  }

}
