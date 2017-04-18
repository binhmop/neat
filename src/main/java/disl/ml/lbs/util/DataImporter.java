package disl.ml.lbs.util;


import java.io.*;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.Collection;
import java.util.StringTokenizer;
import java.util.List;
import java.util.HashMap;
import disl.ml.lbs.trajectory.MoPoint;
import disl.ml.lbs.trajectory.Trajectory;
import edu.gatech.lbs.core.vector.IVector;
import edu.gatech.lbs.core.vector.CartesianVector;

public class DataImporter {
  protected HashMap<Integer, List<Integer>> trajIdClus;// list of trajId in each cluster
  protected HashMap<Integer, List<Integer>> segIdClus;// list of segId in each cluster


  public DataImporter() {
    trajIdClus = new HashMap<Integer, List<Integer>>();
    segIdClus = new HashMap<Integer, List<Integer>>();
  }

  public HashMap<Integer, List<Integer>> getTrajIdClus() {
    return trajIdClus;
  }

  public HashMap<Integer, List<Integer>> getSegIdClus() {
    return segIdClus;
  }

  public Collection<Trajectory> loadTrajectories(String pathToFile) {
    HashMap<Integer, Trajectory> trajectories = new HashMap<Integer, Trajectory>();
    File testData = new File(pathToFile);
    BufferedReader reader = null;
    try {
      reader = new BufferedReader(new FileReader(testData));
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    String line = null;
    StringTokenizer tk = null;

    try {
      while ((line = reader.readLine()) != null) {
        tk = new StringTokenizer(line, " ");
        int trajId = Integer.parseInt(tk.nextToken());
        int segId = Integer.parseInt(tk.nextToken());
        double x = Double.parseDouble(tk.nextToken());
        double y = Double.parseDouble(tk.nextToken());
        // CartesianVector v = new CartesianVector(x,y);
        // ATTN: convert to long only when coordinates in double
        MoPoint p = new MoPoint(segId, (long) x * 1000, (long) y * 1000);

        if (!trajectories.containsKey(trajId)) {

          Trajectory traj = new Trajectory(trajId);
          trajectories.put(trajId, traj);
        }
        trajectories.get(trajId).getPoints().add(p);
      }

    } catch (Exception e) {
      e.printStackTrace();
    }
    try {
      if (reader != null) {
        reader.close();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }

    Collection<Trajectory> trajs = trajectories.values();
    return trajs;

  }

  // to read Traclus results with segId info
  public Collection<List<IVector>> loadTraClusRes(String pathToFile) {
    HashMap<Integer, List<IVector>> traClusters = new HashMap<Integer, List<IVector>>();
    LinkedList<IVector> clus = new LinkedList<IVector>();
    ArrayList<Integer> trajIds = new ArrayList<Integer>();// list of trajId for one cluster
    ArrayList<Integer> segIds = new ArrayList<Integer>();// list of segId for one cluster


    File testData = new File(pathToFile);
    BufferedReader reader = null;
    try {
      reader = new BufferedReader(new FileReader(testData));
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    String line = null;
    int clusterId = 0;

    try {
      reader.readLine();
      reader.readLine();

      while ((line = reader.readLine()) != null) {
        String[] st = line.split(" ");

        if (st[0].equals("cluster")) {

          reader.readLine();// empty line
          traClusters.put(clusterId, (List<IVector>) clus.clone());
          this.trajIdClus.put(clusterId, (List<Integer>) trajIds.clone());
          this.segIdClus.put(clusterId, (List<Integer>) segIds.clone());
          clusterId++;
          clus.clear();
          trajIds.clear();
          segIds.clear();

        } else {
          trajIds.add(Integer.parseInt(st[0]));
          // traClusters.put(clusterId,clus);
          for (int i = 1; i < st.length - 1; i++) {
            if ((i % 3) != 1) continue;
            if (!segIds.contains(Integer.parseInt(st[i]))) segIds.add(Integer.parseInt(st[i]));
            double x = Double.parseDouble(st[i + 1]);
            double y = Double.parseDouble(st[i + 2]);
            // CartesianVector v = new CartesianVector(x,y);
            CartesianVector v = new CartesianVector((long) (x * 1000), (long) y * 1000);
            // Point p = new Point(Integer.parseInt(st[i]),v);
            // i++;
            clus.add(v);

          }

        }
        traClusters.put(clusterId, clus);
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
    try {
      if (reader != null) {
        reader.close();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
    return traClusters.values();

  }

  public static HashMap<Integer, List<Integer>> loadWekaKmeansResult(String pathToFile) {
    // format .arff (weka kmeans results)
    HashMap<Integer, List<Integer>> clusIdToTrajIds = new HashMap<Integer, List<Integer>>();
    File testData = new File(pathToFile);
    BufferedReader br = null;
    try {
      br = new BufferedReader(new FileReader(testData));
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    String line = null;
    // int clusterId=0;

    try {
      do {
        line = br.readLine();
      } while (line.indexOf("@data") == -1);

      while ((line = br.readLine()) != null) {

        String[] st = line.split(",");
        int clusterId = Integer.parseInt(st[st.length - 1].substring(7));
        if (!clusIdToTrajIds.containsKey(clusterId)) {
          List<Integer> trajIds = new LinkedList<Integer>();
          trajIds.add(Integer.parseInt(st[st.length - 2]));
          clusIdToTrajIds.put(clusterId, trajIds);
        } else {
          clusIdToTrajIds.get(clusterId).add(Integer.parseInt(st[st.length - 2]));
        }
      }

    } catch (Exception e) {
      e.printStackTrace();
    }
    try {
      if (br != null) {
        br.close();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
    return clusIdToTrajIds;

  }

  public static void saveToTxt(String filename, HashMap<Integer, List<Trajectory>> clusters) throws IOException {
    BufferedWriter out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(new File(filename))));
    try {
      for (Integer clus : clusters.keySet()) {
        out.write(clus + " ");
        for (Trajectory t : clusters.get(clus)) {
          out.write(t.getId() + " ");

        }
        out.newLine();
      }
      out.close();
    } catch (IOException e) {
      System.err.println("Error: " + e.getMessage());
    }
  }

  public static HashMap<Integer, List<Integer>> loadFromTxt(String filename) {
    HashMap<Integer, List<Integer>> clusters = new HashMap<Integer, List<Integer>>();
    BufferedReader in = null;
    try {
      in = new BufferedReader(new InputStreamReader(new FileInputStream(new File(filename))));

    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
    try {
      String line = null;
      while ((line = in.readLine()) != null) {
        String st[] = line.split(" ");
        int clusId = Integer.parseInt(st[0]);
        List<Integer> traIds = new LinkedList<Integer>();
        for (int i = 1; i < st.length; i++) {
          traIds.add(Integer.parseInt(st[i]));
        }
        clusters.put(clusId, traIds);
      }
      in.close();
    } catch (Exception e) {
      e.printStackTrace();
    }
    return clusters;
  }

  public Collection<List<IVector>> loadTraClusResults(String pathToFile) {
    HashMap<Integer, List<IVector>> traClusters = new HashMap<Integer, List<IVector>>();
    LinkedList<IVector> clus = new LinkedList<IVector>();
    ArrayList<Integer> trajIds = new ArrayList<Integer>();

    File testData = new File(pathToFile);
    BufferedReader reader = null;
    try {
      reader = new BufferedReader(new FileReader(testData));
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    String line = null;
    int clusterId = 0;

    try {
      reader.readLine();
      reader.readLine();

      while ((line = reader.readLine()) != null) {
        String[] st = line.split(" ");

        if (st[0].equals("cluster")) {

          reader.readLine();// empty line
          traClusters.put(clusterId, (List<IVector>) clus.clone());
          this.trajIdClus.put(clusterId, (List<Integer>) trajIds.clone());
          clusterId++;
          clus.clear();
          trajIds.clear();

        } else {
          trajIds.add(Integer.parseInt(st[0]));
          // traClusters.put(clusterId,clus);
          for (int i = 1; i < st.length - 1; i++) {
            if (i % 2 == 0) continue;
            double x = Double.parseDouble(st[i]);
            double y = Double.parseDouble(st[i + 1]);
            CartesianVector v = new CartesianVector((long) (x * 1000), (long) y * 1000);
            // i++;
            clus.add(v);

          }

        }
        traClusters.put(clusterId, clus);
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
    try {
      if (reader != null) {
        reader.close();
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
    return traClusters.values();

  }

}
