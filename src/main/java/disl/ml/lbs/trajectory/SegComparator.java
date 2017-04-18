package disl.ml.lbs.trajectory;


import java.util.Comparator;

public class SegComparator implements Comparator<PointsOnSeg> {
  public SegComparator() {
    super();
  }

  public int compare(PointsOnSeg ps0, PointsOnSeg ps1) {
    if (ps0.getV().size() < ps1.getV().size()) return -1;
    else if (ps0.getV().size() < ps1.getV().size()) return 1;
    else return 0;
  }

}
