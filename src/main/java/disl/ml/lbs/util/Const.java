package disl.ml.lbs.util;


import com.google.common.base.Strings;

public class Const {

  // change the roadmap in the config file to use the correct map associated with traceFile
  public static String configFile = (!Strings.isNullOrEmpty(System.getProperty("configFile"))) ? System
      .getProperty("configFile") : "configs/mapconfig-svg.xml";

  public static String traceFile = (!Strings.isNullOrEmpty(System.getProperty("traceFile"))) ? System
      .getProperty("traceFile") : "configs/traces/sj-1000.txt";

  public static String outputClusteringResult = (!Strings.isNullOrEmpty(System.getProperty("outputClusteringResult"))) ? System
      .getProperty("outputClusteringResult") : "configs/output/neat-clusters.txt";
}
