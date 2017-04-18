# NEAT - road NEtwork Aware Trajectory mining
---

Implementation and data for papers:

* [NEAT: Road Network Aware Trajectory Clustering](http://ieeexplore.ieee.org/document/6257987/)

* [Road-Network Aware Trajectory Clustering: Integrating Locality, Flow, and Density](http://ieeexplore.ieee.org/document/6589570/)

### Build

```
$git clone https://github.com/binhmop/neat.git 

$cd neat

$mvn clean compile package
```
This will create an executable jar at `target/neat-1.0-SNAPSHOT-jar-with-dependencies.jar`

### Import to Eclipse
`Eclipse -> File -> Import -> Existing Maven Projects`

### Run jar

```
$java  -DconfigFile=configs/mapconfig.xml -DtraceFile=configs/traces/sj-1000.txt -jar target/neat-1.0-SNAPSHOT-jar-with-dependencies.jar
```
### Disclaimer
This software is published for academic and non-commercial use only.
