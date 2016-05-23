# Autonomous Rover

This project hosts an autonomous rover simulator and its corresponding implementation on Arduino.

## Installation

Pre-requisites:

* A JVM, such as openjdk
* Java3D libraries
* Arduino and rover hardware (details of hardware to follow)

On Debian/Ubuntu Linux, as root:

```
apt-get install openjdk-8-jdk
apt-get install libjava3d-java

cp /usr/share/java/j3d* /usr/lib/jvm/default-java/jre/lib/ext/
cp /usr/share/java/vecmath* /usr/lib/jvm/default-java/jre/lib/ext/
```

## References

* [Simbad virtual robot simulator](http://simbad.sourceforge.net/)