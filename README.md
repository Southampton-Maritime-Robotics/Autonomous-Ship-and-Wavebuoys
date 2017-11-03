# The Development of an Autonomous Self-Propulsion Vessel (ASV) for Powering and Manoeuvring Tests in an Uncontrolled Environment

This repository is a collection of code and documentation from Group Design Project #40, tasked with converting a 1:60 scale model of a tanker into an autonomous self-propulsion vessel which could self-monitor its performance (seakeeping, manoeuvring and powering). The project is a part of the Master's Degree in Ship Science from the University of Southampton (2013).

A key part of the project was setting up ROS on a linux processor and several complimenting Arduinos to monitor sensors and control hardware. The code for ROS (./ASV) and the Arduinos (./Arduinos) can be found in this repository.

Another part of this project was developing a means to measure the sea state that the vessel was operating in, using a number of wavebuoys. The wavebuoys were kitted out with Lithium batteries, long-range radio communications, SD cards, 9DOF sensors, GPS and bluetooth; and relayed live information back to a shore computer which performed complex analysis via a GUI application (WABDAP). The code for the wavebuoy Arduinos and WABDAP can also be found in this repository (./WaveBuoy).

There were many non-coding tasks in the project, including development of the power train and steering mechanism and accurate model-scaling of the real-life vessel, which can also be read about in the main report.

## Contributors

The main software contributions were from James Hawkes (Arduinos and Wavebuoy), Enrico Anderlini (ROS), and Hieu Le (Rudder Arduinos); heavily assisted by Heather Crossley, Kimberley Neale, James Mozden and Ben Thornton. The supervisory team were Stephen Turnock, Dominic Hudson and Alexander Phillips.

## License

With permission from the copyright holders (Hawkes, Anderlini, Le), the code has been released under the permissive MIT license

## Citation

```
@MastersThesis{ASV,
  author =       Enrico Anderlini and Heather Crossley and James Hawkes and Hieu Le and James Mozden and Kimberley Neale and Ben Thornton,
  title =        "The Development of an Autonomous Self-Propulsion Vessel for Powering and Manoeuvring Tests in an Uncontrolled Environment",
  school =       University of Southampton,
  year =         2013,
  month =        apr,
  type =         "Undergraduate Masters Thesis",
}
```