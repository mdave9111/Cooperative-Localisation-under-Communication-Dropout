# Cooperative Localisation under Communication Dropout

This is a Python simulation assessing how multi agent localization accuracy degrades under lossy communication links. I am developing this as a benchmark for distributed swarm estimation and research proposals.

## Overview

In multi-agent swarm systems, like autonomous marine vehicles or ground rovers, individual agents within the swarm experience odometric drift over time due to sensor noise and corresponding position calculations. GNSS can correct this drift, but limited power, operational constraints and GNSS spoofing/jamming limit it. Only a small subset of agents ('anchors') can possess accurate coordinates in the simulation.

Unanchored agents rely on **cooperative localisation**, measuring range to neighbouring agents and exchanging state estimation packets. However, wireless channels suffer from packet drops, fading and interference.

This project plots the relationship between **packet drop probability (_p_)** against **position estimation error (RMSE)** so baseline state estimation strategies can be compared against more complex techniques.





