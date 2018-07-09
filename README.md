The purpose of the present diploma thesis is the algorithm development
for the scheduling Unmanned Aerial Vehicles (drone) in order to deliver
packets in destinations. More precisely, at first the road network of the
area is simulated and a graph is created, where the drone carrier (which
transports the packets and the drone) is moving. The drone carrier parks
at a node which is close to the packet destination and from there the
drone begins the journey for the packet delivery. When all the packets
have been delivered, information has been gathered for drone energy
consumption, drone flight time and packets throughput. These elements
will be displayed graphically in order to study the performance of the
algorithms.
The program waswritten mainly in programming language C with some data 
structures from C++.



Compile:
g++ -Wall -std=c++0x droneAlgorithms.cpp -o droneAlgorithms

Run:
./droneAlgorithms input.txt


