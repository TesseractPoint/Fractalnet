# Fractalnet
### Solution for 2016 Space Apps Challenge
Fractalnet is a design for a network of low power radio relay devices that can be placed in environments that do not allow long range propagation of RF signals, such as caves, caverns, etc.  The specific use case involves deployment on the moon, Mars, asteroids and other planets during exploration by astronauts and robots.

## Network Description
The Fractalnet network consists primarily of two classes of device: a series of infrastructure devices that relay message packets through the network, and some number of application devices that source and sink the data that is transported by the infrastructure devices.  The infrastructure devices are placed at suitable intervals along a cave wall or tunnel to ensure continuous connectivity between each node and from end to end.  Topologically, the network consists of a number of trees, with a single device (the base station) at the root.  Relative to the base station, there is a defined direction to the flow of traffic, which is either away from the base station (upstream) or towards it (downstream).  In general, all traffic on the network arrives at the base station, where it is logged and made available to other devices.

The base station may also have additional applications running on it, allowing it to serve as a gateway to other networks available in the exploration region, e.g., providing connections to remote operations centers, habitats, spacecraft, etc.

## Device Description
Application devices may be mobile or stationary.  An astronaut who is exploring a tunnel, and moving through it, will make connection to the nearest infrastructure node, handing off the connection to each succeeding node in turn as he progresses into and out of the cave.  An instrument that is being transported into the tunnel to an intended monitoring location, will perform a similar series of hand offs while it is being carried, but once it has been placed at its final destination, it will remain connected to that nearest node.

## Installation
During the first ingress sortie in a cave system, an astronaut or robot team will carry a supply of inactive infrastructure devices.  An application that is running on the explorer's personal network manages the process of installing each new infrastructure node.  The device is activated, registered with the network, and placed on the wall by means of a mounting system.  Finally, the commissioning process is completed by putting the device into normal service mode.

## Usage
The fractalnet network is mostly transparent to users, who only need to be aware of the applications and services that they are using across the network.  The applications themselves, however, must know how to present data to the network, and how to request data from available sources.  The details of this process were not part of the Space Apps Challenge solution at competition time.

Further details of a fully working Fractalnet V.2 can be found elsewhere.


## Challenge Participants
1. Andrew Denio
2. Joe Greene
3. Tony Vaughn

## Official Contact
Email:  tony.vaughn@tesseractpoint.com

## History
TODO: Write history
## Credits
TODO: Write credits
## License
TODO: Write license

