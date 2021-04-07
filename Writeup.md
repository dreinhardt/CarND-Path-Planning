# CarND-Path-Planning-Project
Dominik's Writeup

My comment: I add to that writeup, that my code includes a lot of comments. It describes each step in detail. Furthermore, I closely stick to the project Q&A section. This chapter was very helpful!


## Rquirements for Passing

### Compilation
Requirement: The code compiles correctly.
My comment: I use cmake and make!

### Valid Trajectories

#### The car is able to drive at least 4.32 miles without incident
Requirement: The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.
My comment: As I presented in my demo, the car can drive completely alone and without any accidents.

#### The car drives according to the speed limit
Requirement: The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.
My comment: Because the speed in the code is limitted, it is not possible to drive faster than 50mph.

#### Max Acceleration and Jerk are not Exceeded
Requirement: The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
My comment: I use the following lines to harm the acceleration and breaking of the car in 5 meter per second steps.
if(too_close == true){
  ref_vel -= 0.224;   // ca 5m per second, its under the 10 mps requirement for the jerk algorithm
}
else if (ref_vel < 49.5){
  ref_vel += 0.224;   // if its 'not to close' we constantly add 5 mps to our velocity 
}

#### Car does not have collisions
Requirement: The car must not come into contact with any of the other cars on the road.
My comment: As you can see in the demo, the car will never hit others. In case there is no chouce to switch lanes, it stays within its lane and reduce speed.

#### The car stays in its lane, except for the time between changing lanes
Requirement: The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.
My comment: The lane change is very fast and as soon I set the switch to change the lane, the period of time is within 3 seconds boundary. Lane changes are only possible in case the other lane(s) is free.

#### The car is able to change lanes
Requirement: The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.
My comment: as you can see, the car changes the lane smoothly. In case there is a car coming from behind, it will not move to that lane. In case the lane is free, it will move to that free lane and is doing a change maneuver 

### Reflection
Requirement: This is y reflection on how to generate paths.
My comment: 
The code is well documented and even the tutorial code used from project Q&A is well documented.
Here, I will describe the way how to detect other vehicles and lead through the traffic.

The center of knowledge starts at section ******** START MY CODE ********  (line 108)
Here, I overtake the values from the previous step. 
Afterwards I create checker values, if other cars are infront of our car and  which lanes that cars block possible change maneuvers.
Within the following for loop I check all cars on the map detected from my sensors.
I check in which lanes that cars are located and check how far that car is away from our car.
In case the car is directly infront of us, that step is marked with a boolean called 'too_close'.
After that step, i check for each lane, if there are other cars within that lane which could block a possible change maneuver. If a car is within a specific range infront of us or another fast approching car is backside of us, that lane is marked as non-changable possibility.
After the for cycle ends, I print that lane evaluation. 
In case the actual lane is blocked by a slower car and another lane directly beside of it seems to be free, the car will change to that lane immediately. A requirement is, that the lane number is valid and not out of street or in the opposite lane.