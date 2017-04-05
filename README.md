SO_DATA MANUAL
===============

The package so_data provides components which are required to implement self-organization. 
It provides functionality to send data to the `so_data` topic as well as to aggregate, store and use it for calculations. 
Therewith, it provides the basic mechanisms and the gradient patterns specified by in "Description and composition of bio-inspired design patterns: a complete overview" by Fernandez-Marquez et al. (2013). 
Furthermore, it includes the implementation of the decision and movement patterns. 
Unit tests were done for the core components to ensure correctness of mathematical computations. 
All methods work for up to three dimensions; the code can be adapted for use in more dimensions with minor enhancements. 
The package can be used in combination with the RHBP, but as well on its own. 


Package components
-------------------

The package consists of the following components:

src: 

* **calc.py**: provides helper functions which are used by several components of the package (+ unit test)
* **chemotaxis.py**: module containing chemotaxis pattern implementations (+ unit test)
* **decisions.py**: module containing sample implementations of decision patterns (morphogenesis, gossip, quorum sensing) (+ unit test)
* **flocking.py**: provides methods to calculate the flocking vector based on the paper by Olfati-Saber + mechanism implementation (+ unit test) 
* **flockingrey.py**: provides methods to calculate a flocking vector based on the formulas by Reynolds (+ unit test)
* **foraging.py**: provides methods to calculate flocking based on the formulas by Olfati-Saber (+ unit test)
* **gradient.py**: module including gradient calculations based on Balch and Hybinette as well as Ge and Cui (+ unit test)
* **gradientnode.py**: allows to create nodes which spread artificial gradients 
* **patterns.py**: provides abstract classes for movement and decision pattern implementations
* **posegradienttf.py**: implementation of `topicGradientTf` for the pose topic (geometry msgs pose)
* **posestampedgradienttf.py**: implementation of `topicGradientTf` for the pose topic (geometry msgs stamped pose)
* **repulsion.py**: module including repulsion based on gradients and formula by Fernandez-Marquez (+ unit test)
* **sobroadcaster.py**: allows to publish data to the so_data topic which will be subscribed to in the soBuffer 
* **sobuffer.py**: implements a layer which provides the basic functionality patterns for gradient data: evaporation, spreading (receives gradients), aggregation (+ unit test)  
* **topicgradienttf.py**: abstract class which can be used as a blueprint to transform topics in soMessages and publish them to the `so_data` topic 
* **supplements.py**: module including supplementary mechanisms 

msg:

* **SoMessage.msg**: message file describing gradient data 

srv:
* **EnvGradient.srv**: Service to hand over a list of gradients to be spread by gradientnode

The robot pose is considered to be in the form of a [`geometry_msgs/Pose`](docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html) Message. 


SoMessage
---------

SoMessages are used to specify gradients: either in environment or as robot data. 
The specification of the SoMessage data can lead to varying behaviour. 
The specification of IDs can help to assign gradients to specific tasks / behaviours, but some frameIDs are reserved for special purposes. 

###### Message file

```
Header header 
string  parent_frame

geometry_msgs/Vector3 p
geometry_msgs/Quaternion q
geometry_msgs/Vector3 direction

int8 attraction 
float32 diffusion 
float32 goal_radius 

float32 ev_factor 
float32 ev_time
time ev_stamp

bool moving

diagnostic_msgs/KeyValue[] payload
```

* **header**: standard ROS header 
  * uint32 seq: consecutively increasing ID; set by system
  * time stamp: sec + nsec - has to be set using ros Time 
  * string frame_id: to associate data for a purpose 
* **string parent_frame**: frame ID of the parent / sender (e.g. the sending robot)
* main gradient attributes:
  * **p**: gradient center
  * **q**: quaternion indicating orientation (x,z,y,w as calculated by tf.transformations)
  * **direction**: gradient sector direction; specifies the direction when quaternion indicates 0 angles
  * **attraction**: attractive `1` or repulsive `-1` gradient 
  * **diffusion**: radius in which information will be spread
  * **goal_radius**: area with minimum attraction or maximum repulsion; total reach of gradient = diffusion + goal_radius 
* evaporation:
  * **ev_factor**: evaporation factor `[0,1]` applied on diffusion (`diffusion *= ev_factor`) after `ev_time`; `1` = no evaporation, `0` = data loss after `ev_time`
  * **ev_time**: delta time `>= 0` in which evaporation is applied 
  * **ev_stamp**: time stamp used for evaporation calculations; should be set equal to header time stamp initially 
* **moving**: gradient is moving (True) or static (False)
* **payload**: array of key-value-pairs to store payload data 


The parameters for the gradient sector are already included in the `SoMessage` file, but at the moment always the whole gradient (sphere) is used in calculations.

###### reserved frameIDs:
* **'DEFAULT'**: value is used to specify aggregation option (store_data in soBuffer) for frameIDs having no specific option assigned to  
* **'None'**: value is assigned to all gradient messages which have no frameID when received 


sobuffer(.py)
-------------

The central component of the so_data package is the soBuffer. 
It provides the basic functionalities to gradient messages: evaporation, aggregation and spreading (receiving). 
These are necessary for decision as well as movement patterns.
Gradient data is received via the `so_data` topic and will be evaporated and aggregated by the soBuffer. 
The individual mechanisms can request gradient data needed for the calculations from the soBuffer. 


### soBuffer Parameters 

The soBuffer provides several parameters which can be set to adjust the data storage behaviour. 
The following parameters can be set (default values specified in brackets):

* **aggregation** ({'DEFAULT': 'max'}): dictionary (key: frameID, value: aggregation option) of aggregation type per frameID. 'DEFAULT' is used for all frameIDs which have no option specified in 
the dictionary and has to be specified. Does not affect storage of moving gradients. Enumeration `AGGREGATION` includes the following options:
  * **min** = keep gradient with minimum diffusion radius + goal radius (at a position / within aggregation_distance)
  * **max** = keep gradient with maximum diffusion radius + goal radius(at a position / within aggregation_distance)
  * **avg** = keep average gradient (at a position / within aggregation_distance) 
  * **new** = keep newest / last received gradient (at a position / within aggregation_distance) 
  * **newparent** = stores the newest gradient per parent frame 
  * **newframe** = stores the newest gradient per frame 
* **aggregation_distance** (1.0): radius in which gradient data is aggregated (see aggregation) 
* **min_diffusion** (0.1): float; threshold value specifying minimum diffusion radius gradient has to have when goal_radius == 0.0 to be stored in soBuffer
* **view_distance** (2.0): float; radius in which agent can sense gradients (starting from agent position / agent gradient center). Should be >= goal_radius specified in agent's own gradient.  
* **id** (''): agent's id, gradients with this id are stored in self._own_pos 
* **moving_storage_size** (2): int; defines number of gradients which will be stored for moving gradients. Set 0 to avoid storing moving gradients. 
* **store_all** (True): defines whether all frameIDs will be stored or only frameIDs indicated in framestorage 
* **framestorage** ([]): array listing all frameIDs which should be stored. Empty array leads to not storing any gradients. 
* **pose_frame** ('robot'): frame ID indicating gradient data of agents / robots (poses)
* **ev_thread** (False): bool which can be set to trigger evaporation in a certain frequency by a Timer thread and not by every return and storage process
* **ev_time** (5): delta time in which thread will trigger evaporation of buffer (in seconds)

### Gradient Storage

One key part of the buffer is the storage of incoming gradients. 
`store_data` applies evaporation (see **Evaporation**), stores moving gradient data in `self._moving`, the agent's own position in `self._ownpos` and all other gradients in `self._static`. 

```python
def store_data(self, msg)
```

All gradients are received via the `so_data` topic. 
Each buffer subscribes to this topic when initialized and defines `store_data` as the subscription callback. 

```python
rospy.Subscriber('so_data', SoMessage, self.store_data)
```

`store_data` will then care about storing the data as specified with the soBuffer parameters `aggregation`, `min_diffusion`, `pose_frame`, `id`, `moving_storage_size`, `framestorage` and `aggregation_distance`. 
`aggregation` is a dictionary which allows to specify based on frameIDs how the data should be stored. 
The key `DEFAULT` specifies the aggregation option for all frames for which no aggregation option is defined in `aggregation`. 
Options are `min`, `max`, `avg` which store the gradient with maximum / minimum diffusion and goal radius and respectively the average diffusion and goal radius as well the averaged gradient center (soMessage.p) at a position (or rather within the aggregation distance). 
The option`new` stores the last received gradient.
All of these options stores data based on it's position (p). 
Option `newparent` stores the last received gradient based on the parent frame while `newframe` stores the newest gradient based on the header frame.
All options might be suitable for movement related gradients while `newest` is most appropriate for gradients including payload used for decision making. 
The storing mechanisms allows to store only gradients with certain frameIDs. 
In this case `store_all` has to be set to `false` and the frameIDs to be stored will be specified in `frameids`. 
The `aggregation_distance` attribute ensures that gradients within a certain distance will be aggregated, e.g. gradients centered at (9|9|0) and (8|9|0) will be aggregated when the `aggregation_distance` is set to 1.0. 
Setting this parameter to 0 means that only gradients at the exact same position will be aggregated. 
 
The calculations of the methods which return values for use in behaviours or sensors are based on the stored data.  

### Evaporation 

Evaporation is one of the basic mechanisms presented in the paper by Fernandez-Marquez et al. 
It is applied both before storing received data as well as when requesting a list of gradients from the buffer (options see Aggregation). 
To ensure that all data is up-to-date, each received message is evaporated before it is stored using `_evaporate_msg(msg)`. Parameters: `msg` is a soMessage. 
It returns the evaporated message or `None` in cases where the diffusion radius is smaller than the specified minimum diffusion and the goal radius of the gradient is zero. 

The whole buffer can be evaporated using `_evaporate_buffer()`. 
It will iterate through all stored data (in self._static and self._moving) and update the data based on the evaporation settings. 
This ensures that the used data is always up to date. 

```python 
def _evaporate_msg(self, msg)

def _evaporate_buffer(self)
```

The evaporation frequency (or rather delta t) is specified in the soMessage as `ev_time` and the evaporation factor as `ev_factor`. 
Therewith, evaporation can be specified per gradient. 
`ev_time` has to be larger or equal zero. 
After `ev_time`, the diffusion radius will be multiplied with `ev_factor`: `msg.diffusion *= ev_factor`. 
`ev_factor` has to be set the interval `[0,1]` with `1` leading to no evaporation and `0` to the complete evaporation after `ev_time`. 
In case that `ev_time == 0` and `ev_factor < 1` the gradient will instantly evaporate completely. 

Gradients without goal radius (soMessage `goal_radius = 0`) and a diffusion smaller than the minimum diffusion radius (`min_diffusion`, see soBuffer Parameters) will be deleted immediately. 
 

### Aggregation 

Aggregation is realized in a two step process within the SoBuffer. 
The first level of aggregation is applied when data is stored (see Gradient Storage).
The second level is done when a list of gradients is requested by the buffer. 
SoBuffer will only return gradients which match the specified criteria and are within view distance. 
The criteria are/might be:

* frameids: frameids of gradient data to be returned (None == all frames will be considered) 
* static: static gradient data will be returned
* moving: moving gradient data will be returned 
* frame: frame which will be considered in returning agent sets; if no frame is specified 'pose_frame' will be used

The following options are available:

1. Get all gradients within view distance 

```python
    def gradients(self, frameids=None, static=True, moving=True)
```

2. Get all gradients (view distance = np.inf) 
```python
def all_gradients(self, frameids=None, static=True, moving=True)
```

2. Get all repulsive gradients within view 

```python
    def repulsive_gradients(self, frameids=None, static=True, moving=True)
```

3. Get all attractive gradients within view 

```python
    def attractive_gradients(self, frameids=None, static=True, moving=True)
```

4. Get the relatively nearest attractive gradient (= attractive gradient with minimum attraction based on Balch and Hybinette)

```python
    def max_attractive_gradient(self, frameids=None, static=True, moving=True)
```
 
5. Get the gradient with the strongest potential robot; to compare attractive and repulsive gradients, the attraction value was adjusted calculating (1-attraction) 

```python
    def strongest_gradient(self, frameids=None, static=True, moving=True)
```

6. Get list of agents within view (last received value)

```python
    def agent_list(self, frame, static=False, moving=True)
```

7. Get list of agent data (all received values)

```python
    def agent_set(self, frame)
```

8. Get list of static gradients with a specified frame within a specified view angle

```python
    def static_list_angle(self, frame, view_angle)
```

9. Get attractive gradient with minimum reach (diffusion + goal radius) within view 

```python
def min_reach_attractive_gradient(self, frameids=None, static=True, moving=True)
```

10. Get attractive gradient with maximum reach (diffusion + goal radius) within view

```python
 def max_reach_attractive_gradient(self, frameids=None, static=True, moving=True)
```

11. Get relatively furthest attractive gradient (= attractive gradient with maximum attraction based on Balch and Hybinette)

```python
def min_attractive_gradient(self, frameids=None, static=True, moving=True)
```
 
 
 gradient(.py)
--------------

Module gradient includes two different approaches of calculating the attraction/repulsion of gradients. 
The first approach follows the approach presented in "Social potentials for scalable multi-robot formations" by Balch and Hybinette (2000). 
The second approach is based on "New Potential Functions for Mobile Robot Path Planning" by Ge and Cui (1999) which was enhanced with an inner goal_radius which leads to infinite repulsion. 

##### Balch and Hybinette (2000)

The paper of Balch and Hybinette includes formulas to calculate attraction and repulsion of gradients. Attraction values are within `[0,1]` while repulsion values are within `[0, inf.]`. 
Attraction and repulsion can be combined to generate the movement vector. 
In some scenarios the attractive gradient might not be reached as its attraction and repulsion lead to a zero potential value at a point not being the attractive gradient source. 
Parameters: `gradient` is a soMessage with the attractive/repulsive gradient data, `pose` is the current position of the robot 

```python
def _calc_attractive_gradient(self, gradient, pose)

def _calc_repulsive_gradient(self, gradient, pose)
```

##### Ge and Cui (1999)

In comparison to the approach by Balch and Hybinette, Ge and Cui guarantee with their approach that the attractive gradient source will be reached. 
The implementation of the attractive gradient is similar to Balch and Hybinette, the only difference is that Balch and Hybinette return normalized gradient values while Ge and Cui return absolute values. 
Therewith, to determine the closest attractive gradient the method based on Balch and Hybinette can be used. 
Ge and Cui enhanced the repulsive gradient calculation to ensure that the gradient source can be reached. 
The formulas in this paper were enhanced with setting the repulsive gradient to infinite when the agent is within its `goal_radius`. 
The attractive gradient was enhanced with being set to zero in its goal region. 

```python 
def _calc_attractive_gradient_ge(self, gradient, pose)

def _calc_repulsive_gradient_ge(self, gradient, goal, pose)
```

Decision and Movement Patterns
-------------------------------

### patterns(.py)

Module patterns includes two abstract classes which are blueprints for movement mechanisms and decision mechanisms. 

Movement patterns require the implementation of method `move()` which returns a movement vector.
Decision patterns require the implementation of method `calc_value()` which determines the agent's current value and sets its state accordingly. 
Decision patterns have furthermore function `spread()` which allows to spread the current value of the agent.
It sets furthermore state and value of the mechanism. 

### Movement Patterns / Mechanisms 

All movement mechanisms return `None` when no gradient is within view distance.   

The following movement patterns were implemented: 

1. Repulsion
2. Flocking
3. Chemotaxis
4. Ant Foraging

####  repulsion(.py)

Module repulsion includes two implementations of the repulsion mechanism. 

`RepulsionFernandez` implements the repulsion mechanism as presented in Fernandez-Marquez paper "Description and composition of bio-inspired design patterns".
 It requires, as all patterns, an soBuffer as input. 

```python
class RepulsionFernandez(MovementPattern):
    def __init__(self, buffer, frame=None, static=False, moving=True)
```

`frame` allows to specify the header frame id which is used for agent data. 
If no frame is specified, the pose frame of the buffer will be used. 
`static` and `moving` indicate whether moving gradient data or static gradient data will be returned by the soBuffer. 

The calculation is based on a list of agent gradients returned by buffer method `agent_list`. 

Fernandez-Marquez's formula incorporates that the agents will try to stay as close as possible (outside repulsion radius) when the view distance is larger than the repulsion radius. 

`RepulsionGradient` implements the repulsion mechanism using the formulas to calculate repulsive gradients by Balch and Hybinette (see gradient.py). 

```python 
class RepulsionGradient(MovementPattern):
    def __init__(self, buffer, frame=None, static=False, moving=True)
```

The parameters are similar to `RepulsionFernandez`. 
More information about the gradient calculation can be found in chapter gradient(.py). 

### flocking(.py)

Module flocking contains algorithms to realize flocking in free-space (free flocking). 
It offers all formulas presented in "Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory" by Olfati-Saber (2006) for Algorithm 1.

Algorithm one consists of two parts: 

1. gradient-based term
2. velocity consensus term 

which incorporate the three flocking rules presented by Reynolds. 

Please find more information about the formulas in the paper by Olfati-Saber.

The approach leads to some problems in combination with RHBP and some enhancements might be required as the velocity of the turtles might be zero at several occasions.

The calculations need the following input data: `Boid = collections.namedtuple('Boid', ['p', 'v'])` with `p` being the robot's current position and `v` being the robot's velocity.
`Flocking` implements flocking based on Olfati-Saber as a movement pattern. 
In class `Flocking` data from an SoBuffer instance (from method `agent_set()`) is transformed to match this input and the methods implementing the flocking behaviour based on Olfati-Saber are invoked.

```python

class Flocking(MovementPattern):
    def __init__(self, buffer, a, b, h, epsilon, max_acceleration, frame=None,
                 moving=True, static=False, maxvel=1.0, minvel=0.1)
```

Input are an SoBuffer instance and several parameters for flocking. 
These are `a`, `b`, `h` and `epsilon`. 
`max_acceleration` sets the maximum acceleration the agent will have. 
`maxvel` and `minvel` set furthermore the maximum or minimum velocity. 
`frame` specifies the header frame id which is used for agent data. 
If no frame is specified, the pose frame of the buffer will be used. 
`static` and `moving` indicate whether moving gradient data or static gradient data will be returned by the soBuffer. 

#### flockingrey(.py)

Module flockingrey includes the implementation of flocking based on the formulas presented by Reynolds in his paper [Steering Behaviors For Autonomous Characters](www.red3d.com/cwr/steer/gdc99/).

The calculations require either robot positions or heading vectors (not velocities!). 
The flocking methods need a current gradient of the agent and a list of neighbor gradients. 
The current heading vector of the robot is calculated using the SoMessage values `q` (orientation) and `direction`.
The quaternion q is then transformed to a transformation matrix using method `tf.transformations.quaternion_matrix(quaternion)`.
Multiplying the transformation matrix with the direction vector results in the current heading vector.

The module contains the three methods 'cohesion', 'separation' and 'alignment' described by Reynolds. 
The three movement vectors calculated by these methods can be summed up to determine the overall movement vector. 

Furthermore, the implementation of the flocking mechanism as a subclass of `MovementPattern` (see patterns.py) is provided by this module. 

`FlockingRey` calculates the overall movement vector based on the formulas by Reynolds. 
The calculations of cohesion, separation and alignment require a list of SoMessages as the input. 
Class `FlockingRey` request a list from the buffer using method `agent_list`. 

```python 
class FlockingRey(MovementPattern):
    def __init__(self, buffer, frame=None, moving=True, static=False, maxvel=1.0)
```
A `buffer` has to be handed offer to the pattern implementation.
`Frame` allows to specify the header frame ID which is used for agent data in this setting. 
If no frame is specified, the pose frame parameter of the buffer will be used. 

`Static` and `moving` indicate whether moving gradient data or static gradient data will be returned by the soBuffer. 
`maxvel` sets the maximum length of the movement vector. 


#### chemotaxis(.py)
 
Module chemotaxis includes several implementations of chemotaxis behaviour and makes use of the gradient formulas implemented in module gradient. 

##### ChemotaxisBalch 

ChemotaxisBalch implements chemotaxis behaviour following one goal while avoiding repulsive gradients. 
It is based on the formulas by Balch and Hybinette (see gradient.py). 

```python 
class ChemotaxisBalch(MovementPattern):
    def __init__(self, buffer, frames=None, moving=True,
                 static=True, maxvel=1.0, minvel=0.1)
```

A buffer has to be handed over to the mechanism.
Frames allows to specify a list of gradient frame IDs which will be considered in the movement vector calculation.
Moving and static define whether moving or static gradient will be considered. 
Only moving gradients with the pose frame specified in SoBuffer will be considered in this case. 
Maxvel and minvel specify the maximum and respectively minimum velocity of the agent / length of the movement vector. 

The mechanism request the relatively closest attractive gradient from the buffer (minimum attraction value; `get_attractive_gradient()` in SoBuffer) and aims to reach this goal. 
Furthermore, it requests a list of repulsive vectors within view. 
The movement vector for the attractive gradient is calculated and added to the sum of the repulsive movement vectors to determine the overall movement vector. 

##### ChemotaxisGe 

ChemotaxisGe implements chemotaxis behaviour following one goal while avoiding repulsive gradients. 
It is based on the formulas by Ge and Cui (see gradient.py). 

```python
class ChemotaxisGe(MovementPattern):
    def __init__(self, buffer, frames=None, moving=True, static=True, maxvel=1.0, minvel=0.1)
```
A buffer has to be handed over to the mechanism.
Frames allows to specify a list of gradient frame IDs which will be considered in the movement vector calculation.
Moving and static define whether moving or static gradient will be considered. 
Only moving gradients with the pose frame specified in SoBuffer will be considered in this case. 
Maxvel and minvel specify the maximum and respectively minimum velocity of the agent / length of the movement vector. 

The mechanism request the relatively closest attractive gradient from the buffer (minimum attraction value; `get_attractive_gradient()` in SoBuffer) and aims to reach this goal. 
Furthermore, it requests a list of repulsive vectors within view. 
The movement vector for the attractive gradient is calculated and added to the sum of the repulsive movement vectors to determine the overall movement vector. 


##### Other Chemotaxis Implemenations

Module chemotaxis includes furthermore the following implementations of chemotaxis behaviour:

* `class CollisionAvoidance(MovementPattern)`: mechanism to avoid all repulsive gradients
* `class FollowAll(MovementPattern)`: mechanism to follow the overall potential (all gradients within view)
* `class AvoidAll(MovementPattern)`: mechanism to avoid all gradients within view 
* `class FollowMax(MovementPattern)`: mechanism to follow the strongest gradient (max potential) 
* `class FollowMin(MovementPattern)`: mechanism to follow the relatively furthest gradient
* `class FollowMinReach(MovementPattern)`: mechanism to follow the gradient with minimum reach within view
* `class FollowMaxReach(MovementPattern)`: mechanism to follow the gradient with maximum reach within view 


#### Foraging

Module Foraging includes several behaviours required for the ant foraging pattern. 
The included behaviours are:

* Decision Mechanism: decision between exploration and exploitation
* Movement Mechanisms:
    * Exploration: random movement of the agent
    * Exploitation: chemotaxis behaviour following a pheromone trail
    * Return to nest: movement towards nest while depositing pheromones
    
##### Decision Mechanism 

The decision mechanism `ForagingDecision` is an implementation of the decision pattern. 

```python
class ForagingDecision(DecisionPattern):
    def __init__(self, buffer=None, probability=0.5)
```   

A `buffer` can be handed over as a parameter as this is standard for all decision mechanisms. 
But it is currently not used in the decision or rather `calc_value()` implementation.

`probablity` is the exploration probability. 
With this probability, the state will be set to `Exploration`. 
Otherwise, `Exploitation` will be set as the state.
`probability` is assigned to the value of the decision mechanism.

##### Exploration

Exploration is a Movement Pattern which creates random movement vectors. 

```Python
class Exploration(MovementPattern):
    def __init__(self, buffer=None, maxvel=1.0, minvel=0.1)
```

A buffer has to be handed over to the mechanism.
Maxvel and minvel specify the maximum and respectively minimum velocity of the agent / length of the movement vector. 


##### Exploitation

Exploitation lets the agent follow a trail of pheromones. 
It follows all gradients within specified view angles.

```python
class Exploitation(MovementPattern):
    def __init__(self, buffer, frames, angle_xy=1.3, angle_yz=np.pi, maxvel=1.0, 
                 minvel=0.5)
```

A buffer has to be handed over to the mechanism.
Frames allows to specify a list of gradient frame IDs which will be considered in the movement vector calculation.
angle_xy specifies the view angle in the xy-plane.
angle_yz specified the view angle in the yz-plane. 
Maxvel and minvel specify the maximum and respectively minimum velocity of the agent / length of the movement vector. 


##### Return to Nest 

DepositPheromones is a movement pattern which lets the agent return to the nest and deposit pheromones on the way.
It is based on the ChemotaxisGe behaviour. 

Before the agent moves, it will deposit a pheromone at the place it currently is posed on. 
The movement vector is calculated using the move() implementation of the ChemotaxisGe behaviour. 

```python
class DepositPheromones(ChemotaxisGe):
    def __init__(self, buffer, frames=None, moving=False, static=True, 
                 maxvel=1.0, minvel=0.5, frame='Pheromone', attraction=1, 
                 ev_factor=0.9, ev_time=5)
```
A buffer has to be handed over to the mechanism.
Frames allows to specify a list of gradient frame IDs which will be considered in the movement vector calculation.

Moving and static define whether moving or static gradient will be considered. 
Only moving gradients with the pose frame specified in SoBuffer will be considered in this case. 
Maxvel and minvel specify the maximum and respectively minimum velocity of the agent / length of the movement vector. 

frame, attraction, ev_factor and ev_time are the paramters for the spread pheromone gradients. 

### decisions(.py)

Module decisions includes sample implementations of decision patterns. 
The implementation of the decision mechanisms is application specific. 

#### Sample Gossip Mechanism

Module decision includes a sample gossip mechanism which determines the maximum value spread as a payload attribute. 
The received maximum values are compared to the own current value.
In case that a received value is larger than the current value, the current value will be updated. 

```python
class GossipMax(DecisionPattern):
    def __init__(self, buffer, frame, key, state=1, moving=True, static=False, 
                 diffusion=np.inf, goal_radius=0, ev_factor=1.0, ev_time=0.0)
```

A SoBuffer has to be handed over to the mechanism which will provide the necessary gradients data. 
In this case the data will be provided by method `agent_list`. 
Furthermore a frame and a key have to be specified to indicate which gradient frame id is used for the gossip mechanism and which key the payload data to be considered has. 
Parameter state sets the initial value which will be compared and spread. 
Moving and static specify whether moving or static gradient data should be returned by the soBuffer.
The other parameters allow to adjust the gradient message which will be spread to fit the application scenario. 


#### Sample Morphogenesis Mechanism 

Module decision includes a sample morphogenesis mechanism which determines the barycenter of a robot group and lets it spread a center gradient. 
Each agent determines the distance to each received neighbor gradient. 
The agents spread morphogenetic gradients which include the position and the current sum of distances if the robots.  
The own and the received sum of distances will be compared. 
The barycenter is the agent which has the smallest sum of distances. 

```python 
class MorphogenesisBarycenter(DecisionPattern):
    def __init__(self, buffer, frame, key, moving=True, static=False,
                 goal_radius=0.5, ev_factor=1.0, ev_time=0.0, diffusion=np.inf,
                 attraction=-1, state='None', goal_center=2.0,
                 moving_center=False, attraction_center=1,
                 diffusion_center=20)
```

In this mechanism a buffer is required too. 
The data used in the calculation will be provided by method `agent_list` of the buffer.
A frame and a key have to be specified to indicate which gradient frame id is used for the morphogenesis mechanism and which key the payload data under consideration has.
Moving and static defines whether moving or static gradient data will be returned by the buffer. Usually moving gradients are considered in this pattern. 
The other parameters allow to adjust the gradient message with morphogenetic data that will be spread. 
All parameters with '_center' define the center gradient message which is spread by the barycenter. 


#### Quorum

Pattern quorum sensing could be implemented independent from particular scenarios.
Quorum is a decision which is solely based on the number of neighbors within view of an agent. 
In case that a threshold number of agents within view is passed, the agent's state will be set to True. 
Otherwise the state is False. 

```python 
class Quorum(DecisionPattern):
    def __init__(self, buffer, threshold, frame=None, value=0, state=False,
                 moving=True, static=False)
```

An SoBuffer instance has to be handed over to the mechanism.
It provides a list of neighbors of the agents with method `agent_list()`. 
threshold defines the number of agents which should be at least within view distance. 
frame defines the header frame ID of the quorum gradients. 
value is the initial value of the agent. 
state is the initial state of the agent. 
Moving and static defines whether moving or static gradient data will be returned by the buffer. Usually moving gradients are considered in this pattern. 


supplements(.py)
------------------

Module supplementary includes additional mechanism implementations which are not directly related to the patterns described by Fernandez-Marquez et al., but build using them.

It includes the following mechanisms:

1. DepositPheromonesMin: this mechanism follows the gradient with minimum reach (FollowMinReach) within view while depositing pheromones

```python
class DepositPheromonesMin(FollowMinReach)
```

2. DepositPheromonesRandom: this mechanism lets an agent move randomly (Exploration) while depositing pheromones 

```python
class DepositPheromonesRandom(Exploration)
```

3. SpreadGradient: this mechanisms enables the spreading of a gradient at the current position of the robot; can be executed by using DecisionBehaviour of rhbp_selforga package 
 
 ```python
class SpreadGradient(object)
```


calc(.py)
---------

The file calc.py includes some basic vector (Vector3) calculations which are commonly required. These are:

* `def unit_vector(vector)` and `def unit_vector3(vector)`: returns a unit vector based on the input vector
* `def angle_between(v1, v2)`: returns the directed vector between two vectors (np.array) (max. 2D at the moment)   
* `def angle_between_vector3(v1, v2)`: returns directed vector between two vectors (Vector3)
* `def get_gradient_distance(gradpos, pose)`: returns distance between agent and gradient center 
* `def vector_length(vector)`: returns the length of a vector 
* `def delta_vector(q1, q2)`: returns difference between vector q1 and q2 
* `def add_vectors(q1, q2)`: returns sum of two vectors 
* `def adjust_length(q, length)`: returns a vector in direction of `q` with length `length`
* `def random_vector(length)`: returns a random vector with length `length`


sobroadcaster(.py)
------------------

The class SoBroadcaster can be used to publish data to the topic `so_data`. This topic is used for self-organization purposes and requires the message format `soMessage`. 
Within a message all necessary gradient information can be specified to enable the calculations for different self-organization behaviours. 
It can be used to spread gradients present in the environment or to let the robots spread specific gradients. 

Data can be send using the method 

```python
def send_data(self, message)
```

which requires a SoMessage or a list of SoMessages as input. 

The soBroadcaster can either be used in a separate node to send data in a certain frequency (to be specified by `rospy.Rate()`) or within behaviours to send gradient messages when necessary. 
Therewith, `soBroadcaster` is the basis for the implementation of spreading behaviour. 


gradientnode(.py)
------------------
Node enabling to send artificial gradients in the environment. 
Gradients can be specified in the list in method get_gradient(index) and set via the gradient launch file. 
The spreading frequency can be set (`ros.Rate()`) via the launch file too. 

There are two methods included which are purely for convenience. 
The first one is the `create_gradient(...)` method allowing to create a soMessage by specifying the required parameters and setting defaults for the rest. 
The second method is `get_gradient(index)` which returns a gradient list based on the index position handed over. 
New gradient lists can be added to the currently available set and the method can be used in other files, e.g. in main.py of the swarm_behaviour package to draw the gradients in the turtlesim environment. 


topicgradienttf(.py), posegradienttf(.py), posestampedgradienttf(.py)
---------------------------------------------------------------------

topicGradientTf offers an abstract class which can be used as a blueprint to transform data received by a topic to a gradient message and spreach it. 
A sample implementation can be found in poseGradientTf and poseStrampedGradientTf. 
There a subscription to the pose topic (geometry_msgs/pose or poseStamped) is made and the received data is put into a soMessage and buffered in `self._current_value`. 
The message can either be spread right away (as part of the subscription callback) or the sending is done in a ROS node with a specific frequency. 
Using the class within a ROS node leads to sending the last buffered soMessage. 