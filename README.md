SO_DATA MANUAL
===============

The package so_data provides components which are required to implement self-organization. 
It provides functionality to send data to the `so_data` topic as well as to aggregate, store and use it for calculations. 
Therewith, it provides the basic mechanisms and the gradient patterns specified by in "Description and composition of bio-inspired design patterns: a complete overview" by Fernandez-Marquez et al. (2013). 
Unit tests were done for the core components to ensure correctness of mathematical computations. 
All methods work for up to three dimensions; the code can be adapted for use in more dimensions with minor enhancements. 
The package can be used in combination with the RHBP, but as well on its own. 


Package components
-------------------

The package consists of the following components:

src: 

* **calc.py**: provides helper functions which are used by several components of the package (+ unit test)
* **flocking.py**: provides methods to calculate the flocking vector based on the paper by Olfati-Saber (+ unit test) 
* **flockingrey.py**: provides methods to calculate a flocking vector based on the formulas by Reynolds (+ unit test)
* **gradientnode.py**: allows to create nodes which spread artificial gradients 
* **sobroadcaster.py**: allows to publish data to the so_data topic which will be subscribed to in the soBuffer 
* **sobuffer.py**: implements a layer which allows to store gradient data and to do calculations necessary for self-organizing behaviour (+ unit test)  
* **topicgradienttf.py**: abstract class which can be used as a blueprint to transform topics in soMessages and publish them to the `so_data` topic 
* **posegradienttf.py**: implementation of `topicGradientTf` for the pose topic 

msg:

* **SoMessage.msg**: message file describing gradients 

The robot pose is considered to be in the form of a [`geometry_msgs/Pose`](docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html) Message. 


SoMessage
---------

soMessages are used to specify gradients: either in environment or as robot data. 
The specification of the soMessage data can lead to varying behaviour. 
The specification of frameIDs can help to assign gradients to specific tasks / behaviours, but some frameIDs are reserved for special purposes. 

###### Message file

```
Header header 

geometry_msgs/Vector3 p
geometry_msgs/Quaternion q

int8 attraction 
float32 diffusion 
float32 goal_radius 

float32 ev_factor 
float32 ev_time
time ev_stamp

geometry_msgs/Vector3 direction
float32 angle_x
float32 angle_y

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
  * **attraction**: attractive `1` or repulsive `-1` gradient 
  * **diffusion**: radius in which information will be spread
  * **goal_radius**: area with minimum attraction or maximum repulsion; total reach of gradient = diffusion + goal_radius 
* evaporation:
  * **ev_factor**: evaporation factor `[0,1]` applied on diffusion (`diffusion *= ev_factor`) after `ev_time`; `1` = no evaporation, `0` = data loss after `ev_time`
  * **ev_time**: delta time `>= 0` in which evaporation is applied 
  * **ev_stamp**: time stamp used for evaporation calculations; should be set equal to header time stamp initially 
* gradient sector:
  * **direction**: gradient sector direction; specifies the direction when quaternion indicates 0 angles
  * **angle_x**: first gradient angle [0, 2*pi]
  * **angle_y**: second gradient angle [0, 2*pi]
* **moving**: gradient is moving (True) or static (False)
* **payload**: array of key-value-pairs to store payload data 


The parameters for the gradient sector are already included in the `SoMessage` file, but at the moment always the whole gradient (sphere) is used in calculations.
###### reserved frameIDs:
* **'DEFAULT'**: value is used to specify aggregation option (store_data in soBuffer) for frameIDs having no specific option assigned to  
* **'None'**: value is assigned to all gradient messages which have no frameID when received 



sobuffer(.py)
-------------

The central component of the so_data package is the soBuffer. 
It subscribes to the so_data topic and stores all received gradients in an aggregated manner. 
Furthermore, it includes several calculation options which are based on gradients. 
The soBuffer provides calculations necessary to implement self-organizing behaviours. 
Various parameters allow to adjust the soBuffer to the needs of a self-organization pattern. 

### soBuffer Parameters 

The soBuffer provides several parameters which can be set to adjust the data storage and calculation behaviour. 
The following parameters can be set (default values specified in brackets):

* **aggregation** ({'DEFAULT': 'max'}): dictionary (key: frameID, value: aggregation option) of aggregation type per frameID. 'DEFAULT' is used for all frameIDs which have no option specified in 
the dictionary and has to be specified. Does not affect storage of moving gradients. Enumeration `AGGREGATION` includes the following options:
  * **min** = keep gradient with minimum diffusion radius + goal radius (at a position / within aggregation_distance)
  * **max** = keep gradient with maximum diffusion radius + goal radius(at a position / within aggregation_distance)
  * **avg** = keep average gradient (at a position / within aggregation_distance) 
  * **new** = keep newest / last received gradient (at a position / within aggregation_distance) 
  * **newparent** = stores the newest gradient per parent frame 
* **aggregation_distance** (1.0): radius in which gradient data is aggregated (see aggregation) 

* **min_diffusion** (0.1): float; threshold value specifying minimum diffusion radius gradient has to have when goal_radius == 0.0 to be stored in soBuffer
* **view_distance** (2.0): float; radius in which agent can sense gradients (starting from agent position / agent gradient center). Should be >= goal_radius specified in agent's own gradient.  
* **id** (''): agent's id, gradients with this id are stored in self._own_pos 

* **result** (None): specifies how soBuffer data (within agent's view) is aggregated s.t. one value is returned. 
A list of result options has to be passed over.
 Enumeration `RESULT` provides the following options:
  * **all** = movement vector considering all vectors of the potential field will be returned 
  * **max** = movement vector based on maximum repulsion / attraction (goal+diffusion) will be returned 
  * **near** = movement vector following nearest attractive gradient by avoiding repulsive gradients will be returned; robot might not reach gradient source  
  * **reach** = movement vector following nearest attractive gradient by avoiding repulsive gradients will be returned; allows to reach gradient source in comparison to 'near' 
  * **avoid** = movement vector leading away from all sensed gradients will be returned 
  * **flocking** = movement vector for flocking motion based on Olfati-Saber
  * **flockingrey** = movement vector for flocking motion based on Reynolds 
  * **collision** = movement vector avoiding all repulsive gradients 
* **result_moving** (True): consider moving gradients (True) or not (False) in calculations 
* **result_static** (True): consider static gradients (True) or not (False) in calculations (except for flocking and repulsion which are always based only on moving gradients)
  
* **repulsion** (None): collision avoidance between neighbors / agents (moving gradients). 
Considers only moving gradients with `self._pose_frame` frame ID. 
Should only be used on its own or in combination mit result_moving = False. 
Options can be specified via enumeration `REPULSION` and the following settings:
  * **gradient** = gradient/potential field approach is used to calculate repulsion vector (formulas of 'reach' option of result)
  * **repulsion** = repulsion vector is calculated based on formula presented in Fernandez-Marquez et al.
  * **reach** = moving vectors are considered in vector calculations based on Ge & Cui (result == reach is required)
  
* **moving_storage_size** (2): int; defines number of gradients which will be stored for moving gradients. Set 0 to avoid storing moving gradients. 

* **store_all** (True): defines whether all frameIDs will be stored or only frameIDs indicated in framestorage 
* **framestorage** ([]): array listing all frameIDs which should be stored. Empty array leads to not storing any gradients. 

* **max_velocity** (1.0): maximum velocity of robot (= length of returned vector)
* **min_velocity** (0.1): minimum velocity of robot (= length of returned vector)

* **pose_frame** ('robot'): frame ID indicating gradient data of agents / robots (poses)

Chemotaxis parameters:

* **chem_frames** ([]): specifies which frame IDs should be considered in calculating the movement vectors based on the gradient fields; empty list means all frames will be considered

Flocking parameters:

These parameters are only relevant for the flocking calculations based on Olfati-Saber (`RESULT.FLOCKING`). 

* **a, b**: action function parameters with `0 < a <= b; c = |a-b|/np.sqrt(4ab)`
* **h**: parameter (0,1) specifying boundaries of bump function
* **epsilon**: sigma norm parameter (0,1)
* **max_acceleration**: maximum acceleration of robot

Decision patterns 

* **decision** (None): `DECISION` enum value indicating which gradient data is returned

Quorum Sensing parameters: 

* **threshold** (2): quorum sensing threshold to be passed 

Morpogenesis parameters: 

Morphogenetic gradients have to be `moving = True` as they are tied to agents and the differentiation based on the parent_frame is important. 

* **state** (STATE.None): should be set to one option of Enumeration `STATE`, e.g. 'Center' or 'None'
* **key** (None): specifies payload KeyValue key for morphogenetic data
* **morph_frame** ('morphogenesis'): specifies frame ID (header frame_id) for morphogenetic gradients. 

Gossip parameters:

* **gossip_frame** ('gossip'): frame ID indicating gradient data for gossip patterns
* **gossip_key** (''): specifies payload KeyValue key for gossip data 


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
Option `newparent` stores the last received gradient based on the parent frame.
All options might be suitable for movement related gradients while `newest` is most appropriate for gradients including payload used for decision making. 
The storing mechanisms allows to store only gradients with certain frameIDs. 
In case that **only** moving gradients should be stored, `store_all` should be set to `False` and no frameId specified in `framestorage`. 
The `aggregation_distance` attribute ensures that gradients within a certain distance will be aggregated, e.g. gradients centered at (9|9|0) and (8|9|0) will be aggregated when the `aggregation_distance` is set to 1.0. 
Setting this parameter to 0 means that only gradients at the exact same position will be aggregated. 
 
The calculations of the methods which return values for use in behaviours or sensors are based on the stored data.  

### Methods for use in behaviours and sensors 

The following methods can be used to calculate information necessary for behaviours and sensors. 

##### Get movement vector

Invoking `get_current_gradient` will return a movement vector based on the parameters `result` and `repulsion`. 
This vector can be used to let the agent move, e.g. to gradient source or away from all gradients within view distance.
It is possible to specify a list of result options. 
The movement vectors of each result option calculation will be summed up. 
The result options should be chosen in a useful way, e.g. it makes sense to combine `RESULT.FLOCKING` with `RESULT.NEAR` while the combination of `RESULT.NEAR` and `RESULT.REACH` might not lead to a viable behaviour of the agent.
Furthermore, the `repulsion` vector calculated based on the repulsion parameter is added to the result. 
More information is specified in the respective subsections. 

```python
def get_current_gradient(self)
```

##### Get attractive gradients within view

In cases when an attractive gradient should be reached, attractive gradients have to be within view distance. 
The method `get_attractive_gradients_view` determines whether at least one attractive gradient is within view distance of the agent and returns True (within view)/False (not within view). 

```python
def get_attractive_gradients_view(self)
```


##### Get attractive gradient distance

To enable activation based on the distance to the nearest attractive gradient, the method `get_attractive_distance` calculates the distance to the nearest attractive gradient. 

```python
def get_attractive_distance(self)
```


##### Determine if agent senses potential field

`get_attraction_bool` determines whether the agent is currently under influence of a potential field. 
Returns True if there is no potential sensed, False otherweise. 

```python
def get_attraction_bool(self)
```

##### Flocking

The flocking calculations are based on gradient data of agents. 
This means only moving gradients with `header.frame_id == self._pose_frame` will be considered in the calculations.

###### Flocking based on Olfati-Saber

Note: This option is not working well within the RHBP setting so far! 

Invoking method `result_flocking` will return a movement vector which can be used for the flocking behaviour. 
It uses the methods implemented in flocking.py to calculate the acceleration vector.
The acceleration vector will be truncated s.t. it has the maximum length of `max_acceleration`. 
Accelaration and current velocity will be added up and truncated, s.t. the velocity is not larger than `max_velocity`. 
The calculation is solely based on moving gradient data as flocking is the result of the interaction of several moving entities.  

```python
def result_flocking(self) 
```

###### Flocking based on Reynolds 

`RESULT.FLOCKINGREY` invokes a method in the soBuffer which calculates the movement vector for flocking based on the formulas presented by Reynolds.
It uses position and heading vector of the agents. 
The methods needed can be found in flockingrey.py.

```python
def result_flockingrey(self)
```

##### Goal Achievement

The method `get_goal_reached` returns True / False based on whether the gradient source was reached or not (attraction == 0, if attractive gradient is available). 
E.g. to be used in sensors to bind activation on achievement of goal (e.g. with Boolean Activator). 
In case that more than one goal exists (>1 attractive gradient within view distance), the sensor will only consider the goal which is relatively nearest (smallest attraction value).
If no goal is available, the method will return False. 
Parameters: `frameids` specifying which gradients should be considered in the calculation (optional). 

```python
def get_goal_reached(self)
```

##### Decision Patterns

All decision patterns, namely Quorum Sensing, Morphogenesis and Gossip, make use of data provided by other agents.
Therefore, lists with the relevant gradients will be returned invoking function `get_decision`.

```python
    def get_decision(self, option)
```

`option` specifies one option of enumeration `DECISION`. 
It could be either:

* **morph**: returns a list of gradients within view distance and header frame ID `self.morph_frame`
* **quorum**: returns a list of gradients within view distance and header frame ID `self._pose_frame` (agent positions)
* **gossip**: returns a list of gradients within view distance and header frame ID `self.gossip_frame`

All of these possibilities make use of method `decision_list(self,frame)` which determines the list to return.

For options `morph` and `gossip`, the returned values will be stored in a list too which can be used in a sensor to determine whether the used data has changed. 

As the patterns are very application specific, the more detailed calculations are done in the behaviours implementations. 

### Gradient calculation 

The soBuffer includes two different approaches of calculating the attraction/repulsion of gradients. 
The first approach follows the approach presented in "Social potentials for scalable multi-robot formations" by Balch and Hybinette (2000). 
The second approach is based on "New Potential Functions for Mobile Robot Path Planning" by Ge and Cui (1999) which was enhanced with an inner goal_radius which leads to infinite repulsion. 

##### Balch and Hybinette (2000)

The paper of Balch and Hybinette includes formulas to calculate attraction and repulsion of gradients. Attraction values are within `[0,1]` while repulsion values are within `[0, inf.]`. 
Attraction and repulsion can be combined to generate the movement vector. 
In some scenarios the attractive gradient might not be reached as its attraction and repulsion lead to a zero potential value at a point not being the attractive gradient source. 
Parameters: `gradient` is a soMessage with the attractive/repulsive gradient data

```python
def _calc_attractive_gradient(self, gradient)

def _calc_repulsive_gradient(self, gradient)
```

##### Ge and Cui (1999)

In comparison to the approach by Balch and Hybinette, Ge and Cui guarantee with their approach that the attractive gradient source will be reached. 
The implementation of the attractive gradient is similar to Balch and Hybinette, the only difference is that Balch and Hybinette return normalized gradient values while Ge and Cui return absolute values. 
Therewith, to determine the closest attractive gradient the method based on Balch and Hybinette can be used. 
Ge and Cui enhanced the repulsive gradient calculation to ensure that the gradient source can be reached. 
The formulas in this paper were enhanced with setting the repulsive gradient to infinite when the agent is within its `goal_radius`. 
To ensure that the calculation of the repulsive vector works as expected, the attractive gradient was not enhanced with being set to zero in its goal region. 
By setting `collision_avoidance = 'reach'`, both static and moving gradients will be considered in the gradient calculation. 

```python 
def _calc_attractive_gradient_ge(self, gradient)

def _calc_repulsive_gradient_ge(self, gradient, goal)
```


### Basic Mechanisms

#### Evaporation 

Evaporation is one of the basic mechanisms presented in the paper by Fernandez-Marquez et al. 
It is applied both before storing received data as well as when requesting the current movement gradient (get_current_gradient). 
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
 
 
#### Repulsion

Repulsion is specified as a basic mechanism as well. 
It leads to avoiding collision between agents and can help to distribute the agents uniformly in a specific area. 
Fernandez-Marquez et al. provide a formula to calculate a repulsive vector in their paper which allows to avoid neighbors, but as well to keep a defined distance (by setting `repulsion_radius < view_distance`). 
A different option is to calculate the repulsion vector on basis of repulsive gradient calculations.
In this case only avoiding the neighbors is possible. 
In the buffer are both options implemented. 
The repulsion vector depends on the parameters `repulsion_radius` which is defined as the `goal_radius + diffusion <= view_distance` of the agent (`self._own_pos`) and `view_distance` which is set as a parameter of the buffer and should be `>= goal_radius` of the agent in most cases. 

The type of repulsion is set with the soBuffer parameter `repulsion`. 
In `get_current_gradient` the repulsion vector is added to the movement vector. 
Solely the collision avoidance vector is returned when `result = []`. 

##### Gradient based

Invoking `_gradient_repulsion` will return a vector pointing away from all neighbors calculated with the gradient formulas specified by Balch and Hybinette (2000) (see **Gradient calculation**). 
The calculation is completely based on the received gradients including the positions of the neighbors, their extent (diffusion + goal_radius) and the agents own gradient (position, repulsive radius = diffusion + goal radius). 

```python
def _gradient_repulsion(self)
```


##### Fernandez-Marquez et al. 

The formula presented by Fernandez-Marquez et al. does not consider goal and diffusion radius. 
Instead neighbors are only seen as a point. 
In this implementation, we considered the shortest distance between the two neighbors, meaning the distance between the two gradient centers minus the goal radii of both agents (can be seen as hardware size of agent). 
Similar to the gradient based repulsion version, a vector pointing away from all neighbors within view distance is returned. 
In case that two robots are at the same position, it returns a random repulsion vector which is as long as the repulsion_radius. 
 
```python
 def _repulsion_vector(self)
 ```


#### Spreading

The basic mechanism spreading is realized as a publisher - subscriber scenario in ROS. 
Every agent / soBuffer listens to the so_data topic and data can be published to this topic by all agents. 
To be able to control the spreading frequency, it is necessary to create separate nodes for each publisher. 
Currently, only a single master setup is used, but real spreading decentralisation can be reached in a multi master setup (future work). 

The data used / available for each agent in the calculations is currently restricted by the parameter `view_distance.`

More information can be found in the section **sobroadcaster.py**. 

#### Aggregation 

Aggregation is also a part of the basic mechanisms presented by Fernandez-Marquez et al. Part of the aggregation process is done in the `store_data` method. 

##### Neighbors

soBuffer keeps a certain number (`moving_storage_size`) of gradients of the neighbors. 
E.g. at least two per moving gradient are needed for flocking as the agent's velocity has to be calculated. 
The moving gradients are especially needed for repulsion, flocking and quorum sensing. Details can be found in the relevant sections. 

##### Other gradients 

soBuffer aggregates with the `aggregation` option the incoming gradient data and stores per position / within a specified aggregation radius only one gradient. 
But when requesting the `current_gradient` the stored data is aggregated to return one vector. 
With parameters `result_moving` and `result_static` can be defined which gradients (moving/static/both) will be considered in the calculation. 
There are different options available which can be set using the `result` parameter. The gradients which will be aggregated can be restricted to a set of frameIDs. 

```python
def get_current_gradient(self)
```

Options:

* **all** 

Option `all` returns a vector which is the sum of all attractive and repulsive potentials within view distance. The calculation is based on the formulas by Balch and Hybinette. 

```python
def result_all(self)
```

* **max** 

Option `max` returns a vector which lets the agent move away or move towards the maximum gradient (largest attraction). 
The maximum relative distance is considered as the potential field formulas by Blach and Hybinette return normalized attraction/repulsion values. 
To compare repulsion and attraction, the attraction value is subtracted from 1 (`1 - attraction`) as attraction decreases being closer to the gradient center.
In case that no gradient is within view distance, a zero vector will be returned. 
If the agent is within the goal radius of a repulsive gradient, a random vector leading away from the repulsive gradient is returned (length = goal radius + diffusion). 

```python
def result_max(self)
```

* **near** 

Option `near` follows the nearest attractive gradient (minimum attraction) by avoiding all repulsive gradients (non-neighbors). 
First the (relatively) nearest attractive gradient is determined and the vector to follow it calculated (nearest gradient in terms of place means gradient with smallest attraction). 
Afterwards, the repulsive vectors are calculated and summed up. 
If the agent is within the goal radius of a repulsive gradient, a random vector (length = goal radius + diffusion) leading away from the repulsive gradient will be added to the repulsion vector. 
The repulsive vector is added to the attractive vector and the sum is returned.  
 
```python
def result_near(self)
```

* **reach** 

Option `reach` returns a vector which enables the agent to follow the attractive gradient by avoiding all repulsive gradients. 
In contrast to option `near`, it always leads to reaching the gradient source (`goal_radius`) of the attractive gradient. 
The calculation is based on the enhanced formulas by Ge and Cui (1999) (see **Gradient calculation**). 

```python
def result_reach(self)
```

* **avoid** 

Option `avoid` returns a movement vector leading away from all gradients within view distance. 
Regardless of the specified `attraction` value in the soMessage, the repulsive gradient method based on Balch and Hybinette is used to calculate the movement vector and the repulsive vectors from all gradients are summed up.  

```python
def result_avoid(self)
```

* **collision**

Option `collision` returns a movement vector leading away from all repulsive gradients within view distance. 
It uses the gradient calculation based on Balch and Hybinette.

```python
def result_collision(self)
```

* **flocking and flockingrey**

Options `flocking` and `flockingrey` return a movement vector resulting in flocking behaviour. 
For more information see subsection Flocking. 


flocking(.py)
-------------

The flocking.py file contains algorithms to realize flocking in free-space (free flocking). 
It offers all formulas presented in "Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory" by Olfati-Saber (2006) for Algorithm 1.
Algorithm one consists of two parts: 

1. gradient-based term
2. velocity consensus term 

which incorporate the three flocking rules presented by Reynolds. 

Please find more information about the formulas in the paper by Olfati-Saber.

The approach leads to some problems in combination with RHBP and some enhancements might be required.

The calculations need the following input data: `Boid = collections.namedtuple('Boid', ['p', 'v'])` with `p` being the robot's current position and `v` being the robot's velocity.



flockingrey(.py)
---------------

Approach based on formulas / description by Reynold in his paper [Steering Behaviors For Autonomous Characters](www.red3d.com/cwr/steer/gdc99/).

Alignment calculation is based on heading vectors (not on velocity).

The calculation is done either on the robot position or its heading vector.
The input of the flocking methods are the agents current gradient and a list of neighbor gradients (moving and repulsive).
The current heading vector in method `alignment` is calculated using the values `q` (orientation) and `direction` of `soMessage`.
The quaternion is transformed to a transformation matrix using method `tf.transformations.quaternion_matrix(quaternion)`.
Multiplying the transformation matrix with the direction vector results in the current heading vector.


calc(.py)
---------

The file calc.py includes some basic vector (Vector3) calculations which are commonly required. These are:

* `def unit_vector(vector)` and `def unit_vector3(vector)`: returns a unit vector based on the input vector
* `def angle_between(v1, v2)`: returns the directed vector between two vectors (max. 2D at the moment)   
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


topicgradienttf(.py) and posegradienttf(.py)
--------------------------------------------

topicGradientTf offers an abstract class which can be used as a blueprint to transform data received by a topic to a gradient message and spreach it. 
A sample implementation can be found in poseGradientTf. 
There a subscription to the pose topic (geometry_msgs/pose) is made and the received data is put into a soMessage and buffered in `self._current_value`. 
The message can either be spread right away (as part of the subscription callback) or the sending is done in a ROS node with a specific frequency. 
Using the class within a ROS node leads to sending the last buffered soMessage. 