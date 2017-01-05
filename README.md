SO_DATA MANUAL
===============

The package so_data provides components which are required to implement self-organization. It provides functionality to send data to the soData topic as well as to aggregate, store and use it for 
calculations. Unit tests were done for the core components to ensure correctness of mathematical computations. All methods work for up to three dimensions; with minor enhancements of the code it can be used 
for more dimensions. 

Package components
-------------------

The package consists of the following components:

* **calc.py**: provides helper functions which are used by several components of the package (+ unit test)
* **flocking.py**: provides methods to calculate the flocking vector based on the paper of Olfati-Saber (+ unit test) 
* **soBroadcaster.py**: allows to publish data to the soData topic which will be subscribed to in the soBuffer 
* **soBuffer.py**: implements a layer which allows to store gradient data and to do calculations necessary for self-organizing behaviour (+ unit test)  
* **gradientSensor.py**: implements a complex version of the Simple Topic Sensor, the Gradient Sensor
* **soMessage.msg**: message file describing gradients 


soMessage
---------

soMessages are used to specify gradients: either in environment or as neighbour data. The specification of the soMessage data can lead to varying behaviour. The specification of frameIDs can help to
 assign gradients to specific tasks / behaviours, but some frameIDs are reserved for special purposes. 

###### elements:

* **Header header**: standard ROS header 
  * uint32 seq: consecutively increasing ID; set by system
  * time stamp: sec + nsec - has to be set using ros Time 
  * string frame_id: to associate data for a purpose 
* **geometry_msgs/Vector3 p**: gradient center 
* **int8 attraction**: attractive `1` or repulsive `-1` gradient 
* **float32 diffusion**: radius in which information will be spread
* **float32 goal_radius**: area with minimum attraction or maximum repulsion; total reach of gradient = diffusion + goal_radius 
* **float32 ev_factor**: evaporation factor `[0,1]` applied on diffusion (`diffusion *= ev_factor`) after `ev_time`; `1` = no evaporation, `0` = data loss after `ev_time`
* **float32 ev_time**: delta time `>= 0` in which evaporation is applied 
* **float32 angle**: gradient sector size
* **geometry_msgs/Vector3 direction**: gradient sector direction
* **diagnostic_msgs/KeyValue[]** payload: array of key-value-pairs to store payload data 


###### reserved frameIDs:
* **'robot' + ID**: frameID indicating that gradient of another robot is received, ID is usually a number 
* **'DEFAULT'**: value is used to specify aggregation option (store_data in soBuffer) for frameIDs having no specific option assigned to 
* **'None'**: value is assigned to all gradient messages which have no frameID when received 



soBuffer(.py)
-------------

The central component of the so_data package is the soBuffer. It subscribes to the soData topic and stores all received gradients in an aggregated manner. Furthermore, it includes several calculation options 
which are based on gradients. Behaviours can use the soBuffer to implement self-organizing behaviour. The Gradient Sensor uses an instance of the soBuffer too, to enable activation calculation based on 
gradient attraction/repulsion. Various parameters allow to adjust the soBuffer to the needs of a self-organization pattern. 

### soBuffer Parameters 

The soBuffer provides several parameters which can be set to adjust the data storage and calculation behaviour. The following parameters can be set (default values specified in brackets):

* **aggregation** ({'DEFAULT': 'max'}): dictionary (key: frameID, value: aggregation option) of aggregation type per frameID. 'DEFAULT' is used for all frameIDs which have no option specified in 
the dictionary and has to be specified. Does not affect storage of neighbor gradients. Aggregation options:
  * **min** = keep gradient with minimum diffusion radius + goal radius (at a position / within aggregation_distance)
  * **max** = keep gradient with maximum diffusion radius + goal radius(at a position / within aggregation_distance)
  * **avg** = keep average gradient (at a position / within aggregation_distance) 
  * **newest** = keep newest / last received gradient (at a position / within aggregation_distance) 
* **min_diffusion** (0.1): float; threshold value specifying minimum diffusion radius gradient has to have when goal_radius == 0.0 to be stored in soBuffer
* **view_distance** (2.0): float; radius in which agent can sense gradients. Should be >= goal_radius specified in agent's own gradient.  
* **id** (''): 'robotX' with X = robot's id; frameID's in this form are considered as robot position data / neighbor gradients. Gradients with own id are stored in self._ownpos 
* **result** (''): specifies how soBuffer data (within agent's view) is aggregated s.t. one value is returned. Options:
  * **all** = movement vector considering all vectors of the potential field will be returned 
  * **max** = movement vector based on maximum repulsion / attraction (goal+diffusion) will be returned 
  * **near** = movement vector following nearest attractive gradient by avoiding repulsive gradients will be returned; robot might not reach gradient source  
  * **reach** = movement vector following nearest attractive gradient by avoiding repulsive gradients will be returned; allows to reach gradient source in comparison to 'near' 
  * **avoid** = movement vector leading away from all sensed gradients will be returned 
* **collision_avoidance** ('repulsion'): collision avoidance between neighbors / agents. Options:
  * **gradient** = gradient/potential field approach is used to calculate repulsion vector (formulas of 'reach' option of result)
  * **repulsion** = repulsion vector is calculated based on formula presented in Fernandez-Marquez et al.
* **store_neighbors** (True): bool; defines whether neighbor gradients / robot position data should be stored 
* **neighbor_storage_size** (2): int; defines number of gradients which will be stored for robot position data / neighbor gradients. 
* **framestorage** ([]): array listing all frameIDs which should be stored. Empty array leads to storage of all frameIDs. 'robot' leads to storing no gradients. 
* **aggregation_distance** (1.0): radius in which gradient data is aggregated (see aggregation) 


### Gradient Storage

One key part of the buffer is the storage of incoming gradients. `store_data` applies evaporation (see **Evaporation**), stores neighbor gradient data in `self._neighbors`, the agent's own position in `self._ownpos` and
  all other frameIDs in `self._data`. 

```python
def store_data(self, msg)
```

All gradients are received via the `soData` topic. Each buffer subscribes to this topic when initialized and defines `store_data` as the subscription callback. 

```python
rospy.Subscriber('soData', soMessage, self.store_data)
```

`store_data` will then care about storing the data as specified with the soBuffer parameters `aggregation`, `min_diffusion`, `id`, `store_neighbors`, `neighbor_storage_size`, `framestorage` and `aggregation_distance`. 
`aggregation` is a dictionary which allows to specify based on frameIDs how the data should be stored. The key `DEFAULT` specifies the aggregation option for all frames for which no aggregation option is 
 defined in `aggregation`. Options are `min`, `max`, `avg` which store the gradient with maximum / minimum diffusion and goal radius and respectively the average diffusion and goal radius as well the averaged 
 gradient center (soMessage.p). The option`newest` stores the last received gradient. All options might be suitable for movement related gradients while `newest` is most appropriate for gradients including payload used for decision making. The storing mechanisms allows to store only gradients with certain frameIDs. 
 In case that **only** neighbor gradients should be stored, `'robot'` should be specified as the only frameID in `framestorage`. 
 
 The calculations of the methods which return values used in behaviours or sensors are based on the stored data.  

### Methods for use in behaviours and sensors 

The following methods can be used to calculate information necessary for behaviours and sensors. 

##### Agent Density Function 

returns True / False based on the quantity of agents within view distance is over / below threshold. Depends solely on agent gradients (frameID: 'robotX'). 
Can be used to implement Quorum Sensing. Parameter: `threshold` which has to be reached.

```python
def quorum(self, threshold)
```

##### Flocking

```python
def 
```

##### Goal Achievement

returns True / False based on whether the gradient source was reached or not ('normalized' attraction == 0). E.g. to be used in sensors to bind activation on achievement of goal (e.g. with Boolean Activator). 
 Parameters: current `pose` of the agent, `frameids` specifying which gradients should be considered in the calculation (optional). 

```python
def get_goal_reached(self, pose, frameids=[])
```


### Gradient calculation 

The soBuffer includes two different approaches of calculating the attraction/repulsion of gradients. The first approach follows the approach presented in "Social potentials for scalable multi-robot formations"
by Balch and Hybinette (2000). The second approach is based on "New Potential Functions for Mobile Robot Path Planning" by Ge and Cui (1999) which was enhanced with an inner goal_radius which leads to 
infinite repulsion. 

##### Balch and Hybinette (2000)

The paper of Balch and Hybinette includes formulas to calculate attraction and repulsion of gradients. Attraction values are within `[0,1]` while repulsion values are within `[0, inf.]`. Attraction and 
   repulsion can be combined to generate the movement vector (see [TODO]). In some scenarious the attractive gradient might not be reached as it attraction and repulsion lead to a zero potential value
   at a point not being the attractive gradient source. 
   Parameters: `gradient` is a soMessage with the attractive/repulsive gradient data, `pose` is the current position of the agent. 

```python
def _calc_attractive_gradient(gradient, pose)

def _calc_repulsive_gradient(gradient, pose)
```

##### Ge and Cui (1999)

In comparison to the approach by Balch and Hybinette, Ge and Cui guarantee with their approach, that the attractive gradient source is reached. The implementation of the attractive gradient is similar to
Balch and Hybinette, the only difference is that Balch and Hybinette return normalized gradient values while Ge and Cui return absolute values. Therewith, to determine the closest attractive gradient 
the method based on Balch and Hybinette can be used. Ge and Cui enhanced the repulsive gradient calculation to ensure that the gradient source can be reached. 


```python 
def _calc_attractive_gradient_ge(gradient, pose)

def _calc_repulsive_gradient_ge(gradient, goal, pose)
```


### Basic Mechanisms

##### Evaporation 

Evaporation is one of the basic mechanisms presented in the paper by Fernandez-Marquez et al. It is applied both before storing received data as well as when requesting the current gradient to follow (get_current_gradient). 
To ensure that all data is up-to-date, each received message is evaporated before it is stored using `_evaporate_msg(msg)`. Parameters: `msg` is a soMessage. 
It returns the evaporated message or `None` in cases where the diffusion radius is smaller than the specified minimum diffusion and the goal radius of the gradient is zero. 
The whole buffer (self._data) can be evaporated using `_evaporate_buffer()`.

```python 
def _evaporate_msg(self, msg)

def _evaporate_buffer(self)
```

The evaporation frequency (or rather delta t) is specified in the soMessage as `ev_time` and the evaporation factor as `ev_factor`. Therewith, evaporation can be specified per gradient. `ev_time` has to be larger or equal zero. `ev_factor` has to be set in 
the interval `[0,1]` with `1` leading to no evaporation and `0` to the complete evaporation after `ev_time`. In case that `ev_time == 0` and `ev_factor < 1` the gradient will instantly evaporate completely. 

Gradients without goal radius (soMessage `goal_radius = 0`) and a diffusion smaller than the minimum diffusion radius (`min_diffusion`, see soBuffer Parameters) will be deleted immediately. 
 
 
##### Repulsion

Repulsion is specified as a basic mechanism as well. It leads to avoiding collision between agents and can help to distribute the agents uniformly in a specific area. Fernandez-Marquez et al. provide a formula to calculate a repulsive vector in their paper, but it could be calculated on basis of repulsive gradients too. 
In the buffer are both options implemented. The repulsion vector depends on the parameters `repulsion_radius` which is defined as the `goal_radius + diffusion <= view_distance` of the agent (`self._own_pos`) 
and `view_distance` which is set as a parameter of the buffer and should be `>= goal_radius` of the agent. 

###### Gradient based

Invoking `_gradient_repulsion` will return a vector pointing away from all neighbors calculated with the gradient formulas specified by Balch and Hybinette (2000) (see **Gradient calculation**). 
The calculation is completely based on the received gradients including the positions of the neighbors, their extent (diffusion + goal_radius) and the agents own gradient 
(position, repulsive radius = diffusion + goal radius). 

```python
def _gradient_repulsion(self)
```


###### Fernandez-Marquez et al. 

The formula presented by Fernandez-Marquez et al. does not consider goal and diffusion radius. Instead neighbors are only seen as a point. Similar to the gradient based repulsion version, a vector pointing
 away from all neighbors within view distance is returned. In case that two robots are at the same position, it returns a random repulsion vector which is as long as the repulsion_radius. 
 
```python
 def _repulsion_vector(self)
 ```


flocking(.py)
-------------

The flocking.py file contains algorithms to realize flocking in free-sprace (free flocking). 

1. gradient-based term
2. velocity consensus term 
