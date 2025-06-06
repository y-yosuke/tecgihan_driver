# API Documentation for `force_to_wrench`

## Class: `ForceToWrench`

ROS Node for converting Vector3Stamped to WrenchStamped ROS Topic

### `__init__`()

```python
__init__(self)
```

Constructor for ForceToWrench.


### `listener_callback`()

```python
listener_callback(self, msg: geometry_msgs.msg._vector3_stamped.Vector3Stamped)
```

Method called when ROS Topic `/dma03_publisher/force` is published.

**Args:**

- `msg` (Vector3Stamped): ROS Topic Data


### `main`()

```python
main(args=None)
```

Main routine with ForceToWrench.


