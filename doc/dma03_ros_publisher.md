# API Documentation for `dma03_ros_publisher`

## Class: `DMA03Publisher`

ROS Publisher for DMA-03 for Robot amplifier

### `__init__`()

```python
__init__(self)
```

Constructor for DMA03Publisher.

**Args:**

- `str`: Node name.


### `cleanup`()

```python
cleanup(self)
```

Cleaning up when stopping the node.


### `event_callback`()

```python
event_callback(self)
```

Method for publishing ROS Topic.

**Returns:**

- `bool`: True if executed.


### `parameter_callback`()

```python
parameter_callback(self, params)
```

Method called when a ROS Pamameter has changed.

**Args:**

- `params` (list[Parameter]): List of ROS Parameter(s).

**Returns:**

- `SetParametersResult`: Result of setting Parameter.


### `main`()

```python
main(args=None)
```

Main routine with DMA03Publisher.


