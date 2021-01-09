# CARLA C++ Client Example Code
This package illustrates how to compile and link CARLA libraries to create C++ client.

## Usage
In CARLA terminal, type 
```bash
./CarlaUE4.sh
```

Then, in a new terminal, type
```bash
rosrun cpp_client_demo cpp_client_demo
```

You will see a car spawned in CARLA GUI moving forward according to the throttle applied.

## Dependencies
This package depends on LibCarla 0.9.10.1, which can be found [here](https://github.com/lb-robotics/libcarla).

