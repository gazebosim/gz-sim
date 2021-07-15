\page model_command Model Command

## Overview
`ign model` command allows you to get information about the models for a given running `ignition` simulation.

For each model, it is possible to get information about its
 -  Pose: Pose of the model
 -  Links: Pose, mass, and inertial matrix of the link
 -  Joints: Parent link, child link and joint type.

### Available options for the model command

```
  --list                     Get a list of the available models.        
  -m [--model] arg           Select the model to be shown.              
  -l [--link]  arg           Select a link to show its properties.      
                             If no arg is passed all links are printed  
                             Requires the -m option                     
                                                                        
  -j [--joint] arg           Select a joint to show its properties.     
                             If no arg is passed all joints are printed 
                             Requires the -m option                     

  -h [ --help ]              Print the help message.
```


## Example running the diff_drive world

To try out this command we need first a running simulation. Let's load the diff_drive ignition simulation. In a terminal, run:

    ign diff_drive.sdf

Once Ignition Gazebo is up, we can use the ign model command to get information of the simulation.
Open a new terminal and enter:

    ign model --list

And available models should be printed:


    Available models:
        - ground_plane
        - vehicle_blue
        - vehicle_green

Once you get the name of the model you want to see, you may run the following commands to get its properties.

`ign model -m <model_name>` to get the **complete information of the model**. e.g. 

    ign model -m vehicle_blue

```
      Requesting state for world [diff_drive]...

      Name: vehicle_blue
        - Pose: 
            [0.000000 | 2.000000 | 0.325000]
            [0.000000 | 0.000000 | 0.000000]
      
        - Link [9]
          - Name: chassis
          - Parent: vehicle_blue [8]
          - Mass: [1.143950]
          - Inertial Matrix: 
              [0.126164 | 0.000000 | 0.000000]
              [0.000000 | 0.416519 | 0.000000]
              [0.000000 | 0.000000 | 0.481014]
          - Pose: 
              [-0.151427 | 0.000000 | 0.175000]
              [0.000000 | 0.000000 | 0.000000]
        - Link [12]
          - Name: left_wheel
          - Parent: vehicle_blue [8]
          - Mass: [2.000000]
          - Inertial Matrix: 
              [0.145833 | 0.000000 | 0.000000]
              [0.000000 | 0.145833 | 0.000000]
              [0.000000 | 0.000000 | 0.125000]
          - Pose: 
              [0.554283 | 0.625029 | -0.025000]
              [-1.570700 | 0.000000 | 0.000000]
        - Link [15]
          - Name: right_wheel
          - Parent: vehicle_blue [8]
          - Mass: [2.000000]
          - Inertial Matrix: 
              [0.145833 | 0.000000 | 0.000000]
              [0.000000 | 0.145833 | 0.000000]
              [0.000000 | 0.000000 | 0.125000]
          - Pose: 
              [0.554282 | -0.625029 | -0.025000]
              [-1.570700 | 0.000000 | 0.000000]
        - Link [18]
          - Name: caster
          - Parent: vehicle_blue [8]
          - Mass: [1.000000]
          - Inertial Matrix: 
              [0.100000 | 0.000000 | 0.000000]
              [0.000000 | 0.100000 | 0.000000]
              [0.000000 | 0.000000 | 0.100000]
          - Pose: 
              [-0.957138 | 0.000000 | -0.125000]
              [0.000000 | 0.000000 | 0.000000]
        - Joint [21]
          - Name: left_wheel_joint
          - Parent: vehicle_blue [8]
          - Joint type:  revolute
          - Parent Link: [left_wheel]
          - Child Link:  [chassis]
        - Joint [22]
          - Name: right_wheel_joint
          - Parent: vehicle_blue [8]
          - Joint type:  revolute
          - Parent Link: [right_wheel]
          - Child Link:  [chassis]
        - Joint [23]
          - Name: caster_wheel
          - Parent: vehicle_blue [8]
          - Joint type:  ball
          - Parent Link: [caster]
          - Child Link:  [chassis]
```


`ign model -m <model_name> --pose` to get the **pose** information. e.g. 

    ign model -m vehicle_blue --pose


```
      Requesting state for world [diff_drive]...

      Name: vehicle_blue
        - Pose: 
            [0.000000 | 2.000000 | 0.325000]
            [0.000000 | 0.000000 | 0.000000]
```


To get the information of **all the model links** enter 

    ign model -m <model_name> --link


Or you can get the information of a **single link** by adding the name as argument. e.g. 

    ign model -m vehicle_blue --link caster

```
      Requesting state for world [diff_drive]...

        - Link [18]
          - Name: caster
          - Parent: vehicle_blue [8]
          - Mass: [1.000000]
          - Inertial Matrix: 
              [0.100000 | 0.000000 | 0.000000]
              [0.000000 | 0.100000 | 0.000000]
              [0.000000 | 0.000000 | 0.100000]
          - Pose: 
              [-0.957138 | 0.000000 | -0.125000]
              [0.000000 | 0.000000 | 0.000000]
```


To get the information of **all the model joints** enter 

    ign model -m <model_name> --joint

Or you can get the information of a **single joint** by adding the name as argument. e.g. 

    ign model -m vehicle_blue --joint caster_wheel

```
      Requesting state for world [diff_drive]...
      
        - Joint [23]
          - Name: caster_wheel
          - Parent: vehicle_blue [8]
          - Joint type:  ball
          - Parent Link: [caster]
          - Child Link:  [chassis]
```
