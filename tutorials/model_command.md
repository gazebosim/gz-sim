\page model_command Model Command

## Overview
This command allows you to get the following information about models in the running world:

- Model name.
- Model pose.
- Model links.
  - Link name.
  - Link parent.
  - Link mass.
  - Link inertial matrix.
  - Link pose.
- Model joints.
  - Joint name.
  - Joint parent.
  - Joint parent link.
  - Joint child link.

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

After having an Ignition Gazebo server running with the diff_drive world, you may check the available models running `ign model --list`

<details>
  <summary>Click to see output of `ign model --list`</summary>

```
      Requesting state for world [diff_drive] on service [/world/diff_drive/state]...
      
      Available models:
          - ground_plane
          - vehicle_blue
          - vehicle_green
```
</details>

Once you get the name of the model you want to see, you may run the following commands to get its properties.

`ign model -m arg` to get the full information of the model. e.g. `ign model -m vehicle_blue`
<details>
  <summary>Click to see output of `ign model -m vehicle_blue`</summary>

```
      Requesting state for world [diff_drive] on service [/world/diff_drive/state]...

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
</details>


`ign model -m arg --pose` to get the pose information. e.g. `ign model -m vehicle_blue --pose`

<details>
  <summary>Click to see output of `ign model -m vehicle_blue --pose`</summary>

```
      Requesting state for world [diff_drive] on service [/world/diff_drive/state]...

      Name: vehicle_blue
        - Pose: 
            [0.000000 | 2.000000 | 0.325000]
            [0.000000 | 0.000000 | 0.000000]
```
</details>

`ign model -m arg --link` to get the links information. e.g. `ign model -m vehicle_blue --link`

<details>
  <summary>Click to see output of `ign model -m vehicle_blue --link`</summary>

```
      Requesting state for world [diff_drive] on service [/world/diff_drive/state]...

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
```
</details>

`ign model -m arg --joint` to get the joints information. e.g. `ign model -m vehicle_blue --joint`

<details>
  <summary>Click to see output of `ign model -m vehicle_blue --joint`</summary>

```
      Requesting state for world [diff_drive] on service [/world/diff_drive/state]...

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
</details>

`ign model -m arg --link arg` to get a certain link information. e.g. `ign model -m vehicle_blue --link`

<details>
  <summary>Click to see output of `ign model -m vehicle_blue --link`</summary>

```
      Requesting state for world [diff_drive] on service [/world/diff_drive/state]...

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
</details>

`ign model -m arg --joint arg` to get a certain joint information. e.g. `ign model -m vehicle_blue --joint caster_wheel`

<details>
  <summary>Click to see output `ign model -m vehicle_blue --joint caster_wheel`</summary>

```
      Requesting state for world [diff_drive] on service [/world/diff_drive/state]...
      
        - Joint [23]
          - Name: caster_wheel
          - Parent: vehicle_blue [8]
          - Joint type:  ball
          - Parent Link: [caster]
          - Child Link:  [chassis]
```
</details>

