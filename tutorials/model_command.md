\page model_command Model Command

## Overview
`gz model` command allows you to get information about the models for a given running Gazebo simulation.

For each model, it is possible to get information about its
 -  Pose: Pose of the model
 -  Links: Pose, mass, and inertial matrix of the link and attached sensors
 -  Joints: Parent link, child link and joint type.

## Example running the diff_drive world

To try out this command we need first a running simulation. Let's load the `diff_drive` example world. In a terminal, run:

    gz sim diff_drive.sdf

Once Gazebo is up, we can use the gz model command to get information of the simulation.
Open a new terminal and enter:

    gz model --list

And available models should be printed:

    Available models:
        - ground_plane
        - vehicle_blue
        - vehicle_green

Once you get the name of the model you want to see, you may run the following commands to get its properties.

`gz model -m <model_name>` to get the **complete information of the model**. e.g.

    gz model -m vehicle_blue

```
			Requesting state for world [diff_drive]...

			Model: [8]
			- Name: vehicle_blue
			- Pose [ XYZ (m) ] [ RPY (rad) ]:
					[0.000000 | 2.000000 | 0.325000]
					[0.000000 | 0.000000 | 0.000000]

			- Link [9]
					- Name: chassis
					- Parent: vehicle_blue [8]
					- Mass (kg): [1.143950]
					- Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:
    						[0.000000 0.000000 0.000000]
    						[0.000000 -0.000000 0.000000]
					- Inertial Matrix (kg⋅m^2):
							[0.126164 | 0.000000 | 0.000000]
							[0.000000 | 0.416519 | 0.000000]
							[0.000000 | 0.000000 | 0.481014]
					- Pose [ XYZ (m) ] [ RPY (rad) ]:
							[-0.151427 | 0.000000 | 0.175000]
							[0.000000 | 0.000000 | 0.000000]
			- Link [12]
					- Name: left_wheel
					- Parent: vehicle_blue [8]
					- Mass (kg): [2.000000]
					- Inertial Pose:
							[0.000000 | 0.000000 | 0.000000]
					- Inertial Matrix (kg⋅m^2):
							[0.145833 | 0.000000 | 0.000000]
							[0.000000 | 0.145833 | 0.000000]
							[0.000000 | 0.000000 | 0.125000]
					- Pose [ XYZ (m) ] [ RPY (rad) ]:
							[0.554283 | 0.625029 | -0.025000]
							[-1.570700 | 0.000000 | 0.000000]
			- Link [15]
					- Name: right_wheel
					- Parent: vehicle_blue [8]
					- Mass (kg): [2.000000]
					- Inertial Pose:
							[0.000000 | 0.000000 | 0.000000]
					- Inertial Matrix (kg⋅m^2):
							[0.145833 | 0.000000 | 0.000000]
							[0.000000 | 0.145833 | 0.000000]
							[0.000000 | 0.000000 | 0.125000]
					- Pose [ XYZ (m) ] [ RPY (rad) ]:
							[0.554282 | -0.625029 | -0.025000]
							[-1.570700 | 0.000000 | 0.000000]
			- Link [18]
					- Name: caster
					- Parent: vehicle_blue [8]
					- Mass (kg): [1.000000]
					- Inertial Pose:
							[0.000000 | 0.000000 | 0.000000]
					- Inertial Matrix (kg⋅m^2):
							[0.100000 | 0.000000 | 0.000000]
							[0.000000 | 0.100000 | 0.000000]
							[0.000000 | 0.000000 | 0.100000]
					- Pose [ XYZ (m) ] [ RPY (rad) ]:
							[-0.957138 | 0.000000 | -0.125000]
							[0.000000 | 0.000000 | 0.000000]
			- Joint [21]
					- Name: left_wheel_joint
					- Parent: vehicle_blue [8]
					- Type:  revolute
					- Parent Link: left_wheel
					- Child Link:  chassis
					- Pose [ XYZ (m) ] [ RPY (rad) ]:
					[0.000000 | 0.000000 | 0.000000]
					[0.000000 | 0.000000 | 0.000000]
					- Axis position [ XYZ ]:
					[0 | 0 | 1]
			- Joint [22]
					- Name: right_wheel_joint
					- Parent: vehicle_blue [8]
					- Type:  revolute
					- Parent Link: right_wheel
					- Child Link:  chassis
					- Pose [ XYZ (m) ] [ RPY (rad) ]:
					[0.000000 | 0.000000 | 0.000000]
					[0.000000 | 0.000000 | 0.000000]
					- Axis position [ XYZ ]:
					[0 | 0 | 1]
			- Joint [23]
					- Name: caster_wheel
					- Parent: vehicle_blue [8]
					- Type: ball
					- Parent Link: chassis [9]
					- Child Link: caster [18]
					- Pose [ XYZ (m) ] [ RPY (rad) ]:
						[0.000000 0.000000 0.000000]
						[0.000000 -0.000000 0.000000]

```


`gz model -m <model_name> --pose` to get the **pose** information. e.g.

    gz model -m vehicle_blue --pose


```
      Requesting state for world [diff_drive]...

			Model: [8]
				- Name: vehicle_blue
				- Pose [ XYZ (m) ] [ RPY (rad) ]:
            [0.000000 | 2.000000 | 0.325000]
            [0.000000 | 0.000000 | 0.000000]
```


To get the information of **all the model links** enter

    gz model -m <model_name> --link


Or you can get the information of a **single link** by adding the name as argument. e.g.

    gz model -m vehicle_blue --link caster

```
      Requesting state for world [diff_drive]...

				- Link [18]
					- Name: caster
					- Parent: vehicle_blue [8]
					- Mass (kg): [1.000000]
					- Inertial Pose:
							[0.000000 | 0.000000 | 0.000000]
					- Inertial Matrix (kg⋅m^2):
							[0.100000 | 0.000000 | 0.000000]
							[0.000000 | 0.100000 | 0.000000]
							[0.000000 | 0.000000 | 0.100000]
					- Pose [ XYZ (m) ] [ RPY (rad) ]:
							[-0.957138 | 0.000000 | -0.125000]
							[0.000000 | -0.000000 | 0.000000]
```


To get the information of **all the model joints** enter

    gz model -m <model_name> --joint

Or you can get the information of a **single joint** by adding the name as argument. e.g.

    gz model -m vehicle_blue --joint caster_wheel

```
      Requesting state for world [diff_drive]...

				- Joint [23]
					- Name: caster_wheel
					- Parent: vehicle_blue [8]
					- Type:  ball
					- Parent Link: caster
					- Child Link:  chassis
					- Pose [ XYZ (m) ] [ RPY (rad) ]:
						[0.000000 | 0.000000 | 0.000000]
						[0.000000 | -0.000000 | 0.000000]
```
