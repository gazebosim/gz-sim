\page distributedsimulation Distributed Simulation

## Goals

* Simulation can be distributed among 1 or more processes
* These processes can be running on the same machine or different ones
* These processes must be kept in sync
* Results can't be interpolated or missed
* We should reduce the amount of duplicate effort across processes

## High-Level design

Each `gz sim` instance has the ability to run with the `--network-role` flag.
When the flag is present, the instance attempts to join a distributed simulation
environment by utilizing `gz-transport`. gz-transport is used to register and
track available peers, as well as synchronize clock and state among multiple
distributed environment participants.

Distributed environment participants can take one of the following roles.

* Primary - responsible for distributing work and synchronizing clocks among the
            other participants
* Secondary - responsible for receiving work from the primary instance and
              executing physics and sensor simulation.  Results are reported
              back to the Primary.

The distribution of simulation work utilizes the concept of performers in
order to set where physics simulation will occur. A performer is an additional
annotation in an SDF file which marks each model that will be a performer.
There can be 0 to N performers being simulated at an instance at a time.
Each level can only be simulated by one secondary at a time, so performers
in the same level are always in the same instance. If there are more levels
active than instances, multiple levels will be allocated to each secondary.

## Assumptions

* When executing in a distributed environment, each `gz sim` instance only
  has one `SimulationRunner` instance, which means that instance is incapable
  of simulating multiple worlds.

* Distributed lockstep - all simulation runners step at the same time. If a
  particular instance is running slower than the rest, it will have an
  impact on the total simulation throughput.

* Fixed runners - all simulation runners have to be defined ahead of time.
  If a runner joins or leaves the graph after simulation has started, simulation
  will terminate.

## Execution flow

### Configuration and launch

Multiple `gz sim` executables are started on the same local area network,
each with the `--distributed` flag set.

#### Command line options

The primary instance will read several command line options to dictate its behavior.

* **--network-role=primary** - Dictates that the role of this
    participant is a Primary. Capitalization of "primary" is not important.
* **--network-secondaries=`<N>`** - The number of secondaries expected
    to join. Simulation will not begin until **N** secondaries have been
    discovered.

The secondary instances will only read the role command line option

* **--network-role=secondary** - Dictates that the role of this
    participant is a Secondary. Capitalization of "secondary" is not important.

### Discovery

Once the `gz sim` instance is started, it will begin a process of
discovering peers in the network. Each peer will send an announcement in
the `/announce` topic when it joins or leaves the network, and also
periodically sends a heartbeat on `/heartbeat`.

Simulation is allowed to begin once each secondary has discovered the
primary and the primary has discovered the correct number of secondaries.

If at any time the primary or any secondaries leave the network, either
intentionally (through shutdown) or unintentionally (segfault or network
issues), then the rest of the simulation graph will get a signal and shut down
safely.

There are two possible signals that can be received. The first is an intentional
announcement from a network peer that it is shutting down. The second is when
a peer fails to receive a heartbeat from another peer after a specified
duration. Both of these signals will cause the termination of the simulation.

### Distribution

After discovery, the `NetworkManager` works on the initial distribution of
performers across the network graph according to the levels they're in. If
there are more performers than secondaries, then some secondaries will receive
multiple performers. On the other hand, if performers are located in less
levels than secondaries, some secondaries will be left idle.

As simulation proceeds and performers move across levels, their affinities will
be updated as part of the message sent on the `/step` topic in order to
avoid duplicate levels across secondaries. The primary, on the other hand,
keeps all performers loaded, but performs no physics simulation.

### Stepping

Stepping happens in 2 stages: the primary update and the secondaries update,
according to the diagram below:

<img src="https://raw.githubusercontent.com/gazebosim/gz-sim/main/tutorials/files/distributed_step.png"/>

1. The primary publishes a `SimulationStep` message on the `/step` topic,
containing:

    * The current sim time, iteration, step size and paused state.
    * The latest secondary-to-performer affinity changes.
    * **Upcoming**: The updated state of all performers which are changing secondaries.

2. Each secondary receives the step message, and:

    * Loads / unloads performers according to the received affinities
    * Runs one simulation update iteration
    * Then publishes its updated  performer states on the `/step_ack` topic.

3. The primary waits until it gets step acks from all secondaries.

4. The primary runs a step update:

    * Update its state with the states received from secondaries.
    * The `LevelManager` checks for level changes according to these new states
    * The `SceneBroadcaster` plugin publishes an updated scene to the GUI
      for any level changes.

5. The primary initiates a new iteration.

### Interaction

All interaction with the simulation environment should happen via the same
topics that are used for non-distributed simulation, which should all be
provided by the primary. Therefore, play/pause and GUI functionality all
interact with the simulation primary instance, which in turn propagates the
commands to the secondaries.
