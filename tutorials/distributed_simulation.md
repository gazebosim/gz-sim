\page distributedsimulation

# Distributed Simulation

## Goals

* Simulation can be distributed among 1 or more processes
* These processes can be running on the same machine or different ones
* These processes must be kept in sync
* Results can't be interpolated or missed
* We should reduce the amount of duplicate effort across processes

## High-Level design

Each ign-gazebo instance has the ability to run with the `--distributed` flag.
When the flag is present, the instance attempts to join a distributed simulation
environment by utilizing `ign-transport`. Ign-transport is used to register and
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
Performers are allocated to secondaries using a round-robin fashion. If there
are more performers than secondaries, multiple performers will be allocated to each secondary.

## Assumptions

* When executing in a distributed environment, each `ign-gazebo` instance only
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

Multiple `ign-gazebo` executables are started on the same local area network,
each with the `--distributed` flag set.

The primary instance will read several environment variables to dictate its behavior.

* **IGN_GAZEBO_NETWORK_ROLE=PRIMARY** - Dictates that the role of this
    participant is a Primary
* **IGN_GAZEBO_NETWORK_SECONDARIES=<N>** - The number of secondaries expected
    to join. Simulation will not begin until **N** secondaries have been
    discovered.

The secondary instances will only read the role environment variable

* **IGN_GAZEBO_NETWORK_ROLE=SECONDARY** - Dictates that the role of this
    participant is a Secondary

### Discovery

Once the `ign-gazebo` instance is started, it will begin a process of
discovering peers in the network. Each peer will send an announcement when it
joins or leaves the network, and also periodically sends a heartbeat.

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

After discovery, the `NetworkManager` and `SyncManager` work to distribute
performers across the network graph. Each performer specified in the SDF file
gets assigned to a network secondary. If there are more performers than
secondaries, then some secondaries will receive multiple performers.

When a secondary is assigned a performer, it is marked as active and not
static (dynamic) in the physics simulation environment. For unassigned
performers, they are treated as static objects at this point.

The primary performs no physics simulation at this point.

### Stepping

At the beginning of each simulation step, the simulation primary sends the
current iteration, step size, and simulation clock time.

Each secondary then proceeds to simulate that iteration, and sends the results
back to the simulation primary.  Each secondary then sends an additional `ack`
signal back to the simulation primary to indicate that the current iteration
is done.

Once all secondary `ack` signals have been received, the primary allows
simulation to continue to the next step.

### Interaction

All interaction with the simulation environment happens via the primary.

Play/pause and GUI functionality all work with the simulation primary instance.
