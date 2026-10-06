# September 24 read-only K1 findings and remaining S2 evidence

These are engineering observations reported by the coordinating agent during
the authorized SSH/ROS inspection, not a passed S2 commissioning stage. The K1
was reachable through MLWorkstation; an installed legacy SDK was inspected.
The retained commissioning record does not yet establish its approved device
identity/firmware/profile/controller combination.

Reported diagnostic counts:

- The broad read-only ROS probe received **7,412 joint messages in 15 seconds**.
- The robot-local camera probe received **201 native camera messages in 20 seconds**.
- The corresponding direct MLWorkstation native-image probe received **zero
  images**. The derived camera bridge was still pending in this observation
  record; later bridge results belong in a separate receipt.

These counts establish observed delivery in those probes. They do not establish
source acquisition age, sensor-clock synchronization, absence of buffered old
packets, cross-sensor coherence, a guaranteed frequency or approved physical
motion. The raw probe captures must be retained and linked before treating the
numbers as a complete acceptance artifact. No full S2 predicate is marked passed
merely from the counts.

The source-only observation bridge and robot-side camera compressor have no
vendor SDK movement, RPC client or command-subscription route. Code inspection
supports that bounded capability claim; it does not establish exclusive ownership
against other applications already running on the robot.

Unresolved evidence includes exact commissioned identity/mode, joint/frame
validation, acquisition/buffering/skew bounds, velocity and transform uncertainty,
motor-health interpretation, physical support, registered target/profile geometry,
owner/watchdog enforcement and independent stop measurements. These gaps keep
physical task dispatch inhibited. S2 observation qualification remains blocked;
S4/S5 physical commissioning has not been performed.

The example `examples/stages/s2-observation-preflight.json` records one prospective
S2 acceptance target and one **preflight refusal**, with zero qualified acceptance
attempts. The exploratory probes above are not retroactively labelled calibrated
S2 trials. Missing raw/provenance records remain explicit required artifacts.
