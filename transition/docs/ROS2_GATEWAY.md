# ROS 2 gateway v1

`deployment/ros2_gateway.py` exposes the shared Backend interface through real ROS 2 publishers/subscribers. It uses `std_msgs/msg/String` with a strictly validated JSON envelope. This deliberately avoids a custom interface package/build for v0; it does not provide ROS IDL type checking of individual JSON fields. A stable later interface should use project-specific messages. [Official String definition](https://github.com/ros2/common_interfaces/blob/jazzy/std_msgs/msg/String.msg).

The task runtime remains responsible for scheduling, dependency versions, task authority and raw decision scoring. This gateway is a local backend transport, not a second scheduler or an authorization server. Its executable accepts only `logical` and `isaac` backends. It cannot select or initialize the physical K1 SDK.

## Topics and delivery

| Topic | Direction | QoS | Payload |
|---|---|---|---|
| `/transition/command` | Into gateway | Reliable, volatile, depth 20 | Start/status/stop request |
| `/transition/event` | Out of gateway | Reliable, transient-local, depth 100 | Correlated event, error or stop acknowledgement |
| `/transition/state` | Out of gateway | Reliable, transient-local, depth 1 | Latest measured/backend Observation |

The gateway polls and publishes state on a 20 Hz timer. This is a target polling rate, not a real-time guarantee: SDK/HTTP latency can delay callbacks. Transient-local delivery helps late subscribers obtain retained events/state, but does not replace the persistent SQLite ledger. Clients must check gateway/backend boot IDs and timestamps; a cached DDS state can be stale. [Jazzy QoS API](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/qos.py).

Every outgoing envelope includes `schema_version`, `message_type`, `request_id` (null for periodic state or unparseable requests), `gateway_boot_id`, `clock_id`, `published_at`, and `backend`. Events contain the serialized `BackendEvent`; state contains the serialized `Observation`. Asynchronous events retain the original start request ID. Status queries have their own request ID.

## Strict input schema

A start request has exactly these keys:

```json
{
  "schema_version": 1,
  "request_id": "unique-request-id",
  "operation": "start",
  "clock_id": "host-boot-id-from-current-state",
  "command": {
    "command_id": "globally-unique-command-id",
    "run_id": "experiment-run-id",
    "skill_id": "point-a",
    "kind": "point",
    "parameters": {"profile": "point_a"},
    "dependency_versions": {},
    "authority_id": "runtime-authority-id",
    "authority_revision": 1,
    "issued_at": 1234.0,
    "deadline": 1254.0
  }
}
```

The shown clock values are placeholders. Clients use the same host's `time.monotonic()` and current state `clock_id`; copying the example timestamps will fail. V1 supports localhost clients on the same kernel/boot only. It rejects mismatched clock IDs rather than comparing unrelated hosts' monotonic clocks. A remote UI should communicate with the runtime through its HTTP transport; a cross-host ROS client needs a separately specified clock conversion contract.

`parameters` accepts only the named `point_a`, `point_b`, or `home` profile. Additional envelope/command fields, duplicate JSON keys, nonfinite values, wrong scalar types, arbitrary joint/pose input and messages over 16 KiB are rejected. Identifiers have bounded length. Dependency/authority revisions are nonnegative integers. Issuance may be at most 120 seconds old, and the total command window at most 120 seconds. New dispatch additionally requires an unexpired deadline and fresh, connected, quiescent backend state with no fault.

Status and stop requests replace `command` with `command_id` and use `operation: "status"` or `"stop"`. A stop must target the observed active command. Its `stop_requested` reply means only that the request was processed; query/poll for observed cancellation. Status does not create or reissue an unknown command.

## Duplicate and restart semantics

The gateway persists command payload, original request correlation and dispatch intent **before** calling `Backend.start`. It keeps the latest result/evidence. It never forwards an identical command ID twice, including after restart. A changed payload with the same command ID fails. Request IDs also bind to an exact envelope; use a fresh request ID for a new status query. Retrying an existing request ID returns its original response, which may be an acceptance receipt; it is not a live status query.

On a new request for an existing nonterminal command, the gateway queries backend history. Missing/failed history becomes `unknown` and blocks subsequent motion. It never concludes that an unobserved dispatch did not execute. A logical-backend process restart intentionally loses its own history; the gateway test demonstrates that retained intent prevents replay even in that case. An unresolved journal entry with no matching active backend command also forces non-quiescent/faulted gateway state.

Keep gateway and backend journals. Do not delete them to bypass reconciliation. A local lock prevents two gateways using the same ledger. The ledger records its backend identity and cannot be reused for a different backend. This is not network-wide ownership: never run a direct Isaac command producer and ROS gateway dispatch concurrently against the same worker.

## Isolated deployment

The verified configuration is Ubuntu 24.04, Python 3.12, ROS 2 Jazzy and `rmw_fastrtps_cpp`, with a nonzero dedicated ROS domain, localhost-only discovery, no static peers, and no discovery-server/custom DDS-profile overrides. The executable checks these settings. Discovery configuration is containment for this local test, not authentication or a security boundary against another process on the same host. [Official Jazzy discovery configuration](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Advanced/Improved-Dynamic-Discovery.rst).

From the remote project checkout:

```sh
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src"
export ROS_DOMAIN_ID=173
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
python3 deployment/ros2_gateway.py --backend logical --journal .runtime/ros-logical.sqlite
```

For Isaac, first reserve the worker as the sole command owner, then select `--backend isaac --isaac-endpoint http://127.0.0.1:8767` and a separate persistent journal. Only loopback HTTP endpoints are accepted by this executable. No ROS extension is required inside Isaac: the gateway invokes `IsaacBackend` out of process. The physical K1 remains outside this entry point.

## Verification and evidence

`tests/test_ros2_gateway.py` covers nine pure protocol cases, including malformed envelopes, correlation, restart ambiguity, duplicates and stop targeting. `deployment/test_ros2_roundtrip.py` starts a separate gateway process and a ROS client, discovers publishers, dispatches one approved profile, checks the result and state, queries status and verifies a duplicate returns the same terminal timestamp. It sets the isolated DDS environment itself and terminates only its own gateway process.

```sh
source /opt/ros/jazzy/setup.bash
python3 deployment/test_ros2_roundtrip.py --backend logical --output artifacts/ros2-logical-roundtrip.json
```

The retained `artifacts/ros2-logical-roundtrip.json` records a successful **actual Jazzy/DDS** roundtrip through the logical fixture. Its terminal evidence states `physics_validated: false`. This proves ROS transport and gateway behavior, not robot physics, human performance or physical hardware readiness. The final `artifacts/ros2-isaac-roundtrip.json` separately records a successful actual ROS 2 → HTTP adapter → native Isaac fixed-base K1 run after the source-clock correction. Its measured target error was 4.24 mm, maximum joint error 0.01150 rad, and settling dwell 0.300000016 seconds of advancing simulation time. The worker was quiescent afterward; status and duplicate requests returned the same terminal evidence without a second dispatch. All six tested source/configuration file hashes match the local files. This establishes the tested fixed-base arm pipeline only.

A preliminary native ROS→Isaac run is retained as `artifacts/ros2-isaac-roundtrip-before-source-dwell.json`. Its measured target checks passed, but it predates the correction from wall-clock to simulation-clock settling dwell. It is not the final native validation receipt. Native velocity disagreement is retained; quiescence uses explicitly labeled finite-difference position velocity from consecutive physics samples.
