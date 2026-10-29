# Outdoor Localization Status

The outdoor localization path needs code fixes before it can be used for
vehicle operation.

## Implementation status

`localization.launch.py` selects the outdoor path with `is_sim=False` and
`is_indoor=False`. It starts a local EKF, an optional RTK receiver, a navsat
transform and a global EKF.

Known blockers in the source:

- Navsat input remappings are unfinished.
- `rtk_manager.py` uses absolute topic names, which do not follow its `gps` namespace.
- `set_datum_node.py` has unresolved parameter-loading code.
- `_localization_debugger.launch.py` raises an unimplemented assertion.

## Receiver configuration

The standalone receiver launch and full localization launch have different defaults:

| Setting | `rtk.launch.py` | `localization.launch.py` |
| --- | --- | --- |
| Serial device | `/dev/ttyACM0` | `/dev/ttyACM1` |
| Baud rate | 250000 | 115200 |

Set both to match the receiver. The standalone launch also defaults to NTRIP
server `nrtk-swepos.lm.se`, port 80 and mountpoint `MSM_GNSS`. Supply your
correction-service credentials through local configuration; keep them out of Git.

## Topic wiring

The receiver publishes `/fix`, `/speed`, `/heading_motion`, `/heading_vehicle`,
`/heading_accuracy` and `/nmea`, and receives corrections on `/rtcm`.
Connect the NTRIP and navsat nodes to these actual topics:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /fix
ros2 topic echo /fix --once
```

After fixing the pipeline, check correction reception, fix status, covariance
and frame alignment before connecting a driving controller.

See the [localization package reference](https://github.com/KTH-SML/svea/blob/main/src/svea_localization/README.md)
for launch files and parameters.
