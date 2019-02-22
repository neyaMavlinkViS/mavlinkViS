AltitudeMSLTuple altitude  # The current optional altitude mean-sea-level of the unmanned platform.
AltitudeAGLTuple altitude_agl  # The current optional altitude above-ground-level of the unmanned platform.
float64 altitude_asf   # The current optional altitude above-sea-floor of the unmanned platform.
Orientation3DMeasurement attitude  # The current orientation (roll, pitch, yaw) of the unmanned platform.
RMSOrientationErrorType attitude_rms  # The current RMS value of the validity of the orientation data.
float64 depth   # The current optional depth of the maritime unmanned plaform.
float64 heading   # The current optional heading of the unmanned platform.
NavigationSolutionEnumTypeLDM navigation_solution  # The desired navigation solution (estimated or measured)
Position3DWGS84Tuple position  # The current position (latitude, longitude) of the unmanned platform
string session_id   # sessionID identifier for one or a series of related message exchanges. The sessionID value in response messages should be set to the same value as the sessionID for the corresponding request messages. This attribute may be empty in published response messages that are sent based on subscriptions to the response message type
string source_subsystem_id   # The unique identifier of the originating source sub-system (e.g. engine, payload, sensor, etc.) of the status interface
string source_system_id   # The unique identifier of the originating source (e.g. vehicle) of the status interface
string target_subsystem_id   # The unique identifier of the destination (e.g. a client, an application, etc.) of the status interface
string target_system_id   # The unique identifier of the destination (e.g. control station) of the status interface
float64 time_stamp   # timeStamp is the time at which the InterfaceSetType is sent.
RMSPositionErrorType xy_position_rms  # The current RMS value of the validity of the position data.
RMSDistanceErrorType z_position_rms  # The current RMS value of the validity of the position data.
