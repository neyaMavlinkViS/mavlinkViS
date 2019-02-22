AltitudeMSLTuple altitude  # The optional desired altitude used for the unmanned air platform.
AltitudeAGLTuple altitude_agl  # The optional desired altitude used for the unmanned air platform.
float64 altitude_asf   # The optional desired altitude used for the unmanned air platform.
Orientation3DPlatformXYZMeasurement attitude  # The desired orientation (roll, pitch, yaw) of the unmanned platform.
RMSOrientationErrorType attitude_rms  # The desired RMS value of the validity of the orientation data.
float64 depth   # The optional desired depth used for the unmanned maritime platform.
Position3DWGS84Tuple position  # The desired position (latitude, longitude) of the unmanned platform
string session_id   # sessionID identifier for one or a series of related message exchanges. The sessionID value in response messages should be set to the same value as the sessionID for the corresponding request messages. This attribute may be empty in published response messages that are sent based on subscriptions to the response message type
string source_subsystem_id   # The unique identifier of the originating subsystem (e.g. a client, operator, etc.) of the commanding interface
string source_system_id   # The unique identifier of the originating source (e.g. control station) of the commanding interface
string target_subsystem_id   # The unique identifier of the destination subsystems (e.g. engine, payload, sensor, etc.) of the commanding interface
string target_system_id   # The unique identifier of the destination (e.g. vehicle) of the commanding interface
float64 time_stamp   # timeStamp is the time at which the InterfaceSetType is sent.
RMSPositionErrorType xy_position_rms  # The desired RMS value of the validity of the position data.
RMSPositionErrorType z_position_rms  # The desired RMS value of the validity of the position data.
