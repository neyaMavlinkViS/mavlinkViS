AltitudeMSLTuple commanded_altitude  # The optional commanded altitude used for the unmanned air platform.
AltitudeAGLTuple commanded_altitude_agl  # The optional commanded altitude used for the unmanned air platform.
float64 commanded_altitude_asf   # The optional commanded altitude used for the unmanned air platform.
Orientation3DPlatformXYZMeasurement commanded_attitude  # The commanded orientation (roll, pitch, yaw) of the unmanned platform.
RMSOrientationErrorType commanded_attitude_rms  # The commanded RMS value of the validity of the orientation data.
float64 commanded_depth   # The optional commanded depth used for the unmanned maritime platform.
Position3DWGS84Tuple commanded_position  # The commanded position (latitude, longitude) of the unmanned platform
RMSPositionErrorType commanded_x_y_position_rms  # The commanded RMS value of the validity of the position data.
RMSPositionErrorType commanded_z_position_rms  # The commanded RMS value of the validity of the position data.
string session_id   # sessionID identifier for one or a series of related message exchanges. The sessionID value in response messages should be set to the same value as the sessionID for the corresponding request messages. This attribute may be empty in published response messages that are sent based on subscriptions to the response message type
string source_subsystem_id   # The unique identifier of the originating source sub-system (e.g. engine, payload, sensor, etc.) of the status interface
string source_system_id   # The unique identifier of the originating source (e.g. vehicle) of the status interface
string target_subsystem_id   # The unique identifier of the destination (e.g. a client, an application, etc.) of the status interface
string target_system_id   # The unique identifier of the destination (e.g. control station) of the status interface
float64 time_stamp   # timeStamp is the time at which the InterfaceSetType is sent.
