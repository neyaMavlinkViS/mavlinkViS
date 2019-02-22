float64 magnetic_variation   # The location specific magnetic variation.
string session_id   # sessionID identifier for one or a series of related message exchanges. The sessionID value in response messages should be set to the same value as the sessionID for the corresponding request messages. This attribute may be empty in published response messages that are sent based on subscriptions to the response message type
string source_subsystem_id   # The unique identifier of the originating subsystem (e.g. a client, operator, etc.) of the commanding interface
string source_system_id   # The unique identifier of the originating source (e.g. control station) of the commanding interface
string target_subsystem_id   # The unique identifier of the destination subsystems (e.g. engine, payload, sensor, etc.) of the commanding interface
string target_system_id   # The unique identifier of the destination (e.g. vehicle) of the commanding interface
float64 time_stamp   # timeStamp is the time at which the InterfaceSetType is sent.
