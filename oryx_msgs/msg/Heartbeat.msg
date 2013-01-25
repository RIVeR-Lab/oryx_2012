#Message for sending a heartbeat to the oryx_diagnostics management software.

#timestamp (does not use header to save weight. Used by the diagnostics package for transmission latency purposes
time timestamp

#The unique node_id that was provided at registration with the diagnostics system to the node
uint16 node_id

