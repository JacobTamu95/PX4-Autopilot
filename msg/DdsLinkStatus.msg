# uXRCE-DDS client link usage (one sample per second, logged by default).
#
# Mirrors TelemetryStatus for the MAVLink links so every flight log shows how much
# of the DDS link is used and by which topics.
#
# Wire rates are measured at the transport (every XRCE frame, including session,
# ping, time-sync and acknowledgement traffic) and include the serial framing
# (flag, addresses, length, CRC and byte stuffing), so they are the bytes on the UART.
# Per-topic rates count serialized sample payload only, so they sum to less than the
# wire rate; the difference is protocol overhead.
#
# Per-topic arrays are indexed in dds_topics.yaml order: topic_tx_* follows
# `publications`, topic_rx_* follows `subscriptions` then `subscriptions_multi`.
# topic_list_hash identifies that list (FNV-1a 32 of the topic names) so a log
# viewer can recover the names for the firmware that recorded the log.

uint64 timestamp			# time since system start (microseconds)

uint8 TRANSPORT_SERIAL = 0
uint8 TRANSPORT_UDP = 1
uint8 transport

bool connected
bool timesync_converged

uint32 baudrate				# serial baud rate (0 for UDP)
float32 data_rate			# link capacity (Bytes/s): baudrate/10 for 8N1 serial, 0 if unknown

float32 tx_rate				# bytes on the wire, client -> agent (Bytes/s)
float32 rx_rate				# bytes on the wire, agent -> client (Bytes/s)
float32 tx_frame_rate			# XRCE frames sent (1/s)
float32 rx_frame_rate			# XRCE frames received (1/s)
float32 tx_payload_rate			# serialized topic payload sent (Bytes/s)
float32 rx_payload_rate			# serialized topic payload received (Bytes/s)

uint32 tx_send_errors			# total frames the transport failed to write
uint32 tx_samples_dropped		# total outgoing samples dropped (output stream full)
uint32 session_count			# sessions established since boot (>1 means reconnects)

uint8 MAX_TOPICS = 48
uint8 num_tx_topics			# valid entries in topic_tx_*
uint8 num_rx_topics			# valid entries in topic_rx_*
uint32 topic_list_hash			# FNV-1a 32 over the topic names (see header)

float32[48] topic_tx_rate		# per outgoing topic payload (Bytes/s)
float32[48] topic_rx_rate		# per incoming topic payload (Bytes/s)
uint16[48] topic_tx_sample_rate		# per outgoing topic samples (1/s)
uint16[48] topic_rx_sample_rate		# per incoming topic samples (1/s)
