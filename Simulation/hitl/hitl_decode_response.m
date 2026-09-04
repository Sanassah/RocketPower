function r = hitl_decode_response(buf)
%HITL_DECODE_RESPONSE Decode a 26-byte HITLResponsePacket, matching
%   FlightComputer/src/telemetry/Packet.h's HITLResponsePacket struct
%   exactly (magic already stripped/validated by the caller -- see
%   hitl_read_response.m). Keep in sync by hand with that struct.
%     offset  size  field
%     0       2     magic (validated by caller, not re-checked here)
%     2       4     uint32 timestamp_ms
%     6       1     uint8  state
%     7       16    float  fin_deg[4]
%     23      1     uint8  attitude_status
%     24      2     uint16 checksum (validated by caller)
    buf = uint8(buf(:))';
    r.timestamp_ms    = double(typecast(buf(3:6), 'uint32'));
    r.state            = double(buf(7));
    r.fin_deg           = double(typecast(buf(8:23), 'single'));
    r.attitude_status   = double(buf(24));
end
