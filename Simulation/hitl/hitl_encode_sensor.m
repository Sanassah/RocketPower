function bytes = hitl_encode_sensor(seq, quat, gyro, linAccel, highg, altM, vvel)
%HITL_ENCODE_SENSOR Build a SensorInjectPacket byte row exactly matching
%   FlightComputer/src/telemetry/Packet.h's SensorInjectPacket struct
%   (#pragma pack(push,1), little-endian -- MATLAB on Windows is already
%   little-endian, so no byte-swapping needed). Keep this in sync BY HAND
%   with that struct if either side ever changes field order/count.
%
%   quat = [w x y z]; gyro/linAccel/highg = [x y z] row or column, any mix.
    magic  = uint8([hex2dec('C5'), hex2dec('17')]);
    seqB   = typecast(uint32(seq), 'uint8');

    floats = single([quat(:); gyro(:); linAccel(:); highg(:); altM; vvel]);
    if numel(floats) ~= 15
        error('hitl_encode_sensor:badSize', 'expected 15 floats, got %d', numel(floats));
    end
    floatB = typecast(floats, 'uint8')';   % column of floats -> row of bytes

    payload  = [magic, seqB, floatB];
    checksum = uint16(mod(sum(uint32(payload)), 65536));
    bytes    = [payload, typecast(checksum, 'uint8')];
end
