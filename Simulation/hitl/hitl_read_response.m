function resp = hitl_read_response(port, timeoutS)
%HITL_READ_RESPONSE Block (up to timeoutS) reading one HITLResponsePacket
%   from an open serialport, resyncing on its magic bytes -- same tolerant
%   scan-and-discard pattern the ground station's own decoder uses (see
%   GroundStation/core/packet_decoder.py's find_packet_start()). Returns []
%   on timeout (link dropped or Teensy not running the rocketpower_hitl
%   build), otherwise a struct from hitl_decode_response.m.
    RESP_SIZE = 26;
    resp = [];
    tStart = tic;
    have = 0;
    buf = zeros(1, RESP_SIZE, 'uint8');

    while toc(tStart) < timeoutS
        if port.NumBytesAvailable == 0
            continue;
        end
        b = read(port, 1, 'uint8');
        if have == 0
            if b ~= hex2dec('C6'), continue; end
            buf(1) = b; have = 1;
        elseif have == 1
            if b ~= hex2dec('18'), have = 0; continue; end
            buf(2) = b; have = 2;
        else
            have = have + 1;
            buf(have) = b;
            if have == RESP_SIZE
                payload  = buf(1:end-2);
                expected = uint16(mod(sum(uint32(payload)), 65536));
                stored   = typecast(buf(end-1:end), 'uint16');
                if expected == stored
                    resp = hitl_decode_response(buf);
                    return;
                end
                have = 0;   % bad checksum -- resync from scratch
            end
        end
    end
end
