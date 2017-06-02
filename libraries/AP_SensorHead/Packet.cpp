#include "Packet.h"

int BSStream::read()
{
    while (true) {

        uint8_t c = _base->read();

        if(c == Packet::ESC) {
            _escape_pending = true;
            continue;
        }

        if(!_escape_pending) {
            if(c == Packet::START) {
                continue;
            }
            else if(c == Packet::END) {
                return EOP;
            }
            else {
                return c;
            }
        } else {
            _escape_pending = false;
            return c;
        }
    }
}

size_t BSStream::write(std::vector<uint8_t> &vals)
{

    size_t bytesWritten = 0;

    _base->write(Packet::START);
    for(uint8_t b : vals) {
        if(b == Packet::ESC) {
            _base->write(Packet::ESC);
            _base->write(b);
            bytesWritten+= 2;
        }
        else if(b == Packet::START) {
            _base->write(Packet::ESC);
            _base->write(b);
            bytesWritten+= 2;
        }
        else if(b == Packet::END) {
            _base->write(Packet::ESC);
            _base->write(b);
            bytesWritten+= 2;
        }
        else {
            _base->write(b);
            bytesWritten++;
        }
    }
    _base->write(Packet::END);
    bytesWritten++;

    return bytesWritten;
}
