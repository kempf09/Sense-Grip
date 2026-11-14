// Feetech-style (CF35-12) packet codec. Pure C++, no Arduino headers.
#pragma once
#include <stddef.h>
#include <stdint.h>

enum : uint8_t { SV_PING = 1, SV_READ = 2, SV_WRITE = 3 };

// FF FF id len instr params... ~sum. Returns bytes written (n+6). out must hold n+6.
size_t svBuildPacket(uint8_t id, uint8_t instr, const uint8_t* params, uint8_t n, uint8_t* out);

enum class SvParse : uint8_t { OK, NEED_MORE, BAD_CHECKSUM, WRONG_ID };
// Reply layout: FF FF id len status params[len-2] checksum.
// consumed: OK/WRONG_ID = bytes through the packet's checksum; NEED_MORE = garbage
// before the header (the partial packet stays in the buffer); BAD_CHECKSUM = garbage
// plus the 2 header bytes (so the caller rescans after the bad header).
SvParse svParseReply(const uint8_t* buf, size_t len, uint8_t wantId, uint8_t& status,
                     const uint8_t*& data, uint8_t& n, size_t& consumed);

uint16_t svEncodeSM15(int32_t v);
int32_t  svDecodeSM15(uint16_t raw);
int16_t  svDecodeSM10(uint16_t raw);
// bit0 voltage, bit1 angle sensor, bit2 temperature, bit3 current, bit5 overload.
// "" when none of mask 0x2F is set. Always NUL-terminates (cap > 0), truncates safely.
void     svStatusText(uint8_t bits, char* out, size_t cap);

struct ServoFeedback {
  int32_t position; int16_t speed; float load; float voltage;
  uint8_t temp; uint8_t status; bool moving; int16_t current;
};
// Feedback block regs 56..73 (18 bytes), offsets as in Release 3.0.3 pollMotor().
ServoFeedback svDecodeFeedback(const uint8_t b[18]);

// Full-angle flag (register 18 bit 4) with failure tolerance: a good read sets the
// flag from the bit; a failed read keeps the last good value, and only
// SV_PHASE_MAX_FAILS consecutive failed reads clear it.
constexpr uint8_t SV_PHASE_MAX_FAILS = 3;
struct SvPhaseLatch { bool fullAngle; uint8_t fails; };
void svPhaseUpdate(SvPhaseLatch& l, bool readOk, uint8_t phase);
