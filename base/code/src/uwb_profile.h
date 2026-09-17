/*
 * UWB radio profile.
 *
 * Every field here has to be identical on the anchors and on the tag, so this
 * file is kept byte for byte the same in base/code/src/ and code/drone/src/.
 * Change one, change the other.
 *
 * Two profiles are available. LONG_RANGE is the default: it buys roughly 6 dB
 * of link budget over FAST, which is close to twice the usable distance, and
 * pays for it in airtime. A full ranging cycle takes about 8x longer, so the
 * position update rate drops accordingly.
 *
 *   profile     | data rate | preamble | PRF    | frame airtime | range
 *   ------------|-----------|----------|--------|---------------|-------
 *   FAST        | 850 kbps  | 256      | 16 MHz | ~0.5 ms       | baseline
 *   LONG_RANGE  | 110 kbps  | 2048     | 64 MHz | ~4 ms         | ~2x
 *
 * Switching profiles changes the antenna delay, so a board calibrated on one
 * is not calibrated on the other. The anchors store the calibration under a
 * per profile key for that reason.
 */

#pragma once

#include <DW1000NgConfiguration.hpp>
#include <DW1000NgConstants.hpp>

/* Pick one. */
#define UWB_PROFILE_LONG_RANGE 1
#define UWB_PROFILE_FAST       0

#if UWB_PROFILE_LONG_RANGE == UWB_PROFILE_FAST
#error "pick exactly one UWB profile"
#endif

#if UWB_PROFILE_LONG_RANGE

#define UWB_PROFILE_NAME "long range"
/* short token, NVS keys cap at 15 characters */
#define UWB_PROFILE_KEY "lr"

static const device_configuration_t UWB_RADIO_CONFIG = {
    false,                        /* extendedFrameLength */
    true,                         /* receiverAutoReenable */
    true,                         /* smartPower */
    true,                         /* frameCheck */
    false,                        /* nlos optimization */
    SFDMode::DECAWAVE_SFD,        /* Decawave recommends the non standard SFD at 110 kbps */
    Channel::CHANNEL_5,
    DataRate::RATE_110KBPS,
    PulseFrequency::FREQ_64MHZ,
    PreambleLength::LEN_2048,
    PreambleCode::CODE_9          /* channel 5 at 64 MHz only accepts 9 to 12 */
};

/* Preamble 2048 means a PAC of 64, so the preamble is 32 PACs long. Four times
   that leaves room for a receiver that enables early. */
#define UWB_PREAMBLE_DETECTION_TIMEOUT 128

/* Decawave's rule: preamble length + 1 + SFD length - PAC size.
   2048 + 1 + 64 - 64 */
#define UWB_SFD_DETECTION_TIMEOUT 2049

/* One frame is about 4 ms of air; this also has to cover the reply delay and
   the scheduling slack on both sides. Keeps receiveFrame() from hanging. */
#define UWB_RECEIVE_FRAME_TIMEOUT_US 25000

#else /* UWB_PROFILE_FAST */

#define UWB_PROFILE_NAME "fast"
#define UWB_PROFILE_KEY "fast"

static const device_configuration_t UWB_RADIO_CONFIG = {
    false,                        /* extendedFrameLength */
    true,                         /* receiverAutoReenable */
    true,                         /* smartPower */
    true,                         /* frameCheck */
    false,                        /* nlos optimization */
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3          /* channel 5 at 16 MHz accepts 3 and 4 */
};

/* Preamble 256 means a PAC of 16, so 16 PACs of preamble. */
#define UWB_PREAMBLE_DETECTION_TIMEOUT 64

/* 256 + 1 + 8 - 16, rounded up */
#define UWB_SFD_DETECTION_TIMEOUT 273

#define UWB_RECEIVE_FRAME_TIMEOUT_US 5000

#endif

/* How long the tag waits before sending the final message of the exchange. It
   is bounded by how fast the tag can write the frame over SPI, not by the data
   rate, so it is the same on both profiles. Raise it if exchanges fail. */
#define UWB_FINAL_MESSAGE_DELAY_US 1500
