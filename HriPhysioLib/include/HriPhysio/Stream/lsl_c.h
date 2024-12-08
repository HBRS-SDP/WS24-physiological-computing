#ifndef LSL_C_H
#define LSL_C_H

/** @file lsl_c.h LSL C API for the lab streaming layer
 *
 * The lab streaming layer provides a set of functions to make instrument data accessible
 * in real time within a lab network. From there, streams can be picked up by recording programs,
 * viewing programs or custom experiment applications that access data streams in real time.
 *
 * The API covers two areas:
 * - The "push API" allows to create stream outlets and to push data (regular or irregular
 * measurement time series, event data, coded audio/video frames, etc.) into them.
 * - The "pull API" allows to create stream inlets and read time-synched experiment data from them
 *   (for recording, viewing or experiment control).
 *
 * To use this library you need to link to the liblsl library that comes with
 * this header. Under Visual Studio the library is linked in automatically.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "HriPhysio/Stream/lsl/common.h"
#include "HriPhysio/Stream/lsl/inlet.h"
#include "HriPhysio/Stream/lsl/outlet.h"
#include "HriPhysio/Stream/lsl/resolver.h"
#include "HriPhysio/Stream/lsl/streaminfo.h"
#include "HriPhysio/Stream/lsl/types.h"
#include "HriPhysio/Stream/lsl/xml.h"

#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif
