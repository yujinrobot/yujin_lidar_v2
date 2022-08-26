/*********************************************************************
*  Copyright (c) 2022, YujinRobot Corp.
*
*  Non-monifiable freely redistributable software(FRS)
*
*  - Redistribution. Redistribution and use in binary form, without modification,
*    are permitted provided that the following conditions are met:
*  - Redistributions must reproduce the above copyright notice and the following
*    disclaimer in the documentation and/or other materials provided with the distribution.
*  - Neither the name of YujinRobot Corporation nor the names of its suppliers may be used
*    to endorse or promote products derived from this software without specific prior written permission.
*
*  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OFOR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*********************************************************************/
#ifndef YRL_LOG_HPP
#define YRL_LOG_HPP

#define YRL_LOG_USER    0
#define YRL_LOG_FATAL   1
#define YRL_LIDAR_INFO  2
#define YRL_LOG_ERROR   3
#define YRL_LOG_WARN    4
#define YRL_LOG_INFO    5
#define YRL_LOG_MESSAGE 6
#define YRL_LOG_DEBUG   7
#define YRL_LOG_TRACE   8
#define YRL_LOG_ALL     10

#define LOG_LV  5

//__FILE__, __FUNCTION__, __LINE__

#define LOGPRINT(PREFIX, LEVEL, ARGS) \
  if (LOG_LV > LEVEL) { \
    if (LEVEL != YRL_LIDAR_INFO) { printf("["); printf(#PREFIX); printf("]");} \
    printf("["); printf(#LEVEL); printf("]"); \
    if (LEVEL == YRL_LOG_ERROR) printf("[%s:%d] ", __FUNCTION__, __LINE__);  \
    if (LEVEL == YRL_LOG_INFO) printf("[%s:%d] ", __FUNCTION__, __LINE__);  \
    if (LEVEL == YRL_LOG_DEBUG) printf("[%s:%d] ", __FUNCTION__, __LINE__);  \
    printf(" "); printf ARGS; }

#endif //YRL_LOG_HPP