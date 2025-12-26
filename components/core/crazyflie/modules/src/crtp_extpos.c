#include <stddef.h>      // for NULL
#include <string.h>      // for memcpy
#include "crtp.h"
#include "estimator.h"
#include "log.h"
#include "crtp_extpos.h"
#include "crtp_localization_service.h"

typedef enum {
    EXTPOS_CHANNEL = 0,
    GENERIC_CHANNEL = 1,
} extposChannel_t;

typedef struct __attribute__((packed)) {
    float x;
    float y;
    float z;
} extpos_t;

static extpos_t ext_pos;
static poseMeasurement_t ext_pose;
static float extPosStdDev = 0.01f;
static float extQuatStdDev = 4.5e-3f;

// ---------- LOG GROUP ----------
LOG_GROUP_START(ext_pos)
LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)

// ---------- CRTP HANDLER ----------
static void extposCrtpCB(CRTPPacket *pk)
{
    if (pk == NULL) {
        return;
    }
    if (pk->data == NULL) {
        return;
    }

    if (pk->channel == EXTPOS_CHANNEL) {
        if (pk->size != sizeof(extpos_t)) {
            return;
        }

        extpos_t p;
        memcpy(&p, pk->data, sizeof(p));

        ext_pos = p;

        positionMeasurement_t meas;
        meas.x      = p.x;
        meas.y      = p.y;
        meas.z      = p.z;
        meas.stdDev = extPosStdDev;

        estimatorEnqueuePosition(&meas);
    } else if (pk->channel == GENERIC_CHANNEL) {
        if (pk->size < 1) {
            return;
        }
        const uint8_t type = pk->data[0];
        if (type == EXT_POSE) {
            if (pk->size < (1 + sizeof(struct CrtpExtPose))) {
                return;
            }
            const struct CrtpExtPose* pose = (const struct CrtpExtPose*)&pk->data[1];
            ext_pose.x = pose->x;
            ext_pose.y = pose->y;
            ext_pose.z = pose->z;
            ext_pose.quat.x = pose->qx;
            ext_pose.quat.y = pose->qy;
            ext_pose.quat.z = pose->qz;
            ext_pose.quat.w = pose->qw;
            ext_pose.stdDevPos = extPosStdDev;
            ext_pose.stdDevQuat = extQuatStdDev;
            estimatorEnqueuePose(&ext_pose);
        }
    }
}

void locSrvInit(void)
{
    crtpRegisterPortCB(CRTP_PORT_LOCALIZATION , extposCrtpCB);
}
