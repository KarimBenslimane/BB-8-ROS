//
// Created by renzo on 24-4-17.
//

#ifndef PROJECT_BLUETOOTH_H
#define PROJECT_BLUETOOTH_H

#include "ros/ros.h"
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

struct hci_request ble_hci_request(uint16_t ocf, int clen, void * status, void * cparam)
{
    struct hci_request rq;
    memset(&rq, 0, sizeof(rq));
    rq.ogf = OGF_LE_CTL;
    rq.ocf = ocf;
    rq.cparam = cparam;
    rq.clen = clen;
    rq.rparam = status;
    rq.rlen = 1;
    return rq;
}

class Bluetooth {
    public:
        Bluetooth();
        ~Bluetooth();
        int update();
    private:
        int startScan();
        int stopScan();
        int device;
        le_set_scan_enable_cp scan_cp;
};

#endif //PROJECT_BLUETOOTH_H
