//
// Created by renzo on 24-4-17.
//

#include <bb8_bluetooth_positioning/Bluetooth.h>


int main(int argc, char **argv)
{
    /**
    * Initialize rosnode with name 'controlnode'.
    */
    ros::init(argc, argv, "bb8_bluetooth_positioning_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    Bluetooth* bluetooth = new Bluetooth();
    ROS_INFO("%s", "Starting control loop");
    while (ros::ok())
    {
        bluetooth->update();
        /**
         * Call all callbacks waiting to be called at this point in time.
         */
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

Bluetooth::Bluetooth(){
    // Get HCI device.
    const int device = hci_open_dev(hci_get_route(NULL));
    if ( device < 0 ) {
        perror("Failed to open HCI device.");
        exit(0);
    }
    if(startScan() == 0){
        hci_close_dev(device);
        exit(0);
    }
}

Bluetooth::~Bluetooth() {
    stopScan();
    hci_close_dev(device);
}

int Bluetooth::update(){
    uint8_t buf[HCI_MAX_EVENT_SIZE];
    evt_le_meta_event * meta_event;
    le_advertising_info * info;
    int len;

    len = read(device, buf, sizeof(buf));
    if ( len >= HCI_EVENT_HDR_SIZE ) {
        meta_event = (evt_le_meta_event*)(buf+HCI_EVENT_HDR_SIZE+1);
        if ( meta_event->subevent == EVT_LE_ADVERTISING_REPORT ) {
            uint8_t reports_count = meta_event->data[0];
            void * offset = meta_event->data + 1;
            while ( reports_count-- ) {
                info = (le_advertising_info *)offset;
                char addr[18];
                ba2str(&(info->bdaddr), addr);
                printf("%s - RSSI %d\n", addr, (char)info->data[info->length]);
                offset = info->data + info->length + 2;
            }
        }
    }
}

int Bluetooth::startScan(){
    int ret, status;
    // Set BLE scan parameters.
    le_set_scan_parameters_cp scan_params_cp;
    memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type 			= 0x00;
    scan_params_cp.interval 		= htobs(0x0010);
    scan_params_cp.window 			= htobs(0x0010);
    scan_params_cp.own_bdaddr_type 	= 0x00; // Public Device Address (default).
    scan_params_cp.filter 			= 0x00; // Accept all.

    struct hci_request scan_params_rq = ble_hci_request(OCF_LE_SET_SCAN_PARAMETERS, LE_SET_SCAN_PARAMETERS_CP_SIZE, &status, &scan_params_cp);
    ret = hci_send_req(device, &scan_params_rq, 1000);
    if ( ret < 0 ) {
        perror("Failed to set scan parameters data.");
        return 0;
    }

    // Enable scanning.

    le_set_scan_enable_cp scan_cp;
    memset(&scan_cp, 0, sizeof(scan_cp));
    scan_cp.enable 		= 0x01;	// Enable flag.
    scan_cp.filter_dup 	= 0x00; // Filtering disabled.

    struct hci_request enable_adv_rq = ble_hci_request(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE, &status, &scan_cp);

    ret = hci_send_req(device, &enable_adv_rq, 1000);
    if ( ret < 0 ) {
        perror("Failed to enable scan.");
        return 0;
    }

    // Get Results.
    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    if ( setsockopt(device, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0 ) {
        perror("Could not set socket options\n");
        return 0;
    }
    return 1;
}

int Bluetooth::stopScan(){
    int ret, status;
    // Disable scanning.
    memset(&scan_cp, 0, sizeof(scan_cp));
    scan_cp.enable = 0x00;	// Disable flag.

    struct hci_request disable_adv_rq = ble_hci_request(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE, &status, &scan_cp);
    ret = hci_send_req(device, &disable_adv_rq, 1000);
    if ( ret < 0 ) {
        perror("Failed to disable scan.");
        return 0;
    }
    return 1;
}