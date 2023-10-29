#ifndef __BLE_MAGNETO__
#define __BLE_MAGNETO__

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"

class HeartrateService {
public:
    uint16_t Heartrate_UUID= 0x180D;
    uint16_t char_UUID_X = 0x1800;
    uint16_t char_UUID_Y = 0x1801;
    uint16_t char_UUID_Z = 0x1802;

    HeartrateService(BLE &_ble, int M_data[3]) :
        ble(_ble), M_X(char_UUID_X, M_data),M_Y(char_UUID_Y, M_data),M_Z(char_UUID_Z, M_data)
    {
        GattCharacteristic *charTable[] = {&M_X,&M_Y,&M_Z};
        GattService         HeartrateService(HeartrateService::Heartrate_UUID, charTable, 3);
        ble.gattServer().addService(HeartrateService);
    }

    void updateHeartRate(int M_data[3]) {
        ble_error_t err;
        int x=M_data[0];
        int y=M_data[1];
        int z=M_data[2];
        printf("%d  %d  %d\n",x,y,z);
        ble.gattServer().write(M_X.getValueHandle(), (uint8_t*)&x, 4);
        ble.gattServer().write(M_Y.getValueHandle(), (uint8_t*)&y, 4);
        ble.gattServer().write(M_Z.getValueHandle(), (uint8_t*)&z, 4);


    }

private:

    ReadOnlyGattCharacteristic<int>  M_X;
    ReadOnlyGattCharacteristic<int>  M_Y;
    ReadOnlyGattCharacteristic<int>  M_Z;
    BLE                              &ble;
};

#endif /* #ifndef __BLE_BUTTON_SERVICE_H__ */
