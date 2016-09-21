// コンパイル前に ツール>マイコンボード>Koozyt DASH BLE を選択しておくこと

// 実装に際しての参考情報
// http://bril-tech.blogspot.jp/2014/05/bluetoothsmartmbed-1.html
// http://pentacreation.com/blog/2015/12/151207.html

// 通信速度を上げる方法
//
// BLE_APIのprojectconfig.hにパッチを当てる
// Macの場合、パッチ対象のprojectconfig.h は下記にある
// $HOME/Library/Arduino15/packages/koozyt/hardware/nrf51/{VERSION}/libraries/BLE_API/utility/projectconfig.h
//
// Diff
// - #define CFG_GAP_CONNECTION_MIN_INTERVAL_MS 100 //< Minimum acceptable connection interval
// - #define CFG_GAP_CONNECTION_MAX_INTERVAL_MS 500 //< Maximum acceptable connection interval
// + #define CFG_GAP_CONNECTION_MIN_INTERVAL_MS 10 //< Minimum acceptable connection interval
// + #define CFG_GAP_CONNECTION_MAX_INTERVAL_MS 50 //< Maximum acceptable connection interval
//
// さらなる情報はこちらを参照
// https://redbearlab.zendesk.com/entries/61838525--Solved-BLE-Nano-Slow-throughput
// https://developer.mbed.org/users/robo8080/code/BLE_RCBController_Motor/diff/4f33a5a25063/BLE_API_Native_IRC/hw/nRF51822n/projectconfig.h

#include <BLE_API.h>
#include <SPI.h>

const char NAME[] = "Hurray Q.board";

const uint8_t uuidWrite[] = {0x2C, 0xDE, 0xFB, 0x11, 0x58, 0x02, 0x4D, 0xFF, 0x91, 0x1C, 0x9B, 0x6E, 0x52, 0xE0, 0xD6, 0xC1};
const uint8_t uuidStatus[] = {0xAC, 0x32, 0x77, 0x2B, 0x84, 0xE2, 0x43, 0xE4, 0xBA, 0xF0, 0x30, 0x6F, 0x56, 0x3E, 0x33, 0x99};
const uint8_t uuidService[] = {0x27, 0x37, 0xF0, 0xF6, 0x34, 0x4A, 0x4B, 0xBD, 0xBF, 0xDF, 0x03, 0x72, 0xDE, 0x35, 0x09, 0x03};

BLEDevice ble;

uint8_t txBuffer[1] = {0};
uint8_t rxBuffer[1] = {0};

uint8_t statusPayload[1] = {0};
uint8_t writePayload[20] = {0};

GattCharacteristic characteristicStatus(uuidStatus, statusPayload, 1, sizeof(statusPayload), GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic characteristicWrite(uuidWrite, writePayload, 1, sizeof(writePayload), GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE);
GattCharacteristic *characteristics[] = {&characteristicStatus, &characteristicWrite};
GattService service(uuidService, characteristics, sizeof(characteristics) / sizeof(GattCharacteristic*));

//Ticker ticker;
//
//void m_1s_handle() {
//  char blank = '_';
//  ble.updateCharacteristicValue(characteristicStatus.getHandle(), &blank, 1);
//}

//SPI handler
void spi_slave_event_handler(spi_slave_evt_t event) {
  if (event.evt_type == SPI_SLAVE_XFER_DONE) {
    SPIS1.spi_slave_buffers_set(txBuffer, rxBuffer, sizeof(txBuffer), sizeof(rxBuffer));
    ble.updateCharacteristicValue(characteristicStatus.getHandle(), rxBuffer, 1);
  }
}

void disconnectionCallback() {
  ble.startAdvertising();
}

void onDataWritten(uint16_t charHandle) {
  uint8_t buf[20] = {0};
  uint16_t bytesRead;
  uint32_t err_code = NRF_SUCCESS;
  if (charHandle == characteristicWrite.getHandle()) {
    bytesRead = sizeof(buf);
    ble.readCharacteristicValue(characteristicWrite.getHandle(), buf, &bytesRead);
    for (uint16_t i = 0; i < bytesRead; i++) {
      Serial.write(buf[i]);
      Serial.flush();
    }
  }
}

void setup() {
  uint32_t err_code = NRF_SUCCESS;
  uint8_t val = 0;

  delay (1000);

  err_code = ble.init();

  ble.onDisconnection(disconnectionCallback);
  ble.onDataWritten(onDataWritten);

  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (const uint8_t *)NAME, strlen(NAME));
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (const uint8_t *)uuidService, sizeof(uuidService));
  ble.setAdvertisingInterval(1600); /* 160=100ms; in multiples of 0.625ms. */
  ble.setAdvertisingTimeout(0); /* 0 is disable the advertising timeout. */

  ble.addService(service);

  ble.startAdvertising();

  //SPI
  SPIS1.begin(spi_slave_event_handler);
  SPIS1.spi_slave_buffers_set(txBuffer, rxBuffer, sizeof(txBuffer), sizeof(rxBuffer));

//  ticker.attach(m_1s_handle, 1);
}

void loop() {
  //  ble.waitForEvent();
}
