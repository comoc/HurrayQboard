/*
1. 書き込みの準備
1-1.「環境設定」から追加のボードマネージャのURLに下記の1行を追加する。
http://koozyt.github.io/qboard_arduino_stm32/package_koozyt_qboard_index.json,http://koozyt.github.io/qboard_arduino_nrf51/package_koozyt_qboard-nrf51_index.json
1-2.「ツール」→「ボード」→「ボードマネージャ」から「Koozyt Q.board」で始まる2つのパッケージを追加する。
1-3. 下記URLからQ.board用のFree IMUをダウンロードしインストールする
https://github.com/koozyt/qboard_freeimu
1-4. Q.boardを接続する

2. 書き込み
2-1. 「ツール」→「ボード」→「Koozyt Q.board (Native USB Port)」を選択する
2-2. 「ツール」→「シリアルポート」からQ.boardが接続されているシリアルポートを選択する
2-3. 「スケッチ」→「マイコンボードに書き込む」
*/

#include <QBoard_FreeIMU.h>
#include <float.h>

const int BLE_PIN = 5;
const int INTERVAL = 60;//msec
unsigned long last_time = 0;
FreeIMU imu = FreeIMU();

void setup() {
  // Optional: Use Bean.setAccelerationRange() to set the sensitivity
  // to something other than the default of ±2g.
  //  Serial.begin(9600) ; // Serial port is already opened at 115200 bps by default.
  //  Bean.setLed(0, 0, 0);

  imu.init(true);
  imu.accgyromagn.setMgntEnabled(false); // If the parameter is true, it will be very slow to call getValues().

  pinMode(BLE_PIN, OUTPUT);
  digitalWrite(BLE_PIN, HIGH);
  SPI_2.begin(BLE_PIN);
}

const int LOG_COMPARE_LEN = 7 ;
const int LOG_AV_COUNT = 2 ;
const double LOOP_THR = 4000 * LOG_COMPARE_LEN ;
const int MINIMUM_OFFSET = 4 ;



const int LOGLEN = LOG_COMPARE_LEN * 2 ;

double raw_accel[3] ;
int raw_accel_count = LOG_AV_COUNT ;


double pastlog[LOGLEN][3] ;
int logpos = 0 ;

double prevaccel[3] ;


// 0: stopped 1: finding loop 2: counting
int mode = 0 ;

int loopInterv = -1 ;
int loopInterv_countdown ;

void setMode(char inChar) {
  if ( inChar == 's' ) {
    mode = 1 ;
    for ( int i = 0; i < LOGLEN; ++i )
      pastlog[i][0] = pastlog[i][1] = pastlog[i][2] = 0 ;
    logpos = 0 ;

    raw_accel[0] = raw_accel[1] = raw_accel[2] = 0 ;
    raw_accel_count = LOG_AV_COUNT ;

    //      Bean.setLed(255, 0, 0);
  } else if ( inChar == 'e' ) {
    //      Bean.setLed(0, 0, 0);
    mode = 0 ;
  }

}

void loop() {
  if ( Serial.available() > 0 ) {
    char inChar = Serial.read();
    setMode(inChar);
  }

  while (SerialBLE.available() > 0 ) {
    char inChar = SerialBLE.read();
    Serial.print("BLE: ");
    Serial.println(inChar);
    if (inChar == 's' || inChar == 'e')
      setMode(inChar);
  }

  if ( mode == 0 ) {
    //    Bean.sleep(100);
    delay(100);
    return ;
  }

  //  AccelerationReading accel = Bean.getAcceleration();
  //  raw_accel[0] += accel.xAxis ;
  //  raw_accel[1] += accel.yAxis ;
  //  raw_accel[2] += accel.zAxis ;

  float fvalues[9];
  imu.getValues(fvalues);

  raw_accel[0] += fvalues[0];
  raw_accel[1] += fvalues[1];
  raw_accel[2] += fvalues[2];

  if ( --raw_accel_count != 0 ) return ;
  raw_accel_count = LOG_AV_COUNT ;
  raw_accel[0] /= LOG_AV_COUNT ;
  raw_accel[1] /= LOG_AV_COUNT ;
  raw_accel[2] /= LOG_AV_COUNT ;

  pastlog[logpos][0] = raw_accel[0] - prevaccel[0] ;
  pastlog[logpos][1] = raw_accel[1] - prevaccel[1] ;
  pastlog[logpos][2] = raw_accel[2] - prevaccel[2] ;

  prevaccel[0] = raw_accel[0] ;
  prevaccel[1] = raw_accel[1] ;
  prevaccel[2] = raw_accel[2] ;

  raw_accel[0] = raw_accel[1] = raw_accel[2] = 0 ;

  boolean bIncreasing = false ;
  double prevCorr = DBL_MAX ;
  int i ;
  for ( i = MINIMUM_OFFSET; i < LOGLEN - LOG_COMPARE_LEN; ++i ) {
    double corr = 0 ;
    for ( int j = 0; j < LOG_COMPARE_LEN; ++j ) {
      int s1 = (logpos  + j) % LOGLEN ;
      int s2 = (logpos + i + j) % LOGLEN ;
      corr += pastlog[s1][0] * pastlog[s2][0] + pastlog[s1][1] * pastlog[s2][1] + pastlog[s1][2] * pastlog[s2][2] ;
    }

    if ( !bIncreasing && prevCorr < corr ) bIncreasing = true ;
    else if ( bIncreasing && prevCorr > corr ) {
      //prevCorr = corr ;
      break ;
    }
    prevCorr = corr ;
  }
  double peak = prevCorr ;

  logpos = (logpos + (LOGLEN - 1)) % LOGLEN ;

  if ( i == LOGLEN || peak < LOOP_THR ) {
    loopInterv = -1 ; // Loop reset
    return ;
  }

  // Passed.
  if ( loopInterv == -1 ) {
    loopInterv = i ;
    loopInterv_countdown = i ;
    //Serial.print( "H" ) ;
  } else if (--loopInterv_countdown == 0 ) {
    Serial.print( "H" ) ;

    unsigned long time = millis();
    unsigned long dt = time - last_time;
    if (dt >= INTERVAL) {
      char spi_buffer = 'H';
      SPI_2.transfer(BLE_PIN, &spi_buffer, sizeof(spi_buffer));
    }

    //Serial.print( loopInterv ) ;
    loopInterv_countdown = loopInterv ;
  }
  //Bean.sleep(1000/15);
}
