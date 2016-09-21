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

// Only two customizable variables
const int ORIG_FPS = 1000 ;           // loop()が秒間何回呼ばれるか。Q Boardなら1000固定。
const double LOOP_THR = 80000000 ;    // 繰り返しと判定されるスレッショルド。加速度として得られる値の二乗に比例。

const int PROCESS_FPS = 20 ;  // 実際に自己相関計算を行うサンプル単位
const double WINDOW_HALF_SIZE_IN_SEC = 0.8 ; // 計算を行うウィンドウサイズ。小さい方がきびきび反応するが、遅い周期を検出できない。大きい方が遅い運動を検出できるが、動かし始めてからカウントアップ開始までの時間、止めてからカウント停止までの時間が遅くなる。

#include <QBoard_FreeIMU.h>
#include <float.h>

// consts
const int WINDOW_HALF_SIZE = (int)(PROCESS_FPS * WINDOW_HALF_SIZE_IN_SEC) ;
const int AVERAGE_WINDOW_SIZE = (ORIG_FPS/PROCESS_FPS) ;// raw input average factor
const int MINIMUM_OFFSET = PROCESS_FPS * 1 / 5 ;
const int WINDOW_SIZE = WINDOW_HALF_SIZE * 2 ;

const int BLE_PIN = 5;
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


double raw_accel[3] ;
int raw_accel_count = AVERAGE_WINDOW_SIZE ;


double accel_diff_log[WINDOW_SIZE][3] ;
double accel_log[WINDOW_SIZE][3] ;
int logpos = 0 ;

//double prevaccel[3] ;


// 0: stopped 1: finding loop 2: counting
int mode = 0 ;

int loopInterv = -1 ;
int loopInterv_countdown ;

void setMode(char inChar) {
  if ( inChar == 's' ) {
    mode = 1 ;
    for ( int i = 0; i < WINDOW_SIZE; ++i )
      accel_diff_log[i][0] = accel_diff_log[i][1] = accel_diff_log[i][2] = 0 ;
    logpos = 0 ;

    raw_accel[0] = raw_accel[1] = raw_accel[2] = 0 ;
    raw_accel_count = AVERAGE_WINDOW_SIZE ;

    //      Bean.setLed(255, 0, 0);
  } else if ( inChar == 'e' ) {
    //      Bean.setLed(0, 0, 0);
    mode = 0 ;
  }

}

void loop() {
    //Serial.print( "." ) ;
    //Serial.flush() ;

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
    delay(100);
    return ;
  }

  float fvalues[9];
  imu.getValues(fvalues);

  // raw_accel is for averaging raw input
  raw_accel[0] += fvalues[0];
  raw_accel[1] += fvalues[1];
  raw_accel[2] += fvalues[2];

  if ( --raw_accel_count != 0 ) return ;

  raw_accel_count = AVERAGE_WINDOW_SIZE ;
  raw_accel[0] /= AVERAGE_WINDOW_SIZE ;
  raw_accel[1] /= AVERAGE_WINDOW_SIZE ;
  raw_accel[2] /= AVERAGE_WINDOW_SIZE ;

  accel_log[logpos][0] = raw_accel[0] ;
  accel_log[logpos][1] = raw_accel[1] ;
  accel_log[logpos][2] = raw_accel[2] ;

  accel_diff_log[logpos][0] = raw_accel[0] - accel_log[(logpos+1)%WINDOW_SIZE][0] ;
  accel_diff_log[logpos][1] = raw_accel[1] - accel_log[(logpos+1)%WINDOW_SIZE][1] ;
  accel_diff_log[logpos][2] = raw_accel[2] - accel_log[(logpos+1)%WINDOW_SIZE][2] ;

//  accel_diff_log[logpos][0] = raw_accel[0] - prevaccel[0] ;
//  accel_diff_log[logpos][1] = raw_accel[1] - prevaccel[1] ;
//  accel_diff_log[logpos][2] = raw_accel[2] - prevaccel[2] ;
//  prevaccel[0] = raw_accel[0] ;
//  prevaccel[1] = raw_accel[1] ;
//  prevaccel[2] = raw_accel[2] ;

  raw_accel[0] = raw_accel[1] = raw_accel[2] = 0 ;


  double av_accel[3] = {0,0,0} ;
  int i ;
  for( i=0;i<WINDOW_SIZE;++i ){
    av_accel[0] += accel_log[i][0] ;
    av_accel[1] += accel_log[i][1] ;
    av_accel[2] += accel_log[i][2] ;
  }
  av_accel[0] /= WINDOW_SIZE ;
  av_accel[1] /= WINDOW_SIZE ;
  av_accel[2] /= WINDOW_SIZE ;

  boolean bIncreasing = false ;
  double prevCorr = DBL_MAX , prevOri = 0 ;
  for ( i = MINIMUM_OFFSET; i < WINDOW_SIZE - WINDOW_HALF_SIZE; ++i ) {
    double corr = 0 , ori = 0 ;
    for ( int j = 0; j < WINDOW_HALF_SIZE; ++j ) {
      int s1 = (logpos  + j) % WINDOW_SIZE ;
      int s2 = (logpos + i + j) % WINDOW_SIZE ;
      corr += (accel_diff_log[s1][0] * accel_diff_log[s2][0] + accel_diff_log[s1][1] * accel_diff_log[s2][1] + accel_diff_log[s1][2] * accel_diff_log[s2][2])/(LOOP_THR*WINDOW_HALF_SIZE) ;

      double cross_product_x = (accel_diff_log[s1][1]*accel_diff_log[s2][2] - accel_diff_log[s1][2]*accel_diff_log[s2][1])/LOOP_THR ;
      double cross_product_y = (accel_diff_log[s1][2]*accel_diff_log[s2][0] - accel_diff_log[s1][0]*accel_diff_log[s2][2])/LOOP_THR ;
      double cross_product_z = (accel_diff_log[s1][0]*accel_diff_log[s2][1] - accel_diff_log[s1][1]*accel_diff_log[s2][0])/LOOP_THR ;

      ori += cross_product_x * av_accel[0] + cross_product_y * av_accel[1] + cross_product_z * av_accel[2] ;
    }

    if ( !bIncreasing && prevCorr < corr ) bIncreasing = true ;
    else if ( bIncreasing && prevCorr > corr ) {// スコアが減少に転じた
      break ;
    }
    prevCorr = corr ;
    prevOri = ori ;
  }
  double peak = prevCorr ;
  double peakori = prevOri ;

  logpos = (logpos + (WINDOW_SIZE - 1)) % WINDOW_SIZE ;

  if ( i == WINDOW_SIZE || peak < 1 ) {
    loopInterv = -1 ; // Loop reset
    return ;
  }

  // Passed.
  if ( loopInterv == -1 ) {
    loopInterv = i ;
    loopInterv_countdown = i ;
    //Serial.print( "H" ) ;
  } else if (--loopInterv_countdown == 0 ) {
    Serial.print( peakori>0 ? "R" : "L" ) ;
    //Serial.println(peakori) ;
    loopInterv_countdown = loopInterv ;
  }
}
