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

// １波長の最長長(秒単位
const double WAVELENGTH_MAX_SEC = 2.5 ;
// １波長の最短長(秒単位
const double WAVELENGTH_MIN_SEC = 0.5 ;
// チェックする波長長の刻み
const double WAVELENGTH_STEP_SEC = 0.1 ;
// 回転されたと判断するスレッショルド
const double LOOP_CHECK_THR = 1100000 ;

const int PROCESS_FPS = 20 ;    // 平滑化後FPS


#include <QBoard_FreeIMU.h>
#include <float.h>
#include <math.h>

// consts
const int ORIG_FPS = 1000 ;           // loop()が秒間何回呼ばれるか。Q Boardなら1000固定。
const int SAMPLES = (int)(PROCESS_FPS * WAVELENGTH_MAX_SEC) ;
const double SAMPLES_INV = 1.0/(double)SAMPLES ;
const int AVERAGE_WINDOW_SIZE = ORIG_FPS / PROCESS_FPS ;
const int WAVELEN_MAX = (int)(PROCESS_FPS * WAVELENGTH_MAX_SEC) ;
const int WAVELEN_MIN = (int)(PROCESS_FPS * WAVELENGTH_MIN_SEC) ;
const int WAVELEN_STEP = (int)(PROCESS_FPS * WAVELENGTH_STEP_SEC) ;

const int BLE_PIN = 5;
FreeIMU imu = FreeIMU();

double raw_accel[2] ;
int raw_accel_count = AVERAGE_WINDOW_SIZE ;
double accel_log[SAMPLES][2] ;
int logpos = 0 ;

double cos_tbl[1+(WAVELEN_MAX-WAVELEN_MIN)/WAVELEN_STEP][SAMPLES] ;
double sin_offs[1+(WAVELEN_MAX-WAVELEN_MIN)/WAVELEN_STEP] ;

int no_out_countdown  ;
double prev_best_t0 ;

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

  logpos = 0 ;
  raw_accel[0] = raw_accel[1] = 0 ;
  raw_accel_count = AVERAGE_WINDOW_SIZE ;

  no_out_countdown = 0 ;
  prev_best_t0 = DBL_MAX ;

  // Make cosine table
  int wl_id = 0 ;
  for( int wavelen = WAVELEN_MIN ; wavelen <= WAVELEN_MAX ; wavelen += WAVELEN_STEP , ++wl_id ){
    double* tbl_tgt = cos_tbl[wl_id] ;
    sin_offs[wl_id] = (int)(wavelen*3/4) ;
    for( int i=0;i<SAMPLES;++i ){
      double th = 2 * M_PI * i / (double)wavelen ;
      tbl_tgt[i] = cos( th ) ;
    }
  }
}

void loop() {
  float fvalues[9];
  imu.getValues(fvalues);

  // raw_accel is for averaging raw input. discard gravity axis
  raw_accel[0] += fvalues[1];
  raw_accel[1] += fvalues[2];

  if ( --raw_accel_count != 0 ) return ;

  raw_accel_count = AVERAGE_WINDOW_SIZE ;
  raw_accel[0] /= AVERAGE_WINDOW_SIZE ;
  raw_accel[1] /= AVERAGE_WINDOW_SIZE ;

  
  accel_log[logpos][0] = raw_accel[0] ;
  accel_log[logpos][1] = raw_accel[1] ;

  raw_accel[0] = raw_accel[1] = 0 ;

  double accel_log_dc[2] = {0,0} ;
  for( int i=0;i<SAMPLES;++i ){
    accel_log_dc[0] += accel_log[i][0] ;
    accel_log_dc[1] += accel_log[i][1] ;
  }
  accel_log_dc[0] *= SAMPLES_INV ;
  accel_log_dc[1] *= SAMPLES_INV ;

  // Convolution
  int wl_id = 0 ;
  int best_wl = -1 , best_wl_power = 0 ;
  double best_t0 , best_t1 ;
  int wavelen ;
  for( wavelen = WAVELEN_MIN ; wavelen <= WAVELEN_MAX ; wavelen += WAVELEN_STEP , ++wl_id ){
    double cos0_power = 0 , sin0_power = 0 ;
    double cos1_power = 0 , sin1_power = 0 ;
    double* c_tbl = cos_tbl[wl_id] ;

    int si = sin_offs[wl_id] ;
    for( int ci=0;ci<SAMPLES;++ci ){
      double* samp = accel_log[(logpos+ci)%SAMPLES] ;

      double samp0_norm = (samp[0] - accel_log_dc[0]) * SAMPLES_INV ;
      double samp1_norm = (samp[1] - accel_log_dc[1]) * SAMPLES_INV ;
      
      cos0_power += c_tbl[ci] * samp0_norm ;
      sin0_power += c_tbl[si] * samp0_norm ;

      cos1_power += c_tbl[ci] * samp1_norm ;
      sin1_power += c_tbl[si] * samp1_norm ;

      if( ++si >= SAMPLES ) si = 0 ;
    }
    
    double pw0 = cos0_power*cos0_power + sin0_power*sin0_power ;
    double pw1 = cos1_power*cos1_power + sin1_power*sin1_power ;
    if( pw0 > best_wl_power ){
      best_wl_power = pw0 ;
      best_wl = wl_id ;
      best_t0 = atan2( sin0_power , cos0_power ) ;
      best_t1 = atan2( sin1_power , cos1_power ) ;
    }
  }

  if( best_wl != wl_id-1  && best_wl_power > LOOP_CHECK_THR ){
    if( (no_out_countdown==0 || --no_out_countdown == 0) && prev_best_t0 != DBL_MAX && prev_best_t0 * best_t0 <= 0 ){
      // Execute here only when t0 crosses zero
      double wavelen_sec = WAVELENGTH_MIN_SEC + best_wl * WAVELENGTH_STEP_SEC ;
      no_out_countdown = (best_wl + WAVELEN_MIN) / 3 ; // あんまりすぐに次の表示はしない
      boolean bOri = ( ( best_t0>best_t1 && best_t0-best_t1 >= M_PI ) || ( best_t0<=best_t1 && best_t1-best_t0 < M_PI ) ) ;

      Serial.print( bOri?"R:":"L:" ) ;
      Serial.println( wavelen_sec ) ;
      //Serial.println( best_wl_power ) ;

    }
    prev_best_t0 = best_t0 ;

  } else {
    prev_best_t0 = DBL_MAX ;
  }

  // Increment logpos (backward direction)
  logpos = (logpos+(SAMPLES-1))%SAMPLES ;
}
