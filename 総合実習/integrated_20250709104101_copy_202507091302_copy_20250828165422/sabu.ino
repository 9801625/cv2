#include <FlexCAN_T4.h>
#include <Wire.h>  // I2C通信を行うためのライブラリ
#include <Adafruit_PWMServoDriver.h>  // PCA9685を使うためのAdafruit公式ライブラリ
//SDA（データ） → Teensy 4.0 の ピン 18
//SCL（クロック） → Teensy 4.0 の ピン 19
// PCA9685のインスタンスを作成（I2Cアドレス0x40をデフォルトとして使用）
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Teensy 4.0 の CAN1 を使用（RX=22, TX=23）
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// サーボのパルス幅（4096ステップ中の値）
#define SERVOMIN  150  // サーボの最小パルス幅（最も左、または最も上など）
#define SERVOMAX  600  // サーボの最大パルス幅（最も右、または最も下など）

// writeMicroseconds()を使うときのマイクロ秒単位のパルス幅
#define USMIN  600     // 約150に対応（＝0.6ms）
#define USMAX  2400    // 約600に対応（＝2.4ms）

#define SERVO_FREQ 50  // サーボモーターは通常50Hz（20ms周期）で駆動

// 現在動作中のサーボチャンネル
uint8_t servonum = 0;
uint8_t servonum1 = 1;

constexpr int motor_id_gm = 1;
constexpr uint32_t can_tx_id_gm = 0x1FF;
constexpr uint32_t mode_switch_id_gm = 0x700 + motor_id_gm;

constexpr int motor_id_1 = 1;
constexpr uint32_t can_tx_id = 0x200;
constexpr uint32_t mode_switch_id_1 = 0x700 + motor_id_1;

constexpr int motor_id_2 = 2;
constexpr uint32_t mode_switch_id_2 = 0x700 + motor_id_2;

float current_A = 0.0;  // 初期電流

int d_in0=7; //もともと10  7
int d_in1=8; //もともと9  8
int d_in2=9; //もともと8  9
int d_in3=10; //もともと7  10

int d_in0_state=0,d_in1_state=0,d_in2_state=0,d_in3_state=0;//ESP32の出力が接続されたピン
int d_integrated=0;//ESP32の信号を統合したもの(現在4ビット)

uint16_t microsec = 1500;
uint16_t microsec1 = 1500;

// モード切り替えコマンド（最初に1回送る）
void send_mode_switch_command() {
  CAN_message_t msg;
  msg.id = mode_switch_id_1;
  msg.id = mode_switch_id_2;
  msg.len = 8;
  msg.flags.extended = 0;
  msg.buf[0] = 0xFC;
  for (int i = 1; i < 8; i++) msg.buf[i] = 0x00;

  if (Can0.write(msg)) {
    Serial.println("Mode Switch: OK");
  } else {
    Serial.println("Mode Switch: NO");
  }

  delay(10);
}

// M2006に電流指令を送る
void send_current_to_motor(float current,int motor_id,uint32_t can_tx_id) {
  constexpr float MAX_CUR = 10.0;
  constexpr int MAX_CUR_VAL = 10000;

  float val = current * (MAX_CUR_VAL / MAX_CUR);
  val = constrain(val, -MAX_CUR_VAL, MAX_CUR_VAL);
  int16_t transmit_val = static_cast<int16_t>(val);

  CAN_message_t msg;
  msg.id = can_tx_id;
  msg.len = 8;
  msg.flags.extended = 0;
  for (int i = 0; i < 8; i++) msg.buf[i] = 0;

  msg.buf[(motor_id - 1) * 2]     = (transmit_val >> 8) & 0xFF;
  msg.buf[(motor_id - 1) * 2 + 1] = transmit_val & 0xFF;

  if (Can0.write(msg)) {
    Serial.println("Current Command: OK");
  } else {
    Serial.println("Current Command: NO");
  }
}

void setup() {
  Serial.begin(115200);
  //while (!Serial);

  // CANの初期化
  Can0.begin();
  Can0.setBaudRate(1000000);  // 1 Mbps
  Can0.setMaxMB(16);          // メールボックス数
  Can0.enableFIFO();          // FIFO受信（今回は使わないけど推奨）
  Can0.enableFIFOInterrupt();
  Can0.onReceive(nullptr);    // 受信コールバック不要

  delay(500);  // 安定化待ち

  Serial.println("CAN initialized. Sending mode switch...");
  send_mode_switch_command();
  //------ここまでがCANの初期設定動作。以降PCA9685の初期設定動作。
  pwm.begin();  // PCA9685の初期化（I2C通信開始）
  // 内部オシレータの周波数を明示的に設定（デフォルトは25MHzだが誤差がある）
  pwm.setOscillatorFrequency(27000000);  // 27MHzに調整（必要に応じて変更）
  // サーボに合わせたPWM周波数を設定（通常50Hz）
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);  // 初期化後の小休止（安定化のため）
  // ESP32からの入力信号
  pinMode(d_in0,INPUT_PULLUP);
  pinMode(d_in1,INPUT_PULLUP);
  pinMode(d_in2,INPUT_PULLUP);
  pinMode(d_in3,INPUT_PULLUP);
  pwm.writeMicroseconds(0, 1500);
  pwm.writeMicroseconds(1, 1500);
}

// 秒単位でパルス幅を設定する関数（あまり正確ではない）
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;  // 1秒 = 1,000,000マイクロ秒
  pulselength /= SERVO_FREQ;  // 1周期あたりのマイクロ秒
  Serial.print(pulselength); Serial.println(" us per period");

  pulselength /= 4096;  // 1ステップあたりの時間（12ビット分解能）
  Serial.print(pulselength); Serial.println(" us per bit");

  pulse *= 1000000;  // 秒 → マイクロ秒
  pulse /= pulselength;  // ステップ数に変換

  Serial.println(pulse);  // デバッグ表示
  pwm.setPWM(n, 0, pulse);  // サーボn番にパルスを出力
}

void loop() {
  d_in0_state=digitalRead(d_in0);
  d_in1_state=digitalRead(d_in1);
  d_in2_state=digitalRead(d_in2);
  d_in3_state=digitalRead(d_in3);
  d_in1_state=d_in1_state<<1;
  d_in2_state=d_in2_state<<2;
  d_in3_state=d_in3_state<<3;
  d_integrated=d_in0_state+d_in1_state+d_in2_state+d_in3_state;
  //めんどくさいのでビット反転してない
  //unsigned long now = millis();
  switch(d_integrated){
    case 0x01:
      //通常、無入力なら0x00になってる
      current_A = 2.0;
      send_current_to_motor(current_A,motor_id_1,can_tx_id);
      delay(10);  // 周期制御（10ms）
      break;
    case 0x02:
      current_A = -2.0;
      send_current_to_motor(current_A,motor_id_1,can_tx_id);
      delay(10);  // 周期制御（10ms）
      break;
    case 0x03:
      current_A = 3.0;
      send_current_to_motor(current_A,motor_id_gm,can_tx_id_gm);
      delay(10);
      break;
    case 0x04:
      current_A = -3.0;
      send_current_to_motor(current_A,motor_id_gm,can_tx_id_gm);
      delay(10);
      break;
    case 0x05:
      if(microsec<USMAX){
        microsec=microsec+10;
      }
      pwm.writeMicroseconds(servonum, microsec);
      delay(10);  // 周期制御（10ms）
      break;
    case 0x06:
      if(microsec>USMIN){
        microsec=microsec-10;
      }
      pwm.writeMicroseconds(servonum, microsec);
      delay(10);  // 周期制御（10ms）
      break;
    case 0x07:
      if(microsec1<USMAX){
        microsec1=microsec1+10;
      }
      pwm.writeMicroseconds(servonum1, microsec1);
      delay(10);  // 周期制御（10ms）
      break;
    case 0x08:
      if(microsec1>USMIN){
        microsec1=microsec1-10;
      }
      pwm.writeMicroseconds(servonum1, microsec1);
      delay(10);  // 周期制御（10ms）
      break;
    case 0x09:
      current_A = 2.0;
      send_current_to_motor(current_A,motor_id_2,can_tx_id);
      delay(10);  // 周期制御（10ms）
      break;
    case 0x0A:
      current_A = -2.0;
      send_current_to_motor(current_A,motor_id_2,can_tx_id);
      delay(10);  // 周期制御（10ms）
      break;
    default:
      //無入力状態。アームを現状維持できるようにする
      current_A = 0.0;
      send_current_to_motor(current_A,motor_id_gm,can_tx_id_gm);
      send_current_to_motor(current_A,motor_id_1,can_tx_id);
      send_current_to_motor(current_A,motor_id_2,can_tx_id);
      send_current_to_motor(current_A,motor_id_3,can_tx_id);
      delay(10);  // 周期制御（10ms）
      break;
  }

}