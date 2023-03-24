#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPL3115A2.h>
#include <TinyGPSPlus.h>
#include <LoRa_E32.h>
#include <SD.h>
#include <EEPROM.h>
#include "kalmanfilter4d.h"
#include <Ewma.h>

#define PIN_BME280_SPI_CS 15
#define PIN_SDCARD_SPI_CS 15
// #define PIN_LORA_TX
// #define PIN_LORA_RX
#define PINO_TETIKLEME 1
#define PIN_LORA_AUX 1
#define PIN_LORA_M0 1
#define PIN_LORA_M1 1

#define GPS_BAUD 9600
#define GPS_DEAD_TIME 100
#define CLAIBRATION_LOOP 200

#define LORA_ADDH 0X00
#define LORA_ADDL 0X00
#define LORA_CHN 0X00

#define EEPROM_ADDR 1

#define BASINC_MIN    700.0f
#define BASINC_MAX    1100.0f
#define SICAKLIK_MIN  0.0f
#define SICAKLIK_MAX  40.0f

#define ATESLEME_IVME_ESIGI       9.8f
#define ATESLEME_YUKSEKLIK_ESIGI  10.0f
#define APOGEE_IVME_ESIGI         5.0f

#define IVMELENME_YUKARI 0
#define IVMELENME_ASAGI  1
// #define PIN_BME280_SPI_MISO
// #define PIN_BME280_SPI_MOSI
// #define PIN_BME280_SPI_SCK
void initUART();
void initPeriph();
void initPins();
void initFilters();
bool checkEmerg();
void emergInit();
void calibratePressure();
void readPressure(bool wAltitude);
void readGPS();
void readAcc();
void calcAlt();
void getLoraConfig();
void printParameters(struct Configuration configuration);
void veriPaketiOlustur();
void veriPaketiGonder();
void SD_write(String input);
//  Lora E32
//  GPS neo6m x2
//  MPL 3115A2
//  BME280
//  SDcard
//  IMU
// 1500m parachute
// telemtri roket -> basinc ivme gps, faydali yuk -> gps

template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

TinyGPSPlus gps;
Adafruit_BME280 basinc_bme280(PIN_BME280_SPI_CS);
Adafruit_MPL3115A2 basinc_mpl31;
LoRa_E32 lora(Serial3, PIN_LORA_AUX, PIN_LORA_M0, PIN_LORA_M1, UART_BPS_RATE_115200);

// LowPass<2> LP_Altitude(1e2, 1e3, true);
// LowPass<2> LP_Press(1e2, 1e3, true);
Ewma EMA_Alt(0.1);   // Less smoothing - faster to detect changes, but more prone to noise

typedef struct PressureStruct
{
  float basinc_hpa = 0.0, sicaklik_c = 0.0, yukseklik_m = 0.0, yerBasinci_hpa = 0.0, yerYuksekligi_m = 0.0, mutlakBasinc_hpa = 0.0, mutlakYukseklik_m = 0.0;
  float lastPress = 0.0;
  bool veriGecerliMi = true;
} pressure_typedef;

typedef struct AccStruct
{
  int ax = 0, ay = 0, az = 0;
  float compAcc;
  uint8_t ivmelenmeYonu;
} acc_typedef;

typedef struct GpsStruct
{
  float lat = 0.0, lng = 0.0, yukseklik = 0.0;
  int uydu_sayisi = 0;
  bool veri_gecerli_mi = false;
} gps_typedef;

typedef struct HardwareStruct
{
  bool basincBME280 = false, basincMPL31 = false, loraModulu = false, SDKart = false;
} hardware_typedef;

typedef struct LogFileStruct
{ 
  File logDosyasi;
  String logDosyaAdi;
  int paketNo = 0;
  String veriPaketi;
  ResponseContainer lora_cvp;
}logFile_typedef;

enum ROKET_DURUMU{
  ROKET_YERDE,
  ROKET_ATESLENDI,
  ROKET_APOGEE,
  ROKET_INIS,
  ROKET_BALISTIK
};

typedef struct kalmanStruct{
float yukseklik_cm;
float ivme_cm_s2;
float kf_out_yukseklik_m;
// float kf_out_hiz_m_s;
unsigned long timeStamp;
}kalman_typedef;

typedef struct apogeeStruct{
  float max_yukseklik = 0;
  unsigned long yukseklik_time = 0;
  bool timer_aktif = false;
  bool apogee_ok = false;
  unsigned int apogee_suresi_ms = 500;
}apogee_typedef;

typedef struct
{
  pressure_typedef basincSensoru_MPL;
  pressure_typedef basincSensoru_BME;
  gps_typedef gpsModulu;
  acc_typedef ivmeolcerSensoru;
  hardware_typedef donanimDurumu;
  logFile_typedef telemetri;
  kalman_typedef kalmanParam;
  ROKET_DURUMU roketDurumu;
  apogee_typedef apogeeVerisi;
} sistem_typedef;

sistem_typedef FezaRoketSistemi;

unsigned long telemetriTimer = millis();
unsigned long gpsTimer = millis();
unsigned long sensorTimer = millis();

void setup()
{
  if(checkEmerg())
    {
      // todo emergInit();
    }
  else
  {
    initUART();
    initPeriph();
    initPins();
    initFilters();
    calibratePressure();
    FezaRoketSistemi.roketDurumu = ROKET_YERDE;
  }
  FezaRoketSistemi.kalmanParam.timeStamp = micros();
}

void loop()
{
  if (millis() - sensorTimer > 10)
  {
    readPressure(true);
    readAcc();
    calcAlt();
    sensorTimer = millis();
  }
  if (millis() - gpsTimer > 500)
  {
    readGPS();
    gpsTimer = millis();
  }
  if (millis() - telemetriTimer > 1000)
  {
    veriPaketiOlustur();
    veriPaketiGonder(); // Hem telemetriden gonderiyor hem de sd karta kaydediyor
    telemetriTimer = millis();
  }
  switch (FezaRoketSistemi.roketDurumu) 
  {
    case ROKET_YERDE:
    {
      if(FezaRoketSistemi.ivmeolcerSensoru.compAcc>=ATESLEME_IVME_ESIGI && FezaRoketSistemi.kalmanParam.kf_out_yukseklik_m>=ATESLEME_YUKSEKLIK_ESIGI)
      {
        FezaRoketSistemi.roketDurumu = ROKET_ATESLENDI;
        EEPROM.write(EEPROM_ADDR, 1);
        break;
      }
    }
    case ROKET_ATESLENDI:
    {
      // todo apogeeKontrol();
      if((FezaRoketSistemi.kalmanParam.kf_out_yukseklik_m>FezaRoketSistemi.apogeeVerisi.max_yukseklik * 1.20) && FezaRoketSistemi.ivmeolcerSensoru.compAcc > ATESLEME_IVME_ESIGI && FezaRoketSistemi.ivmeolcerSensoru.ivmelenmeYonu==IVMELENME_YUKARI)
      {
        FezaRoketSistemi.apogeeVerisi.max_yukseklik = FezaRoketSistemi.kalmanParam.kf_out_yukseklik_m;
        FezaRoketSistemi.apogeeVerisi.timer_aktif = false;
      }
      else{
        float topIvme = FezaRoketSistemi.ivmeolcerSensoru.ax + FezaRoketSistemi.ivmeolcerSensoru.ay + FezaRoketSistemi.ivmeolcerSensoru.az;
        if(FezaRoketSistemi.apogeeVerisi.timer_aktif == false)
        {
          FezaRoketSistemi.apogeeVerisi.yukseklik_time = millis();
          FezaRoketSistemi.apogeeVerisi.timer_aktif = true;
        }
        else if((millis() - FezaRoketSistemi.apogeeVerisi.yukseklik_time > FezaRoketSistemi.apogeeVerisi.apogee_suresi_ms) && ((-5.0<topIvme && topIvme<5.0) || (FezaRoketSistemi.ivmeolcerSensoru.compAcc<APOGEE_IVME_ESIGI && FezaRoketSistemi.ivmeolcerSensoru.ivmelenmeYonu==IVMELENME_ASAGI)))
        {
         digitalWrite(PINO_TETIKLEME, HIGH);
         delay(1000);
         digitalWrite(PINO_TETIKLEME, LOW);
         
         FezaRoketSistemi.roketDurumu = ROKET_APOGEE;
         break;
        }
        // todo balistik dusme konrolu
      }
      break;
    }
    case ROKET_APOGEE:
    {
      // todo balistik dusme kontrolu eger parasut ilk tetiklemede acilmazsa balistik durumuna gec
      if(FezaRoketSistemi.apogeeVerisi.apogee_ok == false)
      {
        FezaRoketSistemi.apogeeVerisi.apogee_ok = true;
        EEPROM.write(EEPROM_ADDR, 2);
        FezaRoketSistemi.roketDurumu = ROKET_INIS;
      }
      break;
    }
    case ROKET_BALISTIK:
    {
      /** barutu 2 defa daha uzun surelerle tetiklemek **/
      digitalWrite(PINO_TETIKLEME, HIGH);
      delay(2000);
      digitalWrite(PINO_TETIKLEME, LOW);
      delay(1000);
      digitalWrite(PINO_TETIKLEME, HIGH);
      delay(2000);
      digitalWrite(PINO_TETIKLEME, LOW);
      // todo roket dusme kontrolu eger balistik devam ederse bir kac defa barutu tetiklemeye calis olmazsa inise gec
      FezaRoketSistemi.roketDurumu = ROKET_INIS;
      break;
    }
    case ROKET_INIS:
    {

      break;
    }
  }
}

void readPressure(bool wAltitude)
{
  if (FezaRoketSistemi.donanimDurumu.basincBME280 == true)
  {
    FezaRoketSistemi.basincSensoru_BME.basinc_hpa = basinc_bme280.readPressure() / 100.0F;
    FezaRoketSistemi.basincSensoru_BME.yukseklik_m = basinc_bme280.readAltitude(FezaRoketSistemi.basincSensoru_BME.yerBasinci_hpa);
    FezaRoketSistemi.basincSensoru_BME.sicaklik_c = basinc_bme280.readTemperature();
    FezaRoketSistemi.basincSensoru_BME.mutlakBasinc_hpa = abs(FezaRoketSistemi.basincSensoru_BME.yerBasinci_hpa - FezaRoketSistemi.basincSensoru_BME.basinc_hpa);
     FezaRoketSistemi.basincSensoru_BME.veriGecerliMi = (FezaRoketSistemi.basincSensoru_BME.basinc_hpa >= BASINC_MIN && FezaRoketSistemi.basincSensoru_BME.basinc_hpa <= BASINC_MAX) && (FezaRoketSistemi.basincSensoru_BME.sicaklik_c >= SICAKLIK_MIN && FezaRoketSistemi.basincSensoru_BME.sicaklik_c <= SICAKLIK_MAX);

    FezaRoketSistemi.basincSensoru_BME.mutlakYukseklik_m = abs(FezaRoketSistemi.basincSensoru_BME.yukseklik_m - FezaRoketSistemi.basincSensoru_BME.yerYuksekligi_m);
    FezaRoketSistemi.basincSensoru_BME.mutlakYukseklik_m = basinc_bme280.readAltitude(FezaRoketSistemi.basincSensoru_BME.yerBasinci_hpa);
  }
  if (FezaRoketSistemi.donanimDurumu.basincMPL31 == true)
  {
    FezaRoketSistemi.basincSensoru_MPL.basinc_hpa = basinc_mpl31.getPressure();
    FezaRoketSistemi.basincSensoru_MPL.yukseklik_m = basinc_mpl31.getAltitude();
    FezaRoketSistemi.basincSensoru_MPL.sicaklik_c = basinc_mpl31.getTemperature();
    FezaRoketSistemi.basincSensoru_MPL.veriGecerliMi = (FezaRoketSistemi.basincSensoru_MPL.basinc_hpa >= BASINC_MIN && FezaRoketSistemi.basincSensoru_MPL.basinc_hpa <= BASINC_MAX) && (FezaRoketSistemi.basincSensoru_MPL.sicaklik_c >= SICAKLIK_MIN && FezaRoketSistemi.basincSensoru_MPL.sicaklik_c <= SICAKLIK_MAX);
   
    FezaRoketSistemi.basincSensoru_MPL.mutlakBasinc_hpa = abs(FezaRoketSistemi.basincSensoru_MPL.yerBasinci_hpa - FezaRoketSistemi.basincSensoru_MPL.basinc_hpa);
    FezaRoketSistemi.basincSensoru_MPL.mutlakYukseklik_m = abs(FezaRoketSistemi.basincSensoru_MPL.yukseklik_m - FezaRoketSistemi.basincSensoru_MPL.yerYuksekligi_m);
  }
  if(!wAltitude) return;

  if(FezaRoketSistemi.basincSensoru_BME.veriGecerliMi && FezaRoketSistemi.basincSensoru_MPL.veriGecerliMi)
  {
    float yukseklik = FezaRoketSistemi.basincSensoru_BME.mutlakYukseklik_m * 0.4 + FezaRoketSistemi.basincSensoru_MPL.mutlakYukseklik_m * 0.6;
    yukseklik *= 100.0f;
    FezaRoketSistemi.kalmanParam.yukseklik_cm = EMA_Alt.filter(yukseklik);
  }
  else if(FezaRoketSistemi.basincSensoru_BME.veriGecerliMi)
  {
    float yukseklik = FezaRoketSistemi.basincSensoru_BME.mutlakYukseklik_m;
    yukseklik *= 100.0f;
    FezaRoketSistemi.kalmanParam.yukseklik_cm = EMA_Alt.filter(yukseklik);
  }
  else{
    float yukseklik = FezaRoketSistemi.basincSensoru_MPL.mutlakYukseklik_m;
    yukseklik *= 100.0f;
    FezaRoketSistemi.kalmanParam.yukseklik_cm = EMA_Alt.filter(yukseklik);
  }
}
void readGPS()
{
  unsigned long t1 = millis();
  if (Serial1.available())
  {
    while (Serial1.available())
    {
      gps.encode(Serial1.read());
      FezaRoketSistemi.gpsModulu.veri_gecerli_mi = true;
      if (millis() - t1 > GPS_DEAD_TIME)
      {
        FezaRoketSistemi.gpsModulu.veri_gecerli_mi = false;
        return;
      }
    }
    if (FezaRoketSistemi.gpsModulu.veri_gecerli_mi == true)
    {
      if (gps.location.isValid())
      {
        FezaRoketSistemi.gpsModulu.lat = gps.location.lat();
        FezaRoketSistemi.gpsModulu.lng = gps.location.lng();
      }
      if (gps.satellites.isValid())
      {
        FezaRoketSistemi.gpsModulu.uydu_sayisi = gps.satellites.value();
      }
      if (gps.altitude.isValid())
      {
        FezaRoketSistemi.gpsModulu.yukseklik = gps.altitude.meters();
      }
    }
  }
}
void readAcc()
{
  int ax = 0, ay = 0, az = 0;
  /**
   ** ivmeolcer sensorunden veri okuma kodlari
   **/
  FezaRoketSistemi.ivmeolcerSensoru.compAcc = sqrt(ax * ax + ay * ay + az * az);
  FezaRoketSistemi.ivmeolcerSensoru.ax = ax;
  FezaRoketSistemi.ivmeolcerSensoru.ay = ay;
  FezaRoketSistemi.ivmeolcerSensoru.az = az;
  FezaRoketSistemi.ivmeolcerSensoru.ivmelenmeYonu = (ax+ay+az)<0; //toplam ivme sifirdan kucukse sart ifades true=1 olur ve bu da define edilen ivme yonu asagiya denk gelmektedir
}
void calcAlt()
{
  FezaRoketSistemi.kalmanParam.ivme_cm_s2 = FezaRoketSistemi.ivmeolcerSensoru.compAcc * 100.0f; //assuming acceleration in m/s/s
  kalmanFilter4d_predict((micros()-FezaRoketSistemi.kalmanParam.timeStamp) / 1000000.0f);
  float kf_yukseklik_cm, kf_hiz_cm_s;
  kalmanFilter4d_update(FezaRoketSistemi.kalmanParam.yukseklik_cm, FezaRoketSistemi.kalmanParam.ivme_cm_s2, &kf_yukseklik_cm, &kf_hiz_cm_s);
  FezaRoketSistemi.kalmanParam.timeStamp = micros();
  FezaRoketSistemi.kalmanParam.kf_out_yukseklik_m = kf_yukseklik_cm * 100.0;
  // FezaRoketSistemi.kalmanParam.kf_out_hiz_m_s = kf_hiz_cm_s * 100.0;
  Serial.print("********Kalman filtresi verileri********\n");
  Serial.print("**\tBME sensoru mutlak yuksekligi: ");Serial.println(FezaRoketSistemi.basincSensoru_BME.mutlakYukseklik_m);
  Serial.print("**\tMPL sensoru mutlak yuksekligi: ");Serial.println(FezaRoketSistemi.basincSensoru_MPL.mutlakYukseklik_m);
  Serial.print("**\tFiltre icin hesaplanan mutlak yukseklik cm: ");Serial.println(FezaRoketSistemi.kalmanParam.yukseklik_cm);
  Serial.print("**\tFiltre icin hesaplanan ivme cm/s/s: ");Serial.println(FezaRoketSistemi.kalmanParam.ivme_cm_s2);
  Serial.println("\t******Filtre Ciktisi******");
  Serial.print("\tYUKSEKLIK cm: ");Serial.println(FezaRoketSistemi.kalmanParam.kf_out_yukseklik_m);
  // Serial.print("\tHIZ cm/s: ");Serial.println(FezaRoketSistemi.kalmanParam.kf_out_hiz_m_s);
  Serial.print("************************************\n");
}
void calibratePressure()
{
  float basinc1_temp = 0, basinc2_temp = 0, yukseklik1_temp = 0, yukseklik2_temp = 0;
  delay(5000);
  for (size_t i = 0; i < CLAIBRATION_LOOP; i++)
  {
    readPressure(false);
    basinc1_temp += FezaRoketSistemi.basincSensoru_BME.basinc_hpa;
    basinc2_temp += FezaRoketSistemi.basincSensoru_MPL.basinc_hpa;
    yukseklik1_temp += FezaRoketSistemi.basincSensoru_BME.yukseklik_m;
    yukseklik2_temp += FezaRoketSistemi.basincSensoru_MPL.yukseklik_m;
  }
  FezaRoketSistemi.basincSensoru_BME.yerBasinci_hpa = basinc1_temp / CLAIBRATION_LOOP;
  FezaRoketSistemi.basincSensoru_BME.yerYuksekligi_m = yukseklik1_temp / CLAIBRATION_LOOP;
  FezaRoketSistemi.basincSensoru_MPL.yerBasinci_hpa = basinc2_temp / CLAIBRATION_LOOP;
  FezaRoketSistemi.basincSensoru_MPL.yerYuksekligi_m = yukseklik2_temp / CLAIBRATION_LOOP;
}


void initUART()
{
  /*
  *******************
  * serial ayarlari *
  *******************
  */
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("DEBUG Serial OK");
   
  /*
  *****************************************************
  * GPS serial ayarlari ve kutuphane version kontrolu *
  *****************************************************
  */
  Serial1.begin(GPS_BAUD);
  while (!Serial1)
    ;
  Serial.println("GPS Serial OK");
  Serial.print("GPS Plus library test -> ");
  Serial.println(_GPS_VERSION); // Hata verirse dogru GPS kutuphanesi eklenmemis demektir

  /*
  *******************
  * LORA ayarlari *
  *******************
  */
  if (lora.begin()) //LORA modulunu baslatma
  {
    Serial.println("LoRa modulu OK");
    getLoraConfig();                                              //lora configurasyonlarini okuma
    while (FezaRoketSistemi.donanimDurumu.loraModulu == false)    //yer istasyonundan gecerli mesaj gelene kadar
    {
      Serial.println("Yer istasyonu bekleniyor ...");
      lora.sendFixedMessage(LORA_ADDH, LORA_ADDL, LORA_CHN, "hello from rocket");
      // ! yer istasyonu aynisini gondermesi lazim
      char initMsg[] = "hello from gs";   /*yer istasyonundan gelmesi gereken mesaj*/
      unsigned long t1 = millis();
      while (millis() - t1 < 1000)    // 1 saniye boyunca mesaji bekle
        ;
      if (!lora.available())          // eger mesaj gelmemisse devam et
        continue;
      FezaRoketSistemi.telemetri.lora_cvp = lora.receiveInitialMessage(sizeof(char) * strlen(initMsg));
      if ((FezaRoketSistemi.telemetri.lora_cvp.status.code == E32_SUCCESS) && (FezaRoketSistemi.telemetri.lora_cvp.data.equals(initMsg)))   //Beklenen mesaj dogru bir sekilde gelmis ise
      {
        Serial.println("Yer istasyonu ile baglanti basarili");
        FezaRoketSistemi.donanimDurumu.loraModulu = true;
        break;
      }
      else
      {
        Serial.print("LoRa yer istasyonu kontrol mesaji alinirken hata olustu ==> Hata kodu: ");
        Serial.println(FezaRoketSistemi.telemetri.lora_cvp.status.getResponseDescription());
        Serial.print("\t==>Alinan mesaj: ");
        Serial.println(FezaRoketSistemi.telemetri.lora_cvp.data);
        FezaRoketSistemi.donanimDurumu.loraModulu = false;
      }
    }
  }
  else
  {
    Serial.println("LoRa modulu ERR");
  }
}
void initPeriph()
{
  /*
   *********************************
   * BME basinc sensorunu baslatma *
   *********************************
  */
  if (basinc_bme280.begin())
  {
    Serial.println("BME280 OK"); // Sensor ile baglanti basarili
    FezaRoketSistemi.donanimDurumu.basincBME280 = true;
  }
  else
  {
    Serial.println("BME280 ERR"); // Sensor ile baglanti basarisiz
  }

  /*
   *************************************
   * MPL3115 basinc sensorunu baslatma *
   *************************************
  */
  if (basinc_mpl31.begin())
  {
    Serial.println("MPL31 OK"); // Sensor ile baglanti basarili
    FezaRoketSistemi.donanimDurumu.basincMPL31 = true;
    basinc_mpl31.setMode(MPL3115A2_BAROMETER);
  }
  else
  {
    Serial.println("MPL31 ERR"); // Sensor ile baglanti basarisiz
  }

  
  /*
   *****************************
   * SD Card modulunu baslatma *
   *****************************
  */
  if (SD.begin(PIN_SDCARD_SPI_CS))
  {
    Serial.println("SD modulu OK"); // Sensor ile baglanti basarili
    FezaRoketSistemi.donanimDurumu.SDKart = true;
  }
  else
  {
    Serial.println("SD modulu ERR"); // Sensor ile baglanti basarisiz
  }
  int fileNum = 0;
  String filename = "roketVeriLog_";
  String fileExt = ".csv";
  if (SD.exists("./" + filename + String(fileNum) + fileExt))
  {
    Serial.print(filename + String(fileNum + fileExt + " dosyasi bulundu\n"));
    while (SD.exists("./" + filename + String(++fileNum) + fileExt)) // Dosya sayisi cok fazla olmasin sd kartta
    {
      Serial.print(filename + String(fileNum + fileExt + " dosyasi bulundu\n"));
    }
    FezaRoketSistemi.telemetri.logDosyasi = SD.open("./" + filename + String(fileNum) + fileExt, FILE_WRITE);
    FezaRoketSistemi.telemetri.logDosyasi.close();
    if (SD.exists("./" + filename + String(fileNum) + fileExt))
    {
      Serial.print("******* ");
      Serial.print("./" + filename + String(fileNum) + fileExt);
      Serial.print(" dosyasi basariyla olusturuldu");
      Serial.println(" *******");
      FezaRoketSistemi.telemetri.logDosyaAdi = "./" + filename + String(fileNum) + fileExt;
      SD_write("Paket no,Basinc 1,Basinc 2,Yukseklik 1,Yukseklik 2,GPS lat,GPS lng\n");
    }
    else
    {
      Serial.println("LOG dosyasi olusuturlamadi\tSD kartini kontrol edin");
      FezaRoketSistemi.donanimDurumu.SDKart = false;
    }
  }
}
void initPins()
{
  pinMode(PINO_TETIKLEME, OUTPUT);
  digitalWrite(PINO_TETIKLEME, LOW);
}
void initFilters()
{
  kalmanFilter4d_configure(1000.0f * KF_ACCELBIAS_VARIANCE, KF_ADAPT, FezaRoketSistemi.kalmanParam.yukseklik_cm, 0.0f, 0.0f);
}

bool checkEmerg()
{
  uint8_t eepromVal = EEPROM.read(EEPROM_ADDR);
  if (eepromVal == 255 || eepromVal == 0)
  {
    return false;
    FezaRoketSistemi.roketDurumu = ROKET_YERDE;
  }
  else if(eepromVal == 1)
  {
    FezaRoketSistemi.roketDurumu = ROKET_ATESLENDI;
  }
  else if(eepromVal == 2){
    FezaRoketSistemi.roketDurumu = ROKET_INIS;
  }
  return true;
}

void getLoraConfig()
{
  ResponseStructContainer c;
  c = lora.getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration *)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  printParameters(configuration);

  ResponseStructContainer cMi;
  cMi = lora.getModuleInformation();
  // It's important get information pointer before all other operation
  ModuleInformation mi = *(ModuleInformation *)cMi.data;

  Serial.println(cMi.status.getResponseDescription());
  Serial.println(cMi.status.code);
  c.close();
  cMi.close();
}
void printParameters(struct Configuration configuration)
{
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD BIN: "));
  Serial.print(configuration.HEAD, BIN);
  Serial.print(" ");
  Serial.print(configuration.HEAD, DEC);
  Serial.print(" ");
  Serial.println(configuration.HEAD, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH BIN: "));
  Serial.println(configuration.ADDH, BIN);
  Serial.print(F("AddL BIN: "));
  Serial.println(configuration.ADDL, BIN);
  Serial.print(F("Chan BIN: "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit BIN    : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDataRate BIN : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRate());
  Serial.print(F("SpeedAirDataRate BIN  : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRate());

  Serial.print(F("OptionTrans BIN       : "));
  Serial.print(configuration.OPTION.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFixedTransmissionDescription());
  Serial.print(F("OptionPullup BIN      : "));
  Serial.print(configuration.OPTION.ioDriveMode, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getIODroveModeDescription());
  Serial.print(F("OptionWakeup BIN      : "));
  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
  Serial.print(F("OptionFEC BIN         : "));
  Serial.print(configuration.OPTION.fec, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFECDescription());
  Serial.print(F("OptionPower BIN       : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());

  Serial.println("----------------------------------------");
}

void veriPaketiOlustur()
{
  /**
   * https://www.javainuse.com/bytesize adresinden yazdirilacak paket uzunlugu kontrol edilebilir
   * paket boyutu uzunlugundan daha fazla olmasi gerekir
   * en az 2 kati olursa iyi olur
   **/
  char paket[300] = {'\0'}; // Paket no,Basinc 1,Basinc 2,Yukseklik 1,Yukseklik 2,GPS lat,GPS lng -> tahmini paket boyutu 65Bytes

  snprintf(paket, 300, "%04d,%05.2f,%05.2f,%05.2f,%05.2f,%02.8f,%02.8f\n",
           ++FezaRoketSistemi.telemetri.paketNo,
           (double)FezaRoketSistemi.basincSensoru_BME.basinc_hpa,
           (double)FezaRoketSistemi.basincSensoru_MPL.basinc_hpa,
           (double)FezaRoketSistemi.basincSensoru_BME.mutlakYukseklik_m,
           (double)FezaRoketSistemi.basincSensoru_MPL.mutlakYukseklik_m,
           (double)FezaRoketSistemi.gpsModulu.lat,
           (double)FezaRoketSistemi.gpsModulu.lng);
  FezaRoketSistemi.telemetri.veriPaketi = String(paket);
}
void veriPaketiGonder()
{
  lora.sendFixedMessage(LORA_ADDH, LORA_ADDL, LORA_CHN, FezaRoketSistemi.telemetri.veriPaketi);
  SD_write(FezaRoketSistemi.telemetri.veriPaketi);
}
void SD_write(String input)
{
  FezaRoketSistemi.telemetri.logDosyasi = SD.open(FezaRoketSistemi.telemetri.logDosyaAdi, FILE_WRITE);
  if (!FezaRoketSistemi.telemetri.logDosyasi)
  {
    FezaRoketSistemi.telemetri.logDosyasi.close();
    FezaRoketSistemi.telemetri.logDosyasi = SD.open(FezaRoketSistemi.telemetri.logDosyaAdi, FILE_WRITE);
    if (!FezaRoketSistemi.telemetri.logDosyasi)
    {
      Serial.println("Veriler SD kartindaki LOG dosyasina yazilamadi...");
    }
    else
    {
      FezaRoketSistemi.telemetri.logDosyasi.print(input);
    }
  }
  else
  {
    FezaRoketSistemi.telemetri.logDosyasi.print(input);
  }
  FezaRoketSistemi.telemetri.logDosyasi.close();
}