//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2017-10-24 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2021-03-06 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=328p aes=no

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER
// #define NDEBUG
#define HIDE_IGNORE_MSG

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <Switch.h>
#include <INA219_WE.h> //https://github.com/jp112sdl/INA219_WE

#define MEASURE_INTERVAL            1
#define POWERMETER_CYCLIC_INTERVAL  120

#define SWITCH_PIN          6
#define INVERT_SWITCH_PIN   true //true = ACTIVE LOW
#define BUTTON_PIN          8
#define LED_PIN             4
#define ANALOG_VOLTAGE_PIN A2

// number of available peers per channel
#define PEERS_PER_SWCHANNEL     8
#define PEERS_PER_PMCHANNEL     8
#define PEERS_PER_SENSORCHANNEL 8

// all library classes are placed in the namespace 'as'
using namespace as;

typedef struct {
  uint16_t Current   = 0;
  uint16_t Voltage   = 0;
} sensorValues;

sensorValues actualValues ;
sensorValues lastValues;
uint8_t averaging = 1;
bool    resetAverageCounting = false;
// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xf3, 0x5a, 0x01},     // Device ID
  "HBPMSwUSB0",           // Device Serial
  {0xf3, 0x5a},           // Device Model
  0x10,                   // Firmware Version
  as::DeviceType::Switch, // Device Type
  {0x01, 0x00}            // Info Bytes
};

// Configure the used hardware
typedef AvrSPI<10, 11, 12, 13> RadioSPI;
typedef AskSin<StatusLed<LED_PIN>, NoBattery, Radio<RadioSPI, 2> > Hal;
Hal hal;

DEFREGISTER(Reg0, MASTERID_REGS, DREG_INTKEY, DREG_LOCALRESETDISABLE)
class PMSw1List0 : public RegList0<Reg0> {
  public:
    PMSw1List0(uint16_t addr) : RegList0<Reg0>(addr) {}
    void defaults () {
      clear();
    }
};

bool outputOn() {
  return (digitalRead(SWITCH_PIN) == (INVERT_SWITCH_PIN == true ? LOW:HIGH));
}
typedef SwitchChannel<Hal, PEERS_PER_SWCHANNEL, PMSw1List0> SwChannel;

DEFREGISTER(MReg1, CREG_AES_ACTIVE, CREG_AVERAGING, CREG_TX_MINDELAY, CREG_TX_THRESHOLD_CURRENT, CREG_TX_THRESHOLD_VOLTAGE )
class MeasureList1 : public RegList1<MReg1> {
  public:
    MeasureList1 (uint16_t addr) : RegList1<MReg1>(addr) {}
    void defaults () {
      clear();
      txMindelay(8);
      txThresholdCurrent(100);
      txThresholdVoltage(100);
      averaging(1);
    }
};

DEFREGISTER(SensorReg1, CREG_AES_ACTIVE, CREG_LEDONTIME, CREG_TRANSMITTRYMAX, CREG_COND_TX_THRESHOLD_HI, CREG_COND_TX_THRESHOLD_LO, CREG_COND_TX, CREG_COND_TX_DECISION_ABOVE, CREG_COND_TX_DECISION_BELOW)
class SensorList1 : public RegList1<SensorReg1> {
  public:
    SensorList1 (uint16_t addr) : RegList1<SensorReg1>(addr) {}
    void defaults () {
      clear();
      transmitTryMax(6);
      condTxDecisionAbove(200);
      condTxDecisionBelow(0);
      condTxFalling(false);
      condTxRising(false);
      condTxCyclicBelow(false);
      condTxCyclicAbove(false);
      condTxThresholdHi(0);
      condTxThresholdLo(0);
    }
};

class PowerEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint8_t typ, uint16_t current, uint16_t voltage ) {
      Message::init(0x0d, msgcnt, typ, (typ == AS_MESSAGE_POWER_EVENT_CYCLIC) ? BCAST : BIDI | BCAST, (current >> 8), (current) & 0xff);
      pload[0] = (voltage >> 8) & 0xff;
      pload[1] = (voltage) & 0xff;
    }
};

class PowerMeterChannel : public Channel<Hal, MeasureList1, EmptyList, List4, PEERS_PER_PMCHANNEL, PMSw1List0>, public Alarm {
    PowerEventMsg msg;
    uint16_t txThresholdCurrent;
    uint16_t txThresholdVoltage;
    uint8_t txMindelay;
  public:
    PowerMeterChannel () : Channel(), Alarm(0), txThresholdCurrent(0), txThresholdVoltage(0), txMindelay(8) {}
    virtual ~PowerMeterChannel () {}
    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      static uint8_t tickCount = 0;
      tick = seconds2ticks(delay());
      tickCount++;

      uint8_t msgType = 0;

      if (tickCount > (POWERMETER_CYCLIC_INTERVAL / delay()))  {
        msgType = AS_MESSAGE_POWER_EVENT_CYCLIC;
      }

      if ((txThresholdCurrent   > 0) && (abs((int)(actualValues.Current   - lastValues.Current)  ) >= (int)txThresholdCurrent))   msgType = AS_MESSAGE_POWER_EVENT;
      if ((txThresholdVoltage   > 0) && (abs((int)(actualValues.Voltage   - lastValues.Voltage)  ) >= (int)txThresholdVoltage))   msgType = AS_MESSAGE_POWER_EVENT;

      if ((msgType != AS_MESSAGE_POWER_EVENT) && (actualValues.Voltage > 0) && (lastValues.Voltage == 0)) msgType = AS_MESSAGE_POWER_EVENT;
      msg.init(device().nextcount(), msgType, actualValues.Current, actualValues.Voltage);
      switch (msgType) {
        case AS_MESSAGE_POWER_EVENT_CYCLIC:
          DPRINTLN(F("PowerMeterChannel - SENDING CYCLIC MESSAGE"));
          tickCount = 0;
          device().broadcastEvent(msg);
          break;
        case AS_MESSAGE_POWER_EVENT:
          if (device().getMasterID() > 0) {
            DPRINTLN(F("PowerMeterChannel - SENDING EVENT MESSAGE"));
            device().sendMasterEvent(msg);
          }
          break;
      }

      lastValues.Current = actualValues.Current;
      lastValues.Voltage = actualValues.Voltage;

      sysclock.add(*this);
    }

    uint32_t delay() {
      return max(1, txMindelay);
    }

    void configChanged() {
      DPRINTLN(F("PowerMeterChannel Config changed List1"));
      txThresholdCurrent   = this->getList1().txThresholdCurrent();      // 1 mA   = 1
      txThresholdVoltage   = this->getList1().txThresholdVoltage();      // 10.0V  = 100
      txMindelay           = this->getList1().txMindelay();
      averaging            = this->getList1().averaging();
      DPRINT(F("txMindelay           = ")); DDECLN(txMindelay);
      DPRINT(F("txThresholdCurrent   = ")); DDECLN(txThresholdCurrent);
      DPRINT(F("txThresholdVoltage   = ")); DDECLN(txThresholdVoltage);
    }

    void setup(Device<Hal, PMSw1List0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }
};

class SensorChannel : public Channel<Hal, SensorList1, EmptyList, List4, PEERS_PER_SENSORCHANNEL, PMSw1List0>, public Alarm {
  public:

    SensorChannel () : Channel(), Alarm(0) {}
    virtual ~SensorChannel () {}

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      static uint8_t evcnt = 0;
      bool sendMsg = false;
      tick = seconds2ticks(10);

      if (outputOn() == true) {
        SensorEventMsg& rmsg = (SensorEventMsg&)device().message();
        static uint8_t aboveMsgSent = false;
        static uint8_t belowMsgSent = false;
        switch (number()) {
          case 3:
            // Stromsensor
            if (this->getList1().condTxRising() == true)  {
              if (actualValues.Current > this->getList1().condTxThresholdHi()) {
                if ((aboveMsgSent == false && this->getList1().condTxCyclicAbove() == false) || this->getList1().condTxCyclicAbove() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionAbove(), false , false);
                  sendMsg = true;
                  aboveMsgSent = true;
                }
              } else {
                aboveMsgSent = false;
              }
            }
            if (this->getList1().condTxFalling() == true) {
              if (actualValues.Current < this->getList1().condTxThresholdLo()) {
                if ((belowMsgSent == false && this->getList1().condTxCyclicBelow() == false) || this->getList1().condTxCyclicBelow() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionBelow(), false , false);
                  sendMsg = true;
                  belowMsgSent = true;
                }
              } else {
                belowMsgSent = false;
              }
            }
            break;
          case 4:
            // Spannungssensor
            if (this->getList1().condTxRising() == true)  {
              if (actualValues.Voltage > this->getList1().condTxThresholdHi()) {
                if ((aboveMsgSent == false && this->getList1().condTxCyclicAbove() == false) || this->getList1().condTxCyclicAbove() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionAbove(), false , false);
                  sendMsg = true;
                  aboveMsgSent = true;
                }
              } else {
                aboveMsgSent = false;
              }
            }
            if (this->getList1().condTxFalling() == true) {
              if (actualValues.Voltage < this->getList1().condTxThresholdLo()) {
                if ((belowMsgSent == false && this->getList1().condTxCyclicBelow() == false) || this->getList1().condTxCyclicBelow() == true) {
                  rmsg.init(device().nextcount(), number(), evcnt++, this->getList1().condTxDecisionBelow(), false , false);
                  sendMsg = true;
                  belowMsgSent = true;
                }
              } else {
                belowMsgSent = false;
              }
            }
            break;
        }

        if (sendMsg) {
          device().sendPeerEvent(rmsg, *this);
          sendMsg = false;
        }
      }
      sysclock.add(*this);
    }

    void configChanged() {
      /*DPRINT(F("SensorChannel ")); DDEC(number()); DPRINTLN(F(" Config changed List1"));

      DPRINT(F("transmitTryMax      = ")); DDECLN(this->getList1().transmitTryMax());
      DPRINT(F("condTxDecisionAbove = ")); DDECLN(this->getList1().condTxDecisionAbove());
      DPRINT(F("condTxDecisionBelow = ")); DDECLN(this->getList1().condTxDecisionBelow());
      DPRINT(F("condTxFalling       = ")); DDECLN(this->getList1().condTxFalling());
      DPRINT(F("condTxRising        = ")); DDECLN(this->getList1().condTxRising());
      DPRINT(F("condTxCyclicAbove   = ")); DDECLN(this->getList1().condTxCyclicAbove());
      DPRINT(F("condTxCyclicBelow   = ")); DDECLN(this->getList1().condTxCyclicBelow());
      DPRINT(F("condTxThresholdHi   = ")); DDECLN(this->getList1().condTxThresholdHi());
      DPRINT(F("condTxThresholdLo   = ")); DDECLN(this->getList1().condTxThresholdLo());*/
    }

    void setup(Device<Hal, PMSw1List0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      return 0;
    }
};

class MixDevice : public ChannelDevice<Hal, VirtBaseChannel<Hal, PMSw1List0>, 4, PMSw1List0> {
    class MeasureAlarm : public Alarm {
        MixDevice& dev;
      protected:
        INA219_WE ina219;
      private:
        bool first;
      public:
        MeasureAlarm (MixDevice& d) : Alarm (0), dev(d), first(true) {}
        virtual ~MeasureAlarm () {}

        void initINA219() {
          Wire.begin();
          ina219.init();
          ina219.setBusRange(BRNG_16);
          ina219.setADCMode(SAMPLE_MODE_64);
          //ina219.setPGain(PG_320); //3.2A
          ina219.setPGain(PG_160); //1.6A
        }

        void trigger (AlarmClock& clock)  {
          if (first == true) {
            first = false;
            initINA219();
          }

          set(seconds2ticks(MEASURE_INTERVAL));
          static uint32_t Current    = 0;
          static uint32_t Voltage    = 0;
          static uint8_t  avgCounter = 0;

          if (resetAverageCounting == true) {
            //DPRINTLN(F("*** RESETTING AVERAGING ***"));
            resetAverageCounting = false;
            avgCounter = 0;
            Voltage = 0;
            Current = 0;
          }

          if (avgCounter < averaging) {
            uint32_t v = ina219.getBusVoltage_V();
            //DPRINT("v:");DDECLN(v);
            Voltage += v;
            uint16_t c = ina219.getCurrent_mA();
            //DPRINT("c:");DDECLN(c);
            Current += c;
            avgCounter++;
          } else {
            actualValues.Voltage = (Voltage / averaging);
            actualValues.Current = outputOn() ? (Current / averaging) : 0;
            resetAverageCounting = true;
          }

          clock.add(*this);
        }
    } sensorMeasure;
  public:
    VirtChannel<Hal, SwChannel,         PMSw1List0> c1;
    VirtChannel<Hal, PowerMeterChannel, PMSw1List0> c2;
    VirtChannel<Hal, SensorChannel,     PMSw1List0> c3, c4;
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, PMSw1List0>, 4, PMSw1List0> DeviceType;
    MixDevice (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr), sensorMeasure(*this) {
      DeviceType::registerChannel(c1, 1);
      DeviceType::registerChannel(c2, 2);
      DeviceType::registerChannel(c3, 3);
      DeviceType::registerChannel(c4, 4);

      sysclock.add(sensorMeasure);
    }
    virtual ~MixDevice () {}

    SwChannel& switchChannel ()  {
      return c1;
    }
    PowerMeterChannel& powermeterChannel ()  {
      return c2;
    }
    SensorChannel& sensorChannel3Current ()  {
      return c3;
    }
    SensorChannel& sensorChannel4Voltage ()  {
      return c4;
    }

    virtual void configChanged () {
      DPRINTLN(F("MixDevice Config changed List0"));
    }
};
MixDevice sdev(devinfo, 0x20);
ConfigToggleButton<MixDevice> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  bool first = sdev.init(hal);
  sdev.switchChannel().init(SWITCH_PIN, INVERT_SWITCH_PIN);
  buttonISR(cfgBtn, BUTTON_PIN);
  if( first == true ) {
    sdev.switchChannel().peer(cfgBtn.peer());
  }
  sdev.initDone();
}

void loop() {
  hal.runready();
  sdev.pollRadio();
}
