#include <kinetis.h>
#include <core_pins.h>
#include <pins_arduino.h>
#include <ADC.h>
#include <stdio.h>
#include <algorithm>

#undef min
#undef max
#undef DEG_TO_RAD

#include "motor.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pb_common.h"
#include "PacketSerial.h" // For COBS decoder
#include "cobs.hpp"
#include "crc8.h"

static constexpr uint32_t PIT_TEN = 1; // Timer enable
static constexpr uint32_t PIT_TIE = 2; // Interrupt enable


template<class T>
int Sign(const T &x)
{
  return x<0 ? T(-1) : T(1);
}


template<class T>
constexpr T Clamp(T x, T a, T b)
{
  return x>b ? b : (x<a ? a : x);
}


template<int PIN>
class Blinker
{
  elapsedMillis millisSinceToggle;
  bool state;
public:
  Blinker() : state(false) {}

  void toggle(int intervall = 0)
  {
    if (millisSinceToggle >= intervall)
    {
      state = !state;
      digitalWriteFast(PIN, state ? HIGH : LOW);
      millisSinceToggle = 0;
    }
  }
  
  void writeHigh(int intervall = 0)
  {
    if (millisSinceToggle >= intervall && state != true)
    {
      state = true;
      digitalWriteFast(PIN, HIGH);
      millisSinceToggle = 0;
    }
  }
  
  void writeLow(int intervall = 0)
  {
    if (millisSinceToggle >= intervall && state != false)
    {
      millisSinceToggle = 0;
      state = false;
      digitalWriteFast(PIN, LOW);
    }    
  }
};


class DisableIrq
{
public:
  DisableIrq()
  {
    __disable_irq();
  }
  
  ~DisableIrq()
  {
    __enable_irq();
  }
};


template<class T>
class Scope
{
  using value_type = T;
  static constexpr int SCOPE_SIZE = 1024;
  volatile int scope_index = 0;
  volatile T scope_readings[SCOPE_SIZE];
  volatile uint32_t scope_period;
  elapsedMicros scope_since_fill_begin;
  
public:
  bool ScopeIsFull(const DisableIrq &)
  {
    return scope_index >= SCOPE_SIZE;
  }

  size_t Count() const
  {
    return scope_index;
  }
  
  T operator[](size_t i) const
  {
    return scope_readings[i];
  }
  
  uint32_t Period() const
  {
    return scope_period;
  }
  
  void FillScope(T value)
  {
    if (scope_index >= SCOPE_SIZE)
      return;
    if (scope_index == 0)
      scope_since_fill_begin = 0;
    scope_readings[scope_index++] = value;
    if (scope_index == SCOPE_SIZE)
      scope_period = scope_since_fill_begin;
  }

  void Reset()
  {
    scope_index = 0;
  }
};


struct EWMA
{
  const float alpha;
  
  float value = 0.f;
  
  EWMA(const float _alpha) :
    alpha{_alpha} {}
  
  void Update(float input)
  {
    value = input*alpha + (1.f-alpha)*value;
  }
};


Blinker<LED_BUILTIN> blinker;

// Serial == USB.
// Serial1 == Pins 0 and 1
// Class hierarchy diverges at the Stream parent, but the parent does not have the common .begin method. :-(
#if 0
auto &com_serial = Serial;
#else
auto &com_serial = Serial1;
#endif

// Modulo operation that turns negative numbers into positive.
inline float mod_proper(float x, float y)
{
  float a = std::fmod(x, y);
  a = a >= 0.f ? a : a+y;
  return a;
}

static constexpr int16_t MAX_CURRENT_LIMIT_MILLIAMP = 2000;
static constexpr float MAX_FRACTIONAL_MOTOR_VOLTAGE = 0.2f;
static constexpr int32_t SPEED_CONTROL_LIMIT_HZ = 100;

static constexpr float DEG_TO_RAD = M_PI/180.f;



static constexpr int32_t PWM_MAX_VALUE = 255;


void ThingsToDoOnPwmCounterOverflow();
void ThingsToDoOnCurrentMeasurementAvailable();

float applied_fractional_motor_voltage = 0;
elapsedMillis g_since_last_control_msg = 0;
volatile bool g_stop = true;

volatile uint32_t counting_on_pwm_counter_overflow = 0;
volatile uint32_t counting_on_current_measurement = 0;
volatile uint32_t timing_on_current_measurement = 0;
volatile uint32_t timing_on_pwm_counter_overflow = 0;
volatile uint32_t timing_on_speed_control = 0;

namespace PWM_FTM0
{
static constexpr int32_t PWM_MAX_VALUE = ::PWM_MAX_VALUE;
static constexpr uint32_t PRESCALE = 2; // Divide clock by 1<<PRESCALE.
static constexpr uint32_t PWM_FREQ = F_BUS/(1<<PRESCALE)/PWM_MAX_VALUE/2; // Frequency of reaching the MOD value. Factor two because of center aligned pwm mode.
static constexpr float DT = 1./PWM_FREQ;  // In seconds.
}


namespace Messaging
{
  void SendStatus();
  void SendScope();
}

namespace MeasurementOutput
{
  void MaybeSendReport();
  void UpdateOnCurrentMeasurement();
}

namespace AnalogMeasurements
{
ADC adc;
static constexpr int ISENSE_PIN_1 = 36;
static constexpr int ISENSE_PIN_2 = 37;

static constexpr int CURRENT_SENSE_GAIN = 64;
static constexpr float CURRENT_VREF = 1.2f;
static constexpr int CURRENT_SENSE_RESOLUTION = 8; // TODO: Change this to 9?!
static constexpr float CURRENT_SENSE_RESISTOR = 0.01f;
static constexpr int32_t CURRENT_COUNTS_TO_MILLIAMP = CURRENT_VREF*1000.f/(CURRENT_SENSE_GAIN*CURRENT_SENSE_RESISTOR*(1<<CURRENT_SENSE_RESOLUTION)); // TODO: Fix point calculation with a few fractional digits.
static constexpr float CURRENT_SAMPLE_PERIOD = PWM_FTM0::DT;

volatile int16_t last_current_reading_counts = 0;
volatile int16_t last_current_reading_milliamps = 0;

namespace NonCurrentReadings {
void Setup();
}


void ConfigureAdc1ToBeTriggeredByPDB()
{
  // Defaults to trigger by PDB.
  ADC1_SC2  |= ADC_SC2_ADTRG; // Hardware triggered
  adc.adc1->recalibrate();
  adc.adc1->enableInterrupts();
  adc.adc1->startDifferentialFast(ISENSE_PIN_1, ISENSE_PIN_2);
}


void ConfigureAdc1ToRunContinuously()
{
  adc.adc1->recalibrate();
  adc.adc1->enableInterrupts();
  adc.adc1->startContinuousDifferential(ISENSE_PIN_1, ISENSE_PIN_2);
}


void Setup()
{
  pinMode(ISENSE_PIN_1, INPUT);
  pinMode(ISENSE_PIN_2, INPUT);
  // High speed setting takes up to 7 us for a continuous measurement (time between IRQ).
  // 10 us in continuous differential mode.
  adc.adc1->setResolution(CURRENT_SENSE_RESOLUTION);
  adc.adc1->setConversionSpeed(ADC_HIGH_SPEED);
  adc.adc1->setSamplingSpeed(ADC_VERY_HIGH_SPEED);
  adc.adc1->setReference(ADC_REF_1V2);
  adc.adc1->enablePGA(CURRENT_SENSE_GAIN);
  ConfigureAdc1ToBeTriggeredByPDB();
  //ConfigureAdc1ToRunContinuously();
  NonCurrentReadings::Setup();
}


extern "C" void adc1_isr(void)
{
  static_assert(CURRENT_COUNTS_TO_MILLIAMP < (1<<8), "So the result can be represented with 16 bits. Max 32 Amp are supported this way."); 
  last_current_reading_counts = (int16_t)ADC1_RA;
  last_current_reading_milliamps = CURRENT_COUNTS_TO_MILLIAMP*last_current_reading_counts;
  ThingsToDoOnCurrentMeasurementAvailable();
  NVIC_CLEAR_PENDING(IRQ_ADC1);
}


namespace NonCurrentReadings
{
static constexpr int IDX_VOLTAGE = 0;
static constexpr int IDX_TEMP    = 1;
static constexpr int NUM_QUANTITIES= 2;
static constexpr int PINS[NUM_QUANTITIES] = { 16, 15 };
static constexpr uint32_t ADC0_READ_PERIOD_MILLIS = 100;
static constexpr uint32_t RESOLUTION_BITS = 12;
static constexpr uint16_t MAX_READING_VAL = (1<<RESOLUTION_BITS)-1;

volatile uint16_t last_readings[NUM_QUANTITIES] = { 0, 0 };
volatile int reading_idx = IDX_VOLTAGE;

inline ADC_Module& Adc() { return *adc.adc0; }


void SetupInterruptTimer()
{
  // TODO: Dont' copy paste code. Refactor this stuff together with stuff from the velocity controller to commonly used code.
  // Configure timer to periodically trigger an ISR.
  // Ref: K10 Sub-Family Reference Manual. Pg. 909
  static constexpr uint32_t COUNTS = F_CPU*1.e-3*ADC0_READ_PERIOD_MILLIS;
  static constexpr uint32_t PIT_TIE = 2; // Interrupt enable
  static constexpr uint32_t PIT_TEN = 1; // Timer enable
  // Turn on PIT
  SIM_SCGC6 |= SIM_SCGC6_PIT; // Provide timer with clock signal.
  PIT_MCR = 0x00; 
  PIT_LDVAL1 = COUNTS; // Setup the period.
  PIT_TCTRL1 = PIT_TIE;  // Interrupt enable for channel x
  PIT_TCTRL1 |= PIT_TEN; // Timer Enable
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1); // Interrupt enable, again.
}


void Setup()
{
  for (int p : PINS)
    pinMode(p, INPUT);
  // Taking a measurement with this config will take 200 cycles or so.
  Adc().setConversionSpeed(ADC_MED_SPEED);
  Adc().setSamplingSpeed(ADC_MED_SPEED);
  Adc().setResolution(RESOLUTION_BITS);
  SetupInterruptTimer();
}


void MaybeStartNextMeasurement()
{
  if (!Adc().isConverting())
  {
    reading_idx = (reading_idx + 1) % NUM_QUANTITIES;
    Adc().startReadFast(PINS[reading_idx]);
  }
}

int MaybeUpdateReadings()
{
  if (Adc().isComplete())
  {
    last_readings[reading_idx] = (uint16_t)Adc().readSingle();
    return reading_idx;
  }
  return -1;
}


extern "C" void pit1_isr(void)
{
  MaybeUpdateReadings();
  MaybeStartNextMeasurement();
  PIT_TFLG1 = PIT_TFLG_TIF;
  NVIC_CLEAR_PENDING(IRQ_PIT_CH1);
}


int16_t ConvertToDegrees(uint16_t temp_counts)
{
  // Table generated by Calculations.ipynb Notebook. Values depend on voltage divide resistor and the NTC response curve.
  static constexpr size_t TEMP_LOOKUP_SIZE_BITS = 8;
  static constexpr size_t TEMP_LOOKUP_SIZE = 1<<TEMP_LOOKUP_SIZE_BITS;
  static constexpr size_t TABLE_SCALE = 127; // Table contains temperature in degrees multiplied by TABLE_SCALE. So we have 7 fractional bits.
  static const int16_t temp_lookup[TEMP_LOOKUP_SIZE] = {-6985, -5172, -3659, -2705, -1991, -1425, -948, -524, -156, 175, 481, 762, 1019, 1274, 1492, 1709, 1923, 2120, 2306, 2492, 2659, 2820, 2981, 3142, 3286, 3427, 3567, 3708, 3844, 3968, 4092, 4217, 4341, 4463, 4581, 4692, 4803, 4914, 5025, 5131, 5231, 5332, 5432, 5533, 5633, 5732, 5824, 5916, 6008, 6100, 6192, 6290, 6379, 6465, 6550, 6635, 6720, 6806, 6891, 6976, 7057, 7137, 7217, 7297, 7377, 7457, 7537, 7617, 7698, 7774, 7850, 7925, 8001, 8077, 8153, 8229, 8303, 8376, 8450, 8523, 8596, 8669, 8742, 8815, 8888, 8963, 9034, 9106, 9177, 9248, 9319, 9390, 9461, 9532, 9602, 9672, 9742, 9812, 9882, 9952, 10022, 10091, 10166, 10235, 10305, 10375, 10444, 10514, 10584, 10654, 10723, 10793, 10863, 10933, 11003, 11073, 11143, 11213, 11283, 11358, 11428, 11499, 11571, 11642, 11713, 11784, 11856, 11927, 11998, 12070, 12143, 12216, 12289, 12362, 12435, 12508, 12586, 12659, 12734, 12809, 12885, 12961, 13037, 13112, 13188, 13264, 13340, 13419, 13498, 13577, 13656, 13735, 13814, 13898, 13977, 14061, 14144, 14227, 14310, 14393, 14476, 14559, 14645, 14733, 14821, 14909, 14997, 15085, 15173, 15262, 15361, 15455, 15549, 15642, 15736, 15830, 15927, 16027, 16127, 16228, 16328, 16428, 16530, 16638, 16745, 16853, 16961, 17075, 17186, 17303, 17419, 17535, 17652, 17768, 17893, 18019, 18145, 18271, 18397, 18532, 18669, 18806, 18942, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050, 19050};
  
  static constexpr size_t ADDITIONAL_COUNT_BITS = RESOLUTION_BITS - TEMP_LOOKUP_SIZE_BITS; // Fractional bits with which we can interpolate neighboring table entries.
  static_assert(ADDITIONAL_COUNT_BITS >= 0, "Not considering anyting else");
  static constexpr uint32_t FPART_MAX = 1<<ADDITIONAL_COUNT_BITS;

  // Interpolation with fixed point arithmetic.
  const uint32_t c = temp_counts;
  uint32_t ipart = c >> ADDITIONAL_COUNT_BITS;
  uint32_t fpart = c - (ipart << ADDITIONAL_COUNT_BITS);
  uint32_t y0 = temp_lookup[ipart];
  uint32_t y = y0;
  if (ipart < TEMP_LOOKUP_SIZE-1)
  {
    uint32_t y1 = temp_lookup[ipart+1];
    y  = (y0*FPART_MAX + ((y1 - y0)*fpart))/FPART_MAX; // Linear interpolation.
  }
  return y / TABLE_SCALE; // Should yield degrees
}


float ConvertToMillivolt(uint16_t counts)
{
  static constexpr float RL = 1.;
  static constexpr float RH = 10.f;
  static constexpr float DIVIDER_FACTOR = RL/(RL+RH);
  static constexpr float VREF = 3.3f;
  static constexpr float FACTOR = VREF/DIVIDER_FACTOR/MAX_READING_VAL;
  //static constexpr size_t FRACTIONAL_BITS = 8;
  //static constexpr size_t FIXPOINT_SCALE = 1<<FRACTIONAL_BITS;
  //uint32_t factor = VREF/DIVIDER_FACTOR/MAX_READING_VAL*FIXPOINT_SCALE;
  //uint16_t val = uint32_t(factor*counts) >> FRACTIONAL_BITS;
  return counts*FACTOR;
}


void CopyReadingsToMessageSafe(MotorStatusMsg &msg)
{
  NVIC_DISABLE_IRQ(IRQ_PIT_CH1);
  uint16_t readings[NUM_QUANTITIES] {
    last_readings[0],
    last_readings[1]
  };
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
  msg.voltage = ConvertToMillivolt(readings[IDX_VOLTAGE]);
  msg.temperature = ConvertToDegrees(readings[IDX_TEMP]);
}


} // namespace NonCurrentReadings

using NonCurrentReadings::CopyReadingsToMessageSafe;

} //namespace AnalogMeasurements



namespace MotorEncoderReading
{
void DisableIrq();
void EnableIrq();

static constexpr int pins[] = { 5, 4, 3 };
static constexpr int8_t FAIL_DIR_VALUE = 3;

struct TransitionMatrixEntry
{
  TransitionMatrixEntry(int dir)
    : dir(dir)
  {
  }
  int8_t dir;
};
static const TransitionMatrixEntry transition_matrix[8][8] =  {
  { { FAIL_DIR_VALUE}, { 0}, { 0}, { 0}, { 0}, { 0}, { 0}, { FAIL_DIR_VALUE} },
  { { FAIL_DIR_VALUE}, { 0}, { 0}, { -1}, { 0}, { 1}, { 0}, { FAIL_DIR_VALUE} },
  { { FAIL_DIR_VALUE}, { 0}, { 0}, { 1}, { 0}, { 0}, { -1}, { FAIL_DIR_VALUE} },
  { { FAIL_DIR_VALUE}, { 1}, { -1}, { 0}, { 0}, { 0}, { 0}, { FAIL_DIR_VALUE} },
  { { FAIL_DIR_VALUE}, { 0}, { 0}, { 0}, { 0}, { -1}, { 1}, { FAIL_DIR_VALUE} },
  { { FAIL_DIR_VALUE}, { -1}, { 0}, { 0}, { 1}, { 0}, { 0}, { FAIL_DIR_VALUE} },
  { { FAIL_DIR_VALUE}, { 0}, { 1}, { 0}, { -1}, { 0}, { 0}, { FAIL_DIR_VALUE} },
  { { FAIL_DIR_VALUE}, { 0}, { 0}, { 0}, { 0}, { 0}, { 0}, { FAIL_DIR_VALUE} }
};


volatile uint8_t previous_state = 0;
volatile uint8_t state = 0;
volatile float velocity_estimate = 0;
elapsedMicros micros_since_last_state_change = 0;
volatile int8_t direction = 0;

void UpdateStateVars(uint8_t cur)
{
  uint8_t prev = state;
  state = cur;
  int32_t new_dir = -transition_matrix[prev][cur].dir;
  if (new_dir != FAIL_DIR_VALUE)
  {
    if (new_dir != direction)
      velocity_estimate = 0.f;
    else
    {
      velocity_estimate = (1.e6f/6.f)*new_dir/micros_since_last_state_change;
    }
    direction = new_dir;
  }
  else // Error state where rotor 'jumped' over successive position.
  {
    direction = 0;
    velocity_estimate = 0.f;
  }
  micros_since_last_state_change = 0;
}



void UpdateSpeedMeasurement()
{
  static constexpr uint32_t PERIOD_CUTOFF_MICROS = 1000*50; // Waited 0.1 sec for a encoder tick?
  DisableIrq(); 
  if (micros_since_last_state_change >= PERIOD_CUTOFF_MICROS)
  {
    velocity_estimate = 0.f;
    micros_since_last_state_change = PERIOD_CUTOFF_MICROS;
  }
  EnableIrq();
}


// Hall effect sensor A is represented by the least significant bit.
__attribute__((always_inline)) inline uint8_t ReadPins()
{
  uint8_t val1 = digitalReadFast(pins[0]);
  uint8_t val2 = digitalReadFast(pins[1]);
  uint8_t val3 = digitalReadFast(pins[2]);
  return (val3<<2)|(val2<<1)|val1;
}


void OnEncoderPinsChange()
{
  auto encoder_pins = ReadPins();
  UpdateStateVars(encoder_pins);
}


void ConfigurePin(int pin)
{
  // See attachInterrupt() in pins_teensy.c.
  uint32_t mask = 0x0B; // Trigger on pin change
  mask = (mask << 16) | 0x01000000;
  volatile uint32_t *config = portConfigRegister(pin);
  uint32_t cfg = *config;
  cfg &= ~0x000F0000;   // disable any previous interrupt
  cfg |= mask;
  *config = cfg;
}


void Setup()
{
    for (auto pin : pins)
    {
      pinMode(pin, INPUT_PULLUP);
    }
    
    ConfigurePin(pins[0]);
    ConfigurePin(pins[1]);
    ConfigurePin(pins[2]);

    attachInterruptVector(IRQ_PORTA, porta_isr);
    attachInterruptVector(IRQ_PORTB, portb_isr);
    attachInterruptVector(IRQ_PORTC, portc_isr);
    attachInterruptVector(IRQ_PORTD, portd_isr);
    attachInterruptVector(IRQ_PORTE, porte_isr);
    // Fetch current state
    UpdateStateVars(ReadPins());
}


extern "C" void porta_isr(void)
{
  OnEncoderPinsChange();
  const uint32_t isfr = PORTA_ISFR;
  PORTA_ISFR = isfr;
}
extern "C" void portb_isr(void)
{
  OnEncoderPinsChange();
  const uint32_t isfr = PORTB_ISFR;
  PORTB_ISFR = isfr;
}
extern "C" void portc_isr(void)
{
  OnEncoderPinsChange();
  const uint32_t isfr = PORTC_ISFR;
  PORTC_ISFR = isfr;
}
extern "C" void portd_isr(void)
{
  OnEncoderPinsChange();
  const uint32_t isfr = PORTD_ISFR;
  PORTD_ISFR = isfr;
}
extern "C" void porte_isr(void)
{
  OnEncoderPinsChange();
  const uint32_t isfr = PORTE_ISFR;
  PORTE_ISFR = isfr;
}


void EnableIrq()
{
  NVIC_ENABLE_IRQ(IRQ_PORTA);
  NVIC_ENABLE_IRQ(IRQ_PORTB);
  NVIC_ENABLE_IRQ(IRQ_PORTC);
  NVIC_ENABLE_IRQ(IRQ_PORTD);
  NVIC_ENABLE_IRQ(IRQ_PORTE);
}

void DisableIrq()
{
  NVIC_DISABLE_IRQ(IRQ_PORTA);
  NVIC_DISABLE_IRQ(IRQ_PORTB);
  NVIC_DISABLE_IRQ(IRQ_PORTC);
  NVIC_DISABLE_IRQ(IRQ_PORTD);
  NVIC_DISABLE_IRQ(IRQ_PORTE);
}


}


namespace PWM_FTM0
{ 
#undef FTM_CONF_BDMMODE
#define FTM_CONF_BDMMODE(n)    (((n) & 3) << 6)    // Behavior when in debug mode
#undef FTM_CONF_NUMTOF
#define FTM_CONF_NUMTOF(n)     (((n) & 31) << 0)   // ratio of counter overflows to TOF bit set

/* For mapping from pin number to port and number see
 * CORE_PINN_PORTSET
 * and
 * CORE_PINN_BIT
 */

enum PhaseDisableMask : uint8_t
{
  // See manual on FTMx_OUTMASK register. Each of these flags disables a complementary pair of two channels. Hence the 3's.
  PHASE_A = 3 << 0,
  PHASE_B = 3 << 2,
  PHASE_C = 3 << 4,
  ALL_ENABLED = 0
};

/* Setup PWM duty cyle for FTM0 Channels 0, 1, 2, 3, 4, 5.
 * 0 and 1 are complementary
 * 2 and 3 are complementary
 * 4 and 5 are complementary
 * Channel,  Pin,   Inverted
 * 0         22     no
 * 1         23     yes
 * 2         9      no
 * 3         10     yes
 * Pin refers to the pin on the teensy 3.2 board
*/
void Setup()
{
  const int32_t mod = PWM_MAX_VALUE;  // The number of counts equivalent to the cycle period.
  
  // See System Integration Module manual: System Clock Gating Control Register.
  // Enable FTM0 and FTM0 module clock.
  SIM_SCGC6|=SIM_SCGC6_FTM0;
  // Ports deal with mapping from signal source to GPIO pins. See Chapter 10 on signal multiplexing.
  SIM_SCGC5=SIM_SCGC5|SIM_SCGC5_PORTA|SIM_SCGC5_PORTB|SIM_SCGC5_PORTC|SIM_SCGC5_PORTD|SIM_SCGC5_PORTE; //enable port A/B/C/D/E clock
  
  FTM0_SC=0; // We have to disable stuff here to get our changes further down here accepted.
  FTM0_CONF=FTM_CONF_BDMMODE(3); //Set up BDM in 11. Refers to debug halt mode. FTM timer set to fully functional.  
  FTM0_FMS=0x00; //clear the WPEN so that WPDIS may be set in FTM0_MODE reg. WP means write protect.
  FTM0_MODE|=FTM_MODE_WPDIS|FTM_MODE_FTMEN; //enable write the FTM CnV register // FTM_MODE_INIT  
  FTM0_SC=FTM_SC_CLKS(1)|FTM_SC_PS(PRESCALE); // Select system clock. Set prescaler.

  // Counting mode and pulse polarity configuration.
  FTM0_C0SC= FTM_CSC_ELSB;
  FTM0_C1SC= FTM_CSC_ELSB;
  FTM0_C2SC= FTM_CSC_ELSB;
  FTM0_C3SC= FTM_CSC_ELSB;
  FTM0_C4SC= FTM_CSC_ELSB;
  FTM0_C5SC= FTM_CSC_ELSB;
  
  // Running at 48 MHz, DTVAL being the number of (prescaled) clock ticks, a DTVAL of 48 amounts to a dead time of ca. 1 us.
  FTM0_DEADTIME=FTM_DEADTIME_DTPS(1)|FTM_DEADTIME_DTVAL(5);
  
  FTM0_COMBINE=FTM_COMBINE_COMP0 | FTM_COMBINE_COMP1 | FTM_COMBINE_COMP2; //complementary mode
  FTM0_COMBINE|=FTM_COMBINE_DTEN0 | FTM_COMBINE_DTEN1 | FTM_COMBINE_DTEN2; // dead timer insertion enabled in complementary mode
  FTM0_COMBINE|=FTM_COMBINE_SYNCEN0 | FTM_COMBINE_SYNCEN1 | FTM_COMBINE_SYNCEN2; // Synchronize Cuint8_t pwm_out*(n)V and C(n+1)V registers;
  FTM0_COMBINE|=FTM_COMBINE_COMBINE0 | FTM_COMBINE_COMBINE1 | FTM_COMBINE_COMBINE2; // Combine mode. See Freescale AN3729.
  FTM0_SYNC=FTM_SYNC_CNTMAX; // Activate new CxV values when the timer reaches MOD.

  FTM0_MOD=mod-1; // The modulo value. That is the value where the cycle restarts.
  FTM0_CNTIN=-mod;
  // Connect phases to low side.
  // Channels 1, 3, and 5 are set to follow 0, 2, and 4, respectively.
  // We set the values nontheless since they determine the location of the falling edge of the pwm pulse.
  FTM0_C0V = 0;
  FTM0_C1V = 0;
  FTM0_C2V = 0;
  FTM0_C3V = 0;
  FTM0_C4V = 0;
  FTM0_C5V = 0;
  FTM0_SYNC |= FTM_SYNC_SWSYNC;

  // Connect the pins to the FTM timer.
  CORE_PIN22_CONFIG = PORT_PCR_MUX(4); // | PORT_PCR_PE; // ALT4 : FTM0_CH0
  CORE_PIN23_CONFIG = PORT_PCR_MUX(4); // | PORT_PCR_PE; // ALT4 : FTM0_CH1
  CORE_PIN9_CONFIG = PORT_PCR_MUX(4); // | PORT_PCR_PE; 
  CORE_PIN10_CONFIG = PORT_PCR_MUX(4); // | PORT_PCR_PE;
  CORE_PIN6_CONFIG = PORT_PCR_MUX(4);
  CORE_PIN20_CONFIG = PORT_PCR_MUX(4);
  
  /* ---  enable PDB to trigger measurement and isr invocation ---*/
  static constexpr uint32_t PDB_CHC1_TOS = 0x0100;
  static constexpr uint32_t PDB_CHC1_EN = 0x01;
  static constexpr uint32_t PDB_TRGSEL_FTM0 = 8; // pg. 109.
  static constexpr uint32_t PDB_CONFIG_ = PDB_SC_TRGSEL(PDB_TRGSEL_FTM0) | PDB_SC_PDBEN | PDB_SC_PRESCALER(0) | PDB_SC_MULT(0);
  static constexpr uint16_t PDB_PERIOD = std::numeric_limits<uint16_t>::max();  // Run for the entire pwm cycle. FTM will reset counter when new cycle starts.
  static constexpr uint16_t PDB_MEASURE_DELAY = (PWM_MAX_VALUE-0)*(1<<PRESCALE);    // At the center. When FTM counted from -MOD up to 0.
  static constexpr float    PDB_ISR_DELAY_SEC = 5.e-6f;
  static constexpr uint16_t PDB_ISR_DELAY     = PDB_MEASURE_DELAY+uint16_t(F_BUS*PDB_ISR_DELAY_SEC); // Between current measurement and pwm parameter committment.
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  FTM0_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN; // First enable trigger. So we trigger the PDB when the counter reacher CNTIN.
  PDB0_MOD = PDB_PERIOD;
  PDB0_CH1DLY0 = PDB_MEASURE_DELAY;
  PDB0_IDLY = PDB_ISR_DELAY; // Interrupt delay
  PDB0_CH1C1 = PDB_CHC1_TOS | PDB_CHC1_EN; // Enable pre-trigger
  PDB0_SC = PDB_CONFIG_ | PDB_SC_LDOK | PDB_SC_PDBIE;
  
  //FTM0_MODE = FTM_MODE_FTMEN;
  NVIC_ENABLE_IRQ(IRQ_PDB);
}


void SetDutyCycle(uint8_t cycle0, uint8_t cycle1, uint8_t cycle2, PhaseDisableMask disabled = PhaseDisableMask::ALL_ENABLED)
{
  FTM0_C0V = -cycle0;
  FTM0_C2V = -cycle1;
  FTM0_C4V = -cycle2;
  FTM0_C1V = cycle0;
  FTM0_C3V = cycle1;
  FTM0_C5V = cycle2;
  FTM0_OUTMASK = disabled;
  FTM0_SYNC |= FTM_SYNC_SWSYNC;// Activate!
}



extern "C" void pdb_isr(void)
{
  ::ThingsToDoOnPwmCounterOverflow();
  PDB0_SC &= ~PDB_SC_PDBIF;
  NVIC_CLEAR_PENDING(IRQ_PDB);
}


}


void UpdateVoltagePwm()
{
  MotorEncoderReading::DisableIrq();
  uint8_t encoder_pins = MotorEncoderReading::state;
  MotorEncoderReading::EnableIrq();
  int8_t desired_dir = applied_fractional_motor_voltage < 0.f ? -1 : 1;
  float magnitude = applied_fractional_motor_voltage < 0.f ? -applied_fractional_motor_voltage : applied_fractional_motor_voltage;
  magnitude = magnitude > MAX_FRACTIONAL_MOTOR_VOLTAGE ? MAX_FRACTIONAL_MOTOR_VOLTAGE : magnitude;
  uint8_t pwm_val = Clamp<int32_t>(PWM_MAX_VALUE*magnitude, 0, PWM_MAX_VALUE);
  uint8_t pwm_phase[3] = { 0, 0, 0 };
  auto mask = PWM_FTM0::PhaseDisableMask::ALL_ENABLED;
  if (desired_dir > 0)
  {
    switch(encoder_pins)
    {
      case 1: // 001 (Hall Sensor readings,CBA)
        mask = PWM_FTM0::PhaseDisableMask::PHASE_C;
        pwm_phase[1] = 0;
        pwm_phase[0] = pwm_val;
        break;
      case 5: // 101
        mask = PWM_FTM0::PhaseDisableMask::PHASE_A;
        pwm_phase[2] = pwm_val;
        pwm_phase[1] = 0;
        break;
      case 4: // 100
        mask = PWM_FTM0::PhaseDisableMask::PHASE_B;
        pwm_phase[2] = pwm_val;
        pwm_phase[0] = 0;
        break;
      case 6: // 110
        mask = PWM_FTM0::PhaseDisableMask::PHASE_C;
        pwm_phase[1] = pwm_val;
        pwm_phase[0] = 0;
        break;
      case 2: // 010
        mask = PWM_FTM0::PhaseDisableMask::PHASE_A;
        pwm_phase[1] = pwm_val;
        pwm_phase[2] = 0;
        break;
      case 3: // 011
        mask = PWM_FTM0::PhaseDisableMask::PHASE_B;
        pwm_phase[0] = pwm_val;
        pwm_phase[2] = 0;
        break;
      default:
        break;
    }
  }
  else
  {
    // We merely have to swap the signals to the active phases!
    // I.e. when phase A was +V and C was GND, now it is GND and +V.
    switch(encoder_pins)
    {
      case 1: // 001 (Hall Sensor readings,CBA)
        mask = PWM_FTM0::PhaseDisableMask::PHASE_C;
        pwm_phase[1] = pwm_val;
        pwm_phase[0] = 0;
        break;
      case 5: // 101
        mask = PWM_FTM0::PhaseDisableMask::PHASE_A;
        pwm_phase[2] = 0;
        pwm_phase[1] = pwm_val;
        break;
      case 4: // 100
        mask = PWM_FTM0::PhaseDisableMask::PHASE_B;
        pwm_phase[2] = 0;
        pwm_phase[0] = pwm_val;
        break;
      case 6: // 110
        mask = PWM_FTM0::PhaseDisableMask::PHASE_C;
        pwm_phase[1] = 0;
        pwm_phase[0] = pwm_val;
        break;
      case 2: // 010
        mask = PWM_FTM0::PhaseDisableMask::PHASE_A;
        pwm_phase[1] = 0;
        pwm_phase[2] = pwm_val;
        break;
      case 3: // 011
        mask = PWM_FTM0::PhaseDisableMask::PHASE_B;
        pwm_phase[0] = 0;
        pwm_phase[2] = pwm_val;
        break;
      default:
        break;
    }
  }
//   {
//     static uint8_t counter = 0;
//     if (++counter == 0)
//     {
//       char buffer[128];
//       snprintf(buffer, 128, "@DBG;%i;%i;%i;%i;%i$", (int)encoder_pins, (int)pwm_phase[0], (int)pwm_phase[1], (int)pwm_phase[2], (int)mask);
//       Serial.print(buffer);
//     }
//   }
  PWM_FTM0::SetDutyCycle(pwm_phase[0], pwm_phase[1], pwm_phase[2], mask);
}



namespace CurrentLimiter
{
  float desired_fractional_motor_voltage = 0;

  namespace Internal
  {
    static constexpr int HYSTERESIS_TIME_IN_PWM_TICKS = 2; // Ca 1 ms?
    int active_for_pwm_ticks = 0;
  }

  void Update()
  {
      using namespace Internal;
      int16_t reading = AnalogMeasurements::last_current_reading_milliamps;
      if (std::abs(reading) > MAX_CURRENT_LIMIT_MILLIAMP)
      {
        active_for_pwm_ticks = HYSTERESIS_TIME_IN_PWM_TICKS;
        applied_fractional_motor_voltage = 0;
      }
      else if (active_for_pwm_ticks>0)
      {
        --active_for_pwm_ticks;
        applied_fractional_motor_voltage = 0;
      }
      else
      {
        applied_fractional_motor_voltage = desired_fractional_motor_voltage;
      }
  }
} // namespace CurrentLimiter


struct SeriesPI
{
  const float ka, kb, integral_clamp;
  float integral = 0;

  SeriesPI(const float _ka, const float _kb, const float _integral_clamp):
    ka{_ka}, kb{_kb}, integral_clamp{_integral_clamp}
    {}
  
  float Update(float desired, float actual)
  {
    float control_value = ka*(desired - actual);
    integral += kb*control_value;
    integral = Clamp(integral, -integral_clamp, integral_clamp);
    control_value += integral;
    return control_value;
  }
};



namespace SpeedController
{
static constexpr float DT = 1.e-3f; // Secs.
static constexpr float FILTER_DT = 0.01f;
static constexpr float PI_KC = 0.004f;
static constexpr float PI_KD = DT*5.;
static constexpr float PI_CLAMP = 0.15f; // Fractional PWM.
static constexpr float ZERO_SPEED_THRESHOLD = 0.5f;
static constexpr float EWMA_ALPHA = PWM_FTM0::DT/FILTER_DT;

volatile float desired_speed = 0;

SeriesPI pi{
  PI_KC,
  PI_KD,
  PI_CLAMP
};

EWMA speed_filter{EWMA_ALPHA};


void Enable(bool active)
{
  PIT_TCTRL0 = active ? (PIT_TCTRL0 | PIT_TEN) : (PIT_TCTRL0 & ~PIT_TEN); 
}

// bool IsEnabled()
// {
//   return PIT_TCTRL0 & PIT_TEN;
// }

void Setup()
{
  // Configure timer to periodically trigger an ISR.
  // Ref: K10 Sub-Family Reference Manual. Pg. 909
  static constexpr uint32_t COUNTS = F_CPU*DT;  
  // Turn on PIT
  SIM_SCGC6 |= SIM_SCGC6_PIT; // Provide timer with clock signal.
  PIT_MCR = 0x00; 
  PIT_LDVAL0 = COUNTS; // Setup the period.
  // Enable interrupt enable for channel x. Timer initially disabled.
  PIT_TCTRL0 = PIT_TIE;
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0); // Interrupt enable, again.
}


void OpenLoopControl()
{
  CurrentLimiter::desired_fractional_motor_voltage = 0.001f*desired_speed;
}


void ClosedLoopControl()
{
  MotorEncoderReading::DisableIrq();
  float speed = MotorEncoderReading::velocity_estimate;
  MotorEncoderReading::EnableIrq();
  speed_filter.Update(speed);
  speed = speed_filter.value;
  float out = pi.Update(desired_speed, speed); 
//   if (std::abs(desired_speed) <= 0 && std::abs(speed) < ZERO_SPEED_THRESHOLD)
//   {
//     CurrentLimiter::desired_fractional_motor_voltage = 0.f;
//   }
//   else
  {
    CurrentLimiter::desired_fractional_motor_voltage = out;
  }
}



void Update()
{
  auto _t = micros();
  //OpenLoopControl();
  ClosedLoopControl();
  timing_on_speed_control = micros() - _t;
}


extern "C" void pit0_isr(void)
{
  Update();
  PIT_TFLG0 = PIT_TFLG_TIF;
  NVIC_CLEAR_PENDING(IRQ_PIT_CH0);
}

} // namespace SpeedController



void ThingsToDoOnCurrentMeasurementAvailable()
{
  counting_on_current_measurement++;
}


void ThingsToDoOnPwmCounterOverflow()
{
    elapsedMicros _t;
    CurrentLimiter::Update();
    UpdateVoltagePwm();
    timing_on_pwm_counter_overflow = _t;
    counting_on_pwm_counter_overflow++;
    _t = 0;
    MeasurementOutput::UpdateOnCurrentMeasurement();
    MotorEncoderReading::UpdateSpeedMeasurement();
    timing_on_current_measurement = _t;
}





class MotorControlMsgReader
{ 
  MotorControlMsg the_msg;
  bool was_decoded = false;
  
  static constexpr uint8_t PACKET_MARKER=0;
  static constexpr size_t RECV_BUFFER_SIZE=256;
  uint8_t recv_buffer[RECV_BUFFER_SIZE];
  size_t recv_buffer_index = 0;
  
  bool check_checksum_and_move_buffer_ptr(const uint8_t* &buffer, size_t &size)
  {
    using ChecksumType = decltype(crc8(nullptr, 0, 0));
    if (size <= sizeof(ChecksumType))
      return false;
    static_assert(std::is_same<ChecksumType, uint8_t>::value, "Must match because of following line");
    uint8_t cs = buffer[size-sizeof(ChecksumType)];
    size-=sizeof(ChecksumType);
    uint8_t computed_cs = crc8(buffer, size, 0);
    return cs == computed_cs;
  }
  
  void on_packet_received(const uint8_t* buffer, size_t size)
  {
    if (check_checksum_and_move_buffer_ptr(buffer, size))
    {
      pb_istream_t pb_stream = pb_istream_from_buffer(buffer, size);
      was_decoded = pb_decode(&pb_stream, MotorControlMsg_fields, &the_msg);
    }
  }
  
public:
  const MotorControlMsg* putc_and_maybe_decode_msg(uint8_t data);
};


const MotorControlMsg* MotorControlMsgReader::putc_and_maybe_decode_msg(uint8_t data)
{
  was_decoded = false;
  if (data == PACKET_MARKER)
  {
    uint8_t _decodeBuffer[recv_buffer_index];
    size_t numDecoded = COBS::decode(recv_buffer,
                                            recv_buffer_index,
                                            _decodeBuffer);
    on_packet_received(_decodeBuffer, numDecoded);
    recv_buffer_index = 0;
  }
  else
  {
    if ((recv_buffer_index + 1) < RECV_BUFFER_SIZE)
    {
      recv_buffer[recv_buffer_index++] = data;
    }
    else
    {
      // Error, buffer overflow if we write.
      recv_buffer_index = 0;
    }
  }
  return was_decoded ? &the_msg : nullptr;
}


void EnsureEnableState(bool enable);
void Enable();
void Disable();

void ProcessControlInput(const MotorControlMsg &msg)
{
  //EnsureEnableState(msg.enable);
  //if (msg.enable)
  EnsureEnableState(true);
  {
    auto speed = Clamp(msg.speed, -SPEED_CONTROL_LIMIT_HZ, SPEED_CONTROL_LIMIT_HZ);
    SpeedController::desired_speed = speed;
  }
  g_since_last_control_msg = 0;
}


void EnsureEnableState(bool want_enabled)
{
  if (want_enabled)
  {
    if (g_stop)
    {
      Enable();
    }
  }
  else
  {
    if (!g_stop)
    {
      Disable();
    }
  }
}


void Enable()
{
  SpeedController::Enable(true);
  g_stop = false;
}


void Disable()
{
  SpeedController::Enable(false);
  SpeedController::desired_speed = 0;
  applied_fractional_motor_voltage = 0;
  CurrentLimiter::desired_fractional_motor_voltage = 0;
  g_stop = true;
}


MotorControlMsgReader control_msg_reader;



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  SpeedController::Setup();
  AnalogMeasurements::Setup();
  MotorEncoderReading::Setup();
  PWM_FTM0::Setup();
  
  com_serial.begin(115200);
  if (com_serial)
  {
  // ignore stuff still in the buffer
    while (com_serial.available())  
      com_serial.read();
  } 
  
  // See kinetis.h
  // Cortex-M4: 0,16,32,48,64,80,96,112,128,144,160,176,192,208,224,240
  NVIC_SET_PRIORITY(IRQ_PIT_CH1, 64); // Other measurements.
  NVIC_SET_PRIORITY(IRQ_PIT_CH0, 48); // Speed controller
  NVIC_SET_PRIORITY(IRQ_PDB, 32); // Pwm ticker
  NVIC_SET_PRIORITY(IRQ_ADC1, 16); // Current measurment
  //NVIC_SET_PRIORITY(IRQ_FTM0, 16); // Pwm ticker
  // IRQ from pins get highest priority. Because commutation is measured there.
  NVIC_SET_PRIORITY(IRQ_PORTA, 0); 
  NVIC_SET_PRIORITY(IRQ_PORTB, 0);
  NVIC_SET_PRIORITY(IRQ_PORTC, 0);
  NVIC_SET_PRIORITY(IRQ_PORTD, 0);
  NVIC_SET_PRIORITY(IRQ_PORTE, 0);
  
  Disable();
}




void loop() 
{
  blinker.toggle(g_stop ? 50 : 300);

  if (com_serial.available())
  {
    static constexpr int MAX_READ = 128;
    for (int n_read = 0; n_read < MAX_READ && com_serial.available(); ++n_read)
    {
      uint8_t data = com_serial.read();
      const auto *msg = control_msg_reader.putc_and_maybe_decode_msg(data);
      if (msg)
        ProcessControlInput(*msg);
    }
  }
  
  if (g_since_last_control_msg > 250)
    EnsureEnableState(false); // Enabling is done in ProcessControlInput

  MeasurementOutput::MaybeSendReport();
}



namespace MeasurementOutput
{
Scope<int16_t> scope_the_current;
static constexpr float CURRENT_AVG_TIME_CONST_F_SEC = 0.1f;
float avg_current_milliamp = 0.f;


void UpdateAverageCurrent()
{
  // This shit takes 5 us. But it works. Careful when doing measurements: Higher priority isr could run, artificially increasing the time for functions like this to complete.
  static constexpr float ALPHA = AnalogMeasurements::CURRENT_SAMPLE_PERIOD/CURRENT_AVG_TIME_CONST_F_SEC;
  const float val = AnalogMeasurements::last_current_reading_milliamps;
  avg_current_milliamp = ALPHA*val + (1.f-ALPHA)*avg_current_milliamp;
}

void UpdateOnCurrentMeasurement()
{
  scope_the_current.FillScope(AnalogMeasurements::last_current_reading_counts);
  UpdateAverageCurrent();
}


elapsedMillis sinceLastStatus = 0;
elapsedMillis sinceLastScope = 0;

void MaybeSendReport()
{
  if (sinceLastStatus > 500)
  {
    Messaging::SendStatus();
    
//     __disable_irq();
//     uint32_t fu1 = counting_on_pwm_counter_overflow;
//     uint32_t fu2 = counting_on_current_measurement;
//     __enable_irq();
//     char buffer[256];
//     sprintf(buffer, "@DBG;%lu;%lu;%lu;%lu$", timing_on_pwm_counter_overflow, timing_on_current_measurement, fu1/sinceLastStatus, fu2/sinceLastStatus);
//     Serial.write(buffer);
    //counting_on_pwm_counter_overflow = 0;
    //counting_on_current_measurement = 0;
    
    sinceLastStatus = 0;    
    return;
  }
  
  if (sinceLastScope > 500 && scope_the_current.ScopeIsFull(DisableIrq()))
  {
    Messaging::SendScope();
    DisableIrq guard;
    scope_the_current.Reset();
    sinceLastScope = 0;
    return;
  }
}

}


namespace Messaging
{

class SerialOutCobsWithChecksum
{
  struct Writer
  { 
    void operator()(const uint8_t *buffer, size_t s)
    {
      com_serial.write(buffer, s);
    }
  };
  COBSIncremental<Writer> cobs;
  uint8_t checksum = 0;
  
public:
  SerialOutCobsWithChecksum() :
    cobs(Writer{})
    {}
  
  ~SerialOutCobsWithChecksum()
  {
    cobs.encode_incremental(&checksum, 1);
    // After that, dtor of cobs class will write final block.
  }
  
  void encode_incremental(const uint8_t *buf, size_t count)
  {
    checksum = crc8(buf, count, checksum);
    cobs.encode_incremental(buf, count);
  }
};


bool SerialOutCobsWriteCallback(pb_ostream_t *this_, const uint8_t *buf, size_t count)
{
  auto* cobs = (SerialOutCobsWithChecksum*)this_->state;
  cobs->encode_incremental(buf, count);
  return true;
}


void Send(const void *msg, const pb_field_t *fields)
{
  com_serial.write((uint8_t)0);
  {
    SerialOutCobsWithChecksum cobs{};
    pb_ostream_t os {
      SerialOutCobsWriteCallback,
      &cobs,
      SIZE_MAX,
      0
    };
    pb_encode(&os, fields, msg);
    // Dtor call on SerialOutCobsWithChecksum writes final block!
    // Thus 0-delimiter must be written after cobs goes out of scope.
  }
  com_serial.write((uint8_t)0);
}


void Fill(MotorStatusMsg &msg)
{
    {
      DisableIrq guard;
      msg.phase_current = ::MeasurementOutput::avg_current_milliamp;
      msg.speed = MotorEncoderReading::velocity_estimate; //SpeedController::speed_filter.value;
      msg.pwm_magnitude = applied_fractional_motor_voltage;
    }
    AnalogMeasurements::CopyReadingsToMessageSafe(msg);
}


// void Send(const MotorStatusMsg &msg)
// {
//   Send(&msg, MotorStatusMsg_fields);
// }


// Now the oscilloscope message
// ----------------------------

using ScopeType = Scope<int16_t>;

bool WriteScopeEntryCallback(pb_ostream_t *os, const pb_field_t *field, void *const *arg)
{
  auto* scope = (const ScopeType*)*arg;
 
  // We write a packed array. The code is adapted from 'encode_array' in pb_encode.c.
  
  // First write tag with wire type 'string'.
  if (!pb_encode_tag(os, PB_WT_STRING, field->tag))
    return false;
  
  // Then determine the size of the payload, i.e. the encoded array.
  size_t encoded_array_size = 0;
  {
    pb_ostream_t sizestream = PB_OSTREAM_SIZING;
    for (size_t i = 0; i < scope->Count(); i++)
    {
        pb_encode_svarint(&sizestream, (*scope)[i]);
    }
    encoded_array_size = sizestream.bytes_written;
  }
  
  // Write the size.
  if (!pb_encode_varint(os, encoded_array_size))
      return false;
  
  // Write the data.
  for (size_t i=0; i<scope->Count(); ++i)
  {
    if (!pb_encode_svarint(os, (*scope)[i]))
      return false;
  }
  return true;
}


void Fill(MotorScopeMsg &msg, const ScopeType &scope)
{
  msg.values.funcs.encode = WriteScopeEntryCallback;
  msg.values.arg = (void*)&scope;
  msg.period = scope.Period();
}


void SendStatus()
{
  MotorReportContainerMsg msg;
  msg.which_payload = MotorReportContainerMsg_status_tag;
  Fill(msg.payload.status);
  Send(&msg, MotorReportContainerMsg_fields);
}


void SendScope()
{
  MotorReportContainerMsg msg;
  msg.which_payload = MotorReportContainerMsg_scope_phase_current_tag;
  Fill(msg.payload.scope_phase_current, MeasurementOutput::scope_the_current);
  Send(&msg, MotorReportContainerMsg_fields);
}

}

