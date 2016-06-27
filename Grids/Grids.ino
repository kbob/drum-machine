// Mutable Instruments' Grids module ported to Teensy
// https://github.com/pichenettes/eurorack

// Hardware:
//
//   Teensy LC
//  
//   Digital Inputs
//      pin 0 - MIDI in
//      pin 2 - tap/reset button
//      pin 3 - clock input
//      pin 4 - reset input
//
//   Analog Inputs
//      pin 14 (A0) - Bass Fill
//      pin 15 (A1) - Chaos
//      pin 16 (A2) - Snare Fill
//      pin 17 (A3) - Y
//      pin 21 (A7) - High Hat Fill
//      pin 23 (A9) - X
//      pin 26 (A12) - Clock
//
//   LED Outputs
//      pin 5 - Bass LED
//      pin 6 - Snare LED
//      pin 7 - High Hat LED
//      pin 13 - Tempo LED
//
//   Signal Outputs
//      pin 20 - Bass Trigger
//      pin 19 - Snare Trigger
//      pin 18 - High Hat Trigger
//      pin  8 - Bass Accent
//      pin  9 - Snare Accent
//      pin 10 - High Hat Accent
//      pin 11 - clock echo
//      pin 12 - random bit

#include <Bounce.h>
#include <IntervalTimer.h>

#include "avrlib_base.h"
#include "avrlib_op.h"

#include "clock.h"
#include "hardware_config.h"
#include "pattern_generator.h"

#define MIDI_IN_PIN              0
#define TAP_BUTTON_PIN           2
#define CLOCK_INPUT_PIN          3
#define RESET_INPUT_PIN          4
#define BASS_LED_PIN             5
#define SNARE_LED_PIN            6
#define HIGH_HAT_LED_PIN         7
#define BASS_ACCENT_PIN          8
#define SNARE_ACCENT_PIN         9
#define HIGH_HAT_ACCENT_PIN     10
#define CLOCK_ECHO_PIN          11
#define RANDOM_BIT_PIN          12
#define TEMPO_LED_PIN           13
#define BASS_FILL_PIN           A0
#define CHAOS_PIN               A1
#define SNARE_FILL_PIN          A2
#define Y_PIN                   A3
#define BASS_TRIGGER_PIN        20
#define SNARE_TRIGGER_PIN       19
#define HIGH_HAT_TRIGGER_PIN    18
#define HIGH_HAT_FILL_PIN       A7
#define X_PIN                   A9
#define TEMPO_PIN               A12

#define CLOCK_FREQ_HZ           8000
#define CLOCK_INTERVAL_uSEC     (1000000 / CLOCK_FREQ_HZ)
#define LONG_PRESS_MSEC         625

IntervalTimer clock_timer;
Bounce tap_bounce = Bounce(TAP_BUTTON_PIN, 10);

class Leds {

public:
    void Write(uint8_t state)
    {
        for (uint8_t i = 0; i < LED_COUNT; i++)
            digitalWrite(pin_map[i], (state & 1 << i) ? HIGH : LOW);
    }

private:
    enum { LED_COUNT = 4 };
    static const uint8_t pin_map[LED_COUNT];
};

const uint8_t Leds::pin_map[Leds::LED_COUNT] = {
    TEMPO_LED_PIN,
    HIGH_HAT_LED_PIN,
    SNARE_LED_PIN,
    BASS_LED_PIN,
};

class Inputs {

public:
    uint8_t Read()
    {
        return (digitalRead(CLOCK_INPUT_PIN) ? 0 : grids::INPUT_CLOCK |
                digitalRead(RESET_INPUT_PIN) ? 0 : grids::INPUT_RESET |
                digitalRead(TAP_BUTTON_PIN)  ? 0 : grids::INPUT_SW_RESET);
    }
};

class AdcInputScanner {

public:
    AdcInputScanner()
        : index(0)
    {
        analogReadResolution(16);
    }

    uint16_t Read8(uint8_t chan)
    {
        return values[chan] >> 8;
    }

    void Scan()
    {
        // Read one channel.
        values[index] = analogRead(pin_map[index]);
        if (++index == grids::ADC_CHANNEL_LAST)
            index = 0;
    }

private:
    volatile uint16_t values[grids::ADC_CHANNEL_LAST];
    volatile uint8_t index;

    static const uint8_t pin_map[grids::ADC_CHANNEL_LAST];
};

const uint8_t AdcInputScanner::pin_map[grids::ADC_CHANNEL_LAST] = {
    [grids::ADC_CHANNEL_X_CV]          = X_PIN,
    [grids::ADC_CHANNEL_Y_CV]          = Y_PIN,
    [grids::ADC_CHANNEL_RANDOMNESS_CV] = CHAOS_PIN,
    [grids::ADC_CHANNEL_BD_DENSITY_CV] = BASS_FILL_PIN,
    [grids::ADC_CHANNEL_SD_DENSITY_CV] = SNARE_FILL_PIN,
    [grids::ADC_CHANNEL_HH_DENSITY_CV] = HIGH_HAT_FILL_PIN,
    [grids::ADC_CHANNEL_TEMPO]         = TEMPO_PIN,
};

class ShiftRegister {

public:
    void Write(uint8_t state)
    {
        for (uint8_t i = 0; i < 8; i++)
            digitalWrite(pin_map[i], (state & 1 << i) ? HIGH : LOW);
    }

private:
    static const uint8_t pin_map[8];
};

const uint8_t ShiftRegister::pin_map[8] = {
    BASS_TRIGGER_PIN,
    SNARE_TRIGGER_PIN,
    HIGH_HAT_TRIGGER_PIN,
    BASS_ACCENT_PIN,
    SNARE_ACCENT_PIN,
    HIGH_HAT_ACCENT_PIN,
    CLOCK_ECHO_PIN,
    RANDOM_BIT_PIN,
};

class MidiInput {

public:
    static inline uint8_t readable()
    {
        return 0;
    }

    static inline uint8_t ImmediateRead()
    {
        return 0;
    }
};

namespace grids {
  using namespace avrlib;

  const uint8_t ticks_granularity[] = { 6, 3, 1 };

  enum Parameter {
    PARAMETER_NONE,
    PARAMETER_WAITING,
    PARAMETER_CLOCK_RESOLUTION,
    PARAMETER_TAP_TEMPO,
    PARAMETER_SWING,
    PARAMETER_GATE_MODE,
    PARAMETER_OUTPUT_MODE,
    PARAMETER_CLOCK_OUTPUT
  };

  enum SwitchState {
    SWITCH_STATE_PRESSED       = 0,
    SWITCH_STATE_RELEASED      = 1,
    SWITCH_STATE_JUST_PRESSED  = 2,
    SWITCH_STATE_JUST_RELEASED = 3,
  };

  Leds leds;
  Inputs inputs;
  AdcInputScanner adc;
  ShiftRegister shift_register;
  MidiInput midi;

  uint32_t tap_duration;
  uint8_t led_pattern;
  uint8_t led_off_timer;

  int8_t swing_amount;

  volatile Parameter parameter = PARAMETER_NONE;
  volatile bool long_press_detected = false;

  inline void HandleTapButton() {
    SwitchState switch_state;
    static uint16_t switch_hold_time = 0;

    if (tap_bounce.update()) {
        if (tap_bounce.fallingEdge())
            switch_state = SWITCH_STATE_JUST_PRESSED;
        else // tap_bounce.risingEdge()
            switch_state = SWITCH_STATE_JUST_RELEASED;
    } else {
        switch_state = (SwitchState)tap_bounce.read();
    }

    if (switch_state == SWITCH_STATE_JUST_PRESSED) {
      if (parameter == PARAMETER_NONE) {
        if (!pattern_generator.tap_tempo()) {
          pattern_generator.Reset();
          if (pattern_generator.factory_testing() ||
              clock.bpm() >= 40 ||
              clock.locked()) {
            clock.Reset();
          }
        } else {
#if 0            
          uint32_t new_bpm = (F_CPU * 60UL) / (32L * kUpdatePeriod * tap_duration);
#else
          uint32_t new_bpm = 60 * CLOCK_FREQ_HZ / tap_duration;
#endif
          if (new_bpm >= 30 && new_bpm <= 480) {
            clock.Update(new_bpm, pattern_generator.clock_resolution());
            clock.Reset();
            clock.Lock();
          } else {
            clock.Unlock();
          }
          tap_duration = 0;
        }
      }
      switch_hold_time = 0;
    } else if (switch_state == SWITCH_STATE_PRESSED) {
      ++switch_hold_time;
      if (switch_hold_time == 5000) { // 625 msec
        long_press_detected = true;
      }
    }
  }

  inline void HandleClockResetInputs() {
    static uint8_t previous_inputs;

    uint8_t inputs_value = ~inputs.Read();
    uint8_t num_ticks = 0;
    uint8_t increment = ticks_granularity[pattern_generator.clock_resolution()];

    // CLOCK
    if (clock.bpm() < 40 && !clock.locked()) {
      if ((inputs_value & INPUT_CLOCK) && !(previous_inputs & INPUT_CLOCK)) {
        num_ticks = increment;
      }
      if (!(inputs_value & INPUT_CLOCK) && (previous_inputs & INPUT_CLOCK)) {
        pattern_generator.ClockFallingEdge();
      }
      if (midi.readable()) {
        uint8_t byte = midi.ImmediateRead();
        if (byte == 0xf8) {
          num_ticks = 1;
        } else if (byte == 0xfa) {
          pattern_generator.Reset();
        }
      }
    } else {
      clock.Tick();
      clock.Wrap(swing_amount);
      if (clock.raising_edge()) {
        num_ticks = increment;
      }
      if (clock.past_falling_edge()) {
        pattern_generator.ClockFallingEdge();
      }
    }

    // RESET
    if ((inputs_value & INPUT_RESET) && !(previous_inputs & INPUT_RESET)) {
      pattern_generator.Reset();

      // !! HACK AHEAD !!
      // 
      // Earlier versions of the firmware retriggered the outputs
      // whenever a RESET signal was received. This allowed for nice
      // drill'n'bass effects, but made synchronization with another
      // sequencer a bit glitchy (risk of double notes at the
      // beginning of a pattern). It was later decided to remove this
      // behaviour and make the RESET transparent (just set the step
      // index without producing any trigger) - similar to the MIDI
      // START message. However, the factory testing script relies on
      // the old behaviour.  To solve this problem, we reproduce this
      // behaviour the first 5 times the module is powered. After the
      // 5th power-on (or settings change) cycle, this odd behaviour
      // disappears.
      if (pattern_generator.factory_testing() ||
          clock.bpm() >= 40 ||
          clock.locked()) {
        pattern_generator.Retrigger();
        clock.Reset();
      }
    }
    previous_inputs = inputs_value;

    if (num_ticks) {
      swing_amount = pattern_generator.swing_amount();
      pattern_generator.TickClock(num_ticks);
    }
  }

  inline void UpdateShiftRegister() {
    static uint8_t previous_state = 0;
    if (pattern_generator.state() != previous_state) {
      previous_state = pattern_generator.state();
      shift_register.Write(previous_state);
      if (!previous_state) {
        // Switch off the LEDs, but not now.
        led_off_timer = 200;
      } else {
        // Switch on the LEDs with a new pattern.
        led_pattern = pattern_generator.led_pattern();
        led_off_timer = 0;
      }
    }
  }

  inline void UpdateLeds() {
    uint8_t pattern;
    if (parameter == PARAMETER_NONE) {
      if (led_off_timer) {
        --led_off_timer;
        if (!led_off_timer) {
          led_pattern = 0;
        }
      }
      pattern = led_pattern;
      if (pattern_generator.tap_tempo()) {
        if (pattern_generator.on_beat()) {
          pattern |= LED_CLOCK;
        }
      } else {
        if (pattern_generator.on_first_beat()) {
          pattern |= LED_CLOCK;
        }
      }
    } else {
      pattern = LED_CLOCK;
      switch (parameter) {
        case PARAMETER_CLOCK_RESOLUTION:
          pattern |= LED_BD >> pattern_generator.clock_resolution();
          break;

        case PARAMETER_CLOCK_OUTPUT:
          if (pattern_generator.output_clock()) {
            pattern |= LED_ALL;
          }
          break;

        case PARAMETER_SWING:
          if (pattern_generator.swing()) {
            pattern |= LED_ALL;
          }
          break;

        case PARAMETER_OUTPUT_MODE:
          if (pattern_generator.output_mode() == OUTPUT_MODE_DRUMS) {
            pattern |= LED_ALL;
          }
          break;

        case PARAMETER_TAP_TEMPO:
          if (pattern_generator.tap_tempo()) {
            pattern |= LED_ALL;
          }
          break;

        case PARAMETER_GATE_MODE:
          if (pattern_generator.gate_mode()) {
            pattern |= LED_ALL;
          }
          break;

        case PARAMETER_NONE:
        case PARAMETER_WAITING:
          // do nothing
          break;
      }
    }
    leds.Write(pattern);
  }

  void ScanPots() {
    static int16_t pot_values[8];

    if (long_press_detected) {
      if (parameter == PARAMETER_NONE) {
        // Freeze pot values
        for (uint8_t i = 0; i < 8; ++i) {
          pot_values[i] = adc.Read8(i);
        }
        parameter = PARAMETER_WAITING;
      } else {
        parameter = PARAMETER_NONE;
        pattern_generator.SaveSettings();
      }
      long_press_detected = false;
    }

    if (parameter == PARAMETER_NONE) {
      uint8_t bpm = adc.Read8(ADC_CHANNEL_TEMPO);
      bpm = U8U8MulShift8(bpm, 220) + 20;
      if (bpm != clock.bpm() && !clock.locked()) {
        clock.Update(bpm, pattern_generator.clock_resolution());
      }
      PatternGeneratorSettings* settings = pattern_generator.mutable_settings();
      settings->options.drums.x = ~adc.Read8(ADC_CHANNEL_X_CV);
      settings->options.drums.y = ~adc.Read8(ADC_CHANNEL_Y_CV);
      settings->options.drums.randomness = ~adc.Read8(ADC_CHANNEL_RANDOMNESS_CV);
      settings->density[0] = ~adc.Read8(ADC_CHANNEL_BD_DENSITY_CV);
      settings->density[1] = ~adc.Read8(ADC_CHANNEL_SD_DENSITY_CV);
      settings->density[2] = ~adc.Read8(ADC_CHANNEL_HH_DENSITY_CV);
    } else {
      for (uint8_t i = 0; i < 8; ++i) {
        int16_t value = adc.Read8(i);
        int16_t delta = value - pot_values[i];
        if (delta < 0) {
          delta = -delta;
        }
        if (delta > 32) {
          pot_values[i] = value;
          switch (i) {
            case ADC_CHANNEL_BD_DENSITY_CV:
              parameter = PARAMETER_CLOCK_RESOLUTION;
              pattern_generator.set_clock_resolution((255 - value) >> 6);
              clock.Update(clock.bpm(), pattern_generator.clock_resolution());
              pattern_generator.Reset();
              break;

            case ADC_CHANNEL_SD_DENSITY_CV:
              parameter = PARAMETER_TAP_TEMPO;
              pattern_generator.set_tap_tempo(!(value & 0x80));
              if (!pattern_generator.tap_tempo()) {
                clock.Unlock();
              }
              break;

            case ADC_CHANNEL_HH_DENSITY_CV:
              parameter = PARAMETER_SWING;
              pattern_generator.set_swing(!(value & 0x80));
              break;

            case ADC_CHANNEL_X_CV:
              parameter = PARAMETER_OUTPUT_MODE;
              pattern_generator.set_output_mode(!(value & 0x80) ? 1 : 0);
              break;

            case ADC_CHANNEL_Y_CV:
              parameter = PARAMETER_GATE_MODE;
              pattern_generator.set_gate_mode(!(value & 0x80));
              break;

            case ADC_CHANNEL_RANDOMNESS_CV:
              parameter = PARAMETER_CLOCK_OUTPUT;
              pattern_generator.set_output_clock(!(value & 0x80));
              break;
          }
        }
      }
    }
  }
};

////////////////////////////////////////////////////////////////////////////////

static void clock_isr()
{
    ++grids::tap_duration;
    grids::HandleTapButton();
    grids::HandleClockResetInputs();

    grids::adc.Scan();
  
    grids::pattern_generator.IncrementPulseCounter();
    grids::UpdateShiftRegister();
    grids::UpdateLeds();
}

void setup()
{
    pinMode(MIDI_IN_PIN,          INPUT);
    pinMode(TAP_BUTTON_PIN,       INPUT_PULLUP);
    pinMode(CLOCK_INPUT_PIN,      INPUT_PULLUP);
    pinMode(RESET_INPUT_PIN,      INPUT_PULLUP);
    pinMode(BASS_LED_PIN,         OUTPUT);
    pinMode(SNARE_LED_PIN,        OUTPUT);
    pinMode(HIGH_HAT_LED_PIN,     OUTPUT);
    pinMode(BASS_ACCENT_PIN,      OUTPUT);
    pinMode(SNARE_ACCENT_PIN,     OUTPUT);
    pinMode(HIGH_HAT_ACCENT_PIN,  OUTPUT);
    pinMode(CLOCK_ECHO_PIN,       OUTPUT);
    pinMode(RANDOM_BIT_PIN,       OUTPUT);
    pinMode(TEMPO_LED_PIN,        OUTPUT);
    pinMode(BASS_FILL_PIN,        INPUT);
    pinMode(CHAOS_PIN,            INPUT);
    pinMode(SNARE_FILL_PIN,       INPUT);
    pinMode(Y_PIN,                INPUT);
    pinMode(BASS_TRIGGER_PIN,     OUTPUT);
    pinMode(SNARE_TRIGGER_PIN,    OUTPUT);
    pinMode(HIGH_HAT_TRIGGER_PIN, OUTPUT);
    pinMode(HIGH_HAT_FILL_PIN,    INPUT);
    pinMode(X_PIN,                INPUT);
    pinMode(TEMPO_PIN,            INPUT);

    digitalWrite(BASS_LED_PIN,         LOW);
    digitalWrite(SNARE_LED_PIN,        LOW);
    digitalWrite(HIGH_HAT_LED_PIN,     LOW);
    digitalWrite(BASS_ACCENT_PIN,      LOW);
    digitalWrite(SNARE_ACCENT_PIN,     LOW);
    digitalWrite(HIGH_HAT_ACCENT_PIN,  LOW);
    digitalWrite(CLOCK_ECHO_PIN,       LOW);
    digitalWrite(RANDOM_BIT_PIN,       LOW);
    digitalWrite(TEMPO_LED_PIN,        LOW);
    digitalWrite(BASS_TRIGGER_PIN,     LOW);
    digitalWrite(SNARE_TRIGGER_PIN,    LOW);
    digitalWrite(HIGH_HAT_TRIGGER_PIN, LOW);

    grids::clock.Init();
    grids::pattern_generator.Init();

    clock_timer.begin(clock_isr, CLOCK_INTERVAL_uSEC);
}

#define QN_MSEC (60 * 1000 / 120)

void loop()
{
    grids::ScanPots();
}
