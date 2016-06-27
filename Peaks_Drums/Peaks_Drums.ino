// The drum kit section of Mutable Instruments' Peaks module ported to Teensy
//
// Hardware:
//
//   Teensy 3.X with Teensy Audio Adapter Board
//
//   Audio Adapter Connections
//      GND
//      pin  9 - BCLK
//      pin 11 - MCLK
//      pin 13 - RX
//      pin 18 - SDA
//      pin 19 - SCL
//      pin 22 - TX
//      pin 23 - LRCLK
//      +3.3V
//
//   Digital Inputs
//      pin  2 - Bass Trigger
//      pin  3 - Snare Trigger
//      pin  4 - High Hat Trigger
//      pin  5 - Bass Accent
//      pin  6 - Snare Accent
//      pin  7 - High Hat Accent
//      pin 12 - Program Mode
//
//   Analog Inputs
//      pin 14 (A0) - frequency
//      pin 15 (A1) - punch
//      pin 16 (A2) - tone
//      pin 17 (A3) - decay/snappy
//
//   Digital Outputs
//      pin  8 - bass LED
//      pin 10 - snare LED
//      pin 20 - high hat LED


// Bass Parameters
//   - frequency
//   - punch
//   - tone
//   - decay
//
// Snare Parameters
//   - frequency
//   - punch
//   - tone
//   - snappy
//
// High Hat Parameters
//   (none)

#include "control_sgtl5000.h"
#include "kinetis.h"

#include "Bounce.h"

namespace peaks {

    // Code in this namespace is copied nearly verbatim from Peaks.
    // https://github.com/pichenettes/eurorack

    const uint16_t kAdcThresholdUnlocked = 1 << (16 - 10);  // 10 bits
    const uint16_t kAdcThresholdLocked = 1 << (16 - 8);  // 8 bits

    enum ControlBitMask {
      CONTROL_GATE = 1,
      CONTROL_GATE_RISING = 2,
      CONTROL_GATE_FALLING = 4,

      CONTROL_GATE_AUXILIARY = 16,
      CONTROL_GATE_RISING_AUXILIARY = 32,
      CONTROL_GATE_FALLING_AUXILIARY = 64
    };

    enum ControlMode {
      CONTROL_MODE_FULL,
      CONTROL_MODE_HALF
    };

    const uint16_t lut_svf_cutoff[] = {
          35,     37,     39,     41,
          44,     46,     49,     52,
          55,     58,     62,     66,
          70,     74,     78,     83,
          88,     93,     99,    105,
         111,    117,    124,    132,
         140,    148,    157,    166,
         176,    187,    198,    210,
         222,    235,    249,    264,
         280,    297,    314,    333,
         353,    374,    396,    420,
         445,    471,    499,    529,
         561,    594,    629,    667,
         706,    748,    793,    840,
         890,    943,    999,   1059,
        1122,   1188,   1259,   1334,
        1413,   1497,   1586,   1681,
        1781,   1886,   1999,   2117,
        2243,   2377,   2518,   2668,
        2826,   2994,   3172,   3361,
        3560,   3772,   3996,   4233,
        4485,   4751,   5033,   5332,
        5648,   5983,   6337,   6713,
        7111,   7532,   7978,   8449,
        8949,   9477,  10037,  10628,
       11254,  11916,  12616,  13356,
       14138,  14964,  15837,  16758,
       17730,  18756,  19837,  20975,
       22174,  23435,  24761,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,  25078,  25078,  25078,
       25078,
    };

    const uint16_t lut_svf_damp[] = {
       65534,  49166,  46069,  43993,
       42386,  41058,  39917,  38910,
       38007,  37184,  36427,  35726,
       35070,  34454,  33873,  33322,
       32798,  32299,  31820,  31361,
       30920,  30496,  30086,  29690,
       29306,  28935,  28574,  28224,
       27883,  27551,  27228,  26912,
       26605,  26304,  26010,  25723,
       25441,  25166,  24896,  24631,
       24371,  24116,  23866,  23620,
       23379,  23141,  22908,  22678,
       22452,  22229,  22010,  21794,
       21581,  21371,  21164,  20960,
       20759,  20560,  20365,  20171,
       19980,  19791,  19605,  19421,
       19239,  19059,  18882,  18706,
       18532,  18360,  18190,  18022,
       17856,  17691,  17528,  17367,
       17207,  17049,  16892,  16737,
       16583,  16431,  16280,  16131,
       15982,  15836,  15690,  15546,
       15403,  15261,  15120,  14981,
       14843,  14705,  14569,  14434,
       14300,  14167,  14036,  13905,
       13775,  13646,  13518,  13391,
       13265,  13140,  13015,  12892,
       12769,  12648,  12527,  12407,
       12287,  12169,  12051,  11934,
       11818,  11703,  11588,  11474,
       11361,  11249,  11137,  11026,
       10915,  10805,  10696,  10588,
       10480,  10373,  10266,  10160,
       10055,   9950,   9846,   9742,
        9639,   9537,   9435,   9333,
        9233,   9132,   9033,   8933,
        8835,   8737,   8639,   8542,
        8445,   8349,   8253,   8158,
        8063,   7969,   7875,   7782,
        7689,   7596,   7504,   7413,
        7321,   7231,   7140,   7050,
        6961,   6872,   6783,   6695,
        6607,   6519,   6432,   6346,
        6259,   6173,   6088,   6003,
        5918,   5833,   5749,   5665,
        5582,   5499,   5416,   5334,
        5251,   5170,   5088,   5007,
        4926,   4846,   4766,   4686,
        4607,   4527,   4449,   4370,
        4292,   4214,   4136,   4059,
        3982,   3905,   3828,   3752,
        3676,   3601,   3525,   3450,
        3375,   3301,   3226,   3152,
        3078,   3005,   2932,   2859,
        2786,   2713,   2641,   2569,
        2497,   2426,   2355,   2284,
        2213,   2142,   2072,   2002,
        1932,   1862,   1793,   1724,
        1655,   1586,   1518,   1449,
        1381,   1313,   1246,   1178,
        1111,   1044,    977,    911,
         844,    778,    712,    647,
         581,    516,    450,    385,
         321,    256,    192,    127,
          63,
    };

    static inline int32_t CLIP(int32_t sample) {
        if (sample < -32768)
            return -32768;
        if (sample > +32767)
            return +32767;
        return sample;
    }

    inline int16_t Interpolate824(const int16_t* table, uint32_t phase) {
      int32_t a = table[phase >> 24];
      int32_t b = table[(phase >> 24) + 1];
      return a + ((b - a) * static_cast<int32_t>((phase >> 8) & 0xffff) >> 16);
    }

    inline uint16_t Interpolate824(const uint16_t* table, uint32_t phase) {
      uint32_t a = table[phase >> 24];
      uint32_t b = table[(phase >> 24) + 1];
      return a + ((b - a) * static_cast<uint32_t>((phase >> 8) & 0xffff) >> 16);
    }

    inline int16_t Interpolate824(const uint8_t* table, uint32_t phase) {
      int32_t a = table[phase >> 24];
      int32_t b = table[(phase >> 24) + 1];
      return (a << 8) + \
          ((b - a) * static_cast<int32_t>(phase & 0xffffff) >> 16) - 32768;
    }

    class Excitation {
     public:
      Excitation() { }
      ~Excitation() { }

      void Init() {
        delay_ = 0;
        decay_ = 4093;
        counter_ = 0;
        state_ = 0;
      }

      void set_delay(uint16_t delay) {
        delay_ = delay;
      }

      void set_decay(uint16_t decay) {
        decay_ = decay;
      }

      void Trigger(int32_t level) {
        level_ = level;
        counter_ = delay_ + 1;
      }

      bool done() {
        return counter_ == 0;
      }

      inline int32_t Process() {
        state_ = (state_ * decay_ >> 12);
        if (counter_ > 0) {
          --counter_;
          if (counter_ == 0) {
            state_ += level_ < 0 ? -level_ : level_;
          }
        }
        return level_ < 0 ? -state_ : state_;
      }

     private:
      uint32_t delay_;
      uint32_t decay_;
      int32_t counter_;
      int32_t state_;
      int32_t level_;

    };

    enum SvfMode {
      SVF_MODE_LP,
      SVF_MODE_BP,
      SVF_MODE_HP
    };

    class Svf {
     public:
      Svf() { }
      ~Svf() { }

      void Init() {
        lp_ = 0;
        bp_ = 0;
        frequency_ = 33 << 7;
        resonance_ = 16384;
        dirty_ = true;
        punch_ = 0;
        mode_ = SVF_MODE_BP;
      }

      void set_frequency(int16_t frequency) {
        dirty_ = dirty_ || (frequency_ != frequency);
        frequency_ = frequency;
      }

      void set_resonance(int16_t resonance) {
        resonance_ = resonance;
        dirty_ = true;
      }

      void set_punch(uint16_t punch) {
        punch_ = (static_cast<uint32_t>(punch) * punch) >> 24;
      }

      void set_mode(SvfMode mode) {
        mode_ = mode;
      }

      int32_t Process(int32_t in) {
        if (dirty_) {
          f_ = Interpolate824(lut_svf_cutoff, frequency_ << 17);
          damp_ = Interpolate824(lut_svf_damp, resonance_ << 17);
          dirty_ = false;
        }
        int32_t f = f_;
        int32_t damp = damp_;
        if (punch_) {
          int32_t punch_signal = lp_ > 4096 ? lp_ : 2048;
          f += ((punch_signal >> 4) * punch_) >> 9;
          damp += ((punch_signal - 2048) >> 3);
        }
        int32_t notch = in - (bp_ * damp >> 15);
        lp_ += f * bp_ >> 15;
        lp_ = CLIP(lp_);
        int32_t hp = notch - lp_;
        bp_ += f * hp >> 15;
        bp_ = CLIP(bp_);

        return mode_ == SVF_MODE_BP ? bp_ : (mode_ == SVF_MODE_HP ? hp : lp_);
      }

     private:
      bool dirty_;

      int16_t frequency_;
      int16_t resonance_;

      int32_t punch_;
      int32_t f_;
      int32_t damp_;

      int32_t lp_;
      int32_t bp_;

      SvfMode mode_;
    };

    class Random {
     public:
      static inline uint32_t state() { return rng_state_; }

      static inline void Seed(uint16_t seed) {
        rng_state_ = seed;
      }

      static inline uint32_t GetWord() {
        rng_state_ = rng_state_ * 1664525L + 1013904223L;
        return state();
      }

      static inline int16_t GetSample() {
        return static_cast<int16_t>(GetWord() >> 16);
      }

      static inline float GetFloat() {
        return static_cast<float>(GetWord()) / 4294967296.0f;
      }

     private:
      static uint32_t rng_state_;
    };

    uint32_t Random::rng_state_ = 0x21;

    class BassDrum {
     public:
      BassDrum() { }
      ~BassDrum() { }

      void Init() {
        pulse_up_.Init();
        pulse_down_.Init();
        attack_fm_.Init();
        resonator_.Init();

        pulse_up_.set_delay(0);
        pulse_up_.set_decay(3340);

        pulse_down_.set_delay(1.0e-3 * 48000);
        pulse_down_.set_decay(3072);

        attack_fm_.set_delay(4.0e-3 * 48000);
        attack_fm_.set_decay(4093);

        resonator_.set_punch(32768);
        resonator_.set_mode(SVF_MODE_BP);

        set_frequency(0);
        set_decay(32768);
        set_tone(32768);
        set_punch(65535);

        lp_state_ = 0;
      }

      int16_t ProcessSingleSample(uint8_t control) {
        if (control & CONTROL_GATE_RISING) {
          pulse_up_.Trigger(12 * 32768 * 0.7);
          pulse_down_.Trigger(-19662 * 0.7);
          attack_fm_.Trigger(18000);
        }
        int32_t excitation = 0;
        excitation += pulse_up_.Process();
        excitation += !pulse_down_.done() ? 16384 : 0;
        excitation += pulse_down_.Process();
        attack_fm_.Process();
        resonator_.set_frequency(frequency_ +
                                 (attack_fm_.done() ? 0 : 17 << 7));

        int32_t resonator_output =
            (excitation >> 4) + resonator_.Process(excitation);
        lp_state_ += (resonator_output - lp_state_) * lp_coefficient_ >> 15;
        int32_t output = lp_state_;
        output = CLIP(output);
        return output;
      }

      void Configure(uint16_t* parameter, ControlMode control_mode) {
        if (control_mode == CONTROL_MODE_HALF) {
          set_frequency(0);
          set_punch(40000);
          set_tone(8192 + (parameter[0] >> 1));
          set_decay(parameter[1]);
        } else {
          set_frequency(parameter[0] - 32768);
          set_punch(parameter[1]);
          set_tone(parameter[2]);
          set_decay(parameter[3]);
        }
      }

      void set_frequency(int16_t frequency) {
        frequency_ = (31 << 7) + (static_cast<int32_t>(frequency) * 896 >> 15);
      }

      void set_decay(uint16_t decay) {
        uint32_t scaled;
        uint32_t squared;
        scaled = 65535 - decay;
        squared = scaled * scaled >> 16;
        scaled = squared * scaled >> 18;
        resonator_.set_resonance(32768 - 128 - scaled);
      }

      void set_tone(uint16_t tone) {
        uint32_t coefficient = tone;
        coefficient = coefficient * coefficient >> 16;
        lp_coefficient_ = 512 + (coefficient >> 2) * 3;
      }

      void set_punch(uint16_t punch) {
        resonator_.set_punch(punch * punch >> 16);
      }

     private:
      Excitation pulse_up_;
      Excitation pulse_down_;
      Excitation attack_fm_;
      Svf resonator_;

      int32_t frequency_;
      int32_t lp_coefficient_;
      int32_t lp_state_;
    };

    class SnareDrum {
     public:
      SnareDrum() { }
      ~SnareDrum() { }

      void Init() {
        excitation_1_up_.Init();
        excitation_1_up_.set_delay(0);
        excitation_1_up_.set_decay(1536);

        excitation_1_down_.Init();
        excitation_1_down_.set_delay(1e-3 * 48000);
        excitation_1_down_.set_decay(3072);

        excitation_2_.Init();
        excitation_2_.set_delay(1e-3 * 48000);
        excitation_2_.set_decay(1200);

        excitation_noise_.Init();
        excitation_noise_.set_delay(0);

        body_1_.Init();
        body_2_.Init();

        noise_.Init();
        noise_.set_resonance(2000);
        noise_.set_mode(SVF_MODE_BP);

        set_tone(0);
        set_snappy(32768);
        set_decay(32768);
        set_frequency(0);
      }

      int16_t ProcessSingleSample(uint8_t control) {
        if (control & CONTROL_GATE_RISING) {
          excitation_1_up_.Trigger(15 * 32768);
          excitation_1_down_.Trigger(-1 * 32768);
          excitation_2_.Trigger(13107);
          excitation_noise_.Trigger(snappy_);
        }

        int32_t excitation_1 = 0;
        excitation_1 += excitation_1_up_.Process();
        excitation_1 += excitation_1_down_.Process();
        excitation_1 += !excitation_1_down_.done() ? 2621 : 0;

        int32_t body_1 = body_1_.Process(excitation_1) + (excitation_1 >> 4);

        int32_t excitation_2 = 0;
        excitation_2 += excitation_2_.Process();
        excitation_2 += !excitation_2_.done() ? 13107 : 0;

        int32_t body_2 = body_2_.Process(excitation_2) + (excitation_2 >> 4);
        int32_t noise_sample = Random::GetSample();
        int32_t noise = noise_.Process(noise_sample);
        int32_t noise_envelope = excitation_noise_.Process();
        int32_t sd = 0;
        sd += body_1 * gain_1_ >> 15;
        sd += body_2 * gain_2_ >> 15;
        sd += noise_envelope * noise >> 15;
        sd = CLIP(sd);
        return sd;
      }

      void Configure(uint16_t* parameter, ControlMode control_mode) {
        if (control_mode == CONTROL_MODE_HALF) {
          set_frequency(0);
          set_decay(32768);
          set_tone(parameter[0]);
          set_snappy(parameter[1]);
        } else {
          set_frequency(parameter[0] - 32768);
          set_tone(parameter[1]);
          set_snappy(parameter[2]);
          set_decay(parameter[3]);
        }
      }

      void set_tone(uint16_t tone) {
        gain_1_ = 22000 - (tone >> 2);
        gain_2_ = 22000 + (tone >> 2);
      }

      void set_snappy(uint16_t snappy) {
        snappy >>= 1;
        if (snappy >= 28672) {
          snappy = 28672;
        }
        snappy_ = 512 + snappy;
      }

      void set_decay(uint16_t decay) {
        body_1_.set_resonance(29000 + (decay >> 5));
        body_2_.set_resonance(26500 + (decay >> 5));
        excitation_noise_.set_decay(4092 + (decay >> 14));
      }

      void set_frequency(int16_t frequency) {
        int16_t base_note = 52 << 7;
        int32_t transposition = frequency;
        base_note += transposition * 896 >> 15;
        body_1_.set_frequency(base_note);
        body_2_.set_frequency(base_note + (12 << 7));
        noise_.set_frequency(base_note + (48 << 7));
      }

     private:
      Excitation excitation_1_up_;
      Excitation excitation_1_down_;
      Excitation excitation_2_;
      Excitation excitation_noise_;
      Svf body_1_;
      Svf body_2_;
      Svf noise_;

      int32_t gain_1_;
      int32_t gain_2_;

      uint16_t snappy_;
    };

    class HighHat {
     public:
      HighHat() { }
      ~HighHat() { }

      void Init() {
        noise_.Init();
        noise_.set_frequency(105 << 7);  // 8kHz
        noise_.set_resonance(24000);
        noise_.set_mode(SVF_MODE_BP);

        vca_coloration_.Init();
        vca_coloration_.set_frequency(110 << 7);  // 13kHz
        vca_coloration_.set_resonance(0);
        vca_coloration_.set_mode(SVF_MODE_HP);

        vca_envelope_.Init();
        vca_envelope_.set_delay(0);
        vca_envelope_.set_decay(4093);
      }

      int16_t ProcessSingleSample(uint8_t control) {
        if (control & CONTROL_GATE_RISING) {
          vca_envelope_.Trigger(32768 * 15);
        }

        phase_[0] += 48318382;
        phase_[1] += 71582788;
        phase_[2] += 37044092;
        phase_[3] += 54313440;
        phase_[4] += 66214079;
        phase_[5] += 93952409;

        int16_t noise = 0;
        noise += phase_[0] >> 31;
        noise += phase_[1] >> 31;
        noise += phase_[2] >> 31;
        noise += phase_[3] >> 31;
        noise += phase_[4] >> 31;
        noise += phase_[5] >> 31;
        noise <<= 12;

        // Run the SVF at the double of the original sample rate for stability.
        int32_t filtered_noise = 0;
        filtered_noise += noise_.Process(noise);
        filtered_noise += noise_.Process(noise);

        // The 808-style VCA amplifies only the positive section of the signal.
        if (filtered_noise < 0) {
          filtered_noise = 0;
        } else if (filtered_noise > 32767) {
          filtered_noise = 32767;
        }

        int32_t envelope = vca_envelope_.Process() >> 4;
        int32_t vca_noise = envelope * filtered_noise >> 14;
        vca_noise = CLIP(vca_noise);
        int32_t hh = 0;
        hh += vca_coloration_.Process(vca_noise);
        hh += vca_coloration_.Process(vca_noise);
        hh <<= 1;
        hh = CLIP(hh);
        return hh;
      }

      void Configure(uint16_t* parameter, ControlMode control_mode) { }

     private:
      Svf noise_;
      Svf vca_coloration_;
      Excitation vca_envelope_;

      uint32_t phase_[6];
    };

} // end namespace peaks

//////////////////////////////////////////////////////////////////////////////

#define BASS_TRIGGER_PIN         2
#define SNARE_TRIGGER_PIN        3
#define HIGH_HAT_TRIGGER_PIN     4
#define BASS_ACCENT_PIN          5
#define SNARE_ACCENT_PIN         6
#define HIGH_HAT_ACCENT_PIN      7
#define PROGRAM_MODE_PIN        12
#define PARAM1_PIN              A0
#define PARAM2_PIN              A1
#define PARAM3_PIN              A2
#define PARAM4_PIN              A3

#define BASS_LED_PIN 8
#define SNARE_LED_PIN 10
#define HIGH_HAT_LED_PIN 20

#define LED_SHORT_MSEC 50
#define LED_LONG_MSEC 500

const uint8_t pin_map[] = {
    PARAM1_PIN,
    PARAM2_PIN,
    PARAM3_PIN,
    PARAM4_PIN,
};
const size_t PARAM_COUNT = (&pin_map)[1] - pin_map;

AudioControlSGTL5000 audioShield;

peaks::BassDrum bass;
peaks::SnareDrum snare;
peaks::HighHat high_hat;

peaks::ControlBitMask bass_trigger;
peaks::ControlBitMask snare_trigger;
peaks::ControlBitMask high_hat_trigger;

bool bass_is_accented;
bool snare_is_accented;
bool high_hat_is_accented;

Bounce bass_trigger_input = Bounce(BASS_TRIGGER_PIN, 1);
Bounce snare_trigger_input = Bounce(SNARE_TRIGGER_PIN, 1);
Bounce high_hat_trigger_input = Bounce(HIGH_HAT_TRIGGER_PIN, 1);
Bounce program_mode_input = Bounce(PROGRAM_MODE_PIN, 20);

uint16_t prev_pot_values[PARAM_COUNT];
uint16_t pot_threshold[PARAM_COUNT];
uint8_t mod_select;

void LockPots()
{
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        pot_threshold[i] = peaks::kAdcThresholdLocked;
        prev_pot_values[i] = analogRead(pin_map[i]);
    }
}

void UnlockPot(size_t index)
{
    pot_threshold[index] = peaks::kAdcThresholdUnlocked;
}

void setup_drums()
{
    bass.Init();
    snare.Init();
    high_hat.Init();
}

void next_sample(int16_t *left_sample, int16_t *right_sample)
{
    int32_t b = bass.ProcessSingleSample(bass_trigger) >> 2;
    int32_t s = snare.ProcessSingleSample(snare_trigger) >> 2;
    int32_t h = high_hat.ProcessSingleSample(high_hat_trigger);

    if (bass_is_accented)
        b <<= 1;
    if (snare_is_accented)
        s <<= 1;
    if (high_hat_is_accented)
        h <<= 1;

    bass_trigger = peaks::CONTROL_GATE;
    snare_trigger = peaks::CONTROL_GATE;
    high_hat_trigger = peaks::CONTROL_GATE;

    *left_sample = peaks::CLIP(b + s);
    *right_sample = peaks::CLIP(b + h);
}

void config_i2s(void)
{
    // MCLK needs to be 48e6 / 1088 * 256 = 11.29411765 MHz ->
    // 44.117647 kHz sample rate
    //
    #if F_CPU == 96000000 || F_CPU == 48000000 || F_CPU == 24000000
      // PLL is at 96 MHz in these modes
      #define MCLK_MULT 2
      #define MCLK_DIV  17
    #elif F_CPU == 72000000
      #define MCLK_MULT 8
      #define MCLK_DIV  51
    #elif F_CPU == 120000000
      #define MCLK_MULT 8
      #define MCLK_DIV  85
    #elif F_CPU == 144000000
      #define MCLK_MULT 4
      #define MCLK_DIV  51
    #elif F_CPU == 168000000
      #define MCLK_MULT 8
      #define MCLK_DIV  119
    #elif F_CPU == 16000000
      #define MCLK_MULT 12
      #define MCLK_DIV  17
    #else
      #error "This CPU Clock Speed is not supported by the Audio library";
    #endif

    #if F_CPU >= 20000000
      #define MCLK_SRC  3  // the PLL
    #else
      #define MCLK_SRC  0  // system clock
    #endif

    SIM_SCGC6 |= SIM_SCGC6_I2S;

    // enable MCLK output
    I2S0_MCR = I2S_MCR_MICS(MCLK_SRC) | I2S_MCR_MOE;
    I2S0_MDR = I2S_MDR_FRACT((MCLK_MULT-1)) | I2S_MDR_DIVIDE((MCLK_DIV-1));

    // configure transmitter
    I2S0_TMR = 0;
    I2S0_TCR1 = I2S_TCR1_TFW(1);  // watermark at half fifo size
    I2S0_TCR2 = I2S_TCR2_SYNC(0) | I2S_TCR2_BCP | I2S_TCR2_MSEL(1)
              | I2S_TCR2_BCD | I2S_TCR2_DIV(3);
    I2S0_TCR3 = I2S_TCR3_TCE;
    I2S0_TCR4 = I2S_TCR4_FRSZ(1) | I2S_TCR4_SYWD(15) | I2S_TCR4_MF
              | I2S_TCR4_FSE | I2S_TCR4_FSP | I2S_TCR4_FSD;
    I2S0_TCR5 = I2S_TCR5_WNW(15) | I2S_TCR5_W0W(15) | I2S_TCR5_FBT(15);

    // configure pin mux for 3 clock signals
    CORE_PIN23_CONFIG = PORT_PCR_MUX(6); // pin 23, PTC2, I2S0_TX_FS (LRCLK)
    CORE_PIN9_CONFIG  = PORT_PCR_MUX(6); // pin  9, PTC3, I2S0_TX_BCLK
    CORE_PIN11_CONFIG = PORT_PCR_MUX(6); // pin 11, PTC6, I2S0_MCLK
    CORE_PIN22_CONFIG = PORT_PCR_MUX(6); // pin 22, PTC1, I2S0_TXD0
}

void setup_i2s()
{
    config_i2s();
    NVIC_ENABLE_IRQ(IRQ_I2S0_TX);
    I2S0_TCSR |= I2S_TCSR_TE
               | I2S_TCSR_BCE
               | I2S_TCSR_FR
               | I2S_TCSR_FRIE
               | I2S_TCSR_FEIE;
}

void i2s0_tx_isr(void)
{
    if (I2S0_TCSR & I2S_TCSR_FEF)
        I2S0_TCSR |= I2S_TCSR_FEF;
    while (I2S0_TCSR & I2S_TCSR_FRF) {
        int16_t left_sample, right_sample;
        next_sample(&left_sample, &right_sample);
        I2S0_TDR0 = left_sample;
        I2S0_TDR0 = right_sample;
    }
}

void setup_triggers()
{
    // pinMode(..., INPUT_PULLDOWN);
    *portConfigRegister(BASS_TRIGGER_PIN)     = PORT_PCR_MUX(1) | PORT_PCR_PE;
    *portConfigRegister(SNARE_TRIGGER_PIN)    = PORT_PCR_MUX(1) | PORT_PCR_PE;
    *portConfigRegister(HIGH_HAT_TRIGGER_PIN) = PORT_PCR_MUX(1) | PORT_PCR_PE;
    *portConfigRegister(PROGRAM_MODE_PIN)     = PORT_PCR_MUX(1) | PORT_PCR_PE;
    *portConfigRegister(BASS_ACCENT_PIN)      = PORT_PCR_MUX(1) | PORT_PCR_PE;
    *portConfigRegister(SNARE_ACCENT_PIN)     = PORT_PCR_MUX(1) | PORT_PCR_PE;
    *portConfigRegister(HIGH_HAT_ACCENT_PIN)  = PORT_PCR_MUX(1) | PORT_PCR_PE;

    pinMode(BASS_LED_PIN, OUTPUT);
    pinMode(SNARE_LED_PIN, OUTPUT);
    pinMode(HIGH_HAT_LED_PIN, OUTPUT);
    digitalWrite(BASS_LED_PIN, LOW);
    digitalWrite(SNARE_LED_PIN, LOW);
    digitalWrite(HIGH_HAT_LED_PIN, LOW);
}

void setup()
{
    setup_drums();

    setup_i2s();

    audioShield.enable();
    // audioShield.volume(0.45);
    audioShield.volume(0.80);

    setup_triggers();

    analogReadResolution(16);
    LockPots();
}

void loop()
{
    static uint32_t bass_LED_timer;
    static uint32_t snare_LED_timer;
    static uint32_t high_hat_LED_timer;

    bass_trigger_input.update();
    snare_trigger_input.update();
    high_hat_trigger_input.update();
    program_mode_input.update();
    if (bass_trigger_input.risingEdge()) {
        bass_trigger = peaks::CONTROL_GATE_RISING;
        bass_is_accented = digitalRead(BASS_ACCENT_PIN);
        digitalWrite(BASS_LED_PIN, HIGH);
        if (bass_LED_timer < LED_SHORT_MSEC)
            bass_LED_timer = LED_SHORT_MSEC;
    }
    if (snare_trigger_input.risingEdge()) {
        snare_trigger = peaks::CONTROL_GATE_RISING;
        snare_is_accented = digitalRead(SNARE_ACCENT_PIN);
        digitalWrite(SNARE_LED_PIN, HIGH);
        if (snare_LED_timer < LED_SHORT_MSEC)
            snare_LED_timer = LED_SHORT_MSEC;
    }
    if (high_hat_trigger_input.risingEdge()) {
        high_hat_trigger = peaks::CONTROL_GATE_RISING;
        high_hat_is_accented = digitalRead(HIGH_HAT_ACCENT_PIN);
        digitalWrite(HIGH_HAT_LED_PIN, HIGH);
        if (high_hat_LED_timer < LED_SHORT_MSEC)
            high_hat_LED_timer = LED_SHORT_MSEC;
    }
    if (program_mode_input.risingEdge()) {
        mod_select = !mod_select;
        *(mod_select ? &snare_LED_timer : &bass_LED_timer) = LED_LONG_MSEC;
        if (mod_select) {
            digitalWrite(SNARE_LED_PIN, HIGH);
            snare_LED_timer = LED_LONG_MSEC;
        } else {
            digitalWrite(BASS_LED_PIN, HIGH);
            bass_LED_timer = LED_LONG_MSEC;
        }
        LockPots();
    }

    static uint32_t last_time;
    uint32_t now = millis();
    if (now == last_time)
        return;
    last_time = now;

    // Once a millisecond...
    if (bass_LED_timer && !--bass_LED_timer)
        digitalWrite(BASS_LED_PIN, LOW);
    if (snare_LED_timer && !--snare_LED_timer)
        digitalWrite(SNARE_LED_PIN, LOW);
    if (high_hat_LED_timer && !--high_hat_LED_timer)
        digitalWrite(HIGH_HAT_LED_PIN, LOW);

    // scan next param
    // if pot moved, update drum

    static uint16_t current_params[2][PARAM_COUNT] = {
        { 32768, 65535, 32768, 32768 },
        { 32768,     0, 32768, 32768 },
    };
    static uint8_t param_index;

    uint16_t prev_val = prev_pot_values[param_index];
    uint16_t pot_val = analogRead(pin_map[param_index]);
    if (abs(pot_val - prev_val) >= pot_threshold[param_index]) {
        prev_pot_values[param_index] = pot_val;
        current_params[mod_select][param_index] = pot_val;
        UnlockPot(param_index);
        if (mod_select)
            snare.Configure(current_params[1], peaks::CONTROL_MODE_FULL);
        else
            bass.Configure(current_params[0], peaks::CONTROL_MODE_FULL);
    }

    param_index++;
    param_index %= PARAM_COUNT;
}
