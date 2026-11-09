// ===========================================================================
//  Polyphonic FM synth + effects  -  ESP32-WROVER-KIT
//  DIN-5 MIDI in (6N138 -> UART2 RX) -> I2S DAC out, split-screen UI on the LCD
//
//  Sounds:   Rhodes, Pad, Organ, Vibes              (encoder 1)
//  Effects:  Reverb, Delay, Chorus, Phaser          (encoder 2)
//  Output:   Volume / Gain                          (encoder 3)
//
//  Arduino IDE setup:
//    * Boards Manager: "esp32 by Espressif Systems" version 2.0.x  (e.g. 2.0.17)
//    * Board: "ESP32 Wrover Module"   CPU 240 MHz   (PSRAM setting doesn't matter)
//    * Libraries (Library Manager): "Adafruit GFX Library", "Adafruit ILI9341"
//
//  Controls
//    Encoder 1 turn ......... select sound
//    Encoder 2 turn ......... change amount of the highlighted effect
//    Encoder 2 push ......... highlight next effect   (BOOT button on the kit does the same)
//    Encoder 2 long push .... switch encoder 3 between VOLUME and GAIN
//    Encoder 3 turn ......... volume (or gain)
//    MIDI: PC = sound, CC7 volume, CC91 reverb, CC94 delay, CC93 chorus, CC95 phaser
//    Serial Monitor: 'n'/'p' = next/previous sound, 'f' = next effect
// ===========================================================================
#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "esp_heap_caps.h"
#include "driver/i2s.h"
#include "driver/uart.h"

// ---------------------------------------------------------------------------
//  Hardware + engine configuration
// ---------------------------------------------------------------------------

// --- Pins --------------------------------------------------------------------
// GPIO16/17 = PSRAM (never use).  GPIO12 = flash-voltage strapping pin (avoid).
#define PIN_MIDI_RX    34   // from 6N138 optocoupler output (input-only pin)
#define PIN_I2S_DOUT   14   // DAC DIN
#define PIN_I2S_BCK    26   // DAC BCK
#define PIN_I2S_WS     27   // DAC LRCK / WS

// Built-in LCD (already wired on the WROVER-KIT V4.1)
#define LCD_RESET      18
#define LCD_SCLK       19
#define LCD_DC         21
#define LCD_CS         22
#define LCD_MOSI       23
#define LCD_BL         5
#define LCD_BL_ON      LOW  // backlight is switched by a transistor: LOW = on

// --- Rotary encoders ---------------------------------------------------------
// Pins 35/36/39 are input-only and have NO internal pull-ups: the encoder board
// must supply them (KY-040 style modules do; for bare encoders add 10k to 3.3V).
// Swap A/B (or set ENC_REVERSE) if an encoder turns the wrong way.
#define ENC1_A         35   // encoder 1: sound select
#define ENC1_B         36
#define ENC2_A         39   // encoder 2: effect amount
#define ENC2_B         25
#define ENC3_A         13   // encoder 3: volume / gain
#define ENC3_B         15
#define BTN_PIN        0    // GPIO0 = BOOT button on the kit. Wire encoder 2's push switch
                            // between GPIO0 and GND to use it (parallel to BOOT).
#define ENC_STEPS_PER_DETENT 4   // 4 for most encoders, 2 if one click gives 2 steps
#define ENC_REVERSE    0
#define LONG_PRESS_MS  600

// --- MIDI --------------------------------------------------------------------
#define MIDI_UART      UART_NUM_2
#define MIDI_CHANNEL   0    // 0 = OMNI (all channels), 1..16 = single channel
#define MIDI_DEBUG     0    // 1 = print every incoming MIDI message to Serial

// --- Audio -------------------------------------------------------------------
#define SAMPLE_RATE    44100
#define BLOCK_FRAMES   64        // 64 frames @44.1k = 1.45 ms per block
#define NUM_VOICES     16
#define MOD_DEPTH      1.5f      // modulation index (in cycles) at level 1.0
#define MASTER_GAIN    0.30f

// --- Debug -------------------------------------------------------------------
#define BOOT_TEST_TONE 1         // play a short C major chord at boot (no MIDI needed)

// MIDI event passed from the MIDI/UI tasks to the audio task.
// (Declared up here on purpose: the Arduino IDE auto-generates function prototypes
//  above the first function, so types used in function signatures must come first.)
struct MidiMsg { uint8_t status, d1, d2; };

// ===== BEGIN DSP (hardware independent; compiled on a PC for testing) =======

// Effect indices (order used everywhere: arrays, UI, MIDI)
enum { FX_REVERB = 0, FX_DELAY = 1, FX_CHORUS = 2, FX_PHASER = 3, NUM_FX = 4 };

// ---------------------------------------------------------------------------
//  Patches.  Times in seconds; decay/release = "time to fall 60 dB".
//  op[0] = OP1 ... op[3] = OP4 (OP4 has the feedback loop).
//  keyLvl: level change per octave away from middle C (negative = softer/duller
//          at the top; used on modulators for natural key-scaling of brightness)
//  keyDecay: decay/release gets shorter by 2^(keyDecay) per octave above middle C
// ---------------------------------------------------------------------------
struct OpPatch {
    float ratio, detune, level, a, d, s, r, velSens, keyLvl;
};

struct Patch {
    const char* name;
    uint8_t alg;      // 0..7, see algorithm list below
    float feedback;   // 0..~0.5 (cycles) on OP4
    float keyDecay;
    float fx[NUM_FX]; // default effect amounts: reverb, delay, chorus, phaser
    OpPatch op[4];
};

static const Patch PATCHES[] = {
  // Rhodes (mellow / dark): two slightly detuned carriers give slow beating warmth.
  // OP2 adds a 1:1 "bark" that blooms with velocity and fades over ~2 s,
  // OP4 is a faint, very short tine tick. (Modulators share their carrier's detune.) Higher notes get duller and shorter.
  { "Rhodes", 3, 0.00f, 0.35f, { 0.25f, 0.00f, 0.50f, 0.00f }, {
      {  1.0f, -4, 0.62f, 0.006f, 5.50f, 0.00f, 0.50f, 0.35f,  0.00f },
      {  1.0f, -4, 0.420f, 0.003f, 2.40f, 0.00f, 0.30f, 0.95f, -0.35f },
      {  1.0f,  5, 0.50f, 0.006f, 4.50f, 0.00f, 0.50f, 0.35f,  0.00f },
      { 14.0f,  5, 0.100f, 0.002f, 0.20f, 0.00f, 0.10f, 1.00f, -0.50f } } },

  // Pad: three carriers (two detuned unisons + octave shimmer), slow bloom.
  { "Pad", 6, 0.08f, 0.00f, { 0.45f, 0.12f, 0.40f, 0.12f }, {
      {  1.0f, -9, 0.36f, 0.55f, 1.50f, 0.90f, 1.60f, 0.15f,  0.00f },
      {  1.0f,  0, 0.09f, 0.90f, 2.00f, 0.55f, 1.60f, 0.25f, -0.15f },
      {  1.0f,  9, 0.36f, 0.65f, 1.50f, 0.90f, 1.60f, 0.15f,  0.00f },
      {  2.0f,  3, 0.14f, 0.80f, 1.50f, 0.70f, 1.60f, 0.20f, -0.10f } } },

  // Organ: additive drawbars (16', 8', 4', 2 2/3'), no decay, short release.
  { "Organ", 7, 0.06f, 0.00f, { 0.18f, 0.00f, 0.30f, 0.40f }, {
      {  1.0f, 0, 0.36f, 0.006f, 0.05f, 1.00f, 0.05f, 0.00f, 0.00f },
      {  0.5f, 0, 0.28f, 0.006f, 0.05f, 1.00f, 0.05f, 0.00f, 0.00f },
      {  2.0f, 0, 0.24f, 0.006f, 0.05f, 1.00f, 0.05f, 0.00f, 0.00f },
      {  3.0f, 0, 0.14f, 0.006f, 0.05f, 1.00f, 0.05f, 0.00f, 0.00f } } },

  // Vibes: soft mallet "tick" (OP2, ratio 4, dies in a fraction of a second) on a
  // long ringing bar; the second stack adds the bar's 4th partial.
  { "Vibes", 3, 0.00f, 0.25f, { 0.30f, 0.12f, 0.30f, 0.00f }, {
      {  1.0f,  0, 0.70f, 0.002f, 3.20f, 0.00f, 0.70f, 0.30f,  0.00f },
      {  4.0f,  0, 0.20f, 0.001f, 0.16f, 0.00f, 0.10f, 1.00f, -0.30f },
      {  4.0f,  0, 0.12f, 0.002f, 1.20f, 0.00f, 0.40f, 0.30f, -0.20f },
      { 10.0f,  0, 0.05f, 0.001f, 0.07f, 0.00f, 0.05f, 1.00f, -0.30f } } },
};
static const int NUM_PATCHES = sizeof(PATCHES) / sizeof(PATCHES[0]);

// ---------------------------------------------------------------------------
//  4-operator polyphonic FM engine (DX21-style).
//
//  Algorithms  (A<-B means B modulates A;  [..] = audible carrier)
//    0:  [1]<-2<-3<-4          4:  [1]<-4 + [2]<-4 + [3]<-4
//    1:  [1]<-2<-(3 + 4)       5:  [1]<-2<-3 + [4]
//    2:  [1]<-(2 + 3<-4)       6:  [1]<-2 + [3] + [4]
//    3:  [1]<-2 + [3]<-4       7:  [1] + [2] + [3] + [4]  (additive)
//
//  Threading: every public method except program() must be called from ONE
//  task (the audio task).
// ---------------------------------------------------------------------------
#define SINE_BITS 12
#define SINE_SIZE (1 << SINE_BITS)

class FMSynth {
public:
    void init() {
        for (int i = 0; i <= SINE_SIZE; i++)
            sine_[i] = sinf(2.0f * 3.14159265358979f * (float)i / (float)SINE_SIZE);
        memset(voices_, 0, sizeof(voices_));
        resetControllers();
        programChange(0);
    }

    void noteOn(uint8_t note, uint8_t vel) {
        if (vel == 0) { noteOff(note); return; }
        Voice& v = allocVoice(note);
        const bool fresh = !v.active;       // avoid clicks when re-using a sounding voice
        v.active = true; v.keyDown = true; v.sustained = false;
        v.note = note; v.alg = cur_.alg; v.fb = cur_.feedback;
        v.age = ++ageCounter_;
        if (fresh) { v.y0 = v.y1 = 0.f; }

        const float base = 440.0f * exp2f(((float)note - 69.0f) / 12.0f);
        const float vn   = (float)vel / 127.0f;
        const float oct  = ((float)note - 60.0f) / 12.0f;
        float kd = exp2f(-cur_.keyDecay * oct);
        if (kd < 0.2f) kd = 0.2f; else if (kd > 3.0f) kd = 3.0f;

        for (int i = 0; i < 4; i++) {
            const OpPatch& p = cur_.op[i];
            Op& o = v.op[i];
            float kl = 1.0f + p.keyLvl * oct;
            if (kl < 0.05f) kl = 0.05f; else if (kl > 2.0f) kl = 2.0f;
            o.hz   = base * p.ratio * exp2f(p.detune / 1200.0f);
            o.lvl  = p.level * (1.0f - p.velSens + p.velSens * vn) * kl;
            if (fresh) { o.phase = 0; o.env = 0.f; }
            o.stage = ST_ATTACK;
            o.aInc  = 1.0f / (fmaxf(0.001f, p.a * atkScale_) * (float)SAMPLE_RATE);
            o.sus   = p.s;
            o.dCoef = expf(-6.9078f / (fmaxf(0.005f, p.d * kd) * (float)SAMPLE_RATE));
            o.rTime = p.r * kd;
        }
    }

    void noteOff(uint8_t note) {
        for (int i = 0; i < NUM_VOICES; i++) {
            Voice& v = voices_[i];
            if (v.active && v.keyDown && v.note == note) {
                v.keyDown = false;
                if (sustainPedal_) v.sustained = true;
                else releaseVoice(v);
            }
        }
    }

    void controlChange(uint8_t cc, uint8_t val) {
        switch (cc) {
            case 1:   modWheel_ = val / 127.0f; break;                       // vibrato depth
            case 64: {
                bool down = val >= 64;
                if (sustainPedal_ && !down) {
                    for (int i = 0; i < NUM_VOICES; i++)
                        if (voices_[i].active && voices_[i].sustained && !voices_[i].keyDown) {
                            voices_[i].sustained = false; releaseVoice(voices_[i]);
                        }
                }
                sustainPedal_ = down; break; }
            case 70:  cur_.alg = val >> 4; break;                            // algorithm (new notes)
            case 71:  cur_.feedback = val / 127.0f * 0.5f; break;            // OP4 feedback (new notes)
            case 72:  relScale_ = exp2f(((float)val - 64.0f) / 16.0f); break;// release time scale
            case 73:  atkScale_ = exp2f(((float)val - 64.0f) / 16.0f); break;// attack time scale
            case 74:  bright_ = val / 64.0f; break;                          // brightness (live)
            case 120: allSoundOff(); break;
            case 121: resetControllers(); break;
            case 123: allNotesOff(); break;
            default: break;
        }
    }

    void pitchBend(int16_t value /* -8192..8191 */) { bendSemis_ = (float)value / 8192.0f * 2.0f; }
    void programChange(uint8_t p) { if (p < NUM_PATCHES) { cur_ = PATCHES[p]; program_ = p; } }
    int  program() const { return program_; }

    void allNotesOff() {
        for (int i = 0; i < NUM_VOICES; i++)
            if (voices_[i].active) { voices_[i].keyDown = false; voices_[i].sustained = false; releaseVoice(voices_[i]); }
    }
    void allSoundOff() {
        for (int i = 0; i < NUM_VOICES; i++) {
            voices_[i].active = false;
            for (int k = 0; k < 4; k++) { voices_[i].op[k].stage = ST_IDLE; voices_[i].op[k].env = 0.f; }
        }
    }
    int activeVoices() const {
        int n = 0; for (int i = 0; i < NUM_VOICES; i++) n += voices_[i].active; return n;
    }

    // Renders `n` (<= BLOCK_FRAMES) mono samples, multiplied by `gain`.
    void renderMono(float* mix, int n, float gain) {
        lfoPhase_ += 5.5f * (float)n / (float)SAMPLE_RATE;
        if (lfoPhase_ >= 1.0f) lfoPhase_ -= 1.0f;
        const float vib = sinf(6.2831853f * lfoPhase_) * modWheel_ * 0.6f;
        const float pm  = exp2f((bendSemis_ + vib) / 12.0f);

        for (int i = 0; i < n; i++) mix[i] = 0.f;
        for (int i = 0; i < NUM_VOICES; i++)
            if (voices_[i].active) renderVoice(voices_[i], mix, n, pm);
        for (int i = 0; i < n; i++) mix[i] *= gain;
    }

private:
    enum { ST_IDLE = 0, ST_ATTACK, ST_DECAY, ST_RELEASE };

    struct Op {
        uint32_t phase, inc;
        float env, aInc, sus, dCoef, rCoef, rTime, lvl, hz;
        uint8_t stage;
    };
    struct Voice {
        bool active, keyDown, sustained;
        uint8_t note, alg;
        uint32_t age;
        float fb, y0, y1;
        Op op[4];
    };

    float    sine_[SINE_SIZE + 1];
    Voice    voices_[NUM_VOICES];
    Patch    cur_;
    volatile int program_ = 0;
    uint32_t ageCounter_ = 0;
    bool     sustainPedal_ = false;
    float    modWheel_ = 0, bendSemis_ = 0, lfoPhase_ = 0;
    float    bright_ = 1.0f, atkScale_ = 1.0f, relScale_ = 1.0f;

    void resetControllers() {
        sustainPedal_ = false; modWheel_ = 0; bendSemis_ = 0;
        bright_ = 1.0f; atkScale_ = 1.0f; relScale_ = 1.0f;
    }

    inline float fastSin(uint32_t ph) const {
        const uint32_t idx = ph >> (32 - SINE_BITS);
        const float frac = (float)(ph & ((1u << (32 - SINE_BITS)) - 1)) * (1.0f / (float)(1u << (32 - SINE_BITS)));
        const float a = sine_[idx], b = sine_[idx + 1];
        return a + (b - a) * frac;
    }

    // One operator sample. `mod` = phase modulation in cycles (1.0 = 2*pi).
    inline float runOp(Op& o, float mod) {
        switch (o.stage) {
            case ST_IDLE: return 0.f;
            case ST_ATTACK:
                o.env += o.aInc;
                if (o.env >= 1.f) { o.env = 1.f; o.stage = ST_DECAY; }
                break;
            case ST_DECAY:
                o.env = o.sus + (o.env - o.sus) * o.dCoef;
                if (o.sus < 1e-4f && o.env < 1e-4f) { o.env = 0.f; o.stage = ST_IDLE; return 0.f; }
                break;
            default: // ST_RELEASE
                o.env *= o.rCoef;
                if (o.env < 1e-4f) { o.env = 0.f; o.stage = ST_IDLE; return 0.f; }
                break;
        }
        if (mod > 7.5f) mod = 7.5f; else if (mod < -7.5f) mod = -7.5f;
        const uint32_t pmod = ((uint32_t)(int32_t)(mod * 268435456.0f)) << 4;   // cycles -> 2^32 phase
        const float y = fastSin(o.phase + pmod) * o.env;
        o.phase += o.inc;
        return y;
    }

    void releaseVoice(Voice& v) {
        for (int i = 0; i < 4; i++) {
            Op& o = v.op[i];
            if (o.stage != ST_IDLE) {
                o.rCoef = expf(-6.9078f / (fmaxf(0.003f, o.rTime * relScale_) * (float)SAMPLE_RATE));
                o.stage = ST_RELEASE;
            }
        }
    }

    Voice& allocVoice(uint8_t note) {
        for (int i = 0; i < NUM_VOICES; i++)            // 1. retrigger same note
            if (voices_[i].active && voices_[i].note == note) return voices_[i];
        for (int i = 0; i < NUM_VOICES; i++)            // 2. free voice
            if (!voices_[i].active) return voices_[i];
        int best = -1; uint32_t bestAge = 0xFFFFFFFFu;  // 3. oldest released voice
        for (int i = 0; i < NUM_VOICES; i++)
            if (!voices_[i].keyDown && voices_[i].age < bestAge) { bestAge = voices_[i].age; best = i; }
        if (best >= 0) return voices_[best];
        best = 0; bestAge = voices_[0].age;             // 4. oldest overall
        for (int i = 1; i < NUM_VOICES; i++)
            if (voices_[i].age < bestAge) { bestAge = voices_[i].age; best = i; }
        return voices_[best];
    }

    void renderVoice(Voice& v, float* mix, int n, float pm) {
        const float nyq = 0.45f * (float)SAMPLE_RATE;
        const float incScale = 4294967296.0f / (float)SAMPLE_RATE;
        float amp[4], mA[4];
        for (int i = 0; i < 4; i++) {
            float hz = v.op[i].hz * pm; if (hz > nyq) hz = nyq;
            v.op[i].inc = (uint32_t)(hz * incScale);
            amp[i] = v.op[i].lvl;
            mA[i]  = v.op[i].lvl * MOD_DEPTH * bright_;
        }
        Op* op = v.op;

        for (int s = 0; s < n; s++) {
            const float o4 = runOp(op[3], v.fb * (v.y0 + v.y1) * 0.5f);
            v.y1 = v.y0; v.y0 = o4;
            const float m4 = o4 * mA[3];
            float o1, o2, o3, out;
            switch (v.alg) {
            default:
            case 0: o3 = runOp(op[2], m4);  o2 = runOp(op[1], o3 * mA[2]);
                    o1 = runOp(op[0], o2 * mA[1]);  out = o1 * amp[0]; break;
            case 1: o3 = runOp(op[2], 0.f); o2 = runOp(op[1], m4 + o3 * mA[2]);
                    o1 = runOp(op[0], o2 * mA[1]);  out = o1 * amp[0]; break;
            case 2: o3 = runOp(op[2], m4);  o2 = runOp(op[1], 0.f);
                    o1 = runOp(op[0], o3 * mA[2] + o2 * mA[1]);  out = o1 * amp[0]; break;
            case 3: o3 = runOp(op[2], m4);  o2 = runOp(op[1], 0.f);
                    o1 = runOp(op[0], o2 * mA[1]);  out = o1 * amp[0] + o3 * amp[2]; break;
            case 4: o3 = runOp(op[2], m4);  o2 = runOp(op[1], m4);  o1 = runOp(op[0], m4);
                    out = o1 * amp[0] + o2 * amp[1] + o3 * amp[2]; break;
            case 5: o3 = runOp(op[2], 0.f); o2 = runOp(op[1], o3 * mA[2]);
                    o1 = runOp(op[0], o2 * mA[1]);  out = o1 * amp[0] + o4 * amp[3]; break;
            case 6: o3 = runOp(op[2], 0.f); o2 = runOp(op[1], 0.f);
                    o1 = runOp(op[0], o2 * mA[1]);  out = o1 * amp[0] + o3 * amp[2] + o4 * amp[3]; break;
            case 7: o3 = runOp(op[2], 0.f); o2 = runOp(op[1], 0.f); o1 = runOp(op[0], 0.f);
                    out = o1 * amp[0] + o2 * amp[1] + o3 * amp[2] + o4 * amp[3]; break;
            }
            mix[s] += out;
        }

        // Voice is finished once every carrier envelope is idle.
        static const uint8_t carrierMask[8] = { 0x1, 0x1, 0x1, 0x5, 0x7, 0x9, 0xD, 0xF };
        const uint8_t mask = carrierMask[v.alg & 7];
        bool alive = false;
        for (int i = 0; i < 4; i++)
            if ((mask >> i) & 1) alive |= (op[i].stage != ST_IDLE);
        if (!alive) v.active = false;
    }
};

// ---------------------------------------------------------------------------
//  Effects: phaser -> chorus (mono to stereo) -> ping-pong delay -> reverb.
//  Each effect has one "amount" (0..1); 0 = off.  The internal settings
//  (rates, feedback, delay time ...) are the constants below.
//  The structure follows the effect set of Marcel Licence's ML_SynthTools
//  (reverb / delay / chorus / phaser) but the code here is a fresh implementation.
// ---------------------------------------------------------------------------
static void* fxAlloc(size_t bytes) {
    void* p = NULL;
#ifdef ARDUINO_ARCH_ESP32
    p = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!p) p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    p = malloc(bytes);
#endif
    if (p) memset(p, 0, bytes);
    return p;
}

#define FX_PI          3.14159265358979f
#define PH_RATE_HZ     0.35f      // phaser sweep speed
#define PH_FMIN        250.0f     // phaser sweep range (Hz)
#define PH_FMAX        3200.0f
#define PH_FEEDBACK    0.55f
#define CH_LEN         2048       // chorus buffer (samples, power of 2)
#define CH_RATE_HZ     0.8f
#define CH_BASE        0.014f     // seconds
#define CH_DEPTH       0.0045f    // seconds
#define DLY_SECONDS    0.36f      // delay time
#define DLY_FEEDBACK   0.55f
#define REV_FEEDBACK   0.87f      // reverb decay
#define REV_DAMP       0.30f
#define REV_INPUT      0.12f
#define REV_WET        0.80f

class Effects {
public:
    // returns false if some effect could not get memory (that effect is bypassed)
    bool init() {
        bool ok = true;
        chBuf_ = (float*)fxAlloc(sizeof(float) * CH_LEN);   ok &= (chBuf_ != NULL);

        dlyLen_ = (uint32_t)(DLY_SECONDS * SAMPLE_RATE);
        dlyL_ = (int16_t*)fxAlloc(sizeof(int16_t) * dlyLen_);
        dlyR_ = (int16_t*)fxAlloc(sizeof(int16_t) * dlyLen_);
        if (!dlyL_ || !dlyR_) { ok = false; dlyL_ = dlyR_ = NULL; }

        static const int combLen[4] = { 1116, 1277, 1422, 1557 };
        static const int apLen[2]   = { 556, 341 };
        bool rv = true;
        for (int c = 0; c < 4; c++) {
            combL_[c].init(combLen[c], &rv);
            combR_[c].init(combLen[c] + 23, &rv);
        }
        for (int a = 0; a < 2; a++) {
            apL_[a].init(apLen[a], &rv);
            apR_[a].init(apLen[a] + 23, &rv);
        }
        revOk_ = rv; ok &= rv;
        return ok;
    }

    // mono: synth signal (modified in place by the phaser). L/R: stereo out.
    void process(float* mono, float* L, float* R, int n, const float* amt) {
        phaser(mono, n, amt[FX_PHASER]);
        chorus(mono, L, R, n, amt[FX_CHORUS]);
        delay(L, R, n, amt[FX_DELAY]);
        reverb(L, R, n, amt[FX_REVERB]);
    }

private:
    // ---------------- phaser: 4 first-order all-passes, swept, with feedback --
    float phZ_[4] = {0, 0, 0, 0}, phFb_ = 0, phLfo_ = 0, phA_ = 0;

    void phaser(float* x, int n, float amt) {
        phLfo_ += PH_RATE_HZ * (float)n / (float)SAMPLE_RATE;
        if (phLfo_ >= 1.0f) phLfo_ -= 1.0f;
        const float lfo = 0.5f + 0.5f * sinf(2.0f * FX_PI * phLfo_);
        const float f   = PH_FMIN * powf(PH_FMAX / PH_FMIN, lfo);
        const float t   = tanf(FX_PI * f / (float)SAMPLE_RATE);
        const float a1  = (t - 1.0f) / (t + 1.0f);
        const float a0  = phA_;
        phA_ = a1;
        if (amt < 0.005f) return;
        const float wet = 0.5f * amt, dry = 1.0f - wet;
        for (int i = 0; i < n; i++) {
            const float a = a0 + (a1 - a0) * ((float)i / (float)n);
            float s = x[i] + phFb_ * PH_FEEDBACK;
            for (int k = 0; k < 4; k++) {
                const float y = a * s + phZ_[k];
                phZ_[k] = s - a * y;
                s = y;
            }
            phFb_ = s;
            x[i] = x[i] * dry + s * wet;
        }
    }

    // ---------------- chorus: two LFO-modulated taps -> stereo -----------------
    float*   chBuf_ = NULL;
    uint32_t chW_ = 0;
    float    chPh_ = 0;

    void chorus(const float* in, float* L, float* R, int n, float amt) {
        if (!chBuf_) { for (int i = 0; i < n; i++) L[i] = R[i] = in[i]; return; }
        const float inc  = CH_RATE_HZ / (float)SAMPLE_RATE;
        const float base = CH_BASE * SAMPLE_RATE, depth = CH_DEPTH * SAMPLE_RATE;
        const float wet  = 0.7f * amt;
        for (int i = 0; i < n; i++) {
            chBuf_[chW_] = in[i];
            chPh_ += inc; if (chPh_ >= 1.0f) chPh_ -= 1.0f;
            float pl = chPh_,         pr = chPh_ + 0.25f; if (pr >= 1.0f) pr -= 1.0f;
            const float tl = 4.0f * fabsf(pl - 0.5f) - 1.0f;      // triangle -1..1
            const float tr = 4.0f * fabsf(pr - 0.5f) - 1.0f;
            float yl = tap(base + depth * tl), yr = tap(base - depth * tr);
            L[i] = in[i] + wet * yl;
            R[i] = in[i] + wet * yr;
            chW_ = (chW_ + 1) & (CH_LEN - 1);
        }
    }
    inline float tap(float delaySamples) const {
        const float pos = (float)chW_ - delaySamples + (float)CH_LEN;
        const uint32_t i0 = (uint32_t)pos;
        const float fr = pos - (float)i0;
        const float a = chBuf_[i0 & (CH_LEN - 1)], b = chBuf_[(i0 + 1) & (CH_LEN - 1)];
        return a + (b - a) * fr;
    }

    // ---------------- ping-pong delay (16-bit lines to save RAM) ---------------
    int16_t *dlyL_ = NULL, *dlyR_ = NULL;
    uint32_t dlyLen_ = 0, dlyP_ = 0;

    static inline int16_t clip16(float v) {
        v *= 32767.0f;
        if (v > 32767.0f) v = 32767.0f; else if (v < -32767.0f) v = -32767.0f;
        return (int16_t)v;
    }
    void delay(float* L, float* R, int n, float amt) {
        if (!dlyL_) return;
        const float wet = 0.65f * amt;
        const float k = 1.0f / 32768.0f;
        for (int i = 0; i < n; i++) {
            const float yL = dlyL_[dlyP_] * k, yR = dlyR_[dlyP_] * k;   // from DLY_SECONDS ago
            const float x = 0.5f * (L[i] + R[i]);
            dlyL_[dlyP_] = clip16(x + DLY_FEEDBACK * yR);
            dlyR_[dlyP_] = clip16(yL);
            L[i] += wet * yL;
            R[i] += wet * yR;
            if (++dlyP_ >= dlyLen_) dlyP_ = 0;
        }
    }

    // ---------------- reverb: 4 damped combs + 2 all-passes per channel --------
    struct Comb {
        float* buf = NULL; int len = 0, idx = 0; float filt = 0;
        void init(int l, bool* ok) { len = l; buf = (float*)fxAlloc(sizeof(float) * l); if (!buf) *ok = false; }
        inline float run(float in) {
            const float y = buf[idx];
            filt = y * (1.0f - REV_DAMP) + filt * REV_DAMP;
            buf[idx] = in + filt * REV_FEEDBACK;
            if (++idx >= len) idx = 0;
            return y;
        }
    };
    struct Allpass {
        float* buf = NULL; int len = 0, idx = 0;
        void init(int l, bool* ok) { len = l; buf = (float*)fxAlloc(sizeof(float) * l); if (!buf) *ok = false; }
        inline float run(float in) {
            const float b = buf[idx];
            buf[idx] = in + b * 0.5f;
            if (++idx >= len) idx = 0;
            return b - in;
        }
    };
    Comb combL_[4], combR_[4];
    Allpass apL_[2], apR_[2];
    bool revOk_ = false;

    void reverb(float* L, float* R, int n, float amt) {
        if (!revOk_) return;
        const float wet = REV_WET * amt;
        for (int i = 0; i < n; i++) {
            const float in = (L[i] + R[i]) * 0.5f * REV_INPUT;
            float oL = 0, oR = 0;
            for (int c = 0; c < 4; c++) { oL += combL_[c].run(in); oR += combR_[c].run(in); }
            for (int a = 0; a < 2; a++) { oL = apL_[a].run(oL);   oR = apR_[a].run(oR); }
            L[i] += wet * oL;
            R[i] += wet * oR;
        }
    }
};

static inline float softClip(float x) {
    if (x > 3.f) x = 3.f; else if (x < -3.f) x = -3.f;
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);   // tanh approximation
}

// ===== END DSP ===============================================================

// ---------------------------------------------------------------------------
//  Application
//
//  Core 1: audio task (drain MIDI queue, synth -> effects -> I2S), prio 20
//  Core 0: MIDI parser task (prio 10), encoder poll task (prio 5, 1 kHz),
//          UI task (LCD + encoder actions + serial commands, prio 2)
// ---------------------------------------------------------------------------
// State shared between UI/MIDI/audio. Single bytes/floats: atomic enough here.
static struct Shared {
    volatile uint8_t volume   = 100;   // 0..127  (post-effects master level)
    volatile uint8_t gain     = 64;    // 0..127  (64 = 0 dB, synth level into the effects)
    volatile float   fx[NUM_FX] = { 0, 0, 0, 0 };
    volatile uint8_t selFx    = 0;
    volatile bool    gainMode = false; // false: encoder 3 edits volume, true: gain
} sh;

static QueueHandle_t midiQueue;
static FMSynth synth;
static Effects fxEngine;
static Adafruit_ILI9341 tft(&SPI, LCD_DC, LCD_CS, LCD_RESET);

static void loadPatchFx(int p) {
    for (int i = 0; i < NUM_FX; i++) sh.fx[i] = PATCHES[p].fx[i];
}

// ----- MIDI dispatch (runs in audio task) -----------------------------------
static void dispatch(const MidiMsg& m) {
    const uint8_t type = m.status & 0xF0;
    const uint8_t ch   = (m.status & 0x0F) + 1;
#if MIDI_DEBUG
    Serial.printf("MIDI ch%d status=%02X d1=%d d2=%d\n", ch, m.status, m.d1, m.d2);
#endif
    if (MIDI_CHANNEL != 0 && ch != MIDI_CHANNEL) return;

    switch (type) {
        case 0x80: synth.noteOff(m.d1); break;
        case 0x90: synth.noteOn(m.d1, m.d2); break;        // vel 0 handled inside
        case 0xB0:
            switch (m.d1) {
                case 7:  sh.volume = m.d2; break;
                case 91: sh.fx[FX_REVERB] = m.d2 / 127.0f; break;
                case 94: sh.fx[FX_DELAY]  = m.d2 / 127.0f; break;
                case 93: sh.fx[FX_CHORUS] = m.d2 / 127.0f; break;
                case 95: sh.fx[FX_PHASER] = m.d2 / 127.0f; break;
                default: synth.controlChange(m.d1, m.d2); break;
            }
            break;
        case 0xC0:
            if (m.d1 < NUM_PATCHES) { synth.programChange(m.d1); loadPatchFx(m.d1); }
            break;
        case 0xE0: synth.pitchBend((int16_t)(((m.d2 << 7) | m.d1) - 8192)); break;
        default: break;
    }
}

// ----- Audio task -----------------------------------------------------------
static void audioTask(void*) {
    static int16_t buf[BLOCK_FRAMES * 2];
    static float mono[BLOCK_FRAMES], L[BLOCK_FRAMES], R[BLOCK_FRAMES];
    for (;;) {
        MidiMsg m;
        while (xQueueReceive(midiQueue, &m, 0) == pdTRUE) dispatch(m);

        float amt[NUM_FX];
        for (int i = 0; i < NUM_FX; i++) amt[i] = sh.fx[i];
        const float gainF = MASTER_GAIN * exp2f(((float)sh.gain - 64.0f) / 32.0f);   // 0.25x..~4x
        const float v = (float)sh.volume / 127.0f;
        const float vol = v * v;

        synth.renderMono(mono, BLOCK_FRAMES, gainF);
        fxEngine.process(mono, L, R, BLOCK_FRAMES, amt);

        for (int i = 0; i < BLOCK_FRAMES; i++) {
            buf[2 * i]     = (int16_t)(softClip(L[i]) * vol * 32767.0f);
            buf[2 * i + 1] = (int16_t)(softClip(R[i]) * vol * 32767.0f);
        }
        size_t written;
        i2s_write(I2S_NUM_0, buf, sizeof(buf), &written, portMAX_DELAY);  // paces the loop
    }
}

// ----- MIDI input task ------------------------------------------------------
static void midiTask(void*) {
    uint8_t rx[64];
    uint8_t status = 0, data[2] = {0, 0}, idx = 0, need = 0;
    for (;;) {
        int n = uart_read_bytes(MIDI_UART, rx, sizeof(rx), pdMS_TO_TICKS(5));
        for (int i = 0; i < n; i++) {
            const uint8_t b = rx[i];
            if (b >= 0xF8) continue;                         // realtime: ignore
            if (b & 0x80) {
                if (b < 0xF0) {
                    status = b; idx = 0;
                    const uint8_t hi = b & 0xF0;
                    need = (hi == 0xC0 || hi == 0xD0) ? 1 : 2;
                } else { status = 0; idx = 0; }              // sysex / system common
            } else if (status) {
                data[idx++] = b;
                if (idx >= need) {
                    MidiMsg m = { status, data[0], (uint8_t)(need == 2 ? data[1] : 0) };
                    xQueueSend(midiQueue, &m, 0);
                    idx = 0;                                 // running status
                }
            }
        }
    }
}

// ----- Encoders (polled at 1 kHz) -------------------------------------------
struct Encoder {
    uint8_t pinA, pinB;
    bool accel;              // turn faster -> bigger steps
    uint8_t state;
    int8_t acc;
    uint32_t lastDetent;
    volatile int delta;      // consumed by the UI task
};
static Encoder enc[3] = {
    { ENC1_A, ENC1_B, false, 0, 0, 0, 0 },
    { ENC2_A, ENC2_B, true,  0, 0, 0, 0 },
    { ENC3_A, ENC3_B, true,  0, 0, 0, 0 },
};
static volatile int btnShort = 0, btnLong = 0;

static void encoderTask(void*) {
    // quadrature transition table, index = (old<<2)|new, state = (A<<1)|B
    static const int8_t T[16] = { 0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0 };
    bool btnDown = false, longSent = false; uint32_t btnT = 0; uint8_t btnDeb = 0;

    for (int i = 0; i < 3; i++) {
        pinMode(enc[i].pinA, INPUT_PULLUP); pinMode(enc[i].pinB, INPUT_PULLUP);
        enc[i].state = (digitalRead(enc[i].pinA) << 1) | digitalRead(enc[i].pinB);
    }
    pinMode(BTN_PIN, INPUT_PULLUP);

    for (;;) {
        const uint32_t now = millis();
        for (int i = 0; i < 3; i++) {
            Encoder& e = enc[i];
            const uint8_t s = (digitalRead(e.pinA) << 1) | digitalRead(e.pinB);
            if (s != e.state) {
                e.acc += T[(e.state << 2) | s];
                e.state = s;
                int dir = 0;
                if (e.acc >= ENC_STEPS_PER_DETENT)       { dir = +1; e.acc = 0; }
                else if (e.acc <= -ENC_STEPS_PER_DETENT) { dir = -1; e.acc = 0; }
                if (dir) {
                    if (ENC_REVERSE) dir = -dir;
                    int mult = 1;
                    if (e.accel) { const uint32_t dt = now - e.lastDetent; mult = (dt < 25) ? 4 : (dt < 60) ? 2 : 1; }
                    e.lastDetent = now;
                    __atomic_fetch_add(&e.delta, dir * mult, __ATOMIC_RELAXED);
                }
            }
        }
        // push button: short press / long press, 10 ms debounce
        const bool raw = (digitalRead(BTN_PIN) == LOW);
        if (raw != btnDown) {
            if (++btnDeb >= 10) {
                btnDeb = 0; btnDown = raw;
                if (btnDown) { btnT = now; longSent = false; }
                else if (!longSent) btnShort++;
            }
        } else btnDeb = 0;
        if (btnDown && !longSent && (now - btnT) > LONG_PRESS_MS) { longSent = true; btnLong++; }

        vTaskDelay(1);
    }
}

static inline int takeDelta(int i) { return __atomic_exchange_n(&enc[i].delta, 0, __ATOMIC_RELAXED); }

// ----- LCD ------------------------------------------------------------------
//  y=0..23    header
//  y=26..190  left: SOUND list  |  right: EFFECTS (4 bars)
//  y=196..239 VOLUME and GAIN bars
static const char* FX_NAMES[NUM_FX] = { "Reverb", "Delay", "Chorus", "Phaser" };
#define ROW_Y0   44
#define ROW_H    36
#define COL_ROW  0x2104        // dark grey
#define COL_BAR  0x4208        // bar track

static void drawHeader(int voices) {
    tft.fillRect(0, 0, 320, 24, ILI9341_NAVY);
    tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(8, 4);  tft.print("FM SYNTH");
    char t[12]; snprintf(t, sizeof(t), "V:%02d", voices);
    tft.setCursor(256, 4); tft.print(t);
}

static void drawSoundRow(int i, bool sel) {
    const int y = ROW_Y0 + i * ROW_H;
    tft.fillRect(4, y, 152, 32, sel ? ILI9341_CYAN : COL_ROW);
    tft.setTextSize(2); tft.setTextColor(sel ? ILI9341_BLACK : ILI9341_WHITE);
    tft.setCursor(12, y + 8); tft.print(PATCHES[i].name);
}

static void drawFxRow(int i, bool sel, float amt) {
    const int y = ROW_Y0 + i * ROW_H;
    const int pct = (int)(amt * 100.0f + 0.5f);
    tft.fillRect(164, y, 152, 32, sel ? ILI9341_ORANGE : COL_ROW);
    tft.setTextSize(2); tft.setTextColor(sel ? ILI9341_BLACK : ILI9341_WHITE);
    tft.setCursor(170, y + 3); tft.print(FX_NAMES[i]);
    char t[8]; snprintf(t, sizeof(t), "%3d", pct);
    tft.setCursor(276, y + 3); tft.print(t);
    tft.fillRect(170, y + 22, 140, 7, sel ? ILI9341_BLACK : COL_BAR);
    const int w = (int)(140.0f * amt);
    if (w > 0) tft.fillRect(170, y + 22, w, 7, sel ? ILI9341_WHITE : ILI9341_GREEN);
}

static void drawBottom(int vol, int gain, bool gainMode) {
    tft.fillRect(0, 194, 320, 46, ILI9341_BLACK);
    tft.setTextSize(2);
    // volume
    tft.setTextColor(gainMode ? ILI9341_LIGHTGREY : ILI9341_YELLOW);
    tft.setCursor(8, 204); tft.print("VOL");
    tft.fillRect(56, 202, 94, 16, COL_BAR);
    tft.fillRect(56, 202, (int)(94.0f * vol / 127.0f), 16, gainMode ? ILI9341_GREEN : ILI9341_YELLOW);
    char t[12]; snprintf(t, sizeof(t), "%d%%", (int)(100.0f * vol / 127.0f + 0.5f));
    tft.setTextSize(1); tft.setTextColor(ILI9341_LIGHTGREY);
    tft.setCursor(56, 224); tft.print(t);
    // gain (64 = 0 dB, 32 steps per 6 dB)
    tft.setTextSize(2); tft.setTextColor(gainMode ? ILI9341_YELLOW : ILI9341_LIGHTGREY);
    tft.setCursor(168, 204); tft.print("GAIN");
    tft.fillRect(222, 202, 90, 16, COL_BAR);
    tft.fillRect(222, 202, (int)(90.0f * gain / 127.0f), 16, gainMode ? ILI9341_YELLOW : ILI9341_GREEN);
    snprintf(t, sizeof(t), "%+.1f dB", (gain - 64) * 6.02f / 32.0f);
    tft.setTextSize(1); tft.setTextColor(ILI9341_LIGHTGREY);
    tft.setCursor(222, 224); tft.print(t);
}

static void drawAll(int prog, int sel, const float* fx, int vol, int gain, bool gm, int voices) {
    tft.fillScreen(ILI9341_BLACK);
    drawHeader(voices);
    tft.setTextSize(1); tft.setTextColor(ILI9341_LIGHTGREY);
    tft.setCursor(8, 30);   tft.print("SOUND");
    tft.setCursor(168, 30); tft.print("EFFECTS");
    tft.drawFastVLine(159, 26, 164, COL_BAR);
    tft.drawFastHLine(0, 192, 320, COL_BAR);
    for (int i = 0; i < NUM_PATCHES; i++) drawSoundRow(i, i == prog);
    for (int i = 0; i < NUM_FX; i++)      drawFxRow(i, i == sel, fx[i]);
    drawBottom(vol, gain, gm);
}

static void initLcd() {
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, LCD_BL_ON);
    SPI.begin(LCD_SCLK, -1, LCD_MOSI, LCD_CS);   // no MISO needed for drawing
    tft.begin();
    tft.setRotation(1);                          // landscape, 320x240
}

// ----- UI task: encoder actions + redraw only what changed -------------------
static void uiTask(void*) {
    int   shProg = synth.program(), shSel = sh.selFx, shVol = sh.volume, shGain = sh.gain, shVoices = -1;
    bool  shGm = sh.gainMode;
    float shFx[NUM_FX]; for (int i = 0; i < NUM_FX; i++) shFx[i] = sh.fx[i];
    drawAll(shProg, shSel, shFx, shVol, shGain, shGm, 0);
    shVoices = 0;

    for (;;) {
        // --- encoder 1: sound ---
        int d = takeDelta(0);
        if (d) {
            int p = ((synth.program() + (d > 0 ? 1 : -1)) % NUM_PATCHES + NUM_PATCHES) % NUM_PATCHES;
            MidiMsg m = { 0xC0, (uint8_t)p, 0 };           // handled by the audio task
            xQueueSend(midiQueue, &m, 0);
        }
        // --- encoder 2: amount of the selected effect ---
        d = takeDelta(1);
        if (d) {
            float v = sh.fx[sh.selFx] + d * 0.02f;
            sh.fx[sh.selFx] = v < 0.f ? 0.f : (v > 1.f ? 1.f : v);
        }
        // --- encoder 3: volume or gain ---
        d = takeDelta(2);
        if (d) {
            int v = (sh.gainMode ? sh.gain : sh.volume) + d * 2;
            v = v < 0 ? 0 : (v > 127 ? 127 : v);
            if (sh.gainMode) sh.gain = v; else sh.volume = v;
        }
        // --- buttons ---
        while (btnShort > 0) { btnShort--; sh.selFx = (sh.selFx + 1) % NUM_FX; }
        while (btnLong  > 0) { btnLong--;  sh.gainMode = !sh.gainMode; }

        // --- serial shortcuts ---
        while (Serial.available()) {
            const char c = Serial.read();
            if (c == 'n' || c == 'p') {
                int p = ((synth.program() + (c == 'n' ? 1 : -1)) % NUM_PATCHES + NUM_PATCHES) % NUM_PATCHES;
                MidiMsg m = { 0xC0, (uint8_t)p, 0 }; xQueueSend(midiQueue, &m, 0);
            } else if (c == 'f') sh.selFx = (sh.selFx + 1) % NUM_FX;
        }

        // --- redraw what changed ---
        const int prog = synth.program();
        if (prog != shProg) { drawSoundRow(shProg, false); drawSoundRow(prog, true); shProg = prog; }

        const int sel = sh.selFx;
        if (sel != shSel) {
            const int old = shSel; shSel = sel;
            drawFxRow(old, false, sh.fx[old]); shFx[old] = sh.fx[old];
            drawFxRow(sel, true,  sh.fx[sel]); shFx[sel] = sh.fx[sel];
        }
        for (int i = 0; i < NUM_FX; i++) {
            const float a = sh.fx[i];
            if ((int)(a * 100.0f + 0.5f) != (int)(shFx[i] * 100.0f + 0.5f)) { shFx[i] = a; drawFxRow(i, i == shSel, a); }
        }
        const int vol = sh.volume, gain = sh.gain; const bool gm = sh.gainMode;
        if (vol != shVol || gain != shGain || gm != shGm) { shVol = vol; shGain = gain; shGm = gm; drawBottom(vol, gain, gm); }

        const int voices = synth.activeVoices();
        if (voices != shVoices) { shVoices = voices; drawHeader(voices); }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

// ----- Hardware init --------------------------------------------------------
static void initMidiUart() {
    uart_config_t cfg = {};
    cfg.baud_rate  = 31250;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;
    uart_param_config(MIDI_UART, &cfg);
    // RX only. TX untouched (default UART2 TX is GPIO17 = PSRAM on a WROVER!)
    uart_set_pin(MIDI_UART, UART_PIN_NO_CHANGE, PIN_MIDI_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MIDI_UART, 1024, 0, 0, NULL, 0);
}

static void initI2S() {
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 4;
    cfg.dma_buf_len          = BLOCK_FRAMES;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;          // silence on underrun instead of repeating
    i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = I2S_PIN_NO_CHANGE;
    pins.bck_io_num   = PIN_I2S_BCK;
    pins.ws_io_num    = PIN_I2S_WS;
    pins.data_out_num = PIN_I2S_DOUT;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;
    i2s_set_pin(I2S_NUM_0, &pins);
    i2s_zero_dma_buffer(I2S_NUM_0);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nESP32 FM synth + FX starting");
    Serial.println("Serial: 'n'/'p' = next/prev sound, 'f' = next effect");

    synth.init();
    loadPatchFx(0);
    const bool fxOk = fxEngine.init();
    Serial.printf("Effects memory %s, free heap %u bytes\n", fxOk ? "OK" : "INCOMPLETE (some effects bypassed)",
                  (unsigned)ESP.getFreeHeap());

    midiQueue = xQueueCreate(256, sizeof(MidiMsg));
    initLcd();
    initI2S();
    initMidiUart();

    xTaskCreatePinnedToCore(audioTask,   "audio",   8192, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(midiTask,    "midi",    4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(encoderTask, "encoder", 3072, NULL,  5, NULL, 0);
    xTaskCreatePinnedToCore(uiTask,      "ui",      6144, NULL,  2, NULL, 0);
}

void loop() {
#if BOOT_TEST_TONE
    static const uint8_t chord[3] = { 60, 64, 67 };
    for (int i = 0; i < 3; i++) { MidiMsg m = { 0x90, chord[i], 100 }; xQueueSend(midiQueue, &m, 0); }
    vTaskDelay(pdMS_TO_TICKS(1200));
    for (int i = 0; i < 3; i++) { MidiMsg m = { 0x80, chord[i], 0 };   xQueueSend(midiQueue, &m, 0); }
#endif
    vTaskDelete(NULL);   // all the work happens in the tasks
}
