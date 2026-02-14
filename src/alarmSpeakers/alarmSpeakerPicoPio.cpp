#include "alarmSpeakerPicoPio.h"

// pio program that will drive two pins to create a square wave of a given frequency, with 50% duty cycle, and the ability to stop the sound by setting the pins low and skipping the delay loop. The program reads the delay period from the OSR, so the main code can change the frequency on the fly by writing to the OSR.

/* https://wokwi.com/tools/pioasm
    set    x, 2                       ; 0b10, when inverted changes to 0b01, so the two pins will be driven in opposite states
    mov    isr, x
.wrap_target
    mov    pins, isr
    mov    isr, !isr
read:
    pull   noblock
    mov    x, osr
    set y,0
    jmp  x!=y, play
    mov pins, x         ; x is 0b00, set both pins low
    jmp read            ; loop, skip delay and setting pins
play:
    mov    y, x
delayLoop:
    jmp    y--, delayLoop
.wrap
*/
#define alarm_speaker_pio_wrap_target 2
#define alarm_speaker_pio_wrap 11
static const uint16_t alarm_speaker_pio_program_instructions[] = {
    0xe022, //  0: set    x, 2
    0xa0c1, //  1: mov    isr, x
            //     .wrap_target
    0xa006, //  2: mov    pins, isr
    0xa0ce, //  3: mov    isr, !isr
    0x8080, //  4: pull   noblock
    0xa027, //  5: mov    x, osr
    0xe040, //  6: set    y, 0
    0x00aa, //  7: jmp    x != y, 10
    0xa001, //  8: mov    pins, x
    0x0004, //  9: jmp    4
    0xa041, // 10: mov    y, x
    0x008b, // 11: jmp    y--, 11
            //     .wrap
};

static const struct pio_program alarm_speaker_pio_program = {
    .instructions = alarm_speaker_pio_program_instructions,
    .length = alarm_speaker_pio_wrap + 1,
    .origin = -1,
};

static inline void alarm_speaker_pio_program_init(PIO pio, uint sm, uint offset, uint pin)
{
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin + 1);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 2, true); // two pins
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + alarm_speaker_pio_wrap_target, offset + alarm_speaker_pio_wrap);
    sm_config_set_out_pins(&c, pin, 2); // two pins
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_put(pio, sm, 0);
    pio_sm_set_enabled(pio, sm, true);
}

void AlarmSpeakerPicoPio::begin()
{
    uint offset;
    pio_claim_free_sm_and_add_program(&alarm_speaker_pio_program, &pio, &sm, &offset);
    alarm_speaker_pio_program_init(pio, sm, offset, _pin1); // 2 consecutive pins
}

void AlarmSpeakerPicoPio::playFrequency(uint32_t freq)
{
    uint32_t period = 0;
    if (freq != 0) {
        period = F_CPU / freq / 2;
    } // else if freq is 0, period stays 0 which will stop the sound
    if (period != lastPeriod) {
        lastPeriod = period;
        pio_sm_put(pio, sm, period);
    }
}
