/*
AUTONOMOUS DRONE SYSTEM: PROJECT REPORT.c
* Author : Moeti KG
* Uno -> Betaflight MSP RC override “auto-brake” (C / AVR-GCC / Atmel
Studio)
* MCU: ATmega328P @ 16 MHz (Arduino Uno)
*
* HC-SR04:
* TRIG = PB1 (Arduino D9)
* ECHO = PB0 (Arduino D8)
*
* UART to Flight Controller:
* Baud 115200, 8N1, U2X enabled (double speed)
* IMPORTANT: Level-shift Uno TX (5V) down to 3.3V for FC RX!
*
* Betaflight:
* - Enable MSP on the wired UART @ 115200
* - CLI: set msp_override_channels_mask = 15 ; save (to allow R,P,Y,T)
* - Add MSP OVERRIDE mode in Modes tab (assign a switch) */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
/* ----------- Pins ----------- */
#define TRIG_PIN PB1 // D9
#define ECHO_PIN PB0 // D8
/* ----------- RC ranges ----------- */
#define RC_MIN 1000
#define RC_MID 1500
#define RC_MAX 2000
/* ----------- Behavior tuning ----------- */
#define NUM_CHANNELS 8
/* Start braking below this distance (cm). 2 m = 200 cm */ static
const float TRIGGER_CM = 10.0f;
/* Full brake at/below this distance (cm). Keep < TRIGGER_CM */ static
const float HARD_BRAKE_CM = 30.0f;
/* Pitch-back limits (1500 = neutral, >1500 = pitch back) */
#define PITCH_BACK_MAX 1800
#define PITCH_BACK_MIN 1550
/* Simple low-pass for distance */ static
const float ALPHA = 0.35f;

/* MSP v1 command */
#define MSP_SET_RAW_RC 200
/* ---- small helpers (no stdlib needed) ---- */ static
inline uint16_t clamp_u16(int v, int lo, int hi) {
	if (v < lo) return (uint16_t)lo;
	if (v > hi) return (uint16_t)hi;
	return (uint16_t)v;
	} static inline float fmaxf_local(float a, float b){ return (a > b) ? a : b;
	} static inline float fminf_local(float a, float b){ return (a < b) ? a : b;
}
/* ----------- UART (USART0) @ 115200, 8N1, double speed ----------- */
static void uart_init_115200(void) {
	// Double speed mode
	UCSR0A = _BV(U2X0);
	// Baud: UBRR = F_CPU/(8*BAUD) - 1 => ~16 for 115200 with U2X=1
	UBRR0H = 0;
	UBRR0L = 16;
	// Frame: 8N1
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
	// Enable TX (RX optional if you want to read MSP responses)
	UCSR0B = _BV(TXEN0); // | _BV(RXEN0);
	} static void uart_write_byte(uint8_t b) {
	while (!(UCSR0A & _BV(UDRE0))) { /* wait */ }
	UDR0 = b;
}
/* ----------- MSP: send RC override (8 channels, uint16 LE) ----------- */
static void msp_send_rc(const uint16_t ch[NUM_CHANNELS]) { uint8_t csum
	= 0;
	const uint8_t payload_size = NUM_CHANNELS * 2;
	// Header "$M<"
	uart_write_byte('$');
	uart_write_byte('M');
	uart_write_byte('<');
	// Size and Cmd
	uart_write_byte(payload_size); csum ^= payload_size;
	uart_write_byte(MSP_SET_RAW_RC); csum ^= MSP_SET_RAW_RC;
	// Payload: 8 * uint16_t little-endian 
	for
	(uint8_t i = 0; i < NUM_CHANNELS; i++) {
		uint8_t lo = (uint8_t)(ch[i] & 0xFF);
		uint8_t hi = (uint8_t)((ch[i] >> 8) & 0xFF);
		uart_write_byte(lo); csum ^= lo;
		uart_write_byte(hi); csum ^= hi;
	}
	// Checksum
	uart_write_byte(csum);
}

/* ----------- HC-SR04 distance (cm) using Timer1 ----------- */
/*
* Timer1 prescaler = 8 -> 1 tick = 0.5 us @ 16 MHz
* We:
* - issue 10 us TRIG
* - wait for ECHO rising (with timeout)
* - reset TCNT1, start timer, wait for ECHO falling (with timeout)
* - convert ticks -> us -> cm (us / 58)
*/ static float
measure_distance_cm(void) {
	// Ensure TRIG low for 2 us
	PORTB &= ~_BV(TRIG_PIN);
	_delay_us(2);
	// 10 us pulse
	PORTB |= _BV(TRIG_PIN);
	_delay_us(10);
	PORTB &= ~_BV(TRIG_PIN);
	// Wait for ECHO rising (timeout loop)
	uint32_t guard = 0; while (!(PINB &
	_BV(ECHO_PIN))) {
		if (++guard > 60000UL) return 9999.0f; // ~timeout
	}
	// Setup Timer1: prescaler 8 -> 0.5us per tick
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	TCCR1B = _BV(CS11); // start (prescaler 8)
	// Wait for ECHO falling, stop on timeout too
	guard = 0;
	while (PINB & _BV(ECHO_PIN)) {
		if (++guard > 65000UL) { TCCR1B = 0; return 9999.0f; }
	}
	// Stop timer
	TCCR1B = 0;
	uint16_t ticks = TCNT1; // 0.5 us per tick
	float us = (float)ticks * 0.5f;
	float cm = us / 58.0f; // HC-SR04: ~58 us per cm (round-trip)
	return cm;
}
/* ----------- GPIO init ----------- */ static
void io_init(void) {
	// TRIG (PB1) as output, ECHO (PB0) as input
	DDRB |= _BV(TRIG_PIN);
	DDRB &= ~_BV(ECHO_PIN);
	// Ensure TRIG low
	PORTB &= ~_BV(TRIG_PIN);

}
/* ========================= main ========================= */
int main(void) { io_init(); uart_init_115200();
	float dFilt = 400.0f; // filtered distance (cm), start large
	while (1) {
		/* --- sense distance --- */
		float d = measure_distance_cm();
		if (d > 0.0f && d < 600.0f) {
			dFilt = ALPHA * d + (1.0f - ALPHA) * dFilt;
		}
		/* --- compute pitch override from distance --- */
		uint16_t ch[NUM_CHANNELS];
		// Base channels (R,P,Y,T,AUX1..AUX4). Assign individually (no C99 VLA
		 ch[0] = RC_MID; // Roll
		ch[1] = RC_MID; // Pitch (will modify below)
		ch[2] = RC_MID; // Yaw
		ch[3] = 1050; // Throttle baseline (adjust as desired)
		ch[4] = 1500; // AUX1 ch[5] = 1500; // AUX2
		ch[6] = 1500; // AUX3 ch[7] = 1500; // AUX4
		uint16_t pitchCmd = RC_MID; // 1500 = neutral, >1500 pitches back
		if (dFilt < TRIGGER_CM) {
			float span = fmaxf_local(1.0f, (TRIGGER_CM - HARD_BRAKE_CM));
			float x = (TRIGGER_CM - dFilt) / span; // 0..1 if (x <
	        x = 0.0f; if (x > 1.0f) x = 1.0f;
			float val = (float)PITCH_BACK_MIN + x * (float)(PITCH_BACK_MAX -
			PITCH_BACK_MIN);
			pitchCmd = clamp_u16((int)val, RC_MIN, RC_MAX);
		}
		ch[1] = pitchCmd;
		/* --- send MSP RC override at ~50 Hz --- */
		msp_send_rc(ch);
		_delay_ms(20); // ~50 Hz loop
	}
}