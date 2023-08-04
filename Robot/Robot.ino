#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define UNO
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define MEGA
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#define LEONARDO
#endif

#define SELECT_SKETCH 2
// 1: Controller
// 2: Debugger

#if !defined(MEGA)

#ifdef SELECT_SKETCH
#undef SELECT_SKETCH
#endif

void setup() {}
void loop() {}

#endif

