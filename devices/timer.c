#include "devices/timer.h"

#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>

#include "threads/interrupt.h"
#include "threads/io.h"
#include "threads/synch.h"
#include "threads/thread.h"

/* See [8254] for hardware details of the 8254 timer chip. */

#if TIMER_FREQ < 19
#error 8254 timer requires TIMER_FREQ >= 19
#endif
#if TIMER_FREQ > 1000
#error TIMER_FREQ <= 1000 recommended
#endif

/* Number of timer ticks since OS booted. */
static int64_t ticks;
static int64_t wake_up_ticks;

/* Number of loops per timer tick.
     Initialized by timer_calibrate(). */
static unsigned loops_per_tick;

static intr_handler_func timer_interrupt;
static bool too_many_loops(unsigned loops);
static void busy_wait(int64_t loops);
static void real_time_sleep(int64_t num, int32_t denom);

/* Sets up the 8254 Programmable Interval Timer (PIT) to
     interrupt PIT_FREQ times per second, and registers the
     corresponding interrupt. */
void timer_init(void) { // PIT는 주기적으로 카운트를 내리다가 카운트가 0이 되면 인터럽트를 발생시키고, 그 후 다시 초기값으로 되돌아가는 식으로 동작합
    /* 8254 input frequency divided by TIMER_FREQ, rounded to
         nearest. */
    uint16_t count = (1193180 + TIMER_FREQ / 2) / TIMER_FREQ;

    outb(0x43, 0x34); /* CW: counter 0, LSB then MSB, mode 2, binary. */
    outb(0x40, count & 0xff);
    outb(0x40, count >> 8);

    intr_register_ext(0x20, timer_interrupt, "8254 Timer"); // 정해진 빈도로 인터럽트를 발생
}

/* Calibrates loops_per_tick, used to implement brief delays. */
// 얼마나 많은 루프가 하나의 타이머 틱에 해당하는지를 계산
void timer_calibrate(void) {
    unsigned high_bit, test_bit;

    ASSERT(intr_get_level() == INTR_ON);
    printf("Calibrating timer...  ");

    /* Approximate loops_per_tick as the largest power-of-two
         still less than one timer tick. */
    loops_per_tick = 1u << 10;
    while (!too_many_loops(loops_per_tick << 1)) {
        loops_per_tick <<= 1;
        ASSERT(loops_per_tick != 0);
    }

    /* Refine the next 8 bits of loops_per_tick. */
    high_bit = loops_per_tick;
    for (test_bit = high_bit >> 1; test_bit != high_bit >> 10; test_bit >>= 1)
        if (!too_many_loops(high_bit | test_bit))
            loops_per_tick |= test_bit;

    printf("%'" PRIu64 " loops/s.\n", (uint64_t)loops_per_tick * TIMER_FREQ);
}

/* Returns the number of timer ticks since the OS booted. */
// 운영 체제가 부팅된 이후 경과한 타이머 틱의 수를 반환
int64_t timer_ticks(void) {
    enum intr_level old_level = intr_disable();
    int64_t t = ticks;
    intr_set_level(old_level);
    barrier();
    return t;
}

/* Returns the number of timer ticks elapsed since THEN, which
     should be a value once returned by timer_ticks(). */
int64_t timer_elapsed(int64_t then) {
    return timer_ticks() - then;
}

/* Suspends execution for approximately TICKS timer ticks. */
// 주어진 시간(ticks) 동안 스레드를 대기하게 만드는 함수
void timer_sleep(int64_t ticks) {
    int64_t start = timer_ticks();  // 현재 시간을 계산

    ASSERT(intr_get_level() == INTR_ON);
    // ticks 만큼의 시간이 경과할 때까지 계속해서 thread_yield() 함수를 호출하여 CPU를 양보
    while (timer_elapsed(start) < ticks)  // 경과한 시간을 계산
        thread_yield();                  

    // 현재 틱수에 잠을 자는 시간을 틱단위로 지정 (현재 틱 수 + 일어나야할 틱 수)
    // sleep_list에 일어날 틱 수를 저장해놓음
}

/* Suspends execution for approximately MS milliseconds. */
void timer_msleep(int64_t ms) {
    real_time_sleep(ms, 1000);
}

/* Suspends execution for approximately US microseconds. */
void timer_usleep(int64_t us) {
    real_time_sleep(us, 1000 * 1000);
}

/* Suspends execution for approximately NS nanoseconds. */
void timer_nsleep(int64_t ns) {
    real_time_sleep(ns, 1000 * 1000 * 1000);
}

/* Prints timer statistics. */
void timer_print_stats(void) {
    printf("Timer: %" PRId64 " ticks\n", timer_ticks());
}

/* Timer interrupt handler. */
static void timer_interrupt(struct intr_frame *args UNUSED) {
    ticks++;
    thread_tick();
}

/* Returns true if LOOPS iterations waits for more than one timer
     tick, otherwise false. */
static bool too_many_loops(unsigned loops) {
    /* Wait for a timer tick. */
    int64_t start = ticks;
    while (ticks == start)
        barrier();

    /* Run LOOPS loops. */
    start = ticks;
    busy_wait(loops);

    /* If the tick count changed, we iterated too long. */
    barrier();
    return start != ticks;
}

/* Iterates through a simple loop LOOPS times, for implementing
     brief delays.

     Marked NO_INLINE because code alignment can significantly
     affect timings, so that if this function was inlined
     differently in different places the results would be difficult
     to predict. */
static void NO_INLINE busy_wait(int64_t loops) {
    while (loops-- > 0)
        barrier();
}

/* Sleep for approximately NUM/DENOM seconds. */
static void real_time_sleep(int64_t num, int32_t denom) {
    /* Convert NUM/DENOM seconds into timer ticks, rounding down.

         (NUM / DENOM) s
         ---------------------- = NUM * TIMER_FREQ / DENOM ticks.
         1 s / TIMER_FREQ ticks
         */
    int64_t ticks = num * TIMER_FREQ / denom;

    ASSERT(intr_get_level() == INTR_ON);
    if (ticks > 0) {
        /* We're waiting for at least one full timer tick.  Use
             timer_sleep() because it will yield the CPU to other
             processes. */
        timer_sleep(ticks);
    } else {
        /* Otherwise, use a busy-wait loop for more accurate
             sub-tick timing.  We scale the numerator and denominator
             down by 1000 to avoid the possibility of overflow. */
        ASSERT(denom % 1000 == 0);
        busy_wait(loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000));
    }
}
