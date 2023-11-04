#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "gpio.h"

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define EMPIRICAL_NANOSLEEP_OVERHEAD_US 12
#define MINIMUM_NANOSLEEP_TIME_US 5

// Pin modes
#define INPUT 0
#define OUTPUT 1
#define REGISTER_BLOCK_SIZE (4 * 1024)

/*********** Rockchip RK3566 *************/

#define RK3566_GPIO4_BASE 0xfe770000U

// gpio offset
#define RK3566_GPIO_SWPORT_DR_L_OFFSET 0x00U
#define RK3566_GPIO_SWPORT_DR_H_OFFSET 0x04U
#define RK3566_GPIO_SWPORT_DDR_L_OFFSET 0x08U
#define RK3566_GPIO_EXT_PORT_OFFSET 0x70U

/*********** Rockchip RK3566 *************/

static volatile uint32_t *s_GPIO_registers = NULL;
static volatile uint32_t *s_Timer1Mhz = NULL;

namespace rgb_matrix {
#define GPIO_BIT(x) (1ull << x)

static void gpio_write_mode(int index, int value) {
  uint32_t *reg_addr =
      (uint32_t *)((unsigned char *)s_GPIO_registers +
                   RK3566_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2));
  int index_offset = index % 16;
  uint32_t reg_val = *reg_addr;

  reg_val |= 0x1 << (16 + index_offset); // bit_enable;
  if (0 == value)
    reg_val &= ~(1 << index_offset);
  else
    reg_val |= (1 << index_offset);
  *reg_addr = reg_val;
}

GPIO::GPIO() : output_bits_(0), input_bits_(0), reserved_bits_(0) {}

gpio_bits_t GPIO::InitOutputs(gpio_bits_t outputs,
                              bool adafruit_pwm_transition_hack_needed) {
  if (s_GPIO_registers == NULL) {
    fprintf(stderr, "Attempt to init outputs but not yet Init()-ialized.\n");
    return 0;
  }

  const int kMaxAvailableBit = 22;
  for (int pin = 0; pin <= kMaxAvailableBit; ++pin) {
    if (outputs & GPIO_BIT(pin)) {
      gpio_write_mode(pin, OUTPUT);
    }
  }

  return outputs;
}

gpio_bits_t GPIO::RequestInputs(gpio_bits_t inputs) {
  if (s_GPIO_registers == NULL) {
    fprintf(stderr, "Attempt to init inputs but not yet Init()-ialized.\n");
    return 0;
  }

  const int kMaxAvailableBit = 22;
  for (int pin = 0; pin <= kMaxAvailableBit; ++pin) {
    if (inputs & GPIO_BIT(pin)) {
      gpio_write_mode(pin, INPUT);
    }
  }

  return inputs;
}

static int GetNumCores() { return 4; }

static uint32_t *mmap_rk_register() {

  int mem_fd;
  if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
    return NULL;
  }

  uint32_t *result =
      (uint32_t *)mmap(NULL,                // Any adddress in our space will do
                       REGISTER_BLOCK_SIZE, // Map length
                       PROT_READ | PROT_WRITE, // Enable r/w on GPIO registers.
                       MAP_SHARED,
                       mem_fd,           // File to map
                       RK3566_GPIO4_BASE // Offset to bcm register
      );
  close(mem_fd);

  if (result == MAP_FAILED) {
    perror("mmap error: ");
    fprintf(stderr, "MMapping from base 0x%X\n", RK3566_GPIO4_BASE);
    return NULL;
  }
  return result;
}

static bool mmap_all_rk_registers_once() {
  if (s_GPIO_registers != NULL)
    return true; // alrady done.

  // The common GPIO registers.
  s_GPIO_registers = mmap_rk_register();

  return s_GPIO_registers != NULL;
}

bool GPIO::Init(int slowdown) {

  // Pre-mmap all bcm registers we need now and possibly in the future, as to
  // allow  dropping privileges after GPIO::Init() even as some of these
  // registers might be needed later.
  if (!mmap_all_rk_registers_once())
    return false;

  gpio_set_bits_low_ = s_GPIO_registers;
  gpio_set_bits_high_ =
      gpio_set_bits_low_ + (RK3566_GPIO_SWPORT_DR_H_OFFSET / sizeof(uint32_t));
  gpio_read_bits_low_ =
      s_GPIO_registers + (RK3566_GPIO_EXT_PORT_OFFSET / sizeof(uint32_t));
  gpio_read_bits_high_ =
      gpio_read_bits_low_ + (RK3566_GPIO_SWPORT_DR_H_OFFSET / sizeof(uint32_t));

  return true;
}

bool GPIO::IsPi4() { return true; }

namespace {
// Manual timers.
class Timers {
public:
  static bool Init();
  static void sleep_nanos(long t);
};

// Simplest of PinPulsers. Uses somewhat jittery and manual timers
// to get the timing, but not optimal.
class TimerBasedPinPulser : public PinPulser {
public:
  TimerBasedPinPulser(GPIO *io, gpio_bits_t bits,
                      const std::vector<int> &nano_specs)
      : io_(io), bits_(bits), nano_specs_(nano_specs) {
    if (!s_Timer1Mhz) {
      fprintf(stderr,
              "FYI: not running as root which means we can't properly "
              "control timing unless this is a real-time kernel. Expect color "
              "degradation. Consider running as root with sudo.\n");
    }
  }

  virtual void SendPulse(int time_spec_number) {
    io_->ClearBits(bits_);
    Timers::sleep_nanos(nano_specs_[time_spec_number]);
    io_->SetBits(bits_);
  }

private:
  GPIO *const io_;
  const gpio_bits_t bits_;
  const std::vector<int> nano_specs_;
};

static void busy_wait_nanos(long nanos);
static void (*busy_wait_impl)(long) = busy_wait_nanos;

// Best effort write to file. Used to set kernel parameters.
static void WriteTo(const char *filename, const char *str) {
  const int fd = open(filename, O_WRONLY);
  if (fd < 0)
    return;
  (void)write(fd, str, strlen(str)); // Best effort. Ignore return value.
  close(fd);
}

// By default, the kernel applies some throtteling for realtime
// threads to prevent starvation of non-RT threads. But we
// really want all we can get iff the machine has more cores and
// our RT-thread is locked onto one of these.
// So let's tell it not to do that.
static void DisableRealtimeThrottling() {
  if (GetNumCores() == 1)
    return; // Not safe if we don't have > 1 core.
  // We need to leave the kernel a little bit of time, as it does not like
  // us to hog the kernel solidly. The default of 950000 leaves 50ms that
  // can generate visible flicker, so we reduce that to 1ms.
  WriteTo("/proc/sys/kernel/sched_rt_runtime_us", "999000");
}

bool Timers::Init() {
  if (!s_GPIO_registers)
    return false;

  // Choose the busy-wait loop that fits our Pi.

  busy_wait_impl = busy_wait_nanos;

  DisableRealtimeThrottling();
  // If we have it, we run the update thread on core3. No perf-compromises:
  WriteTo("/sys/devices/system/cpu/cpu3/cpufreq/scaling_governor",
          "performance");
  return true;
}

static uint32_t JitterAllowanceMicroseconds() {
  return EMPIRICAL_NANOSLEEP_OVERHEAD_US + 10; // this one is fast.
}

void Timers::sleep_nanos(long nanos) {
  // For smaller durations, we go straight to busy wait.

  // For larger duration, we use nanosleep() to give the operating system
  // a chance to do something else.

  // However, these timings have a lot of jitter, so if we have the 1Mhz timer
  // available, we use that to accurately mesure time spent and do the
  // remaining time with busy wait. If we don't have the timer available
  // (not running as root), we just use nanosleep() for larger values.

  if (s_Timer1Mhz) {
    static long kJitterAllowanceNanos = JitterAllowanceMicroseconds() * 1000;
    if (nanos > kJitterAllowanceNanos + MINIMUM_NANOSLEEP_TIME_US * 1000) {
      const uint32_t before = *s_Timer1Mhz;
      struct timespec sleep_time = {0, nanos - kJitterAllowanceNanos};
      nanosleep(&sleep_time, NULL);
      const uint32_t after = *s_Timer1Mhz;
      const long nanoseconds_passed = 1000 * (uint32_t)(after - before);
      if (nanoseconds_passed > nanos) {
        return; // darn, missed it.
      } else {
        nanos -= nanoseconds_passed; // remaining time with busy-loop
      }
    }
  } else {
    // Not running as root, not having access to 1Mhz timer. Approximate large
    // durations with nanosleep(); small durations are done with busy wait.
    if (nanos >
        (EMPIRICAL_NANOSLEEP_OVERHEAD_US + MINIMUM_NANOSLEEP_TIME_US) * 1000) {
      struct timespec sleep_time = {0, nanos - EMPIRICAL_NANOSLEEP_OVERHEAD_US *
                                                   1000};
      nanosleep(&sleep_time, NULL);
      return;
    }
  }

  busy_wait_impl(nanos); // Use model-specific busy-loop for remaining time.
}

static void busy_wait_nanos(long nanos) {
  if (nanos < 20)
    return;
  // Interesting, the Pi4 is _slower_ than the Pi3 ? At least for this busy loop
  for (uint32_t i = (nanos - 5) * 100 / 132; i != 0; --i) {
    asm("");
  }
}

} // end anonymous namespace

// Public PinPulser factory
PinPulser *PinPulser::Create(GPIO *io, gpio_bits_t gpio_mask,
                             bool allow_hardware_pulsing,
                             const std::vector<int> &nano_wait_spec) {
  if (!Timers::Init())
    return NULL;

  // if (allow_hardware_pulsing && HardwarePinPulser::CanHandle(gpio_mask)) {
  //   return new HardwarePinPulser(gpio_mask, nano_wait_spec);
  //} else {
  return new TimerBasedPinPulser(io, gpio_mask, nano_wait_spec);
  //}
}

// For external use, e.g. in the matrix for extra time.
uint32_t GetMicrosecondCounter() {
  if (s_Timer1Mhz)
    return *s_Timer1Mhz;

  // When run as non-root, we can't read the timer. Fall back to slow
  // operating-system ways.
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const uint64_t micros = ts.tv_nsec / 1000;
  const uint64_t epoch_usec = (uint64_t)ts.tv_sec * 1000000 + micros;
  return epoch_usec & 0xFFFFFFFF;
}

} // namespace rgb_matrix