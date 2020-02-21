/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file arduino_i2c.cpp
 *
 * I2C interface for a custom Arduino data collection solution
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <stdint.h>
#include <string.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/debug_key_value.h>
#include <uORB/uORB.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <errno.h>
#include <poll.h>

/* Imports used in other drivers that were unnecessary so far
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/types.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/drv_orb_dev.h>
#include <drivers/drv_sensor.h>
#include <sys/ioctl.h>

#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>

#include "board_config.h"

// This include was in the drivers that used a ring buffer, but
// it's unclear if this driver needs a ring buffer.
#include <drivers/device/ringbuffer.h>
*/

// Arduino is on the I2C bus
#define ARDUINO_BUS PX4_I2C_BUS_ONBOARD
// Make a couple of valid address for Arduinos to be.  Unclear
// if this is the right way to go about that.
#define ARDUINO_ADDR0 0x03
#define ARDUINO_ADDR1 0x73

// Define IOCTL commands
#define ARDUINO_IOCTL_RESET 0x0
#define ARDUINO_IOCTL_SELF_CHECK 0x1
#define ARDUINO_IOCTL_POLLRATE 0x2

// Define wire commands
#define ARDUINO_WIRE_RESET 0x0
#define ARDUINO_WIRE_SELF_CHECK 0x1
#define ARDUINO_WIRE_READ 0x02

// Define a device path.  Do there need to be more?
#define ARDUINO_DEVICE_BASE_PATH "/dev/arduino"
#define ARDUINO_DEVICE_PATH0 "/dev/arduino0"
#define ARDUINO_DEVICE_PATH1 "/dev/arduino1"

#define ARDUINO_POLLRATE_MAX 20
#define ARDUINO_POLLRATE_DEFAULT 5

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/**
 * A driver to collect data from an Arduino connected over
 * I2C
 *
 * This is to allow custom sensor integration like PWM RPM
 * sensors that are trivial to read from an Arduino but
 * hard to read into a Pixhawk
 */
class Arduino : public device::I2C {
public:
  Arduino(int address, const char *path);
  virtual ~Arduino();

  virtual int init();
  virtual ssize_t read(char *buffer, size_t buflen);
  virtual int ioctl(int cmd, unsigned long arg);

protected:
  virtual int probe();

private:
  orb_advert_t _orb_handle;
  work_s _work;
  float _conversion_interval;
  int _address;

  int _reset();
  int _self_check();
  int _collect();
  void _start();
  void _stop();
  void _cycle();
  static void _cycle_trampoline(void *arg);
};

extern "C" __EXPORT int arduino_main(int argc, char *argv[]);

/**
 * Constructor for the Arduino driver state object.
 *
 * All this really does is set the variables and call the I2C
 * constructor
 */
Arduino::Arduino(int address, const char *path)
    : I2C("Arduino", path, ARDUINO_BUS, address, 100000), _orb_handle(nullptr),
      _conversion_interval(0), _address(address) {
  memset(&_work, 0, sizeof(_work));
}

Arduino::~Arduino() {
  // Stop scheduled/scheduling work
  _stop();

  // Unadvertise from uOrb
  if (_orb_handle != nullptr) {
    orb_unadvertise(_orb_handle);
  }
}

/**
 * This initializes the driver by advertising on uOrb and calling
 * I2C::init()
 *
 * This shouldn't be doing anything fancy for the time being.
 */
int Arduino::init() {
  if (OK != I2C::init()) {
      DEVICE_LOG("Failed to initialize I2C");
      return PX4_ERROR;
  }

  if (_orb_handle == nullptr) {
    struct debug_key_value_s report = {};
    _orb_handle = orb_advertise(ORB_ID(debug_key_value), &report);
  }

  if (_orb_handle == nullptr) {
    DEVICE_DEBUG("Failed to advertise on uORB");
    return PX4_ERROR;
  }

  _start();

  if (_collect() != PX4_OK) {
      return PX4_ERROR;
  }

  return PX4_OK;
}

/**
 * Read implementation for the Arduino driver
 *
 * This reads a set of measurements from the Arduino
 * and reports these via debug messages.  It will probably
 * need to have some extra logic to prime the Arduino for reading
 * measurements as opposed to health check etc.
 */
ssize_t Arduino::read(char *buffer, size_t buflen) {
  int count = buflen / (4 * sizeof(float));
  uint8_t raw_vals[8] = {0};
  uint8_t wire_cmd = ARDUINO_WIRE_READ;
  int ret = transfer(&wire_cmd, 1, nullptr, 0);
  if (ret < 0) {
    return PX4_ERROR;
  }
  ret = transfer(nullptr, 0, &raw_vals[0], 4 * sizeof(float));

  // This is a dirty hack to facilitate transferring floats over
  // an iterface that works on bytes.  The float array is just
  // cast to a byte array and set over the wire where it is
  // then cast back into a float array from a byte array.
  float *vals = (float *)raw_vals;

  if (ret < 0) {
    DEVICE_DEBUG("error reading from sensor: %d", ret);
    return ret;
  }

  if (count > 0) {
    memcpy(buffer, vals, 4 * sizeof(float));
  }

  // it should return PX4_OK
  ret = PX4_OK;

  return ret;
}

/**
 * IOCTL implementation for the Arduino driver
 *
 * This method mostly just matches on command inputs and
 * uses the helper functions to actually perform any of the
 * IOCTL functions.
 */
int Arduino::ioctl(int cmd, unsigned long arg) {
  int ret = PX4_OK;
  switch (cmd) {
  case ARDUINO_IOCTL_RESET:
    ret = _reset();
    break;
  case ARDUINO_IOCTL_SELF_CHECK:
    ret = _self_check();
    break;
  case ARDUINO_IOCTL_POLLRATE:
    if (arg < ARDUINO_POLLRATE_MAX) {
      _conversion_interval = USEC2TICK(1000000 / arg);
    } else {
      ret = -EINVAL;
    }
    break;
  default:
    ret = -EINVAL;
    break;
  }
  return ret;
}

/**
 * Probe implementation for the Arduino driver (stub)
 *
 * Does a reset followed by a health check to check the
 * initial functionality of the Arduino.
 */
int Arduino::probe() {
  return OK;
  /*
  int result1 = ioctl(ARDUINO_IOCTL_RESET, 0);
  if (result1 != PX4_OK) {
    return result1;
  }
  int result2 = ioctl(ARDUINO_IOCTL_SELF_CHECK, 0);
  if (result2 != PX4_OK) {
    return result2;
  }
  return PX4_OK;
  */
}

/**
 * Helper method to send a reset command to the Arduino (stub)
 *
 * TODO
 *
 * This method should send a reset command and if it appears
 * to be successful, return PX4_OK.  If the reset doesn't
 * appear to have worked, return an error code.
 */
int Arduino::_reset() {
  int ret = PX4_OK;
  uint8_t wire_cmd = ARDUINO_WIRE_RESET;
  int ret2 = transfer(&wire_cmd, 1, nullptr, 0);
  if (ret2 < 0) {
    ret = PX4_ERROR;
  }
  return ret;
}

/**
 * Helper method to do a health check of the Arduino (stub)
 *
 * TODO
 *
 * This method should send a command over the wire instructing
 * the Arduino to send a health report (probably just a uint8_t)
 * back on the next request from the driver.  Then the driver
 * should read that health report and determine the status of the Arduino.
 */
int Arduino::_self_check() {
  int ret = PX4_OK;
  uint8_t wire_cmd = ARDUINO_WIRE_SELF_CHECK;
  int ret2 = transfer(&wire_cmd, 1, nullptr, 0);
  if (ret2 < 0) {
    ret = PX4_ERROR;
  }
  return ret;
}

int Arduino::_collect() {
  
  uint8_t raw_vals;
  static uint8_t data_tx;
  struct debug_key_value_s reports;
  struct debug_key_value_s dbg;
  int ret;

  int debug_sub_fd = orb_subscribe(ORB_ID(debug_key_value));
  bool updated = false;
  orb_check(debug_sub_fd, &updated);

  if(updated){
      orb_copy(ORB_ID(debug_key_value), debug_sub_fd, &dbg);
      orb_unsubscribe(debug_sub_fd);

  /* filter message based on its key attribute */
  if(strcmp(dbg.key, "chall") == 0) {
      data_tx = dbg.value;

  ret = transfer(&data_tx, 1, nullptr, 0);  // send data to fpga
  if (ret < 0) {
    return PX4_ERROR;
  }
 
  ret = transfer(nullptr, 0, &raw_vals, 1);     // read data from fpga

  if (ret != PX4_OK) {
    DEVICE_LOG("error reading from sensor: %d", ret);
    return ret;
  }

  uint64_t timestamp_us = hrt_absolute_time();
  
  if (_address == ARDUINO_ADDR0) {
    reports = (debug_key_value_s) {
        timestamp_us,
        raw_vals,
        "PUF_res",
    };
  } else {
    reports = (debug_key_value_s) {
        timestamp_us,
        raw_vals,
        "PUF_res",
    };
  }

  // Send the measurements out over uOrb to get included in the
  // logs and sent over Mavlink to the ground.
  if (_orb_handle != nullptr) {
    orb_publish(ORB_ID(debug_key_value), _orb_handle, &reports);
  } else {
    _orb_handle = orb_advertise(ORB_ID(debug_key_value), &reports);
  }

}
  // If we've made it to here, then nothing has gone wrong and
  // it should return PX4_OK
  ret = PX4_OK;

  return ret;
}

orb_unsubscribe(debug_sub_fd);

}

void Arduino::_start() {
  DEVICE_LOG("Starting Arduino work");
  work_queue(HPWORK, &_work, (worker_t)&Arduino::_cycle_trampoline, this,
             USEC2TICK(_conversion_interval));
}

void Arduino::_stop() { work_cancel(HPWORK, &_work); }

void Arduino::_cycle() {
  DEVICE_LOG("Running Arduino work");
  if (PX4_OK != _collect()) {
    DEVICE_LOG("Failed to collect data!");
    /* if error restart the measurement state machine */
    _start();
    return;
  }

  work_queue(HPWORK, &_work, (worker_t)&Arduino::_cycle_trampoline, this,
             USEC2TICK(_conversion_interval));
}

void Arduino::_cycle_trampoline(void *arg) {
  Arduino *dev = (Arduino *)arg;
  dev->_cycle();
}

namespace arduino {
Arduino *ard0 = nullptr;
Arduino *ard1 = nullptr;

void start(int address);
void stop(int address);
void reset(int address);

void start(int address) {
  Arduino **ard = nullptr;
  const char *dev_path = nullptr;
  if (address == ARDUINO_ADDR0) {
    ard = &ard0;
    dev_path = ARDUINO_DEVICE_PATH0;
  } else if (address == ARDUINO_ADDR1) {
    ard = &ard1;
    dev_path = ARDUINO_DEVICE_PATH1;
  } else {
    errx(1, "Driver failed to start, not one of the supported addresses");
  }
  if (*ard != nullptr) {
    errx(1, "Already started");
  }

  *ard = new Arduino(address, dev_path);

  if (*ard == nullptr) {
    errx(1, "Driver failed to start, Arduino constructor failed.");
  }

  if (OK != (*ard)->init()) {
    errx(1, "Driver failed to start, Arduino init failed.");
  }

  /*
   * TODO Probably need to open the device here
   * and do a couple IOCTL calls, but I'll fix that later
   */

  int filep = open(dev_path, O_RDONLY);

  ioctl(filep, ARDUINO_IOCTL_POLLRATE, ARDUINO_POLLRATE_DEFAULT);

  exit(0);
}

void stop(int address) {
  Arduino **ard = nullptr;
  if (address == ARDUINO_ADDR0) {
    ard = &ard0;
  } else if (address == ARDUINO_ADDR1) {
    ard = &ard1;
  } else {
    errx(1, "Failed to stop driver");
  }

  if (*ard == nullptr) {
    errx(1, "No driver to stop");
  }

  delete *ard;

  *ard = nullptr;

  exit(0);
}

void reset(int address) {
  Arduino **ard = nullptr;
  const char *dev_path = nullptr;
  if (address == ARDUINO_ADDR0) {
    ard = &ard0;
    dev_path = ARDUINO_DEVICE_PATH0;
  } else if (address == ARDUINO_ADDR1) {
    ard = &ard1;
    dev_path = ARDUINO_DEVICE_PATH1;
  } else {
    errx(1, "Failed to reset device.");
  }

  if (*ard == nullptr) {
    errx(1, "No driver loaded for this device.");
  }

  // TODO Make ioctl call to reset the device
  int filep = -1;
  filep = open(dev_path, O_RDONLY);

  if (filep < 0) {
    errx(1, "Failed to open file descriptor");
  }

  if (ioctl(filep, ARDUINO_IOCTL_RESET, 0) < 0) {
    ::close(filep);
    errx(1, "Failed to reset device.");
  }

  ::close(filep);
  exit(0);
}
} // namespace arduino

/**
 * Main method for the Arduino driver because apparently drivers
 * have main methods.
 */
int arduino_main(int argc, char *argv[]) {
  int ch;
  int myoptind = 1;
  const char *myoptarg = nullptr;
  int address = -1;

  while ((ch = px4_getopt(argc, argv, "a:", &myoptind, &myoptarg)) != EOF) {
    switch (ch) {
    case 'a':
      address = (int)strtol(myoptarg, NULL, 16);
      break;
    default:
      PX4_WARN("Unknown option!");
      return -1;
    }
  }
  if (address < 0) {
    PX4_ERR("Failed to find address option or invalid address was parsed");
    return -1;
  }
  if (myoptind >= argc) {
    PX4_ERR("unrecognized command, try 'start', 'stop', or 'reset'");
    return -1;
  }

  if (!strcmp(argv[myoptind], "start")) {
    arduino::start(address);
  }

  if (!strcmp(argv[myoptind], "stop")) {
    arduino::stop(address);
  }

  if (!strcmp(argv[myoptind], "reset")) {
    arduino::reset(address);
  }

  return 0;
}