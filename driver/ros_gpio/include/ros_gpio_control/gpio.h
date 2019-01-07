/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 ThundeRatz

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#ifndef ROS_GPIO_CONTROL_GPIO_H
#define ROS_GPIO_CONTROL_GPIO_H

#include <string>

#include <poll.h>

class GPIO
{
public:
  static const std::string IN, OUT, OUT_LOW, OUT_HIGH;
  static const std::string EDGE_NONE, EDGE_RISE, EDGE_FALL,
         EDGE_BOTH;
  explicit GPIO(int gpio);
  ~GPIO();
  void export_gpio();
  int poll(int timeout);
  void poll();
  void direction(const std::string &direction);
  void active_low();
  void edge(const std::string &edge_type);
  void operator=(bool value);
  operator bool() const;

private:
  // value_fd é salvo por ser o mais acessado e para não abrir e
  // fechar toda a hora
  int fd_value;
  int gpio;
  struct pollfd poll_targets;
  void write_to_file(const std::string& name, const std::string& value);
  int read() const;
  bool exported();
};

class GPIOButton
{
public:
  explicit GPIOButton(int gpio, int switch_debounce = 50);
  ~GPIOButton();
  operator bool() const;
private:
  GPIO button;
};

#endif  // ROS_GPIO_CONTROL_GPIO_H
