/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation, Inc. nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <map>
#include <string>
#include <vector>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>

namespace ros_parameter {

  template <class T>
  class Parameter
  {
  public:
    typedef boost::shared_ptr<T> Ptr;

    Parameter() : name_(), data_() {}
    ~Parameter() {}

    Parameter(const std::string &name, const T &default_value)
    : name_(name), data_(Ptr(new T(default_value))) {}

    Parameter(const Parameter<T> &other) {
      this->data_ = other.data_;
    }

    T data() {
      return *(this->data_);
    }

    void get_data(T &data) {
      data = *(this->data_);
    }

    Ptr data_ptr() {
      return this->data_;
    }

    void data(const T &data) {
      *(this->data_) = data;
    }

    void data_ptr(const Ptr &data_ptr) {
      data_ptr = this->data_;
    }

    std::string name() const {
      return this->name_;
    }

    void name(const std::string &name) {
      name = this->name_;
    }

  private:
    std::string name_;
    Ptr data_;
  };

  class ParameterGroup
  {
  public:
    typedef boost::function<void(ParameterGroup&, std::map<std::string, bool>&)> CallbackType;
    typedef boost::variant<Parameter<bool>, Parameter<int>, Parameter<double>, Parameter<std::string> > ParameterType;

    ParameterGroup() {}
    ~ParameterGroup() {}

    template <typename T>
    void add_parameter(const Parameter<T> param) {
      std::string name = param.name();
      this->parameters_[name] = param;
    }

    void on_change(CallbackType callback) {
      this->on_change_callback_ = callback;
    }

    template <typename T>
    Parameter<T> get(const std::string &name) {
      return boost::get<Parameter<T> >(this->parameters_[name]);
    }

    template <typename T>
    void get_data(const std::string &name, T &data) {
      Parameter<T> param = this->get<T>(name);
      param.get_data(data);
    }

  private:
    CallbackType on_change_callback_;
    std::map<std::string, ParameterType> parameters_;
  };

} // namespace ros_parameter
