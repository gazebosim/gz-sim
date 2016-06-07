/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <cmath>
#include <iostream>
#include <ignition/math/SignalStatsPrivate.hh>
#include <ignition/math/SignalStats.hh>

using namespace ignition;
using namespace math;

//////////////////////////////////////////////////
SignalStatistic::SignalStatistic()
  : dataPtr(new SignalStatisticPrivate)
{
  this->dataPtr->data = 0.0;
  this->dataPtr->extraData = 0.0;
  this->dataPtr->count = 0;
}

//////////////////////////////////////////////////
SignalStatistic::~SignalStatistic()
{
  delete this->dataPtr;
  this->dataPtr = 0;
}

//////////////////////////////////////////////////
size_t SignalStatistic::Count() const
{
  return this->dataPtr->count;
}

//////////////////////////////////////////////////
void SignalStatistic::Reset()
{
  this->dataPtr->data = 0;
  this->dataPtr->count = 0;
}

//////////////////////////////////////////////////
double SignalMaximum::Value() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
std::string SignalMaximum::ShortName() const
{
  return "max";
}

//////////////////////////////////////////////////
void SignalMaximum::InsertData(const double _data)
{
  if (this->dataPtr->count == 0 || _data > this->dataPtr->data)
  {
    this->dataPtr->data = _data;
  }
  this->dataPtr->count++;
}

//////////////////////////////////////////////////
double SignalMean::Value() const
{
  if (this->dataPtr->count == 0)
  {
    return 0;
  }
  return this->dataPtr->data / this->dataPtr->count;
}

//////////////////////////////////////////////////
std::string SignalMean::ShortName() const
{
  return "mean";
}

//////////////////////////////////////////////////
void SignalMean::InsertData(const double _data)
{
  this->dataPtr->data += _data;
  this->dataPtr->count++;
}

//////////////////////////////////////////////////
double SignalMinimum::Value() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
std::string SignalMinimum::ShortName() const
{
  return "min";
}

//////////////////////////////////////////////////
void SignalMinimum::InsertData(const double _data)
{
  if (this->dataPtr->count == 0 || _data < this->dataPtr->data)
  {
    this->dataPtr->data = _data;
  }
  this->dataPtr->count++;
}

//////////////////////////////////////////////////
double SignalRootMeanSquare::Value() const
{
  if (this->dataPtr->count == 0)
  {
    return 0;
  }
  return sqrt(this->dataPtr->data / this->dataPtr->count);
}

//////////////////////////////////////////////////
std::string SignalRootMeanSquare::ShortName() const
{
  return "rms";
}

//////////////////////////////////////////////////
void SignalRootMeanSquare::InsertData(const double _data)
{
  this->dataPtr->data += _data * _data;
  this->dataPtr->count++;
}

//////////////////////////////////////////////////
double SignalMaxAbsoluteValue::Value() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
std::string SignalMaxAbsoluteValue::ShortName() const
{
  return "maxAbs";
}

//////////////////////////////////////////////////
void SignalMaxAbsoluteValue::InsertData(const double _data)
{
  double absData = std::abs(_data);
  if (absData > this->dataPtr->data)
  {
    this->dataPtr->data = absData;
  }
  this->dataPtr->count++;
}

//////////////////////////////////////////////////
// wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
// based on Knuth's algorithm
double SignalVariance::Value() const
{
  if (this->dataPtr->count < 2)
    return 0.0;

  // variance = M2 / (n - 1)
  return this->dataPtr->data / (this->dataPtr->count - 1);
}

//////////////////////////////////////////////////
std::string SignalVariance::ShortName() const
{
  return "var";
}

//////////////////////////////////////////////////
// wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
// based on Knuth's algorithm
void SignalVariance::InsertData(const double _data)
{
  // n++
  this->dataPtr->count++;

  // delta = x - mean
  double delta = _data - this->dataPtr->extraData;

  // mean += delta / n
  this->dataPtr->extraData += delta / this->dataPtr->count;

  // M2 += delta*(x - mean)
  this->dataPtr->data += delta * (_data - this->dataPtr->extraData);
}

//////////////////////////////////////////////////
SignalStats::SignalStats()
  : dataPtr(new SignalStatsPrivate)
{
}

//////////////////////////////////////////////////
SignalStats::~SignalStats()
{
}

//////////////////////////////////////////////////
size_t SignalStats::Count() const
{
  if (this->dataPtr->stats.empty())
    return 0;

  return this->dataPtr->stats.front()->Count();
}

//////////////////////////////////////////////////
std::map<std::string, double> SignalStats::Map() const
{
  std::map<std::string, double> map;
  for (auto const &statistic : this->dataPtr->stats)
  {
    map[statistic->ShortName()] = statistic->Value();
  }
  return map;
}

//////////////////////////////////////////////////
void SignalStats::InsertData(const double _data)
{
  for (auto &statistic : this->dataPtr->stats)
  {
    statistic->InsertData(_data);
  }
}

//////////////////////////////////////////////////
bool SignalStats::InsertStatistic(const std::string &_name)
{
  // Check if the statistic is already inserted
  {
    auto map = this->Map();
    if (map.find(_name) != map.end())
    {
      std::cerr << "Unable to InsertStatistic ["
                << _name
                << "] since it has already been inserted."
                << std::endl;
      return false;
    }
  }

  SignalStatisticPtr stat;
  if (_name == "max")
  {
    stat.reset(new SignalMaximum());
  }
  else if (_name == "maxAbs")
  {
    stat.reset(new SignalMaxAbsoluteValue());
  }
  else if (_name == "mean")
  {
    stat.reset(new SignalMean());
  }
  else if (_name == "min")
  {
    stat.reset(new SignalMinimum());
  }
  else if (_name == "rms")
  {
    stat.reset(new SignalRootMeanSquare());
  }
  else if (_name == "var")
  {
    stat.reset(new SignalVariance());
  }
  else
  {
    // Unrecognized name string
    std::cerr << "Unable to InsertStatistic ["
              << _name
              << "] since it is an unrecognized name."
              << std::endl;
    return false;
  }
  this->dataPtr->stats.push_back(stat);
  return true;
}

//////////////////////////////////////////////////
bool SignalStats::InsertStatistics(const std::string &_names)
{
  if (_names.empty())
  {
    std::cerr << "Unable to InsertStatistics "
              << "since no names were supplied."
              << std::endl;
    return false;
  }

  bool result = true;
  std::vector<std::string> names;
  // Replace the following with std::string functions
  // boost::split(names, _names, boost::is_any_of(","));
  // std::regex_token_iterator may be considered when it
  // is supported by gcc
  {
    std::string::size_type start = 0;
    std::string::size_type end = _names.find(',', start);
    while (end != std::string::npos)
    {
      names.push_back(_names.substr(start, end-start));
      start = end + 1;
      end = _names.find(',', start);
    }
    if (start < _names.length())
    {
      names.push_back(_names.substr(start));
    }
  }
  for (auto const &name : names)
  {
    result = result && this->InsertStatistic(name);
  }
  return result;
}

//////////////////////////////////////////////////
void SignalStats::Reset()
{
  for (auto &statistic : this->dataPtr->stats)
  {
    statistic->Reset();
  }
}

//////////////////////////////////////////////////
SignalStats &SignalStats::operator=(const SignalStats &_s)
{
  this->dataPtr = _s.dataPtr->Clone();
  return *this;
}
