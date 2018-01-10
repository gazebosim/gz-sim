/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_TEMPERATURE_HH_
#define IGNITION_MATH_TEMPERATURE_HH_

#include <iostream>
#include <memory>

#include <ignition/math/config.hh>
#include "ignition/math/Helpers.hh"

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    // Forward declare private data class.
    class TemperaturePrivate;

    /// \brief A class that stores temperature information, and allows
    /// conversion between different units.
    ///
    /// This class is mostly for convenience. It can be used to easily
    /// convert between temperature units and encapsulate temperature values.
    ///
    /// The default unit is Kelvin. Most functions that accept a double
    /// value will assume the double is Kelvin. The exceptions are a few of
    /// the conversion functions, such as CelsiusToFahrenheit. Similarly,
    /// most doubles that are returned will be in Kelvin.
    ///
    /// ## Example usage ##
    ///
    /// ### Convert from Kelvin to Celsius ###
    ///
    ///     double celsius = ignition::math::Temperature::KelvinToCelsius(2.5);
    ///
    /// ### Create and use a Temperature object ###
    ///
    ///     ignition::math::Temperature temp(123.5);
    ///     std::cout << "Temperature in Kelvin = " << temp << std::endl;
    ///     std::cout << "Temperature in Celsius = "
    ///               << temp.Celsius() << std::endl;
    ///
    ///     temp += 100.0;
    ///     std::cout << "Temperature + 100.0 = " << temp << "K" << std::endl;
    ///
    ///     ignition::math::Temperature newTemp(temp);
    ///     newTemp += temp + 23.5;
    ///     std::cout << "Copied the temp object and added 23.5K. newTemp = "
    ///               << newTemp.Fahrenheit() << "F" << std::endl;
    ///
    class IGNITION_MATH_VISIBLE Temperature
    {
      /// \brief Default constructor
      public: Temperature();

      /// \brief Kelvin value constructor. This is a conversion constructor
      /// \param[in] _temp Temperature in Kelvin
      // cppcheck-suppress noExplicitConstructor
      public: Temperature(const double _temp);

      /// \brief Copy constructor
      /// \param[in] _temp Temperature object to copy.
      public: Temperature(const Temperature &_temp);

      /// \brief Destructor
      public: virtual ~Temperature();

      /// \brief Convert Kelvin to Celsius
      /// \param[in] _temp Temperature in Kelvin
      /// \return Temperature in Celsius
      public: static double KelvinToCelsius(const double _temp);

      /// \brief Convert Kelvin to Fahrenheit
      /// \param[in] _temp Temperature in Kelvin
      /// \return Temperature in Fahrenheit
      public: static double KelvinToFahrenheit(const double _temp);

      /// \brief Convert Celsius to Fahrenheit
      /// \param[in] _temp Temperature in Celsius
      /// \return Temperature in Fahrenheit
      public: static double CelsiusToFahrenheit(const double _temp);

      /// \brief Convert Celsius to Kelvin
      /// \param[in] _temp Temperature in Celsius
      /// \return Temperature in Kelvin
      public: static double CelsiusToKelvin(const double _temp);

      /// \brief Convert Fahrenheit to Celsius
      /// \param[in] _temp Temperature in Fahrenheit
      /// \return Temperature in Celsius
      public: static double FahrenheitToCelsius(const double _temp);

      /// \brief Convert Fahrenheit to Kelvin
      /// \param[in] _temp Temperature in Fahrenheit
      /// \return Temperature in Kelvin
      public: static double FahrenheitToKelvin(const double _temp);

      /// \brief Set the temperature from a Kelvin value
      /// \param[in] _temp Temperature in Kelvin
      public: void SetKelvin(const double _temp);

      /// \brief Set the temperature from a Celsius value
      /// \param[in] _temp Temperature in Celsius
      public: void SetCelsius(const double _temp);

      /// \brief Set the temperature from a Fahrenheit value
      /// \param[in] _temp Temperature in Fahrenheit
      public: void SetFahrenheit(const double _temp);

      /// \brief Get the temperature in Kelvin
      /// \return Temperature in Kelvin
      public: double Kelvin() const;

      /// \brief Get the temperature in Celsius
      /// \return Temperature in Celsius
      public: double Celsius() const;

      /// \brief Get the temperature in Fahrenheit
      /// \return Temperature in Fahrenheit
      public: double Fahrenheit() const;

      /// \brief Accessor operator
      /// \return Temperature in Kelvin
      /// \sa Kelvin()
      public: double operator()() const;

      /// \brief Assignment operator
      /// \param[in] _temp Temperature in Kelvin
      /// \return Reference to this instance
      public: Temperature &operator=(const double _temp);

      /// \brief Assignment operator
      /// \param[in] _temp Temperature object
      /// \return Reference to this instance
      public: Temperature &operator=(const Temperature &_temp);

      /// \brief Addition operator
      /// \param[in] _temp Temperature in Kelvin
      /// \return Resulting temperature
      public: Temperature operator+(const double _temp);

      /// \brief Addition operator
      /// \param[in] _temp Temperature object
      /// \return Resulting temperature
      public: Temperature operator+(const Temperature &_temp);

      /// \brief Addition operator for double type.
      /// \param[in] _t Temperature in kelvin
      /// \param[in] _temp Temperature object
      /// \return Resulting temperature
      public: friend Temperature operator+(double _t, const Temperature &_temp)
      {
        return _t + _temp.Kelvin();
      }

      /// \brief Addition assignment operator
      /// \param[in] _temp Temperature in Kelvin
      /// \return Reference to this instance
      public: const Temperature &operator+=(const double _temp);

      /// \brief Addition assignment operator
      /// \param[in] _temp Temperature object
      /// \return Reference to this instance
      public: const Temperature &operator+=(const Temperature &_temp);

      /// \brief Subtraction operator
      /// \param[in] _temp Temperature in Kelvin
      /// \return Resulting temperature
      public: Temperature operator-(const double _temp);

      /// \brief Subtraction operator
      /// \param[in] _temp Temperature object
      /// \return Resulting temperature
      public: Temperature operator-(const Temperature &_temp);

      /// \brief Subtraction operator for double type.
      /// \param[in] _t Temperature in kelvin
      /// \param[in] _temp Temperature object
      /// \return Resulting temperature
      public: friend Temperature operator-(double _t, const Temperature &_temp)
      {
        return _t - _temp.Kelvin();
      }

      /// \brief Subtraction assignment operator
      /// \param[in] _temp Temperature in Kelvin
      /// \return Reference to this instance
      public: const Temperature &operator-=(const double _temp);

      /// \brief Subtraction assignment operator
      /// \param[in] _temp Temperature object
      /// \return Reference to this instance
      public: const Temperature &operator-=(const Temperature &_temp);

      /// \brief Multiplication operator
      /// \param[in] _temp Temperature in Kelvin
      /// \return Resulting temperature
      public: Temperature operator*(const double _temp);

      /// \brief Multiplication operator
      /// \param[in] _temp Temperature object
      /// \return Resulting temperature
      public: Temperature operator*(const Temperature &_temp);

      /// \brief Multiplication operator for double type.
      /// \param[in] _t Temperature in kelvin
      /// \param[in] _temp Temperature object
      /// \return Resulting temperature
      public: friend Temperature operator*(double _t, const Temperature &_temp)
      {
        return _t * _temp.Kelvin();
      }

      /// \brief Multiplication assignment operator
      /// \param[in] _temp Temperature in Kelvin
      /// \return Reference to this instance
      public: const Temperature &operator*=(const double _temp);

      /// \brief Multiplication assignment operator
      /// \param[in] _temp Temperature object
      /// \return Reference to this instance
      public: const Temperature &operator*=(const Temperature &_temp);

      /// \brief Division operator
      /// \param[in] _temp Temperature in Kelvin
      /// \return Resulting temperature
      public: Temperature operator/(const double _temp);

      /// \brief Division operator
      /// \param[in] _temp Temperature object
      /// \return Resulting temperature
      public: Temperature operator/(const Temperature &_temp);

      /// \brief Division operator for double type.
      /// \param[in] _t Temperature in kelvin
      /// \param[in] _temp Temperature object
      /// \return Resulting temperature
      public: friend Temperature operator/(double _t, const Temperature &_temp)
      {
        return _t / _temp.Kelvin();
      }

      /// \brief Division assignment operator
      /// \param[in] _temp Temperature in Kelvin
      /// \return Reference to this instance
      public: const Temperature &operator/=(const double _temp);

      /// \brief Division assignment operator
      /// \param[in] _temp Temperature object
      /// \return Reference to this instance
      public: const Temperature &operator/=(const Temperature &_temp);

      /// \brief Equal to operator
      /// \param[in] _temp The temperature to compare
      /// \return true if the temperatures are the same, false otherwise
      public: bool operator==(const Temperature &_temp) const;

      /// \brief Equal to operator, where the value of _temp is assumed to
      /// be in Kelvin
      /// \param[in] _temp The temperature (in Kelvin) to compare
      /// \return true if the temperatures are the same, false otherwise
      public: bool operator==(const double _temp) const;

      /// \brief Inequality to operator
      /// \param[in] _temp The temperature to compare
      /// \return false if the temperatures are the same, true otherwise
      public: bool operator!=(const Temperature &_temp) const;

      /// \brief Inequality to operator, where the value of _temp is assumed to
      /// be in Kelvin
      /// \param[in] _temp The temperature (in Kelvin) to compare
      /// \return false if the temperatures are the same, true otherwise
      public: bool operator!=(const double _temp) const;

      /// \brief Less than to operator
      /// \param[in] _temp The temperature to compare
      /// \return True if this is less than _temp.
      public: bool operator<(const Temperature &_temp) const;

      /// \brief Less than operator, where the value of _temp is assumed to
      /// be in Kelvin
      /// \param[in] _temp The temperature (in Kelvin) to compare
      /// \return True if this is less than _temp.
      public: bool operator<(const double _temp) const;

      /// \brief Less than or equal to operator
      /// \param[in] _temp The temperature to compare
      /// \return True if this is less than or equal _temp.
      public: bool operator<=(const Temperature &_temp) const;

      /// \brief Less than or equal operator,
      /// where the value of _temp is assumed to be in Kelvin
      /// \param[in] _temp The temperature (in Kelvin) to compare
      /// \return True if this is less than or equal to _temp.
      public: bool operator<=(const double _temp) const;

      /// \brief Greater than operator
      /// \param[in] _temp The temperature to compare
      /// \return True if this is greater than _temp.
      public: bool operator>(const Temperature &_temp) const;

      /// \brief Greater than operator, where the value of _temp is assumed to
      /// be in Kelvin
      /// \param[in] _temp The temperature (in Kelvin) to compare
      /// \return True if this is greater than _temp.
      public: bool operator>(const double _temp) const;

      /// \brief Greater than or equal to operator
      /// \param[in] _temp The temperature to compare
      /// \return True if this is greater than or equal to _temp.
      public: bool operator>=(const Temperature &_temp) const;

      /// \brief Greater than equal operator,
      /// where the value of _temp is assumed to be in Kelvin
      /// \param[in] _temp The temperature (in Kelvin) to compare
      /// \return True if this is greater than or equal to _temp.
      public: bool operator>=(const double _temp) const;

      /// \brief Stream insertion operator
      /// \param[in] _out the output stream
      /// \param[in] _temp Temperature to write to the stream
      /// \return the output stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const ignition::math::Temperature &_temp)
              {
                _out << _temp.Kelvin();
                return _out;
              }

      /// \brief Stream extraction operator
      /// \param[in] _in the input stream
      /// \param[in] _temp Temperature to read from to the stream. Assumes
      /// temperature value is in Kelvin.
      /// \return the input stream
      public: friend std::istream &operator>>(std::istream &_in,
                  ignition::math::Temperature &_temp)
              {
                // Skip white spaces
                _in.setf(std::ios_base::skipws);

                double kelvin;
                _in >> kelvin;

                _temp.SetKelvin(kelvin);
                return _in;
              }

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
      /// \brief Private data pointer.
      private: std::unique_ptr<TemperaturePrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
    };
    }
  }
}
#endif
