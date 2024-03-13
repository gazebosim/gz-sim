/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
// This code is adapted from https://wiki.ogre3d.org/All-purpose+script+parser

#ifndef RENDERING__MATERIALPARSER__CONFIGLOADER_HH_
#define RENDERING__MATERIALPARSER__CONFIGLOADER_HH_

#include <cassert>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace gz
{
namespace sim
{
class ConfigNode;

class ConfigLoader
{
public:
  static void LoadMaterialFiles(ConfigLoader * _c);

  ConfigLoader();

  ~ConfigLoader();

  // For a line like
  // entity animals/dog
  // {
  //    ...
  // }
  // The type is "entity" and the name is "animals/dog"
  // Or if animal/dog was not there then name is ""
  virtual ConfigNode * GetConfigScript(const std::string & _name);

  virtual std::map<std::string, ConfigNode *> GetAllConfigScripts();

  virtual void ParseScript(std::ifstream & _stream);

protected:
  std::map<std::string, ConfigNode *> mScriptList;

  enum Token
  {
    TOKEN_Text,
    TOKEN_NewLine,
    TOKEN_OpenBrace,
    TOKEN_CloseBrace,
    TOKEN_EOF,
  };

  Token tok, lastTok;
  std::string tokVal, lastTokVal;

  void parseNodes(std::ifstream & stream, ConfigNode * parent);
  void nextToken(std::ifstream & stream);
  void skipNewLines(std::ifstream & stream);

  virtual void clearScriptList();
};

class ConfigNode
{
public:
  explicit ConfigNode(ConfigNode * _parent,
    const std::string & _name = "untitled");

  ~ConfigNode();

  inline void SetName(const std::string & _name)
  {
    this->mName = _name;
  }

  inline std::string & GetName()
  {
    return mName;
  }

  inline void AddValue(const std::string & _value)
  {
    mValues.push_back(_value);
  }

  inline void ClearValues()
  {
    mValues.clear();
  }

  inline std::vector<std::string> & GetValues()
  {
    return mValues;
  }

  inline const std::string & GetValue(unsigned int index = 0)
  {
    assert(index < mValues.size());
    return mValues[index];
  }

  inline void GetValuesInFloat(std::vector<float> & floatValues)
  {
    for (const auto & str : mValues)
    {
      floatValues.push_back(std::stof(str));
    }
  }

  void GetColorValues(math::Color & _colorValues, unsigned int _size);

  ConfigNode * AddChild(
    const std::string & _name = "untitled", bool _replaceExisting = false);

  ConfigNode * FindChild(const std::string & _name, bool _recursive = false);

  inline std::vector<ConfigNode *> & GetChildren()
  {
    return mChildren;
  }

  inline ConfigNode * GetChild(unsigned int index = 0)
  {
    assert(index < mChildren.size());
    return mChildren[index];
  }

  void SetParent(ConfigNode * newParent);

  inline ConfigNode * GetParent()
  {
    return mParent;
  }

private:
  std::string mName;

  std::vector<std::string> mValues;

  std::vector<ConfigNode *> mChildren;

  ConfigNode * mParent;

  // The last child node's index found with a call to findChild()
  int mLastChildFound;

  std::vector<ConfigNode *> ::iterator iter;

  bool removeSelf;
};
}  // namespace sim
}  // namespace gz

#endif  // RENDERING__MATERIALPARSER__CONFIGLOADER_HH_
