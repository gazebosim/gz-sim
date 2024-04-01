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

#include <map>
#include <exception>
#include <fstream>
#include <filesystem>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/math/Color.hh>
#include "gz/sim/InstallationDirectories.hh"

#include "ConfigLoader.hh"

using namespace gz;
using namespace sim;

void ConfigLoader::LoadMaterialFiles(ConfigLoader * _c)
{
  try
  {
    std::string installedConfig = common::joinPaths(
        gz::sim::getMediaInstallDir(),
        "gazebo.material");
    std::ifstream in(installedConfig, std::ios::binary);
    _c->ParseScript(in);
  }
  catch(std::filesystem::filesystem_error & e)
  {
    gzerr << e.what() << std::endl;
  }
}

ConfigLoader::ConfigLoader()
{
}

ConfigLoader::~ConfigLoader()
{
  clearScriptList();
}

void ConfigLoader::clearScriptList()
{
  for (auto & i : mScriptList)
  {
    delete i.second;
  }
  mScriptList.clear();
}

ConfigNode * ConfigLoader::GetConfigScript(const std::string & _name)
{
  auto i = mScriptList.find(_name);
  // If found..
  if (i != mScriptList.end())
  {
    return i->second;
  }
  else
  {
    return nullptr;
  }
}

std::map<std::string, ConfigNode *> ConfigLoader::GetAllConfigScripts()
{
  return mScriptList;
}

void ConfigLoader::ParseScript(std::ifstream & _stream)
{
  // Get first token
  nextToken(_stream);
  if (tok == TOKEN_EOF)
  {
    _stream.close();
    return;
  }

  // Parse the script
  parseNodes(_stream, 0);

  _stream.close();
}

void ConfigLoader::nextToken(std::ifstream & stream)
{
  lastTok = tok;
  lastTokVal = tokVal;

  // EOF token
  if (stream.eof())
  {
    tok = TOKEN_EOF;
    return;
  }

  // Get next character
  int ch = stream.get();
  if (ch == -1)
  {
    tok = TOKEN_EOF;
    return;
  }
  // Skip leading spaces / tabs
  while ((ch == ' ' || ch == 9) && !stream.eof())
  {
    ch = stream.get();
  }

  if (stream.eof())
  {
    tok = TOKEN_EOF;
    return;
  }

  // Newline token
  if (ch == '\r' || ch == '\n')
  {
    do
    {
      ch = stream.get();
    } while ((ch == '\r' || ch == '\n') && !stream.eof());

    stream.unget();

    tok = TOKEN_NewLine;
    return;
  }
  else if (ch == '{')
  {
    // Open brace token
    tok = TOKEN_OpenBrace;
    return;
  }
  else if (ch == '}')
  {
    // Close brace token
    tok = TOKEN_CloseBrace;
    return;
  }

  // Text token
  if (ch < 32 || ch > 122)
  {  // Verify valid char
    throw std::runtime_error(
      "Parse Error: Invalid character, ConfigLoader::load()");
  }

  tokVal = "";
  tok = TOKEN_Text;
  do
  {
    // Skip comments
    if (ch == '/')
    {
      int ch2 = stream.peek();

      // C++ style comment (//)
      if (ch2 == '/')
      {
        stream.get();
        do
        {
          ch = stream.get();
        } while (ch != '\r' && ch != '\n' && !stream.eof());

        tok = TOKEN_NewLine;
        return;
      }
      else if (ch2 == '*')
      {
        stream.get();
        do
        {
          ch = stream.get();
          ch2 = stream.peek();
        } while (!(ch == '*' && ch2 == '/') && !stream.eof());
        stream.get();

        do
        {
          ch = stream.get();
        } while (ch != '\r' && ch != '\n' && !stream.eof());
        continue;
      }
    }

    // Add valid char to tokVal
    tokVal += static_cast<char>(ch);

    // Next char
    ch = stream.get();
  } while (ch > 32 && ch <= 122 && !stream.eof());

  stream.unget();

  return;
}

void ConfigLoader::skipNewLines(std::ifstream & stream)
{
  while (tok == TOKEN_NewLine)
  {
    nextToken(stream);
  }
}

void ConfigLoader::parseNodes(std::ifstream & stream, ConfigNode * parent)
{
  typedef std::pair < std::string, ConfigNode * > ScriptItem;

  while (true)
  {
    switch (tok)
    {
      // Node
      case TOKEN_Text:
        // Add the new node
        ConfigNode * newNode;
        if (parent)
        {
          newNode = parent->AddChild(tokVal);
        }
        else
        {
          newNode = new ConfigNode(0, tokVal);
        }

        // Get values
        nextToken(stream);
        while (tok == TOKEN_Text)
        {
          newNode->AddValue(tokVal);
          nextToken(stream);
        }

        // Add root nodes to scriptList
        if (!parent)
        {
          std::string key;

          if (newNode->GetValues().empty())
          {
            key = newNode->GetName() + ' ';
          }
          else
          {
            key = newNode->GetName() + ' ' + newNode->GetValues().front();
          }

          mScriptList.insert(ScriptItem(key, newNode));
        }

        skipNewLines(stream);

        // Add any sub-nodes
        if (tok == TOKEN_OpenBrace)
        {
          // Parse nodes
          nextToken(stream);
          parseNodes(stream, newNode);
          // Check for matching closing brace
          if (tok != TOKEN_CloseBrace)
          {
            throw std::runtime_error("Parse Error: Expecting closing brace");
          }
          nextToken(stream);
          skipNewLines(stream);
        }

        break;

      // Out of place brace
      case TOKEN_OpenBrace:
        throw std::runtime_error("Parse Error: Opening brace out of plane");
        break;

      // Return if end of nodes have been reached
      case TOKEN_CloseBrace:
        return;

      // Return if reached end of file
      case TOKEN_EOF:
        return;

      case TOKEN_NewLine:
        nextToken(stream);
        break;

      default:
        break;
    }
  }
}

ConfigNode::ConfigNode(ConfigNode * _parent, const std::string & _name)
{
  mName = _name;
  mParent = _parent;
  // For proper destruction
  removeSelf = true;
  mLastChildFound = -1;

  // Add self to parent's child list
  // (unless this is the root node being created)
  if (_parent != NULL)
  {
    mParent->mChildren.push_back(this);
    iter = --(mParent->mChildren.end());
  }
}

ConfigNode::~ConfigNode()
{
  // Delete all children
  std::vector < ConfigNode * > ::iterator i;
  for (i = mChildren.begin(); i != mChildren.end(); i++)
  {
    ConfigNode * node = *i;
    node->removeSelf = false;
    delete node;
  }
  mChildren.clear();

  // Remove self from parent's child list
  if (removeSelf && mParent != NULL)
  {
    mParent->mChildren.erase(iter);
  }
}

void ConfigNode::GetColorValues(gz::math::Color & _colorValues,
  unsigned int _size)
{
  std::vector<float> floatValues;
  ConfigNode::GetValuesInFloat(floatValues);
  if (floatValues.size() < _size)
  {
    gzerr << "Bad material file." << std::endl;
    floatValues.resize(_size);
  }

  // clamp the color values to valid ranges
  for (unsigned int i = 0; i < floatValues.size(); ++i)
  {
    if (!(floatValues[i] >= 0))
    {
      floatValues[i] = 0;
    }
    if (floatValues[i] > 1)
    {
      floatValues[i] = floatValues[i]/255.0f;
    }
  }

  if (_size == 3)
  {
    _colorValues = gz::math::Color(floatValues[0], floatValues[1],
                  floatValues[2]);
  }
  if (_size == 4)
  {
    _colorValues = gz::math::Color(floatValues[0], floatValues[1],
                  floatValues[2], floatValues[3]);
  }
}

ConfigNode * ConfigNode::AddChild(
  const std::string & _name, bool _replaceExisting)
{
  if (_replaceExisting)
  {
    ConfigNode * node = FindChild(_name, false);
    if (node)
    {
      return node;
    }
  }
  return new ConfigNode(this, _name);
}

ConfigNode * ConfigNode::FindChild(const std::string & _name, bool _recursive)
{
  int indx, prevC, nextC;
  int childCount = static_cast<int>(mChildren.size());

  if (mLastChildFound != -1)
  {
    // If possible, try checking nodes neighboring the last successful search
    // (often nodes searched for in sequence, so this will std search speeds).
    prevC = mLastChildFound - 1;
    if (prevC < 0)
    {
      prevC = 0;
    }
    else if (prevC >= childCount)
    {
      prevC = childCount - 1;
    }
    nextC = mLastChildFound + 1;
    if (nextC < 0)
    {
      nextC = 0;
    }
    else if (nextC >= childCount)
    {
      nextC = childCount - 1;
    }
    for (indx = prevC; indx <= nextC; ++indx)
    {
      ConfigNode * node = mChildren[indx];
      if (node->mName == _name)
      {
        mLastChildFound = indx;
        return node;
      }
    }

    // If not found that way, search for the node from start to finish,
    //  avoiding the already searched area above.
    for (indx = nextC + 1; indx < childCount; ++indx)
    {
      ConfigNode * node = mChildren[indx];
      if (node->mName == _name)
      {
        mLastChildFound = indx;
        return node;
      }
    }
    for (indx = 0; indx < prevC; ++indx)
    {
      ConfigNode * node = mChildren[indx];
      if (node->mName == _name)
      {
        mLastChildFound = indx;
        return node;
      }
    }
  }
  else
  {
    // Search for the node from start to finish
    for (indx = 0; indx < childCount; ++indx)
    {
      ConfigNode * node = mChildren[indx];
      if (node->mName == _name)
      {
        mLastChildFound = indx;
        return node;
      }
    }
  }

  // If not found, search child nodes (if recursive == true)
  if (_recursive)
  {
    for (indx = 0; indx < childCount; ++indx)
    {
      mChildren[indx]->FindChild(_name, _recursive);
    }
  }

  // Not found anywhere
  return NULL;
}

void ConfigNode::SetParent(ConfigNode * _newParent)
{
  // Remove self from current parent
  mParent->mChildren.erase(iter);

  // Set new parent
  mParent = _newParent;

  // Add self to new parent
  mParent->mChildren.push_back(this);
  iter = --(mParent->mChildren.end());
}
