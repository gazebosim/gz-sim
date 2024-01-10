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

void ConfigLoader::loadMaterialFiles(ConfigLoader * c)
{
  try
  {
    std::string installedConfig = common::joinPaths(
        gz::sim::getMediaInstallDir(),
        "gazebo.material");
    std::ifstream in(installedConfig, std::ios::binary);
    c->parseScript(in);
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
  for (auto & i : m_scriptList) {
    delete i.second;
  }
  m_scriptList.clear();
}

ConfigNode * ConfigLoader::getConfigScript(const std::string & name)
{
  auto i = m_scriptList.find(name);
  // If found..
  if (i != m_scriptList.end()) {
    return i->second;
  } else {
    return nullptr;
  }
}

std::map<std::string, ConfigNode *> ConfigLoader::getAllConfigScripts()
{
  return m_scriptList;
}

void ConfigLoader::parseScript(std::ifstream & stream)
{
  // Get first token
  _nextToken(stream);
  if (tok == TOKEN_EOF) {
    stream.close();
    return;
  }

  // Parse the script
  _parseNodes(stream, 0);

  stream.close();
}

void ConfigLoader::_nextToken(std::ifstream & stream)
{
  lastTok = tok;
  lastTokVal = tokVal;

  // EOF token
  if (stream.eof()) {
    tok = TOKEN_EOF;
    return;
  }

  // Get next character
  int ch = stream.get();
  if (ch == -1) {
    tok = TOKEN_EOF;
    return;
  }
  // Skip leading spaces / tabs
  while ((ch == ' ' || ch == 9) && !stream.eof()) {
    ch = stream.get();
  }

  if (stream.eof()) {
    tok = TOKEN_EOF;
    return;
  }

  // Newline token
  if (ch == '\r' || ch == '\n') {
    do{
      ch = stream.get();
    } while ((ch == '\r' || ch == '\n') && !stream.eof());

    stream.unget();

    tok = TOKEN_NewLine;
    return;
  } else if (ch == '{') {
    // Open brace token
    tok = TOKEN_OpenBrace;
    return;
  } else if (ch == '}') {
    // Close brace token
    tok = TOKEN_CloseBrace;
    return;
  }

  // Text token
  if (ch < 32 || ch > 122) {  // Verify valid char
    throw std::runtime_error(
      "Parse Error: Invalid character, ConfigLoader::load()");
  }

  tokVal = "";
  tok = TOKEN_Text;
  do{
    // Skip comments
    if (ch == '/') {
      int ch2 = stream.peek();

      // C++ style comment (//)
      if (ch2 == '/') {
        stream.get();
        do{
          ch = stream.get();
        } while (ch != '\r' && ch != '\n' && !stream.eof());

        tok = TOKEN_NewLine;
        return;
      } else if (ch2 == '*') {
        stream.get();
        do{
          ch = stream.get();
          ch2 = stream.peek();
        } while (!(ch == '*' && ch2 == '/') && !stream.eof());
        stream.get();
        do{
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

void ConfigLoader::_skipNewLines(std::ifstream & stream)
{
  while (tok == TOKEN_NewLine) {
    _nextToken(stream);
  }
}

void ConfigLoader::_parseNodes(std::ifstream & stream, ConfigNode * parent)
{
  typedef std::pair < std::string, ConfigNode * > ScriptItem;

  while (true) {
    switch (tok) {
      // Node
      case TOKEN_Text:
        // Add the new node
        ConfigNode * newNode;
        if (parent) {
          newNode = parent->addChild(tokVal);
        } else {
          newNode = new ConfigNode(0, tokVal);
        }

        // Get values
        _nextToken(stream);
        while (tok == TOKEN_Text) {
          newNode->addValue(tokVal);
          _nextToken(stream);
        }

        // Add root nodes to scriptList
        if (!parent) {
          std::string key;

          if (newNode->getValues().empty()) {
            key = newNode->getName() + ' ';
          } else {
            key = newNode->getName() + ' ' + newNode->getValues().front();
          }

          m_scriptList.insert(ScriptItem(key, newNode));
        }

        _skipNewLines(stream);

        // Add any sub-nodes
        if (tok == TOKEN_OpenBrace) {
          // Parse nodes
          _nextToken(stream);
          _parseNodes(stream, newNode);
          // Check for matching closing brace
          if (tok != TOKEN_CloseBrace) {
            throw std::runtime_error("Parse Error: Expecting closing brace");
          }
          _nextToken(stream);
          _skipNewLines(stream);
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
        _nextToken(stream);
        break;

      default:
        break;
    }
  }
}

ConfigNode::ConfigNode(ConfigNode * parent, const std::string & name)
{
  m_name = name;
  m_parent = parent;
  _removeSelf = true;  // For proper destruction
  m_lastChildFound = -1;

  // Add self to parent's child list
  // (unless this is the root node being created)
  if (parent != NULL) {
    m_parent->m_children.push_back(this);
    _iter = --(m_parent->m_children.end());
  }
}

ConfigNode::~ConfigNode()
{
  // Delete all children
  std::vector < ConfigNode * > ::iterator i;
  for (i = m_children.begin(); i != m_children.end(); i++) {
    ConfigNode * node = *i;
    node->_removeSelf = false;
    delete node;
  }
  m_children.clear();

  // Remove self from parent's child list
  if (_removeSelf && m_parent != NULL) {
    m_parent->m_children.erase(_iter);
  }
}

void ConfigNode::getColorValues(gz::math::Color & colorValues,
  unsigned int size)
{
  std::vector<float> floatValues;
  ConfigNode::getValuesInFloat(floatValues);
  if (floatValues.size() < size) {
    gzerr << "Bad material file." << std::endl;
    floatValues.resize(size);
  }
  if (size == 3) {
    colorValues = gz::math::Color(floatValues[0], floatValues[1],
                  floatValues[2]);
  }
  if (size == 4) {
    colorValues = gz::math::Color(floatValues[0], floatValues[1],
                  floatValues[2], floatValues[3]);
  }
}

ConfigNode * ConfigNode::addChild(
  const std::string & name, bool replaceExisting)
{
  if (replaceExisting) {
    ConfigNode * node = findChild(name, false);
    if (node) {
      return node;
    }
  }
  return new ConfigNode(this, name);
}

ConfigNode * ConfigNode::findChild(const std::string & name, bool recursive)
{
  int indx, prevC, nextC;
  int childCount = static_cast<int>(m_children.size());

  if (m_lastChildFound != -1) {
    // If possible, try checking nodes neighboring the last successful search
    // (often nodes searched for in sequence, so this will std search speeds).
    prevC = m_lastChildFound - 1;
    if (prevC < 0) {
      prevC = 0;
    } else if (prevC >= childCount) {
      prevC = childCount - 1;
    }
    nextC = m_lastChildFound + 1;
    if (nextC < 0) {
      nextC = 0;
    } else if (nextC >= childCount) {
      nextC = childCount - 1;
    }
    for (indx = prevC; indx <= nextC; ++indx) {
      ConfigNode * node = m_children[indx];
      if (node->m_name == name) {
        m_lastChildFound = indx;
        return node;
      }
    }

    // If not found that way, search for the node from start to finish,
    //  avoiding the already searched area above.
    for (indx = nextC + 1; indx < childCount; ++indx) {
      ConfigNode * node = m_children[indx];
      if (node->m_name == name) {
        m_lastChildFound = indx;
        return node;
      }
    }
    for (indx = 0; indx < prevC; ++indx) {
      ConfigNode * node = m_children[indx];
      if (node->m_name == name) {
        m_lastChildFound = indx;
        return node;
      }
    }
  } else {
    // Search for the node from start to finish
    for (indx = 0; indx < childCount; ++indx) {
      ConfigNode * node = m_children[indx];
      if (node->m_name == name) {
        m_lastChildFound = indx;
        return node;
      }
    }
  }

  // If not found, search child nodes (if recursive == true)
  if (recursive) {
    for (indx = 0; indx < childCount; ++indx) {
      m_children[indx]->findChild(name, recursive);
    }
  }

  // Not found anywhere
  return NULL;
}

void ConfigNode::setParent(ConfigNode * newParent)
{
  // Remove self from current parent
  m_parent->m_children.erase(_iter);

  // Set new parent
  m_parent = newParent;

  // Add self to new parent
  m_parent->m_children.push_back(this);
  _iter = --(m_parent->m_children.end());
}
