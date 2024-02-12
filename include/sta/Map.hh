// OpenSTA, Static Timing Analyzer
// Copyright (c) 2023, Parallax Software, Inc.
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <map>
#include <algorithm>

#include <mutex>
#include <shared_mutex>

namespace sta {

// Add convenience functions around STL container.
template <class KEY, class VALUE, class CMP = std::less<KEY> >
class Map : public std::map<KEY, VALUE, CMP>
{
public:
  Map() :
    std::map<KEY, VALUE, CMP>()
  {
  }
  explicit Map(const CMP &cmp) :
    std::map<KEY, VALUE, CMP>(cmp)
  {
  }

  // Find out if key is in the set.
  bool
  hasKey(const KEY key) const
  {
    return this->find(key) != this->end();
  }

  // Find the value corresponding to key.
  VALUE
  findKey(const KEY key) const
  {
    auto find_iter = this->find(key);
    if (find_iter != this->end())
      return find_iter->second;
    else
      return nullptr;
  }
  void
  findKey(const KEY key,
	  // Return Values.
	  VALUE &value,
	  bool &exists) const
  {
    auto find_iter = this->find(key);
    if (find_iter != this->end()) {
      value = find_iter->second;
      exists = true;
    }
    else
      exists = false;
  }
  void
  findKey(const KEY &key,
	  // Return Values.
	  KEY &map_key,
	  VALUE &value,
	  bool &exists) const
  {
    auto find_iter = this->find(key);
    if (find_iter != this->end()) {
      map_key = find_iter->first;
      value = find_iter->second;
      exists = true;
    }
    else
      exists = false;
  }

  void
  insert(const KEY &key,
	 VALUE value)
  {
    this->operator[](key) = value;
  }

  void
  deleteContents()
  {
    Iterator iter(this);
    while (iter.hasNext())
      delete iter.next();
  }

  void
  deleteKeysContents()
  {
    for (auto key_value : this) {
      KEY key = key_value.first;
      VALUE value = key_value.second;
      delete key;
      delete value;
    }
  }

  void
  deleteArrayContents()
  {
    Iterator iter(this);
    while (iter.hasNext())
      delete [] iter.next();
  }

  void
  deleteContentsClear()
  {
    deleteContents();
    std::map<KEY, VALUE, CMP>::clear();
  }

  // Java style container itererator
  //  Map::Iterator<string *, Value, stringLess> iter(map);
  //  while (iter.hasNext()) {
  //    Value *v = iter.next();
  //  }
  class Iterator
  {
  public:
    Iterator() : container_(nullptr) {}
    explicit Iterator(std::map<KEY, VALUE, CMP> *container) :
      container_(container)
    { if (container_ != nullptr) iter_ = container_->begin(); }
    explicit Iterator(std::map<KEY, VALUE, CMP> &container) :
      container_(&container)
    { if (container_ != nullptr) iter_ = container_->begin(); }
    void init(std::map<KEY, VALUE, CMP> *container)
    { container_ = container; if (container_ != nullptr) iter_=container_->begin();}
    void init(std::map<KEY, VALUE, CMP> &container)
    { container_ = &container; if (container_ != nullptr) iter_=container_->begin();}
    bool hasNext() { return container_ != nullptr && iter_ != container_->end(); }
    VALUE next() { return iter_++->second; }
    void next(KEY &key,
	      VALUE &value)
    { key = iter_->first; value = iter_->second; iter_++; }
    std::map<KEY, VALUE, CMP> *container() { return container_; }

  private:
    std::map<KEY, VALUE, CMP> *container_;
    typename std::map<KEY, VALUE, CMP>::iterator iter_;
  };

  class ConstIterator
  {
  public:
    ConstIterator() : container_(nullptr) {}
    explicit ConstIterator(const std::map<KEY, VALUE, CMP> *container) :
      container_(container)
    { if (container_ != nullptr) iter_ = container_->begin(); }
    explicit ConstIterator(const std::map<KEY, VALUE, CMP> &container) :
      container_(&container)
    { if (container_ != nullptr) iter_ = container_->begin(); }
    void init(const std::map<KEY, VALUE, CMP> *container)
    { container_ = container; if (container_ != nullptr) iter_=container_->begin();}
    void init(const std::map<KEY, VALUE, CMP> &container)
    { container_ = &container; if (container_ != nullptr) iter_=container_->begin();}
    bool hasNext() { return container_ != nullptr && iter_ != container_->end(); }
    VALUE next() { return iter_++->second; }
    void next(KEY &key,
	      VALUE &value)
    { key = iter_->first; value = iter_->second; iter_++; }
    const std::map<KEY, VALUE, CMP> *container() { return container_; }

  private:
    const std::map<KEY, VALUE, CMP> *container_;
    typename std::map<KEY, VALUE, CMP>::const_iterator iter_;
  };
};

template <class KEY, class VALUE, class CMP = std::less<KEY>>
class ConcurrentMap : private Map<KEY, VALUE, CMP>
{
  std::shared_mutex mutex_;

  using Unsafe = Map<KEY, VALUE, CMP>;

public:
  using Unsafe::Map;

  // Write methods (need unique_lock)
  void
  insert(const KEY& key, VALUE value)
  {
    std::unique_lock lock(mutex_);
    Unsafe::operator[](key) = value;
  }

  void
  clear()
  {
    std::unique_lock lock(mutex_);
    Unsafe::clear();
  }

  // Read methods (shared_lock suffices)
  const VALUE&
  operator[](const KEY& key)
  {
    std::shared_lock lock(mutex_);
    return Unsafe::operator[](key);
  }

  size_t
  size()
  {
    std::shared_lock lock(mutex_);
    return Unsafe::size();
  }

  VALUE
  findKey(const KEY key)
  {
    std::shared_lock lock(mutex_);
    return Unsafe::findKey(key);
  }

  Unsafe::iterator begin()
  {
    std::shared_lock lock(mutex_);
    return Unsafe::begin();
  }

  Unsafe::iterator end()
  {
    std::shared_lock lock(mutex_);
    return Unsafe::end();
  }

  // Iterator

  // Derived methods (safe methods calls, no explicit locks needed)
  bool
  empty()
  {
    return size() == 0;
  }
};

} // namespace
