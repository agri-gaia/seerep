// Copyright (c) 2013, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally from:
// https://github.com/ros/common_msgs/blob/50ee957/sensor_msgs/include/sensor_msgs/impl/point_cloud2_iterator.h

#ifndef seerep_hdf5_fb__IMPL__POINT_CLOUD2_ITERATOR_HPP_
#define seerep_hdf5_fb__IMPL__POINT_CLOUD2_ITERATOR_HPP_

#include <seerep-msgs/point_cloud_2_generated.h>
#include <seerep-msgs/point_field_generated.h>

#include <cstdarg>
#include <sstream>
#include <string>
#include <vector>

namespace seerep_hdf5_fb
{
namespace impl
{
// TODO remove default constructor
template <typename T, typename TT, typename U, typename C, template <typename> class V>
PointCloud2IteratorBase<T, TT, U, C, V>::PointCloud2IteratorBase()
{
}

/** Assignment operator
 * @param iter the iterator to copy data from
 * @return a reference to *this
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, C, V>::operator=(const V<T>& iter)
{
  if (this != &iter)
  {
    point_step_ = iter.point_step_;
    data_char_ = iter.data_char_;
    data_ = iter.data_;
    data_end_ = iter.data_end_;
    is_bigendian_ = iter.is_bigendian_;
  }

  return *this;
}

/** Access the i th element starting at the current pointer (useful when a field has several elements of the same
 * type)
 * @param i
 * @return a reference to the i^th value from the current position
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
TT& PointCloud2IteratorBase<T, TT, U, C, V>::operator[](size_t i) const
{
  return *(data_ + i);
}

/** Dereference the iterator. Equivalent to accessing it through [0]
 * @return the value to which the iterator is pointing
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
TT& PointCloud2IteratorBase<T, TT, U, C, V>::operator*() const
{
  return *data_;
}

/** Increase the iterator to the next element
 * @return a reference to the updated iterator
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, C, V>::operator++()
{
  data_char_ += point_step_;
  data_ = reinterpret_cast<TT*>(data_char_);
  return *static_cast<V<T>*>(this);
}

/** Basic pointer addition
 * @param i the amount to increase the iterator by
 * @return an iterator with an increased position
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
V<T> PointCloud2IteratorBase<T, TT, U, C, V>::operator+(int i)
{
  V<T> res = *static_cast<V<T>*>(this);

  res.data_char_ += i * point_step_;
  res.data_ = reinterpret_cast<TT*>(res.data_char_);

  return res;
}

/** Increase the iterator by a certain amount
 * @return a reference to the updated iterator
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, C, V>::operator+=(int i)
{
  data_char_ += i * point_step_;
  data_ = reinterpret_cast<TT*>(data_char_);
  return *static_cast<V<T>*>(this);
}

/** Compare to another iterator
 * @return whether the current iterator points to a different address than the other one
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
bool PointCloud2IteratorBase<T, TT, U, C, V>::operator!=(const V<T>& iter) const
{
  return iter.data_ != data_;
}

/** Return the end iterator
 * @return the end iterator (useful when performing normal iterator processing with ++)
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
V<T> PointCloud2IteratorBase<T, TT, U, C, V>::end() const
{
  V<T> res = *static_cast<const V<T>*>(this);
  res.data_ = data_end_;
  return res;
}

}  // namespace impl
}  // namespace seerep_hdf5_fb

#endif  // seerep_hdf5_fb__IMPL__POINT_CLOUD2_ITERATOR_HPP_
