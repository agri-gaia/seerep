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
// https://github.com/ros/common_msgs/blob/275b09a/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h

#ifndef SENSOR_MSGS__POINT_CLOUD2_ITERATOR_FB_HPP_
#define SENSOR_MSGS__POINT_CLOUD2_ITERATOR_FB_HPP_

#include <flatbuffers/reflection.h>
#include <seerep-msgs/point_cloud_2_generated.h>
#include <seerep-msgs/point_field_generated.h>

#include <string>
#include <vector>

namespace seerep_hdf5_fb
{
namespace impl
{
/** Private base class for PointCloud2Iterator and PointCloud2ConstIterator
 * T is the type of the value on which the child class will be templated
 * TT is the type of the value to be retrieved (same as T except for constness)
 * U is the type of the raw data in PointCloud2 (only uchar and const uchar are supported)
 * V is the derived class (yop, curiously recurring template pattern)
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
class PointCloud2IteratorBase
{
public:
  PointCloud2IteratorBase();

  /** Assignment operator
   * @param iter the iterator to copy data from
   * @return a reference to *this
   */
  V<T>& operator=(const V<T>& iter);

  /** Access the i th element starting at the current pointer (useful when a field has several elements of the same
   * type)
   * @param i
   * @return a reference to the i^th value from the current position
   */
  TT& operator[](size_t i) const;

  /** Dereference the iterator. Equivalent to accessing it through [0]
   * @return the value to which the iterator is pointing
   */
  TT& operator*() const;

  /** Increase the iterator to the next element
   * @return a reference to the updated iterator
   */
  V<T>& operator++();

  /** Basic pointer addition
   * @param i the amount to increase the iterator by
   * @return an iterator with an increased position
   */
  V<T> operator+(int i);

  /** Increase the iterator by a certain amount
   * @return a reference to the updated iterator
   */
  V<T>& operator+=(int i);

  /** Compare to another iterator
   * @return whether the current iterator points to a different address than the other one
   */
  bool operator!=(const V<T>& iter) const;

  /** Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++)
   */
  V<T> end() const;

  /** The raw data in uchar* where the iterator is */
  U* data_char_;
  /** The cast data where the iterator is */
  TT* data_;
  /** The end() pointer of the iterator */
  TT* data_end_;
  /** Whether the fields are stored as bigendian */
  bool is_bigendian_;
  /** The "point_step" of the point cloud */
  uint32_t point_step_;
  /** Offset of the field to iterate on */
  uint32_t field_offset_;
};
}  // namespace impl

/**
 * @brief Class that can iterate over a flatbuffers PointCloud2 (used for writing into the data field)
 */
template <typename T>
class PointCloud2Iterator
  : public impl::PointCloud2IteratorBase<T, T, unsigned char, seerep::fb::PointCloud2, PointCloud2Iterator>
{
public:
  /**
   * @brief Construct a new PointCloud2Iterator
   *
   * @param data pointer to a uint8_t array to iterate over
   * @param offset the offset of the field to iterate on
   * @param pointStep the pointStep of the point cloud
   * @param height the height of the point cloud (1 for unorganized point clouds)
   * @param width the width of the point cloud
   */
  PointCloud2Iterator(uint8_t* data, uint32_t offset, uint32_t pointStep, uint32_t height, uint32_t width)
    : impl::PointCloud2IteratorBase<T, T, unsigned char, seerep::fb::PointCloud2,
                                    seerep_hdf5_fb::PointCloud2Iterator>::PointCloud2IteratorBase()
  {
    this->field_offset_ = offset;
    this->point_step_ = pointStep;
    this->data_char_ = reinterpret_cast<unsigned char*>(data + this->field_offset_);
    this->data_ = reinterpret_cast<T*>(this->data_char_);
    // last element is at: offset + (n -1) * pointStep
    this->data_end_ = reinterpret_cast<T*>(data + offset + (height * width - 1) * pointStep);
  }
};

/**
 * @brief Class that can iterate over a flatbuffers PointCloud2 (used for reading the data field)
 */
template <typename T>
class PointCloud2ConstIterator
  : public impl::PointCloud2IteratorBase<T, const T, const unsigned char, const seerep::fb::PointCloud2,
                                         PointCloud2ConstIterator>
{
public:
  // TODO same parameter order as above
  /**
   * @brief Construct a new PointCloud2ConstIterator
   *
   * @param data pointer to a const uint8_t array to iterate over
   * @param offset the offset of the field to iterate on
   * @param height the height of the point cloud (1 for unorganized point clouds)
   * @param width the width of the point cloud
   * @param pointStep the pointStep of the point cloud
   */
  PointCloud2ConstIterator(const uint8_t* data, uint32_t offset, uint32_t height, uint32_t width, uint32_t pointStep)
    : impl::PointCloud2IteratorBase<T, const T, const unsigned char, const seerep::fb::PointCloud2,
                                    seerep_hdf5_fb::PointCloud2ConstIterator>::PointCloud2IteratorBase()
  {
    this->field_offset_ = offset;
    this->point_step_ = pointStep;
    this->data_char_ = reinterpret_cast<const unsigned char*>(data + this->field_offset_);
    this->data_ = reinterpret_cast<const T*>(this->data_char_);
    // last element is at: offset + (n -1) * pointStep
    this->data_end_ = reinterpret_cast<const T*>(data + offset + (height * width - 1) * pointStep);
  }
};
}  // namespace seerep_hdf5_fb

#include "impl/hdf5-fb-point-cloud2-iterator.hpp"  // NOLINT

#endif  // SENSOR_MSGS__POINT_CLOUD2_ITERATOR_FB_HPP_
