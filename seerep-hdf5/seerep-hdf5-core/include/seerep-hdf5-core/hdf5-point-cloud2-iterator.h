// Copyright (c) 2013, Open Source Robotics Foundation, Inc.
// Copyright (c) 2022, Sebastian Pütz, DFKI PBR
// Copyright (c) 2023, Julian Arkenau, DFKI PBR
//
// Source: https://github.com/ros/common_msgs/blob/275b09a/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h
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

#ifndef SEEREP_HDF5_CORE_POINT_CLOUD2_ITERATOR_H
#define SEEREP_HDF5_CORE_POINT_CLOUD2_ITERATOR_H

namespace seerep_hdf5_core
{
namespace impl
{
/** @brief Private base class for PointCloud2Iterator and PointCloud2ConstIterator
 * T is the type of the value on which the child class will be templated
 * TT is the type of the value to be retrieved (same as T except for constness)
 * U is the type of the raw data in PointCloud2 (only uchar and const uchar are supported)
 * V is the derived class (yop, curiously recurring template pattern)
 */
template <typename T, typename TT, typename U, template <typename> class V>
class PointCloud2IteratorBase
{
public:
  PointCloud2IteratorBase() = default;

  /**
   * @brief Assignment operator
   *
   * @param iter the iterator to copy data from
   * @return a reference to *this
   */
  V<T>& operator=(const V<T>& iter);

  /**
   * @brief Access the i th element starting at the current pointer.
   * @param i
   * @return a reference to the i^th value from the current position
   */
  TT& operator[](size_t i) const;

  /**
   * @brief Dereference the iterator.
   *
   * Equivalent to accessing it through [0]
   *
   * @return the value to which the iterator is pointing
   */
  TT& operator*() const;

  /** @brief Increase the iterator to the next element
   *
   *  @return a reference to the updated iterator
   */
  V<T>& operator++();

  /**
   * @brief Basic pointer addition
   *
   * @param i the amount to increase the iterator by
   * @return an iterator with an increased position
   */
  V<T> operator+(int i);

  /**
   * @brief Increase the iterator by a certain amount
   *
   * @return a reference to the updated iterator
   */
  V<T>& operator+=(int i);

  /**
   * @brief Compare to another iterator
   *
   * @return whether the current iterator points to a different address than the other one
   */
  bool operator!=(const V<T>& iter) const;

  /**
   * @brief Return the end iterator
   *
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
  /** Point step of the point cloud */
  uint32_t point_step_;
};
}  // namespace impl

/**
 * @brief Class that can write fields of a point cloud into a byte array
 */
template <typename T>
class PointCloud2Iterator : public impl::PointCloud2IteratorBase<T, T, unsigned char, PointCloud2Iterator>
{
public:
  /**
   * @brief Construct a new PointCloud2Iterator
   *
   * @param data uint8_t pointer to write to
   * @param offset the offset of the field to iterate on
   * @param pointStep the pointStep of the point cloud
   * @param height the height of the point cloud (1 for unorganized point clouds)
   * @param width the width of the point cloud
   */
  PointCloud2Iterator(uint8_t* data, uint32_t offset, uint32_t pointStep, uint32_t height, uint32_t width)
    : impl::PointCloud2IteratorBase<T, T, unsigned char, seerep_hdf5_core::PointCloud2Iterator>::PointCloud2IteratorBase()
  {
    this->point_step_ = pointStep;
    this->data_char_ = reinterpret_cast<unsigned char*>(data + offset);
    this->data_ = reinterpret_cast<T*>(this->data_char_);
    // last element is at: offset + (n - 1) * pointStep
    this->data_end_ = reinterpret_cast<T*>(data + offset + (height * width - 1) * pointStep);
  }
};

/**
 * @brief Class that can read fields from a byte array
 */
template <typename T>
class PointCloud2ConstIterator
  : public impl::PointCloud2IteratorBase<T, const T, const unsigned char, PointCloud2ConstIterator>
{
public:
  /**
   * @brief Construct a new PointCloud2ConstIterator
   *
   * @param data uint8_t pointer to write from
   * @param offset the offset of the field to iterate on
   * @param pointStep the pointStep of the point cloud
   * @param height the height of the point cloud (1 for unorganized point clouds)
   * @param width the width of the point cloud
   */
  PointCloud2ConstIterator(const uint8_t* data, uint32_t offset, uint32_t pointStep, uint32_t height, uint32_t width)
    : impl::PointCloud2IteratorBase<T, const T, const unsigned char,
                                    seerep_hdf5_core::PointCloud2ConstIterator>::PointCloud2IteratorBase()
  {
    this->point_step_ = pointStep;
    this->data_char_ = reinterpret_cast<const unsigned char*>(data + offset);
    this->data_ = reinterpret_cast<const T*>(this->data_char_);
    // last element is at: offset + (n -1) * pointStep
    this->data_end_ = reinterpret_cast<const T*>(data + offset + (height * width - 1) * pointStep);
  }
};
}  // namespace seerep_hdf5_core

#include "impl/hdf5-point-cloud2-iterator.hpp"  // NOLINT

#endif  // SEEREP_HDF5_CORE_POINT_CLOUD2_ITERATOR_H
