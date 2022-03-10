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

#ifndef seerep_io_pb__IMPL__POINT_CLOUD2_ITERATOR_HPP_
#define seerep_io_pb__IMPL__POINT_CLOUD2_ITERATOR_HPP_

#include <seerep-msgs/point_cloud_2.pb.h>
#include <seerep-msgs/point_field.pb.h>
#include <cstdarg>
#include <sstream>
#include <string>
#include <vector>

namespace
{
/** Return the size of a datatype (which is an enum of seerep::PointField::) in bytes
 * @param datatype one of the enums of seerep::PointField::
 */
inline int sizeOfPointField(int datatype)
{
  switch (datatype)
  {
    case seerep::PointField_Datatype_INT8:
    case seerep::PointField_Datatype_UINT8:
      return 1;
    case seerep::PointField_Datatype_INT16:
    case seerep::PointField_Datatype_UINT16:
      return 2;
    case seerep::PointField_Datatype_INT32:
    case seerep::PointField_Datatype_UINT32:
    case seerep::PointField_Datatype_FLOAT32:
      return 4;
    case seerep::PointField_Datatype_FLOAT64:
      return 8;

    default:
      std::stringstream err;
      err << "PointField of type " << datatype << " does not exist" << std::endl;
      throw std::runtime_error(err.str());
      return -1;
  }
}

/** Private function that adds a PointField to the "fields" member of a PointCloud2
 * @param cloud_msg the PointCloud2 to add a field to
 * @param name the name of the field
 * @param count the number of elements in the PointField
 * @param datatype the datatype of the elements
 * @param offset the offset of that element
 * @return the offset of the next PointField that will be added to the PointCloud2
 */
inline int addPointField(seerep::PointCloud2& cloud_msg, const std::string& name, int count, int datatype, int offset)
{
  seerep::PointField* field_ptr = cloud_msg.add_fields();

  field_ptr->set_name(name);
  field_ptr->set_count(count);
  field_ptr->set_datatype(seerep::PointField_Datatype(datatype));
  field_ptr->set_offset(offset);

  // Update the offset
  return offset + count * sizeOfPointField(datatype);
}
}  // namespace

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace seerep_io_pb
{
inline PointCloud2Modifier::PointCloud2Modifier(seerep::PointCloud2& cloud_msg) : cloud_msg_(cloud_msg)
{
}

inline size_t PointCloud2Modifier::size() const
{
  return cloud_msg_.data().size() / cloud_msg_.point_step();
}

inline void PointCloud2Modifier::reserve(size_t size)
{
  cloud_msg_.mutable_data()->reserve(size * cloud_msg_.point_step());
}

inline void PointCloud2Modifier::resize(size_t size)
{
  cloud_msg_.mutable_data()->resize(size * cloud_msg_.point_step());

  // Update height/width
  if (cloud_msg_.height() == 1)
  {
    cloud_msg_.set_width(size);
    cloud_msg_.set_row_step(size * cloud_msg_.point_step());
  }
  else
  {
    if (cloud_msg_.width() == 1)
    {
      cloud_msg_.set_height(static_cast<uint32_t>(size));
    }
    else
    {
      cloud_msg_.set_height(1);
      cloud_msg_.set_width(size);
      cloud_msg_.set_row_step(size * cloud_msg_.point_step());
    }
  }
}

inline void PointCloud2Modifier::clear()
{
  cloud_msg_.mutable_data()->clear();

  // Update height/width
  if (cloud_msg_.height() == 1)
  {
    cloud_msg_.clear_row_step();
    cloud_msg_.clear_width();
  }
  else
  {
    if (cloud_msg_.width() == 1)
    {
      cloud_msg_.clear_height();
    }
    else
    {
      cloud_msg_.clear_row_step();
      cloud_msg_.clear_width();
      cloud_msg_.clear_height();
    }
  }
}

inline void PointCloud2Modifier::setPointCloud2Fields(int n_fields, ...)
{
  cloud_msg_.clear_fields();
  va_list vl;
  va_start(vl, n_fields);
  int offset = 0;
  for (int i = 0; i < n_fields; ++i)
  {
    // Create the corresponding PointField
    std::string name(va_arg(vl, char*));
    int count(va_arg(vl, int));
    int datatype(va_arg(vl, int));
    offset = addPointField(cloud_msg_, name, count, datatype, offset);
  }
  va_end(vl);

  // Resize the point cloud accordingly
  cloud_msg_.set_point_step(offset);
  cloud_msg_.set_row_step(cloud_msg_.width() * cloud_msg_.point_step());
  cloud_msg_.mutable_data()->resize(cloud_msg_.height() * cloud_msg_.row_step());
}

inline void PointCloud2Modifier::setPointCloud2FieldsByString(int n_fields, ...)
{
  cloud_msg_.clear_fields();
  va_list vl;
  va_start(vl, n_fields);
  int offset = 0;
  for (int i = 0; i < n_fields; ++i)
  {
    // Create the corresponding PointFields
    std::string field_name = std::string(va_arg(vl, char*));
    if (field_name == "xyz")
    {
      // Do x, y and z
      offset = addPointField(cloud_msg_, "x", 1, seerep::PointField_Datatype_FLOAT32, offset);
      offset = addPointField(cloud_msg_, "y", 1, seerep::PointField_Datatype_FLOAT32, offset);
      offset = addPointField(cloud_msg_, "z", 1, seerep::PointField_Datatype_FLOAT32, offset);
      offset += sizeOfPointField(seerep::PointField_Datatype_FLOAT32);
    }
    else
    {
      if ((field_name == "rgb") || (field_name == "rgba"))
      {
        offset = addPointField(cloud_msg_, field_name, 1, seerep::PointField_Datatype_FLOAT32, offset);
        offset += 3 * sizeOfPointField(seerep::PointField_Datatype_FLOAT32);
      }
      else
      {
        va_end(vl);
        throw std::runtime_error("Field " + field_name + " does not exist!");
      }
    }
  }
  va_end(vl);

  // Resize the point cloud accordingly
  cloud_msg_.set_point_step(offset);
  cloud_msg_.set_row_step(cloud_msg_.width() * cloud_msg_.point_step());
  cloud_msg_.mutable_data()->resize(cloud_msg_.height() * cloud_msg_.row_step());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace impl
{
/**
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
PointCloud2IteratorBase<T, TT, U, C, V>::PointCloud2IteratorBase() : data_char_(0), data_(0), data_end_(0)
{
}

/**
 * @param cloud_msg The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
PointCloud2IteratorBase<T, TT, U, C, V>::PointCloud2IteratorBase(C& cloud_msg, const std::string& field_name)
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

/** Common code to set the field of the PointCloud2
 * @param cloud_msg the PointCloud2 to modify
 * @param field_name the name of the field to iterate upon
 * @return the offset at which the field is found
 */
template <typename T, typename TT, typename U, typename C, template <typename> class V>
int PointCloud2IteratorBase<T, TT, U, C, V>::set_field(const seerep::PointCloud2& cloud_msg,
                                                       const std::string& field_name)
{
  is_bigendian_ = cloud_msg.is_bigendian();
  point_step_ = cloud_msg.point_step();
  // make sure the channel is valid

  for (int i = 0; i < cloud_msg.fields_size(); i++)
  {
    const seerep::PointField& field = cloud_msg.fields(i);
    if (field.name() == field_name)
    {
      return field.offset();
    }
  }

  // Handle the special case of r,g,b,a (we assume they are understood as the
  // channels of an rgb or rgba field)

  std::set<std::string> field_names = { "r", "g", "b", "a" };

  if (field_names.find(field_name) != field_names.end())
  {
    for (int i = 0; i < cloud_msg.fields_size(); i++)
    {
      const seerep::PointField& field = cloud_msg.fields(i);

      if (field.name() == "rgb" || field.name() == "rgba")
      {
        if (field_name == "r")
        {
          if (is_bigendian_)
          {
            return field.offset() + 1;
          }
          else
          {
            return field.offset() + 2;
          }
        }
        if (field_name == "g")
        {
          if (is_bigendian_)
          {
            return field.offset() + 2;
          }
          else
          {
            return field.offset() + 1;
          }
        }
        if (field_name == "b")
        {
          if (is_bigendian_)
          {
            return field.offset() + 3;
          }
          else
          {
            return field.offset() + 0;
          }
        }
        if (field_name == "a")
        {
          if (is_bigendian_)
          {
            return field.offset() + 0;
          }
          else
          {
            return field.offset() + 3;
          }
        }
      }
    }
  }
  throw std::runtime_error("Field " + field_name + " does not exist!");
}

}  // namespace impl
}  // namespace seerep_io_pb

#endif  // SENSOR_MSGS__IMPL__POINT_CLOUD2_ITERATOR_HPP_
