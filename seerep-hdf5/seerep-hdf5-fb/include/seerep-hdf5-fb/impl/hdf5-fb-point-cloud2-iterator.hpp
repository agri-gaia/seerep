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

// namespace
// {
// /** Return the size of a datatype (which is an enum of seerep::PointField::) in bytes
//  * @param datatype one of the enums of seerep::PointField::
//  */
// inline int sizeOfPointField(int datatype)
// {
//   switch (datatype)
//   {
//     case seerep::fb::Point_Field_Datatype_INT8:
//     case seerep::fb::Point_Field_Datatype_UINT8:
//       return 1;
//     case seerep::fb::Point_Field_Datatype_INT16:
//     case seerep::fb::Point_Field_Datatype_UINT16:
//       return 2;
//     case seerep::fb::Point_Field_Datatype_INT32:
//     case seerep::fb::Point_Field_Datatype_UINT32:
//     case seerep::fb::Point_Field_Datatype_FLOAT32:
//       return 4;
//     case seerep::fb::Point_Field_Datatype_FLOAT64:
//       return 8;

//     default:
//       std::stringstream err;
//       err << "PointField of type " << datatype << " does not exist" << std::endl;
//       throw std::runtime_error(err.str());
//       return -1;
//   }
// }

// /** Private function that adds a PointField to the "fields" member of a PointCloud2
//  * @param cloud_msg the PointCloud2 to add a field to
//  * @param name the name of the field
//  * @param count the number of elements in the PointField
//  * @param datatype the datatype of the elements
//  * @param offset the offset of that element
//  * @return the offset of the next PointField that will be added to the PointCloud2
//  */
// inline int addPointField(seerep::fb::PointCloud2& cloud_msg, const std::string& name, int count, int datatype,
//                          int offset)
// {
//   // ToDO add point_field to point cloud if it's needed

//   // Update the offset
//   return offset + count * sizeOfPointField(datatype);
// }
// }  // namespace

namespace seerep_hdf5_fb
{
namespace impl
{
template <typename T, typename TT, typename U, template <typename> class V>
PointCloud2IteratorBase<T, TT, U, V>::PointCloud2IteratorBase()
{
}

/** Assignment operator
 * @param iter the iterator to copy data from
 * @return a reference to *this
 */
template <typename T, typename TT, typename U, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, V>::operator=(const V<T>& iter)
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
template <typename T, typename TT, typename U, template <typename> class V>
TT& PointCloud2IteratorBase<T, TT, U, V>::operator[](size_t i) const
{
  return *(data_ + i);
}

/** Dereference the iterator. Equivalent to accessing it through [0]
 * @return the value to which the iterator is pointing
 */
template <typename T, typename TT, typename U, template <typename> class V>
TT& PointCloud2IteratorBase<T, TT, U, V>::operator*() const
{
  return *data_;
}

/** Increase the iterator to the next element
 * @return a reference to the updated iterator
 */
template <typename T, typename TT, typename U, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, V>::operator++()
{
  data_char_ += point_step_;
  data_ = reinterpret_cast<TT*>(data_char_);
  return *static_cast<V<T>*>(this);
}

/** Basic pointer addition
 * @param i the amount to increase the iterator by
 * @return an iterator with an increased position
 */
template <typename T, typename TT, typename U, template <typename> class V>
V<T> PointCloud2IteratorBase<T, TT, U, V>::operator+(int i)
{
  V<T> res = *static_cast<V<T>*>(this);

  res.data_char_ += i * point_step_;
  res.data_ = reinterpret_cast<TT*>(res.data_char_);

  return res;
}

/** Increase the iterator by a certain amount
 * @return a reference to the updated iterator
 */
template <typename T, typename TT, typename U, template <typename> class V>
V<T>& PointCloud2IteratorBase<T, TT, U, V>::operator+=(int i)
{
  data_char_ += i * point_step_;
  data_ = reinterpret_cast<TT*>(data_char_);
  return *static_cast<V<T>*>(this);
}

/** Compare to another iterator
 * @return whether the current iterator points to a different address than the other one
 */
template <typename T, typename TT, typename U, template <typename> class V>
bool PointCloud2IteratorBase<T, TT, U, V>::operator!=(const V<T>& iter) const
{
  return iter.data_ != data_;
}

/** Return the end iterator
 * @return the end iterator (useful when performing normal iterator processing with ++)
 */
template <typename T, typename TT, typename U, template <typename> class V>
V<T> PointCloud2IteratorBase<T, TT, U, V>::end() const
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
template <typename T, typename TT, typename U, template <typename> class V>
uint32_t PointCloud2IteratorBase<T, TT, U, V>::set_field(const seerep::fb::PointCloud2& cloud_msg,
                                                         const std::string& field_name)
{
  is_bigendian_ = cloud_msg.is_bigendian();
  point_step_ = cloud_msg.point_step();
  // make sure the channel is valid

  for (size_t i = 0; i < cloud_msg.fields()->size(); i++)
  {
    const seerep::fb::PointField& field = *cloud_msg.fields()->Get(i);
    if (field.name()->str() == field_name)
    {
      return field.offset();
    }
  }

  // // Handle the special case of r,g,b,a (we assume they are understood as the
  // // channels of an rgb or rgba field)

  std::set<std::string> field_names = { "r", "g", "b", "a" };

  if (field_names.find(field_name) != field_names.end())
  {
    for (size_t i = 0; i < cloud_msg.fields()->size(); i++)
    {
      const seerep::fb::PointField& field = *cloud_msg.fields()->Get(i);

      if (field.name()->str() == "rgb" || field.name()->str() == "rgba")
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
}  // namespace seerep_hdf5_fb

#endif  // seerep_hdf5_fb__IMPL__POINT_CLOUD2_ITERATOR_HPP_
