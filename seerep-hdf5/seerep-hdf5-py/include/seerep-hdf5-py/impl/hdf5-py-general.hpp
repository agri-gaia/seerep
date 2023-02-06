
namespace seerep_hdf5_py
{
template <int NumDimensions>
void Hdf5PyGeneral::writeBoundingBoxLabeled(const std::string& data_group_id,
                                            const std::vector<CategorizedBoundingBoxLabel<NumDimensions>>& bb_labels)
{
  for (auto& category_labels : bb_labels)
  {
    if (!category_labels.labels.empty())
    {
      std::vector<std::string> labels;
      std::vector<std::string> instance_uuids;
      std::vector<std::vector<double>> bounding_boxes;

      labels.reserve(category_labels.labels.size());
      instance_uuids.reserve(category_labels.labels.size());
      bounding_boxes.reserve(category_labels.labels.size());

      for (auto& instance_label : category_labels.labels)
      {
        labels.push_back(instance_label.label.label);
        instance_uuids.push_back(instance_label.label.instance_uuid);

        std::vector<double> box;
        box.reserve(2 * NumDimensions);

        for (int i = 0; i < NumDimensions; i++)
        {
          box.push_back(instance_label.min_point[i]);
        }
        for (int i = 0; i < NumDimensions; i++)
        {
          box.push_back(instance_label.max_point[i]);
        }

        bounding_boxes.push_back(box);
      }

      HighFive::DataSpace labels_data_space = HighFive::DataSpace::From(labels);
      auto dataset_labels = getHdf5DataSet<std::string>(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + category_labels.category,
          labels_data_space);
      dataset_labels->write(labels);

      HighFive::DataSpace instances_data_space = HighFive::DataSpace::From(instance_uuids);
      auto dataset_instances = getHdf5DataSet<std::string>(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + category_labels.category,
          instances_data_space);
      dataset_instances->write(instance_uuids);

      HighFive::DataSpace boxes_data_space = HighFive::DataSpace::From(bounding_boxes);
      auto dataset_boxes = getHdf5DataSet<double>(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES + "_" + category_labels.category,
          boxes_data_space);
      dataset_boxes->write(bounding_boxes);
    }
  }
  m_file->flush();
}

template <int NumDimensions>
std::vector<CategorizedBoundingBoxLabel<NumDimensions>>
Hdf5PyGeneral::readBoundingBoxLabeled(const std::string& data_group_id)
{
  auto hdf_group_ptr = getHdf5Group(data_group_id);

  std::vector<CategorizedBoundingBoxLabel<NumDimensions>> bb_labels;

  const std::vector<std::string> group_datasets = hdf_group_ptr->listObjectNames();
  for (const auto& dataset_name : group_datasets)
  {
    if (dataset_name.rfind(seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES, 0) == 0)
    {
      std::string category = dataset_name.substr(seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES.length() + 1);
      bb_labels.push_back(CategorizedBoundingBoxLabel<NumDimensions>(category));

      auto dataset_labels =
          getHdf5DataSet(data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + category);
      auto dataset_instances =
          getHdf5DataSet(data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + category);
      auto dataset_boxes =
          getHdf5DataSet(data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES + "_" + category);

      if (dataset_instances == nullptr || dataset_instances == nullptr || dataset_boxes == nullptr)
      {
        throw std::invalid_argument("cannot find all bounding box label datasets for category " + category);
      }

      std::vector<std::string> labels;
      std::vector<std::string> instance_uuids;
      std::vector<std::vector<double>> bounding_boxes;

      dataset_labels->read(labels);
      dataset_instances->read(instance_uuids);
      dataset_boxes->read(bounding_boxes);

      if (labels.size() != instance_uuids.size() || labels.size() != bounding_boxes.size())
      {
        throw std::invalid_argument("amounts for labels, instance uuids and bounding boxes do not match for category " +
                                    category);
      }

      for (std::size_t i = 0; i < labels.size(); i++)
      {
        if (bounding_boxes[i].size() != 2 * NumDimensions)
        {
          throw std::invalid_argument(std::to_string(NumDimensions) +
                                      "-dimensional bounding box expected for category " + category + " but got " +
                                      std::to_string(bounding_boxes[i].size() / 2) + "-dimensional one");
        }
        std::array<double, NumDimensions> min_point;
        std::array<double, NumDimensions> max_point;

        for (int j = 0; j < NumDimensions; j++)
        {
          min_point[j] = bounding_boxes[i][j];
          max_point[j] = bounding_boxes[i][j + NumDimensions];
        }

        InstanceLabel label(labels[i], instance_uuids[i]);
        BoundingBoxLabel<NumDimensions> bb_label(label, min_point, max_point);
        bb_labels[bb_labels.size() - 1].addLabel(bb_label);
      }
    }
  }

  return bb_labels;
}

// template <class T>
// void Hdf5PyGeneral::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::Header& header)
// {
//   if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS))
//   {
//     object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, header.stamp().seconds());
//   }
//   else
//   {
//     object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).write(header.stamp().seconds());
//   }

//   if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS))
//   {
//     object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, header.stamp().nanos());
//   }
//   else
//   {
//     object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).write(header.stamp().nanos());
//   }

//   if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID))
//   {
//     object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID, header.frame_id());
//   }
//   else
//   {
//     object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID).write(header.frame_id());
//   }

//   if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ))
//   {
//     object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, header.seq());
//   }
//   else
//   {
//     object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ).write(header.seq());
//   }
// }

// template <class T>
// seerep::Header Hdf5PyGeneral::readHeaderAttributes(HighFive::AnnotateTraits<T>& object, const std::string& id)
// {
//   seerep::Header header;

//   int64_t seconds;
//   int32_t nanos;
//   uint32_t seq;

//   std::string uuidProject = std::filesystem::path(m_file->getName()).filename().stem();

//   object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID).read(*header.mutable_frame_id());

//   object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).read(seconds);
//   object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).read(nanos);
//   object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ).read(seq);

//   header.set_seq(seq);
//   header.mutable_stamp()->set_seconds(seconds);
//   header.mutable_stamp()->set_nanos(nanos);
//   header.set_uuid_project(uuidProject);
//   header.set_uuid_msgs(id);

//   return header;
// }

} /* namespace seerep_hdf5_py */
