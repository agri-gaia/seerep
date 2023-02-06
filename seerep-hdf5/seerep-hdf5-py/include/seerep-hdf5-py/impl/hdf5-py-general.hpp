
namespace seerep_hdf5_py
{
template <int NumDimensions>
void Hdf5PyGeneral::writeBoundingBoxLabeled(
    const std::string& data_group_id,
    const std::vector<seerep_hdf5_py::CategorizedBoundingBoxLabel<NumDimensions>>& bb_labels)
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

        std::vector<double> box(2 * NumDimensions);

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

      HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + category_labels.category,
          HighFive::DataSpace::From(labels));
      datasetLabels.write(labels);

      HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + category_labels.category,
          HighFive::DataSpace::From(instance_uuids));
      datasetInstances.write(instance_uuids);

      HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES + "_" + category_labels.category,
          HighFive::DataSpace::From(bounding_boxes));
      datasetBoxes.write(bounding_boxes);
    }
  }
  m_file->flush();
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
