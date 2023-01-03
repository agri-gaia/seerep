namespace seerep_ros_examples
{
template <typename T>
void Hdf5Node::getROSParameter(const std::string& parameter_name, T& parameter_value, bool required)
{
  if (!private_node_handle_.getParam(parameter_name, parameter_value) && required)
  {
    throw std::runtime_error("Parameter: " + parameter_name + " not available");
  }
}

template <typename T>
void Hdf5Node::dumpMessage(const boost::shared_ptr<const T>& message)
{
  msg_dump_->saveMessage(*message);
}

}  // namespace seerep_ros_examples
