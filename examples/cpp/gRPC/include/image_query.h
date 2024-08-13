#ifndef SEEREP_CPP_EXAMPLES_IMAGE_QUERY_H_
#define SEEREP_CPP_EXAMPLES_IMAGE_QUERY_H_

#include <flatbuffers/grpc.h>
#include <grpcpp/grpcpp.h>
#include <seerep_com/image_service.grpc.fb.h>
#include <seerep_com/meta_operations.grpc.fb.h>

#include <optional>

namespace seerep_cpp_examples
{

class ImageQuery
{
public:
  ImageQuery(std::shared_ptr<grpc::Channel> channel);
  ~ImageQuery() = default;
  void getImages(const std::string& projectUUID);

private:
  std::optional<std::string> getProjectUUID(const std::string& projectName);
  flatbuffers::grpc::Message<seerep::fb::Query>
  createQuery(const std::string& projectUUID);

  std::shared_ptr<grpc::Channel> channel_;
  flatbuffers::grpc::MessageBuilder fbb_;
};

}  // namespace seerep_cpp_examples

#endif  // SEEREP_CPP_EXAMPLES_IMAGE_QUERY_H_
