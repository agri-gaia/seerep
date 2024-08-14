#ifndef SEEREP_CPP_EXAMPLES_TF_QUERY_H_
#define SEEREP_CPP_EXAMPLES_TF_QUERY_H_

#include <flatbuffers/grpc.h>
#include <grpcpp/grpcpp.h>
#include <seerep_com/meta_operations.grpc.fb.h>
#include <seerep_com/tf_service.grpc.fb.h>

#include <optional>

namespace seerep_cpp_examples
{

class TfQuery
{
public:
  TfQuery(std::shared_ptr<grpc::Channel> channel);
  ~TfQuery() = default;
  void getTf(const std::string& projectUUID, const std::string& childFrame,
             const std::string& parentFrame, const int& seconds,
             const int& nanos);

private:
  std::optional<std::string> getProjectUUID(const std::string& projectName);
  flatbuffers::grpc::Message<seerep::fb::TransformStampedQuery>
  createQuery(const std::string& projectUUID, const std::string& childFrame,
              const std::string& parentFrame, const int& seconds,
              const int& nanos);

  std::shared_ptr<grpc::Channel> channel_;
  flatbuffers::grpc::MessageBuilder fbb_;
};

}  // namespace seerep_cpp_examples

#endif  // SEEREP_CPP_EXAMPLES_TF_QUERY_H_
