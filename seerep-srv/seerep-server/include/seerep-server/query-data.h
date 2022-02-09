#ifndef SEEREP_SERVER_QUERY_DATA_H_
#define SEEREP_SERVER_QUERY_DATA_H_

// seerep
#include <seerep-com/query_data.grpc.pb.h>
#include <seerep-core/project-overview.h>

namespace seerep_server
{
class QueryData final : public seerep::QueryData::Service
{
public:
  QueryData(std::shared_ptr<seerep_core::ProjectOverview> projectOverview);

  grpc::Status GetImage(grpc::ServerContext* context, const seerep::Query* request,
                        grpc::ServerWriter<seerep::Image>* writer);

  grpc::Status GetPointCloud2(grpc::ServerContext* context, const seerep::Query* request,
                              grpc::ServerWriter<seerep::PointCloud2>* writer);

private:
  std::shared_ptr<seerep_core::ProjectOverview> projectOverview;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_QUERY_DATA_H_
