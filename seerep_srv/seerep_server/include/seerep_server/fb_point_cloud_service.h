#ifndef SEEREP_SERVER_FB_POINT_CLOUD_SERVICE_H_
#define SEEREP_SERVER_FB_POINT_CLOUD_SERVICE_H_

// seerep
#include <seerep_com/point_cloud_service.grpc.fb.h>
#include <seerep_core/core.h>
#include <seerep_core_fb/core_fb_pointcloud.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
/**
 * @brief Class which implements the flatbuffers PointCloudService defined in seerep-com.
 *        Main task is to forward the calls to the core-fb specific write and read functions.
 *
 */
class FbPointCloudService final : public seerep::fb::PointCloudService::Service
{
public:
  /**
   * @brief Constructs the service based on the general core
   *
   * @param seerepCore a shared pointer to the general core
   */
  FbPointCloudService(std::shared_ptr<seerep_core::Core> seerepCore);
  /**
   * @brief Stream point clouds matching the query to the client
   *
   * @param context custom inital and trailing metadata (currently not used)
   * @param request incoming gRPC query request
   * @param writer  writer object used to stream the point clouds
   * @return grpc::Status status of the request. Did it work?
   */
  grpc::Status GetPointCloud2(grpc::ServerContext* context,
                              const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                              grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* writer) override;
  /**
   * @brief Save point clouds from the incoming gRPC stream on the server
   *
   * @param context custom inital and trailing metadata (currently not used)
   * @param reader incoming message stream of point clouds from the client
   * @param response gRPC message to describe the transmission state of the point clouds
   * @return grpc::Status status of the request. Did it work?
   */
  grpc::Status TransferPointCloud2(grpc::ServerContext* context,
                                   grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* reader,
                                   flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response) override;
  /**
   * @brief Adds a stream of labeled bounding boxes to already stored point clouds
   *
   * @param context custom inital and trailing metadata (currently not used)
   * @param reader incoming message stream of bounding boxes from the client
   * @param response gRPC message to describe the transmission state of the bounding boxes
   * @return grpc::Status status of the request. Did it work?
   */
  // TODO
  // grpc::Status AddBoundingBoxesLabeled(
  //     grpc::ServerContext* context,
  //     grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::BoundingBoxesLabeledStamped>>* reader,
  //     flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response) override;

private:
  /** @brief a shared pointer to the general core */
  std::shared_ptr<seerep_core_fb::CoreFbPointCloud> pointCloudFb;
  /** @brief the logger for the logging framework */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};
} /* namespace seerep_server */

#endif  // SEEREP_SERVER_FB_POINT_CLOUD_SERVICE_H_
