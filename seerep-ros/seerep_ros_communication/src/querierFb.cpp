// grpc
#include <grpc/grpc.h>
#include <grpc/status.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

// seerep
#include <seerep-com/meta-operations.grpc.fb.h>

int main(void)
{
  auto channel = grpc::CreateChannel("localhost:9090", grpc::InsecureChannelCredentials());

  auto stub = seerep::fb::MetaOperations::NewStub(channel);

  flatbuffers::grpc::MessageBuilder mb;
  auto request_offset = seerep::fb::CreateEmpty(mb);
  mb.Finish(request_offset);
  auto request_msg = mb.ReleaseMessage<seerep::fb::Empty>();

  grpc::ClientContext context;

  flatbuffers::grpc::Message<seerep::fb::ProjectInfos> response_msg;

  auto status = stub->GetProjects(&context, request_msg, &response_msg);

  if (status.ok())
  {
    const seerep::fb::ProjectInfos* response = response_msg.GetRoot();

    for (auto project : *response->projects())
    {
      std::cout << project->name()->str() << " " << project->uuid()->str() << std::endl;
    }

    return EXIT_SUCCESS;
  }
  else
  {
    std::cerr << status.error_code() << ": " << status.error_message() << std::endl;
    return EXIT_FAILURE;
  }
}
