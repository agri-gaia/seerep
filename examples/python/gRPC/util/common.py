import os

import grpc


def get_gRPC_channel(target="local"):
    # set the max message size to 1GB (half the size of the server)
    options = [
        ('grpc.max_send_message_length', 1 * 1024 * 1024 * 1024),
        ('grpc.max_receive_message_length', 1 * 1024 * 1024 * 1024),
    ]

    if target == "prod" or target == "dev":
        if target == "prod":
            targetName = "seerep-prod"
            certName = "tls-prod.pem"
        else:
            targetName = "seerep"
            certName = "tls.pem"

        # server with certs
        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        with open(os.path.join(__location__, '../../../certs/' + certName), 'rb') as f:
            root_cert = f.read()
        server = targetName + ".robot.10.249.3.13.nip.io:31723"
        creds = grpc.ssl_channel_credentials(root_cert)
        channel = grpc.secure_channel(server, creds)

    elif target == "local":
        # server without certs
        server = "localhost:9090"
        channel = grpc.insecure_channel(server, options=options)
    else:
        channel = grpc.insecure_channel(target, options=options)

    return channel
