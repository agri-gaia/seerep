#!/usr/bin/env python3

import flatbuffers
from seerep.fb.Empty import Empty
from seerep.fb.meta_operations_grpc_fb import MetaOperationsStub
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import createEmpty

channel = get_gRPC_channel()

stub = MetaOperationsStub(channel)

fbb = flatbuffers.Builder(1024)
responseBuf = stub.LoadProjects(bytes(createEmpty(fbb)))
responseBuf = Empty.GetRootAs(responseBuf)

if responseBuf:
    print("Projects loaded successfully.")
