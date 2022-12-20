#!/usr/bin/env python3

import os
import sys

import flatbuffers
from fb import Empty
from fb import meta_operations_grpc_fb as metaOperations

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()
stub = metaOperations.MetaOperationsStub(channel)

builder = flatbuffers.Builder(1024)
Empty.Start(builder)
EmptyMsg = Empty.End(builder)
builder.Finish(EmptyMsg)
buf = builder.Output()

responseBuf = stub.LoadProjects(bytes(buf))
responseBuf = Empty.Empty.GetRootAs(responseBuf)

if responseBuf:
    print("Projects loaded successfully.")
