# cmake-lint: disable=C0111

# more grpc generate functions here:https://github.com/zhengwx11/FindGRPC

function(GRPC_GENERATE_PYTHON SRCS)
  if(NOT ARGN)
    message(
      SEND_ERROR "Error: GRPC_GENERATE_PYTHON() called without any proto files"
    )
    return()
  endif()

  find_package(Python3)
  if(NOT Python3_Interpreter_FOUND)
    message(SEND_ERROR "Error: Python Interpreter is not found.")
    return()
  endif()

  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m grpc_tools.protoc
    ERROR_VARIABLE _pygrpc_output
  )

  if(NOT (${_pygrpc_output} STREQUAL "Missing input file.\n"))
    message(
      SEND_ERROR
        "Error: grpcio_tools not installed\ntry: sudo pip install grpcio_tools"
    )
    return()
  endif()

  if(GRPC_GENERATE_CPP_APPEND_PATH)
    # Create an include path for each file specified
    foreach(FIL ${ARGN})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(ABS_PATH ${ABS_FIL} PATH)
      list(
        FIND
        _protobuf_include_path
        ${ABS_PATH}
        _contains_already
      )
      if(${_contains_already} EQUAL -1)
        list(
          APPEND
          _protobuf_include_path
          -I
          ${ABS_PATH}
        )
      endif()
    endforeach()
  else()
    set(_protobuf_include_path)
  endif()

  if(DEFINED PROTOBUF_IMPORT_DIRS)
    foreach(DIR ${PROTOBUF_IMPORT_DIRS})
      get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
      list(
        FIND
        _protobuf_include_path
        ${ABS_PATH}
        _contains_already
      )
      if(${_contains_already} EQUAL -1)
        list(
          APPEND
          _protobuf_include_path
          -I
          ${ABS_PATH}
        )
      endif()
    endforeach()
  endif()

  set(${SRCS})
  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    get_filename_component(FIL_DIR ${ABS_FIL} DIRECTORY)

    list(
      APPEND
      _protobuf_include_path
      -I
      ${FIL_DIR}
    )
    list(
      APPEND
      ${SRCS}
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}_pb2.py"
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}_pb2_grpc.py"
    )

    add_custom_command(
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}_pb2.py"
             "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}_pb2_grpc.py"
      COMMAND
        ${Python3_EXECUTABLE} -m grpc_tools.protoc ${_protobuf_include_path}
        --python_out ${CMAKE_BINARY_DIR} --grpc_python_out ${CMAKE_BINARY_DIR}
        ${ABS_FIL}
      DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Running Python protocol buffer compiler on ${FIL} for grpc"
      VERBATIM
    )
  endforeach()

  set(${SRCS}
      ${${SRCS}}
      PARENT_SCOPE
  )
endfunction()
