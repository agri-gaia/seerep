# Flatbuffers Message Abstractions

The message abstractions are a rather experimental way to more easily create test
variations.

**Note**: This API is subject to change in the future, as some functionality
should get decoupled in a more elegant way.

It is currently used to create variations for tests of `gRPC_fb_getInstances.py`.
The tests can be found under
[test_gRPC_fb_getInstances.py](https://github.com/agri-gaia/seerep/blob/main/tests/python/gRPC/instances/test_gRPC_fb_getInstances.py).

The idea is to provide default implementations for common datatypes, but still
allow for modification of parts of that datatype. In some `flatbuffers`
implementations there exists a API for mutability for that but unfortunately not
for python and even when it would, it would still impose other problems.

To tackle the problem, a wrapper is used where a set of enums, which corresponds
to a datatype's components, can be given in order to assemble a datatype with
those components "activated". Function pointers are used to define what should
happen if a component is inactive (often just `None` is returned) and what
should happen if a component is active.

Datatype implementations for `Query` and `QueryInstance` and their abstraction
model can be found
[here](https://github.com/agri-gaia/seerep/blob/main/tests/python/gRPC/util/msg_abs/msgs.py).

## Defining new datatypes for variation testing

First the datatype has to be defined as a class inheriting from FrozenEnum
provided through
[datastructures.py](https://github.com/agri-gaia/seerep/blob/main/tests/python/gRPC/util/datastructures.py),
which is essentially a unmodifiable enum.

This is done for the `FbQuery` datatype, which corresponding `flatbuffers`
definition can be found
[here](https://github.com/agri-gaia/seerep/blob/main/seerep_msgs/fbs/query.fbs).

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/tests/python/gRPC/util/msg_abs/msgs.py:24:40"
```

Then `FbQuery` inherits from `MsgsFb`, which itself is a template type defined in
[msgs_base.py](https://github.com/agri-gaia/seerep/blob/main/tests/python/gRPC/util/msg_abs/msgs_base.py).

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/tests/python/gRPC/util/msg_abs/msgs.py:48:103"
```

In `MsgsFb` `_set_enum_func_mapping()` is a abstractmethod which return type is
a dictionary, which maps the enum types to `MsgsFunctions`. `MsgsFunction`
itself is just a structure to wrap two function pointers. The first function
pointer should be a pointer to the `default_function`, which gets called, if the
component is not set to be active. The second function pointer is the one that
gets called when the component is set active.

On runtime it is checked if all elements of the enum are mapped. The functions
are mostly mapped to default implementations for that specific component datatype.
The default functions implementations can be inspected
[here](https://github.com/agri-gaia/seerep/blob/main/tests/python/gRPC/util/msg_abs/msgs.py)
at the bottom.

The `@expect_component` decorator can be used to define dependencies between the
component datatypes, e.g. for the `instanceuuid()` function to work, the
`EnumFbQuery.PROJECTUUID` component must be set to active.

Lastly the abstractmethod `_assemble_datatype_instance()` has to be implemented,
in the function all of the wrapper managed components should be retrieved and
the datatype should be built and returned. The base class makes sure that all
the components at this point are set.

## Using message abstractions for testing

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/tests/python/gRPC/instances/test_gRPC_fb_getInstances.py:83:113"
```

In this function all the possible datatypes with attached instances are tested
(this is just a snippet of that particular function).

The message abstractions are used by creating a object of `FbQueryInstance` first
and setting the only active enum to `EnumFbQueryInstance.DATATYPE` using the
`enum_types` variable on initialization, as this is the only relevant component
to be modified for the test. In this case it would be relatively easy to test
different sets of components together e.g. testing for interference when specific
datatypes are set and while a polygon is used to restrict the relevant area.

After that `set_active_function()` is used to set a different function pointer
to the active component's function pointer. This essentially sets the second
component of a `MsgsFunction` type in the dictionary returned by
`_set_enum_func_mapping()`, which is discussed in the previous section.

Lastly if a change to the dictionary has been done using `set_active_function()`
or `set_mapped_functions()`, the `assemble_datatype_instance()` has to be called
to reassemble the underlying datatype.

**Note**: On creation of the instance the datatype is assembled automatically in
it's constructor.

Now the assembled datatype can be accessed using the property
`queryinst_builder.datatype_instance` and the underlying flatbuffers builder can
be accessed by using `queryinst_builder.builder` for further use, e.g. like in
this case calling the service function `call_get_instances_fb()` using the
[ServiceManager](python-helpers.md#service_managerpy).

**Note**: The `MsgsBase` class provides it's own `ServiceManager` property for
building components, but that one shouldn't be used as it could change in the
future.

Another snippet to highlight is the following where one of the components of the
datatype itself is inheriting from `MsgsFb`.

```python
--8<-- "https://raw.githubusercontent.com/agri-gaia/seerep/main/tests/python/gRPC/instances/test_gRPC_fb_getInstances.py:201:261"
```

Here `query_builder` is used to build the `Query.Query` datatype in order to
supply that one to the `queryinst_builder` query component. It is important that
`query_builder.assemble_datatype_instance()` is called before
`queryinst_builder.assemble_datatype_instance()`, otherwise the changes by setting
the active function on `query_builder` are not reflected in the
`queryinst_builder.datatype_instance`.

## Inner workings of the `MsgsFb` and `MsgsBase` classes

![message-abstractions](../imgs/message-abstractions.svg){ width=900px }

**Meaning of Symbols and Notations in this diagram**:

- Rectangular boxes: instance methods
- Ellipsis: instance variables
- Line arrows (with text): calls to methods or setting variables (with the help
  of those variables specified by the text)
- Line arrows with numbers: show the order in which things are done
- dotted arrows outgoing: Getter methods/properties
- dotted arrows incoming: Setter methods/properties
- red colored text: already used functionality in the examples above
- purple colored text: `@abstractmethod` also used above
- blue colored text: has a special meaning, is not mirrored exactly by the
  implementation

**Note**: Some details are not shown like the validation methods for the given
enum type. But they are not neccessary to understand the structure.

First in the initialization phase variables (`_builder`, `_service_manager`,
`_channel`, `_active_enums`) are set and managed by the `MsgsFb` instance, those
are needed for the assembly of the datatype instance later. `_builder` is a
simply a flatbuffers builder. `channel` is used to create `ServiceManager`
instance and manage a `grpc_channel` type variable. At last `_active_enums` is a
set of enum elements, which will be used to specify which component of the datatype
is "active" (i.e. which component is set by the `active_function` of the
`MsgsFunctions` class).

After that the `_enum_func_mapping` variable is set by the
`_set_enum_func_mapping()` function. This variable can also get manipulated by
`set_active_function()` or `set_mapped_functions()`. Then
`_assemble_components()` is called, which makes sure that the `_components` are
set, i.e. the functions in the `_enum_func_mapping` are called and the components
are set by those `default_functions` or those `active_functions`,
if their corresponding enum is in the `_active_enums` set.

**Note**: `_components` are implemented as multiple dynamically at runtime
created instance variables with the name of the component specified by the enum
element name.

Finally the on the instance callable `assemble_datatype_instance()` method
triggers a rebuild of the `_assembled_datatype_instance`, by first refreshing
the components and then calling the `_assemble_datatype_instance()` method.
The `assemble_datatype_instance()` method gets also called in the `__init__()`
method, such that the message abstractions always try to guarantee a set
`datatype_instance` variable.
