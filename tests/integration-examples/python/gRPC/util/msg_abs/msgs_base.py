import functools
from abc import ABC, abstractmethod
from enum import Enum
from typing import Callable, Dict, Final, Generic, List, NamedTuple, Set, TypeVar, Union

from flatbuffers import Builder
from grpc import Channel
from gRPC.util.datastructures import FrozenEnum
from seerep.util.service_manager import ServiceManager


class MsgsFunctions(NamedTuple):

    default_function: Callable
    active_function: Callable


T = TypeVar("T")


def expect_component(*args: FrozenEnum):
    def func_passer(func):
        @functools.wraps(func)
        def wrapper(self, *fnargs, **fnkwargs):
            for arg in args:
                self._validate_enum(arg)
                if self.get_component(arg) == None:
                    raise ValueError(f"The component mapped to the enum {arg} is expected to be not None")
            return func(self, *fnargs, **fnkwargs)

        return wrapper

    return func_passer


class MsgsBase(ABC, Generic[T]):
    _ENUMVAR_PREFIX: Final[str] = "_buildvar_"

    @property
    def service_manager(self) -> ServiceManager:
        return self._service_manager

    @property
    def channel(self) -> Channel:
        return self._channel

    @channel.setter
    def set_channel(self, channel: Channel):
        self._service_manager: ServiceManager = ServiceManager(channel)
        self._channel: Channel = channel

    @property
    def active_enums(self) -> Set[FrozenEnum]:
        return self._active_enums

    @active_enums.setter
    def active_enums(self, enum_types: List[FrozenEnum]):
        for enum in enum_types:
            self._validate_enum(enum)
        self._active_enums = set(enum_types)

    def _validate_enum_func_mappings(self):
        # check that every entry is a subclass of enum and that every entry is of same type
        enum_type = type(next(iter(self._enum_func_mapping)))

        if not issubclass(enum_type, FrozenEnum):
            raise KeyError(
                f"a key of the dict returned by implemented _set_enum_func_mapping() is not a subclass of {FrozenEnum}!"
            )

        # check that all keys are of same type
        for t in self._enum_func_mapping:
            if not type(t) == enum_type:
                raise KeyError(
                    "the keys of the dict returned by implemented _set_enum_func_mapping() are of different types!"
                )

        # check that every enum entry is mapped
        for enum_entry in enum_type:
            if self._enum_func_mapping.get(enum_entry, None) is None:
                raise KeyError(
                    f"No mapping of enum entry {enum_entry} to a instance function in implemented _set_enum_func_mapping()!"
                )

        # check that the functions are callables
        for func in self._enum_func_mapping.values():
            if not (callable(func.active_function) or callable(func.default_function)):
                raise ValueError(
                    f"the functions in {func} mapped to a enum in implemented _set_enum_func_mapping() are not all callable!"
                )

    def _validate_enum(self, enum):
        enum_type = type(next(iter(self._enum_func_mapping)))
        if not type(enum) == enum_type:
            raise KeyError(f"the used enum of {self.__class__} is of type {enum_type} not of type {type(enum)}")

    def __init__(self, channel: Channel, enum_types: Set[FrozenEnum] = set()):
        self._channel = channel
        self._active_enums = enum_types
        self._assembled_datatype_instance = None

        self._enum_func_mapping: Dict[FrozenEnum, MsgsFunctions] = self._set_enum_func_mapping()

        if len(self._enum_func_mapping) < 1:
            raise KeyError(f"dict returned by implemented _set_enum_func_mapping() cannot be empty!")

        self._validate_enum_func_mappings()

        # create hidden variables to manage the objects
        for enum_type in self._enum_func_mapping:
            self._set_component(enum_type, None)

        self.assemble_datatype_instance()

    @abstractmethod
    def _set_enum_func_mapping(self) -> Dict[Enum, MsgsFunctions]:
        """
        Returns a mapping from a enum to callable functions.

        Args:
            instance: The instance created by calling the `__new__` method.
        """
        raise NotImplementedError

    def get_mapped_functions(self, enum_type: FrozenEnum) -> Union[MsgsFunctions, None]:
        """
        Returns to the `enum_type` mapped functions.

        Args:
            enum_type: The enum type to get the mapped functions from.

        Returns:
            To the enum_type mapped functions or None if enum_type
            was not mapped to functions.
        """
        return self._enum_func_mapping.get(enum_type, None)

    def set_mapped_functions(self, enum_type: FrozenEnum, function: MsgsFunctions):
        """
        Set `enum_type` mapped functions.

        Args:
            enum_type: The enum type to map the functions to.
            function: A object of type MsgsFunctions holding 2 callable functions.
        """
        self._enum_func_mapping[enum_type] = function
        self._validate_enum_func_mappings()

    def set_active_function(self, enum_type: FrozenEnum, function: Callable):
        """
        Set `enum_type` mapped active function.

        Args:
            enum_type: The enum type to map the function to.
            function: A callable function.
        """
        msg_funcs = MsgsFunctions(self.get_mapped_functions(enum_type).default_function, function)
        self._enum_func_mapping[enum_type] = msg_funcs
        self._validate_enum_func_mappings()

    @classmethod
    def get_enumvar_name(cls, enum_type: FrozenEnum) -> str:
        """
        Retrieve the name of the internally used member variable for this `enum_type`.

        Args:
            enum_type: The enum type to get the enumvar name from.

        Returns:
            the member variable name which is internally used for the components"""
        return cls._ENUMVAR_PREFIX + str(enum_type).split(".")[1]

    def get_component(self, enum_type: Enum):
        """
        Get the targeted component through the given `enum_type`.

        Args:
            enum_type: The enum type to get corresponding value from.

        Returns:
            The data which is stored for the corresponding enum type.
        """
        return getattr(self, self.get_enumvar_name(enum_type))

    def _set_component(self, enum_type: FrozenEnum, data):
        """
        Set the component targeted through the given enum_type.
        The components should only be set through `_assemble_datatype_instance`.

        Args:
            enum_type: The enum type to set the `data` to.
            data: The data to set for this component.
        """
        setattr(self, self.get_enumvar_name(enum_type), data)

    def _assemble_components(self):
        # call the enum mapped active functions according to which enums where given in the set,
        # else call the default function
        all_enum_types = type(next(iter(self._enum_func_mapping)))

        for enum_type in all_enum_types:
            if enum_type in self.active_enums:
                self._set_component(enum_type, self._enum_func_mapping[enum_type].active_function())
            else:
                self._set_component(enum_type, self._enum_func_mapping[enum_type].default_function())

    @property
    def datatype_instance(self) -> T:
        """
        Get the datatype instance managed by this class.

        Returns:
            The instance of the datatype, if assembled through `assemble_datatype_instance`, else `None`.
        """
        return self._assembled_datatype_instance

    @abstractmethod
    def _assemble_datatype_instance(self):
        raise NotImplementedError

    def assemble_datatype_instance(self):
        """
        Function to assemble the targeted datatype message.
        """
        self._assemble_components()

        # debug
        # all_enum_types = type(next(iter(self._enum_func_mapping)))
        # for enum in all_enum_types:
        #     print(self.get_component(enum))

        self._assembled_datatype_instance = self._assemble_datatype_instance()


class MsgsFb(MsgsBase[T]):
    @property
    def builder(self):
        return self._builder

    def __init__(
        self,
        channel: Channel,
        builder: Builder = Builder(),
        enum_types: Set[FrozenEnum] = set(),
    ):
        self._builder = builder
        super().__init__(channel, enum_types)

    # this should only be used for debugging purposes and not for permanent implementation
    def _get_finished_instance(self):
        """
        Returns the assembled flatbuffers object corresponding to the datatype.
        """
        self.builder.Finish(self.datatype_instance)
        # note: this accesses the class behind T,
        # though this is rather hacky and is discouraged by PEP 560
        return self.__class__.__orig_bases__[0].__args__[0].GetRootAs(bytes(self.builder.Output()))

    @abstractmethod
    def _set_enum_func_mapping(self) -> Dict[Enum, MsgsFunctions]:
        """
        Returns a mapping from a enum to callable functions.

        Args:
            instance: The instance created by calling the `__new__` method.
        """
        raise NotImplementedError

    @abstractmethod
    def _assemble_datatype_instance(self):
        raise NotImplementedError
