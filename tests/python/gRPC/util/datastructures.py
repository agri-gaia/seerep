# this will provide a interface to creating a query with the following service
# definition
from enum import Enum, EnumMeta


# credits to https://stackoverflow.com/questions/54274002/python-enum-prevent-invalid-attribute-assignment
class FrozenEnumMeta(EnumMeta):
    "Enum metaclass that freezes an enum entirely"

    def __new__(mcls, name, bases, classdict):
        classdict["__frozenenummeta_creating_class__"] = True
        enum = super().__new__(mcls, name, bases, classdict)
        del enum.__frozenenummeta_creating_class__
        return enum

    def __call__(cls, value, names=None, *, module=None, **kwargs):
        if names is None:  # simple value lookup
            return cls.__new__(cls, value)
        enum = Enum._create_(value, names, module=module, **kwargs)
        enum.__class__ = type(cls)
        return enum

    def __setattr__(cls, name, value):
        members = cls.__dict__.get("_member_map_", {})
        if hasattr(cls, "__frozenenummeta_creating_class__") or name in members:
            return super().__setattr__(name, value)
        if hasattr(cls, name):
            msg = "{!r} object attribute {!r} is read-only"
        else:
            msg = "{!r} object has no attribute {!r}"
        raise AttributeError(msg.format(cls.__name__, name))

    def __delattr__(cls, name):
        members = cls.__dict__.get("_member_map_", {})
        if hasattr(cls, "__frozenenummeta_creating_class__") or name in members:
            return super().__delattr__(name)
        if hasattr(cls, name):
            msg = "{!r} object attribute {!r} is read-only"
        else:
            msg = "{!r} object has no attribute {!r}"
        raise AttributeError(msg.format(cls.__name__, name))


class FrozenEnum(Enum, metaclass=FrozenEnumMeta):
    pass
