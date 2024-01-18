import re

from google.protobuf.descriptor import FieldDescriptor

TYPE_CALLABLE_MAP = {
    FieldDescriptor.TYPE_DOUBLE: float,
    FieldDescriptor.TYPE_FLOAT: float,
    FieldDescriptor.TYPE_INT32: int,
    FieldDescriptor.TYPE_INT64: int,
    FieldDescriptor.TYPE_UINT32: int,
    FieldDescriptor.TYPE_UINT64: int,
    FieldDescriptor.TYPE_SINT32: int,
    FieldDescriptor.TYPE_SINT64: int,
    FieldDescriptor.TYPE_FIXED32: int,
    FieldDescriptor.TYPE_FIXED64: int,
    FieldDescriptor.TYPE_SFIXED32: int,
    FieldDescriptor.TYPE_SFIXED64: int,
    FieldDescriptor.TYPE_BOOL: bool,
    FieldDescriptor.TYPE_STRING: str,
    FieldDescriptor.TYPE_BYTES: str,
    FieldDescriptor.TYPE_ENUM: int,
}

EXTENSION_CONTAINER = "___X"

UPPER_CASE_LETTERS = r"([A-Z])"
RPLC_UPPER_CASE_LETTERS = r"_\1"

# based on https://github.com/benhodgson/protobuf-to-dict
def repeated(type_callable):
    return lambda value_list: [type_callable(value) for value in value_list]


def enum_label_name(field, value):
    return field.enum_type.values_by_number[int(value)].name


def pb_to_dict(pb_object, to_snake_case=False):
    result_dict = {}
    extensions = {}
    for field, value in pb_object.ListFields():
        type_callable = _get_field_value_adaptor(pb_object, field, to_snake_case)
        if field.type == FieldDescriptor.TYPE_MESSAGE:
            # recursively encode protobuf sub-message
            type_callable = lambda pb: pb_to_dict(pb, to_snake_case)

        if field.label == FieldDescriptor.LABEL_REPEATED:
            type_callable = repeated(type_callable)

        if field.is_extension:
            extensions[str(field.number)] = type_callable(value)
            continue

        f_name = field.name
        if to_snake_case:
            f_name = (f_name[0] + re.sub(UPPER_CASE_LETTERS, RPLC_UPPER_CASE_LETTERS, f_name[1:])).lower()

        result_dict[f_name] = type_callable(value)

    if extensions:
        result_dict[EXTENSION_CONTAINER] = extensions
    return result_dict


def _get_field_value_adaptor(pb, field, to_snake_case=False):
    if field.type == FieldDescriptor.TYPE_MESSAGE:
        # recursively encode protobuf sub-message
        return lambda pb: pb_to_dict(pb, to_snake_case)

    if field.type == FieldDescriptor.TYPE_ENUM:
        return lambda value: enum_label_name(field, value)

    if field.type in TYPE_CALLABLE_MAP:
        return TYPE_CALLABLE_MAP[field.type]

    raise TypeError("Field %s.%s has unrecognised type id %d" % (pb.__class__.__name__, field.name, field.type))
