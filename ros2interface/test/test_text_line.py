import pytest

from ros2interface.verb.show import *


@pytest.mark.parametrize("text, expected", [
    ("package/type/Name my_variable", "package/type/Name"),
    ("package/Name my_variable", "package/Name"),
    ("Name my_variable", "Name"),
    ("# comment", None),
    ("#comment", None),
    ("    ", None),
    ("", None),
])
def test_get_interface_string_from_text(text, expected):
    text_line = TextLine(text=text)
    assert text_line._get_interface_string_from_text() == expected


@pytest.mark.parametrize("text, expected", [
    ("package/type/Name my_variable", ("package", "type", "Name")),
    ("package/Name my_variable", ("package", None, "Name")),
    ("Name my_variable", (None, None, "Name")),
    ("# comment", (None, None, None)),
    ("         ", (None, None, None)),
    ("", (None, None, None)),
])
def test_get_interface_parts_from_text(text, expected):
    text_line = TextLine(text=text)
    assert text_line._get_interface_parts_from_text() == expected


@pytest.mark.parametrize("text, default_package, default_type", [
    ("package/type/Name my_variable", None, None),
    ("package/Name my_variable", None, "type", ),
    ("Name my_variable", "package", "type", ),
])
def test_get_interface_parts(text, default_package, default_type):
    text_line = TextLine(
        text=text,
        default_package=default_package,
        default_type=default_type,
    )
    assert text_line._get_interface_parts() == ("package", "type", "Name")


@pytest.mark.parametrize("text, default_package, default_type", [
    ("package/Name my_variable", "package", None),
    ("Name my_variable", "package", None),
    ("Name my_variable", None, "type"),
    ("Name my_variable", None, None),
    ("# comment", "package", None),
    ("test_show# comment", None, "type"),
    ("# comment", None, None),
    ("", "package", None),
    ("", None, "type"),
    ("", None, None),
])
def test_raise_errors_for_get_interface_parts(text, default_package, default_type):
    text_line = TextLine(
        text=text,
        default_package=default_package,
        default_type=default_type,
    )
    with pytest.raises(ValueError):
        text_line._get_interface_parts()


@pytest.mark.parametrize("text, expected", [
    ("package/type/Name my_variable", False),
    ("package/Name my_variable", False),
    ("Name my_variable", False),
    ("# comment", True),
    ("#comment", True),
    ("    ", True),
    ("", True),
])
def test_is_comment_or_whitespace(text, expected):
    text_line = TextLine(text=text)
    assert text_line._is_comment_or_whitespace() == expected


@pytest.mark.parametrize("text, default_package, default_type, expected", [
    ("package/type/Name my_variable", "package", "type", True),
    ("package/type/Name my_variable", "package", None, True),
    ("package/type/Name my_variable", None, "type", True),
    ("package/Name my_variable", "package", "type", True),
    ("package/Name my_variable", "package", None, False),
    ("package/Name my_variable", None, "type", True),
    ("package/Name my_variable", None, None, False),
    ("Name my_variable", "package", "type", True),
    ("Name my_variable", "package", None, False),
    ("Name my_variable", None, "type", False),
    ("Name my_variable", None, None, False),
    ("# comment", "package", None, False),
    ("# comment", None, "type", False),
    ("# comment", None, None, False),
    ("", "package", None, False),
    ("", None, "type", False),
    ("", None, None, False),
])
def test_is_expandable(text, default_package, default_type, expected):
    text_line = TextLine(
        text=text,
        default_package=default_package,
        default_type=default_type,
    )
    assert text_line.is_expandable() == expected


@pytest.mark.parametrize("string, comment_string, expected", [
    ("hi there", "#", None),
    ("hi #there", "#", "hi"),
    ("hi# there", "#", "hi"),
    ("#hi there", "#", ""),
    ("   #hi there", "#", ""),
    ("hi there#", "#", "hi there"),
    ("my string 'hi'", "#", None),
    ("my string '#hi'", "#", None),
    ("my string `\"'hi'\"`", "#", None),
    ("my string '' `` `\"'#hi'\"`", "#", None),
    ("my string", "//", None),
    ("my string//", "//", "my string"),
    ("my //string//", "//", "my"),
    ("//my //string//", "//", ""),
    ("   //my //string//", "//", ""),
    ("my string \\\"\\`\\'hi", "#", None),
])
def test_remove_comment_from_string(string, comment_string, expected):
    if expected is None:
        expected = string
    assert remove_comment_from_string(
        string,
        comment_string=comment_string,
        parenthatic_chars="'\"`"
    ) == expected


@pytest.mark.parametrize("string, idx, expected", [
    ("oh", 1, False),
    ("\\'", 1, True),
    ("hi\\'", 3, True),
    ("\\'", 2, False),
    ("hi\\'", 4, False),
])
def test_escape_chars(string, idx, expected):
    assert is_preceded_by_escape_char(
        string,
        idx,
        escape_char="\\"
    ) == expected


@pytest.mark.parametrize("text, is_raw, expected", [
    ("my_package/type/name value", True, None),
    ("my_package/type/name value #comment", True, None),
    ("my_package/name value #comment", True, None),
    ("name value #comment", True, None),
    ("my_package/type/name value #comment", False, "my_package/type/name value"),
    ("my_package/name value #comment", False, "my_package/name value"),
    ("name value #comment", False, "name value"),
    ("#comment", True, None),
    ("#comment", False, ""),
    ("", True, None),
    ("", False, None),
    ("   ", True, None),
    ("   ", False, ""),
])
def test_get_text(text, is_raw, expected):
    if expected is None:
        expected = text
    text_line = TextLine(
        text=text,
    )
    assert text_line.get_text(raw=is_raw) == expected
