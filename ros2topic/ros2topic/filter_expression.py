# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Evaluation of user supplied filter expressions on messages.

The expression is parsed into a Python AST and interpreted directly, so ``eval()`` is never
used. Only a fixed set of syntax is supported, the only name in scope is ``m`` (the message),
attributes may only be message fields, and only allow-listed builtins and methods can be called.
"""

import ast
import operator

__all__ = ['FilterExpressionError', 'compile_filter']

# Limits that keep a single evaluation from using unreasonable amounts of memory or time.
_MAX_DEPTH = 100
_MAX_SEQUENCE_LENGTH = 100000
_MAX_POWER_BITS = 10000
_MAX_ITERATIONS = 100000
_STEPS_KEY = ''  # not a valid identifier, so it cannot clash with a name in the expression

_BUILTINS = {
    f.__name__: f for f in (
        abs, all, any, bool, float, int, len, list, max, min, round, sorted, str, sum, tuple)
}

_STR_METHODS = frozenset({
    'capitalize', 'count', 'endswith', 'find', 'index', 'isalnum', 'isalpha', 'isdecimal',
    'isdigit', 'islower', 'isnumeric', 'isspace', 'isupper', 'lower', 'lstrip', 'partition',
    'removeprefix', 'removesuffix', 'rfind', 'rindex', 'rpartition', 'rsplit',
    'rstrip', 'split', 'splitlines', 'startswith', 'strip', 'swapcase', 'title', 'upper',
})

# Methods that can be called on values, keyed by the exact type of the value.
_METHODS = {
    str: _STR_METHODS,
    list: frozenset({'count', 'index'}),
    tuple: frozenset({'count', 'index'}),
}

_BINARY_OPERATORS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.FloorDiv: operator.floordiv,
    ast.Mod: operator.mod,
    ast.Pow: operator.pow,
}

_UNARY_OPERATORS = {
    ast.Not: operator.not_,
    ast.USub: operator.neg,
    ast.UAdd: operator.pos,
    ast.Invert: operator.invert,
}

_COMPARE_OPERATORS = {
    ast.Eq: operator.eq,
    ast.NotEq: operator.ne,
    ast.Lt: operator.lt,
    ast.LtE: operator.le,
    ast.Gt: operator.gt,
    ast.GtE: operator.ge,
    ast.Is: operator.is_,
    ast.IsNot: operator.is_not,
    ast.In: lambda a, b: a in b,
    ast.NotIn: lambda a, b: a not in b,
}

_ALLOWED_CONSTANT_TYPES = (int, float, str, bool, type(None))


class FilterExpressionError(Exception):
    """Raised if a filter expression is invalid or fails to evaluate."""


def _is_message(obj):
    return callable(getattr(type(obj), 'get_fields_and_field_types', None))


def _get_field(obj, name):
    if not _is_message(obj) or name not in obj.get_fields_and_field_types():
        raise FilterExpressionError(f"Attribute '{name}' is not allowed")
    return getattr(obj, name)


def _check_binary_operands(op, left, right):
    if op is ast.Pow:
        if isinstance(left, int) and isinstance(right, int):
            if abs(left).bit_length() * abs(right) > _MAX_POWER_BITS:
                raise FilterExpressionError('Result of power is too large')
    elif op is ast.Mult:
        for sequence, count in ((left, right), (right, left)):
            if isinstance(sequence, (str, list, tuple)) and isinstance(count, int):
                if len(sequence) * count > _MAX_SEQUENCE_LENGTH:
                    raise FilterExpressionError('Result of repetition is too large')
    elif op is ast.Mod:
        if isinstance(left, (str, bytes)):
            raise FilterExpressionError('String formatting is not allowed')


def _check_depth(tree):
    stack = [(tree, 1)]
    while stack:
        node, depth = stack.pop()
        if depth > _MAX_DEPTH:
            raise FilterExpressionError('Filter expression is too deeply nested')
        stack.extend((child, depth + 1) for child in ast.iter_child_nodes(node))


def _unsupported(node):
    return FilterExpressionError(f'{type(node).__name__} is not allowed in a filter expression')


def _compile_node(node):
    """Turn an AST node into a function taking a scope (dict of names) and returning a value."""
    compile_method = _COMPILERS.get(type(node))
    if compile_method is None:
        raise _unsupported(node)
    return compile_method(node)


def _compile_constant(node):
    if not isinstance(node.value, _ALLOWED_CONSTANT_TYPES):
        raise _unsupported(node)
    value = node.value
    return lambda scope: value


def _compile_name(node):
    name = node.id

    def get_name(scope):
        try:
            return scope[name]
        except KeyError:
            raise FilterExpressionError(f"Name '{name}' is not defined") from None
    return get_name


def _compile_bool_op(node):
    values = [_compile_node(v) for v in node.values]
    if isinstance(node.op, ast.And):
        def evaluate(scope):
            result = True
            for v in values:
                result = v(scope)
                if not result:
                    break
            return result
    else:
        def evaluate(scope):
            result = False
            for v in values:
                result = v(scope)
                if result:
                    break
            return result
    return evaluate


def _compile_unary_op(node):
    op = _UNARY_OPERATORS.get(type(node.op))
    if op is None:
        raise _unsupported(node.op)
    operand = _compile_node(node.operand)
    return lambda scope: op(operand(scope))


def _compile_bin_op(node):
    op_type = type(node.op)
    op = _BINARY_OPERATORS.get(op_type)
    if op is None:
        raise _unsupported(node.op)
    left = _compile_node(node.left)
    right = _compile_node(node.right)

    def evaluate(scope):
        a = left(scope)
        b = right(scope)
        _check_binary_operands(op_type, a, b)
        return op(a, b)
    return evaluate


def _compile_compare(node):
    ops = []
    for op in node.ops:
        if type(op) not in _COMPARE_OPERATORS:
            raise _unsupported(op)
        ops.append(_COMPARE_OPERATORS[type(op)])
    left = _compile_node(node.left)
    comparators = [_compile_node(c) for c in node.comparators]

    def evaluate(scope):
        a = left(scope)
        for op, comparator in zip(ops, comparators):
            b = comparator(scope)
            if not op(a, b):
                return False
            a = b
        return True
    return evaluate


def _compile_if_exp(node):
    test = _compile_node(node.test)
    body = _compile_node(node.body)
    orelse = _compile_node(node.orelse)
    return lambda scope: body(scope) if test(scope) else orelse(scope)


def _compile_attribute(node):
    value = _compile_node(node.value)
    name = node.attr
    if name.startswith('_'):
        raise FilterExpressionError(f"Attribute '{name}' is not allowed")
    return lambda scope: _get_field(value(scope), name)


def _compile_subscript(node):
    value = _compile_node(node.value)
    index = _compile_node(node.slice)
    return lambda scope: value(scope)[index(scope)]


def _compile_slice(node):
    parts = [None if p is None else _compile_node(p) for p in (node.lower, node.upper, node.step)]

    def evaluate(scope):
        return slice(*[None if p is None else p(scope) for p in parts])
    return evaluate


def _compile_call(node):
    if any(isinstance(a, ast.Starred) for a in node.args) or any(
        k.arg is None for k in node.keywords
    ):
        raise FilterExpressionError('Argument unpacking is not allowed in a filter expression')
    args = [_compile_node(a) for a in node.args]
    keywords = [(k.arg, _compile_node(k.value)) for k in node.keywords]

    def evaluate_args(scope):
        return [a(scope) for a in args], {k: v(scope) for k, v in keywords}

    func = node.func
    if isinstance(func, ast.Name):
        if func.id not in _BUILTINS:
            raise FilterExpressionError(f"Call to function '{func.id}()' is not allowed")
        builtin = _BUILTINS[func.id]

        def call_builtin(scope):
            positional, named = evaluate_args(scope)
            return builtin(*positional, **named)
        return call_builtin

    if isinstance(func, ast.Attribute):
        target = _compile_node(func.value)
        name = func.attr

        def call_method(scope):
            obj = target(scope)
            if name not in _METHODS.get(type(obj), ()):
                raise FilterExpressionError(
                    f"Method '{name}()' is not allowed on '{type(obj).__name__}'")
            positional, named = evaluate_args(scope)
            return getattr(obj, name)(*positional, **named)
        return call_method

    raise FilterExpressionError('Only calls to named functions and methods are allowed')


def _compile_container(node):
    elements = [_compile_node(e) for e in node.elts]
    if isinstance(node, ast.Set):
        if any(isinstance(e, ast.Starred) for e in node.elts):
            raise _unsupported(node)
        return lambda scope: {e(scope) for e in elements}
    if isinstance(node, ast.List):
        return lambda scope: [e(scope) for e in elements]
    return lambda scope: tuple(e(scope) for e in elements)


def _compile_dict(node):
    if any(k is None for k in node.keys):
        raise FilterExpressionError('Dictionary unpacking is not allowed in a filter expression')
    items = [(_compile_node(k), _compile_node(v)) for k, v in zip(node.keys, node.values)]
    return lambda scope: {k(scope): v(scope) for k, v in items}


def _compile_target(target):
    """Return a function binding an iteration value to the names in a comprehension target."""
    if isinstance(target, ast.Name):
        name = target.id

        def bind_name(scope, value):
            scope[name] = value
        return bind_name
    if isinstance(target, (ast.Tuple, ast.List)):
        binders = [_compile_target(t) for t in target.elts]

        def bind_tuple(scope, value):
            values = list(value)
            if len(values) != len(binders):
                raise FilterExpressionError('Cannot unpack value in comprehension')
            for binder, v in zip(binders, values):
                binder(scope, v)
        return bind_tuple
    raise _unsupported(target)


def _compile_comprehension(node):
    generators = []
    for generator in node.generators:
        if generator.is_async:
            raise _unsupported(generator)
        generators.append((
            _compile_target(generator.target),
            _compile_node(generator.iter),
            [_compile_node(i) for i in generator.ifs],
        ))
    if isinstance(node, ast.DictComp):
        key = _compile_node(node.key)
        value = _compile_node(node.value)

        def element(scope):
            return key(scope), value(scope)
    else:
        element = _compile_node(node.elt)

    def iterate(scope, level):
        if level == len(generators):
            yield element(scope)
            return
        bind, iterable, conditions = generators[level]
        for item in iterable(scope):
            steps = scope[_STEPS_KEY]
            steps[0] += 1
            if steps[0] > _MAX_ITERATIONS:
                raise FilterExpressionError('Too many iterations in filter expression')
            bind(scope, item)
            if all(c(scope) for c in conditions):
                yield from iterate(scope, level + 1)

    def new_scope(scope):
        return dict(scope)

    if isinstance(node, ast.ListComp):
        return lambda scope: list(iterate(new_scope(scope), 0))
    if isinstance(node, ast.SetComp):
        return lambda scope: set(iterate(new_scope(scope), 0))
    if isinstance(node, ast.DictComp):
        return lambda scope: dict(iterate(new_scope(scope), 0))
    return lambda scope: iterate(new_scope(scope), 0)


_COMPILERS = {
    ast.Constant: _compile_constant,
    ast.Name: _compile_name,
    ast.BoolOp: _compile_bool_op,
    ast.UnaryOp: _compile_unary_op,
    ast.BinOp: _compile_bin_op,
    ast.Compare: _compile_compare,
    ast.IfExp: _compile_if_exp,
    ast.Attribute: _compile_attribute,
    ast.Subscript: _compile_subscript,
    ast.Slice: _compile_slice,
    ast.Call: _compile_call,
    ast.List: _compile_container,
    ast.Tuple: _compile_container,
    ast.Set: _compile_container,
    ast.Dict: _compile_dict,
    ast.ListComp: _compile_comprehension,
    ast.SetComp: _compile_comprehension,
    ast.DictComp: _compile_comprehension,
    ast.GeneratorExp: _compile_comprehension,
}


def compile_filter(expression):
    """
    Compile a filter expression.

    :param expression: Python expression, in which ``m`` refers to the message
    :returns: function that takes a message and returns the value of the expression
    :raises FilterExpressionError: if the expression is invalid or uses unsupported syntax
    """
    try:
        tree = ast.parse(expression.strip(), '<filter>', 'eval')
    except (SyntaxError, ValueError, RecursionError, MemoryError) as e:
        raise FilterExpressionError(f'Invalid filter expression: {e}') from None
    _check_depth(tree)
    evaluate = _compile_node(tree.body)

    def filter_message(message):
        try:
            return evaluate({'m': message, _STEPS_KEY: [0]})
        except FilterExpressionError:
            raise
        except Exception as e:
            raise FilterExpressionError(
                f'Failed to evaluate filter expression: {type(e).__name__}: {e}') from None
    return filter_message
