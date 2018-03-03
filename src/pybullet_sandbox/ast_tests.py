import ast


class Foo(object):
    def __init__(self, bar):
        self.bar = bar


def const_node(x):
    return ast.Num(n=x)


def add_node(x, y):
    return ast.BinOp(left=x, op=ast.Add(), right=y)


def eval_node(node):
    return ast.Expression(body=node)


def string_node(str):
    return ast.Str(s=str)


def create_object_node(name, args=[], keywords=[], starargs=None, kwargs=None):
    return ast.Call(func=ast.Name(id=name, ctx=ast.Load()), args=args, keywords=keywords, starargs=starargs, kwargs=kwargs)


def eval_it(node, verbose=False):
    fixed = ast.fix_missing_locations(node)
    if verbose:
        print ast.dump(node)
        print ast.dump(fixed)
    return eval(compile(fixed, filename="<ast>", mode='eval'))


def simple_add_test():
    my_node = eval_node(add_node(const_node(40), const_node(2)))
    print eval_it(my_node)


def simple_object_test():
    my_node = eval_node(create_object_node('Foo', [string_node('muh')]))
    foo = eval_it(my_node)
    print foo.bar


simple_add_test()
simple_object_test()