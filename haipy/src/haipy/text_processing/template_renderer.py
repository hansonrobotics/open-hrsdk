#
# Copyright (C) 2017-2025 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
import inspect
import logging
import os
import sys
from functools import partial
from typing import Optional

from jinja2 import Environment, StrictUndefined, UndefinedError

logger = logging.getLogger(__name__)


class Renderer:
    def __init__(self, **env_args):
        strict_undefined = False
        if "strict_undefined" in env_args:
            strict_undefined = env_args.pop("strict_undefined")
        self.env = Environment(
            line_statement_prefix="%", extensions=["jinja2.ext.do"], **env_args
        )
        if strict_undefined:
            self.env.undefined = StrictUndefined

        import haipy.text_processing.template

        current_modules = list(sys.modules.keys())[:]
        logger.debug(
            "Current haipy template modules %s",
            [m for m in current_modules if "haipy" in m],
        )

        for m in current_modules:
            if m.startswith("haipy.text_processing.template.functions"):
                self._add_module(sys.modules[m], "globals")
        for m in current_modules:
            if m.startswith("haipy.text_processing.template.filters"):
                self._add_module(sys.modules[m], "filters")
        for m in current_modules:
            if m.startswith("haipy.text_processing.template.tests"):
                self._add_module(sys.modules[m], "tests")

    def _add_module(self, module, to):
        """
        Adds all of the functions of a module to a dictionary.

        parameters
        ----------
            module: module
                The module to add

            to: str
                the name of the target dict that the functions are about to
                add to
        """
        to_dict = getattr(self.env, to)
        if not to_dict:
            raise ValueError("Target dictionary is not found")
        module_sourcefile = inspect.getsourcefile(module)
        for name, func in inspect.getmembers(module, inspect.isfunction):
            # filters the functions that are directly defined in the module
            func_sourcefile = inspect.getsourcefile(func)
            if func_sourcefile == module_sourcefile:
                if name.startswith("_"):  # ignore internal functions
                    continue
                if name in to_dict:
                    logger.info(
                        'Overwrite %s "%s" from "%s"', to, name, func_sourcefile
                    )
                to_dict[name] = partial(func, renderer=self)
                logger.debug(
                    'Loaded function "%s" from "%s" to "%s"', name, func_sourcefile, to
                )

    def add_filters(self, module):
        self._add_module(module, "filters")

    def add_functions(self, module):
        self._add_module(module, "globals")

    def add_tests(self, module):
        self._add_module(module, "tests")

    def load_ext_modules(self, path, to=None):
        """Load extension module.

        load external filters: load_ext_modules(dir, 'filters')
        load external functions: load_ext_modules(dir, 'globals')
        load external tests: load_ext_modules(dir, 'tests')
        """
        if path is None:
            return
        if os.path.isdir(path):
            sys.path.insert(0, path)
            module_names = [f for f in os.listdir(path) if f.endswith(".py")]
            for py_module in module_names:
                try:
                    m = __import__(py_module[:-3])
                    if to:
                        self._add_module(m, to)
                except ImportError as ex:
                    logger.error("Import module error: %s", ex)
            logger.info("Loaded package extention")

    def render(
        self,
        template: str,
        context: dict,
        output_context: Optional[dict] = None,
        compact=True,
    ):
        try:
            t = self.env.from_string(template, globals=context)
        except TypeError as ex:
            logger.exception(ex)
            raise ex
        except Exception as ex:
            logger.exception("error loading template: %s", template)
            raise ex

        template_context = t.new_context()
        try:
            text = "".join(t.root_render_func(template_context))
        except UndefinedError as ex:
            logger.error("Rendering template %r failed. Error %s", template, ex)
            raise ex
        except Exception as ex:
            logger.error("Rendering template %r failed. Error %s", template, ex)
            raise ex

        if output_context is not None:
            for var, val in template_context.vars.items():
                if not var.startswith("_"):
                    output_context[var] = val

        if compact:
            text = " ".join(text.split())  # remove extra whitespaces
        return text


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    render = Renderer()
    context = {"name": "abc"}
    print(render.render('{%set foo="bar"%}This is {{name|upper}}', context))
    print(context)
    print(render.render('{{time(3600, locale="de_DE")}}', {}))
