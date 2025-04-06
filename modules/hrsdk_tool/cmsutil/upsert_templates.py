##
## Copyright (C) 2017-2025 Hanson Robotics
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.
##

import os
import yaml
import haipy.memory_manager as mm
import logging

logger = logging.getLogger(__name__)


def upsert_prompt_templates(template_file):
    mm.init(os.environ["CLOUD_MONGO_DATABASE_URL"])
    with open(template_file) as f:
        templates = yaml.safe_load(f)
        for key, value in templates.items():
            template = mm.PromptTemplate.find_one(mm.PromptTemplate.name == key)
            template = template.run()
            if template:
                if template.template != value:
                    template.template = value
                    template.replace()
                    logger.info("Update template %s", template.name)
            else:
                template = mm.PromptTemplate(name=key, template=value).create()
                logger.info("Insert template %s", template.name)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    file = os.path.expanduser(
        "~/.cms_content/characters/sophia/processed/prompt_templates.yaml"
    )
    upsert_prompt_templates(file)
