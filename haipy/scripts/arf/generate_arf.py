#
# Copyright (C) 2017-2024 Hanson Robotics
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
import logging
import os

import coloredlogs

from haipy.arf.generators import registered_generators
from haipy.arf.sniff import sniff

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    import argparse

    import pandas as pd

    if os.isatty(2):
        formatter_str = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"
        coloredlogs.install(logging.INFO, fmt=formatter_str)

    parser = argparse.ArgumentParser()
    parser.add_argument("--file", required=True, help="the content file")
    parser.add_argument(
        "-o", default=".", dest="output_dir", help="the output directory"
    )

    args = parser.parse_args()

    def generate(name, csvfile, output_dir):
        x = sniff(csvfile)
        if "Generator" in x:
            generator_name = x["Generator"].lower()
            if generator_name in registered_generators:
                cls = registered_generators[generator_name]
                cls(name, csvfile).generate(
                    os.path.join(output_dir, "soultalk"),
                    os.path.join(output_dir, "btree"),
                )

    if args.file.endswith(".xlsx"):
        # split single spreadsheet to csv files
        sheets = pd.read_excel(args.file, sheet_name=None)
        for name, sheet in sheets.items():
            tmp_file = "/tmp/arf_%s.csv" % name
            sheet.to_csv(tmp_file, index=None)
            generate(name, tmp_file, args.output_dir)
            os.unlink(tmp_file)
    elif args.file.endswith(".csv"):
        name = os.path.splitext(os.path.basename(args.file))[0]
        generate(name, args.file, args.output_dir)
